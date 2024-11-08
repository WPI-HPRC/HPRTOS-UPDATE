/****************************************************************************
 * drivers/power/pm/pm_procfs.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <sys/param.h>

#include <nuttx/nuttx.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/pm.h>

#include "pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STHDR "DOMAIN%-2d                  WAKE           SLEEP          TOTAL\n"
#define PFHDR "CALLBACKS                 IDLE           STANDBY        SLEEP\n"
#define WAHDR "DOMAIN%-2d                  STATE          COUNT          TIME\n"

#ifdef CONFIG_SYSTEM_TIME64
#  define STFMT "%-18s %8" PRIu64 "s %3" PRIu64 "%% %8" PRIu64 "s %3" \
                PRIu64 "%% %8" PRIu64 "s %3" PRIu64 "%%\n"
#  define PFFMT "%-18p %8" PRIu64 "s %3" PRIu64 "%% %8" PRIu64 "s %3" \
                PRIu64 "%% %8" PRIu64 "s %3" PRIu64 "%%\n"
#  define WAFMT "%-25s %-14s %-14" PRIu32 " %" PRIu64 "s\n"
#else
#  define STFMT "%-18s %8" PRIu32 "s %3" PRIu32 "%% %8" PRIu32 "s %3" \
                PRIu32 "%% %8" PRIu32 "s %3" PRIu32 "%%\n"
#  define PFFMT "%-18p %8" PRIu32 "s %3" PRIu32 "%% %8" PRIu32 "s %3" \
                PRIu32 "%% %8" PRIu32 "s %3" PRIu32 "%%\n"
#  define WAFMT "%-25s %-14s %-14" PRIu32 " %" PRIu32 "s\n"
#endif

/* Determines the size of an intermediate buffer that must be large enough
 * to handle the longest line generated by this logic (plus a couple of
 * bytes).
 */

#define PM_LINELEN 128

typedef ssize_t (*pm_read_t)(FAR struct file *filep,
                             FAR char *buffer, size_t buflen);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one open "file" */

struct pm_file_s
{
  struct procfs_file_s base;  /* Base open file structure */
  char line[PM_LINELEN];      /* Pre-allocated buffer for formatted lines */
  int domain;                 /* Domain index */
  pm_read_t read;             /* Read function */
};

struct pm_file_ops_s
{
  FAR const char *name;
  pm_read_t read;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     pm_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode);
static int     pm_close(FAR struct file *filep);
static ssize_t pm_read_state(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t pm_read_wakelock(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t pm_read_preparefail(FAR struct file *filep, FAR char *buffer,
                                   size_t buflen);
static ssize_t pm_read(FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
static int     pm_dup(FAR const struct file *oldp,
                      FAR struct file *newp);

static int     pm_opendir(FAR const char *relpath,
                          FAR struct fs_dirent_s **dir);
static int     pm_closedir(FAR struct fs_dirent_s *dir);
static int     pm_readdir(FAR struct fs_dirent_s *dir,
                          FAR struct dirent *entry);
static int     pm_rewinddir(FAR struct fs_dirent_s *dir);

static int     pm_stat(FAR const char *relpath, FAR struct stat *buf);

static int     pm_get_file_index(FAR const char *relpath);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations g_pm_operations =
{
  pm_open,       /* open */
  pm_close,      /* close */
  pm_read,       /* read */
  NULL,          /* write */
  NULL,          /* poll */

  pm_dup,        /* dup */

  pm_opendir,    /* opendir */
  pm_closedir,   /* closedir */
  pm_readdir,    /* readdir */
  pm_rewinddir,  /* rewinddir */

  pm_stat        /* stat */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pm_file_ops_s g_pm_files[] =
{
  {"state",        pm_read_state},
  {"wakelock",     pm_read_wakelock},
  {"preparefail",  pm_read_preparefail},
};

static FAR const char *g_pm_state[PM_COUNT] =
{
  "normal", "idle", "standby", "sleep"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_get_file_index
 ****************************************************************************/

static int pm_get_file_index(FAR const char *relpath)
{
  int i;

  for (i = 0; i < nitems(g_pm_files); i++)
    {
      if (strncmp(relpath, g_pm_files[i].name,
                  strlen(g_pm_files[i].name)) == 0)
        {
          return i;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: pm_open
 ****************************************************************************/

static int pm_open(FAR struct file *filep, FAR const char *relpath,
                   int oflags, mode_t mode)
{
  FAR struct pm_file_s *pmfile;
  int i;

  finfo("Open '%s'\n", relpath);

  /* This PROCFS file is read-only.  Any attempt to open with write access
   * is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  relpath += strlen("pm/");
  i = pm_get_file_index(relpath);
  if (i < 0)
    {
      return -ENOENT;
    }

  /* Allocate a container to hold the file attributes */

  pmfile = kmm_zalloc(sizeof(struct pm_file_s));
  if (!pmfile)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  pmfile->read = g_pm_files[i].read;
  pmfile->domain = atoi(relpath + strlen(g_pm_files[i].name));

  DEBUGASSERT(pmfile->read);
  DEBUGASSERT(pmfile->domain < CONFIG_PM_NDOMAINS);

  /* Save the attributes as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)pmfile;
  return OK;
}

/****************************************************************************
 * Name: pm_close
 ****************************************************************************/

static int pm_close(FAR struct file *filep)
{
  FAR struct pm_file_s *pmfile;

  /* Recover our private data from the struct file instance */

  pmfile = (FAR struct pm_file_s *)filep->f_priv;
  DEBUGASSERT(pmfile);

  /* Release the file attributes structure */

  kmm_free(pmfile);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: pm_read_state
 *
 * Description:
 *   The statistic values about every domain states.
 *
 ****************************************************************************/

static ssize_t pm_read_state(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct pm_domain_s *dom;
  FAR struct pm_file_s *pmfile;
  time_t sleep[PM_COUNT];
  time_t wake[PM_COUNT];
  irqstate_t flags;
  size_t totalsize = 0;
  size_t linesize;
  size_t copysize;
  off_t offset;
  time_t sum = 0;
  uint32_t state;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  pmfile = (FAR struct pm_file_s *)filep->f_priv;
  dom    = &g_pmdomains[pmfile->domain];
  DEBUGASSERT(pmfile);
  DEBUGASSERT(dom);

  /* Save the file offset and the user buffer information */

  offset = filep->f_pos;

  /* Then list the power state */

  linesize = snprintf(pmfile->line, PM_LINELEN, STHDR, pmfile->domain);
  copysize = procfs_memcpy(pmfile->line, linesize, buffer,
                           buflen, &offset);

  totalsize += copysize;

  flags = pm_domain_lock(pmfile->domain);

  for (state = 0; state < PM_COUNT; state++)
    {
      wake[state] = dom->wake[state].tv_sec;
      sleep[state] = dom->sleep[state].tv_sec;

      if (state == dom->state)
        {
          struct timespec ts;

          clock_systime_timespec(&ts);
          clock_timespec_subtract(&ts, &dom->start, &ts);
          if (dom->in_sleep)
            {
              sleep[state] += ts.tv_sec;
            }
          else
            {
              wake[state] += ts.tv_sec;
            }
        }

      sum += wake[state] + sleep[state];
    }

  pm_domain_unlock(pmfile->domain, flags);

  sum = sum ? sum : 1;

  for (state = 0; state < PM_COUNT && totalsize < buflen; state++)
    {
      time_t total;

      total = wake[state] + sleep[state];

      linesize = snprintf(pmfile->line, PM_LINELEN, STFMT,
                          g_pm_state[state],
                          wake[state],
                          100 * wake[state] / sum,
                          sleep[state],
                          100 * sleep[state] / sum,
                          total,
                          100 * total / sum);
      buffer += copysize;
      buflen -= copysize;

      copysize = procfs_memcpy(pmfile->line, linesize, buffer,
                               buflen, &offset);

      totalsize += copysize;
    }

  filep->f_pos += totalsize;
  return totalsize;
}

static ssize_t pm_read_wakelock(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct pm_domain_s *dom;
  FAR struct pm_file_s *pmfile;
  FAR dq_entry_t *entry;
  irqstate_t flags;
  size_t totalsize = 0;
  size_t linesize;
  size_t copysize;
  off_t offset;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  pmfile = (FAR struct pm_file_s *)filep->f_priv;
  dom    = &g_pmdomains[pmfile->domain];
  DEBUGASSERT(pmfile);
  DEBUGASSERT(dom);

  /* Save the file offset and the user buffer information */

  offset = filep->f_pos;

  /* Then list the power state */

  linesize = snprintf(pmfile->line, PM_LINELEN,
                      WAHDR, pmfile->domain);
  copysize = procfs_memcpy(pmfile->line, linesize, buffer,
                           buflen, &offset);

  totalsize += copysize;

  flags = pm_domain_lock(pmfile->domain);

  entry = dq_peek(&dom->wakelockall);
  for (; entry && totalsize < buflen; entry = dq_next(entry))
    {
      FAR struct pm_wakelock_s *wakelock =
          container_of(entry, struct pm_wakelock_s, fsnode);
      time_t time = wakelock->elapse.tv_sec;

      buffer += copysize;
      buflen -= copysize;

      if (wakelock->count > 0)
        {
          struct timespec ts;

          clock_systime_timespec(&ts);
          clock_timespec_subtract(&ts, &wakelock->start, &ts);

          time += ts.tv_sec;
        }

      linesize = snprintf(pmfile->line, PM_LINELEN, WAFMT,
                          wakelock->name,
                          g_pm_state[wakelock->state],
                          wakelock->count,
                          time);

      copysize = procfs_memcpy(pmfile->line, linesize, buffer,
                               buflen, &offset);

      totalsize += copysize;
    }

  pm_domain_unlock(pmfile->domain, flags);

  filep->f_pos += totalsize;
  return totalsize;
}

/****************************************************************************
 * Name: pm_read_preparefail
 *
 * Description:
 *   The statistic values about prepare callback failed.
 *
 ****************************************************************************/

static ssize_t pm_read_preparefail(FAR struct file *filep, FAR char *buffer,
                                   size_t buflen)
{
  FAR struct pm_preparefail_s *pf;
  FAR struct pm_file_s *pmfile;
  FAR struct pm_callback_s *cb;
  FAR struct pm_domain_s *dom;
  FAR dq_entry_t *entry;
  irqstate_t flags;
  size_t totalsize = 0;
  size_t linesize;
  size_t copysize;
  off_t offset;
  time_t sum = 0;
  uint32_t state;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  pmfile = (FAR struct pm_file_s *)filep->f_priv;
  dom    = &g_pmdomains[pmfile->domain];
  DEBUGASSERT(pmfile);
  DEBUGASSERT(dom);

  /* Save the file offset and the user buffer information */

  offset = filep->f_pos;

  /* Then list the power state */

  linesize = snprintf(pmfile->line, PM_LINELEN, PFHDR);
  copysize = procfs_memcpy(pmfile->line, linesize, buffer,
                           buflen, &offset);
  totalsize += copysize;

  flags = pm_domain_lock(pmfile->domain);

  for (entry = dq_peek(&dom->registry);
       entry; entry = dq_next(entry))
    {
      cb = (FAR struct pm_callback_s *)entry;
      pf = &cb->preparefail;
      for (state = 0; state < PM_COUNT; state++)
        {
          sum +=  pf->duration[state].tv_sec;
        }
    }

  sum = sum ? sum : 1;
  for (entry = dq_peek(&dom->registry);
       entry; entry = dq_next(entry))
    {
      time_t total = 0;

      cb = (FAR struct pm_callback_s *)entry;
      pf = &cb->preparefail;
      for (state = 0; state < PM_COUNT; state++)
        {
          total +=  pf->duration[state].tv_sec;
        }

      if (total == 0)
        {
          continue;
        }

      linesize = snprintf(pmfile->line, PM_LINELEN, PFFMT,
                          cb->prepare,
                          pf->duration[PM_IDLE].tv_sec,
                          100 * pf->duration[PM_IDLE].tv_sec / sum,
                          pf->duration[PM_STANDBY].tv_sec,
                          100 * pf->duration[PM_STANDBY].tv_sec / sum,
                          pf->duration[PM_SLEEP].tv_sec,
                          100 * pf->duration[PM_SLEEP].tv_sec / sum
                         );
      buffer += copysize;
      buflen -= copysize;
      copysize = procfs_memcpy(pmfile->line, linesize, buffer,
                               buflen, &offset);
      totalsize += copysize;
    }

  pm_domain_unlock(pmfile->domain, flags);
  filep->f_pos += totalsize;
  return totalsize;
}

/****************************************************************************
 * Name: pm_read
 ****************************************************************************/

static ssize_t pm_read(FAR struct file *filep, FAR char *buffer,
                       size_t buflen)
{
  FAR struct pm_file_s *pmfile;

  pmfile = (FAR struct pm_file_s *)filep->f_priv;

  return pmfile->read(filep, buffer, buflen);
}

/****************************************************************************
 * Name: pm_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int pm_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct pm_file_s *oldattr;
  FAR struct pm_file_s *newattr;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = (FAR struct pm_file_s *)oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allocate a new container to hold the task and attribute selection */

  newattr = kmm_malloc(sizeof(struct pm_file_s));
  if (!newattr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attributes from the old attributes to the new */

  memcpy(newattr, oldattr, sizeof(struct pm_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newattr;
  return OK;
}

/****************************************************************************
 * Name: pm_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int pm_opendir(FAR const char *relpath, FAR struct fs_dirent_s **dir)
{
  FAR struct procfs_dir_priv_s *level1;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath);

  /* Assume that path refers to the 1st level subdirectory.  Allocate the
   * level1 the dirent structure before checking.
   */

  level1 = kmm_zalloc(sizeof(struct procfs_dir_priv_s));
  if (level1 == NULL)
    {
      ferr("ERROR: Failed to allocate the level1 directory structure\n");
      return -ENOMEM;
    }

  /* Initialize base structure components */

  level1->level    = 1;
  level1->nentries = CONFIG_PM_NDOMAINS * nitems(g_pm_files);

  *dir = (FAR struct fs_dirent_s *)level1;
  return OK;
}

/****************************************************************************
 * Name: pm_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int pm_closedir(FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  kmm_free(dir);
  return OK;
}

/****************************************************************************
 * Name: pm_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int pm_readdir(FAR struct fs_dirent_s *dir,
                      FAR struct dirent *entry)
{
  FAR struct procfs_dir_priv_s *level1;
  int index;
  int domain;
  int fpos;

  DEBUGASSERT(dir);
  level1 = (FAR struct procfs_dir_priv_s *)dir;

  index = level1->index;
  if (index >= level1->nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT
       */

      finfo("Entry %d: End of directory\n", index);
      return -ENOENT;
    }

  domain = index / nitems(g_pm_files);
  fpos   = index % nitems(g_pm_files);

  entry->d_type = DTYPE_FILE;
  snprintf(entry->d_name, NAME_MAX + 1, "%s%d",
           g_pm_files[fpos].name, domain);

  level1->index++;
  return OK;
}

/****************************************************************************
 * Name: pm_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int pm_rewinddir(FAR struct fs_dirent_s *dir)
{
  FAR struct procfs_dir_priv_s *level1;

  DEBUGASSERT(dir);
  level1 = (FAR struct procfs_dir_priv_s *)dir;

  level1->index = 0;
  return OK;
}

/****************************************************************************
 * Name: pm_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int pm_stat(FAR const char *relpath, FAR struct stat *buf)
{
  memset(buf, 0, sizeof(struct stat));

  if (strcmp(relpath, "pm") == 0 || strcmp(relpath, "pm/") == 0)
    {
      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
    }
  else
    {
      relpath += strlen("pm/");
      if (pm_get_file_index(relpath) < 0)
        {
          return -ENOENT;
        }

      buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
    }

  return OK;
}
