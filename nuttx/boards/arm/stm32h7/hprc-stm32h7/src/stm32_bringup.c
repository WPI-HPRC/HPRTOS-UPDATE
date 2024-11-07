/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi2/src/stm32_bringup.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usbmonitor.h>

#ifdef CONFIG_STM32H7_OTGFS
#include "stm32_usbhost.h"
#endif

#ifdef CONFIG_STM32H7_FDCAN
#include "stm32_fdcan_sock.h"
#endif

#ifdef CONFIG_SYSTEMTICK_HOOK
#include <semaphore.h>
#endif

#include "hprc-stm32h7.h"

#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32_spi.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
    struct i2c_master_s *i2c;
    int ret;

    i2c = stm32_i2cbus_initialize(bus);
    if (i2c == NULL)
    {
        syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
    else
    {
        ret = i2c_register(i2c, bus);
        if (ret < 0)
        {
            syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                   bus, ret);
            stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
#ifdef CONFIG_STM32H7_I2C1
    stm32_i2c_register(1);
#endif
#ifdef CONFIG_STM32H7_I2C2
    stm32_i2c_register(2);
#endif
#ifdef CONFIG_STM32H7_I2C3
    stm32_i2c_register(3);
#endif
#ifdef CONFIG_STM32H7_I2C4
    stm32_i2c_register(4);
#endif
}
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;
#ifdef CONFIG_RAMMTD
  uint8_t *ramstart;
#endif

  UNUSED(ret);

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RAMMTD
  /* Create a RAM MTD device if configured */

  ramstart = kmm_malloc(128 * 1024);
  if (ramstart == NULL)
    {
      syslog(LOG_ERR, "ERROR: Allocation for RAM MTD failed\n");
    }
  else
    {
      /* Initialized the RAM MTD */

      struct mtd_dev_s *mtd = rammtd_initialize(ramstart, 128 * 1024);
      if (mtd == NULL)
        {
          syslog(LOG_ERR, "ERROR: rammtd_initialize failed\n");
          kmm_free(ramstart);
        }
      else
        {
          /* Erase the RAM MTD */

          ret = mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: IOCTL MTDIOC_BULKERASE failed\n");
            }

#ifdef CONFIG_FS_LITTLEFS
          /* Register the MTD driver so that it can be accessed from the
           * VFS.
           */

          ret = register_mtddriver("/dev/rammtd", mtd, 0755, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
                     ret);
            }

          /* Mount the LittleFS file system */

          ret = nx_mount("/dev/rammtd", "/mnt/lfs", "littlefs", 0,
                         "forceformat");
          if (ret < 0)
            {
              syslog(LOG_ERR,
                     "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
                     ret);
            }
#endif
        }
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize()
   * starts a thread will monitor for USB connection and
   * disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize USB host: %d\n",
             ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to start USB monitor: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  /* Register the GPIO driver */

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_NETDEV_LATEINIT

#  ifdef CONFIG_STM32H7_FDCAN1
  stm32_fdcansockinitialize(0);
#  endif

#  ifdef CONFIG_STM32H7_FDCAN2
  stm32_fdcansockinitialize(1);
#  endif

#endif

#ifdef CONFIG_SENSORS_QENCODER
#ifdef CONFIG_STM32H7_TIM1_QE
  ret = stm32_qencoder_initialize("/dev/qe0", 1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32H7_TIM3_QE
  ret = stm32_qencoder_initialize("/dev/qe2", 3);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32H7_TIM4_QE
  ret = stm32_qencoder_initialize("/dev/qe3", 4);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

  // Initialize I2C
  stm32_i2ctool();

#ifdef CONFIG_STM32_SPI1
    /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 1\n");
  spi = stm32_spibus_initialize(1);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 1\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port 1\n");

  /* Now bind the SPI interface to the M25P64/128 SPI FLASH driver */

  syslog(LOG_INFO, "Bind SPI to the SPI flash driver\n");

  mtd = m25p_initialize(spi);
  if (!mtd)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port 0 to the SPI FLASH driver\n");
      return -ENODEV;
    }

  syslog(LOG_INFO,
         "Successfully bound SPI port 0 to the SPI FLASH driver\n");
#warning "Now what are we going to do with this SPI FLASH driver?"
#endif

  return OK;
}

#ifdef CONFIG_SYSTEMTICK_HOOK

sem_t g_waitsem;

void board_timerhook(void)
{
  (void)sem_post(&g_waitsem);
}
#endif