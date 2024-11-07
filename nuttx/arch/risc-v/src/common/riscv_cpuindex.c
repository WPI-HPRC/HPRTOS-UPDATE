/****************************************************************************
 * arch/risc-v/src/common/riscv_cpuindex.c
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

int up_cpu_index(void)
{
  return (int)riscv_mhartid();
}

/****************************************************************************
 * Name: up_this_cpu
 *
 * Description:
 *   Return the logical core number. Default implementation is 1:1 mapping,
 *   i.e. physical=logical.
 *
 ****************************************************************************/

int up_this_cpu(void)
{
  return riscv_hartid_to_cpuid((int)riscv_mhartid());
}

/****************************************************************************
 * Name: riscv_hartid_to_cpuid
 *
 * Description:
 *   Convert physical core number to logical core number. Default
 *   implementation is 1:1 mapping, i.e. physical=logical.
 *
 ****************************************************************************/

int weak_function riscv_hartid_to_cpuid(int hart)
{
#ifdef CONFIG_SMP
  return hart - CONFIG_ARCH_RV_HARTID_BASE;
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: riscv_cpuid_to_hartid
 *
 * Description:
 *   Convert logical core number to physical core number. Default
 *   implementation is 1:1 mapping, i.e. physical=logical.
 *
 ****************************************************************************/

int weak_function riscv_cpuid_to_hartid(int cpu)
{
#ifdef CONFIG_SMP
  return cpu + CONFIG_ARCH_RV_HARTID_BASE;
#else
  return (int)riscv_mhartid();
#endif
}