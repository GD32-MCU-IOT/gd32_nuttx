/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_start.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>

#include "arm_internal.h"
#include "itm_syslog.h"
#include "nvic.h"
#include "mpu.h"

#include "gd32h7xx_start.h"
#include "gd32h7xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* .data is positioned first in the primary RAM followed immediately by .bss.
 * The IDLE thread stack lies just after .bss and has size give by
 * CONFIG_IDLETHREAD_STACKSIZE;  The heap then begins just after the IDLE.
 * ARM EABI requires 64 bit stack alignment.
 */

#define HEAP_BASE      ((uintptr_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

const uintptr_t g_idle_topstack = HEAP_BASE;

/****************************************************************************
 * Public Function prototypes
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) noinstrument_function;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

#ifdef CONFIG_ARMV7M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)&_sbss; dest < (uint32_t *)&_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (const uint32_t *)&_eronly,
       dest = (uint32_t *)&_sdata; dest < (uint32_t *)&_edata;
      )
    {
      *dest++ = *src++;
    }

#ifdef CONFIG_ARMV7M_STACKCHECK
  arm_stack_check_init();
#endif

  /* Configure the UART so that we can get debug output as soon as possible */

  gd32_clockconfig();

  arm_fpuconfig();
  gd32_lowsetup();
  showprogress('A');

#ifdef CONFIG_ARMV7M_ITMSYSLOG
  /* Perform ARMv7-M ITM SYSLOG initialization */

  itm_syslog_initialize();
#endif

  /* Initialize onboard resources */

  gd32_boardinitialize();

#ifdef CONFIG_ARCH_DMA
  /* Attach DMA interrupts before drivers start
   * using DMA-backed peripherals.
   */

  arm_dma_initialize();
#endif

  showprogress('B');

#ifdef CONFIG_ARM_MPU
#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  gd32_userspace();
#endif

  /* Configure the MPU to permit user-space access to its FLASH and RAM (for
   * CONFIG_BUILD_PROTECTED) or to manage cache properties in external
   * memory regions.
   */

  gd32_mpu_initialize();
#endif

  showprogress('C');

  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();

  showprogress('D');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('E');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
