/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_start.c
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

#include <nuttx/init.h>

#include "arm_internal.h"
#include "itm_syslog.h"
#include "nvic.h"

#include "gd32e11x.h"
#include "gd32e11x_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HEAP_BASE ((uintptr_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_idle_topstack = HEAP_BASE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_STACKCHECK
void __start(void) noinstrument_function;
#endif

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

#ifdef CONFIG_ARMV7M_STACKCHECK
  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata; )
    {
      *dest++ = *src++;
    }

#ifdef CONFIG_ARMV7M_STACKCHECK
  arm_stack_check_init();
#endif

  /* Clock + low-level UART for early logging */

  gd32_clockconfig();
  gd32_lowsetup();
  showprogress('A');

#ifdef CONFIG_ARMV7M_ITMSYSLOG
  itm_syslog_initialize();
#endif

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('B');

#ifdef CONFIG_BUILD_PROTECTED
  gd32_userspace();
  showprogress('C');
#endif

  gd32_boardinitialize();
  showprogress('D');

  showprogress('\r');
  showprogress('\n');

  nx_start();

  for (; ; );
}
