/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_timerisr.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <debug.h>

#include <nuttx/arch.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"

#include "chip.h"
#include "gd32e11x_rcu.h"

/* SysTick runs from HCLK; configure it for CLK_TCK interrupts */

#define SYSTICK_RELOAD ((GD32_SYSCLK_FREQUENCY / CLK_TCK) - 1)

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

#if !defined(CONFIG_ARMV7M_SYSTICK) && !defined(CONFIG_TIMER_ARCH)
static int gd32_timerisr(int irq, void *context, void *arg)
{
  nxsched_process_timer();
  return 0;
}
#endif

void up_timer_initialize(void)
{
  uint32_t regval;

	/* Set SysTick interrupt to default priority */

  regval = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

#if defined(CONFIG_ARMV7M_SYSTICK) && defined(CONFIG_TIMER_ARCH)
	up_timer_set_lowerhalf(systick_initialize(true, GD32_SYSCLK_FREQUENCY, -1));
#else
  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(GD32_IRQ_SYSTICK, gd32_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* Enable SysTick IRQ */

  up_enable_irq(GD32_IRQ_SYSTICK);
#endif
}
