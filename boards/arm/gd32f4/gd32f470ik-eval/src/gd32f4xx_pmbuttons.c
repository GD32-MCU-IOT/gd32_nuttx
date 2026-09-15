/****************************************************************************
 * boards/arm/gd32f4/gd32f470ik-eval/src/gd32f4xx_pmbuttons.c
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/power/pm.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "gd32f470i_eval.h"

#if defined(CONFIG_PM) && defined(CONFIG_PM_BUTTONS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_BUTTONS
#  error "CONFIG_ARCH_BUTTONS is not defined in the configuration"
#endif

#ifndef CONFIG_ARCH_IRQBUTTONS
#  warning "CONFIG_ARCH_IRQBUTTONS is not defined in the configuration"
#endif

#ifndef CONFIG_PM_BUTTON_ACTIVITY
#  define CONFIG_PM_BUTTON_ACTIVITY 10
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
static int button_handler(int irq, void *context, void *arg);
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_handler
 *
 * Description:
 *   Handle a button wake-up interrupt
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
static int button_handler(int irq, void *context, void *arg)
{
  /* At this point the MCU should have already awakened.  The state
   * change will be handled in the IDLE loop when the system is
   * re-awakened.  The button interrupt handler should be totally
   * ignorant of the PM activities and should report button activity
   * as if nothing special happened.
   */

  _info("TAMPER(PC13) interrupt triggered\n");
  pm_activity(PM_IDLE_DOMAIN, CONFIG_PM_BUTTON_ACTIVITY);
  return OK;
}
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pm_buttons
 *
 * Description:
 *   Configure the TAMPER user button of the GD32F470IK-EVAL board as an
 *   EXTI wakeup source, so it is able to wake up the MCU from the low
 *   power modes (Sleep and Deep-sleep) and to report user activity to
 *   the PM subsystem.
 *
 ****************************************************************************/

void gd32_pm_buttons(void)
{
  /* Initialize the button GPIOs */

  board_button_initialize();

#ifdef CONFIG_ARCH_IRQBUTTONS
  /* Attach the TAMPER button interrupt as the PM wakeup source. */

  board_button_irq(BUTTON_TAMPER, button_handler, NULL);
#endif
}

#endif /* CONFIG_PM && CONFIG_PM_BUTTONS */
