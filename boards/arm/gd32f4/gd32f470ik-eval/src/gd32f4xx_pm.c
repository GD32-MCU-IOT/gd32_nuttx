/****************************************************************************
 * boards/arm/gd32f4/gd32f470ik-eval/src/gd32f4xx_pm.c
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
#include <nuttx/power/pm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "gd32f470i_eval.h"
#include "hardware/gd32f4xx_rcu.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void gd32_pm_softdelay(uint32_t delay)
{
  volatile uint32_t count;

  for (count = 0; count < delay * 10; count++)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void gd32_board_deepsleep_prepare(void)
{
  /* Let any pending console output finish before the clock is touched,
   * otherwise trailing bytes get garbled by the frequency change below.
   */

  gd32_pm_softdelay(30000);

  modifyreg32(GD32_RCU_CTL, 0, RCU_CTL_IRC16MEN);
  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_IRC16MSTB) == 0)
    {
    }

  gd32_pm_softdelay(0x50);
  modifyreg32(GD32_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
              RCU_CFG0_AHBPSC_CKSYS_DIV2);
  gd32_pm_softdelay(0x50);
  modifyreg32(GD32_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
              RCU_CFG0_AHBPSC_CKSYS_DIV4);
  gd32_pm_softdelay(0x50);
  modifyreg32(GD32_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
              RCU_CFG0_AHBPSC_CKSYS_DIV8);
  gd32_pm_softdelay(0x50);
  modifyreg32(GD32_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
              RCU_CFG0_AHBPSC_CKSYS_DIV16);
  gd32_pm_softdelay(0x50);

  modifyreg32(GD32_RCU_CFG0, RCU_CFG0_SCS_MASK, RCU_CFG0_SCS_IRC16M);
  gd32_pm_softdelay(200);
  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK) !=
         RCU_CFG0_SCSS_IRC16M)
    {
    }

  modifyreg32(GD32_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
              RCU_CFG0_AHBPSC_CKSYS_DIV1);
}

/****************************************************************************
 * Name: arm_pminitialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management
 *   subsystem.  This function must be called *very* early in the
 *   initialization sequence *before* any other device drivers are
 *   initialized (since they may attempt to register with the power
 *   management subsystem).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *    None.
 *
 ****************************************************************************/

void arm_pminitialize(void)
{
  /* Initialize the NuttX power management subsystem proper */

  pm_initialize();

#ifdef CONFIG_PM_BUTTONS
  /* Initialize the buttons to wake up the system from low power modes */

  gd32_pm_buttons();
#endif
}

#endif /* CONFIG_PM */
