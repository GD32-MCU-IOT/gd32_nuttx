/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_rcu.c
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

#include "arm_internal.h"

#include "hardware/gd32e11x_rcu.h"

#include "gd32e11x_rcu.h"

void gd32_clockconfig(void)
{
  uint32_t regval;

  /* Minimal bring-up clocking: use IRC8M as SYSCLK, with AHB/APB prescalers
   * all set to DIV1.
   */

  /* Ensure IRC8M is enabled and stable. */

  modifyreg32(GD32_RCU_CTL, 0, RCU_CTL_IRC8MEN);
  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_IRC8MSTB) == 0)
    {
    }

  /* Set prescalers and switch system clock source to IRC8M. */

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_AHBPSC_MASK | RCU_CFG0_APB1PSC_MASK |
              RCU_CFG0_APB2PSC_MASK | RCU_CFG0_SCS_MASK);
  regval |= (RCU_CFG0_AHBPSC_DIV1 | RCU_CFG0_APB1PSC_DIV1 |
             RCU_CFG0_APB2PSC_DIV1 | RCU_CFG0_SCS_IRC8M);
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until IRC8M is the system clock (status bits become 0). */

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK) != 0)
    {
    }
}
