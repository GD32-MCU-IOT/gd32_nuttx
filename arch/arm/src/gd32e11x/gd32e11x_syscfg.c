/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_syscfg.c
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

#include <assert.h>
#include <stdint.h>

#include "arm_internal.h"

#include "hardware/gd32e11x_rcu.h"

#include "gd32e11x_syscfg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void gd32_syscfg_clock_enable(void)
{
  modifyreg32(GD32_RCU_APB2EN, 0, RCU_APB2EN_AFEN);
}

void gd32_syscfg_clock_disable(void)
{
  modifyreg32(GD32_RCU_APB2EN, RCU_APB2EN_AFEN, 0);
}

void gd32_syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin)
{
  uintptr_t extiss;
  uint32_t shift;
  uint32_t mask;

  DEBUGASSERT(exti_pin < 16);

  if (exti_pin < 4)
    {
      extiss = GD32_AFIO_EXTISS0;
    }
  else if (exti_pin < 8)
    {
      extiss = GD32_AFIO_EXTISS1;
    }
  else if (exti_pin < 12)
    {
      extiss = GD32_AFIO_EXTISS2;
    }
  else
    {
      extiss = GD32_AFIO_EXTISS3;
    }

  shift = AFIO_EXTISS_SHIFT(exti_pin);
  mask  = AFIO_EXTISS_MASK(exti_pin);

  modifyreg32(extiss, mask, ((uint32_t)exti_port << shift));
}
