/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_rcu.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_RCU_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_RCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/gd32e11x_rcu.h"

/* Default clock assumptions for minimal bring-up.
 * Boards can override these by defining the macros before including this
 * header.
 */

#ifndef GD32_IRC8M_FREQUENCY
#  define GD32_IRC8M_FREQUENCY   8000000u
#endif

#ifndef GD32_SYSCLK_FREQUENCY
#  define GD32_SYSCLK_FREQUENCY  GD32_IRC8M_FREQUENCY
#endif

#ifndef GD32_HCLK_FREQUENCY
#  define GD32_HCLK_FREQUENCY    GD32_SYSCLK_FREQUENCY
#endif

#ifndef GD32_PCLK1_FREQUENCY
#  define GD32_PCLK1_FREQUENCY   (GD32_HCLK_FREQUENCY/2)
#endif

#ifndef GD32_PCLK2_FREQUENCY
#  define GD32_PCLK2_FREQUENCY   GD32_HCLK_FREQUENCY
#endif

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_clockconfig
 *
 * Description:
 *   Called to initialize the GD32E11X.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void gd32_clockconfig(void);

/****************************************************************************
 * Name: gd32_rcu_periph_clock_enable
 *
 * Description:
 *   Enable the peripherals clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_enable(uint32_t periph);

/****************************************************************************
 * Name: gd32_rcu_periph_clock_disable
 *
 * Description:
 *   Disable the peripherals clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_disable(uint32_t periph);

/****************************************************************************
 * Name: gd32_clock_enable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings
 *   in board.h. This function is only available to support low-power
 *   modes of operation:  When re-awakening from deep-sleep modes, it is
 *   necessary to re-enable/re-start the PLL
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void gd32_clock_enable(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_RCU_H */
