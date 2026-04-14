/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_rcu.h
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

#ifndef __ARCH_ARM_SRC_GD32H7_GD32H7XX_RCU_H
#define __ARCH_ARM_SRC_GD32H7_GD32H7XX_RCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/gd32h7xx_rcu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Data
 ****************************************************************************/

#define RCU_PERI_REG_SHIFT  (6)
#define RCU_GPIOA_EN        ((GD32_RCU_AHB4EN_OFFSET << RCU_PERI_REG_SHIFT) | RCU_AHB4EN_PAEN)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the settings in board.h.
 *   This includes the initialization of clocking using the settings in
 *   board.h.
 *
 ****************************************************************************/

void gd32_clockconfig(void);

/****************************************************************************
 * Name: gd32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings
 *   in the config structure.
 *   This function is only available to support low-power modes of operation:
 *   When re-awakening from deep-sleep modes, it is necessary to re-enable/
 *   re-start the PLL
 *
 *   This function performs a subset of the operations performed by
 *   gd32_clockconfig(): It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_GD32H7_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function
 *   called gd32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void gd32_clock_enable(void);
#endif

/****************************************************************************
 * Name: gd32_board_clockconfig
 *
 * Description:
 *   Any GD32H7 board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_GD32H7_CUSTOM_CLOCKCONFIG
void gd32_board_clockconfig(void);
#endif

/****************************************************************************
 * Name: gd32_rcu_ckout0_config
 *
 * Description:
 *   Configure the CK_OUT0 clock source and divider. CK_OUT0 is connected
 *   to PA8. PA8 should be configured in alternate function mode.
 *
 ****************************************************************************/

void gd32_rcu_ckout0_config(uint32_t src, uint32_t div);

/****************************************************************************
 * Name: gd32_rcu_ckout1_config
 *
 * Description:
 *   Configure the CK_OUT1 clock source and divider. CK_OUT1 is connected
 *   to PC9. PC9 should be configured in alternate function mode.
 *
 ****************************************************************************/

void gd32_rcu_ckout1_config(uint32_t src, uint32_t div);

/****************************************************************************
 * Name: gd32_rcu_lxtal_enable
 *
 * Description:
 *   Enable the External Low Speed crystal oscillator (LXTAL).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_rcu_lxtal_enable(void);

/****************************************************************************
 * Name: gd32_rcu_irc64m_enable
 *
 * Description:
 *   Enable the Internal 64M RC oscillator (IRC64M).
 *
 ****************************************************************************/

void gd32_rcu_irc64m_enable(void);

/****************************************************************************
 * Name: gd32_rcu_irc64m_disable
 *
 * Description:
 *   Disable the Internal 64M RC oscillator (IRC64M).
 *
 ****************************************************************************/

void gd32_rcu_irc64m_disable(void);

/****************************************************************************
 * Name: gd32_rcu_lpirc4m_enable
 *
 * Description:
 *   Enable the Low Power Internal 4M RC oscillator (LPIRC4M).
 *
 ****************************************************************************/

void gd32_rcu_lpirc4m_enable(void);

/****************************************************************************
 * Name: gd32_rcu_lpirc4m_disable
 *
 * Description:
 *   Disable the Low Power Internal 4M RC oscillator (LPIRC4M).
 *
 ****************************************************************************/

void gd32_rcu_lpirc4m_disable(void);

/****************************************************************************
 * Name: gd32_rcu_periph_clock_enable
 *
 * Description:
 *   Enable the peripheral clock.
 *
 * Input Parameters:
 *   periph_clk - Peripheral clock enable identifier
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_enable(uint32_t periph_clk);

/****************************************************************************
 * Name: gd32_rcu_periph_clock_disable
 *
 * Description:
 *   Disable the peripheral clock.
 *
 * Input Parameters:
 *   periph_clk - Peripheral clock control identifier
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_disable(uint32_t periph_clk);

/****************************************************************************
 * Name: gd32_rcu_periph_reset_enable
 *
 * Description:
 *   Enable the peripheral reset.
 *
 * Input Parameters:
 *   periph - Peripheral identifier
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_rcu_periph_reset_enable(uint32_t periph);

/****************************************************************************
 * Name: gd32_rcu_periph_reset_disable
 *
 * Description:
 *   Disable the peripheral reset.
 *
 * Input Parameters:
 *   periph - Peripheral identifier
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_rcu_periph_reset_disable(uint32_t periph);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32H7_GD32H7XX_RCU_H */