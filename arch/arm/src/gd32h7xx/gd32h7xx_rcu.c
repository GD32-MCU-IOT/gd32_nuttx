/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_rcu.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "chip.h"
#include "gd32h7xx_rcu.h"
#include "gd32h7xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow 2 milliseconds for the IRC64M to become ready. */

#define IRC64M_STARTUP_TIMEOUT   (2 * CONFIG_BOARD_LOOPSPERMSEC)

/* Allow 10 milliseconds for the HXTAL to become ready. */

#define HXTAL_STARTUP_TIMEOUT    (10 * CONFIG_BOARD_LOOPSPERMSEC)

/* Allow 20 milliseconds for the PLL to become ready. */

#define PLL_STARTUP_TIMEOUT      (20 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcu_reset
 *
 * Description:
 *   Reset the RCU clock configuration to the default reset state
 *
 ****************************************************************************/

static inline void rcu_reset(void)
{
  uint32_t regval;

  /* Reset CTL register to default state (except IRC64M which
   * should stay enabled)
   */

  regval  = getreg32(GD32_RCU_CTL);
  regval &= ~(RCU_CTL_HXTALEN | RCU_CTL_CKMEN | RCU_CTL_PLL0EN |
              RCU_CTL_PLL1EN | RCU_CTL_PLL2EN | RCU_CTL_HXTALBPS);
  putreg32(regval, GD32_RCU_CTL);

  /* Reset ADDCTL1 register */

  regval  = getreg32(GD32_RCU_ADDCTL1);
  regval &= ~(RCU_ADDCTL1_PLLUSBHS0EN | RCU_ADDCTL1_PLLUSBHS1EN |
              RCU_ADDCTL1_LPIRC4MEN);
  putreg32(regval, GD32_RCU_ADDCTL1);

  /* Reset CFG0 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_APB1PSC_MASK | RCU_CFG0_APB2PSC_MASK |
              RCU_CFG0_APB3PSC_MASK | RCU_CFG0_APB4PSC_MASK |
              RCU_CFG0_AHBPSC_MASK | RCU_CFG0_I2C0SEL_MASK |
              RCU_CFG0_SCS_MASK | RCU_CFG0_RTCDIV_MASK);
  putreg32(regval, GD32_RCU_CFG0);

  /* Reset CFG1 */

  regval  = getreg32(GD32_RCU_CFG1);
  regval &= ~(RCU_CFG1_HPDFSEL | RCU_CFG1_TIMERSEL | RCU_CFG1_PERSEL_MASK |
              RCU_CFG1_RSPDIFSEL_MASK | RCU_CFG1_USART0SEL_MASK |
              RCU_CFG1_USART1SEL_MASK | RCU_CFG1_USART2SEL_MASK |
              RCU_CFG1_USART5SEL_MASK | RCU_CFG1_PLL2RDIV_MASK);
  putreg32(regval, GD32_RCU_CFG1);

  /* Reset CFG2 */

  regval  = getreg32(GD32_RCU_CFG2);
  regval &= ~(RCU_CFG2_SAI2B1SEL_MASK | RCU_CFG2_SAI2B0SEL_MASK |
              RCU_CFG2_SAI1SEL_MASK | RCU_CFG2_SAI0SEL_MASK |
              RCU_CFG2_CKOUT0SEL_MASK | RCU_CFG2_CKOUT1SEL_MASK |
              RCU_CFG2_CKOUT0DIV_MASK | RCU_CFG2_CKOUT1DIV_MASK);
  putreg32(regval, GD32_RCU_CFG2);

  /* Reset CFG3 */

  regval  = getreg32(GD32_RCU_CFG3);
  regval &= ~(RCU_CFG3_ADC01SEL_MASK | RCU_CFG3_ADC2SEL_MASK |
              RCU_CFG3_SDIO1SEL | RCU_CFG3_I2C3SEL_MASK |
              RCU_CFG3_I2C2SEL_MASK | RCU_CFG3_I2C1SEL_MASK);
  putreg32(regval, GD32_RCU_CFG3);

  /* Reset CFG4 */

  regval  = getreg32(GD32_RCU_CFG4);
  regval &= ~(RCU_CFG4_EXMCSEL_MASK | RCU_CFG4_SDIO0SEL);
  putreg32(regval, GD32_RCU_CFG4);

  /* Reset CFG5 */

  regval  = getreg32(GD32_RCU_CFG5);
  regval &= ~(RCU_CFG5_SPI0SEL_MASK | RCU_CFG5_SPI1SEL_MASK |
              RCU_CFG5_SPI2SEL_MASK | RCU_CFG5_SPI3SEL_MASK |
              RCU_CFG5_SPI4SEL_MASK | RCU_CFG5_SPI5SEL_MASK);

  /* Disable all interrupts */

  putreg32(0x14ff0000, GD32_RCU_INT);
  putreg32(0x00700000, GD32_RCU_ADDINT);

  /* Reset PLL0 register */

  putreg32(0x01002020, GD32_RCU_PLL0);

  /* Reset PLL1 register */

  putreg32(0x01012020, GD32_RCU_PLL1);

  /* Reset PLL2 register */

  putreg32(0x01012020, GD32_RCU_PLL2);

  putreg32(0x00000000, GD32_RCU_PLLALL);
  putreg32(0x00010101, GD32_RCU_PLLADDCTL);
  putreg32(0x00000000, GD32_RCU_PLLUSBCFG);
  putreg32(0x00000000, GD32_RCU_PLL0FRA);
  putreg32(0x00000000, GD32_RCU_PLL1FRA);
  putreg32(0x00000000, GD32_RCU_PLL2FRA);
}

/****************************************************************************
 * Name: rcu_enable_hxtal
 *
 * Description:
 *   Enable the External High-speed Crystal Oscillator (HXTAL).
 *
 ****************************************************************************/

static inline bool rcu_enable_hxtal(void)
{
  volatile int32_t timeout;
  uint32_t regval;

  /* Enable HXTAL */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until the HXTAL is ready or until a timeout occurs. */

  for (timeout = HXTAL_STARTUP_TIMEOUT; timeout > 0; timeout--)
    {
      regval = getreg32(GD32_RCU_CTL);
      if ((regval & RCU_CTL_HXTALSTB) != 0)
        {
          return true; /* HXTAL is ready */
        }
    }

  return false; /* Timeout occurred */
}

/****************************************************************************
 * Name: rcu_enable_pll0
 *
 * Description:
 *   Enable PLL0
 *
 ****************************************************************************/

static inline bool rcu_enable_pll0(uint32_t pllcfg)
{
  volatile int32_t timeout;
  uint32_t regval;

  /* Set the PLL0 configuration */

  putreg32(pllcfg, GD32_RCU_PLL0);

  /* Enable PLL0 */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLL0EN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until the PLL0 is ready or until a timeout occurs. */

  for (timeout = PLL_STARTUP_TIMEOUT; timeout > 0; timeout--)
    {
      regval = getreg32(GD32_RCU_CTL);
      if ((regval & RCU_CTL_PLL0STB) != 0)
        {
          return true; /* PLL0 is ready */
        }
    }

  return false; /* Timeout occurred */
}

/****************************************************************************
 * Name: rcu_enable_pll1
 *
 * Description:
 *   Enable PLL1
 *
 ****************************************************************************/

static inline bool rcu_enable_pll1(uint32_t pllcfg)
{
  volatile int32_t timeout;
  uint32_t regval;

  /* Set the PLL1 configuration */

  putreg32(pllcfg, GD32_RCU_PLL1);

  /* Enable PLL1 */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLL1EN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until the PLL1 is ready or until a timeout occurs. */

  for (timeout = PLL_STARTUP_TIMEOUT; timeout > 0; timeout--)
    {
      regval = getreg32(GD32_RCU_CTL);
      if ((regval & RCU_CTL_PLL1STB) != 0)
        {
          return true; /* PLL1 is ready */
        }
    }

  return false; /* Timeout occurred */
}

/****************************************************************************
 * Name: rcu_enable_pll2
 *
 * Description:
 *   Enable PLL2
 *
 ****************************************************************************/

static inline bool rcu_enable_pll2(uint32_t pllcfg)
{
  volatile int32_t timeout;
  uint32_t regval;

  /* Set the PLL2 configuration */

  putreg32(pllcfg, GD32_RCU_PLL2);

  /* Enable PLL2 */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLL2EN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until the PLL2 is ready or until a timeout occurs. */

  for (timeout = PLL_STARTUP_TIMEOUT; timeout > 0; timeout--)
    {
      regval = getreg32(GD32_RCU_CTL);
      if ((regval & RCU_CTL_PLL2STB) != 0)
        {
          return true; /* PLL2 is ready */
        }
    }

  return false; /* Timeout occurred */
}

#ifdef GD32_BOARD_SYSCLK_IRC64MEN
/****************************************************************************
 * Name: gd32_system_clock_irc64m
 *
 * Description:
 *   Select the IRC64M as system clock.
 *
 ****************************************************************************/

static void gd32_system_clock_irc64m(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Wait until IRC64M is stable or the startup time is longer than
   * IRC64M_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC64MSTB);
    }
  while ((0 == stab_flag) && (IRC64M_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC64MSTB))
    {
      while (1)
        {
        }
    }

  /* Set the AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= RCU_CFG0_AHBPSC_CKSYS_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB4 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB4PSC_MASK;
  regval |= RCU_CFG0_APB4PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB3 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB3PSC_MASK;
  regval |= RCU_CFG0_APB3PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB2 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= RCU_CFG0_APB2PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB1 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= RCU_CFG0_APB1PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Select IRC64M as system clock */

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_IRC64M;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until IRC64M is selected as system clock */

  while (0 != (getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_IRC64M))
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_HXTAL)
/****************************************************************************
 * Name: gd32_system_clock_hxtal
 *
 * Description:
 *   Select the HXTAL as system clock.
 *
 ****************************************************************************/

static void gd32_system_clock_hxtal(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Enable HXTAL */

#ifdef GD32_RCU_CTL_HXTALBPSEN

  /* Bypass HXTAL oscillator when using the external clock which drives the
   * OSCIN pin.
   * If use a crystal with HXTAL, do not define GD32_RCU_CTL_HXTALBPSEN.
   */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALBPS;
  putreg32(regval, GD32_RCU_CTL);

#endif

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until HXTAL is stable or the startup time is longer than
   * HXTAL_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB);
    }
  while ((0 == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB))
    {
      while (1)
        {
        }
    }

  /* Configure AHB / APBx prescalers: all at DIV1 for direct HXTAL use */

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_AHBPSC_MASK  | RCU_CFG0_APB4PSC_MASK |
              RCU_CFG0_APB3PSC_MASK | RCU_CFG0_APB2PSC_MASK |
              RCU_CFG0_APB1PSC_MASK);
  regval |= (RCU_CFG0_AHBPSC_CKSYS_DIV1    |
             RCU_CFG0_APB4PSC_CKAHB_DIV1   |
             RCU_CFG0_APB3PSC_CKAHB_DIV1   |
             RCU_CFG0_APB2PSC_CKAHB_DIV1   |
             RCU_CFG0_APB1PSC_CKAHB_DIV1);
  putreg32(regval, GD32_RCU_CFG0);

  /* Select HXTAL as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_HXTAL;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until HXTAL is selected */

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK) !=
         RCU_CFG0_SCSS_HXTAL)
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_PLL_IRC64M)
/****************************************************************************
 * Name: gd32_system_clock_pll_irc64m
 *
 * Description:
 *   Configure the system clock by PLL which selects IRC64M as its clock
 *   source.
 *
 ****************************************************************************/

static void gd32_system_clock_pll_irc64m(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Wait until IRC64M is stable or the startup time expires */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC64MSTB);
    }
  while ((0 == stab_flag) && (IRC64M_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC64MSTB))
    {
      while (1)
        {
        }
    }

  /* Insert TCM wait state for 600MHz operation */

  regval = getreg32(GD32_RCU_APB4EN);
  regval |= RCU_APB4EN_SYSCFGEN;
  putreg32(regval, GD32_RCU_APB4EN);

  regval = getreg32(GD32_SYSCFG_SRAMCFG1);
  regval |= SYSCFG_SRAMCFG1_TCM_WAITSTATE;
  putreg32(regval, GD32_SYSCFG_SRAMCFG1);

  /* Set the AHB = SYSCLK/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= GD32_RCU_CFG0_AHB_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB4 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB4PSC_MASK;
  regval |= GD32_RCU_CFG0_APB4_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB3 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB3PSC_MASK;
  regval |= GD32_RCU_CFG0_APB3_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB2 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= GD32_RCU_CFG0_APB2_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB1 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= GD32_RCU_CFG0_APB1_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* PLL0 select IRC64MDIV, config IRC64MDIV
   * as IRC64M, PLL0 input and output range
   */

  regval = getreg32(GD32_RCU_ADDCTL1);
  regval &= ~(RCU_ADDCTL1_IRC64MDIV_MASK);
  regval |= RCU_ADDCTL1_IRC64M_DIV1;
  putreg32(regval, GD32_RCU_ADDCTL1);

  regval = getreg32(GD32_RCU_PLLALL);
  regval &= ~(RCU_PLLALL_PLLSEL_MASK | RCU_PLLALL_PLL0VCOSEL |
              RCU_PLLALL_PLL0RNG_MASK);
  regval |= (RCU_PLLALL_PLLSEL_IRC64MDIV | RCU_PLLALL_PLL0RNG_4M_8M);
  putreg32(regval, GD32_RCU_PLLALL);

  /* Configure PLL0: PLL0P = IRC64MDIV/16 * PLL0N / PLL0P_DIV = 600MHz
   * (board-provided values)
   */

  regval = getreg32(GD32_RCU_PLL0);
  regval &= (uint32_t)~(RCU_PLL0_PLL0N_MASK | RCU_PLL0_PLL0PSC_MASK |
                        RCU_PLL0_PLL0P_MASK | RCU_PLL0_PLL0R_MASK |
                        RCU_PLL0_PLLSTBSRC);
  regval |= (GD32_PLL_PLL0N | GD32_PLL_PLL0PSC |
             GD32_PLL_PLL0P | GD32_PLL_PLL0R);
  putreg32(regval, GD32_RCU_PLL0);

  /* Configure PLL0Q */

  regval = getreg32(GD32_RCU_PLLADDCTL);
  regval &= ~RCU_PLLADDCTL_PLL0Q_MASK;
  regval |= (GD32_PLL_PLL0Q);
  putreg32(regval, GD32_RCU_PLLADDCTL);

  /* Enable PLL0 P/Q/R outputs */

  regval = getreg32(GD32_RCU_PLLADDCTL);
  regval |= (RCU_PLLADDCTL_PLL0PEN | RCU_PLLADDCTL_PLL0QEN |
             RCU_PLLADDCTL_PLL0REN);
  putreg32(regval, GD32_RCU_PLLADDCTL);

  /* Enable PLL0 */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLL0EN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until PLL0 is stable */

  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_PLL0STB) == 0)
    {
    }

  /* Select PLL0P as system clock */

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_PLL0P;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until PLL0P is selected */

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK) !=
         RCU_CFG0_SCSS_PLL0P)
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_PLL_HXTAL)
/****************************************************************************
 * Name: gd32_system_clock_pll_hxtal
 *
 * Description:
 *   Configure the system clock by PLL which selects HXTAL as its clock
 *   source.
 *
 ****************************************************************************/

static void gd32_system_clock_pll_hxtal(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* enable HXTAL */

#ifdef GD32_RCU_CTL_HXTALBPSEN

  /* Bypass HXTAL oscillator when using the external clock which drives the
   * OSCIN pin.
   * If use a crystal with HXTAL, do not define GD32_RCU_CTL_HXTALBPSEN.
   */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALBPS;
  putreg32(regval, GD32_RCU_CTL);

#endif

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait for HXTAL ready with timeout */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB);
    }
  while ((0 == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB))
    {
      while (1)
        {
        }
    }

  /* Insert TCM wait state for 600MHz */

  regval = getreg32(GD32_RCU_APB4EN);
  regval |= RCU_APB4EN_SYSCFGEN;
  putreg32(regval, GD32_RCU_APB4EN);

  regval = getreg32(GD32_SYSCFG_SRAMCFG1);
  regval |= SYSCFG_SRAMCFG1_TCM_WAITSTATE;
  putreg32(regval, GD32_SYSCFG_SRAMCFG1);

  /* Set the AHB = SYSCLK/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= GD32_RCU_CFG0_AHB_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB4 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB4PSC_MASK;
  regval |= GD32_RCU_CFG0_APB4_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB3 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB3PSC_MASK;
  regval |= GD32_RCU_CFG0_APB3_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB2 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= GD32_RCU_CFG0_APB2_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB1 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= GD32_RCU_CFG0_APB1_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Select HXTAL as PLL source, configure VCO range */

  regval = getreg32(GD32_RCU_PLLALL);
  regval &= ~(RCU_PLLALL_PLLSEL_MASK | RCU_PLLALL_PLL0VCOSEL |
              RCU_PLLALL_PLL0RNG_MASK);
  regval |= (RCU_PLLALL_PLLSEL_HXTAL | RCU_PLLALL_PLL0RNG_4M_8M);
  putreg32(regval, GD32_RCU_PLLALL);

  /* Configure PLL0: PLL0P = HXTAL/5 * PLL0N / PLL0P_DIV = 600MHz
   * (board-provided values)
   */

  regval = getreg32(GD32_RCU_PLL0);
  regval &= (uint32_t)~(RCU_PLL0_PLL0N_MASK | RCU_PLL0_PLL0PSC_MASK |
                        RCU_PLL0_PLL0P_MASK | RCU_PLL0_PLL0R_MASK |
                        RCU_PLL0_PLLSTBSRC);
  regval |= (GD32_PLL_PLL0N | GD32_PLL_PLL0PSC |
             GD32_PLL_PLL0P | GD32_PLL_PLL0R);
  putreg32(regval, GD32_RCU_PLL0);

  /* Configure PLL0Q */

  regval = getreg32(GD32_RCU_PLLADDCTL);
  regval &= ~RCU_PLLADDCTL_PLL0Q_MASK;
  regval |= (GD32_PLL_PLL0Q);
  putreg32(regval, GD32_RCU_PLLADDCTL);

  /* Enable PLL0 P/Q/R outputs */

  regval = getreg32(GD32_RCU_PLLADDCTL);
  regval |= (RCU_PLLADDCTL_PLL0PEN | RCU_PLLADDCTL_PLL0QEN |
             RCU_PLLADDCTL_PLL0REN);
  putreg32(regval, GD32_RCU_PLLADDCTL);

  /* Enable PLL0 */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLL0EN;
  putreg32(regval, GD32_RCU_CTL);

  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_PLL0STB) == 0)
    {
    }

  /* Switch SYSCLK to PLL0P */

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_PLL0P;
  putreg32(regval, GD32_RCU_CFG0);

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK) !=
         RCU_CFG0_SCSS_PLL0P)
    {
    }
}
#endif

/****************************************************************************
 * Name: gd32_system_clock_config
 *
 * Description:
 *   Configure the system clock using the settings in board.h.
 *
 ****************************************************************************/

static void gd32_system_clock_config(void)
{
#ifdef GD32_BOARD_SYSCLK_IRC64MEN

  /* Select IRC64M as SYSCLK based on board.h setting. */

  gd32_system_clock_irc64m();
#elif defined (GD32_BOARD_SYSCLK_HXTAL)

  /* Select HXTAL as SYSCLK based on board.h setting. */

  gd32_system_clock_hxtal();

#elif defined (GD32_BOARD_SYSCLK_PLL_IRC64M)

  /* Select PLL which source is IRC64M as SYSCLK based on board.h setting. */

  gd32_system_clock_pll_irc64m();
#elif defined (GD32_BOARD_SYSCLK_PLL_HXTAL)

  /* Select  PLL which source is HXTAL as SYSCLK based on board.h setting. */

  gd32_system_clock_pll_hxtal();

#else
  #error "Invalid system clock configuration."
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings.
 *
 ****************************************************************************/

void gd32_clockconfig(void)
{
  uint32_t regval;

  /* Enable IRC64M */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_IRC64MEN;
  putreg32(regval, GD32_RCU_CTL);

  regval = getreg32(GD32_SYSCFG_SRAMCFG1);
  regval &= ~SYSCFG_SRAMCFG1_TCM_WAITSTATE;
  putreg32(regval, GD32_SYSCFG_SRAMCFG1);

  /* Reset the RCU clock configuration to the default reset state */

  rcu_reset();

#if defined (SEL_PMU_SMPS_MODE)
  /* Power supply config */

  pmu_smps_ldo_supply_config(SEL_PMU_SMPS_MODE);
#endif

#if defined(CONFIG_ARCH_BOARD_GD32F4_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  gd32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
   * AHB/APBx prescalers and Flash settings
   */

  gd32_system_clock_config();
#endif
}

/****************************************************************************
 * Name: gd32_clock_enable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings
 *   in board.h. This function is only available to support low-power
 *   modes of operation:  When re-awakening from deep-sleep modes, it is
 *   necessary to re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   gd32_clockconfig():  It does not reset any devices, and it does not
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

void gd32_clock_enable(void)
{
#ifdef CONFIG_ARCH_BOARD_GD32H7_CUSTOM_CLOCKCONFIG
  /* Use board-specific clock configuration */

  gd32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
   * AHB/APBx prescalers and Flash settings
   */

  gd32_system_clock_config();
#endif
}

/****************************************************************************
 * Name: gd32_rcu_ckout0_config
 *
 * Description:
 *   Configure the CK_OUT0 clock source and divider. CK_OUT0 is connected
 *   to PA8. PA8 should be configured in alternate function mode.
 *
 ****************************************************************************/

void gd32_rcu_ckout0_config(uint32_t src, uint32_t div)
{
  uint32_t regval;

  regval  = getreg32(GD32_RCU_CFG2);
  regval &= ~(RCU_CFG2_CKOUT0SEL_MASK | RCU_CFG2_CKOUT0DIV_MASK);
  regval |= (src | div);
  putreg32(regval, GD32_RCU_CFG2);
}

/****************************************************************************
 * Name: gd32_rcu_ckout1_config
 *
 * Description:
 *   Configure the CK_OUT1 clock source and divider. CK_OUT1 is connected
 *   to PC9. PC9 should be configured in alternate function mode.
 *
 ****************************************************************************/

void gd32_rcu_ckout1_config(uint32_t src, uint32_t div)
{
  uint32_t regval;

  regval  = getreg32(GD32_RCU_CFG2);
  regval &= ~(RCU_CFG2_CKOUT1SEL_MASK | RCU_CFG2_CKOUT1DIV_MASK);
  regval |= (src | div);
  putreg32(regval, GD32_RCU_CFG2);
}

/****************************************************************************
 * Name: gd32_rcu_periph_clock_enable
 *
 * Description:
 *   Enable the peripheral clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_enable(uint32_t periph_clk)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph_clk & 0x1f;
  reg_off = (periph_clk >> RCU_PERI_REG_SHIFT);

  switch (reg_off)
    {
      case GD32_RCU_AHB1EN_OFFSET:
        regaddr = GD32_RCU_AHB1EN;
        break;

      case GD32_RCU_AHB2EN_OFFSET:
        regaddr = GD32_RCU_AHB2EN;
        break;

      case GD32_RCU_AHB3EN_OFFSET:
        regaddr = GD32_RCU_AHB3EN;
        break;

      case GD32_RCU_AHB4EN_OFFSET:
        regaddr = GD32_RCU_AHB4EN;
        break;

      case GD32_RCU_APB1EN_OFFSET:
        regaddr = GD32_RCU_APB1EN;
        break;

      case GD32_RCU_APB2EN_OFFSET:
        regaddr = GD32_RCU_APB2EN;
        break;

      case GD32_RCU_APB3EN_OFFSET:
        regaddr = GD32_RCU_APB3EN;
        break;

      case GD32_RCU_APB4EN_OFFSET:
        regaddr = GD32_RCU_APB4EN;
        break;

      default:
        break;
    }

  regval = getreg32(regaddr);
  regval |= (1 << bit_pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_rcu_periph_clock_disable
 *
 * Description:
 *   Disable the peripheral clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_disable(uint32_t periph_clk)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph_clk & 0x1f;
  reg_off = (periph_clk >> RCU_PERI_REG_SHIFT);

  switch (reg_off)
    {
      case GD32_RCU_AHB1EN_OFFSET:
        regaddr = GD32_RCU_AHB1EN;
        break;

      case GD32_RCU_AHB2EN_OFFSET:
        regaddr = GD32_RCU_AHB2EN;
        break;

      case GD32_RCU_AHB3EN_OFFSET:
        regaddr = GD32_RCU_AHB3EN;
        break;

      case GD32_RCU_AHB4EN_OFFSET:
        regaddr = GD32_RCU_AHB4EN;
        break;

      case GD32_RCU_APB1EN_OFFSET:
        regaddr = GD32_RCU_APB1EN;
        break;

      case GD32_RCU_APB2EN_OFFSET:
        regaddr = GD32_RCU_APB2EN;
        break;

      case GD32_RCU_APB3EN_OFFSET:
        regaddr = GD32_RCU_APB3EN;
        break;

      case GD32_RCU_APB4EN_OFFSET:
        regaddr = GD32_RCU_APB4EN;
        break;

      default:
        break;
    }

  regval = getreg32(regaddr);
  regval &= ~(1 << bit_pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_rcu_periph_reset_enable
 *
 * Description:
 *   Enable the peripheral reset.
 *
 ****************************************************************************/

void gd32_rcu_periph_reset_enable(uint32_t periph_rst)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph_rst & 0x1f;
  reg_off = (periph_rst >> RCU_PERI_REG_SHIFT);

  switch (reg_off)
    {
      case GD32_RCU_AHB1RST_OFFSET:
        regaddr = GD32_RCU_AHB1RST;
        break;

      case GD32_RCU_AHB2RST_OFFSET:
        regaddr = GD32_RCU_AHB2RST;
        break;

      case GD32_RCU_AHB3RST_OFFSET:
        regaddr = GD32_RCU_AHB3RST;
        break;

      case GD32_RCU_AHB4RST_OFFSET:
        regaddr = GD32_RCU_AHB4RST;
        break;

      case GD32_RCU_APB1RST_OFFSET:
        regaddr = GD32_RCU_APB1RST;
        break;

      case GD32_RCU_APB2RST_OFFSET:
        regaddr = GD32_RCU_APB2RST;
        break;

      case GD32_RCU_APB3RST_OFFSET:
        regaddr = GD32_RCU_APB3RST;
        break;

      case GD32_RCU_APB4RST_OFFSET:
        regaddr = GD32_RCU_APB4RST;
        break;

      default:
        break;
    }

  regval = getreg32(regaddr);
  regval |= (1 << bit_pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_rcu_periph_reset_disable
 *
 * Description:
 *   Disable the peripheral reset.
 *
 ****************************************************************************/

void gd32_rcu_periph_reset_disable(uint32_t periph_rst)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph_rst & 0x1f;
  reg_off = (periph_rst >> RCU_PERI_REG_SHIFT);

  switch (reg_off)
    {
      case GD32_RCU_AHB1RST_OFFSET:
        regaddr = GD32_RCU_AHB1RST;
        break;

      case GD32_RCU_AHB2RST_OFFSET:
        regaddr = GD32_RCU_AHB2RST;
        break;

      case GD32_RCU_AHB3RST_OFFSET:
        regaddr = GD32_RCU_AHB3RST;
        break;

      case GD32_RCU_AHB4RST_OFFSET:
        regaddr = GD32_RCU_AHB4RST;
        break;

      case GD32_RCU_APB1RST_OFFSET:
        regaddr = GD32_RCU_APB1RST;
        break;

      case GD32_RCU_APB2RST_OFFSET:
        regaddr = GD32_RCU_APB2RST;
        break;

      case GD32_RCU_APB3RST_OFFSET:
        regaddr = GD32_RCU_APB3RST;
        break;

      case GD32_RCU_APB4RST_OFFSET:
        regaddr = GD32_RCU_APB4RST;
        break;

      default:
        break;
    }

  regval = getreg32(regaddr);
  regval &= ~(1 << bit_pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_rcu_lxtal_enable
 *
 * Description:
 *   Enable the External Low Speed crystal oscillator (LXTAL).
 *
 ****************************************************************************/

void gd32_rcu_lxtal_enable(void)
{
  uint32_t regval;

  /* Enable LXTAL */

  regval  = getreg32(GD32_RCU_BDCTL);
  regval |= RCU_BDCTL_LXTALEN;
  putreg32(regval, GD32_RCU_BDCTL);

  /* Wait for LXTAL to be ready */

  while ((getreg32(GD32_RCU_BDCTL) & RCU_BDCTL_LXTALSTB) == 0)
    {
    }
}

/****************************************************************************
 * Name: gd32_rcu_irc64m_enable
 *
 * Description:
 *   Enable the Internal 64M RC oscillator (IRC64M).
 *
 ****************************************************************************/

void gd32_rcu_irc64m_enable(void)
{
  uint32_t regval;

  /* Enable IRC64M */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_IRC64MEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait for IRC64M to be ready */

  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_IRC64MSTB) == 0)
    {
    }
}

/****************************************************************************
 * Name: gd32_rcu_irc64m_disable
 *
 * Description:
 *   Disable the Internal 64M RC oscillator (IRC64M).
 *
 ****************************************************************************/

void gd32_rcu_irc64m_disable(void)
{
  uint32_t regval;

  /* Disable IRC64M */

  regval  = getreg32(GD32_RCU_CTL);
  regval &= ~RCU_CTL_IRC64MEN;
  putreg32(regval, GD32_RCU_CTL);
}

/****************************************************************************
 * Name: gd32_rcu_lpirc4m_enable
 *
 * Description:
 *   Enable the Low Power Internal 4M RC oscillator (LPIRC4M).
 *
 ****************************************************************************/

void gd32_rcu_lpirc4m_enable(void)
{
  uint32_t regval;

  /* Enable LPIRC4M */

  regval  = getreg32(GD32_RCU_ADDCTL1);
  regval |= RCU_ADDCTL1_LPIRC4MEN;
  putreg32(regval, GD32_RCU_ADDCTL1);

  /* Wait for LPIRC4M to be ready */

  while ((getreg32(GD32_RCU_ADDCTL1) & RCU_ADDCTL1_LPIRC4MSTB) == 0)
    {
    }
}

/****************************************************************************
 * Name: gd32_rcu_lpirc4m_disable
 *
 * Description:
 *   Disable the Low Power Internal 4M RC oscillator (LPIRC4M).
 *
 ****************************************************************************/

void gd32_rcu_lpirc4m_disable(void)
{
  uint32_t regval;

  /* Disable LPIRC4M */

  regval  = getreg32(GD32_RCU_ADDCTL1);
  regval &= ~RCU_ADDCTL1_LPIRC4MEN;
  putreg32(regval, GD32_RCU_ADDCTL1);
}
