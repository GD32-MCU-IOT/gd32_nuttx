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
#include "gd32e11x_gpio.h"
#include "gd32e11x_rcu.h"
#include "gd32e11x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow 2 milliseconds for the IRC8M to become ready. */

#define IRC8M_STARTUP_TIMEOUT   (2 * CONFIG_BOARD_LOOPSPERMSEC)

/* Allow 10 milliseconds for the HXTAL to become ready. */

#define HXTAL_STARTUP_TIMEOUT   (10 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific clocking initialization logic */

#if defined(CONFIG_GD32E11X_GD32E11X)
#else
#  error "Unknown GD32 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following is to prevent Vcore fluctuations caused by frequency
 * switching. It is strongly recommended to include it to avoid issues.
 */

#define RCU_MODIFY_4(__delay)   do {                                         \
                                    volatile uint32_t i, reg;                \
                                    if (0 != __delay) {                      \
                                        for (i = 0; i < __delay; i++);       \
                                        reg = getreg32(GD32_RCU_CFG0);       \
                                        reg &= ~RCU_CFG0_AHBPSC_MASK;        \
                                        reg |= RCU_CFG0_AHBPSC_DIV2;         \
                                        putreg32(reg, GD32_RCU_CFG0);        \
                                        for (i = 0; i < __delay; i++);       \
                                        reg = getreg32(GD32_RCU_CFG0);       \
                                        reg &= ~RCU_CFG0_AHBPSC_MASK;        \
                                        reg |= RCU_CFG0_AHBPSC_DIV4;         \
                                        putreg32(reg, GD32_RCU_CFG0);        \
                                        for (i = 0; i < __delay; i++);       \
                                        reg = getreg32(GD32_RCU_CFG0);       \
                                        reg &= ~RCU_CFG0_AHBPSC_MASK;        \
                                        reg |= RCU_CFG0_AHBPSC_DIV8;         \
                                        putreg32(reg, GD32_RCU_CFG0);        \
                                        for (i = 0; i < __delay; i++);       \
                                        reg = getreg32(GD32_RCU_CFG0);       \
                                        reg &= ~RCU_CFG0_AHBPSC_MASK;        \
                                        reg |= RCU_CFG0_AHBPSC_DIV16;        \
                                        putreg32(reg, GD32_RCU_CFG0);        \
                                        for (i = 0; i < __delay; i++);       \
                                    }                                        \
                                } while (0);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef GD32_BOARD_SYSCLK_IRC8MEN
/****************************************************************************
 * Name: gd32_system_clock_irc8m
 *
 * Description:
 *   Select the IRC8M as system clock (8MHz).
 *
 ****************************************************************************/

static void gd32_system_clock_irc8m(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Enable IRC8M */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_IRC8MEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until IRC8M is stable or the startup time is longer than
   * IRC8M_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC8MSTB);
    }
  while ((0 == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC8MSTB))
    {
      while (1)
        {
        }
    }

  /* AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= RCU_CFG0_AHBPSC_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB2 = AHB/1 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= RCU_CFG0_APB2PSC_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB1 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= RCU_CFG0_APB1PSC_DIV2;
  putreg32(regval, GD32_RCU_CFG0);

  /* Select IRC8M as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_IRC8M;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until IRC8M is selected as system clock */

  while (0 != (getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK))
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

  /* AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= RCU_CFG0_AHBPSC_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB2 = AHB/1 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= RCU_CFG0_APB2PSC_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB1 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= RCU_CFG0_APB1PSC_DIV2;
  putreg32(regval, GD32_RCU_CFG0);

  /* Select HXTAL as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_HXTAL;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until HXTAL is selected as system clock */

  while (0 == (getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_HXTAL))
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_PLL_IRC8M)
/****************************************************************************
 * Name: gd32_system_clock_pll_irc8m
 *
 * Description:
 *   Configure the system clock by PLL which selects IRC8M as its clock
 *   source. Supports up to 120MHz.
 *
 ****************************************************************************/

static void gd32_system_clock_pll_irc8m(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Enable IRC8M */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_IRC8MEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until IRC8M is stable or the startup time is longer than
   * IRC8M_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC8MSTB);
    }
  while ((0 == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC8MSTB))
    {
      while (1)
        {
        }
    }

  /* IRC8M is stable */

  regval  = getreg32(GD32_FMC_WS);
  regval &= ~FMC_WS_WSCNT_MASK;
  regval |= GD32_FMC_WAIT_STATE;
  putreg32(regval, GD32_FMC_WS);

  /* AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= GD32_RCU_CFG0_AHB_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB2 = AHB/APB2_PSC_DIV */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= GD32_RCU_CFG0_APB2_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB1 = AHB/APB1_PSC_DIV */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= GD32_RCU_CFG0_APB1_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* CK_PLL = (CK_IRC8M/2) * PLLMF  (PLL source is IRC8M/2) */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF_MASK | RCU_CFG0_PLLMF_MSB);
  regval |= GD32_RCU_CFG0_PLLMF;
  putreg32(regval, GD32_RCU_CFG0);

  /* Enable PLL */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLLEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until PLL is stable */

  while (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_PLLSTB))
    {
    }

  /* Select PLL as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_PLL;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until PLL is selected as system clock */

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK)
          != RCU_CFG0_SCSS_PLL)
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_PLL_HXTAL)
/****************************************************************************
 * Name: gd32_system_clock_pll_hxtal
 *
 * Description:
 *   Configure the system clock by PLL which selects HXTAL as its clock
 *   source. Supports up to 120MHz.
 *
 ****************************************************************************/

static void gd32_system_clock_pll_hxtal(void)
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

  /* HXTAL is stable */

  regval  = getreg32(GD32_FMC_WS);
  regval &= ~FMC_WS_WSCNT_MASK;
  regval |= GD32_FMC_WAIT_STATE;
  putreg32(regval, GD32_FMC_WS);

  /* AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= GD32_RCU_CFG0_AHB_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB2 = AHB/APB2_PSC_DIV */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= GD32_RCU_CFG0_APB2_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* APB1 = AHB/APB1_PSC_DIV */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= GD32_RCU_CFG0_APB1_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* CK_PLL = (CK_PREDIV0) * PLLMF
   * Configure PLL source and multiplier
   */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_PLLMF_MASK | RCU_CFG0_PLLMF_MSB);
  regval |= (RCU_PLL_PLLSEL_HXTAL_IRC48M | GD32_RCU_CFG0_PLLMF);
  putreg32(regval, GD32_RCU_CFG0);

#if (CONFIG_GD32E113_BOARD_PREDIV0_SELECT_PLL1)
  /* CK_PREDIV0 = (CK_HXTAL)/2 *10 /10 = 4 MHz */

  regval  = getreg32(GD32_RCU_CFG1);
  regval &= ~(RCU_CFG1_PLLPRESEL | RCU_CFG1_PREDV0SEL |
              RCU_CFG1_PREDV0_MASK);
  regval |= (RCU_CFG1_PLLPRESEL_HXTAL | RCU_CFG1_PREDV0SEL_PLL1 |
             GD32_RCU_PLL1_MULF | GD32_RCU_PREDV1_DIV | GD32_RCU_PREDV0_DIV);
  putreg32(regval, GD32_RCU_CFG1);

  /* enable PLL1 */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLL1EN;
  putreg32(regval, GD32_RCU_CTL);

  /* wait till PLL1 is ready */

  while (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_PLL1STB))
    {
    }

#else
  /* CK_PREDIV0 = (CK_HXTAL)/2 = 4 MHz */

  regval  = getreg32(GD32_RCU_CFG1);
  regval &= ~(RCU_CFG1_PLLPRESEL | RCU_CFG1_PREDV0SEL |
              RCU_CFG1_PREDV0_MASK);
  regval |= (RCU_CFG1_PLLPRESEL_HXTAL | RCU_CFG1_PREDV0SEL_HXTAL_IRC48M |
             GD32_RCU_PREDV0_DIV);
  putreg32(regval, GD32_RCU_CFG1);
#endif

  /* Enable PLL */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLLEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until PLL is stable */

  while (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_PLLSTB))
    {
    }

  /* Select PLL as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_PLL;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until PLL is selected as system clock */

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK)
          != RCU_CFG0_SCSS_PLL)
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
#ifdef GD32_BOARD_SYSCLK_IRC8MEN

  /* Select IRC8M as SYSCLK based on board.h setting. */

  gd32_system_clock_irc8m();

#elif defined (GD32_BOARD_SYSCLK_HXTAL)

  /* Select HXTAL as SYSCLK based on board.h setting. */

  gd32_system_clock_hxtal();

#elif defined (GD32_BOARD_SYSCLK_PLL_IRC8M)

  /* Select PLL which source is IRC8M as SYSCLK based on board.h setting. */

  gd32_system_clock_pll_irc8m();

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
 *   Called to initialize the GD32E11X.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void gd32_clockconfig(void)
{
  uint32_t regval;
  uint32_t timeout = 0;

  /* Reset the RCU clock configuration to the default reset state */

  /* Set IRC8MEN bit */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_IRC8MEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until IRC8M is stable */

  while (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC8MSTB))
    {
    }

  /* If currently using PLL, apply frequency reduction steps to prevent
   * Vcore fluctuation
   */

  if ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK) == RCU_CFG0_SCSS_PLL)
    {
      RCU_MODIFY_4(0x50);
    }

  /* Switch to IRC8M as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  putreg32(regval, GD32_RCU_CFG0);
  do
    {
      timeout++;
    }
  while ((2000 >= timeout));

  /* Reset HXTALEN, CKMEN, PLLEN, PLL1EN and PLL2EN bits */

  regval  = getreg32(GD32_RCU_CTL);
  regval &= ~(RCU_CTL_PLLEN | RCU_CTL_PLL1EN | RCU_CTL_PLL2EN |
              RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
  putreg32(regval, GD32_RCU_CTL);

  /* Disable all interrupts */

  putreg32(0x00ff0000, GD32_RCU_INT);

  /* Reset CFG0 and CFG1 registers */

  putreg32(0x00000000, GD32_RCU_CFG0);
  putreg32(0x00000000, GD32_RCU_CFG1);

  /* Reset HXTALBPS bit */

  regval  = getreg32(GD32_RCU_CTL);
  regval &= ~RCU_CTL_HXTALBPS;
  putreg32(regval, GD32_RCU_CTL);

#if defined(CONFIG_ARCH_BOARD_GD32E11X_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  gd32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
   * AHB/APBx prescalers
   */

  gd32_system_clock_config();
#endif
}

/****************************************************************************
 * Name: gd32_rcu_periph_clock_enable
 *
 * Description:
 *   Enable the peripherals clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_enable(uint32_t periph)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph & 0x1f;
  reg_off = (periph >> 6);
  regaddr = GD32_RCU_BASE + reg_off * 4;

  regval = getreg32(regaddr);
  regval |= (1 << bit_pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_rcu_periph_clock_disable
 *
 * Description:
 *   Disable the peripherals clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_disable(uint32_t periph)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph & 0x1f;
  reg_off = (periph >> 6);
  regaddr = GD32_RCU_BASE + reg_off * 4;

  regval = getreg32(regaddr);
  regval &= ~(1 << bit_pos);
  putreg32(regval, regaddr);
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
 *   If CONFIG_ARCH_BOARD_GD32E11X_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called gd32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM

void gd32_clock_enable(void)
{
#if defined(CONFIG_ARCH_BOARD_GD32E11X_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  gd32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
   * AHB/APBx prescalers
   */

  gd32_system_clock_config();

#endif
}
#endif
