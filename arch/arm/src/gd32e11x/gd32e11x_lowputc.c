/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_lowputc.c
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

#include <arch/board/board.h>

#include "arm_internal.h"

#include "gd32e11x_gpio.h"
#include "gd32e11x_rcu.h"
#include "gd32e11x_lowputc.h"

#include "hardware/gd32e11x_rcu.h"
#include "hardware/gd32e11x_usart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console */

#ifdef HAVE_CONSOLE

#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_USART0
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB2EN
#    define GD32_CONSOLE_APBEN         RCU_APB2EN_USART0EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK2_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_USART0_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_USART0_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_USART0_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_USART0_2STOP
#    define GD32_CONSOLE_TX            GPIO_USART0_TX
#    define GD32_CONSOLE_RX            GPIO_USART0_RX
#    ifdef CONFIG_USART0_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_USART0_RS485_DIR
#      if (CONFIG_USART0_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_USART1
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_USART1EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_USART1_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_USART1_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_USART1_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_USART1_2STOP
#    define GD32_CONSOLE_TX            GPIO_USART1_TX
#    define GD32_CONSOLE_RX            GPIO_USART1_RX
#    ifdef CONFIG_USART1_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_USART1_RS485_DIR
#      if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_USART2
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_USART2EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_USART2_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_USART2_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_USART2_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_USART2_2STOP
#    define GD32_CONSOLE_TX            GPIO_USART2_TX
#    define GD32_CONSOLE_RX            GPIO_USART2_RX
#    ifdef CONFIG_USART2_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_USART2_RS485_DIR
#      if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_UART3
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_UART3EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_UART3_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_UART3_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_UART3_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_UART3_2STOP
#    define GD32_CONSOLE_TX            GPIO_UART3_TX
#    define GD32_CONSOLE_RX            GPIO_UART3_RX
#    ifdef CONFIG_UART3_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_UART3_RS485_DIR
#      if (CONFIG_UART3_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define GD32_CONSOLE_BASE          GD32_UART4
#    define GD32_CONSOLE_APBEN_REG     GD32_RCU_APB1EN
#    define GD32_CONSOLE_APBEN         RCU_APB1EN_UART4EN
#    define GD32_CONSOLE_CLOCK         GD32_PCLK1_FREQUENCY
#    define GD32_CONSOLE_BAUD          CONFIG_UART4_BAUD
#    define GD32_CONSOLE_PARITY        CONFIG_UART4_PARITY
#    define GD32_CONSOLE_NBITS         CONFIG_UART4_BITS
#    define GD32_CONSOLE_2STOP         CONFIG_UART4_2STOP
#    define GD32_CONSOLE_TX            GPIO_UART4_TX
#    define GD32_CONSOLE_RX            GPIO_UART4_RX
#    ifdef CONFIG_UART4_RS485
#      define GD32_CONSOLE_RS485_DIR   GPIO_UART4_RS485_DIR
#      if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
#        define GD32_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define GD32_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  endif

/* CTL0 settings */

#  if GD32_CONSOLE_NBITS == 9
#    define USART_CTL0_WL_VALUE      USART_WL_9BIT
#  else
#    define USART_CTL0_WL_VALUE      USART_WL_8BIT
#  endif

#  if GD32_CONSOLE_PARITY == 1
#    define USART_CTL0_PARITY_VALUE  USART_CTL0_PM_ODD
#  elif GD32_CONSOLE_PARITY == 2
#    define USART_CTL0_PARITY_VALUE  USART_CTL0_PM_EVEN
#  else
#    define USART_CTL0_PARITY_VALUE  USART_CTL0_PM_NONE
#  endif

/* CTL1 settings */

#  if GD32_CONSOLE_2STOP != 0
#    define USART_CTL1_STB_VALUE     USART_CTL1_STB2BIT
#  else
#    define USART_CTL1_STB_VALUE     USART_CTL1_STB1BIT
#  endif

#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  /* Wait until the transmit data register is empty */

  while ((getreg32(GD32_CONSOLE_BASE + GD32_USART_STAT0_OFFSET) &
          USART_STAT0_TBE) == 0)
    {
    }

#ifdef GD32_CONSOLE_RS485_DIR
  gd32_gpio_write(GD32_CONSOLE_RS485_DIR,
                  GD32_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Then send the character */

  putreg32((uint32_t)(ch & USART_DATA_MASK),
           GD32_CONSOLE_BASE + GD32_USART_DATA_OFFSET);

#ifdef GD32_CONSOLE_RS485_DIR
  /* Wait for transmission complete */

  while ((getreg32(GD32_CONSOLE_BASE + GD32_USART_STAT0_OFFSET) &
          USART_STAT0_TC) == 0)
    {
    }

  gd32_gpio_write(GD32_CONSOLE_RS485_DIR,
                  !GD32_CONSOLE_RS485_DIR_POLARITY);
#endif

#else
  (void)ch;
#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: gd32_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void gd32_lowsetup(void)
{
#ifdef HAVE_CONSOLE
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;
  uint32_t udiv;
  uint32_t intdiv;
  uint32_t fradiv;
#endif

  /* Enable console peripheral clock */

  modifyreg32(GD32_CONSOLE_APBEN_REG, 0, GD32_CONSOLE_APBEN);

  /* Configure TX/RX pins (board-provided encodings) */

#ifdef GD32_CONSOLE_TX
  gd32_gpio_config(GD32_CONSOLE_TX);
#endif
#ifdef GD32_CONSOLE_RX
  gd32_gpio_config(GD32_CONSOLE_RX);
#endif

#ifdef GD32_CONSOLE_RS485_DIR
  gd32_gpio_config(GD32_CONSOLE_RS485_DIR);
  gd32_gpio_write(GD32_CONSOLE_RS485_DIR,
                  !GD32_CONSOLE_RS485_DIR_POLARITY);
#endif

#ifndef CONFIG_SUPPRESS_UART_CONFIG

  /* Reset USART and enable USART clock */

  gd32_usart_reset(GD32_CONSOLE_BASE);
  gd32_usart_clock_enable(GD32_CONSOLE_BASE);

  /* Disable USART before configuring it */

  regval  = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);
  regval &= ~USART_CTL0_UEN;
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);

  /* Configure CTL0 word length/parity */

  regval  = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);
  regval &= ~(USART_CTL0_WL | USART_CTL0_PM_MASK | USART_CTL0_PCEN);
  regval |= (USART_CTL0_WL_VALUE | USART_CTL0_PARITY_VALUE);
  if (GD32_CONSOLE_PARITY != 0)
    {
      regval |= USART_CTL0_PCEN;
    }

  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);

  /* Configure stop bits */

  regval  = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL1_OFFSET);
  regval &= ~USART_CTL1_STB_MASK;
  regval |= USART_CTL1_STB_VALUE;
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL1_OFFSET);

  /* Configure baud rate (oversampling by 16, GD32E11x lacks OVSMOD) */

  udiv   = (GD32_CONSOLE_CLOCK + GD32_CONSOLE_BAUD / 2) /
            GD32_CONSOLE_BAUD;
  intdiv = udiv & 0xfff0u;
  fradiv = udiv & 0x0fu;
  regval = (intdiv | fradiv) &
           (USART_BAUD_INTDIV_MASK | USART_BAUD_FRADIV_MASK);
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_BAUD_OFFSET);

  /* Enable Rx, Tx, and USART */

  regval  = getreg32(GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);
  regval |= (USART_CTL0_UEN | USART_CTL0_TEN | USART_CTL0_REN);
  putreg32(regval, GD32_CONSOLE_BASE + GD32_USART_CTL0_OFFSET);

#endif /* !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: gd32_usart_reset
 *
 * Description:
 *   Reset the USART.
 *
 ****************************************************************************/

void gd32_usart_reset(uint32_t usartbase)
{
  uint32_t rcu_rst;
  uintptr_t regaddr;

  switch (usartbase)
    {
      default:
        return;

#ifdef CONFIG_GD32E11X_USART0
      case GD32_USART0:
        rcu_rst = RCU_APB2RST_USART0RST;
        regaddr = GD32_RCU_APB2RST;
        break;
#endif

#ifdef CONFIG_GD32E11X_USART1
      case GD32_USART1:
        rcu_rst = RCU_APB1RST_USART1RST;
        regaddr = GD32_RCU_APB1RST;
        break;
#endif

#ifdef CONFIG_GD32E11X_USART2
      case GD32_USART2:
        rcu_rst = RCU_APB1RST_USART2RST;
        regaddr = GD32_RCU_APB1RST;
        break;
#endif

#ifdef CONFIG_GD32E11X_UART3
      case GD32_UART3:
        rcu_rst = RCU_APB1RST_UART3RST;
        regaddr = GD32_RCU_APB1RST;
        break;
#endif

#ifdef CONFIG_GD32E11X_UART4
      case GD32_UART4:
        rcu_rst = RCU_APB1RST_UART4RST;
        regaddr = GD32_RCU_APB1RST;
        break;
#endif
    }

  modifyreg32(regaddr, 0, rcu_rst);
  modifyreg32(regaddr, rcu_rst, 0);
}

/****************************************************************************
 * Name: gd32_usart_clock_enable
 *
 * Description:
 *   Enable USART clock
 *
 ****************************************************************************/

void gd32_usart_clock_enable(uint32_t usartbase)
{
  uint32_t rcu_en;
  uintptr_t regaddr;

  switch (usartbase)
    {
      default:
        return;

#ifdef CONFIG_GD32E11X_USART0
      case GD32_USART0:
        rcu_en  = RCU_APB2EN_USART0EN;
        regaddr = GD32_RCU_APB2EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_USART1
      case GD32_USART1:
        rcu_en  = RCU_APB1EN_USART1EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_USART2
      case GD32_USART2:
        rcu_en  = RCU_APB1EN_USART2EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_UART3
      case GD32_UART3:
        rcu_en  = RCU_APB1EN_UART3EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_UART4
      case GD32_UART4:
        rcu_en  = RCU_APB1EN_UART4EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif
    }

  modifyreg32(regaddr, 0, rcu_en);
}

/****************************************************************************
 * Name: gd32_usart_clock_disable
 *
 * Description:
 *   Disable USART clock
 *
 ****************************************************************************/

void gd32_usart_clock_disable(uint32_t usartbase)
{
  uint32_t rcu_en;
  uintptr_t regaddr;

  switch (usartbase)
    {
      default:
        return;

#ifdef CONFIG_GD32E11X_USART0
      case GD32_USART0:
        rcu_en  = RCU_APB2EN_USART0EN;
        regaddr = GD32_RCU_APB2EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_USART1
      case GD32_USART1:
        rcu_en  = RCU_APB1EN_USART1EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_USART2
      case GD32_USART2:
        rcu_en  = RCU_APB1EN_USART2EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_UART3
      case GD32_UART3:
        rcu_en  = RCU_APB1EN_UART3EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif

#ifdef CONFIG_GD32E11X_UART4
      case GD32_UART4:
        rcu_en  = RCU_APB1EN_UART4EN;
        regaddr = GD32_RCU_APB1EN;
        break;
#endif
    }

  modifyreg32(regaddr, rcu_en, 0);
}
