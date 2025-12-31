/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "gd32e11x.h"
#include "gd32e11x_gpio.h"
#include "gd32e11x_rcu.h"
#include "gd32e11x_lowputc.h"

#include "hardware/gd32e11x_usart.h"

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_GD32E11X_PM_SERIAL_ACTIVITY)
#  define CONFIG_GD32E11X_PM_SERIAL_ACTIVITY 10
#endif

#if defined(CONFIG_PM)
#  define PM_IDLE_DOMAIN             0 /* Revisit */
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

#ifdef HAVE_SERIALDRIVER

/* Which USART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered USART.
 */

/* First pick the console and ttys0.  This could be any of USART0-2,
 * UART3-4.
 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_UART 0
#    define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define CONSOLE_UART 1
#    define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_UART 2
#    define HAVE_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_UART 3
#    define HAVE_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_UART 4
#    define HAVE_CONSOLE 1
#else
#  undef CONSOLE_UART
#  undef HAVE_CONSOLE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  /* Common USART configuration */

  uint32_t        usartbase;     /* Base address of USART registers */
  uint32_t        baud;          /* Configured baud */
  uint32_t        clock;         /* Clock frequency */
  uint32_t        irq;           /* IRQ associated with this USART */
  uint8_t         parity;        /* 0=none, 1=odd, 2=even */
  uint8_t         bits;          /* Number of bits (7 or 8) */
  bool            stop_2bits;    /* True: Configure with 2 stop bits */
  spinlock_t      lock;          /* Spinlock for LTE UART */

#ifdef CONFIG_SERIAL_TERMIOS

#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool            iflow;         /* input flow control (RTS) enabled */
#  endif
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool            oflow;         /* output flow control (CTS) enabled */
#  endif
  uint32_t        tx_gpio;       /* USART TX GPIO pin configuration */
  uint32_t        rx_gpio;       /* USART RX GPIO pin configuration */

#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint32_t        rts_gpio;      /* UART RTS GPIO pin configuration */
#  endif
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint32_t        cts_gpio;      /* UART CTS GPIO pin configuration */
#  endif

#else /* CONFIG_SERIAL_TERMIOS */

#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  const bool      iflow;         /* input flow control (RTS) enabled */
#  endif
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
  const bool      oflow;         /* output flow control (CTS) enabled */
#  endif
  const uint32_t  tx_gpio;       /* USART TX GPIO pin configuration */
  const uint32_t  rx_gpio;       /* USART RX GPIO pin configuration */

#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  const uint32_t  rts_gpio;      /* UART RTS GPIO pin configuration */
#  endif
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t  cts_gpio;      /* UART CTS GPIO pin configuration */
#  endif

#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef HAVE_RS485
  const uint32_t  rs485_dir_gpio;       /* USART RS-485 DIR GPIO pin */
  const bool      rs485_dir_polarity;   /* USART RS-485 DIR TXEN polarity */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset);
static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value);
static void up_setusartint(struct up_dev_s *priv, uint32_t ie);
static void up_disableusartint(struct up_dev_s *priv, uint32_t *ie);
static void up_restoreusartint(struct up_dev_s *priv, uint32_t ie);

static void gd32_usart_configure(struct uart_dev_s *dev);

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper);
#endif
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* Receive / Transmit buffers */

#ifdef CONFIG_GD32E11X_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif

#ifdef CONFIG_GD32E11X_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif

#ifdef CONFIG_GD32E11X_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif

#ifdef CONFIG_GD32E11X_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

#ifdef CONFIG_GD32E11X_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

/* This describes the state of the GD32 USART0 port. */

#ifdef CONFIG_GD32E11X_USART0

static struct up_dev_s g_usart0priv =
{
  .usartbase     = GD32_USART0,
  .baud          = CONFIG_USART0_BAUD,
  .clock         = GD32_PCLK2_FREQUENCY,
  .irq           = GD32_IRQ_USART0,
  .parity        = CONFIG_USART0_PARITY,
  .bits          = CONFIG_USART0_BITS,
  .stop_2bits    = CONFIG_USART0_2STOP,

  .tx_gpio       = GPIO_USART0_TX,
  .rx_gpio       = GPIO_USART0_RX,
  .lock          = SP_UNLOCKED,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART0_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART0_RTS,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART0_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART0_CTS,
#endif

#ifdef CONFIG_USART0_RS485
  .rs485_dir_gpio = GPIO_USART0_RS485_DIR,
#  if (CONFIG_USART0_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static struct uart_dev_s g_usart0port =
{
#if CONSOLE_UART == 0
  .isconsole = true,
#endif
  .recv      =
  {
    .size    = CONFIG_USART0_RXBUFSIZE,
    .buffer  = g_usart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_USART0_TXBUFSIZE,
    .buffer  = g_usart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_usart0priv,
};

#endif

/* This describes the state of the GD32 USART1 port. */

#ifdef CONFIG_GD32E11X_USART1

static struct up_dev_s g_usart1priv =
{
  .usartbase     = GD32_USART1,
  .baud          = CONFIG_USART1_BAUD,
  .clock         = GD32_PCLK1_FREQUENCY,
  .irq           = GD32_IRQ_USART1,
  .parity        = CONFIG_USART1_PARITY,
  .bits          = CONFIG_USART1_BITS,
  .stop_2bits    = CONFIG_USART1_2STOP,

  .tx_gpio       = GPIO_USART1_TX,
  .rx_gpio       = GPIO_USART1_RX,
  .lock          = SP_UNLOCKED,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART1_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART1_RTS,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART1_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART1_CTS,
#endif

#ifdef CONFIG_USART1_RS485
  .rs485_dir_gpio = GPIO_USART1_RS485_DIR,
#  if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static struct uart_dev_s g_usart1port =
{
#if CONSOLE_UART == 1
  .isconsole = true,
#endif
  .recv      =
  {
    .size    = CONFIG_USART1_RXBUFSIZE,
    .buffer  = g_usart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_USART1_TXBUFSIZE,
    .buffer  = g_usart1txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_usart1priv,
};

#endif

/* This describes the state of the GD32 USART2 port. */

#ifdef CONFIG_GD32E11X_USART2

static struct up_dev_s g_usart2priv =
{
  .usartbase     = GD32_USART2,
  .baud          = CONFIG_USART2_BAUD,
  .clock         = GD32_PCLK1_FREQUENCY,
  .irq           = GD32_IRQ_USART2,
  .parity        = CONFIG_USART2_PARITY,
  .bits          = CONFIG_USART2_BITS,
  .stop_2bits    = CONFIG_USART2_2STOP,

  .tx_gpio       = GPIO_USART2_TX,
  .rx_gpio       = GPIO_USART2_RX,
  .lock          = SP_UNLOCKED,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART2_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART2_RTS,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART2_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART2_CTS,
#endif

#ifdef CONFIG_USART2_RS485
  .rs485_dir_gpio = GPIO_USART2_RS485_DIR,
#  if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static struct uart_dev_s g_usart2port =
{
#if CONSOLE_UART == 2
  .isconsole = true,
#endif
  .recv      =
  {
    .size    = CONFIG_USART2_RXBUFSIZE,
    .buffer  = g_usart2rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_USART2_TXBUFSIZE,
    .buffer  = g_usart2txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_usart2priv,
};

#endif

/* This describes the state of the GD32 UART3 port. */

#ifdef CONFIG_GD32E11X_UART3

static struct up_dev_s g_uart3priv =
{
  .usartbase     = GD32_UART3,
  .baud          = CONFIG_UART3_BAUD,
  .clock         = GD32_PCLK1_FREQUENCY,
  .irq           = GD32_IRQ_UART3,
  .parity        = CONFIG_UART3_PARITY,
  .bits          = CONFIG_UART3_BITS,
  .stop_2bits    = CONFIG_UART3_2STOP,

  .tx_gpio       = GPIO_UART3_TX,
  .rx_gpio       = GPIO_UART3_RX,
  .lock          = SP_UNLOCKED,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART3_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_UART3_RTS,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART3_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_UART3_CTS,
#endif

#ifdef CONFIG_UART3_RS485
  .rs485_dir_gpio = GPIO_UART3_RS485_DIR,
#  if (CONFIG_UART3_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static struct uart_dev_s g_uart3port =
{
#if CONSOLE_UART == 3
  .isconsole = true,
#endif
  .recv      =
  {
    .size    = CONFIG_UART3_RXBUFSIZE,
    .buffer  = g_uart3rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART3_TXBUFSIZE,
    .buffer  = g_uart3txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart3priv,
};

#endif

/* This describes the state of the GD32 UART4 port. */

#ifdef CONFIG_GD32E11X_UART4

static struct up_dev_s g_uart4priv =
{
  .usartbase     = GD32_UART4,
  .baud          = CONFIG_UART4_BAUD,
  .clock         = GD32_PCLK1_FREQUENCY,
  .irq           = GD32_IRQ_UART4,
  .parity        = CONFIG_UART4_PARITY,
  .bits          = CONFIG_UART4_BITS,
  .stop_2bits    = CONFIG_UART4_2STOP,

  .tx_gpio       = GPIO_UART4_TX,
  .rx_gpio       = GPIO_UART4_RX,
  .lock          = SP_UNLOCKED,
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART4_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_UART4_RTS,
#endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART4_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_UART4_CTS,
#endif

#ifdef CONFIG_UART4_RS485
  .rs485_dir_gpio = GPIO_UART4_RS485_DIR,
#  if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};

static struct uart_dev_s g_uart4port =
{
#if CONSOLE_UART == 4
  .isconsole = true,
#endif
  .recv      =
  {
    .size    = CONFIG_UART4_RXBUFSIZE,
    .buffer  = g_uart4rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART4_TXBUFSIZE,
    .buffer  = g_uart4txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart4priv,
};

#endif

/* This table lets us iterate over the configured USARTs */

static struct uart_dev_s * const g_uart_devs[GD32_NUSART] =
{
#ifdef CONFIG_GD32E11X_USART0
  [0] = &g_usart0port,
#endif
#ifdef CONFIG_GD32E11X_USART1
  [1] = &g_usart1port,
#endif
#ifdef CONFIG_GD32E11X_USART2
  [2] = &g_usart2port,
#endif
#ifdef CONFIG_GD32E11X_UART3
  [3] = &g_uart3port,
#endif
#ifdef CONFIG_GD32E11X_UART4
  [4] = &g_uart4port,
#endif
};


#ifdef CONFIG_PM
static struct pm_callback_s g_serial_pmcb =
{
  .notify  = up_pm_notify,
  .prepare = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_setusartint
 ****************************************************************************/

static void up_setusartint(struct up_dev_s *priv, uint32_t ie)
{
  uint32_t ctl0;
  uint32_t ctl1;
  uint32_t ctl2;

  /* Save USART interrupt enables */

  ctl0 = up_serialin(priv, GD32_USART_CTL0_OFFSET);
  ctl1 = up_serialin(priv, GD32_USART_CTL1_OFFSET);
  ctl2 = up_serialin(priv, GD32_USART_CTL2_OFFSET);

  /* Adjust enables */

  ctl0 &= ~USART_CTL0_INT_MASK;
  ctl1 &= ~USART_CTL1_INT_MASK;
  ctl2 &= ~USART_CTL2_INT_MASK;

  ctl0 |= (ie & USART_CTL0_INT_MASK);
  ctl1 |= (ie & USART_CTL1_INT_MASK);
  ctl2 |= (ie & USART_CTL2_INT_MASK);

  /* Update registers */

  up_serialout(priv, GD32_USART_CTL0_OFFSET, ctl0);
  up_serialout(priv, GD32_USART_CTL1_OFFSET, ctl1);
  up_serialout(priv, GD32_USART_CTL2_OFFSET, ctl2);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static void up_disableusartint(struct up_dev_s *priv, uint32_t *ie)
{
  irqstate_t flags;
  uint32_t ctl0;
  uint32_t ctl1;
  uint32_t ctl2;

  flags = spin_lock_irqsave(&priv->lock);

  /* Get current interrupt enable settings */

  ctl0 = up_serialin(priv, GD32_USART_CTL0_OFFSET);
  ctl1 = up_serialin(priv, GD32_USART_CTL1_OFFSET);
  ctl2 = up_serialin(priv, GD32_USART_CTL2_OFFSET);

  /* Save the current enable bits */

  if (ie != NULL)
    {
      *ie = ((ctl0 & USART_CTL0_INT_MASK) |
             (ctl1 & USART_CTL1_INT_MASK) |
             (ctl2 & USART_CTL2_INT_MASK));
    }

  /* Disable all interrupts */

  ctl0 &= ~USART_CTL0_INT_MASK;
  ctl1 &= ~USART_CTL1_INT_MASK;
  ctl2 &= ~USART_CTL2_INT_MASK;

  up_serialout(priv, GD32_USART_CTL0_OFFSET, ctl0);
  up_serialout(priv, GD32_USART_CTL1_OFFSET, ctl1);
  up_serialout(priv, GD32_USART_CTL2_OFFSET, ctl2);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static void up_restoreusartint(struct up_dev_s *priv, uint32_t ie)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  up_setusartint(priv, ie);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: gd32_usart_configure
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void gd32_usart_configure(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;
  uint32_t udiv;
  uint32_t intdiv;
  uint32_t fradiv;

  /* Reset USART */

  gd32_usart_reset(priv->usartbase);

  /* Configure baud rate (GD32E11x uses fixed 16x oversampling) */

  udiv   = (priv->clock + priv->baud / 2) / priv->baud;
  intdiv = udiv & 0xfff0;
  fradiv = udiv & 0xf;

  regval = ((USART_BAUD_FRADIV_MASK | USART_BAUD_INTDIV_MASK) &
            (intdiv | fradiv));
  up_serialout(priv, GD32_USART_BAUD_OFFSET, regval);

  /* Configure parity mode */

  regval = up_serialin(priv, GD32_USART_CTL0_OFFSET);
  regval &= ~(USART_CTL0_PM_MASK | USART_CTL0_WL);

  if (priv->parity == 1)
    {
      /* Configure as odd parity */

      regval |= USART_CTL0_PM_ODD;
    }
  else if (priv->parity == 2)
    {
      /* Configure as even parity */

      regval |= USART_CTL0_PM_EVEN;
    }

  /* Configure USART word length */

  if (priv->bits == 9 || (priv->bits == 8 && priv->parity != 0))
    {
      regval |= USART_WL_9BIT;
    }
  else
    {
      regval |= USART_WL_8BIT;
    }

  up_serialout(priv, GD32_USART_CTL0_OFFSET, regval);

  /* Configure STOP bits */

  regval = up_serialin(priv, GD32_USART_CTL1_OFFSET);
  regval &= ~(USART_CTL1_STB_MASK);

  if (priv->stop_2bits)
    {
      regval |= USART_CTL1_STB2BIT;
    }

  up_serialout(priv, GD32_USART_CTL1_OFFSET, regval);

  /* Configure hardware flow control */

  regval  = up_serialin(priv, GD32_USART_CTL2_OFFSET);
  regval &= ~(USART_CTL2_RTSEN | USART_CTL2_CTSEN);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && \
   !defined(CONFIG_GD32E11X_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      regval |= USART_CTL2_RTSEN;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow && (priv->cts_gpio != 0))
    {
      regval |= USART_CTL2_CTSEN;
    }
#endif

  up_serialout(priv, GD32_USART_CTL2_OFFSET, regval);
}
#endif /* CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;

  /* Enable USART clock */

  gd32_usart_clock_enable(priv->usartbase);

  /* Configure pins for USART use */

  gd32_gpio_config(priv->tx_gpio);
  gd32_gpio_config(priv->rx_gpio);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      gd32_gpio_config(priv->rts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow)
    {
      gd32_gpio_config(priv->cts_gpio);
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      gd32_gpio_config(priv->rs485_dir_gpio);
      gd32_gpio_write(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
    }
#endif

  /* Configure the USART */

  gd32_usart_configure(dev);

  /* Enable Rx, Tx, and the USART */

  regval = up_serialin(priv, GD32_USART_CTL0_OFFSET);
  regval |= (USART_CTL0_UEN | USART_CTL0_TEN | USART_CTL0_REN);
  up_serialout(priv, GD32_USART_CTL0_OFFSET, regval);

#endif /* CONFIG_SUPPRESS_UART_CONFIG */

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;

  /* Disable all interrupts */

  up_disableusartint(priv, NULL);

  /* Disable USART */

  regval = up_serialin(priv, GD32_USART_CTL0_OFFSET);
  regval &= ~USART_CTL0_UEN;
  up_serialout(priv, GD32_USART_CTL0_OFFSET, regval);

  /* Disable USART clock */

  gd32_usart_clock_disable(priv->usartbase);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are
 *   called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disableusartint(priv, NULL);
  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           stat0;
  int                passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current USART status */

      stat0 = up_serialin(priv, GD32_USART_STAT0_OFFSET);

      /* Handle incoming, receive bytes */

      if ((stat0 & USART_STAT0_RBNE) != 0)
        {
          /* Received data ready... process incoming bytes */

          uart_recvchars(dev);
        }

      /* Handle outgoing, transmit bytes */

      if ((stat0 & USART_STAT0_TBE) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

          uart_xmitchars(dev);
        }

      /* Check for errors */

      if ((stat0 & (USART_STAT0_ORERR | USART_STAT0_NERR |
                    USART_STAT0_FERR | USART_STAT0_PERR)) != 0)
        {
          /* Clear the error by reading the data register */

          up_serialin(priv, GD32_USART_DATA_OFFSET);
        }

      /* Break out if there are no further pending interrupts */

      if (stat0 == 0)
        {
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct up_dev_s *priv     = (struct up_dev_s *)dev->priv;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return baud */

        cfsetispeed(termiosp, priv->baud);

        /* Return parity */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                           ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->stop_2bits) ? CSTOPB : 0;

        /* Return flow control */

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |= (priv->oflow) ? CCTS_OFLOW : 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |= (priv->iflow) ? CRTS_IFLOW : 0;
#endif

        /* Return number of bits */

        switch (priv->bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          default:
          case 8:
            termiosp->c_cflag |= CS8;
            break;

          case 9:
            termiosp->c_cflag |= CS8 /* CS9 */;
            break;
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct up_dev_s *priv     = (struct up_dev_s *)dev->priv;
        uint32_t baud;
        uint32_t ie;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Decode baud. */

        ret = OK;
        baud = cfgetispeed(termiosp);

        /* Decode number of bits */

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            nbits = 5;
            break;

          case CS6:
            nbits = 6;
            break;

          case CS7:
            nbits = 7;
            break;

          case CS8:
            nbits = 8;
            break;

          default:
            ret = -EINVAL;
            break;
          }

        /* Decode parity */

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits */

        stop2 = (termiosp->c_cflag & CSTOPB) != 0;

        /* Verify that all settings are valid before committing */

        if (ret == OK)
          {
            /* Commit */

            priv->baud  = baud;
            priv->parity = parity;
            priv->bits  = nbits;
            priv->stop_2bits = stop2;

            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            up_disableusartint(priv, &ie);
            gd32_usart_configure(dev);
            up_restoreusartint(priv, ie);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t data;

  /* Get the Rx byte */

  data = up_serialin(priv, GD32_USART_DATA_OFFSET);

  /* Get the Rx status */

  *status = up_serialin(priv, GD32_USART_STAT0_OFFSET);

  /* Return the received byte */

  return data & USART_DATA_MASK;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;
  uint32_t ie;

  flags = spin_lock_irqsave(&priv->lock);

  ie = up_serialin(priv, GD32_USART_CTL0_OFFSET);
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      ie |= USART_CTL0_RBNEIE;
#endif
    }
  else
    {
      ie &= ~USART_CTL0_RBNEIE;
    }

  up_serialout(priv, GD32_USART_CTL0_OFFSET, ie);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return ((up_serialin(priv, GD32_USART_STAT0_OFFSET) &
           USART_STAT0_RBNE) != 0);
}

/****************************************************************************
 * Name: up_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if USART activated RX flow control to block more incoming
 *   data
 *
 * Input Parameters:
 *   dev       - USART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS) && \
    defined(CONFIG_GD32E11X_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      /* Assert/de-assert nRTS set it high resume/stop sending */

      gd32_gpio_write(priv->rts_gpio, upper);

      if (upper)
        {
          /* With heavy Rx traffic, RXNE might be set and data pending.
           * Returning 'true' in such case would cause RXNE left unhandled
           * and causing interrupt storm. Sending end might be also be slow
           * to react on nRTS, and returning 'true' here would prevent
           * processing that data.
           *
           * Therefore, return 'false' so input data is still being processed
           * until sending end reacts on nRTS signal and stops sending more.
           */

          return false;
        }

      return upper;
    }

#else
  if (priv->iflow)
    {
      /* Is the RX buffer full? */

      if (upper)
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral.  When hardware RTS is enabled, this will
           * prevent more data from coming in.
           *
           * This function is only called when UART recv buffer is full,
           * that is: "dev->recv.head + 1 == dev->recv.tail".
           *
           * Logic in "uart_read" will automatically toggle Rx interrupts
           * when buffer is read empty and thus we do not have to re-
           * enable Rx interrupts.
           */

          uart_disablerxint(dev);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input
           * is received.
           */

          uart_enablerxint(dev);
        }
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      gd32_gpio_write(priv->rs485_dir_gpio, priv->rs485_dir_polarity);
    }
#endif

  up_serialout(priv, GD32_USART_DATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint32_t ie = up_serialin(priv, GD32_USART_CTL0_OFFSET);
      ie |= USART_CTL0_TBEIE;
      up_serialout(priv, GD32_USART_CTL0_OFFSET, ie);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      uint32_t ie = up_serialin(priv, GD32_USART_CTL0_OFFSET);
      ie &= ~USART_CTL0_TBEIE;
      up_serialout(priv, GD32_USART_CTL0_OFFSET, ie);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return ((up_serialin(priv, GD32_USART_STAT0_OFFSET) &
           USART_STAT0_TBE) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return ((up_serialin(priv, GD32_USART_STAT0_OFFSET) &
           USART_STAT0_TC) != 0);
}

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */
        }
        break;

      case(PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */
        }
        break;

      case(PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */
        }
        break;

      case(PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */
        }
        break;

      default:
        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* Disable all USARTS */

  up_disableusartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableusartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableusartint(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  up_disableusartint(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  up_disableusartint(TTYS4_DEV.priv, NULL);
#endif

  /* Configure console early (so we can use it for boot messages) */

#ifdef CONSOLE_UART
  g_uart_devs[CONSOLE_UART]->isconsole = true;
  up_setup(g_uart_devs[CONSOLE_UART]);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  int i;
  int start = 0;
  char devname[16];

#ifdef CONSOLE_UART
  /* Register the console */

  uart_register("/dev/console", g_uart_devs[CONSOLE_UART]);
#endif

  /* Register all USARTs */

  strcpy(devname, "/dev/ttySx");

  for (i = 0; i < GD32_NUSART; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (g_uart_devs[i] == NULL)
        {
          continue;
        }

#ifdef CONSOLE_UART
      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->isconsole)
        {
          continue;
        }
#endif

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + start++;
      uart_register(devname, g_uart_devs[i]);
    }

#ifdef CONFIG_PM
  /* Register to receive power management callbacks */

  int ret = pm_register(&g_serial_pmcb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_UART

  struct up_dev_s *priv = (struct up_dev_s *)g_uart_devs[CONSOLE_UART]->priv;
  uint32_t ie;

  up_disableusartint(priv, &ie);
  arm_lowputc(ch);
  up_restoreusartint(priv, ie);
#endif
}

#endif /* HAVE_SERIALDRIVER */
#endif /* USE_SERIALDRIVER */

#if !defined(USE_SERIALDRIVER)

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_UART
  arm_lowputc(ch);
#endif
}

#endif /* !USE_SERIALDRIVER */
