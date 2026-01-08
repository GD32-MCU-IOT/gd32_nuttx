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

#include "gd32e11x_serial.h"

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is there at least one USART enabled and configured as a RS-232 device? */

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

#  ifdef HAVE_SERIALDRIVER

#    if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)

/* USART DMA priority */
#      if defined(CONFIG_GD32E11X_USART_PRIQ)
#        define USART_DMA_PRIO  CONFIG_GD32E11X_USART_PRIQ
#      else
#        define USART_DMA_PRIO  DMA_PRIO_MEDIUM_SELECT
#      endif
#    endif

#    ifdef SERIAL_HAVE_RXDMA
/* The DMA buffer size when using RX DMA to emulate a FIFO.
 *
 * When streaming data, the generic serial layer will be called every time
 * the FIFO receives half this number of bytes.
 */
#    if !defined(CONFIG_GD32E11X_SERIAL_RXDMA_BUFFER_SIZE)
#      define CONFIG_GD32E11X_SERIAL_RXDMA_BUFFER_SIZE 32
#    endif
#    define RXDMA_BUFFER_MASK   (4 - 1)
#    define RXDMA_BUFFER_SIZE   ((CONFIG_GD32E11X_SERIAL_RXDMA_BUFFER_SIZE \
                                + RXDMA_BUFFER_MASK) \
                                & ~RXDMA_BUFFER_MASK)

#    endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  bool            initialized;   /* Indicate that if has been initialized and HW is setup. */

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

  /* DMA configuration and state */

#ifdef SERIAL_HAVE_RXDMA
  uint32_t        rxdma_channel; /* DMA channel assigned for RX */
  uint32_t        rxdmanext;     /* Next byte in the DMA buffer to be read */
  char *const     rxfifo;        /* Receive DMA buffer */
  DMA_HANDLE      rxdma;         /* currently-open receive DMA stream */
  bool            rxenable;      /* DMA-based reception en/disable */
#endif
#ifdef SERIAL_HAVE_TXDMA
  uint32_t        txdma_channel; /* DMA channel assigned for TX */
  DMA_HANDLE      txdma;         /* currently-open transmit DMA stream */
#endif

  /* Interrupt management */

  uint32_t        ie;            /* Saved interrupt enable bits */
  uint32_t        sr;            /* Saved status register */
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

#if defined(SERIAL_HAVE_TX_DMA) || defined(SERIAL_NOT_HAVE_DMA)
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper);
#endif

static void up_send(struct uart_dev_s *dev, int ch);

#if defined(SERIAL_HAVE_RX_DMA) || defined(SERIAL_NOT_HAVE_DMA) || \
    defined(CONFIG_GD32E11X_SERIALBRK_BSDCOMPAT)
static void up_txint(struct uart_dev_s *dev, bool enable);
#endif

static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/* DMA support functions */

#ifdef SERIAL_HAVE_RXDMA
static int  up_dma_setup(struct uart_dev_s *dev);
static void up_dma_shutdown(struct uart_dev_s *dev);
static int  up_dma_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool up_dma_rxavailable(struct uart_dev_s *dev);
static void up_dma_rx_callback(DMA_HANDLE handle, uint16_t status,
                               void *arg);
static size_t up_dma_nextrx(struct up_dev_s *priv);
#endif

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_txavailable(struct uart_dev_s *dev);
static void up_dma_send(struct uart_dev_s *dev);
static void up_dma_txint(struct uart_dev_s *dev, bool enable);
static void up_dma_tx_callback(DMA_HANDLE handle, uint16_t status,
                               void *arg);
#endif

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART operations for RX/TX DMA */

#if defined(SERIAL_HAVE_RXDMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_uart_ops_rxtx_dma =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_dma_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
  .dmatxavail     = up_dma_txavailable,
  .dmasend        = up_dma_send,
};
#endif

/* UART operations for RX DMA only */

#if defined(SERIAL_HAVE_RXDMA) && !defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_uart_ops_rx_dma =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};
#endif

/* UART operations for TX DMA only */

#if !defined(SERIAL_HAVE_RXDMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_uart_ops_tx_dma =
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
  .txint          = up_dma_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
  .dmatxavail     = up_dma_txavailable,
  .dmasend        = up_dma_send,
};
#endif

/* UART operations without DMA */

#ifdef SERIAL_NOT_HAVE_DMA
static const struct uart_ops_s g_uart_ops_no_dma =
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
#endif

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

/* DMA Receive buffers for DMA-enabled USART ports */

#ifdef CONFIG_GD32E11X_USART0_RXDMA
static char g_usart0rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_GD32E11X_USART1_RXDMA
static char g_usart1rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_GD32E11X_USART2_RXDMA
static char g_usart2rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_GD32E11X_UART3_RXDMA
static char g_uart3rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
#endif

#ifdef CONFIG_GD32E11X_UART4_RXDMA
static char g_uart4rxfifo[RXDMA_BUFFER_SIZE]
  __attribute__((aligned(ARMV7M_DCACHE_LINESIZE)));
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

#ifdef CONFIG_GD32E11X_USART0_RXDMA
  .rxdma_channel = DMAMAP_USART0_RX,
  .rxfifo        = g_usart0rxfifo,
#endif
#ifdef CONFIG_GD32E11X_USART0_TXDMA
  .txdma_channel = DMAMAP_USART0_TX,
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
#if defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rxtx_dma,
#elif defined(CONFIG_GD32E11X_USART0_RXDMA) && !defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rx_dma,
#elif !defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_tx_dma,
#else
  .ops       = &g_uart_ops_no_dma,
#endif
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

#ifdef CONFIG_GD32E11X_USART1_RXDMA
  .rxdma_channel = DMAMAP_USART1_RX,
  .rxfifo        = g_usart1rxfifo,
#endif
#ifdef CONFIG_GD32E11X_USART1_TXDMA
  .txdma_channel = DMAMAP_USART1_TX,
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
#if defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rxtx_dma,
#elif defined(CONFIG_GD32E11X_USART0_RXDMA) && !defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rx_dma,
#elif !defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_tx_dma,
#else
  .ops       = &g_uart_ops_no_dma,
#endif
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

#ifdef CONFIG_GD32E11X_USART2_RXDMA
  .rxdma_channel = DMAMAP_USART2_RX,
  .rxfifo        = g_usart2rxfifo,
#endif
#ifdef CONFIG_GD32E11X_USART2_TXDMA
  .txdma_channel = DMAMAP_USART2_TX,
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
#if defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rxtx_dma,
#elif defined(CONFIG_GD32E11X_USART0_RXDMA) && !defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rx_dma,
#elif !defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_tx_dma,
#else
  .ops       = &g_uart_ops_no_dma,
#endif
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

#ifdef CONFIG_GD32E11X_UART3_RXDMA
  .rxdma_channel = DMAMAP_UART3_RX,
  .rxfifo        = g_uart3rxfifo,
#endif
#ifdef CONFIG_GD32E11X_UART3_TXDMA
  .txdma_channel = DMAMAP_UART3_TX,
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
#if defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rxtx_dma,
#elif defined(CONFIG_GD32E11X_USART0_RXDMA) && !defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_rx_dma,
#elif !defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
  .ops       = &g_uart_ops_tx_dma,
#else
  .ops       = &g_uart_ops_no_dma,
#endif
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

#ifdef CONFIG_GD32E11X_UART4_RXDMA
  .rxdma_channel = DMAMAP_UART4_RX,
  .rxfifo        = g_uart4rxfifo,
#endif
#ifdef CONFIG_GD32E11X_UART4_TXDMA
  .txdma_channel = DMAMAP_UART4_TX,
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
  uint32_t regval;

  /* Save the interrupt mask */

  priv->ie = ie;

  /* And restore the interrupt state (see the interrupt enable/usage
   * table above)
   */

  regval  = up_serialin(priv, GD32_USART_CTL0_OFFSET);
  regval &= ~(USART_CTL0_USED_INTS);
  regval |= (ie & (USART_CTL0_USED_INTS));
  up_serialout(priv, GD32_USART_CTL0_OFFSET, regval);

  regval  = up_serialin(priv, GD32_USART_CTL2_OFFSET);
  regval &= ~USART_CTL2_ERRIE;
  regval |= (ie & USART_CTL2_ERRIE);
  up_serialout(priv, GD32_USART_CTL2_OFFSET, regval);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static void up_disableusartint(struct up_dev_s *priv, uint32_t *ie)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (ie)
    {
      uint32_t ctl0;
      uint32_t ctl2;

      ctl0 = up_serialin(priv, GD32_USART_CTL0_OFFSET);
      ctl2 = up_serialin(priv, GD32_USART_CTL2_OFFSET);

      /* Return the current interrupt mask value for the used interrupts.
       * Notice that this depends on the fact that none of the used interrupt
       * enable bits overlap.  This logic would fail if we needed the break
       * interrupt!
       */

      *ie = (ctl0 & (USART_CTL0_USED_INTS)) | (ctl2 & USART_CTL2_ERRIE);
    }

  /* Disable all interrupts */

  up_setusartint(priv, 0);

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
 * Name: up_dma_nextrx
 *
 * Description:
 *   Get the next DMA reception position
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static size_t up_dma_nextrx(struct up_dev_s *priv)
{
  size_t dmaresidual;

  dmaresidual = gd32_dma_tansnum_get(priv->rxdma);

  return (RXDMA_BUFFER_SIZE - (size_t)dmaresidual);
}
#endif

/****************************************************************************
 * Name: up_dma_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called
 *   for DMA-enabled serial ports.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int up_dma_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int result;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = up_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

  /* Acquire the DMA channel for RX if configured */

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdma_channel != 0)
    {
      priv->rxdma = gd32_dma_channel_alloc(priv->rxdma_channel);
      if (priv->rxdma == NULL)
        {
          return -EBUSY;
        }

      /* Configure for circular DMA reception into the RX FIFO */

      dma_single_data_parameter_struct dma_init_struct;

      dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
      dma_init_struct.memory0_addr = (uint32_t)priv->rxfifo;
      dma_init_struct.number = RXDMA_BUFFER_SIZE;
      dma_init_struct.periph_addr = priv->usartbase + GD32_USART_DATA_OFFSET;
      dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
      dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
      dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
      dma_init_struct.priority = USART_DMA_PRIO;
      dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;

      gd32_dma_setup(priv->rxdma, &dma_init_struct, 1);

      /* Reset DMA reading position */

      priv->rxdmanext = 0;

      /* Enable receive DMA for the UART */

      uint32_t regval = up_serialin(priv, GD32_USART_CTL2_OFFSET);
      regval |= USART_CTL2_DENR;
      up_serialout(priv, GD32_USART_CTL2_OFFSET, regval);

      /* Start the DMA channel */

      gd32_dma_start(priv->rxdma, up_dma_rx_callback, dev, 0);
    }
#endif

  /* Acquire the DMA channel for TX if configured */

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdma_channel != 0)
    {
      priv->txdma = gd32_dma_channel_alloc(priv->txdma_channel);
      if (priv->txdma == NULL)
        {
#ifdef SERIAL_HAVE_RXDMA
          if (priv->rxdma)
            {
              gd32_dma_channel_free(priv->rxdma);
              priv->rxdma = NULL;
            }
#endif

          return -EBUSY;
        }

      /* Enable transmit DMA for the UART */

      uint32_t regval = up_serialin(priv, GD32_USART_CTL2_OFFSET);
      regval |= USART_CTL2_DENT;
      up_serialout(priv, GD32_USART_CTL2_OFFSET, regval);
    }
#endif

  priv->initialized = true;

  return OK;
}
#endif

/****************************************************************************
 * Name: up_dma_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed (DMA version)
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Stop and free the RX DMA channel */

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdma != NULL)
    {
      gd32_dma_stop(priv->rxdma);
      gd32_dma_channel_free(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif

  /* Stop and free the TX DMA channel */

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdma != NULL)
    {
      gd32_dma_stop(priv->txdma);
      gd32_dma_channel_free(priv->txdma);
      priv->txdma = NULL;
    }
#endif

  priv->initialized = false;

  /* Perform the standard shutdown */

  up_shutdown(dev);
}
#endif

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
  int                passes;
  bool handled;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

#if defined(CONFIG_PM) && CONFIG_GD32E11X_PM_SERIAL_ACTIVITY > 0
  /* Report serial activity to the power management logic */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_GD32E11X_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      /* Get the masked USART status */

      priv->sr = up_serialin(priv, GD32_USART_STAT0_OFFSET);

#ifdef HAVE_RS485
      /* Transmission is complete */

      if (((priv->sr & USART_STAT0_TC) != 0) &&
          ((priv->ie & USART_CTL0_TCIE) != 0) &&
          ((priv->ie & USART_CTL0_TBEIE) != 0))
        {
          gd32_gpio_write(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
          up_restoreusartint(priv, priv->ie);
        }
#endif

      /* Handle incoming, receive bytes */

      if (((priv->sr & USART_STAT0_RBNE) != 0) &&
          ((priv->ie & USART_CTL0_RBNEIE) != 0))
        {
          /* Received data ready... process incoming bytes */

          uart_recvchars(dev);
          handled = true;
        }

#ifdef SERIAL_HAVE_RXDMA
      /* Handle IDLE interrupt for DMA reception */

      if (((priv->sr & USART_STAT0_IDLEF) != 0) &&
          ((priv->ie & USART_CTL0_IDLIE) != 0))
        {
          uart_recvchars(dev);
          up_serialin(priv, GD32_USART_DATA_OFFSET);
        }
#endif

      /* Clear USART error flags */

      if ((priv->sr & (USART_STAT0_ORERR | USART_STAT0_NERR |
                       USART_STAT0_FERR | USART_STAT0_PERR)) != 0)
        {
          /* Clear the error by reading the data register */

          up_serialin(priv, GD32_USART_DATA_OFFSET);
        }

      /* Handle outgoing, transmit bytes */

      if (((priv->sr & USART_STAT0_TBE) != 0) &&
          ((priv->ie & USART_CTL0_TBEIE) != 0))
        {
          /* Transmit data register empty ... process outgoing bytes */

          uart_xmitchars(dev);
          handled = true;
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
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* The driver only support 8/9 bit modes and therefore is
         * no way to report 9-bit mode, we always claim 8.
         */

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stop_2bits) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          ((priv->iflow) ? CRTS_IFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          ((priv->oflow) ? CCTS_OFLOW : 0) |
#endif
          CS8;

        cfsetispeed(termiosp, priv->baud);

        /* TODO: CRTS_IFLOW, CCTS_OFLOW */
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Perform some sanity checks before accepting any changes */

        if (((termiosp->c_cflag & CSIZE) != CS8)
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
           )
          {
            ret = -EINVAL;
            break;
          }

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->stop_2bits = (termiosp->c_cflag & CSTOPB) != 0;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif

        /* Note that since there is no way to request 9-bit mode
         * and no way to support 5/6/7-bit modes, we ignore them
         * all here.
         */

        /* Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);

        /* Effect the changes immediately - note that we do not implement
         * TCSADRAIN / TCSAFLUSH
         */

        gd32_usart_configure(dev);
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_GD32E11X_USART_BREAKS
#  ifdef CONFIG_GD32E11X_SERIALBRK_BSDCOMPAT
    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags;
        uint32_t tx_break;

        flags = enter_critical_section();

       /* Disconnect TX to USART */

        up_txint(dev, false);

        /* Configure TX as a GPIO output pin and Send a break signal */

        tx_break = (~(GPIO_CFG_MODE_MASK | GPIO_CFG_OUTPUT_SET) &
                    priv->tx_gpio) | GPIO_CFG_MODE_OUTPUT;
        gd32_gpio_config(tx_break);

        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;

        flags = enter_critical_section();

        /* Connect TX to USART */

        gd32_gpio_config(priv->tx_gpio);

        /* Enable further tx activity */

        up_txint(dev, true);

        leave_critical_section(flags);
      }
      break;
#  else /* CONFIG_GD32E11X_SERIALBRK_BSDCOMPAT */
    case TIOCSBRK:  /* No BSD compatibility: Turn break on for M bit times */
      {
        irqstate_t flags;

        flags = enter_critical_section();
        regval = up_serialin(priv, GD32_USART_CTL0_OFFSET);
        regval |= USART_CTL0_SBKCMD;
        up_serialout(priv, GD32_USART_CTL0_OFFSET, regval);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* No BSD compatibility: May turn off break too soon */
      {
        uint32_t cr1;
        irqstate_t flags;

        flags = enter_critical_section();
        regval = up_serialin(priv, GD32_USART_CTL0_OFFSET);
        regval &= ~ USART_CTL0_SBKCMD;
        up_serialout(priv, GD32_USART_CTL0_OFFSET, regval);
        leave_critical_section(flags);
      }
      break;
#  endif
#endif /* CONFIG_GD32E11X_USART_BREAKS */

  /* Only available in USART0,1,2 */

#ifdef CONFIG_GD32E11X_USART_INVERT
    case TIOCSINVERT:
      {
        uint32_t regaddr;
        irqstate_t flags;

        flags = enter_critical_section();

        regval = up_serialin(priv, GD32_USART_CTL3_OFFSET);

        if (arg & SER_INVERT_ENABLED_RX)
          {
            regval |= USART_CTL3_RINV;
          }
        else
          {
            regval &= ~USART_CTL3_RINV;
          }

        if (arg & SER_INVERT_ENABLED_TX)
          {
            regval |= USART_CTL3_TINV;
          }
        else
          {
            regval &= ~USART_CTL3_TINV;
          }

        /* R/TINV bit fields can written when UEN in CTL0 is 0 */

        regaddr = GD32_USART_CTL0(priv->usartbase);

        /* Clear UEN bit */

        modifyreg32(regaddr, USART_CTL0_UEN, 0);

        up_serialout(priv, GD32_USART_CTL3_OFFSET, regval);

        /* Set UEN bit */

        modifyreg32(regaddr, 0, USART_CTL0_UEN);

        leave_critical_section(flags);
      }
     break;
#endif /* CONFIG_GD32E11X_USART_INVERT */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART DMA buffer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int up_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int c = 0;

  if (up_dma_nextrx(priv) != priv->rxdmanext)
    {
      /* Invalidate the DMA buffer */

      up_invalidate_dcache((uintptr_t)priv->rxfifo,
                           (uintptr_t)priv->rxfifo + RXDMA_BUFFER_SIZE);

      /* Read from the DMA buffer */

      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;
      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          priv->rxdmanext = 0;
        }
    }

  return c;
}
#endif

/****************************************************************************
 * Name: up_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts (DMA version)
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;
  uint32_t ie;

  /* Enable or disable DMA reception */

  priv->rxenable = enable;

  flags = spin_lock_irqsave(&priv->lock);

  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when the DMA receive data */

      ie |= USART_CTL0_IDLEIE;
    }
  else
    {
      ie &= ~(USART_CTL0_RBNEIE | USART_CTL0_PERRIE |
              USART_CTL2_ERRIE);
    }

  /* Then set the new interrupt state */

  up_restoreusartint(priv, ie);
  spin_unlock_irqrestore(&priv->lock, flags);
}
#endif

/****************************************************************************
 * Name: up_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty (DMA version)
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static bool up_dma_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Compare our receive pointer to the current DMA pointer */

  return (up_dma_nextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: up_dma_rx_callback
 *
 * Description:
 *   DMA RX callback - called when DMA buffer is filled
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_rx_callback(DMA_HANDLE handle, uint16_t status, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  if (priv->rxenable && up_dma_rxavailable(dev))
    {
      uart_recvchars(dev);
    }
}
#endif

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_TX_DMA) || defined(SERIAL_NOT_HAVE_DMA)
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
#endif

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_TX_DMA) || defined(SERIAL_NOT_HAVE_DMA)
static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;
  uint32_t ie;

  flags = spin_lock_irqsave(&priv->lock);

  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#ifdef CONFIG_USART_ERRINTS
      ie |= (USART_CTL0_RBNEIE | USART_CTL0_PERRIE | USART_CTL2_ERRIE);
#else
      ie |= USART_CTL0_RBNEIE;
#endif
#endif
    }
  else
    {
      ie &= ~(USART_CTL0_RBNEIE | USART_CTL0_PERRIE | USART_CTL2_ERRIE);
    }

  /* Then set the new interrupt state */

  up_restoreusartint(priv, ie);

  spin_unlock_irqrestore(&priv->lock, flags);
}
#endif

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_TX_DMA) || defined(SERIAL_NOT_HAVE_DMA)
static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return ((up_serialin(priv, GD32_USART_STAT0_OFFSET) &
           USART_STAT0_RBNE) != 0);
}
#endif

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
 * Name: up_dma_send
 *
 * Description:
 *   Start DMA transfer for TX
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_send(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  dma_single_data_parameter_struct dma_init_struct;

  /* Stop any ongoing DMA */

  gd32_dma_stop(priv->txdma);

  /* Reset the number sent */

  dev->dmatx.nbytes = 0;

  /* Configure DMA for transmit */

  dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
  dma_init_struct.memory0_addr = (uint32_t)dev->dmatx.buffer;
  dma_init_struct.number = dev->dmatx.length;
  dma_init_struct.periph_addr = priv->usartbase + GD32_USART_DATA_OFFSET;
  dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
  dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_init_struct.priority = USART_DMA_PRIO;
  dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;

  gd32_dma_setup(priv->txdma, &dma_init_struct, 1);

  /* Start transmission with callback */

  gd32_dma_start(priv->txdma, up_dma_tx_callback, (void *)dev,
                 USART_DMA_INTEN);

  /* Wait for TX to complete */

  while (up_txempty(dev) == 0);
}
#endif

/****************************************************************************
 * Name: up_dma_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts (DMA version)
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_txint(struct uart_dev_s *dev, bool enable)
{
  /* Nothing to do - DMA handles it */
}
#endif

/****************************************************************************
 * Name: up_dma_txavailable
 *
 * Description:
 *   Informs DMA that Tx data is available
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_txavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Only send when the DMA is idle */

  if (gd32_dma_tansnum_get(priv->txdma) == 0)
    {
      uart_xmitchars_dma(dev);
    }
}
#endif

/****************************************************************************
 * Name: up_dma_tx_callback
 *
 * Description:
 *   DMA TX callback - called when DMA transfer completes
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_tx_callback(DMA_HANDLE handle, uint16_t status, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Update bytes transferred */

  if (status & DMA_INTF_FTFIF)
    {
      dev->dmatx.nbytes += dev->dmatx.length;
      if (dev->dmatx.nlength)
        {
          /* Set up DMA on next buffer */

          dma_single_data_parameter_struct dma_init_struct;

          dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
          dma_init_struct.memory0_addr = (uint32_t)dev->dmatx.nbuffer;
          dma_init_struct.number = dev->dmatx.nlength;
          dma_init_struct.periph_addr = priv->usartbase +
                                        GD32_USART_DATA_OFFSET;
          dma_init_struct.periph_memory_width = DMA_WIDTH_8BITS_SELECT;
          dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
          dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
          dma_init_struct.priority = USART_DMA_PRIO;
          dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;

          gd32_dma_setup(priv->txdma, &dma_init_struct, 1);

          dev->dmatx.length  = dev->dmatx.nlength;
          dev->dmatx.nlength = 0;

          gd32_dma_start(priv->txdma, up_dma_tx_callback,
                         (void *)dev, USART_DMA_INTEN);

          return;
        }
    }
  else if (status & DMA_INTF_HTFIF)
    {
      dev->dmatx.nbytes += dev->dmatx.length / 2;
    }

  uart_xmitchars_done(dev);
}
#endif

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RX_DMA) || defined(SERIAL_NOT_HAVE_DMA) || \
    defined(CONFIG_GD32E11X_SERIALBRK_BSDCOMPAT)
static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint32_t ie = priv->ie | USART_CTL0_TBEIE;
      ie |= USART_CTL0_TBEIE;
      up_setusartint(priv, ie);

      /* If RS-485 is supported on this USART, also enable the TCIE */

#  ifdef HAVE_RS485
      if (priv->rs485_dir_gpio != 0)
        {
          ie |= USART_CTL0_INT_TCIE;
        }
#  endif

#else
      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      uint32_t ie = priv->ie & ~USART_CTL0_TBEIE;
      up_setusartint(priv, ie);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}
#endif

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
      case (PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */
        }
        break;

      case (PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */
        }
        break;

      case (PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */
        }
        break;

      case (PM_SLEEP):
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

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Name: gd32_serial_get_uart
 *
 * Description:
 *   Get serial driver structure for GD32 USART
 *
 ****************************************************************************/

#ifdef HAVE_SERIALDRIVER
uart_dev_t *gd32_serial_get_uart(int uart_idx)
{
  if (uart_idx < 0 || uart_idx >= GD32_NUSART || !g_uart_devs[uart_idx])
    {
      return NULL;
    }

  struct up_dev_s *priv = (struct up_dev_s *)g_uart_devs[uart_idx]->priv;
  if (!priv->initialized)
    {
      return NULL;
    }

  return g_uart_devs[uart_idx];
}
#endif /* HAVE_SERIALDRIVER */
#endif /* USE_SERIALDRIVER */

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
  unsigned i;

  /* Disable all USART interrupts */

  for (i = 0; i < GD32_NUSART; i++)
    {
      if (g_uart_devs[i])
        {
          up_disableusartint(g_uart_devs[i]->priv, NULL);
        }
    }

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
 * Name: gd32_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void gd32_serial_dma_poll(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_GD32E11X_USART0_RXDMA
  if (g_usart0priv.rxdma != NULL)
    {
      up_dma_rx_callback(g_usart0priv.rxdma, 0, &g_usart0port);
    }
#endif

#ifdef CONFIG_GD32E11X_USART1_RXDMA
  if (g_usart1priv.rxdma != NULL)
    {
      up_dma_rx_callback(g_usart1priv.rxdma, 0, &g_usart1port);
    }
#endif

#ifdef CONFIG_GD32E11X_USART2_RXDMA
  if (g_usart2priv.rxdma != NULL)
    {
      up_dma_rx_callback(g_usart2priv.rxdma, 0, &g_usart2port);
    }
#endif

#ifdef CONFIG_GD32E11X_UART3_RXDMA
  if (g_uart3priv.rxdma != NULL)
    {
      up_dma_rx_callback(g_uart3priv.rxdma, 0, &g_uart3port);
    }
#endif

#ifdef CONFIG_GD32E11X_UART4_RXDMA
  if (g_uart4priv.rxdma != NULL)
    {
      up_dma_rx_callback(g_uart4priv.rxdma, 0, &g_uart4port);
    }
#endif

  leave_critical_section(flags);
}
#endif

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

#  endif /* HAVE_SERIALDRIVER */
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
