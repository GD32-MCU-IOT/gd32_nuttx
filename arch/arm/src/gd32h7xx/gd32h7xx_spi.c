/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_spi.c
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
 * The external functions, gd32_spi0/1/2/3/4/5select and
 * gd32_spi0/1/2/3/4/5status must be provided by board-specific logic.
 * They are implementations of the select and status methods of the SPI
 * interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 * All other methods (including gd32_spibus_initialize()) are provided by
 * common GD32H7xx logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in gd32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide gd32_spi[n]select() and gd32_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a call to gd32_spibus_initialize() in your low level
 *      application initialization logic.
 *   4. The handle returned by gd32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/clock.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/cache.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "gd32h7xx.h"

#if defined(CONFIG_GD32H7_SPI0) || defined(CONFIG_GD32H7_SPI1) || \
    defined(CONFIG_GD32H7_SPI2) || defined(CONFIG_GD32H7_SPI3) || \
    defined(CONFIG_GD32H7_SPI4) || defined(CONFIG_GD32H7_SPI5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI clock source frequency */

#define GD32_APB1_FREQ          (GD32_PCLK1_FREQUENCY)
#define GD32_APB2_FREQ          (GD32_PCLK2_FREQUENCY)

/* SPI timeout definitions */

#ifndef CONFIG_GD32H7_SPI_TIMEOUTMS
#  define CONFIG_GD32H7_SPI_TIMEOUTMS 500
#endif

#define SPI_TIMEOUT_TICKS       MSEC2TICK(CONFIG_GD32H7_SPI_TIMEOUTMS)

/* SPI DMA definitions */

#ifdef CONFIG_GD32H7_SPI_DMA
#  ifdef CONFIG_ARMV7M_DCACHE
/* D-Cache line size is 32 bytes on Cortex-M7, buffers must be aligned
 * and sized to cache line boundaries to avoid cache coherency issues.
 */

#    define SPI_DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#    define SPI_DMA_BUFFER_ALIGN   aligned_data(ARMV7M_DCACHE_LINESIZE)
#  else
/* No D-Cache, use minimal alignment for DMA */

#    define SPI_DMA_BUFFER_MASK    (sizeof(uint32_t) - 1)
#    define SPI_DMA_BUFFER_ALIGN   aligned_data(sizeof(uint32_t))
#  endif

#  if defined(CONFIG_GD32H7_SPI0_DMA_BUFFER) && \
      CONFIG_GD32H7_SPI0_DMA_BUFFER > 0
#    define SPI0_DMA_BUFSIZE_ADJ   ((CONFIG_GD32H7_SPI0_DMA_BUFFER + \
                                     SPI_DMA_BUFFER_MASK) & ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32H7_SPI1_DMA_BUFFER) && \
      CONFIG_GD32H7_SPI1_DMA_BUFFER > 0
#    define SPI1_DMA_BUFSIZE_ADJ   ((CONFIG_GD32H7_SPI1_DMA_BUFFER + \
                                     SPI_DMA_BUFFER_MASK) & ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32H7_SPI2_DMA_BUFFER) && \
      CONFIG_GD32H7_SPI2_DMA_BUFFER > 0
#    define SPI2_DMA_BUFSIZE_ADJ   ((CONFIG_GD32H7_SPI2_DMA_BUFFER + \
                                     SPI_DMA_BUFFER_MASK) & ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32H7_SPI3_DMA_BUFFER) && \
      CONFIG_GD32H7_SPI3_DMA_BUFFER > 0
#    define SPI3_DMA_BUFSIZE_ADJ   ((CONFIG_GD32H7_SPI3_DMA_BUFFER + \
                                     SPI_DMA_BUFFER_MASK) & ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32H7_SPI4_DMA_BUFFER) && \
      CONFIG_GD32H7_SPI4_DMA_BUFFER > 0
#    define SPI4_DMA_BUFSIZE_ADJ   ((CONFIG_GD32H7_SPI4_DMA_BUFFER + \
                                     SPI_DMA_BUFFER_MASK) & ~SPI_DMA_BUFFER_MASK)
#  endif

#  if defined(CONFIG_GD32H7_SPI5_DMA_BUFFER) && \
      CONFIG_GD32H7_SPI5_DMA_BUFFER > 0
#    define SPI5_DMA_BUFSIZE_ADJ   ((CONFIG_GD32H7_SPI5_DMA_BUFFER + \
                                     SPI_DMA_BUFFER_MASK) & ~SPI_DMA_BUFFER_MASK)
#  endif

#  ifndef CONFIG_GD32H7_SPI_DMATHRESHOLD
#    define CONFIG_GD32H7_SPI_DMATHRESHOLD 4
#  endif
#endif /* CONFIG_GD32H7_SPI_DMA */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_spidev_s
{
  struct spi_dev_s spidev;        /* Externally visible part of the
                                   * SPI interface */
  uint32_t         spibase;       /* SPIn base address */
  uint32_t         spiclock;      /* SPI clock frequency */
#ifdef CONFIG_GD32H7_SPI_INTERRUPTS
  uint8_t          spiirq;        /* SPI IRQ number */
#endif
  mutex_t          lock;          /* Held while chip is selected for
                                   * mutual exclusion */
  uint32_t         frequency;     /* Requested clock frequency */
  uint32_t         actual;        /* Actual clock frequency */
  uint8_t          nbits;         /* Width of word in bits (4 to 32) */
  uint8_t          mode;          /* Mode 0,1,2,3 */
  bool             initialized;   /* Has SPI interface been initialized */
#ifdef CONFIG_GD32H7_SPI_DMA
  volatile uint16_t rxresult;     /* Result of the RX DMA */
  volatile uint16_t txresult;     /* Result of the TX DMA */
#ifdef CONFIG_SPI_TRIGGER
  bool             defertrig;     /* Flag indicating that trigger should
                                   * be deferred */
  bool             trigarmed;     /* Flag indicating that the trigger is
                                   * armed */
#endif
  uint8_t          rxreqid;       /* DMAMUX RX request ID */
  uint8_t          txreqid;       /* DMAMUX TX request ID */
  uint8_t          *rxbuf;        /* RX DMA buffer */
  uint8_t          *txbuf;        /* TX DMA buffer */
  size_t           buflen;        /* DMA buffer length */
  DMA_HANDLE       rxdma;         /* DMA channel handle for RX transfers */
  DMA_HANDLE       txdma;         /* DMA channel handle for TX transfers */
  sem_t            rxsem;         /* Wait for RX DMA to complete */
  sem_t            txsem;         /* Wait for TX DMA to complete */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(struct gd32_spidev_s *priv,
                                  uint32_t offset);
static inline void spi_putreg(struct gd32_spidev_s *priv, uint32_t offset,
                              uint32_t value);
static inline uint8_t spi_getreg8(struct gd32_spidev_s *priv,
                                  uint32_t offset);
static inline void spi_putreg8(struct gd32_spidev_s *priv, uint32_t offset,
                               uint8_t value);
static inline void spi_modifyreg(struct gd32_spidev_s *priv,
                                 uint32_t offset, uint32_t clrbits,
                                 uint32_t setbits);

/* Helper functions */

static void     spi_flush_fifo(struct gd32_spidev_s *priv);

/* SPI methods */

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int      spi_hwfeatures(struct spi_dev_s *dev,
                               spi_hwfeatures_t features);
#endif
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void     spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                             void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int      spi_trigger(struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                             size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                              size_t nwords);
#endif

/* DMA support */

#ifdef CONFIG_GD32H7_SPI_DMA
static int      spi_dmarxwait(struct gd32_spidev_s *priv);
static int      spi_dmatxwait(struct gd32_spidev_s *priv);
static void     spi_dmarxcallback(DMA_HANDLE handle, uint16_t isr,
                                  void *arg);
static void     spi_dmatxcallback(DMA_HANDLE handle, uint16_t isr,
                                  void *arg);
static void     spi_dmarxsetup(struct gd32_spidev_s *priv,
                               void *rxbuffer, void *rxdummy,
                               size_t nwords);
static void     spi_dmatxsetup(struct gd32_spidev_s *priv,
                               const void *txbuffer, const void *txdummy,
                               size_t nwords);
static inline void spi_dmarxstart(struct gd32_spidev_s *priv);
static inline void spi_dmatxstart(struct gd32_spidev_s *priv);
#endif

/* Initialization */

static void     spi_bus_initialize(struct gd32_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI DMA buffers */

#ifdef CONFIG_GD32H7_SPI_DMA
#  if defined(SPI0_DMA_BUFSIZE_ADJ)
static uint8_t g_spi0_txbuf[SPI0_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
static uint8_t g_spi0_rxbuf[SPI0_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
#  endif
#  if defined(SPI1_DMA_BUFSIZE_ADJ)
static uint8_t g_spi1_txbuf[SPI1_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
static uint8_t g_spi1_rxbuf[SPI1_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
#  endif
#  if defined(SPI2_DMA_BUFSIZE_ADJ)
static uint8_t g_spi2_txbuf[SPI2_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
static uint8_t g_spi2_rxbuf[SPI2_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
#  endif
#  if defined(SPI3_DMA_BUFSIZE_ADJ)
static uint8_t g_spi3_txbuf[SPI3_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
static uint8_t g_spi3_rxbuf[SPI3_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
#  endif
#  if defined(SPI4_DMA_BUFSIZE_ADJ)
static uint8_t g_spi4_txbuf[SPI4_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
static uint8_t g_spi4_rxbuf[SPI4_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
#  endif
#  if defined(SPI5_DMA_BUFSIZE_ADJ)
static uint8_t g_spi5_txbuf[SPI5_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
static uint8_t g_spi5_rxbuf[SPI5_DMA_BUFSIZE_ADJ] SPI_DMA_BUFFER_ALIGN;
#  endif
#endif /* CONFIG_GD32H7_SPI_DMA */

#ifdef CONFIG_GD32H7_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct gd32_spidev_s g_spi0dev =
{
  .spidev   =
  {
    .ops = &g_spi0ops
  },
  .spibase  = GD32_SPI0_BASE,
  .spiclock = GD32_APB2_FREQ,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32H7_SPI_INTERRUPTS
  .spiirq   = GD32_IRQ_SPI0,
#endif
#ifdef CONFIG_GD32H7_SPI_DMA
#  ifdef CONFIG_GD32H7_SPI0_DMA
  .rxreqid  = DMA_REQUEST_SPI0_RX,
  .txreqid  = DMA_REQUEST_SPI0_TX,
#    if defined(SPI0_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi0_rxbuf,
  .txbuf    = g_spi0_txbuf,
  .buflen   = SPI0_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxreqid  = 0,
  .txreqid  = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32H7_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct gd32_spidev_s g_spi1dev =
{
  .spidev   =
  {
    .ops = &g_spi1ops
  },
  .spibase  = GD32_SPI1_BASE,
  .spiclock = GD32_APB1_FREQ,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32H7_SPI_INTERRUPTS
  .spiirq   = GD32_IRQ_SPI1,
#endif
#ifdef CONFIG_GD32H7_SPI_DMA
#  ifdef CONFIG_GD32H7_SPI1_DMA
  .rxreqid  = DMA_REQUEST_SPI1_RX,
  .txreqid  = DMA_REQUEST_SPI1_TX,
#    if defined(SPI1_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi1_rxbuf,
  .txbuf    = g_spi1_txbuf,
  .buflen   = SPI1_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxreqid  = 0,
  .txreqid  = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32H7_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi2cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct gd32_spidev_s g_spi2dev =
{
  .spidev   =
  {
    .ops = &g_spi2ops
  },
  .spibase  = GD32_SPI2_BASE,
  .spiclock = GD32_APB1_FREQ,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32H7_SPI_INTERRUPTS
  .spiirq   = GD32_IRQ_SPI2,
#endif
#ifdef CONFIG_GD32H7_SPI_DMA
#  ifdef CONFIG_GD32H7_SPI2_DMA
  .rxreqid  = DMA_REQUEST_SPI2_RX,
  .txreqid  = DMA_REQUEST_SPI2_TX,
#    if defined(SPI2_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi2_rxbuf,
  .txbuf    = g_spi2_txbuf,
  .buflen   = SPI2_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxreqid  = 0,
  .txreqid  = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32H7_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct gd32_spidev_s g_spi3dev =
{
  .spidev   =
  {
    .ops = &g_spi3ops
  },
  .spibase  = GD32_SPI3_BASE,
  .spiclock = GD32_APB2_FREQ,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32H7_SPI_INTERRUPTS
  .spiirq   = GD32_IRQ_SPI3,
#endif
#ifdef CONFIG_GD32H7_SPI_DMA
#  ifdef CONFIG_GD32H7_SPI3_DMA
  .rxreqid  = DMA_REQUEST_SPI3_RX,
  .txreqid  = DMA_REQUEST_SPI3_TX,
#    if defined(SPI3_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi3_rxbuf,
  .txbuf    = g_spi3_txbuf,
  .buflen   = SPI3_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxreqid  = 0,
  .txreqid  = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32H7_SPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi4cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct gd32_spidev_s g_spi4dev =
{
  .spidev   =
  {
    .ops = &g_spi4ops
  },
  .spibase  = GD32_SPI4_BASE,
  .spiclock = GD32_APB2_FREQ,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32H7_SPI_INTERRUPTS
  .spiirq   = GD32_IRQ_SPI4,
#endif
#ifdef CONFIG_GD32H7_SPI_DMA
#  ifdef CONFIG_GD32H7_SPI4_DMA
  .rxreqid  = DMA_REQUEST_SPI4_RX,
  .txreqid  = DMA_REQUEST_SPI4_TX,
#    if defined(SPI4_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi4_rxbuf,
  .txbuf    = g_spi4_txbuf,
  .buflen   = SPI4_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxreqid  = 0,
  .txreqid  = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_GD32H7_SPI5
static const struct spi_ops_s g_spi5ops =
{
  .lock              = spi_lock,
  .select            = gd32_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi5cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = NULL,
};

static struct gd32_spidev_s g_spi5dev =
{
  .spidev   =
  {
    .ops = &g_spi5ops
  },
  .spibase  = GD32_SPI5_BASE,
  .spiclock = GD32_APB2_FREQ,
  .lock     = NXMUTEX_INITIALIZER,
#ifdef CONFIG_GD32H7_SPI_INTERRUPTS
  .spiirq   = GD32_IRQ_SPI5,
#endif
#ifdef CONFIG_GD32H7_SPI_DMA
#  ifdef CONFIG_GD32H7_SPI5_DMA
  .rxreqid  = DMA_REQUEST_SPI5_RX,
  .txreqid  = DMA_REQUEST_SPI5_TX,
#    if defined(SPI5_DMA_BUFSIZE_ADJ)
  .rxbuf    = g_spi5_rxbuf,
  .txbuf    = g_spi5_txbuf,
  .buflen   = SPI5_DMA_BUFSIZE_ADJ,
#    endif
#  else
  .rxreqid  = 0,
  .txreqid  = 0,
#  endif
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg / spi_putreg / spi_getreg8 / spi_putreg8
 ****************************************************************************/

static inline uint32_t spi_getreg(struct gd32_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->spibase + offset);
}

static inline void spi_putreg(struct gd32_spidev_s *priv, uint32_t offset,
                              uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

static inline uint8_t spi_getreg8(struct gd32_spidev_s *priv,
                                  uint32_t offset)
{
  /* GD32H7xx device memory requires 32-bit aligned access.
   * Use 32-bit read and extract LSB.
   */

  return (uint8_t)(getreg32(priv->spibase + offset) & 0xff);
}

static inline void spi_putreg8(struct gd32_spidev_s *priv, uint32_t offset,
                               uint8_t value)
{
  /* GD32H7xx device memory requires 32-bit aligned access.
   * Write as 32-bit with upper bytes zero.
   */

  putreg32((uint32_t)value, priv->spibase + offset);
}

static inline void spi_modifyreg(struct gd32_spidev_s *priv,
                                 uint32_t offset, uint32_t clrbits,
                                 uint32_t setbits)
{
  modifyreg32(priv->spibase + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: spi_wait_flag
 *
 * Description:
 *   Wait for a status flag with timeout.
 *
 * Input Parameters:
 *   priv - SPI device instance
 *   flag - Status flag to wait for (e.g., SPI_STAT_TP, SPI_STAT_RP,
 *          or SPI_STAT_SPD)
 *
 * Returned Value:
 *   OK on success, -ETIMEDOUT on timeout.
 *
 ****************************************************************************/

static inline int spi_wait_flag(struct gd32_spidev_s *priv, uint32_t flag)
{
  clock_t start = clock_systime_ticks();

  while (!(spi_getreg(priv, GD32_SPI_STAT_OFFSET) & flag))
    {
      if ((clock_systime_ticks() - start) > SPI_TIMEOUT_TICKS)
        {
          return -ETIMEDOUT;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: spi_enable
 *
 * Description:
 *   Enable or disable the SPI peripheral.
 *   GD32H7xx requires suspend (MSPDR) before disable if master is active.
 *
 ****************************************************************************/

static void spi_enable(struct gd32_spidev_s *priv, bool enable)
{
  if (enable)
    {
      spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_SPIEN);
    }
  else
    {
      /* If master transfer is active, request suspend first */

      if (spi_getreg(priv, GD32_SPI_CTL0_OFFSET) & SPI_CTL0_MSTART)
        {
          spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_MSPDR);

          /* Wait for suspend with timeout */

          if (spi_wait_flag(priv, SPI_STAT_SPD) != OK)
            {
              spierr("spi_enable: SPD timeout waiting for suspend\n");
            }

          spi_putreg(priv, GD32_SPI_STATC_OFFSET, SPI_STATC_SPDC);
        }

      /* Clear MSTART along with SPIEN to ensure a clean state.
       * This prevents any residual transfer from continuing when SPI is
       * re-enabled, and ensures the 0x5a dummy byte (written for DMA bug
       * workaround) won't be transmitted before DMA is ready.
       */

      spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET,
                    SPI_CTL0_SPIEN | SPI_CTL0_MSTART, 0);
    }
}

/****************************************************************************
 * Name: spi_clear_errors
 ****************************************************************************/

static void spi_clear_errors(struct gd32_spidev_s *priv)
{
  spi_putreg(priv, GD32_SPI_STATC_OFFSET,
             SPI_STATC_TXURERRC | SPI_STATC_RXORERRC |
             SPI_STATC_CRCERRC | SPI_STATC_FERRC |
             SPI_STATC_CONFERRC);
}

/****************************************************************************
 * Name: spi_lock
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }
  else
    {
      return nxmutex_unlock(&priv->lock);
    }
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *   H7xx prescaler is in SPI_CFG0.PSC[2:0] (not CTL0 like E11x).
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t setbits;
  uint32_t actual;

  if (frequency != priv->frequency)
    {
      spi_enable(priv, false);

      if (frequency >= priv->spiclock / 2)
        {
          setbits = SPI_CFG0_PSC_2;
          actual  = priv->spiclock / 2;
        }
      else if (frequency >= priv->spiclock / 4)
        {
          setbits = SPI_CFG0_PSC_4;
          actual  = priv->spiclock / 4;
        }
      else if (frequency >= priv->spiclock / 8)
        {
          setbits = SPI_CFG0_PSC_8;
          actual  = priv->spiclock / 8;
        }
      else if (frequency >= priv->spiclock / 16)
        {
          setbits = SPI_CFG0_PSC_16;
          actual  = priv->spiclock / 16;
        }
      else if (frequency >= priv->spiclock / 32)
        {
          setbits = SPI_CFG0_PSC_32;
          actual  = priv->spiclock / 32;
        }
      else if (frequency >= priv->spiclock / 64)
        {
          setbits = SPI_CFG0_PSC_64;
          actual  = priv->spiclock / 64;
        }
      else if (frequency >= priv->spiclock / 128)
        {
          setbits = SPI_CFG0_PSC_128;
          actual  = priv->spiclock / 128;
        }
      else
        {
          setbits = SPI_CFG0_PSC_256;
          actual  = priv->spiclock / 256;
        }

      spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET,
                    SPI_CFG0_PSC_MASK, setbits);

      spi_enable(priv, true);

      priv->frequency = frequency;
      priv->actual    = actual;
    }

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n",
          frequency, priv->actual);
  return priv->actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *   H7xx: CKPL/CKPH are in SPI_CFG1 (not CTL0 like E11x).
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t setbits;
  uint32_t clrbits;
  int flush_count;

  spiinfo("mode=%d\n", mode);

  if (mode != priv->mode)
    {
      spi_enable(priv, false);

      switch (mode)
        {
          case SPIDEV_MODE0: /* CKPL=0, CKPH=0 */
            setbits = 0;
            clrbits = SPI_CFG1_CKPL | SPI_CFG1_CKPH;
            break;

          case SPIDEV_MODE1: /* CKPL=0, CKPH=1 */
            setbits = SPI_CFG1_CKPH;
            clrbits = SPI_CFG1_CKPL;
            break;

          case SPIDEV_MODE2: /* CKPL=1, CKPH=0 */
            setbits = SPI_CFG1_CKPL;
            clrbits = SPI_CFG1_CKPH;
            break;

          case SPIDEV_MODE3: /* CKPL=1, CKPH=1 */
            setbits = SPI_CFG1_CKPL | SPI_CFG1_CKPH;
            clrbits = 0;
            break;

          default:
            return;
        }

      spi_modifyreg(priv, GD32_SPI_CFG1_OFFSET, clrbits, setbits);

      spi_enable(priv, true);

      /* Flush RX FIFO with limit to prevent infinite loop */

      flush_count = 32;  /* Max FIFO depth is 16 words */
      while ((spi_getreg(priv, GD32_SPI_STAT_OFFSET) &
              SPI_STAT_RP) != 0 && flush_count-- > 0)
        {
          spi_getreg(priv, GD32_SPI_RDATA_OFFSET);
        }

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *   H7xx supports 4-32 bits via SPI_CFG0.DZ[4:0].
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  if (nbits != priv->nbits)
    {
      spi_enable(priv, false);

      /* Clamp to valid range */

      if (nbits < 4)
        {
          nbits = 4;
        }
      else if (nbits > 32)
        {
          nbits = 32;
        }

      /* Set data size DZ[4:0] = nbits - 1 */

      spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET,
                    SPI_CFG0_DZ_MASK, SPI_CFG0_DZ(nbits));

      /* Configure FIFO access mode based on data width */

      if (nbits <= 8)
        {
          /* Byte access for 4-8 bit transfers */

          spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET,
                        SPI_CFG0_WORDEN, SPI_CFG0_BYTEN);
        }
      else if (nbits <= 16)
        {
          /* Half-word access for 9-16 bit transfers */

          spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET,
                        SPI_CFG0_WORDEN | SPI_CFG0_BYTEN, 0);
        }
      else
        {
          /* Word access for 17-32 bit transfers */

          spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET,
                        SPI_CFG0_BYTEN, SPI_CFG0_WORDEN);
        }

      spi_enable(priv, true);

      priv->nbits = nbits;
    }
}

#ifdef CONFIG_SPI_HWFEATURES
/****************************************************************************
 * Name: spi_hwfeatures
 ****************************************************************************/

static int spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features)
{
  /* Not yet implemented */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI.
 *   GD32H7xx uses TXSIZE=1 mode so that MSTART auto-clears after the
 *   single frame transfer completes.
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  uint32_t ret;

  DEBUGASSERT(priv != NULL);

  /* Set TXSIZE=1 before starting transfer.
   * SPI must be disabled to modify CTL1, re-enable after.
   */

  spi_enable(priv, false);
  spi_putreg(priv, GD32_SPI_CTL1_OFFSET, SPI_CTL1_TXSIZE(1));
  spi_enable(priv, true);

  /* Start master transfer */

  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_MSTART);

  /* Wait for TX space available with timeout */

  if (spi_wait_flag(priv, SPI_STAT_TP) != OK)
    {
      spierr("spi_send: TP timeout\n");
      return 0xff;
    }

  /* Write data */

  spi_putreg8(priv, GD32_SPI_TDATA_OFFSET, (uint8_t)(wd & 0xff));

  /* Wait for RX data available with timeout */

  if (spi_wait_flag(priv, SPI_STAT_RP) != OK)
    {
      spierr("spi_send: RP timeout\n");
      return 0xff;
    }

  /* Read received data */

  ret = (uint32_t)spi_getreg8(priv, GD32_SPI_RDATA_OFFSET);

  spiinfo("Sent: %04" PRIx32 " Return: %04" PRIx32 "\n", wd, ret);

  return ret;
}

/****************************************************************************
 * Name: spi_exchange (no DMA) / spi_exchange_nodma (with DMA threshold)
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA.
 *   GD32H7xx: Use TXSIZE mode with MSTART for precise transfer control.
 *
 ****************************************************************************/

static void spi_flush_fifo(struct gd32_spidev_s *priv)
{
  int count = 32;
  uint32_t stat;

  /* Flush RX FIFO */

  while (count-- > 0)
    {
      stat = spi_getreg(priv, GD32_SPI_STAT_OFFSET);
      if ((stat & SPI_STAT_RP) == 0)
        {
          break;
        }

      (void)spi_getreg(priv, GD32_SPI_RDATA_OFFSET);
    }

  /* Clear all error flags */

  spi_putreg(priv, GD32_SPI_STATC_OFFSET,
             SPI_STATC_TXURERRC | SPI_STATC_RXORERRC |
             SPI_STATC_CRCERRC | SPI_STATC_FERRC |
             SPI_STATC_CONFERRC | SPI_STATC_SPDC | SPI_STATC_ETC);
}

#if !defined(CONFIG_GD32H7_SPI_DMA)
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#endif
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  const uint8_t  *btxptr = (const uint8_t *)txbuffer;
  uint8_t        *brxptr = (uint8_t *)rxbuffer;
  const uint16_t *wtxptr = (const uint16_t *)txbuffer;
  uint16_t       *wrxptr = (uint16_t *)rxbuffer;
  uint32_t word;
  uint32_t stat;

  DEBUGASSERT(priv != NULL);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n",
          txbuffer, rxbuffer, nwords);

  if (nwords == 0)
    {
      return;
    }

  /* Check for errors and flush FIFO if needed */

  stat = spi_getreg(priv, GD32_SPI_STAT_OFFSET);
  if (stat & (SPI_STAT_RXORERR | SPI_STAT_TXURERR | SPI_STAT_FERR |
              SPI_STAT_CONFERR | SPI_STAT_CRCERR | SPI_STAT_RP))
    {
      spi_flush_fifo(priv);
    }

  /* Configure TXSIZE and start master transfer.
   * TXSIZE must be set before MSTART, and SPI must be disabled to modify.
   */

  spi_enable(priv, false);

  /* Set TXSIZE for the number of words to transfer */

  spi_putreg(priv, GD32_SPI_CTL1_OFFSET, SPI_CTL1_TXSIZE(nwords));

  spi_enable(priv, true);

  /* Start master transfer */

  spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_MSTART);

  while (nwords-- > 0)
    {
      if (spi_wait_flag(priv, SPI_STAT_TP) != OK)
        {
          spierr("TP timeout!\n");
          return;
        }

      /* Send data - 16-bit or 8-bit mode */

      if (priv->nbits > 8)
        {
          spi_putreg(priv, GD32_SPI_TDATA_OFFSET,
                     wtxptr ? (uint32_t)*wtxptr++ : 0xffff);
        }
      else
        {
          spi_putreg8(priv, GD32_SPI_TDATA_OFFSET,
                      btxptr ? *btxptr++ : 0xff);
        }

      if (spi_wait_flag(priv, SPI_STAT_RP) != OK)
        {
          spierr("RP timeout!\n");
          return;
        }

      /* Receive data - 16-bit or 8-bit mode */

      if (priv->nbits > 8)
        {
          word = spi_getreg(priv, GD32_SPI_RDATA_OFFSET) & 0xffff;
          if (wrxptr)
            {
              *wrxptr++ = (uint16_t)word;
            }
        }
      else
        {
          word = (uint32_t)spi_getreg8(priv, GD32_SPI_RDATA_OFFSET);
          if (brxptr)
            {
              *brxptr++ = (uint8_t)word;
            }
        }
    }
}

/****************************************************************************
 * Name: spi_exchange (DMA version)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA if available and appropriate.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;
  size_t nbytes;
  size_t remaining_words;
  size_t chunk_words;
  size_t chunk_bytes;
  size_t offset_bytes;
  const uint8_t *tx_ptr;
  uint8_t *rx_ptr;
  int ret;
  static uint16_t rxdummy = 0xffff;  /* Must be in RAM for DMA */
  static uint16_t txdummy = 0xffff;  /* Must be in RAM for DMA */

  DEBUGASSERT(priv && priv->spibase);

  /* Convert the number of word to a number of bytes */

  nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;

  if ((priv->rxdma == NULL) || (priv->txdma == NULL) ||
      (priv->rxreqid == 0) || up_interrupt_context() ||
      (priv->txbuf == NULL) || (priv->rxbuf == NULL) ||
      (nbytes <= CONFIG_GD32H7_SPI_DMATHRESHOLD))
    {
      /* Invalid DMA channels, or interrupt context, or no aligned buffers,
       * fall back to non-DMA method.
       */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d (DMA)\n",
          txbuffer, rxbuffer, nwords);

  /* Process data using DMA - may require multiple chunks if data
   * exceeds internal buffer size.
   */

  DEBUGASSERT(priv->txbuf != NULL && priv->rxbuf != NULL);

  remaining_words = nwords;
  offset_bytes = 0;
  tx_ptr = (const uint8_t *)txbuffer;
  rx_ptr = (uint8_t *)rxbuffer;
  ret = 0;

  while (remaining_words > 0 && ret >= 0)
    {
      /* Calculate chunk size (in words) that fits in buffer */

      chunk_bytes = (priv->nbits > 8) ?
                    (remaining_words << 1) : remaining_words;

      if (chunk_bytes > priv->buflen)
        {
          chunk_bytes = priv->buflen;
        }

      chunk_words = (priv->nbits > 8) ?
                    (chunk_bytes >> 1) : chunk_bytes;

      /* Copy chunk to internal aligned buffer */

      if (tx_ptr)
        {
          memcpy(priv->txbuf, tx_ptr + offset_bytes, chunk_bytes);
        }
      else
        {
          /* Fill with dummy data for receive-only transfers */

          memset(priv->txbuf, 0xff, chunk_bytes);
        }

      spi_enable(priv, false);

      /* Set TXSIZE - must be set before MSTART */

      spi_putreg(priv, GD32_SPI_CTL1_OFFSET, SPI_CTL1_TXSIZE(chunk_words));

#if defined(CONFIG_GD32H7_SPI3) || defined(CONFIG_GD32H7_SPI4)
      if (priv->spibase == GD32_SPI3_BASE ||
          priv->spibase == GD32_SPI4_BASE)
        {
          spi_putreg8(priv, GD32_SPI_TDATA_OFFSET, 0x5a);
        }
#endif

      spi_enable(priv, true);

      /* Handle cache coherency for DMA transfer */

#ifdef CONFIG_ARMV7M_DCACHE
      up_clean_dcache((uintptr_t)priv->txbuf,
                      (uintptr_t)priv->txbuf + priv->buflen);
#endif

      /* Configure DMA channels */

      spi_dmarxsetup(priv, priv->rxbuf, &rxdummy, chunk_words);
      spi_dmatxsetup(priv, priv->txbuf, &txdummy, chunk_words);

#ifdef CONFIG_SPI_TRIGGER
      /* Is deferred triggering in effect? */

      if (!priv->defertrig)
        {
#endif

          spi_dmarxstart(priv);
          spi_dmatxstart(priv);

          /* Enable SPI DMA requests - enable RX first, then TX */

          spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET, 0, SPI_CFG0_DMAREN);
          spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET, 0, SPI_CFG0_DMATEN);

          /* Start master transfer if not already started */

          if ((spi_getreg(priv, GD32_SPI_CTL0_OFFSET) &
               SPI_CTL0_MSTART) == 0)
            {
              spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_MSTART);
            }
#ifdef CONFIG_SPI_TRIGGER
        }
      else
        {
          /* Yes.. indicated that we are ready to be started */

          priv->trigarmed = true;
          break;  /* Exit loop for deferred trigger */
        }
#endif

      /* Wait for DMA completion */

      ret = spi_dmatxwait(priv);
      if (ret >= 0)
        {
          ret = spi_dmarxwait(priv);
        }

      /* Disable SPI DMA requests */

      spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET,
                    SPI_CFG0_DMAREN | SPI_CFG0_DMATEN, 0);

      /* Stop DMA channels */

      gd32_dma_stop(priv->txdma);
      gd32_dma_stop(priv->rxdma);

      /* Copy received data from internal aligned buffer to user buffer */

      if (rx_ptr && ret >= 0)
        {
          /* Invalidate RX buffer cache after DMA to read fresh data */

#ifdef CONFIG_ARMV7M_DCACHE
          up_invalidate_dcache((uintptr_t)priv->rxbuf,
                               (uintptr_t)priv->rxbuf + priv->buflen);
#endif

          memcpy(rx_ptr + offset_bytes, priv->rxbuf, chunk_bytes);
        }

      /* Update pointers and remaining count */

      offset_bytes += chunk_bytes;
      remaining_words -= chunk_words;
    }

  /* Check for DMA errors */

  if (ret >= 0)
    {
      if ((priv->rxresult & DMA_INTF_ERRIF) ||
          (priv->txresult & DMA_INTF_ERRIF))
        {
          spierr("DMA transfer error detected\n");
        }
    }

#ifdef CONFIG_SPI_TRIGGER
  if (!priv->defertrig)
    {
      priv->trigarmed = false;
    }
#endif
}
#endif /* CONFIG_GD32H7_SPI_DMA */

/****************************************************************************
 * Name: spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   ENOTSUP  - Trigger not fired due to lack of DMA support
 *   EIO      - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev)
{
#ifdef CONFIG_GD32H7_SPI_DMA
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)dev;

  if (!priv->trigarmed)
    {
      return -EIO;
    }

  spi_dmarxstart(priv);

  /* Enable SPI DMA requests */

  spi_modifyreg(priv, GD32_SPI_CFG0_OFFSET, 0,
                SPI_CFG0_DMAREN | SPI_CFG0_DMATEN);

  /* Start master transfer if not already started */

  if ((spi_getreg(priv, GD32_SPI_CTL0_OFFSET) & SPI_CTL0_MSTART) == 0)
    {
      spi_modifyreg(priv, GD32_SPI_CTL0_OFFSET, 0, SPI_CTL0_MSTART);
    }

  spi_dmatxstart(priv);

  return OK;
#else
  return -ENOSYS;
#endif
}
#endif

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                         size_t nwords)
{
  spi_exchange(dev, txbuffer, NULL, nwords);
}

static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                          size_t nwords)
{
  spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for RX DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI_DMA
static int spi_dmarxwait(struct gd32_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (block until DMA completes). */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret == -EINTR);

  return ret;
}

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for TX DMA to complete.
 *
 ****************************************************************************/

static int spi_dmatxwait(struct gd32_spidev_s *priv)
{
  int ret;

  /* Take the semaphore (block until DMA completes). */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret == -EINTR);

  return ret;
}

/****************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes.
 *
 ****************************************************************************/

static void spi_dmarxcallback(DMA_HANDLE handle, uint16_t isr, void *arg)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)arg;

  /* Save the result of the RX DMA, mask out FEEIF which is
   * expected in Direct mode and does not indicate a real error.
   */

  priv->rxresult = isr & ~DMA_INTF_FEEIF;

  /* Wake up the waiting thread */

  nxsem_post(&priv->rxsem);
}

/****************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the TX DMA completes.
 *
 ****************************************************************************/

static void spi_dmatxcallback(DMA_HANDLE handle, uint16_t isr, void *arg)
{
  struct gd32_spidev_s *priv = (struct gd32_spidev_s *)arg;

  /* Save the result of the TX DMA, mask out FEEIF which is
   * expected in Direct mode and does not indicate a real error.
   */

  priv->txresult = isr & ~DMA_INTF_FEEIF;

  /* Wake up the waiting thread */

  nxsem_post(&priv->txsem);
}

/****************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA.
 *
 ****************************************************************************/

static void spi_dmarxsetup(struct gd32_spidev_s *priv, void *rxbuffer,
                           void *rxdummy, size_t nwords)
{
  struct gd32_dma_config_s config;

  /* Configure the RX DMA */

  config.periph_addr = priv->spibase + GD32_SPI_RDATA_OFFSET;
  config.direction   = DMA_PERIPH_TO_MEMORY;
  config.number      = nwords;
  config.periph_inc  = false;
  config.priority    = DMA_PRIORITY_HIGH;
  config.circular    = false;

  /* Set data width based on SPI configuration */

  if (priv->nbits > 8)
    {
      config.periph_width = DMA_PERIPH_WIDTH_16BIT;
      config.memory_width = DMA_MEMORY_WIDTH_16BIT;
    }
  else
    {
      config.periph_width = DMA_PERIPH_WIDTH_8BIT;
      config.memory_width = DMA_MEMORY_WIDTH_8BIT;
    }

  /* Configure memory address and increment */

  if (rxbuffer)
    {
      config.memory_addr = (uint32_t)rxbuffer;
      config.memory_inc  = true;
    }
  else
    {
      config.memory_addr = (uint32_t)rxdummy;
      config.memory_inc  = false;
    }

  gd32_dma_setup(priv->rxdma, &config);
}

/****************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA.
 *
 ****************************************************************************/

static void spi_dmatxsetup(struct gd32_spidev_s *priv,
                           const void *txbuffer, const void *txdummy,
                           size_t nwords)
{
  struct gd32_dma_config_s config;

  /* Configure the TX DMA */

  config.periph_addr = priv->spibase + GD32_SPI_TDATA_OFFSET;
  config.direction   = DMA_MEMORY_TO_PERIPH;
  config.number      = nwords;
  config.periph_inc  = false;
  config.priority    = DMA_PRIORITY_HIGH;
  config.circular    = false;

  /* Set data width based on SPI configuration */

  if (priv->nbits > 8)
    {
      config.periph_width = DMA_PERIPH_WIDTH_16BIT;
      config.memory_width = DMA_MEMORY_WIDTH_16BIT;
    }
  else
    {
      config.periph_width = DMA_PERIPH_WIDTH_8BIT;
      config.memory_width = DMA_MEMORY_WIDTH_8BIT;
    }

  /* Configure memory address and increment */

  if (txbuffer)
    {
      config.memory_addr = (uint32_t)txbuffer;
      config.memory_inc  = true;
    }
  else
    {
      config.memory_addr = (uint32_t)txdummy;
      config.memory_inc  = false;
    }

  gd32_dma_setup(priv->txdma, &config);
}

/****************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA.
 *
 ****************************************************************************/

static inline void spi_dmarxstart(struct gd32_spidev_s *priv)
{
  priv->rxresult = 0;
  gd32_dma_start(priv->rxdma, spi_dmarxcallback, priv, DMA_CHXCTL_FTFIE);
}

/****************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA.
 *
 ****************************************************************************/

static inline void spi_dmatxstart(struct gd32_spidev_s *priv)
{
  priv->txresult = 0;
  gd32_dma_start(priv->txdma, spi_dmatxcallback, priv, DMA_CHXCTL_FTFIE);
}
#endif /* CONFIG_GD32H7_SPI_DMA */

/****************************************************************************
 * Name: spi_clock_enable / spi_reset / spi_gpio_config
 *
 * Description:
 *   RCU clock enable, reset, and GPIO configuration for each SPI port.
 *   H7xx has 6 SPI controllers: SPI0/3/4/5 on APB2, SPI1/2 on APB1.
 *
 ****************************************************************************/

static void spi_clock_enable(uint32_t spibase)
{
  uint32_t regaddr;
  uint32_t setbits;

  switch (spibase)
    {
#ifdef CONFIG_GD32H7_SPI0
      case GD32_SPI0_BASE:
        regaddr = GD32_RCU_APB2EN;
        setbits = RCU_APB2EN_SPI0EN;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI1
      case GD32_SPI1_BASE:
        regaddr = GD32_RCU_APB1EN;
        setbits = RCU_APB1EN_SPI1EN;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI2
      case GD32_SPI2_BASE:
        regaddr = GD32_RCU_APB1EN;
        setbits = RCU_APB1EN_SPI2EN;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI3
      case GD32_SPI3_BASE:
        regaddr = GD32_RCU_APB2EN;
        setbits = RCU_APB2EN_SPI3EN;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI4
      case GD32_SPI4_BASE:
        regaddr = GD32_RCU_APB2EN;
        setbits = RCU_APB2EN_SPI4EN;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI5
      case GD32_SPI5_BASE:
        regaddr = GD32_RCU_APB2EN;
        setbits = RCU_APB2EN_SPI5EN;
        break;
#endif
      default:
        return;
    }

  modifyreg32(regaddr, 0, setbits);
}

static void spi_reset(uint32_t spibase)
{
  uint32_t regaddr;
  uint32_t setbits;

  switch (spibase)
    {
#ifdef CONFIG_GD32H7_SPI0
      case GD32_SPI0_BASE:
        regaddr = GD32_RCU_APB2RST;
        setbits = RCU_APB2RST_SPI0RST;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI1
      case GD32_SPI1_BASE:
        regaddr = GD32_RCU_APB1RST;
        setbits = RCU_APB1RST_SPI1RST;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI2
      case GD32_SPI2_BASE:
        regaddr = GD32_RCU_APB1RST;
        setbits = RCU_APB1RST_SPI2RST;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI3
      case GD32_SPI3_BASE:
        regaddr = GD32_RCU_APB2RST;
        setbits = RCU_APB2RST_SPI3RST;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI4
      case GD32_SPI4_BASE:
        regaddr = GD32_RCU_APB2RST;
        setbits = RCU_APB2RST_SPI4RST;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI5
      case GD32_SPI5_BASE:
        regaddr = GD32_RCU_APB2RST;
        setbits = RCU_APB2RST_SPI5RST;
        break;
#endif
      default:
        return;
    }

  modifyreg32(regaddr, 0, setbits);
  modifyreg32(regaddr, setbits, 0);
}

static void spi_gpio_config(uint32_t spibase)
{
  switch (spibase)
    {
#ifdef CONFIG_GD32H7_SPI0
      case GD32_SPI0_BASE:
#  ifdef GPIO_SPI0_SCK_PIN
        gd32_gpio_config(GPIO_SPI0_SCK_PIN);
#  endif
#  ifdef GPIO_SPI0_MISO_PIN
        gd32_gpio_config(GPIO_SPI0_MISO_PIN);
#  endif
#  ifdef GPIO_SPI0_MOSI_PIN
        gd32_gpio_config(GPIO_SPI0_MOSI_PIN);
#  endif
        break;
#endif
#ifdef CONFIG_GD32H7_SPI1
      case GD32_SPI1_BASE:
#  ifdef GPIO_SPI1_SCK_PIN
        gd32_gpio_config(GPIO_SPI1_SCK_PIN);
#  endif
#  ifdef GPIO_SPI1_MISO_PIN
        gd32_gpio_config(GPIO_SPI1_MISO_PIN);
#  endif
#  ifdef GPIO_SPI1_MOSI_PIN
        gd32_gpio_config(GPIO_SPI1_MOSI_PIN);
#  endif
        break;
#endif
#ifdef CONFIG_GD32H7_SPI2
      case GD32_SPI2_BASE:
#  ifdef GPIO_SPI2_SCK_PIN
        gd32_gpio_config(GPIO_SPI2_SCK_PIN);
#  endif
#  ifdef GPIO_SPI2_MISO_PIN
        gd32_gpio_config(GPIO_SPI2_MISO_PIN);
#  endif
#  ifdef GPIO_SPI2_MOSI_PIN
        gd32_gpio_config(GPIO_SPI2_MOSI_PIN);
#  endif
        break;
#endif
#ifdef CONFIG_GD32H7_SPI3
      case GD32_SPI3_BASE:
#  ifdef GPIO_SPI3_SCK_PIN
        gd32_gpio_config(GPIO_SPI3_SCK_PIN);
#  endif
#  ifdef GPIO_SPI3_MISO_PIN
        gd32_gpio_config(GPIO_SPI3_MISO_PIN);
#  endif
#  ifdef GPIO_SPI3_MOSI_PIN
        gd32_gpio_config(GPIO_SPI3_MOSI_PIN);
#  endif
        break;
#endif
#ifdef CONFIG_GD32H7_SPI4
      case GD32_SPI4_BASE:
#  ifdef GPIO_SPI4_SCK_PIN
        gd32_gpio_config(GPIO_SPI4_SCK_PIN);
#  endif
#  ifdef GPIO_SPI4_MISO_PIN
        gd32_gpio_config(GPIO_SPI4_MISO_PIN);
#  endif
#  ifdef GPIO_SPI4_MOSI_PIN
        gd32_gpio_config(GPIO_SPI4_MOSI_PIN);
#  endif
        break;
#endif
#ifdef CONFIG_GD32H7_SPI5
      case GD32_SPI5_BASE:
#  ifdef GPIO_SPI5_SCK_PIN
        gd32_gpio_config(GPIO_SPI5_SCK_PIN);
#  endif
#  ifdef GPIO_SPI5_MISO_PIN
        gd32_gpio_config(GPIO_SPI5_MISO_PIN);
#  endif
#  ifdef GPIO_SPI5_MOSI_PIN
        gd32_gpio_config(GPIO_SPI5_MOSI_PIN);
#  endif
        break;
#endif
      default:
        break;
    }
}

/****************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the SPI bus per user manual Sec 27.3.7 sequence.
 *
 ****************************************************************************/

static void spi_bus_initialize(struct gd32_spidev_s *priv)
{
  uint32_t regval;

  /* Enable the SPI clock */

  spi_clock_enable(priv->spibase);

  /* Reset SPI peripheral */

  spi_reset(priv->spibase);

  /* Configure SPI GPIO pins */

  spi_gpio_config(priv->spibase);

  /* Ensure SPI is disabled during configuration */

  spi_enable(priv, false);

  spi_putreg(priv, GD32_SPI_CTL1_OFFSET, 0);

  /* CFG0: 8-bit data, FIFO threshold=1, prescaler=/8, byte access mode */

  regval = SPI_CFG0_DZ_8BIT |
           SPI_CFG0_FIFOLVL_1 |
           SPI_CFG0_PSC_8 |
           SPI_CFG0_BYTEN;
  spi_putreg(priv, GD32_SPI_CFG0_OFFSET, regval);

  /* CFG1: Master mode, software NSS, CKPL=0/CKPH=0 (MODE0), MSB first,
   * full duplex.
   * Set NSSDRV even with software NSS - this enables
   * the NSS output driver which may be required for clock generation.
   * Add MSSD delay for NSS-to-data timing.
   */

  regval = SPI_CFG1_MSTMOD |
           SPI_CFG1_NSSIM |
           SPI_CFG1_NSSDRV |
           SPI_CFG1_MSSD(1);  /* 1 SPI clock delay from NSS to data */
  spi_putreg(priv, GD32_SPI_CFG1_OFFSET, regval);

  /* Note: Do NOT set NSSI in CTL0 */

  /* Note: RXDLYCK is left at reset default (0).
   * This driver does not configure this register.
   */

  /* Clear QCTL to ensure standard SPI mode */

  spi_putreg(priv, GD32_SPI_QCTL_OFFSET, 0);

  /* Initialize state */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Set default frequency (~400KHz) */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Clear errors and status */

  spi_clear_errors(priv);
  spi_putreg(priv, GD32_SPI_STATC_OFFSET,
             SPI_STATC_SPDC | SPI_STATC_ETC);

#ifdef CONFIG_GD32H7_SPI_DMA
  /* Allocate DMA channels if configured */

  if (priv->rxreqid != 0)
    {
      priv->rxdma = gd32_dma_channel_alloc(priv->rxreqid);
      priv->txdma = gd32_dma_channel_alloc(priv->txreqid);

      if (priv->rxdma == NULL || priv->txdma == NULL)
        {
          spierr("ERROR: Failed to allocate DMA channels\n");

          /* Free any allocated channel */

          if (priv->rxdma != NULL)
            {
              gd32_dma_channel_free(priv->rxdma);
              priv->rxdma = NULL;
            }

          if (priv->txdma != NULL)
            {
              gd32_dma_channel_free(priv->txdma);
              priv->txdma = NULL;
            }
        }
      else
        {
          spiinfo("DMA channels allocated: rx=%p tx=%p\n",
                  priv->rxdma, priv->txdma);
        }
    }
#endif

  /* Enable SPI */

  spi_enable(priv, true);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spibus_initialize
 ****************************************************************************/

struct spi_dev_s *gd32_spibus_initialize(int bus)
{
  struct gd32_spidev_s *priv = NULL;
  irqstate_t flags;

  flags = enter_critical_section();

  switch (bus)
    {
#ifdef CONFIG_GD32H7_SPI0
      case 0:
        priv = &g_spi0dev;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI1
      case 1:
        priv = &g_spi1dev;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI2
      case 2:
        priv = &g_spi2dev;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI3
      case 3:
        priv = &g_spi3dev;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI4
      case 4:
        priv = &g_spi4dev;
        break;
#endif
#ifdef CONFIG_GD32H7_SPI5
      case 5:
        priv = &g_spi5dev;
        break;
#endif
      default:
        leave_critical_section(flags);
        return NULL;
    }

  if (!priv->initialized)
    {
      spi_bus_initialize(priv);
      priv->initialized = true;
    }

  leave_critical_section(flags);

  return &priv->spidev;
}

/****************************************************************************
 * Name: gd32_spi_register
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
int gd32_spi_register(int bus)
{
  struct spi_dev_s *spi;

  spi = gd32_spibus_initialize(bus);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  return spi_register(spi, bus);
}
#endif

#endif /* CONFIG_GD32H7_SPI0 || ... || CONFIG_GD32H7_SPI5 */
