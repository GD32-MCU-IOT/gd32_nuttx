/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_i2c.c
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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32h7xx.h"
#include "gd32h7xx_i2c.h"
#include "gd32h7xx_rcu.h"
#include "gd32h7xx_gpio.h"

#ifdef CONFIG_GD32H7_I2C_DMA
#  include "gd32h7xx_dma.h"
#endif

#include <nuttx/cache.h>

#ifdef CONFIG_GD32H7_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED if not using interrupt-driven I2C */

#ifndef CONFIG_GD32H7_I2C_TIMEOSEC
#  define CONFIG_GD32H7_I2C_TIMEOSEC 0
#endif

#ifndef CONFIG_GD32H7_I2C_TIMEOMS
#  define CONFIG_GD32H7_I2C_TIMEOMS  500
#endif

#if CONFIG_GD32H7_I2C_TIMEOSEC == 0 && CONFIG_GD32H7_I2C_TIMEOMS == 0
#  error "I2C timeout must be non-zero"
#endif

#define I2C_TIMEOUT_TICKS \
  MSEC2TICK(CONFIG_GD32H7_I2C_TIMEOSEC * 1000 + CONFIG_GD32H7_I2C_TIMEOMS)

#ifdef CONFIG_GD32H7_I2C_DMA
#  ifndef CONFIG_GD32H7_I2C_DMATHRESHOLD
#    define CONFIG_GD32H7_I2C_DMATHRESHOLD 4
#  endif
#  ifdef CONFIG_ARMV7M_DCACHE
#    ifndef ARMV7M_DCACHE_LINESIZE
#      define ARMV7M_DCACHE_LINESIZE 32
#    endif
#    define I2C_DMA_ALIGN_MASK   (ARMV7M_DCACHE_LINESIZE - 1)
#    define I2C_DMA_BUFSIZE      ((I2C_MAX_RELOAD_SIZE + I2C_DMA_ALIGN_MASK) \
                                  & ~I2C_DMA_ALIGN_MASK)
#    define I2C_DMA_BUF_ALIGN    aligned_data(ARMV7M_DCACHE_LINESIZE)
#    define I2C_IS_ALIGNED(a)    (((uintptr_t)(a) & I2C_DMA_ALIGN_MASK) == 0)
#  endif
#endif

/* I2C interrupt state machine */

#define INTSTATE_IDLE     0  /* Not waiting for interrupt */
#define INTSTATE_WAITING  1  /* Waiting for interrupt */
#define INTSTATE_DONE     2  /* Interrupt activity complete */

/* I2C transfer direction */

#define I2C_DIR_WRITE     0
#define I2C_DIR_READ      1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C hardware configuration */

struct gd32_i2c_config_s
{
  uint32_t i2c_base;          /* I2C register base address */
  uint32_t scl_pin;           /* GPIO configuration for SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA */
  uint32_t clk_bit;           /* RCU APB1EN clock enable bit */
  uint32_t rst_bit;           /* RCU APB1RST reset bit */
  uint32_t clk_cfg_reg;       /* RCU register for I2C clock source */
  uint32_t clk_cfg_shift;     /* Bit shift in clock source register */
  uint32_t clk_cfg_mask;      /* Bit mask in clock source register */
#ifndef CONFIG_I2C_POLLED
  uint32_t ev_irq;            /* Event interrupt number */
  uint32_t er_irq;            /* Error interrupt number */
#endif
#ifdef CONFIG_GD32H7_I2C_DMA
  uint8_t  rxdma_reqid;       /* DMAMUX request ID for RX */
  uint8_t  txdma_reqid;       /* DMAMUX request ID for TX */
#endif
};

/* I2C device private data */

struct gd32_i2c_priv_s
{
  const struct i2c_ops_s     *ops;
  const struct gd32_i2c_config_s *config;
  int                         refs;       /* Reference count */
  mutex_t                     lock;       /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t                       sem_isr;    /* ISR wait semaphore */
#endif
  volatile uint8_t            intstate;   /* Interrupt handshake state */
  uint8_t                     msgc;       /* Message count */
  struct i2c_msg_s           *msgv;       /* Message list */
  uint8_t                    *ptr;        /* Current buffer pointer */
  uint32_t                    frequency;  /* Current I2C frequency */
  volatile int                dcnt;       /* Current message data count */
  uint16_t                    flags;      /* Current message flags */
  uint32_t                    status;     /* End of transfer STAT */
#ifdef CONFIG_GD32H7_I2C_DMA
  DMA_HANDLE                  txdma;      /* TX DMA handle */
  DMA_HANDLE                  rxdma;      /* RX DMA handle */
  sem_t                       txsem;      /* TX DMA completion sem */
  sem_t                       rxsem;      /* RX DMA completion sem */
  volatile uint16_t           txresult;   /* TX DMA result */
  volatile uint16_t           rxresult;   /* RX DMA result */
#ifdef CONFIG_ARMV7M_DCACHE
  uint8_t                    *rxbuf;      /* Cache-aligned RX bounce buf */
#endif
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level I2C register helpers */

static inline uint32_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                        uint32_t offset);
static inline void gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                   uint32_t offset, uint32_t value);
static inline void gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                      uint32_t offset,
                                      uint32_t clearbits,
                                      uint32_t setbits);

/* I2C transfer helpers */

static int  gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv);
static void gd32_i2c_set_timing(struct gd32_i2c_priv_s *priv,
                                uint32_t frequency);
#ifdef CONFIG_I2C_POLLED
static int  gd32_i2c_polling_transfer(struct gd32_i2c_priv_s *priv,
                                      struct i2c_msg_s *msgs, int count);
#endif
#ifdef CONFIG_GD32H7_I2C_DMA
static void gd32_i2c_dmatxcallback(DMA_HANDLE handle, uint16_t isr,
                                   void *arg);
static void gd32_i2c_dmarxcallback(DMA_HANDLE handle, uint16_t isr,
                                   void *arg);
static int  gd32_i2c_dma_write(struct gd32_i2c_priv_s *priv,
                               struct i2c_msg_s *msg);
static int  gd32_i2c_dma_read(struct gd32_i2c_priv_s *priv,
                              struct i2c_msg_s *msg);
static int  gd32_i2c_dma_writeread(struct gd32_i2c_priv_s *priv,
                                   struct i2c_msg_s *wrmsg,
                                   struct i2c_msg_s *rdmsg);
#endif

/* I2C interrupt handler */

#ifndef CONFIG_I2C_POLLED
static int  gd32_i2c_isr(int irq, void *context, void *arg);
#endif

/* I2C ops interface */

static int  gd32_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  gd32_i2c_reset(struct i2c_master_s *dev);
#endif

/* I2C init/deinit */

static void gd32_i2c_init(struct gd32_i2c_priv_s *priv);
static void gd32_i2c_deinit(struct gd32_i2c_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s gd32_i2c_ops =
{
  .transfer = gd32_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset    = gd32_i2c_reset
#endif
};

/* DMA RX bounce buffers - cache-line aligned for safe D-Cache invalidation
 * when user buffer is not cache-line aligned.
 */

#if defined(CONFIG_GD32H7_I2C_DMA) && defined(CONFIG_ARMV7M_DCACHE)
#  ifdef CONFIG_GD32H7_I2C0_DMA
static uint8_t g_i2c0_rxbuf[I2C_DMA_BUFSIZE] I2C_DMA_BUF_ALIGN;
#  endif
#  ifdef CONFIG_GD32H7_I2C1_DMA
static uint8_t g_i2c1_rxbuf[I2C_DMA_BUFSIZE] I2C_DMA_BUF_ALIGN;
#  endif
#  ifdef CONFIG_GD32H7_I2C2_DMA
static uint8_t g_i2c2_rxbuf[I2C_DMA_BUFSIZE] I2C_DMA_BUF_ALIGN;
#  endif
#  ifdef CONFIG_GD32H7_I2C3_DMA
static uint8_t g_i2c3_rxbuf[I2C_DMA_BUFSIZE] I2C_DMA_BUF_ALIGN;
#  endif
#endif

/* I2C0 */

#ifdef CONFIG_GD32H7_I2C0
static const struct gd32_i2c_config_s gd32_i2c0_config =
{
  .i2c_base    = GD32_I2C0_BASE,
  .scl_pin     = GPIO_I2C0_SCL,
  .sda_pin     = GPIO_I2C0_SDA,
  .clk_bit     = RCU_APB1EN_I2C0EN,
  .rst_bit     = RCU_APB1RST_I2C0RST,
  .clk_cfg_reg   = GD32_RCU_CFG0,
  .clk_cfg_shift = RCU_CFG0_I2C0SEL_SHIFT,
  .clk_cfg_mask  = RCU_CFG0_I2C0SEL_MASK,
#ifndef CONFIG_I2C_POLLED
  .ev_irq      = GD32_IRQ_I2C0_EV,
  .er_irq      = GD32_IRQ_I2C0_ER,
#endif
#ifdef CONFIG_GD32H7_I2C0_DMA
  .rxdma_reqid = DMA_REQUEST_I2C0_RX,
  .txdma_reqid = DMA_REQUEST_I2C0_TX,
#endif
};

static struct gd32_i2c_priv_s gd32_i2c0_priv =
{
  .ops      = &gd32_i2c_ops,
  .config   = &gd32_i2c0_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr  = SEM_INITIALIZER(0),
#endif
  .intstate = INTSTATE_IDLE,
#ifdef CONFIG_GD32H7_I2C0_DMA
  .txsem    = SEM_INITIALIZER(0),
  .rxsem    = SEM_INITIALIZER(0),
#ifdef CONFIG_ARMV7M_DCACHE
  .rxbuf    = g_i2c0_rxbuf,
#endif
#endif
};
#endif

/* I2C1 */

#ifdef CONFIG_GD32H7_I2C1
static const struct gd32_i2c_config_s gd32_i2c1_config =
{
  .i2c_base    = GD32_I2C1_BASE,
  .scl_pin     = GPIO_I2C1_SCL,
  .sda_pin     = GPIO_I2C1_SDA,
  .clk_bit     = RCU_APB1EN_I2C1EN,
  .rst_bit     = RCU_APB1RST_I2C1RST,
  .clk_cfg_reg   = GD32_RCU_CFG3,
  .clk_cfg_shift = RCU_CFG3_I2C1SEL_SHIFT,
  .clk_cfg_mask  = RCU_CFG3_I2C1SEL_MASK,
#ifndef CONFIG_I2C_POLLED
  .ev_irq      = GD32_IRQ_I2C1_EV,
  .er_irq      = GD32_IRQ_I2C1_ER,
#endif
#ifdef CONFIG_GD32H7_I2C1_DMA
  .rxdma_reqid = DMA_REQUEST_I2C1_RX,
  .txdma_reqid = DMA_REQUEST_I2C1_TX,
#endif
};

static struct gd32_i2c_priv_s gd32_i2c1_priv =
{
  .ops      = &gd32_i2c_ops,
  .config   = &gd32_i2c1_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr  = SEM_INITIALIZER(0),
#endif
  .intstate = INTSTATE_IDLE,
#ifdef CONFIG_GD32H7_I2C1_DMA
  .txsem    = SEM_INITIALIZER(0),
  .rxsem    = SEM_INITIALIZER(0),
#ifdef CONFIG_ARMV7M_DCACHE
  .rxbuf    = g_i2c1_rxbuf,
#endif
#endif
};
#endif

/* I2C2 */

#ifdef CONFIG_GD32H7_I2C2
static const struct gd32_i2c_config_s gd32_i2c2_config =
{
  .i2c_base    = GD32_I2C2_BASE,
  .scl_pin     = GPIO_I2C2_SCL,
  .sda_pin     = GPIO_I2C2_SDA,
  .clk_bit     = RCU_APB1EN_I2C2EN,
  .rst_bit     = RCU_APB1RST_I2C2RST,
  .clk_cfg_reg   = GD32_RCU_CFG3,
  .clk_cfg_shift = RCU_CFG3_I2C2SEL_SHIFT,
  .clk_cfg_mask  = RCU_CFG3_I2C2SEL_MASK,
#ifndef CONFIG_I2C_POLLED
  .ev_irq      = GD32_IRQ_I2C2_EV,
  .er_irq      = GD32_IRQ_I2C2_ER,
#endif
#ifdef CONFIG_GD32H7_I2C2_DMA
  .rxdma_reqid = DMA_REQUEST_I2C2_RX,
  .txdma_reqid = DMA_REQUEST_I2C2_TX,
#endif
};

static struct gd32_i2c_priv_s gd32_i2c2_priv =
{
  .ops      = &gd32_i2c_ops,
  .config   = &gd32_i2c2_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr  = SEM_INITIALIZER(0),
#endif
  .intstate = INTSTATE_IDLE,
#ifdef CONFIG_GD32H7_I2C2_DMA
  .txsem    = SEM_INITIALIZER(0),
  .rxsem    = SEM_INITIALIZER(0),
#ifdef CONFIG_ARMV7M_DCACHE
  .rxbuf    = g_i2c2_rxbuf,
#endif
#endif
};
#endif

/* I2C3 */

#ifdef CONFIG_GD32H7_I2C3
static const struct gd32_i2c_config_s gd32_i2c3_config =
{
  .i2c_base    = GD32_I2C3_BASE,
  .scl_pin     = GPIO_I2C3_SCL,
  .sda_pin     = GPIO_I2C3_SDA,
  .clk_bit     = RCU_APB1EN_I2C3EN,
  .rst_bit     = RCU_APB1RST_I2C3RST,
  .clk_cfg_reg   = GD32_RCU_CFG3,
  .clk_cfg_shift = RCU_CFG3_I2C3SEL_SHIFT,
  .clk_cfg_mask  = RCU_CFG3_I2C3SEL_MASK,
#ifndef CONFIG_I2C_POLLED
  .ev_irq      = GD32_IRQ_I2C3_EV,
  .er_irq      = GD32_IRQ_I2C3_ER,
#endif
#ifdef CONFIG_GD32H7_I2C3_DMA
  .rxdma_reqid = DMA_REQUEST_I2C3_RX,
  .txdma_reqid = DMA_REQUEST_I2C3_TX,
#endif
};

static struct gd32_i2c_priv_s gd32_i2c3_priv =
{
  .ops      = &gd32_i2c_ops,
  .config   = &gd32_i2c3_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr  = SEM_INITIALIZER(0),
#endif
  .intstate = INTSTATE_IDLE,
#ifdef CONFIG_GD32H7_I2C3_DMA
  .txsem    = SEM_INITIALIZER(0),
  .rxsem    = SEM_INITIALIZER(0),
#ifdef CONFIG_ARMV7M_DCACHE
  .rxbuf    = g_i2c3_rxbuf,
#endif
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2c_getreg / putreg / modifyreg
 ****************************************************************************/

static inline uint32_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->config->i2c_base + offset);
}

static inline void gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                   uint32_t offset, uint32_t value)
{
  putreg32(value, priv->config->i2c_base + offset);
}

static inline void gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                      uint32_t offset,
                                      uint32_t clearbits,
                                      uint32_t setbits)
{
  modifyreg32(priv->config->i2c_base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: gd32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete. In polled mode, spin-wait on status
 *   register. In interrupt mode, wait on semaphore with timeout.
 *
 ****************************************************************************/

static int gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv)
{
#ifdef CONFIG_I2C_POLLED
  clock_t timeout;
  clock_t start;
  uint32_t stat;

  start = clock_systime_ticks();
  timeout = I2C_TIMEOUT_TICKS;

  do
    {
      stat = gd32_i2c_getreg(priv, GD32_I2C_STAT_OFFSET);

      if (stat & I2C_STAT_ERROR_MASK)
        {
          priv->status = stat;
          return -EIO;
        }

      if (priv->intstate == INTSTATE_DONE)
        {
          return OK;
        }
    }
  while (clock_systime_ticks() - start < timeout);

  return -ETIMEDOUT;
#else
  return nxsem_tickwait(&priv->sem_isr, I2C_TIMEOUT_TICKS);
#endif
}

/****************************************************************************
 * Name: gd32_i2c_set_timing
 *
 * Description:
 *   Configure I2C clock timing for the desired frequency.
 *   GD32H7xx uses TIMING register with PSC, SCLL, SCLH, SDADELY, SCLDELY.
 *
 *   Default timing values for IRC64MDIV (64MHz / 2 = 32MHz) source:
 *   100kHz: PSC=0, SCLDELY=6, SDADELY=0, SCLH=0x73, SCLL=0x26
 *   400kHz: PSC=1, SCLDELY=7, SDADELY=0, SCLH=0x87, SCLL=0x2D
 *
 ****************************************************************************/

static void gd32_i2c_set_timing(struct gd32_i2c_priv_s *priv,
                                uint32_t frequency)
{
  uint32_t timing;

  if (priv->frequency == frequency)
    {
      return;
    }

  /* Disable I2C before changing timing */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_I2CEN, 0);

  if (frequency >= 400000)
    {
      /* Fast mode (400kHz) timing from demo:
       * PSC=1, SCLDELY=7, SDADELY=0, SCLH=0x87, SCLL=0x2D
       */

      timing = (1u << I2C_TIMING_PSC_SHIFT) |
               (7u << I2C_TIMING_SCLDELY_SHIFT) |
               (0u << I2C_TIMING_SDADELY_SHIFT) |
               (0x87u << I2C_TIMING_SCLH_SHIFT) |
               (0x2du << I2C_TIMING_SCLL_SHIFT);
    }
  else
    {
      /* Standard mode (100kHz) timing from demo:
       * PSC=0, SCLDELY=6, SDADELY=0, SCLH=0x73, SCLL=0x26
       */

      timing = (0u << I2C_TIMING_PSC_SHIFT) |
               (6u << I2C_TIMING_SCLDELY_SHIFT) |
               (0u << I2C_TIMING_SDADELY_SHIFT) |
               (0x73u << I2C_TIMING_SCLH_SHIFT) |
               (0x26u << I2C_TIMING_SCLL_SHIFT);
    }

  gd32_i2c_putreg(priv, GD32_I2C_TIMING_OFFSET, timing);

  /* Re-enable I2C */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_I2CEN);

  priv->frequency = frequency;
}

/****************************************************************************
 * Name: gd32_i2c_sendstart
 *
 * Description:
 *   Configure CTL1 with slave address, direction, byte count, and
 *   trigger a START condition.
 *
 * Parameters:
 *   priv     - I2C private data
 *   addr     - 7-bit slave address (unshifted)
 *   dir      - I2C_DIR_WRITE (0) or I2C_DIR_READ (1)
 *   nbytes   - Number of bytes (max 255)
 *   reload   - true to use RELOAD mode (for transfers > 255 bytes)
 *   autoend  - true to auto-generate STOP after last byte
 *
 ****************************************************************************/

static void gd32_i2c_sendstart(struct gd32_i2c_priv_s *priv,
                               uint16_t addr, uint8_t dir,
                               uint8_t nbytes, bool reload, bool autoend)
{
  uint32_t ctl1;

  /* Read current CTL1, preserve only non-transfer fields */

  ctl1 = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  ctl1 &= ~(I2C_CTL1_SADDRESS_MASK | I2C_CTL1_TRDIR | I2C_CTL1_ADD10EN |
             I2C_CTL1_BYTENUM_MASK | I2C_CTL1_RELOAD | I2C_CTL1_AUTOEND |
             I2C_CTL1_START | I2C_CTL1_STOP);

  /* Set slave address (7-bit, shifted left by 1) */

  ctl1 |= I2C_CTL1_SADDRESS((uint32_t)addr << 1);

  /* Set direction */

  if (dir == I2C_DIR_READ)
    {
      ctl1 |= I2C_CTL1_TRDIR;
    }

  /* Set byte count */

  ctl1 |= I2C_CTL1_BYTENUM(nbytes);

  /* Set transfer mode */

  if (reload)
    {
      ctl1 |= I2C_CTL1_RELOAD;
    }

  if (autoend)
    {
      ctl1 |= I2C_CTL1_AUTOEND;
    }

  /* Generate START */

  ctl1 |= I2C_CTL1_START;

  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, ctl1);
}

#if defined(CONFIG_I2C_POLLED) || defined(CONFIG_GD32H7_I2C_DMA)
/****************************************************************************
 * Name: gd32_i2c_wait_flag
 *
 * Description:
 *   Poll-wait for a STAT flag to be set, with timeout.
 *
 ****************************************************************************/

static int gd32_i2c_wait_flag(struct gd32_i2c_priv_s *priv,
                              uint32_t flag)
{
  clock_t start;
  clock_t timeout;
  uint32_t stat;

  start = clock_systime_ticks();
  timeout = I2C_TIMEOUT_TICKS;

  while (1)
    {
      stat = gd32_i2c_getreg(priv, GD32_I2C_STAT_OFFSET);

      if (stat & flag)
        {
          return OK;
        }

      if (stat & I2C_STAT_NACK)
        {
          /* Clear NACK flag */

          gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_NACKC);
          return -ENXIO;
        }

      if (stat & I2C_STAT_ERROR_MASK)
        {
          /* Clear all error flags */

          gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);
          return -EIO;
        }

      if (clock_systime_ticks() - start >= timeout)
        {
          return -ETIMEDOUT;
        }
    }
}
#endif /* CONFIG_I2C_POLLED || CONFIG_GD32H7_I2C_DMA */

#ifdef CONFIG_I2C_POLLED
/****************************************************************************
 * Name: gd32_i2c_polling_transfer
 *
 * Description:
 *   Execute I2C messages in polling mode. This is the main polling-mode
 *   transfer function for the GD32H7xx new-style I2C peripheral.
 *
 *   For each message:
 *   - Configure byte count (max 255, use RELOAD for larger)
 *   - Generate START with address and direction
 *   - Transfer data bytes via TI (TX) or RBNE (RX) flags
 *   - Handle RELOAD for messages > 255 bytes
 *   - Last message uses AUTOEND to auto-generate STOP
 *
 ****************************************************************************/

static int gd32_i2c_polling_transfer(struct gd32_i2c_priv_s *priv,
                                     struct i2c_msg_s *msgs, int count)
{
  int ret = OK;
  int i;

  for (i = 0; i < count; i++)
    {
      struct i2c_msg_s *msg = &msgs[i];
      uint8_t *ptr = msg->buffer;
      int remaining = msg->length;
      bool is_read = (msg->flags & I2C_M_READ) != 0;
      bool is_last = (i == count - 1);
      bool restart = (i > 0);

      while (remaining > 0)
        {
          uint8_t chunk;
          bool reload;
          bool autoend;

          /* Calculate chunk size (max 255 per transfer) */

          if (remaining > I2C_MAX_RELOAD_SIZE)
            {
              chunk = I2C_MAX_RELOAD_SIZE;
              reload = true;
              autoend = false;
            }
          else
            {
              chunk = (uint8_t)remaining;
              reload = false;

              /* AUTOEND only on the very last chunk of the last message,
               * unless this message has I2C_M_NOSTOP flag
               */

              autoend = is_last && !(msg->flags & I2C_M_NOSTOP);
            }

          /* For restart (not first msg), the START is a repeated start */

          if (restart || remaining == msg->length)
            {
              gd32_i2c_sendstart(priv, msg->addr,
                                 is_read ? I2C_DIR_READ : I2C_DIR_WRITE,
                                 chunk, reload, autoend);
              restart = false;  /* Only on first chunk */
            }
          else
            {
              /* RELOAD continuation: just update byte count and reload */

              uint32_t ctl1 = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
              ctl1 &= ~(I2C_CTL1_BYTENUM_MASK | I2C_CTL1_RELOAD |
                         I2C_CTL1_AUTOEND);
              ctl1 |= I2C_CTL1_BYTENUM(chunk);
              if (reload)
                {
                  ctl1 |= I2C_CTL1_RELOAD;
                }

              if (autoend)
                {
                  ctl1 |= I2C_CTL1_AUTOEND;
                }

              gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, ctl1);
            }

          /* Transfer data bytes */

          if (is_read)
            {
              int j;
              for (j = 0; j < chunk; j++)
                {
                  ret = gd32_i2c_wait_flag(priv, I2C_STAT_RBNE);
                  if (ret < 0)
                    {
                      goto errout;
                    }

                  *ptr++ = (uint8_t)(gd32_i2c_getreg(priv,
                                     GD32_I2C_RDATA_OFFSET) &
                                     I2C_RDATA_RDATA_MASK);
                }
            }
          else
            {
              int j;
              for (j = 0; j < chunk; j++)
                {
                  ret = gd32_i2c_wait_flag(priv, I2C_STAT_TI);
                  if (ret < 0)
                    {
                      goto errout;
                    }

                  gd32_i2c_putreg(priv, GD32_I2C_TDATA_OFFSET, *ptr++);
                }
            }

          remaining -= chunk;

          /* If reload, wait for TCR before next chunk */

          if (reload)
            {
              ret = gd32_i2c_wait_flag(priv, I2C_STAT_TCR);
              if (ret < 0)
                {
                  goto errout;
                }
            }
        }

      /* For non-AUTOEND, wait for TC then generate STOP if last msg */

      if (is_last && (msg->flags & I2C_M_NOSTOP))
        {
          /* No STOP requested - wait for TC */

          ret = gd32_i2c_wait_flag(priv, I2C_STAT_TC);
          if (ret < 0)
            {
              goto errout;
            }
        }
      else if (!is_last)
        {
          /* Not last message - wait for TC before repeated start */

          ret = gd32_i2c_wait_flag(priv, I2C_STAT_TC);
          if (ret < 0)
            {
              goto errout;
            }
        }
      else
        {
          /* Last message with AUTOEND - wait for STOP detection */

          ret = gd32_i2c_wait_flag(priv, I2C_STAT_STPDET);
          if (ret < 0)
            {
              goto errout;
            }

          /* Clear STPDET flag */

          gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET,
                          I2C_STATC_STPDETC);
        }
    }

  return OK;

errout:

  /* Generate STOP on error */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_STOP);

  /* Clear all status flags */

  gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);

  return ret;
}
#endif /* CONFIG_I2C_POLLED */

/****************************************************************************
 * Name: gd32_i2c_isr
 *
 * Description:
 *   I2C event/error interrupt handler for interrupt-driven transfers.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int gd32_i2c_isr(int irq, void *context, void *arg)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;
  uint32_t stat;

  stat = gd32_i2c_getreg(priv, GD32_I2C_STAT_OFFSET);

  /* Handle errors */

  if (stat & I2C_STAT_ERROR_MASK)
    {
      priv->status = stat;
      gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);

      /* Disable all interrupts */

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                         I2C_CTL0_INT_MASK, 0);

      priv->intstate = INTSTATE_DONE;
      nxsem_post(&priv->sem_isr);
      return OK;
    }

  /* NACK received */

  if (stat & I2C_STAT_NACK)
    {
      gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_NACKC);
      priv->status = stat;

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                         I2C_CTL0_INT_MASK, 0);

      priv->intstate = INTSTATE_DONE;
      nxsem_post(&priv->sem_isr);
      return OK;
    }

  /* Transmit interrupt (TI) */

  if ((stat & I2C_STAT_TI) && priv->dcnt > 0 &&
      !(priv->flags & I2C_M_READ))
    {
      gd32_i2c_putreg(priv, GD32_I2C_TDATA_OFFSET, *priv->ptr++);
      priv->dcnt--;
    }

  /* Receive data not empty (RBNE) */

  if ((stat & I2C_STAT_RBNE) && priv->dcnt > 0 &&
      (priv->flags & I2C_M_READ))
    {
      *priv->ptr++ = (uint8_t)(gd32_i2c_getreg(priv,
                               GD32_I2C_RDATA_OFFSET) &
                               I2C_RDATA_RDATA_MASK);
      priv->dcnt--;
    }

  /* Transfer Complete Reload (TCR) */

  if (stat & I2C_STAT_TCR)
    {
      if (priv->dcnt > 0)
        {
          uint8_t chunk;
          bool reload;

          if (priv->dcnt > I2C_MAX_RELOAD_SIZE)
            {
              chunk = I2C_MAX_RELOAD_SIZE;
              reload = true;
            }
          else
            {
              chunk = (uint8_t)priv->dcnt;
              reload = false;
            }

          uint32_t ctl1 = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
          ctl1 &= ~(I2C_CTL1_BYTENUM_MASK | I2C_CTL1_RELOAD);
          ctl1 |= I2C_CTL1_BYTENUM(chunk);
          if (reload)
            {
              ctl1 |= I2C_CTL1_RELOAD;
            }

          gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, ctl1);
        }
    }

  /* Transfer Complete (TC) - all bytes transferred, no AUTOEND */

  if (stat & I2C_STAT_TC)
    {
      /* Move to next message */

      priv->msgc--;
      if (priv->msgc > 0)
        {
          priv->msgv++;
          priv->ptr = priv->msgv->buffer;
          priv->dcnt = priv->msgv->length;
          priv->flags = priv->msgv->flags;

          bool is_read = (priv->flags & I2C_M_READ) != 0;
          bool is_last = (priv->msgc == 1);
          uint8_t nbytes = (priv->dcnt > I2C_MAX_RELOAD_SIZE) ?
                           I2C_MAX_RELOAD_SIZE : (uint8_t)priv->dcnt;
          bool reload = (priv->dcnt > I2C_MAX_RELOAD_SIZE);
          bool autoend = is_last && !(priv->flags & I2C_M_NOSTOP);

          gd32_i2c_sendstart(priv, priv->msgv->addr,
                             is_read ? I2C_DIR_READ : I2C_DIR_WRITE,
                             nbytes, reload, autoend);
        }
      else
        {
          /* All messages done. Generate STOP unless NOSTOP */

          if (!(priv->flags & I2C_M_NOSTOP))
            {
              gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0,
                                 I2C_CTL1_STOP);
            }

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                             I2C_CTL0_INT_MASK, 0);
          priv->intstate = INTSTATE_DONE;
          nxsem_post(&priv->sem_isr);
        }
    }

  /* Stop detected - transfer complete with AUTOEND */

  if (stat & I2C_STAT_STPDET)
    {
      gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_STPDETC);
      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                         I2C_CTL0_INT_MASK, 0);
      priv->intstate = INTSTATE_DONE;
      nxsem_post(&priv->sem_isr);
    }

  return OK;
}
#endif

/****************************************************************************
 * DMA Transfer Functions
 ****************************************************************************/

#ifdef CONFIG_GD32H7_I2C_DMA

/****************************************************************************
 * Name: gd32_i2c_dmatxcallback
 ****************************************************************************/

static void gd32_i2c_dmatxcallback(DMA_HANDLE handle, uint16_t isr,
                                   void *arg)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;

  gd32_dma_stop(handle);
  priv->txresult = isr | 0x0080;  /* Ensure non-zero */
  nxsem_post(&priv->txsem);
}

/****************************************************************************
 * Name: gd32_i2c_dmarxcallback
 ****************************************************************************/

static void gd32_i2c_dmarxcallback(DMA_HANDLE handle, uint16_t isr,
                                   void *arg)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;

  gd32_dma_stop(handle);
  priv->rxresult = isr | 0x0080;  /* Ensure non-zero */
  nxsem_post(&priv->rxsem);
}

/****************************************************************************
 * Name: gd32_i2c_dma_write
 *
 * Description:
 *   Write data using DMA. Following official H7xx firmware library flow:
 *   1. Configure DMA for M2P transfer with DMAMUX TX request
 *   2. Set AUTOEND, configure address/bytecount/direction, START
 *   3. Enable DENT in CTL0 for DMA transmit
 *   4. Wait for DMA completion
 *   5. Wait for STPDET (AUTOEND generates STOP)
 *
 ****************************************************************************/

static int gd32_i2c_dma_write(struct gd32_i2c_priv_s *priv,
                              struct i2c_msg_s *msg)
{
  struct gd32_dma_config_s dmaconfig;
  int ret;
  int remaining = msg->length;
  uint8_t *buf = msg->buffer;

  while (remaining > 0)
    {
      uint8_t chunk;
      bool autoend;

      if (remaining > I2C_MAX_RELOAD_SIZE)
        {
          chunk = I2C_MAX_RELOAD_SIZE;
          autoend = false;
        }
      else
        {
          chunk = (uint8_t)remaining;
          autoend = !(msg->flags & I2C_M_NOSTOP);
        }

      /* Configure DMA: memory to peripheral, 8-bit, memory increment */

      memset(&dmaconfig, 0, sizeof(dmaconfig));
      dmaconfig.periph_addr = priv->config->i2c_base +
                              GD32_I2C_TDATA_OFFSET;
      dmaconfig.memory_addr = (uint32_t)(uintptr_t)buf;
      dmaconfig.direction   = DMA_MEMORY_TO_PERIPH;
      dmaconfig.number      = chunk;
      dmaconfig.periph_width = DMA_PERIPH_WIDTH_8BIT;
      dmaconfig.memory_width = DMA_MEMORY_WIDTH_8BIT;
      dmaconfig.periph_inc  = false;
      dmaconfig.memory_inc  = true;
      dmaconfig.priority    = DMA_PRIORITY_HIGH;

      gd32_dma_setup(priv->txdma, &dmaconfig);

      /* Flush D-Cache to ensure DMA reads latest data from memory */

#ifdef CONFIG_ARMV7M_DCACHE
      up_clean_dcache((uintptr_t)buf, (uintptr_t)buf + chunk);
#endif

      /* Reset DMA semaphore and result */

      priv->txresult = 0;
      nxsem_reset(&priv->txsem, 0);

      /* Start DMA with FTF interrupt */

      gd32_dma_start(priv->txdma, gd32_i2c_dmatxcallback, priv,
                     DMA_CHXCTL_FTFIE);

      /* Configure I2C: address, direction, byte count, AUTOEND, START */

      if (buf == msg->buffer)
        {
          /* First chunk: send START */

          gd32_i2c_sendstart(priv, msg->addr, I2C_DIR_WRITE, chunk,
                             remaining > I2C_MAX_RELOAD_SIZE, autoend);
        }
      else
        {
          /* Reload continuation */

          uint32_t ctl1 = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
          ctl1 &= ~(I2C_CTL1_BYTENUM_MASK | I2C_CTL1_RELOAD |
                     I2C_CTL1_AUTOEND);
          ctl1 |= I2C_CTL1_BYTENUM(chunk);
          if (remaining > I2C_MAX_RELOAD_SIZE)
            {
              ctl1 |= I2C_CTL1_RELOAD;
            }

          if (autoend)
            {
              ctl1 |= I2C_CTL1_AUTOEND;
            }

          gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, ctl1);
        }

      /* Enable DMA transmit */

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_DENT);

      /* Wait for DMA completion */

      ret = nxsem_tickwait(&priv->txsem, I2C_TIMEOUT_TICKS);
      if (ret < 0)
        {
          gd32_dma_stop(priv->txdma);
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                             I2C_CTL0_DENT, 0);
          goto errout;
        }

      /* Check DMA result for errors */

      if (priv->txresult & DMA_INTF_ERRIF)
        {
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                             I2C_CTL0_DENT, 0);
          ret = -EIO;
          goto errout;
        }

      /* Disable DMA transmit */

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_DENT, 0);

      buf += chunk;
      remaining -= chunk;

      /* If reload needed, wait for TCR */

      if (remaining > 0)
        {
          ret = gd32_i2c_wait_flag(priv, I2C_STAT_TCR);
          if (ret < 0)
            {
              goto errout;
            }
        }
    }

  /* Wait for STOP detection (AUTOEND) */

  if (!(msg->flags & I2C_M_NOSTOP))
    {
      ret = gd32_i2c_wait_flag(priv, I2C_STAT_STPDET);
      if (ret < 0)
        {
          goto errout;
        }

      gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_STPDETC);
    }

  return OK;

errout:
  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_STOP);
  gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);
  return ret;
}

/****************************************************************************
 * Name: gd32_i2c_dma_read
 *
 * Description:
 *   Read data using DMA. Following official H7xx firmware library flow:
 *   1. Configure DMA for P2M transfer with DMAMUX RX request
 *   2. Set AUTOEND, configure address/bytecount/direction, START
 *   3. Enable DENR in CTL0 for DMA receive
 *   4. Wait for DMA completion
 *   5. Wait for STPDET
 *   Handles RELOAD for transfers > 255 bytes.
 *
 ****************************************************************************/

static int gd32_i2c_dma_read(struct gd32_i2c_priv_s *priv,
                             struct i2c_msg_s *msg)
{
  struct gd32_dma_config_s dmaconfig;
  int ret;
  int remaining = msg->length;
  uint8_t *buf = msg->buffer;

  while (remaining > 0)
    {
      uint8_t chunk;
      bool autoend;

      if (remaining > I2C_MAX_RELOAD_SIZE)
        {
          chunk = I2C_MAX_RELOAD_SIZE;
          autoend = false;
        }
      else
        {
          chunk = (uint8_t)remaining;
          autoend = !(msg->flags & I2C_M_NOSTOP);
        }

      /* Configure DMA: peripheral to memory, 8-bit, memory increment */

#ifdef CONFIG_ARMV7M_DCACHE
      /* Use bounce buffer if either end of the DMA range is not
       * cache-line aligned, to prevent D-Cache invalidation from
       * corrupting adjacent data sharing the same cache line.
       */

      bool use_bounce = !I2C_IS_ALIGNED(buf) ||
                        !I2C_IS_ALIGNED(buf + chunk);
      uint8_t *dma_buf = use_bounce ? priv->rxbuf : buf;
#else
      uint8_t *dma_buf = buf;
#endif

      memset(&dmaconfig, 0, sizeof(dmaconfig));
      dmaconfig.periph_addr = priv->config->i2c_base +
                              GD32_I2C_RDATA_OFFSET;
      dmaconfig.memory_addr = (uint32_t)(uintptr_t)dma_buf;
      dmaconfig.direction   = DMA_PERIPH_TO_MEMORY;
      dmaconfig.number      = chunk;
      dmaconfig.periph_width = DMA_PERIPH_WIDTH_8BIT;
      dmaconfig.memory_width = DMA_MEMORY_WIDTH_8BIT;
      dmaconfig.periph_inc  = false;
      dmaconfig.memory_inc  = true;
      dmaconfig.priority    = DMA_PRIORITY_HIGH;

      gd32_dma_setup(priv->rxdma, &dmaconfig);

      /* Invalidate D-Cache before DMA so stale cached data won't
       * be written back over DMA data later.
       */

#ifdef CONFIG_ARMV7M_DCACHE
      up_invalidate_dcache((uintptr_t)dma_buf,
                           (uintptr_t)dma_buf + chunk);
#endif

      /* Reset DMA semaphore and result */

      priv->rxresult = 0;
      nxsem_reset(&priv->rxsem, 0);

      /* Start DMA with FTF interrupt */

      gd32_dma_start(priv->rxdma, gd32_i2c_dmarxcallback, priv,
                     DMA_CHXCTL_FTFIE);

      /* Configure I2C: address, direction, byte count, START */

      if (buf == msg->buffer)
        {
          gd32_i2c_sendstart(priv, msg->addr, I2C_DIR_READ, chunk,
                             remaining > I2C_MAX_RELOAD_SIZE, autoend);
        }
      else
        {
          uint32_t ctl1 = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
          ctl1 &= ~(I2C_CTL1_BYTENUM_MASK | I2C_CTL1_RELOAD |
                     I2C_CTL1_AUTOEND);
          ctl1 |= I2C_CTL1_BYTENUM(chunk);
          if (remaining > I2C_MAX_RELOAD_SIZE)
            {
              ctl1 |= I2C_CTL1_RELOAD;
            }

          if (autoend)
            {
              ctl1 |= I2C_CTL1_AUTOEND;
            }

          gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, ctl1);
        }

      /* Enable DMA receive */

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_DENR);

      /* Wait for DMA completion */

      ret = nxsem_tickwait(&priv->rxsem, I2C_TIMEOUT_TICKS);
      if (ret < 0)
        {
          gd32_dma_stop(priv->rxdma);
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                             I2C_CTL0_DENR, 0);
          goto errout;
        }

      if (priv->rxresult & DMA_INTF_ERRIF)
        {
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                             I2C_CTL0_DENR, 0);
          ret = -EIO;
          goto errout;
        }

      /* Disable DMA receive */

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_DENR, 0);

      /* Invalidate D-Cache again after DMA completion so CPU sees
       * the fresh data written by DMA to memory.
       */

#ifdef CONFIG_ARMV7M_DCACHE
      up_invalidate_dcache((uintptr_t)dma_buf,
                           (uintptr_t)dma_buf + chunk);
      if (use_bounce)
        {
          memcpy(buf, dma_buf, chunk);
        }
#endif

      buf += chunk;
      remaining -= chunk;

      if (remaining > 0)
        {
          ret = gd32_i2c_wait_flag(priv, I2C_STAT_TCR);
          if (ret < 0)
            {
              goto errout;
            }
        }
    }

  /* Wait for STOP detection */

  if (!(msg->flags & I2C_M_NOSTOP))
    {
      ret = gd32_i2c_wait_flag(priv, I2C_STAT_STPDET);
      if (ret < 0)
        {
          goto errout;
        }

      gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_STPDETC);
    }

  return OK;

errout:
  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_STOP);
  gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);
  return ret;
}

/****************************************************************************
 * Name: gd32_i2c_dma_writeread
 *
 * Description:
 *   Combined write-then-read transfer.
 *   Phase 1 (Write): Polling - send register/memory address bytes.
 *                     Write message is typically 1-3 bytes (device address),
 *                     no AUTOEND (no STOP).
 *   Phase 2 (Read):  Read data using DMA with repeated START.
 *
 *   This follows the official H7xx Demo EEPROM read flow:
 *   1. Send address byte(s) without AUTOEND (no STOP)
 *   2. Wait for TC
 *   3. Restart with read direction, use DMA for data reception
 *
 ****************************************************************************/

static int gd32_i2c_dma_writeread(struct gd32_i2c_priv_s *priv,
                                  struct i2c_msg_s *wrmsg,
                                  struct i2c_msg_s *rdmsg)
{
  int ret;
  int j;

  /* Phase 1: Write address/register bytes via polling.
   * In write-then-read pattern the write message is typically just
   * a device register address (1-3 bytes), always below DMA threshold,
   * so polling is the only sensible approach here.
   */

  gd32_i2c_sendstart(priv, wrmsg->addr, I2C_DIR_WRITE,
                     (uint8_t)wrmsg->length, false, false);

  for (j = 0; j < wrmsg->length; j++)
    {
      ret = gd32_i2c_wait_flag(priv, I2C_STAT_TI);
      if (ret < 0)
        {
          goto errout;
        }

      gd32_i2c_putreg(priv, GD32_I2C_TDATA_OFFSET,
                      wrmsg->buffer[j]);
    }

  /* Wait for Transfer Complete (all write bytes sent, no AUTOEND) */

  ret = gd32_i2c_wait_flag(priv, I2C_STAT_TC);
  if (ret < 0)
    {
      goto errout;
    }

  /* Phase 2: Read data with DMA using repeated START */

  ret = gd32_i2c_dma_read(priv, rdmsg);
  return ret;

errout:
  gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_STOP);
  gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);
  return ret;
}

#endif /* CONFIG_GD32H7_I2C_DMA */

/****************************************************************************
 * Name: gd32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function. Routes to DMA or polling based on
 *   configuration and message characteristics.
 *
 *   DMA path is used when:
 *   - DMA is enabled for this I2C port
 *   - Single write message >= threshold -> dma_write
 *   - Single read message >= threshold -> dma_read
 *   - Two messages (write + read) and read >= threshold -> dma_writeread
 *   Otherwise, polling path is used.
 *
 ****************************************************************************/

static int gd32_i2c_transfer(struct i2c_master_s *dev,
                             struct i2c_msg_s *msgs, int count)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  int ret;
  int ret2;

  DEBUGASSERT(count > 0);

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set I2C clock frequency */

  gd32_i2c_set_timing(priv, msgs[0].frequency);

  /* Clear any pending error flags */

  gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);

#ifdef CONFIG_GD32H7_I2C_DMA
  /* Try DMA path if available */

  if (count == 1 && !(msgs[0].flags & I2C_M_READ) &&
      msgs[0].length >= CONFIG_GD32H7_I2C_DMATHRESHOLD &&
      priv->txdma != NULL)
    {
      /* Single write message via DMA */

      ret = gd32_i2c_dma_write(priv, &msgs[0]);
    }
  else if (count == 1 && (msgs[0].flags & I2C_M_READ) &&
           msgs[0].length >= CONFIG_GD32H7_I2C_DMATHRESHOLD &&
           priv->rxdma != NULL)
    {
      /* Single read message via DMA */

      ret = gd32_i2c_dma_read(priv, &msgs[0]);
    }
  else if (count == 2 &&
           !(msgs[0].flags & I2C_M_READ) &&
           (msgs[1].flags & I2C_M_READ) &&
           msgs[1].length >= CONFIG_GD32H7_I2C_DMATHRESHOLD &&
           priv->rxdma != NULL)
    {
      /* Write-then-read via DMA */

      ret = gd32_i2c_dma_writeread(priv, &msgs[0], &msgs[1]);
    }
  else
#endif
    {
      /* Polling transfer */

#ifndef CONFIG_I2C_POLLED
      /* Interrupt-driven transfer */

      priv->msgv = msgs;
      priv->msgc = count;
      priv->ptr = msgs[0].buffer;
      priv->dcnt = msgs[0].length;
      priv->flags = msgs[0].flags;
      priv->intstate = INTSTATE_WAITING;
      priv->status = 0;

      /* Start first message */

      bool is_read = (priv->flags & I2C_M_READ) != 0;
      bool is_last = (count == 1);
      uint8_t nbytes = (priv->dcnt > I2C_MAX_RELOAD_SIZE) ?
                       I2C_MAX_RELOAD_SIZE : (uint8_t)priv->dcnt;
      bool reload = (priv->dcnt > I2C_MAX_RELOAD_SIZE);
      bool autoend = is_last && !(priv->flags & I2C_M_NOSTOP);

      /* Enable all relevant interrupts */

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0,
                         I2C_CTL0_INT_MASK);

      gd32_i2c_sendstart(priv, msgs[0].addr,
                         is_read ? I2C_DIR_READ : I2C_DIR_WRITE,
                         nbytes, reload, autoend);

      /* Wait for ISR to complete */

      ret = gd32_i2c_sem_waitdone(priv);

      /* Disable interrupts */

      gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                         I2C_CTL0_INT_MASK, 0);

      if (ret < 0 && priv->intstate != INTSTATE_DONE)
        {
          /* Timeout - generate STOP and clear flags */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0,
                             I2C_CTL1_STOP);
          gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);
        }
      else if (priv->status & I2C_STAT_NACK)
        {
          ret = -ENXIO;
        }
      else if (priv->status & I2C_STAT_ERROR_MASK)
        {
          ret = -EIO;
        }
      else
        {
          ret = OK;
        }

      priv->intstate = INTSTATE_IDLE;
#else
      /* Pure polling transfer */

      ret = gd32_i2c_polling_transfer(priv, msgs, count);
#endif
    }

  ret2 = nxmutex_unlock(&priv->lock);

  return (ret < 0) ? ret : ret2;
}

/****************************************************************************
 * Name: gd32_i2c_reset
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int gd32_i2c_reset(struct i2c_master_s *dev)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Reset the I2C peripheral */

  gd32_i2c_deinit(priv);
  gd32_i2c_init(priv);

  ret = nxmutex_unlock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: gd32_i2c_init
 *
 * Description:
 *   Initialize the I2C hardware:
 *   1. Enable I2C clock
 *   2. Reset I2C peripheral
 *   3. Configure GPIO pins
 *   4. Set default timing (100kHz)
 *   5. Enable I2C
 *   6. Attach interrupts (if not polled)
 *   7. Allocate DMA channels (if DMA enabled)
 *
 ****************************************************************************/

static void gd32_i2c_init(struct gd32_i2c_priv_s *priv)
{
  const struct gd32_i2c_config_s *config = priv->config;

  /* Enable I2C clock */

  modifyreg32(GD32_RCU_APB1EN, 0, config->clk_bit);

  /* Select IRC64MDIV as I2C clock source (value 2).
   * Timing values are calculated for IRC64MDIV (32MHz) input.
   */

  modifyreg32(config->clk_cfg_reg, config->clk_cfg_mask,
              2u << config->clk_cfg_shift);

  /* Reset I2C peripheral */

  modifyreg32(GD32_RCU_APB1RST, 0, config->rst_bit);
  modifyreg32(GD32_RCU_APB1RST, config->rst_bit, 0);

  /* Configure GPIO pins (AF, open-drain, pull-up) */

  gd32_gpio_config(config->scl_pin);
  gd32_gpio_config(config->sda_pin);

  /* Clear pending flags and disable I2C */

  gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, 0);
  gd32_i2c_putreg(priv, GD32_I2C_STATC_OFFSET, I2C_STATC_ALL);

  /* Set default timing (100kHz, will be reconfigured on first transfer) */

  priv->frequency = 0;
  gd32_i2c_set_timing(priv, 100000);

  /* Enable I2C */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_I2CEN);

#ifndef CONFIG_I2C_POLLED
  /* Attach and enable event/error interrupts */

  irq_attach(config->ev_irq, gd32_i2c_isr, priv);
  irq_attach(config->er_irq, gd32_i2c_isr, priv);
  up_enable_irq(config->ev_irq);
  up_enable_irq(config->er_irq);
#endif

#ifdef CONFIG_GD32H7_I2C_DMA
  /* Allocate DMA channels if per-bus DMA is configured (reqid != 0) */

  if (config->txdma_reqid != 0)
    {
      priv->txdma = gd32_dma_channel_alloc(config->txdma_reqid);
    }

  if (config->rxdma_reqid != 0)
    {
      priv->rxdma = gd32_dma_channel_alloc(config->rxdma_reqid);
    }
#endif
}

/****************************************************************************
 * Name: gd32_i2c_deinit
 *
 * Description:
 *   De-initialize the I2C hardware.
 *
 ****************************************************************************/

static void gd32_i2c_deinit(struct gd32_i2c_priv_s *priv)
{
  const struct gd32_i2c_config_s *config = priv->config;

#ifdef CONFIG_GD32H7_I2C_DMA
  /* Free DMA channels */

  if (priv->txdma != NULL)
    {
      gd32_dma_channel_free(priv->txdma);
      priv->txdma = NULL;
    }

  if (priv->rxdma != NULL)
    {
      gd32_dma_channel_free(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif

#ifndef CONFIG_I2C_POLLED
  /* Disable and detach interrupts */

  up_disable_irq(config->ev_irq);
  up_disable_irq(config->er_irq);
  irq_detach(config->ev_irq);
  irq_detach(config->er_irq);
#endif

  /* Disable I2C */

  gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, 0);

  /* Unconfigure GPIO pins */

  gd32_gpio_unconfig(config->scl_pin);
  gd32_gpio_unconfig(config->sda_pin);

  /* Disable I2C clock */

  modifyreg32(GD32_RCU_APB1EN, config->clk_bit, 0);

  priv->frequency = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2cbus_initialize
 ****************************************************************************/

struct i2c_master_s *gd32_i2cbus_initialize(int port)
{
  struct gd32_i2c_priv_s *priv;
  int ret;

  i2cinfo("I2C%d: initialize\n", port);

  switch (port)
    {
#ifdef CONFIG_GD32H7_I2C0
      case 0:
        priv = &gd32_i2c0_priv;
        break;
#endif
#ifdef CONFIG_GD32H7_I2C1
      case 1:
        priv = &gd32_i2c1_priv;
        break;
#endif
#ifdef CONFIG_GD32H7_I2C2
      case 2:
        priv = &gd32_i2c2_priv;
        break;
#endif
#ifdef CONFIG_GD32H7_I2C3
      case 3:
        priv = &gd32_i2c3_priv;
        break;
#endif
      default:
        i2cerr("ERROR: Invalid I2C port %d\n", port);
        return NULL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  if (priv->refs++ == 0)
    {
      gd32_i2c_init(priv);
    }

  ret = nxmutex_unlock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: gd32_i2cbus_uninitialize
 ****************************************************************************/

int gd32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(dev != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (--priv->refs == 0)
    {
      gd32_i2c_deinit(priv);
    }

  ret = nxmutex_unlock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

#endif /* CONFIG_GD32H7_I2C */
