/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_i2c.c
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

/* Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 * TODO:
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained
 *      by the i2c_init(); it extends the Device structure from the
 *      nuttx/i2c/i2c.h;
 *      Instance points to OPS, to common I2C Hardware private data and
 *      contains its own private data, as frequency, address, mode of
 *      operation (in the future)
 *  - Private: Private data of an I2C Hardware
 *
 * TODO
 *  - Check for all possible deadlocks (as BUSY='1' I2C needs to be reset in
 *    hardware using the I2C_CTL0_SRESET)
 *  - SMBus support (hardware layer timings are already supported) and add
 *    SMBA gpio pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or
 *      - 1 x 10 bit addresses + 1 x 7 bit address (for the slave in
 *        Dual-Address mode)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
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
#include "gd32e11x.h"
#include "gd32e11x_i2c.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_GD32E11X_I2C0) || defined(CONFIG_GD32E11X_I2C1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_GD32E11X_I2C_TIMEOSEC) && !defined(CONFIG_GD32E11X_I2C_TIMEOMS)
#  define CONFIG_GD32E11X_I2C_TIMEOSEC 0
#  define CONFIG_GD32E11X_I2C_TIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_GD32E11X_I2C_TIMEOSEC)
#  define CONFIG_GD32E11X_I2C_TIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_GD32E11X_I2C_TIMEOMS)
#  define CONFIG_GD32E11X_I2C_TIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_GD32E11X_I2CTIMEOTICKS
#  define CONFIG_GD32E11X_I2CTIMEOTICKS \
  (SEC2TICK(CONFIG_GD32E11X_I2C_TIMEOSEC) + \
   MSEC2TICK(CONFIG_GD32E11X_I2C_TIMEOMS))
#endif

#ifndef CONFIG_GD32E11X_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_GD32E11X_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_GD32E11X_I2CTIMEOTICKS)
#endif

/* Macros to convert a I2C pin to a GPIO output */

#define I2C_OUTPUT (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_OD |\
                    GPIO_CFG_SPEED_50MHZ | GPIO_CFG_OUTPUT_SET)

#define MKI2C_OUTPUT(p) (((p) & (GPIO_CFG_PORT_MASK | GPIO_CFG_PIN_MASK)) |\
                         I2C_OUTPUT)

/* I2C DMA priority */

#ifdef CONFIG_GD32E11X_I2C_DMA

#  error "Now I2C DMA has not ready"

#  if defined(CONFIG_I2C_DMAPRIO)
#    if (CONFIG_I2C_DMAPRIO & ~DMA_CHXCTL_PRIO_MASK) != 0
#      error "Illegal value for CONFIG_I2C_DMAPRIO"
#    endif
#    define I2C_DMA_PRIO     CONFIG_I2C_DMAPRIO
#  else
#    define I2C_DMA_PRIO     DMA_PRIO_HIGH_SELECT
#  endif
#endif

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define gd32_i2c_tracereset(p)
#  define gd32_i2c_tracenew(p,s)
#  define gd32_i2c_traceevent(p,e,a)
#  define gd32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum gd32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum gd32_trace_e
{
  I2CEVENT_NONE = 0,
  I2CEVENT_STATE_ERROR,
  I2CEVENT_ISR_SHUTDOWN,
  I2CEVENT_ISR_CALL,
  I2CEVENT_ISR_EMPTY_CALL,
  I2CEVENT_MSG_HANDLING,
  I2CEVENT_POLL_NOT_READY,
  I2CEVENT_EMPTY_MSG,
  I2CEVENT_START,
  I2CEVENT_SENDADDR,
  I2CEVENT_ADDRESS_ACKED,
  I2CEVENT_ADDRESS_NACKED,
  I2CEVENT_NACK,
  I2CEVENT_READ,
  I2CEVENT_READ_ERROR,
  I2CEVENT_ADDRESS_ACKED_READ_1,
  I2CEVENT_ADDRESS_ACKED_READ_2,
  I2CEVENT_WRITE_TO_DR,
  I2CEVENT_WRITE_STOP,
  I2CEVENT_WRITE_RESTART,
  I2CEVENT_WRITE_NO_RESTART,
  I2CEVENT_WRITE_ERROR,
  I2CEVENT_WRITE_FLAG_ERROR,
  I2CEVENT_TC_RESTART,
  I2CEVENT_TC_NO_RESTART,
  I2CEVENT_ERROR
};

/* Trace data */

struct gd32_trace_s
{
  uint32_t status;             /* I2C 32-bit STAT0|STAT1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum gd32_intstate_e event;  /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};

/* I2C Device hardware configuration */

struct gd32_i2c_config_s
{
  uint32_t i2c_base;          /* I2C base address */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  uint32_t event_irq;         /* Event IRQ */
  uint32_t error_irq;         /* Error IRQ */
#endif
};

/* I2C Device Private Data */

struct gd32_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  /* Port configuration */

  const struct gd32_i2c_config_s *config;

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum gd32_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  volatile int dcnt;           /* Current message length */
  uint16_t flags;              /* Current message flags */
  bool check_addr_ack;         /* Flag to signal if on next interrupt address has ACKed */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct gd32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer STAT0|STAT1 status */

  /* I2C DMA support */

#ifdef CONFIG_GD32E11X_I2C_DMA
  DMA_HANDLE      txdma;       /* TX DMA handle */
  DMA_HANDLE      rxdma;       /* RX DMA handle */
  uint8_t         txch;        /* TX DMA channel number */
  uint8_t         rxch;        /* RX DMA channel number */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Clock */

static void            gd32_i2c_periph_reset(uint32_t i2cbase);
static void            gd32_i2c_clock_enable(uint32_t i2cbase);
static void            gd32_i2c_clock_disable(uint32_t i2cbase);

static inline uint16_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                       uint8_t offset);
static inline void     gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                       uint8_t offset, uint16_t value);
static inline void     gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                          uint8_t offset, uint16_t clearbits,
                                          uint16_t setbits);

#ifdef CONFIG_GD32E11X_I2C_DYNTIMEO
static uint32_t    gd32_i2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_GD32E11X_I2C_DYNTIMEO */

static inline int  gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_waitstop(struct gd32_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void  gd32_i2c_tracereset(struct gd32_i2c_priv_s *priv);
static void  gd32_i2c_tracenew(struct gd32_i2c_priv_s *priv,
                               uint32_t status);
static void  gd32_i2c_traceevent(struct gd32_i2c_priv_s *priv,
                                 enum gd32_trace_e event, uint32_t parm);
static void  gd32_i2c_tracedump(struct gd32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void        gd32_i2c_setclock(struct gd32_i2c_priv_s *priv,
                                     uint32_t frequency);
static inline void gd32_i2c_sendstart(struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_clrstart(struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sendstop(struct gd32_i2c_priv_s *priv);
static inline
uint32_t           gd32_i2c_getstatus(struct gd32_i2c_priv_s *priv);

static int         gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv);

#ifndef CONFIG_I2C_POLLED
static int         gd32_i2c_isr(int irq, void *context, void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int         gd32_i2c_init(struct gd32_i2c_priv_s *priv);
static int         gd32_i2c_deinit(struct gd32_i2c_priv_s *priv);
static int         gd32_i2c_transfer(struct i2c_master_s *dev,
                                     struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int         gd32_i2c_reset(struct i2c_master_s *dev);
#endif

/* DMA support */

#ifdef CONFIG_GD32E11X_I2C_DMA
static void        gd32_i2c_dmarxcallback(DMA_HANDLE handle,
                                          uint8_t status, void *arg);
static void        gd32_i2c_dmatxcallback(DMA_HANDLE handle,
                                          uint8_t status, void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C interface */

static const struct i2c_ops_s gd32_i2c_ops =
{
  .transfer = gd32_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset  = gd32_i2c_reset
#endif
};

/* I2C device structures */

#ifdef CONFIG_GD32E11X_I2C0
static const struct gd32_i2c_config_s gd32_i2c0_config =
{
  .i2c_base   = GD32_I2C0_BASE,
  .scl_pin    = GPIO_I2C0_SCL,
  .sda_pin    = GPIO_I2C0_SDA,
#ifndef CONFIG_I2C_POLLED
  .event_irq  = GD32_IRQ_I2C0_EV,
  .error_irq  = GD32_IRQ_I2C0_ER
#endif
};

static struct gd32_i2c_priv_s gd32_i2c0_priv =
{
  .ops        = &gd32_i2c_ops,
  .config     = &gd32_i2c0_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_GD32E11X_I2C_DMA
  .rxch       = DMA_REQ_I2C0_RX,
  .txch       = DMA_REQ_I2C0_TX,
#endif
};
#endif

#ifdef CONFIG_GD32E11X_I2C1
static const struct gd32_i2c_config_s gd32_i2c1_config =
{
  .i2c_base   = GD32_I2C1_BASE,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .event_irq  = GD32_IRQ_I2C1_EV,
  .error_irq  = GD32_IRQ_I2C1_ER
#endif
};

static struct gd32_i2c_priv_s gd32_i2c1_priv =
{
  .ops        = &gd32_i2c_ops,
  .config     = &gd32_i2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_GD32E11X_I2C_DMA
  .rxch       = DMA_REQ_I2C1_RX,
  .txch       = DMA_REQ_I2C1_TX,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2c_periph_reset
 *
 * Description:
 *   Reset the I2C.
 *
 ****************************************************************************/

static void gd32_i2c_periph_reset(uint32_t i2cbase)
{
  uint32_t rcu_rst;
  uint32_t regaddr;

  switch (i2cbase)
    {
    default:
      return;
#ifdef CONFIG_GD32E11X_I2C0
    case GD32_I2C0_BASE:
      rcu_rst = RCU_APB1RST_I2C0RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
#ifdef CONFIG_GD32E11X_I2C1
    case GD32_I2C1_BASE:
      rcu_rst = RCU_APB1RST_I2C1RST;
      regaddr = GD32_RCU_APB1RST;
      break;
#endif
    }

  modifyreg32(regaddr, 0, rcu_rst);
  modifyreg32(regaddr, rcu_rst, 0);
}

/****************************************************************************
 * Name: gd32_i2c_clock_enable
 *
 * Description:
 *   Enable I2C clock
 *
 ****************************************************************************/

static void gd32_i2c_clock_enable(uint32_t i2cbase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  switch (i2cbase)
    {
    default:
      return;
#ifdef CONFIG_GD32E11X_I2C0
    case GD32_I2C0_BASE:
      rcu_en = RCU_APB1EN_I2C0EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32E11X_I2C1
    case GD32_I2C1_BASE:
      rcu_en = RCU_APB1EN_I2C1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
    }

  /* Enable APB 1/2 clock for I2C */

  modifyreg32(regaddr, 0, rcu_en);
}

/****************************************************************************
 * Name: gd32_i2c_clock_disable
 *
 * Description:
 *   Dinable I2C clock
 ****************************************************************************/

static void gd32_i2c_clock_disable(uint32_t i2cbase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  switch (i2cbase)
    {
    default:
      return;
#ifdef CONFIG_GD32E11X_I2C0
    case GD32_I2C0_BASE:
      rcu_en = RCU_APB1EN_I2C0EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
#ifdef CONFIG_GD32E11X_I2C1
    case GD32_I2C1_BASE:
      rcu_en = RCU_APB1EN_I2C1EN;
      regaddr = GD32_RCU_APB1EN;
      break;
#endif
    }

  /* Disable APB 1/2 clock for I2C */

  modifyreg32(regaddr, rcu_en, 0);
}

/****************************************************************************
 * Name: gd32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t gd32_i2c_getreg(struct gd32_i2c_priv_s *priv,
                                       uint8_t offset)
{
  return getreg16(priv->config->i2c_base + offset);
}

/****************************************************************************
 * Name: gd32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_i2c_putreg(struct gd32_i2c_priv_s *priv,
                                   uint8_t offset, uint16_t value)
{
  putreg16(value, priv->config->i2c_base + offset);
}

/****************************************************************************
 * Name: gd32_i2c_modifyreg
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_i2c_modifyreg(struct gd32_i2c_priv_s *priv,
                                      uint8_t offset, uint16_t clearbits,
                                      uint16_t setbits)
{
  modifyreg16(priv->config->i2c_base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: gd32_i2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_I2C_DYNTIMEO
static uint32_t gd32_i2c_toticks(int msgc, struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  return USEC2TICK(CONFIG_GD32E11X_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: gd32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

static inline int gd32_i2c_sem_waitdone(struct gd32_i2c_priv_s *priv)
{
#ifdef CONFIG_I2C_POLLED
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;
  int loop_count = 0;

  timeout = CONFIG_GD32E11X_I2CTIMEOTICKS;
  priv->intstate = INTSTATE_WAITING;
  start = clock_systime_ticks();

  do
    {
      elapsed = clock_systime_ticks() - start;
      gd32_i2c_isr_process(priv);
      loop_count++;
    }
  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
#else
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = enter_critical_section();

  regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  regval |= (I2C_CTL1_ERRIE | I2C_CTL1_EVIE);
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);

  priv->intstate = INTSTATE_WAITING;
  do
    {
#ifdef CONFIG_GD32E11X_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                           gd32_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_GD32E11X_I2CTIMEOTICKS);
#endif
      if (ret < 0)
        {
          break;
        }
    }
  while (priv->intstate != INTSTATE_DONE);

  priv->intstate = INTSTATE_IDLE;

  regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
  regval &= ~I2C_CTL1_INTS_MASK;
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);

  leave_critical_section(flags);
  return ret;
#endif
}

/****************************************************************************
 * Name: gd32_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_waitstop(struct gd32_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t ctl0;
  uint32_t stat0;

  /* Select a timeout */

#ifdef CONFIG_GD32E11X_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_GD32E11X_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_GD32E11X_I2CTIMEOTICKS;
#endif

  /* Wait for stop condition to complete */

  start = clock_systime_ticks();
  do
    {
      /* Calculate elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Check for STOP condition cleared */

      ctl0 = gd32_i2c_getreg(priv, GD32_I2C_CTL0_OFFSET);
      if ((ctl0 & I2C_CTL0_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      stat0 = gd32_i2c_getreg(priv, GD32_I2C_STAT0_OFFSET);
      if ((stat0 & I2C_STAT0_SMBTO) != 0)
        {
          return;
        }
    }
  while (elapsed < timeout);

  /* Timeout occurred */

}

#ifndef CONFIG_I2C_TRACE
#  define gd32_i2c_tracereset(p)
#  define gd32_i2c_tracenew(p,s)
#  define gd32_i2c_traceevent(p,e,a)
#  define gd32_i2c_tracedump(p)
#endif

/****************************************************************************
 * Name: gd32_i2c_traceclear
 *
 * Description:
 *   Clear trace entry
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void gd32_i2c_traceclear(struct gd32_trace_s *trace)
{
  trace->status = 0;
  trace->count  = 0;
  trace->event  = I2CEVENT_NONE;
  trace->parm   = 0;
  trace->time   = 0;
}

/****************************************************************************
 * Name: gd32_i2c_tracereset
 *
 * Description:
 *   Reset the trace info for a new data collection
 *
 ****************************************************************************/

static void gd32_i2c_tracereset(struct gd32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  gd32_i2c_traceclear(&priv->trace[0]);
}

/****************************************************************************
 * Name: gd32_i2c_tracenew
 *
 * Description:
 *   Create a new trace entry
 *
 ****************************************************************************/

static void gd32_i2c_tracenew(struct gd32_i2c_priv_s *priv,
                              uint32_t status)
{
  struct gd32_trace_s *trace = &priv->trace[priv->tndx];

  if (trace->count == 0 || status != trace->status)
    {
      if (trace->count != 0)
        {
          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      gd32_i2c_traceclear(trace);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systime_ticks();
    }
  else
    {
      trace->count++;
    }
}

/****************************************************************************
 * Name: gd32_i2c_traceevent
 *
 * Description:
 *   Record a new trace event
 *
 ****************************************************************************/

static void gd32_i2c_traceevent(struct gd32_i2c_priv_s *priv,
                                enum gd32_trace_e event, uint32_t parm)
{
  struct gd32_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      trace->event = event;
      trace->parm  = parm;

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          return;
        }

      priv->tndx++;
      gd32_i2c_traceclear(&priv->trace[priv->tndx]);
    }
}

/****************************************************************************
 * Name: gd32_i2c_tracedump
 *
 * Description:
 *   Dump the trace results
 *
 ****************************************************************************/

static void gd32_i2c_tracedump(struct gd32_i2c_priv_s *priv)
{
  struct gd32_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i <= priv->tndx; i++)
    {
      trace = &priv->trace[i];
      if (trace->count > 0)
        {
          syslog(LOG_DEBUG,
                 "%2d. STATUS: %08x COUNT: %3d EVENT: %2d PARM: %08x TIME: %d\n",
                 i + 1, trace->status, trace->count, trace->event, trace->parm,
                 (int)(trace->time - priv->start_time));
        }
    }
}
#endif

/****************************************************************************
 * Name: gd32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void gd32_i2c_setclock(struct gd32_i2c_priv_s *priv,
                              uint32_t frequency)
{
  uint16_t ctl0;
  uint16_t ckfg;
  uint16_t trise;
  uint16_t freq_mhz;
  uint16_t speed;

  if (frequency != priv->frequency)
    {
      ctl0 = gd32_i2c_getreg(priv, GD32_I2C_CTL0_OFFSET);
      gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, ctl0 & ~I2C_CTL0_I2CEN);

      freq_mhz = (uint16_t)(GD32_PCLK1_FREQUENCY / 1000000);
      ckfg = 0;

      if (frequency <= 100000)
        {
          speed = (uint16_t)(GD32_PCLK1_FREQUENCY / (frequency << 1));

          if (speed < 4)
            {
              speed = 4;
            }

          ckfg |= speed;
          trise = freq_mhz + 1;
        }
      else
        {
#ifdef CONFIG_GD32E11X_I2C_DUTY16_9
          speed = (uint16_t)(GD32_PCLK1_FREQUENCY / (frequency * 25));
          ckfg |= (I2C_CKCFG_DTCY | I2C_CKCFG_FAST);
#else
          speed = (uint16_t)(GD32_PCLK1_FREQUENCY / (frequency * 3));
          ckfg |= I2C_CKCFG_FAST;
#endif

          if (speed < 1)
            {
              speed = 1;
            }

          ckfg |= speed;
          trise = (uint16_t)(((freq_mhz * 300) / 1000) + 1);
        }

      gd32_i2c_putreg(priv, GD32_I2C_CKCFG_OFFSET, ckfg);
      gd32_i2c_putreg(priv, GD32_I2C_RT_OFFSET, trise);

      gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, ctl0);

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: gd32_i2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ****************************************************************************/

static inline void gd32_i2c_sendstart(struct gd32_i2c_priv_s *priv)
{
  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_START);
}

/****************************************************************************
 * Name: gd32_i2c_clrstart
 *
 * Description:
 *   Clear the STOP, START or PECTRANS bits
 *
 ****************************************************************************/

static inline void gd32_i2c_clrstart(struct gd32_i2c_priv_s *priv)
{
  /* Clear the STOP, START or PECTRANS bits */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                     I2C_CTL0_START | I2C_CTL0_STOP | I2C_CTL0_PECTRANS, 0);
}

/****************************************************************************
 * Name: gd32_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void gd32_i2c_sendstop(struct gd32_i2c_priv_s *priv)
{
  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN,
                     I2C_CTL0_STOP);
}

/****************************************************************************
 * Name: gd32_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (STAT1 merged to STAT0)
 *
 ****************************************************************************/

static inline uint32_t gd32_i2c_getstatus(struct gd32_i2c_priv_s *priv)
{
  uint32_t status = gd32_i2c_getreg(priv, GD32_I2C_STAT0_OFFSET);
  status |= ((uint32_t)gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET)) << 16;
  return status;
}

/****************************************************************************
 * Name: gd32_i2c_isr_process
 *
 * Description:
 *   I2C interrupt service routine
 *
 ****************************************************************************/

static int gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv)
{
  uint32_t status;
#ifndef CONFIG_I2C_POLLED
  uint32_t regval;
#endif

  /* Get state of the I2C controller (register STAT0 only)
   *
   * Get control register STAT0 only as reading both STAT0 and STAT1
   * clears the ADDR flag(possibly others) causing the hardware to
   * advance to the next state without the proper action being taken.
   */

  status = gd32_i2c_getreg(priv, GD32_I2C_STAT0_OFFSET);

  /* Update private version of the state */

  priv->status = status;

  /* Check if this is a new transmission so to set up the
   * trace table accordingly.
   */

  gd32_i2c_tracenew(priv, status);
  gd32_i2c_traceevent(priv, I2CEVENT_ISR_CALL, 0);

  /* Messages handling (1/2)
   *
   * Message handling should only operate when a message has been completely
   * sent and after the ISR had the chance to run to set bits after the last
   * written/read byte, i.e. priv->dcnt == -1.
   */

  if (priv->dcnt == -1 && priv->msgc > 0)
    {
      /* Check for error conditions that would prevent message handling */

      if (((priv->msgv[0].flags & I2C_M_NOSTART) != 0 &&
           (status & I2C_STAT0_TBE) == 0) ||
          ((priv->msgv[0].flags & I2C_M_NOSTART) == 0 &&
           (status & I2C_STAT0_SBSEND) == 0))
        {
#if defined(CONFIG_I2C_POLLED)
          return OK;
#else
          priv->status |= I2C_STAT0_SMBTO;
          goto state_error;
#endif
        }


      /* Get current message to process data and copy to private structure */

      priv->ptr = priv->msgv->buffer;
      priv->dcnt = priv->msgv->length;
      priv->flags = priv->msgv->flags;


      /* Decrease counter to indicate the number of messages left */

      priv->msgc--;

      /* Move to next message or set to NULL if last message */

      if (priv->msgc == 0)
        {
          /* No more messages */
        }
      else
        {
          /* Move to next message */

          priv->msgv++;
        }

      /* Trace event */

      gd32_i2c_traceevent(priv, I2CEVENT_MSG_HANDLING, priv->msgc);
    }

  /* Address Handling
   *
   * Check if a start bit was set and transmit address with proper format.
   */

  if ((status & I2C_STAT0_SBSEND) != 0)
    {
      /* Start bit is set */


      /* Check for empty message */

      if (priv->dcnt > 0)
        {
          /* Send address byte */

          gd32_i2c_putreg(priv, GD32_I2C_DATA_OFFSET,
                          (priv->flags & I2C_M_TEN) ?
                          0 : ((priv->msgv->addr << 1) |
                          (priv->flags & I2C_M_READ)));

          /* Flag that address has just been sent */

          priv->check_addr_ack = true;

          gd32_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgv->addr);
        }
      else
        {
          gd32_i2c_traceevent(priv, I2CEVENT_EMPTY_MSG, 0);

          priv->dcnt = -1;

#ifndef CONFIG_I2C_POLLED
          /* Restart ISR by setting an interrupt buffer bit */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_BUFIE);
#endif
        }
    }

  /* Check for NACK after an address */

#ifndef CONFIG_I2C_POLLED
  else if ((status & I2C_STAT0_ADDSEND) == 0 && priv->check_addr_ack)
    {

      /* Terminate message chain */

      priv->dcnt = -1;
      priv->msgc = 0;
      priv->check_addr_ack = false;

      /* Send stop bit */

      gd32_i2c_sendstop(priv);

      gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_NACKED, priv->msgv->addr);
    }
#endif

  /* ACK in read mode */

  else if ((priv->flags & I2C_M_READ) != 0 &&
           (status & I2C_STAT0_ADDSEND) != 0 &&
           priv->check_addr_ack)
    {
      /* Reset check addr flag */

      priv->check_addr_ack = false;

      if (priv->dcnt == 1)
        {
          /* Single byte read */


          /* Set POS bit to zero */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_POAP, 0);

          /* Set NACK */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN, 0);

#ifndef CONFIG_I2C_POLLED
          /* Enable RxNE and TxE buffers */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_BUFIE);
#endif

          /* Clear ADDR flag */

          status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

          /* Send Stop/Restart */

          if (priv->msgc > 0)
            {
              gd32_i2c_sendstart(priv);
            }
          else
            {
              gd32_i2c_sendstop(priv);
            }

          gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED_READ_1, 0);
        }
      else if (priv->dcnt == 2)
        {
          /* Two byte read */

          /* Set POS bit */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_POAP);

          /* Set NACK */

          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN, 0);

          /* Clear ADDR flag */

          status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

          gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED_READ_2, 0);
        }
      else
        {

          /* Clear ADDR flag */

          status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

          gd32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED, 0);

#ifndef CONFIG_I2C_POLLED
          if (priv->dcnt > 3)
            {
              gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0,
                                 I2C_CTL1_BUFIE);
            }
#endif
        }
    }

  /* Write mode */

  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & I2C_STAT0_BTC) != 0 &&
           priv->dcnt == 0)
    {
      /* After last byte */

      if (priv->msgc == 0)
        {
          /* Send stop bit */

          gd32_i2c_sendstop(priv);

          priv->dcnt--;
          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_STOP, priv->dcnt);
        }
      else if (priv->msgc > 0 &&
               (priv->msgv->flags == 0 ||
                (priv->msgv[0].flags & I2C_M_READ) != 0))
        {
          /* Send restart */

          gd32_i2c_sendstart(priv);
          gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);


          priv->dcnt--;
          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_RESTART, priv->dcnt);
        }
      else
        {
          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_FLAG_ERROR,
                              priv->msgv->flags);
        }

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);
    }
  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & (I2C_STAT0_ADDSEND | I2C_STAT0_TBE)) != 0 &&
           priv->dcnt != 0)
    {
      /* Write mode - transmit data */

      /* Clear ADDR flag */

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

      priv->check_addr_ack = false;

#ifndef CONFIG_I2C_POLLED
      if (priv->dcnt == 1 &&
          (priv->msgc == 0 || (priv->msgv->flags & I2C_M_NOSTART) == 0))
        {
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, I2C_CTL1_BUFIE, 0);
        }
#endif

      /* Transmit byte */

      gd32_i2c_putreg(priv, GD32_I2C_DATA_OFFSET, *priv->ptr++);
      gd32_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
      priv->dcnt--;

      if ((status & I2C_STAT0_ADDSEND) != 0 && priv->dcnt > 0)
        {
          /* Send one more byte */

          gd32_i2c_putreg(priv, GD32_I2C_DATA_OFFSET, *priv->ptr++);
          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
          priv->dcnt--;
        }

#ifndef CONFIG_I2C_POLLED
      if (((status & I2C_STAT0_ADDSEND) != 0 && priv->dcnt > 0) ||
          (priv->msgc > 0 && (priv->msgv->flags & I2C_M_NOSTART) != 0))
        {
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, 0, I2C_CTL1_BUFIE);
        }
#endif

      if (priv->dcnt == 0 &&
          priv->msgc > 0 && (priv->msgv->flags & I2C_M_NOSTART) != 0)
        {
          priv->dcnt = -1;
          gd32_i2c_traceevent(priv, I2CEVENT_WRITE_NO_RESTART, priv->dcnt);
        }
    }

  else if ((priv->flags & I2C_M_READ) != 0 &&
           (status & (I2C_STAT0_RBNE | I2C_STAT0_BTC)) != 0)
    {
      /* Read mode */

      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

#ifndef CONFIG_I2C_POLLED
      if (priv->dcnt < 5)
        {
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL1_OFFSET, I2C_CTL1_BUFIE, 0);
        }

      /*  BTF: N-2/N-1, set NACK, read N-2 */

      if (priv->dcnt == 3)
        {
          gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                             I2C_CTL0_ACKEN, 0);
        }

      /*  BTF: N-1/N, STOP/START, read N-1, N */

      else if (priv->dcnt == 2)
        {
          if (priv->msgc > 0)
            {
              gd32_i2c_sendstart(priv);
            }
          else
            {
              gd32_i2c_sendstop(priv);
            }

          /* Read byte N-1 */

          *priv->ptr++ = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
          priv->dcnt--;
        }

      /* Read last or current byte */

      *priv->ptr++ = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
      priv->dcnt--;

      if (priv->dcnt == 0)
        {
          priv->dcnt = -1;
        }
#else
      /* Polled mode: only read DATA when RBNE is asserted.
       * Handle the final 3/2 bytes only when BTC is asserted (BTC implies
       * 2 bytes are available: one in DATA, one in shift).
       */

      if (priv->dcnt > 3)
        {
          if ((status & I2C_STAT0_RBNE) != 0)
            {
              uint8_t byte = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
              *priv->ptr++ = byte;
              priv->dcnt--;

              if (priv->dcnt == 0)
                {
                  priv->dcnt = -1;
                }
            }
        }
      else if (priv->dcnt == 3)
        {
          if ((status & (I2C_STAT0_BTC | I2C_STAT0_RBNE)) ==
              (I2C_STAT0_BTC | I2C_STAT0_RBNE))
            {
              /*  BTF: N-2/N-1, set NACK, read N-2 */

              gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET,
                                 I2C_CTL0_ACKEN, 0);

              uint8_t byte = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
              *priv->ptr++ = byte;
              priv->dcnt--;
            }
        }
      else if (priv->dcnt == 2)
        {
          if ((status & (I2C_STAT0_BTC | I2C_STAT0_RBNE)) ==
              (I2C_STAT0_BTC | I2C_STAT0_RBNE))
            {
              /*  BTF: N-1/N, STOP/START, read N-1, N */

              if (priv->msgc > 0)
                {
                  gd32_i2c_sendstart(priv);
                }
              else
                {
                  gd32_i2c_sendstop(priv);
                }

              uint8_t byte1 = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
              *priv->ptr++ = byte1;
              priv->dcnt--;

              uint8_t byte2 = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
              *priv->ptr++ = byte2;
              priv->dcnt--;

              if (priv->dcnt == 0)
                {
                  priv->dcnt = -1;
                }
            }
        }
      else if (priv->dcnt == 1)
        {
          if ((status & I2C_STAT0_RBNE) != 0)
            {
              uint8_t byte = gd32_i2c_getreg(priv, GD32_I2C_DATA_OFFSET);
              *priv->ptr++ = byte;
              priv->dcnt--;

              if (priv->dcnt == 0)
                {
                  priv->dcnt = -1;
                }
            }
        }

#endif
    }

  /* Empty call handler */

  else if (priv->dcnt == -1 && priv->msgc == 0)
    {
      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);
      gd32_i2c_traceevent(priv, I2CEVENT_ISR_EMPTY_CALL, 0);
    }

  /* Error handler */

  else
    {
#ifdef CONFIG_I2C_POLLED
      gd32_i2c_traceevent(priv, I2CEVENT_POLL_NOT_READY, 0);
#else
      status |= (gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET) << 16);

      if ((status & I2C_STAT0_ERROR_MASK) != 0)
        {
          priv->status |= I2C_STAT0_BERR;
        }


      priv->dcnt = -1;
      priv->msgc = 0;
      gd32_i2c_traceevent(priv, I2CEVENT_STATE_ERROR, 0);
#endif
    }

  /* Check for errors */

  if ((status & I2C_STAT0_ERROR_MASK) != 0)
    {
      gd32_i2c_traceevent(priv, I2CEVENT_ERROR,
                          status & I2C_STAT0_ERROR_MASK);

#if !defined(CONFIG_I2C_POLLED)
state_error:
#endif
      gd32_i2c_putreg(priv, GD32_I2C_STAT0_OFFSET, 0);

      priv->dcnt = -1;
      priv->msgc = 0;
    }

  /* Messages handling (2/2) - Shutdown ISR */

  if (priv->dcnt == -1 && priv->msgc == 0)
    {
      gd32_i2c_traceevent(priv, I2CEVENT_ISR_SHUTDOWN, 0);

      priv->msgv = NULL;

#ifdef CONFIG_I2C_POLLED
      priv->intstate = INTSTATE_DONE;
#else
      /* Clear all interrupts */

      regval  = gd32_i2c_getreg(priv, GD32_I2C_CTL1_OFFSET);
      regval &= ~I2C_CTL1_INTS_MASK;
      gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);

      if (priv->intstate == INTSTATE_WAITING)
        {
          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int gd32_i2c_isr(int irq, void *context, void *arg)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return gd32_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: gd32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int gd32_i2c_init(struct gd32_i2c_priv_s *priv)
{
  uint32_t regval;


  /* Enable AFIO clock (required for pin remapping) */

  modifyreg32(GD32_RCU_APB2EN, 0, RCU_APB2EN_AFEN);

  /* Enable I2C clock */

  gd32_i2c_clock_enable(priv->config->i2c_base);

  /* Verify clock is enabled */

  regval = getreg32(GD32_RCU_APB1EN);

  /* Reset I2C peripheral */

  gd32_i2c_periph_reset(priv->config->i2c_base);

  /* Note: I2C0 default pins are PB6(SCL)/PB7(SDA), no AFIO remap needed.
   * Only set I2C0_REMAP if using PB8(SCL)/PB9(SDA).
   * Since we use PB6/PB7, do NOT set AFIO_PCF0_I2C0_REMAP!
   */

#ifdef CONFIG_GD32E11X_I2C0
  if (priv->config->i2c_base == GD32_I2C0_BASE)
    {
      /* Clear I2C0_REMAP to use default PB6/PB7 pins */
      
      regval = getreg32(GD32_AFIO_PCF0);
      regval &= ~AFIO_PCF0_I2C0_REMAP;  /* Clear remap bit for PB6/PB7 */
      putreg32(regval, GD32_AFIO_PCF0);
    }
#endif

  /* Configure GPIO pins */

  if (gd32_gpio_config(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (gd32_gpio_config(priv->config->sda_pin) < 0)
    {
      gd32_gpio_unconfig(priv->config->scl_pin);
      return ERROR;
    }

#ifndef CONFIG_I2C_POLLED
  /* Attach interrupt handlers */

  irq_attach(priv->config->event_irq, gd32_i2c_isr, priv);
  irq_attach(priv->config->error_irq, gd32_i2c_isr, priv);

  /* Enable interrupts */

  up_enable_irq(priv->config->event_irq);
  up_enable_irq(priv->config->error_irq);
#endif

  /* Set peripheral clock frequency in CTL1 */

  regval = GD32_PCLK1_FREQUENCY / 1000000;
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);

  /* Force frequency to be set on first transfer */

  priv->frequency = 0;

  /* Set initial clock to 100kHz */

  gd32_i2c_setclock(priv, 100000);

  /* Enable I2C first */

  gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_I2CEN);
  up_udelay(10);
  
  /* Check if bus is already busy (stuck from previous session) */
  
  regval = gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET);
  if ((regval & I2C_STAT1_I2CBSY) != 0)
    {
      
      /* Software reset I2C peripheral (SRESET bit) - this clears all registers */
      
      gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_SRESET);
      up_udelay(10);
      
      /* Clear SRESET */
      
      gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, 0);
      up_udelay(10);
      
      /* Re-configure CTL1 (peripheral clock) - cleared by SRESET */
      
      regval = GD32_PCLK1_FREQUENCY / 1000000;
      gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);
      
      /* Re-configure clock - cleared by SRESET */
      
      gd32_i2c_setclock(priv, 100000);
      
      /* Re-enable I2C */
      
      gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_I2CEN);
      up_udelay(10);
      
      /* Verify bus is now idle */
      
      regval = gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET);
              
      if ((regval & I2C_STAT1_I2CBSY) != 0)
        {
          /* Software reset failed, need GPIO-level bus recovery */
          
          
          /* Disable I2C before GPIO manipulation */
          
          gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, 0);
          
          /* Reconfigure pins as GPIO open-drain outputs */
          
          uint32_t scl_gpio = (priv->config->scl_pin & ~(GPIO_CFG_MODE_MASK | GPIO_CFG_CTL_MASK)) |
                              GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTOD | 
                              GPIO_CFG_SPEED_50MHZ | GPIO_CFG_OUTPUT_SET;
          uint32_t sda_gpio = (priv->config->sda_pin & ~(GPIO_CFG_MODE_MASK | GPIO_CFG_CTL_MASK)) |
                              GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTOD | 
                              GPIO_CFG_SPEED_50MHZ | GPIO_CFG_OUTPUT_SET;
          
          gd32_gpio_config(scl_gpio);
          gd32_gpio_config(sda_gpio);
          
          /* Read initial pin states */
          
          bool scl_state = gd32_gpio_read(scl_gpio);
          bool sda_state = gd32_gpio_read(sda_gpio);
          
          /* If either line is stuck low, this indicates hardware problem */
          
          if (!scl_state || !sda_state)
            {
              /* Hardware problem detected */
            }
          
          /* Generate 9 clock pulses to release stuck slave */
          
          for (int i = 0; i < 9; i++)
            {
              gd32_gpio_write(scl_gpio, 0);
              up_udelay(10);
              gd32_gpio_write(scl_gpio, 1);
              up_udelay(10);
            }
          
          /* Generate STOP condition (SDA low->high while SCL high) */
          
          gd32_gpio_write(sda_gpio, 0);
          up_udelay(10);
          gd32_gpio_write(scl_gpio, 0);
          up_udelay(10);
          gd32_gpio_write(scl_gpio, 1);
          up_udelay(10);
          gd32_gpio_write(sda_gpio, 1);
          up_udelay(10);
          
          /* Check final pin states */
          
          scl_state = gd32_gpio_read(scl_gpio);
          sda_state = gd32_gpio_read(sda_gpio);
          
          /* Restore GPIO as I2C alternate function */
          
          gd32_gpio_config(priv->config->scl_pin);
          gd32_gpio_config(priv->config->sda_pin);
          
          /* Re-configure and enable I2C */
          
          regval = GD32_PCLK1_FREQUENCY / 1000000;
          gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, regval);
          gd32_i2c_setclock(priv, 100000);
          gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, I2C_CTL0_I2CEN);
          up_udelay(10);
          
          regval = gd32_i2c_getreg(priv, GD32_I2C_STAT1_OFFSET);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int gd32_i2c_deinit(struct gd32_i2c_priv_s *priv)
{
  gd32_i2c_putreg(priv, GD32_I2C_CTL0_OFFSET, 0);
  gd32_i2c_putreg(priv, GD32_I2C_CTL1_OFFSET, 0);

  gd32_gpio_unconfig(priv->config->scl_pin);
  gd32_gpio_unconfig(priv->config->sda_pin);

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->event_irq);
  up_disable_irq(priv->config->error_irq);
  irq_detach(priv->config->event_irq);
  irq_detach(priv->config->error_irq);
#endif

  gd32_i2c_clock_disable(priv->config->i2c_base);
  return OK;
}

/****************************************************************************
 * Name: gd32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int gd32_i2c_transfer(struct i2c_master_s *dev,
                            struct i2c_msg_s *msgs, int count)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  uint32_t status = 0;
  int ret;

  DEBUGASSERT(count);

  /* Get exclusive access to the I2C bus */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for any STOP condition to complete */

  gd32_i2c_sem_waitstop(priv);

  /* Clear error status */

  gd32_i2c_putreg(priv, GD32_I2C_STAT0_OFFSET, 0);

  /* Clear any pending start condition */

  gd32_i2c_clrstart(priv);

  /* Initialize message pointers */

  priv->dcnt = 0;
  priv->ptr = NULL;
  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset trace */

  gd32_i2c_tracereset(priv);

  /* Set I2C clock frequency */

  gd32_i2c_setclock(priv, msgs->frequency);

  /* Enable ACK */

  gd32_i2c_modifyreg(priv, GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_ACKEN);

  /* Set state to wait for first message */

  priv->dcnt   = -1;
  priv->status = 0;
  priv->check_addr_ack = true;

  /* Send start condition */

  gd32_i2c_sendstart(priv);

  /* Wait for transfer complete */

  if (gd32_i2c_sem_waitdone(priv) < 0)
    {
      status = gd32_i2c_getstatus(priv);
      ret = -ETIMEDOUT;

      /* Clear any pending start/stop */

      gd32_i2c_clrstart(priv);

      status = priv->status & 0xffff;
    }
  else
    {
      /* Transfer completed successfully */

      status = priv->status & 0xffff;
      ret = OK;
    }

  /* Check for error status */

  if ((status & I2C_STAT0_ERROR_MASK) != 0)
    {
      if (status & I2C_STAT0_BERR)
        {
          ret = -EIO;
        }
      else if (status & I2C_STAT0_LOSTARB)
        {
          ret = -EAGAIN;
        }
      else if (status & I2C_STAT0_AERR)
        {
          ret = -ENXIO;
        }
      else if (status & I2C_STAT0_OUERR)
        {
          ret = -EIO;
        }
    }
  else if ((status & (I2C_STAT1_I2CBSY << 16)) != 0)
    {
      /* Bus is still busy */

      ret = -EBUSY;
    }

  /* Dump trace */

  gd32_i2c_tracedump(priv);

  /* Ensure ISR can't overwrite user data */

  priv->dcnt = 0;
  priv->ptr = NULL;

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int gd32_i2c_reset(struct i2c_master_s *dev)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  gd32_i2c_deinit(priv);

  /* Use GPIO to un-wedge the bus */

  scl_gpio = (priv->config->scl_pin & ~(GPIO_CFG_MODE_MASK | GPIO_CFG_CTL_MASK)) |
             GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTOD | 
             GPIO_CFG_SPEED_50MHZ | GPIO_CFG_OUTPUT_SET;
  sda_gpio = (priv->config->sda_pin & ~(GPIO_CFG_MODE_MASK | GPIO_CFG_CTL_MASK)) |
             GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTOD | 
             GPIO_CFG_SPEED_50MHZ | GPIO_CFG_OUTPUT_SET;

  gd32_gpio_config(scl_gpio);
  gd32_gpio_config(sda_gpio);

  /* Let SDA go high */

  gd32_gpio_write(sda_gpio, 1);

  /* Clock the bus until any slaves release it */

  clock_count = 0;
  while (!gd32_gpio_read(sda_gpio))
    {
      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Check for clock stretching */

      stretch_count = 0;
      while (!gd32_gpio_read(scl_gpio))
        {
          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      gd32_gpio_write(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high */

      gd32_gpio_write(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate start followed by stop */

  gd32_gpio_write(sda_gpio, 0);
  up_udelay(10);
  gd32_gpio_write(scl_gpio, 0);
  up_udelay(10);
  gd32_gpio_write(scl_gpio, 1);
  up_udelay(10);
  gd32_gpio_write(sda_gpio, 1);
  up_udelay(10);

  /* Revert GPIO configuration */

  gd32_gpio_unconfig(sda_gpio);
  gd32_gpio_unconfig(scl_gpio);

  /* Re-init the port */

  gd32_i2c_init(priv);

  /* Restore frequency */

  gd32_i2c_setclock(priv, frequency);
  ret = OK;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *gd32_i2cbus_initialize(int port)
{
  struct gd32_i2c_priv_s * priv = NULL;

#if GD32_PCLK1_FREQUENCY < 2000000
#   warning GD32_I2C_INIT: Peripheral clock must be at least 2 MHz
  return NULL;
#endif

  switch (port)
    {
    case 0:
      priv = (struct gd32_i2c_priv_s *)&gd32_i2c0_priv;
      break;
#ifdef CONFIG_GD32E11X_I2C1
    case 1:
      priv = (struct gd32_i2c_priv_s *)&gd32_i2c1_priv;
      break;
#endif
    default:
      return NULL;
    }

  nxmutex_lock(&priv->lock);

  if (priv->refs++ == 0)
    {
      gd32_i2c_init(priv);
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: gd32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int gd32_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);

  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  gd32_i2c_deinit(priv);

  nxmutex_unlock(&priv->lock);
  return OK;
}

#endif
