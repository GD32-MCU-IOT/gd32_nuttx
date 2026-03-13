/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_timer.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32e11x.h"
#include "gd32e11x_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define tmrinfo  _info
#else
#  define tmrinfo  _none
#endif

/* Timer clock input ********************************************************/

/* When APB prescaler is > 1, timer clock is 2x APB clock */

#ifndef GD32_APB1_TIMER_CLKIN
#  define GD32_APB1_TIMER_CLKIN   (2 * GD32_PCLK1_FREQUENCY)
#endif

#ifndef GD32_APB2_TIMER_CLKIN
#  define GD32_APB2_TIMER_CLKIN   (2 * GD32_PCLK2_FREQUENCY)
#endif

/* GPIO configuration support ***********************************************/

/* Check if any timer that has channel I/O pins is enabled */

#if defined(CONFIG_GD32E11X_TIMER0)  || defined(CONFIG_GD32E11X_TIMER1)  || \
    defined(CONFIG_GD32E11X_TIMER2)  || defined(CONFIG_GD32E11X_TIMER3)  || \
    defined(CONFIG_GD32E11X_TIMER4)  || defined(CONFIG_GD32E11X_TIMER7)  || \
    defined(CONFIG_GD32E11X_TIMER8)  || defined(CONFIG_GD32E11X_TIMER9)  || \
    defined(CONFIG_GD32E11X_TIMER10) || defined(CONFIG_GD32E11X_TIMER11) || \
    defined(CONFIG_GD32E11X_TIMER12) || defined(CONFIG_GD32E11X_TIMER13)
#  define HAVE_TIMER_GPIO_CONFIG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Timer private device structure */

struct gd32_timer_priv_s
{
  const struct gd32_timer_ops_s *ops;  /* Timer operations */
  gd32_timer_mode_t mode;              /* Timer mode */
  uint32_t base;                       /* Timer base address */
  uint32_t rcuen;                      /* RCU clock enable bit */
  uint32_t rcurst;                     /* RCU reset bit */
  uint32_t rcureg;                     /* RCU enable register */
  uint32_t rcurstreg;                  /* RCU reset register */
  int irq;                             /* Update IRQ */
  uint8_t nchannels;                   /* Number of channels */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Timer helper methods */

static inline uint16_t gd32_timer_getreg16(struct gd32_timer_priv_s *priv,
                                           uint8_t offset);
static inline void gd32_timer_putreg16(struct gd32_timer_priv_s *priv,
                                       uint8_t offset, uint16_t value);
static inline void gd32_timer_modifyreg16(struct gd32_timer_priv_s *priv,
                                          uint8_t offset, uint16_t clearbits,
                                          uint16_t setbits);
static inline uint32_t gd32_timer_getreg32(struct gd32_timer_priv_s *priv,
                                           uint8_t offset);
static inline void gd32_timer_putreg32(struct gd32_timer_priv_s *priv,
                                       uint8_t offset, uint32_t value);

/* Timer operations */

static int gd32_timer_setmode(struct gd32_timer_dev_s *dev,
                              gd32_timer_mode_t mode);
static int gd32_timer_setclock(struct gd32_timer_dev_s *dev, uint32_t freq);
static uint32_t gd32_timer_getclock(struct gd32_timer_dev_s *dev);
static void gd32_timer_setperiod(struct gd32_timer_dev_s *dev,
                                 uint32_t period);
static uint32_t gd32_timer_getperiod(struct gd32_timer_dev_s *dev);
static uint32_t gd32_timer_getcounter(struct gd32_timer_dev_s *dev);
static void gd32_timer_setcounter(struct gd32_timer_dev_s *dev,
                                  uint32_t count);
static void gd32_timer_reload_counter(struct gd32_timer_dev_s *dev);
static void gd32_timer_enable(struct gd32_timer_dev_s *dev);
static void gd32_timer_disable(struct gd32_timer_dev_s *dev);
static int gd32_timer_setchannel(struct gd32_timer_dev_s *dev,
                                 uint8_t channel, gd32_timer_channel_t mode);
static int gd32_timer_setcompare(struct gd32_timer_dev_s *dev,
                                 uint8_t channel, uint32_t compare);
static int gd32_timer_getcapture(struct gd32_timer_dev_s *dev,
                                 uint8_t channel);
static int gd32_timer_setisr(struct gd32_timer_dev_s *dev, xcpt_t handler,
                             void *arg, int source);
static void gd32_timer_enableint(struct gd32_timer_dev_s *dev, int source);
static void gd32_timer_disableint(struct gd32_timer_dev_s *dev, int source);
static void gd32_timer_ackint(struct gd32_timer_dev_s *dev, int source);
static int gd32_timer_checkint(struct gd32_timer_dev_s *dev, int source);

/* GPIO configuration helper */

#ifdef HAVE_TIMER_GPIO_CONFIG
static void gd32_timer_gpio_config(uint32_t cfg, gd32_timer_channel_t mode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Timer operations */

static const struct gd32_timer_ops_s g_timer_ops =
{
  .setmode        = gd32_timer_setmode,
  .setclock       = gd32_timer_setclock,
  .getclock       = gd32_timer_getclock,
  .setperiod      = gd32_timer_setperiod,
  .getperiod      = gd32_timer_getperiod,
  .getcounter     = gd32_timer_getcounter,
  .setcounter     = gd32_timer_setcounter,
  .reload_counter = gd32_timer_reload_counter,
  .enable         = gd32_timer_enable,
  .disable        = gd32_timer_disable,
  .setchannel     = gd32_timer_setchannel,
  .setcompare     = gd32_timer_setcompare,
  .getcapture     = gd32_timer_getcapture,
  .setisr         = gd32_timer_setisr,
  .enableint      = gd32_timer_enableint,
  .disableint     = gd32_timer_disableint,
  .ackint         = gd32_timer_ackint,
  .checkint       = gd32_timer_checkint,
};

/* TIMER0 - Advanced Timer */

#ifdef CONFIG_GD32E11X_TIMER0
static struct gd32_timer_priv_s g_timer0_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER0_BASE,
  .rcuen     = RCU_APB2EN_TIMER0EN,
  .rcurst    = RCU_APB2RST_TIMER0RST,
  .rcureg    = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
  .irq       = GD32_IRQ_TIMER0_UP_TIMER9,
  .nchannels = 4,
};
#endif

/* TIMER1 - General Timer */

#ifdef CONFIG_GD32E11X_TIMER1
static struct gd32_timer_priv_s g_timer1_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER1_BASE,
  .rcuen     = RCU_APB1EN_TIMER1EN,
  .rcurst    = RCU_APB1RST_TIMER1RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER1,
  .nchannels = 4,
};
#endif

/* TIMER2 - General Timer */

#ifdef CONFIG_GD32E11X_TIMER2
static struct gd32_timer_priv_s g_timer2_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER2_BASE,
  .rcuen     = RCU_APB1EN_TIMER2EN,
  .rcurst    = RCU_APB1RST_TIMER2RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER2,
  .nchannels = 4,
};
#endif

/* TIMER3 - General Timer */

#ifdef CONFIG_GD32E11X_TIMER3
static struct gd32_timer_priv_s g_timer3_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER3_BASE,
  .rcuen     = RCU_APB1EN_TIMER3EN,
  .rcurst    = RCU_APB1RST_TIMER3RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER3,
  .nchannels = 4,
};
#endif

/* TIMER4 - General Timer */

#ifdef CONFIG_GD32E11X_TIMER4
static struct gd32_timer_priv_s g_timer4_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER4_BASE,
  .rcuen     = RCU_APB1EN_TIMER4EN,
  .rcurst    = RCU_APB1RST_TIMER4RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER4,
  .nchannels = 4,
};
#endif

/* TIMER5 - Basic Timer */

#ifdef CONFIG_GD32E11X_TIMER5
static struct gd32_timer_priv_s g_timer5_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER5_BASE,
  .rcuen     = RCU_APB1EN_TIMER5EN,
  .rcurst    = RCU_APB1RST_TIMER5RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER5,
  .nchannels = 0,  /* Basic timer has no channels */
};
#endif

/* TIMER6 - Basic Timer */

#ifdef CONFIG_GD32E11X_TIMER6
static struct gd32_timer_priv_s g_timer6_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER6_BASE,
  .rcuen     = RCU_APB1EN_TIMER6EN,
  .rcurst    = RCU_APB1RST_TIMER6RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER6,
  .nchannels = 0,  /* Basic timer has no channels */
};
#endif

/* TIMER7 - Advanced Timer */

#ifdef CONFIG_GD32E11X_TIMER7
static struct gd32_timer_priv_s g_timer7_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER7_BASE,
  .rcuen     = RCU_APB2EN_TIMER7EN,
  .rcurst    = RCU_APB2RST_TIMER7RST,
  .rcureg    = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
  .irq       = GD32_IRQ_TIMER7_UP_TIMER12,
  .nchannels = 4,
};
#endif

/* TIMER8 - General Timer (2 channels, shared IRQ with TIMER0_BRK) */

#ifdef CONFIG_GD32E11X_TIMER8
static struct gd32_timer_priv_s g_timer8_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER8_BASE,
  .rcuen     = RCU_APB2EN_TIMER8EN,
  .rcurst    = RCU_APB2RST_TIMER8RST,
  .rcureg    = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
  .irq       = GD32_IRQ_TIMER0_BRK_TIMER8,
  .nchannels = 2,
};
#endif

/* TIMER9 - General Timer (1 channel, shared IRQ with TIMER0_UP) */

#ifdef CONFIG_GD32E11X_TIMER9
static struct gd32_timer_priv_s g_timer9_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER9_BASE,
  .rcuen     = RCU_APB2EN_TIMER9EN,
  .rcurst    = RCU_APB2RST_TIMER9RST,
  .rcureg    = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
  .irq       = GD32_IRQ_TIMER0_UP_TIMER9,
  .nchannels = 1,
};
#endif

/* TIMER10 - General Timer (1 channel, shared IRQ with TIMER0_TRG_CMT) */

#ifdef CONFIG_GD32E11X_TIMER10
static struct gd32_timer_priv_s g_timer10_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER10_BASE,
  .rcuen     = RCU_APB2EN_TIMER10EN,
  .rcurst    = RCU_APB2RST_TIMER10RST,
  .rcureg    = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
  .irq       = GD32_IRQ_TIMER0_TRG_CMT_TIMER10,
  .nchannels = 1,
};
#endif

/* TIMER11 - General Timer (2 channels, shared IRQ with TIMER7_BRK) */

#ifdef CONFIG_GD32E11X_TIMER11
static struct gd32_timer_priv_s g_timer11_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER11_BASE,
  .rcuen     = RCU_APB1EN_TIMER11EN,
  .rcurst    = RCU_APB1RST_TIMER11RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER7_BRK_TIMER11,
  .nchannels = 2,
};
#endif

/* TIMER12 - General Timer (1 channel, shared IRQ with TIMER7_UP) */

#ifdef CONFIG_GD32E11X_TIMER12
static struct gd32_timer_priv_s g_timer12_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER12_BASE,
  .rcuen     = RCU_APB1EN_TIMER12EN,
  .rcurst    = RCU_APB1RST_TIMER12RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER7_UP_TIMER12,
  .nchannels = 1,
};
#endif

/* TIMER13 - General Timer (1 channel, shared IRQ with TIMER7_TRG_CMT) */

#ifdef CONFIG_GD32E11X_TIMER13
static struct gd32_timer_priv_s g_timer13_priv =
{
  .ops       = &g_timer_ops,
  .mode      = GD32_TIMER_MODE_UNUSED,
  .base      = GD32_TIMER13_BASE,
  .rcuen     = RCU_APB1EN_TIMER13EN,
  .rcurst    = RCU_APB1RST_TIMER13RST,
  .rcureg    = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
  .irq       = GD32_IRQ_TIMER7_TRG_CMT_TIMER13,
  .nchannels = 1,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_timer_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t gd32_timer_getreg16(struct gd32_timer_priv_s *priv,
                                           uint8_t offset)
{
  return getreg16(priv->base + offset);
}

/****************************************************************************
 * Name: gd32_timer_putreg16
 *
 * Description:
 *   Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_timer_putreg16(struct gd32_timer_priv_s *priv,
                                       uint8_t offset, uint16_t value)
{
  putreg16(value, priv->base + offset);
}

/****************************************************************************
 * Name: gd32_timer_modifyreg16
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_timer_modifyreg16(struct gd32_timer_priv_s *priv,
                                          uint8_t offset, uint16_t clearbits,
                                          uint16_t setbits)
{
  modifyreg16(priv->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: gd32_timer_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t gd32_timer_getreg32(struct gd32_timer_priv_s *priv,
                                           uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: gd32_timer_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void gd32_timer_putreg32(struct gd32_timer_priv_s *priv,
                                       uint8_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: gd32_timer_clock_enable
 *
 * Description:
 *   Enable timer clocks
 *
 ****************************************************************************/

static void gd32_timer_clock_enable(struct gd32_timer_priv_s *priv)
{
  modifyreg32(priv->rcureg, 0, priv->rcuen);
}

/****************************************************************************
 * Name: gd32_timer_clock_disable
 *
 * Description:
 *   Disable timer clocks
 *
 ****************************************************************************/

static void gd32_timer_clock_disable(struct gd32_timer_priv_s *priv)
{
  modifyreg32(priv->rcureg, priv->rcuen, 0);
}

/****************************************************************************
 * Name: gd32_timer_reset
 *
 * Description:
 *   Reset timer into known state
 *
 ****************************************************************************/

static void gd32_timer_reset(struct gd32_timer_priv_s *priv)
{
  /* Reset timer - setting the reset bit */

  modifyreg32(priv->rcurstreg, 0, priv->rcurst);

  /* Clear reset bit */

  modifyreg32(priv->rcurstreg, priv->rcurst, 0);
}

/****************************************************************************
 * Name: gd32_timer_setmode
 *
 * Description:
 *   Set timer mode
 *
 ****************************************************************************/

static int gd32_timer_setmode(struct gd32_timer_dev_s *dev,
                              gd32_timer_mode_t mode)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint16_t val = TIMER_CTL0_CEN | TIMER_CTL0_ARSE;

  DEBUGASSERT(priv != NULL);

  /* Decode operational modes */

  switch (mode & GD32_TIMER_MODE_MASK)
    {
      case GD32_TIMER_MODE_DISABLED:
        val = 0;
        break;

      case GD32_TIMER_COUNTER_UP:
        break;

      case GD32_TIMER_COUNTER_DOWN:
        val |= TIMER_CTL0_DIR;
        break;

      case GD32_TIMER_COUNTER_CENTER:

        /* Our default:
         * Interrupts are generated on compare, when counting down
         */

        val |= TIMER_CTL0_CAM_CENTER_DOWN;
        break;

      case GD32_TIMER_SP_MODE_SINGLE:
        val |= TIMER_CTL0_SPM;
        break;

      default:
        return -EINVAL;
    }

  /* Set timer mode */

  gd32_timer_reload_counter(dev);
  gd32_timer_putreg16(priv, GD32_TIMER_CTL0_OFFSET, val);

  /* Advanced timer requires this */

#if defined(CONFIG_GD32E11X_TIMER0) || defined(CONFIG_GD32E11X_TIMER7)
  if (priv->base == GD32_TIMER0_BASE || priv->base == GD32_TIMER7_BASE)
    {
      /* Enable main output (primary output enable) */

      gd32_timer_modifyreg16(priv, GD32_TIMER_CCHP_OFFSET, 0,
                             TIMER_CCHP_POEN);
    }
#endif

  priv->mode = mode;

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_setclock
 *
 * Description:
 *   Set the clock frequency by setting the prescaler
 *
 ****************************************************************************/

static int gd32_timer_setclock(struct gd32_timer_dev_s *dev, uint32_t freq)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint32_t prescaler;
  uint32_t timclk;

  DEBUGASSERT(priv != NULL);

  /* Get the timer clock frequency */

  if (priv->rcureg == GD32_RCU_APB1EN)
    {
      /* Timers on APB1 */

      timclk = GD32_APB1_TIMER_CLKIN;
    }
  else
    {
      /* Timers on APB2 */

      timclk = GD32_APB2_TIMER_CLKIN;
    }

  /* Calculate prescaler value */

  prescaler = (timclk / freq) - 1;

  /* Set prescaler */

  gd32_timer_putreg16(priv, GD32_TIMER_PSC_OFFSET, (uint16_t)prescaler);

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_getclock
 *
 * Description:
 *   Get the current clock frequency
 *
 ****************************************************************************/

static uint32_t gd32_timer_getclock(struct gd32_timer_dev_s *dev)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint32_t prescaler;
  uint32_t timclk;

  DEBUGASSERT(priv != NULL);

  /* Get the timer clock frequency */

  if (priv->rcureg == GD32_RCU_APB1EN)
    {
      timclk = GD32_APB1_TIMER_CLKIN;
    }
  else
    {
      timclk = GD32_APB2_TIMER_CLKIN;
    }

  /* Get prescaler value */

  prescaler = gd32_timer_getreg16(priv, GD32_TIMER_PSC_OFFSET);

  return timclk / (prescaler + 1);
}

/****************************************************************************
 * Name: gd32_timer_setperiod
 *
 * Description:
 *   Set the timer period (auto-reload value)
 *
 ****************************************************************************/

static void gd32_timer_setperiod(struct gd32_timer_dev_s *dev,
                                 uint32_t period)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  gd32_timer_putreg16(priv, GD32_TIMER_CAR_OFFSET, (uint16_t)period);
}

/****************************************************************************
 * Name: gd32_timer_getperiod
 *
 * Description:
 *   Get the timer period (auto-reload value)
 *
 ****************************************************************************/

static uint32_t gd32_timer_getperiod(struct gd32_timer_dev_s *dev)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  return gd32_timer_getreg16(priv, GD32_TIMER_CAR_OFFSET);
}

/****************************************************************************
 * Name: gd32_timer_getcounter
 *
 * Description:
 *   Get the current counter value
 *
 ****************************************************************************/

static uint32_t gd32_timer_getcounter(struct gd32_timer_dev_s *dev)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  return gd32_timer_getreg16(priv, GD32_TIMER_CNT_OFFSET);
}

/****************************************************************************
 * Name: gd32_timer_setcounter
 *
 * Description:
 *   Set the timer counter value
 *
 ****************************************************************************/

static void gd32_timer_setcounter(struct gd32_timer_dev_s *dev,
                                  uint32_t count)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  gd32_timer_putreg16(priv, GD32_TIMER_CNT_OFFSET, (uint16_t)count);
}

/****************************************************************************
 * Name: gd32_timer_reload_counter
 *
 * Description:
 *   Trigger an update event to reload the counter and prescaler immediately
 *
 ****************************************************************************/

static void gd32_timer_reload_counter(struct gd32_timer_dev_s *dev)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  gd32_timer_modifyreg16(priv, GD32_TIMER_SWEVG_OFFSET, 0,
                         TIMER_SWEVG_UPG);
}

/****************************************************************************
 * Name: gd32_timer_enable
 *
 * Description:
 *   Enable the timer counter (set CEN bit)
 *
 ****************************************************************************/

static void gd32_timer_enable(struct gd32_timer_dev_s *dev)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  gd32_timer_modifyreg16(priv, GD32_TIMER_CTL0_OFFSET, 0, TIMER_CTL0_CEN);
}

/****************************************************************************
 * Name: gd32_timer_disable
 *
 * Description:
 *   Disable the timer counter (clear CEN bit)
 *
 ****************************************************************************/

static void gd32_timer_disable(struct gd32_timer_dev_s *dev)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  gd32_timer_modifyreg16(priv, GD32_TIMER_CTL0_OFFSET, TIMER_CTL0_CEN, 0);
}

/****************************************************************************
 * Name: gd32_timer_gpio_config
 *
 * Description:
 *   Configure or unconfigure a GPIO pin for timer channel use.
 *   When a channel mode is set (non-zero), the GPIO is configured for
 *   timer alternate function. When the mode is disabled (zero), the GPIO
 *   is reset to its default input state.
 *
 *   This follows the same pattern as gd32's gd32_tim_gpioconfig().
 *
 ****************************************************************************/

#ifdef HAVE_TIMER_GPIO_CONFIG
static void gd32_timer_gpio_config(uint32_t cfg, gd32_timer_channel_t mode)
{
  if (mode != GD32_TIMER_CH_DISABLED)
    {
      gd32_gpio_config(cfg | GPIO_CFG_SPEED_50MHZ);
    }
  else
    {
      gd32_gpio_unconfig(cfg);
    }
}
#endif

/****************************************************************************
 * Name: gd32_timer_setchannel
 *
 * Description:
 *   Set the timer channel mode
 *
 ****************************************************************************/

static int gd32_timer_setchannel(struct gd32_timer_dev_s *dev,
                                 uint8_t channel, gd32_timer_channel_t mode)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint16_t chctl_val = 0;
  uint16_t chctl2_val;
  uint8_t  chctl_offset;
  uint8_t  chctl_shift;
  uint8_t  chctl2_shift;

  DEBUGASSERT(priv != NULL);

  /* Check if channel is valid */

  if (channel >= priv->nchannels)
    {
      return -EINVAL;
    }

  /* Determine register offset and bit positions */

  if (channel < 2)
    {
      chctl_offset = GD32_TIMER_CHCTL0_OFFSET;
      chctl_shift = (channel == 0) ? 0 : 8;
    }
  else
    {
      chctl_offset = GD32_TIMER_CHCTL1_OFFSET;
      chctl_shift = (channel == 2) ? 0 : 8;
    }

  chctl2_shift = channel * 4;

  /* Get current CHCTL2 value */

  chctl2_val = gd32_timer_getreg16(priv, GD32_TIMER_CHCTL2_OFFSET);

  /* Disable channel */

  chctl2_val &= ~(0x0f << chctl2_shift);
  gd32_timer_putreg16(priv, GD32_TIMER_CHCTL2_OFFSET, chctl2_val);

  /* Process mode */

  if (mode == GD32_TIMER_CH_DISABLED)
    {
      return OK;
    }

  /* Output compare modes - PWM */

  if (mode & GD32_TIMER_OC_MODE_PWM0)
    {
      /* PWM mode 0: Output is active when CNT < CHxCV */

      chctl_val = TIMER_CHCTL0_CH0COMCTL_PWM0 << chctl_shift;
      chctl_val |= TIMER_CHCTL0_CH0COMSEN << chctl_shift;  /* Enable preload */

      chctl2_val |= (1 << chctl2_shift);        /* Enable output */

      if (mode & GD32_TIMER_OC_POLARITY_LOW)
        {
          chctl2_val |= (1 << (1 + chctl2_shift));  /* Low polarity */
        }
    }
  else if (mode & GD32_TIMER_OC_MODE_PWM1)
    {
      /* PWM mode 1: Output is active when CNT > CHxCV */

      chctl_val = TIMER_CHCTL0_CH0COMCTL_PWM1 << chctl_shift;
      chctl_val |= TIMER_CHCTL0_CH0COMSEN << chctl_shift;  /* Enable preload */

      chctl2_val |= (1 << chctl2_shift);        /* Enable output */

      if (mode & GD32_TIMER_OC_POLARITY_LOW)
        {
          chctl2_val |= (1 << (1 + chctl2_shift));  /* Low polarity */
        }
    }

  /* Input capture modes */

  else if (mode & GD32_TIMER_IC_SELECTION)
    {
      /* Configure as input capture, mapped to TI directly */

      chctl_val = 0x01 << chctl_shift;  /* IC mapped to TI */

      /* Set filter */

      chctl_val |= ((mode & GD32_TIMER_IC_FILTER_MASK) >> 12) <<
                  (4 + chctl_shift);

      chctl2_val |= (1 << chctl2_shift);  /* Enable capture */

      if (mode & GD32_TIMER_IC_POLARITY_FALLING)
        {
          chctl2_val |= (1 << (1 + chctl2_shift));  /* Falling edge */
        }
      else if (mode & GD32_TIMER_IC_POLARITY_BOTH)
        {
          chctl2_val |= (1 << (1 + chctl2_shift));
          chctl2_val |= (1 << (3 + chctl2_shift));  /* Both edges */
        }
    }

  /* Update CHCTL register */

  gd32_timer_modifyreg16(priv, chctl_offset, 0xff << chctl_shift, chctl_val);

  /* Update CHCTL2 register */

  gd32_timer_putreg16(priv, GD32_TIMER_CHCTL2_OFFSET, chctl2_val);

  /* Configure GPIO pin for this channel.
   *
   * Select output or input pin configuration based on channel mode.
   * For output compare / PWM modes, use the CHxOUT pin definition.
   * For input capture modes, use the CHxIN pin definition.
   * For disabled mode, unconfigure the previously used pin.
   *
   * This follows the same approach as gd32's gd32_tim_setchannel().
   */

#ifdef HAVE_TIMER_GPIO_CONFIG
    {
      uint32_t gpio_out = 0;
      uint32_t gpio_in = 0;

    switch (priv->base)
      {
#ifdef CONFIG_GD32E11X_TIMER0
        case GD32_TIMER0_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER0_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER0_CH0OUT_0;
                gpio_in  = GPIO_TIMER0_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER0_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER0_CH1OUT_0;
                gpio_in  = GPIO_TIMER0_CH1IN_0;
                break;
#endif
#if defined(GPIO_TIMER0_CH2OUT_0)
              case 2:
                gpio_out = GPIO_TIMER0_CH2OUT_0;
                gpio_in  = GPIO_TIMER0_CH2IN_0;
                break;
#endif
#if defined(GPIO_TIMER0_CH3OUT_0)
              case 3:
                gpio_out = GPIO_TIMER0_CH3OUT_0;
                gpio_in  = GPIO_TIMER0_CH3IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER0 */

#ifdef CONFIG_GD32E11X_TIMER1
        case GD32_TIMER1_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER1_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER1_CH0OUT_0;
                gpio_in  = GPIO_TIMER1_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER1_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER1_CH1OUT_0;
                gpio_in  = GPIO_TIMER1_CH1IN_0;
                break;
#endif
#if defined(GPIO_TIMER1_CH2OUT_0)
              case 2:
                gpio_out = GPIO_TIMER1_CH2OUT_0;
                gpio_in  = GPIO_TIMER1_CH2IN_0;
                break;
#endif
#if defined(GPIO_TIMER1_CH3OUT_0)
              case 3:
                gpio_out = GPIO_TIMER1_CH3OUT_0;
                gpio_in  = GPIO_TIMER1_CH3IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER1 */

#ifdef CONFIG_GD32E11X_TIMER2
        case GD32_TIMER2_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER2_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER2_CH0OUT_0;
                gpio_in  = GPIO_TIMER2_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER2_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER2_CH1OUT_0;
                gpio_in  = GPIO_TIMER2_CH1IN_0;
                break;
#endif
#if defined(GPIO_TIMER2_CH2OUT_0)
              case 2:
                gpio_out = GPIO_TIMER2_CH2OUT_0;
                gpio_in  = GPIO_TIMER2_CH2IN_0;
                break;
#endif
#if defined(GPIO_TIMER2_CH3OUT_0)
              case 3:
                gpio_out = GPIO_TIMER2_CH3OUT_0;
                gpio_in  = GPIO_TIMER2_CH3IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER2 */

#ifdef CONFIG_GD32E11X_TIMER3
        case GD32_TIMER3_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER3_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER3_CH0OUT_0;
                gpio_in  = GPIO_TIMER3_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER3_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER3_CH1OUT_0;
                gpio_in  = GPIO_TIMER3_CH1IN_0;
                break;
#endif
#if defined(GPIO_TIMER3_CH2OUT_0)
              case 2:
                gpio_out = GPIO_TIMER3_CH2OUT_0;
                gpio_in  = GPIO_TIMER3_CH2IN_0;
                break;
#endif
#if defined(GPIO_TIMER3_CH3OUT_0)
              case 3:
                gpio_out = GPIO_TIMER3_CH3OUT_0;
                gpio_in  = GPIO_TIMER3_CH3IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER3 */

#ifdef CONFIG_GD32E11X_TIMER4
        case GD32_TIMER4_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER4_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER4_CH0OUT_0;
                gpio_in  = GPIO_TIMER4_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER4_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER4_CH1OUT_0;
                gpio_in  = GPIO_TIMER4_CH1IN_0;
                break;
#endif
#if defined(GPIO_TIMER4_CH2OUT_0)
              case 2:
                gpio_out = GPIO_TIMER4_CH2OUT_0;
                gpio_in  = GPIO_TIMER4_CH2IN_0;
                break;
#endif
#if defined(GPIO_TIMER4_CH3OUT_0)
              case 3:
                gpio_out = GPIO_TIMER4_CH3OUT_0;
                gpio_in  = GPIO_TIMER4_CH3IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER4 */

#ifdef CONFIG_GD32E11X_TIMER7
        case GD32_TIMER7_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER7_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER7_CH0OUT_0;
                gpio_in  = GPIO_TIMER7_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER7_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER7_CH1OUT_0;
                gpio_in  = GPIO_TIMER7_CH1IN_0;
                break;
#endif
#if defined(GPIO_TIMER7_CH2OUT_0)
              case 2:
                gpio_out = GPIO_TIMER7_CH2OUT_0;
                gpio_in  = GPIO_TIMER7_CH2IN_0;
                break;
#endif
#if defined(GPIO_TIMER7_CH3OUT_0)
              case 3:
                gpio_out = GPIO_TIMER7_CH3OUT_0;
                gpio_in  = GPIO_TIMER7_CH3IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER7 */

#ifdef CONFIG_GD32E11X_TIMER8
        case GD32_TIMER8_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER8_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER8_CH0OUT_0;
                gpio_in  = GPIO_TIMER8_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER8_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER8_CH1OUT_0;
                gpio_in  = GPIO_TIMER8_CH1IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER8 */

#ifdef CONFIG_GD32E11X_TIMER9
        case GD32_TIMER9_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER9_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER9_CH0OUT_0;
                gpio_in  = GPIO_TIMER9_CH0IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER9 */

#ifdef CONFIG_GD32E11X_TIMER10
        case GD32_TIMER10_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER10_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER10_CH0OUT_0;
                gpio_in  = GPIO_TIMER10_CH0IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER10 */

#ifdef CONFIG_GD32E11X_TIMER11
        case GD32_TIMER11_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER11_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER11_CH0OUT_0;
                gpio_in  = GPIO_TIMER11_CH0IN_0;
                break;
#endif
#if defined(GPIO_TIMER11_CH1OUT_0)
              case 1:
                gpio_out = GPIO_TIMER11_CH1OUT_0;
                gpio_in  = GPIO_TIMER11_CH1IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER11 */

#ifdef CONFIG_GD32E11X_TIMER12
        case GD32_TIMER12_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER12_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER12_CH0OUT_0;
                gpio_in  = GPIO_TIMER12_CH0IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER12 */

#ifdef CONFIG_GD32E11X_TIMER13
        case GD32_TIMER13_BASE:
          switch (channel)
            {
#if defined(GPIO_TIMER13_CH0OUT_0)
              case 0:
                gpio_out = GPIO_TIMER13_CH0OUT_0;
                gpio_in  = GPIO_TIMER13_CH0IN_0;
                break;
#endif
              default:
                break;
            }
          break;
#endif /* CONFIG_GD32E11X_TIMER13 */

        default:
          break;
      }

    /* Apply GPIO configuration based on channel mode */

    if (mode & (GD32_TIMER_IC_SELECTION | GD32_TIMER_IC_PWM))
      {
        /* Input capture mode - configure as input */

        if (gpio_in != 0)
          {
            gd32_timer_gpio_config(gpio_in, mode);
          }
      }
    else
      {
        /* Output compare / PWM / disabled mode */

        if (gpio_out != 0)
          {
            gd32_timer_gpio_config(gpio_out, mode);
          }
      }
  }
#endif /* HAVE_TIMER_GPIO_CONFIG */

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_setcompare
 *
 * Description:
 *   Set the capture/compare value for a channel
 *
 ****************************************************************************/

static int gd32_timer_setcompare(struct gd32_timer_dev_s *dev,
                                 uint8_t channel, uint32_t compare)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  if (channel >= priv->nchannels)
    {
      return -EINVAL;
    }

  /* Write to the appropriate CHxCV register */

  switch (channel)
    {
      case GD32_TIMER_CH_0:
        gd32_timer_putreg16(priv, GD32_TIMER_CH0CV_OFFSET,
                            (uint16_t)compare);
        break;

      case GD32_TIMER_CH_1:
        gd32_timer_putreg16(priv, GD32_TIMER_CH1CV_OFFSET,
                            (uint16_t)compare);
        break;

      case GD32_TIMER_CH_2:
        gd32_timer_putreg16(priv, GD32_TIMER_CH2CV_OFFSET,
                            (uint16_t)compare);
        break;

      case GD32_TIMER_CH_3:
        gd32_timer_putreg16(priv, GD32_TIMER_CH3CV_OFFSET,
                            (uint16_t)compare);
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_getcapture
 *
 * Description:
 *   Get the capture/compare value for a channel
 *
 ****************************************************************************/

static int gd32_timer_getcapture(struct gd32_timer_dev_s *dev,
                                 uint8_t channel)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  if (channel >= priv->nchannels)
    {
      return -EINVAL;
    }

  /* Read from the appropriate CHxCV register */

  switch (channel)
    {
      case GD32_TIMER_CH_0:
        return gd32_timer_getreg16(priv, GD32_TIMER_CH0CV_OFFSET);

      case GD32_TIMER_CH_1:
        return gd32_timer_getreg16(priv, GD32_TIMER_CH1CV_OFFSET);

      case GD32_TIMER_CH_2:
        return gd32_timer_getreg16(priv, GD32_TIMER_CH2CV_OFFSET);

      case GD32_TIMER_CH_3:
        return gd32_timer_getreg16(priv, GD32_TIMER_CH3CV_OFFSET);

      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: gd32_timer_setisr
 *
 * Description:
 *   Set the interrupt handler
 *
 ****************************************************************************/

static int gd32_timer_setisr(struct gd32_timer_dev_s *dev, xcpt_t handler,
                             void *arg, int source)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  int irq;

  DEBUGASSERT(priv != NULL);

  /* Get the IRQ number based on the interrupt source.
   * Note: Some timers share IRQ vectors:
   *   TIMER0_UP / TIMER9
   *   TIMER0_BRK / TIMER8
   *   TIMER0_TRG_CMT / TIMER10
   *   TIMER7_UP / TIMER12
   *   TIMER7_BRK / TIMER11
   *   TIMER7_TRG_CMT / TIMER13
   * When using shared IRQs, the ISR must check which timer actually
   * triggered the interrupt (via INTF register flags).
   */

  switch (source)
    {
      case GD32_TIMER_INT_UP:
      case GD32_TIMER_INT_CH0:
      case GD32_TIMER_INT_CH1:
      case GD32_TIMER_INT_CH2:
      case GD32_TIMER_INT_CH3:
      case GD32_TIMER_INT_CMT:
      case GD32_TIMER_INT_TRG:
      case GD32_TIMER_INT_BRK:
        irq = priv->irq;
        break;

      default:
        return -EINVAL;
    }

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(irq);
      irq_detach(irq);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(irq, handler, arg);
  up_enable_irq(irq);

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_enableint
 *
 * Description:
 *   Enable a timer interrupt
 *
 ****************************************************************************/

static void gd32_timer_enableint(struct gd32_timer_dev_s *dev, int source)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint16_t setbits = 0;

  DEBUGASSERT(priv != NULL);

  switch (source)
    {
      case GD32_TIMER_INT_UP:
        setbits = TIMER_DMAINTEN_UPIE;
        break;

      case GD32_TIMER_INT_CH0:
        setbits = TIMER_DMAINTEN_CH0IE;
        break;

      case GD32_TIMER_INT_CH1:
        setbits = TIMER_DMAINTEN_CH1IE;
        break;

      case GD32_TIMER_INT_CH2:
        setbits = TIMER_DMAINTEN_CH2IE;
        break;

      case GD32_TIMER_INT_CH3:
        setbits = TIMER_DMAINTEN_CH3IE;
        break;

      case GD32_TIMER_INT_CMT:
        setbits = TIMER_DMAINTEN_CMTIE;
        break;

      case GD32_TIMER_INT_TRG:
        setbits = TIMER_DMAINTEN_TRGIE;
        break;

      case GD32_TIMER_INT_BRK:
        setbits = TIMER_DMAINTEN_BRKIE;
        break;

      default:
        return;
    }

  gd32_timer_modifyreg16(priv, GD32_TIMER_DMAINTEN_OFFSET, 0, setbits);
}

/****************************************************************************
 * Name: gd32_timer_disableint
 *
 * Description:
 *   Disable a timer interrupt
 *
 ****************************************************************************/

static void gd32_timer_disableint(struct gd32_timer_dev_s *dev, int source)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint16_t clearbits = 0;

  DEBUGASSERT(priv != NULL);

  switch (source)
    {
      case GD32_TIMER_INT_UP:
        clearbits = TIMER_DMAINTEN_UPIE;
        break;

      case GD32_TIMER_INT_CH0:
        clearbits = TIMER_DMAINTEN_CH0IE;
        break;

      case GD32_TIMER_INT_CH1:
        clearbits = TIMER_DMAINTEN_CH1IE;
        break;

      case GD32_TIMER_INT_CH2:
        clearbits = TIMER_DMAINTEN_CH2IE;
        break;

      case GD32_TIMER_INT_CH3:
        clearbits = TIMER_DMAINTEN_CH3IE;
        break;

      case GD32_TIMER_INT_CMT:
        clearbits = TIMER_DMAINTEN_CMTIE;
        break;

      case GD32_TIMER_INT_TRG:
        clearbits = TIMER_DMAINTEN_TRGIE;
        break;

      case GD32_TIMER_INT_BRK:
        clearbits = TIMER_DMAINTEN_BRKIE;
        break;

      default:
        return;
    }

  gd32_timer_modifyreg16(priv, GD32_TIMER_DMAINTEN_OFFSET, clearbits, 0);
}

/****************************************************************************
 * Name: gd32_timer_ackint
 *
 * Description:
 *   Acknowledge (clear) a timer interrupt
 *
 ****************************************************************************/

static void gd32_timer_ackint(struct gd32_timer_dev_s *dev, int source)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint16_t clearbits = 0;

  DEBUGASSERT(priv != NULL);

  switch (source)
    {
      case GD32_TIMER_INT_UP:
        clearbits = TIMER_INTF_UPIF;
        break;

      case GD32_TIMER_INT_CH0:
        clearbits = TIMER_INTF_CH0IF;
        break;

      case GD32_TIMER_INT_CH1:
        clearbits = TIMER_INTF_CH1IF;
        break;

      case GD32_TIMER_INT_CH2:
        clearbits = TIMER_INTF_CH2IF;
        break;

      case GD32_TIMER_INT_CH3:
        clearbits = TIMER_INTF_CH3IF;
        break;

      case GD32_TIMER_INT_CMT:
        clearbits = TIMER_INTF_CMTIF;
        break;

      case GD32_TIMER_INT_TRG:
        clearbits = TIMER_INTF_TRGIF;
        break;

      case GD32_TIMER_INT_BRK:
        clearbits = TIMER_INTF_BRKIF;
        break;

      default:
        return;
    }

  gd32_timer_putreg16(priv, GD32_TIMER_INTF_OFFSET, ~clearbits & 0xffff);
}

/****************************************************************************
 * Name: gd32_timer_checkint
 *
 * Description:
 *   Check if a timer interrupt is pending
 *
 ****************************************************************************/

static int gd32_timer_checkint(struct gd32_timer_dev_s *dev, int source)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;
  uint16_t checkbits = 0;
  uint16_t regval;

  DEBUGASSERT(priv != NULL);

  switch (source)
    {
      case GD32_TIMER_INT_UP:
        checkbits = TIMER_INTF_UPIF;
        break;

      case GD32_TIMER_INT_CH0:
        checkbits = TIMER_INTF_CH0IF;
        break;

      case GD32_TIMER_INT_CH1:
        checkbits = TIMER_INTF_CH1IF;
        break;

      case GD32_TIMER_INT_CH2:
        checkbits = TIMER_INTF_CH2IF;
        break;

      case GD32_TIMER_INT_CH3:
        checkbits = TIMER_INTF_CH3IF;
        break;

      case GD32_TIMER_INT_CMT:
        checkbits = TIMER_INTF_CMTIF;
        break;

      case GD32_TIMER_INT_TRG:
        checkbits = TIMER_INTF_TRGIF;
        break;

      case GD32_TIMER_INT_BRK:
        checkbits = TIMER_INTF_BRKIF;
        break;

      default:
        return -EINVAL;
    }

  regval = gd32_timer_getreg16(priv, GD32_TIMER_INTF_OFFSET);

  return (regval & checkbits) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_timer_init
 *
 * Description:
 *   Initialize a timer device
 *
 ****************************************************************************/

struct gd32_timer_dev_s *gd32_timer_init(int timer)
{
  struct gd32_timer_priv_s *priv;

  /* Get timer device structure */

  switch (timer)
    {
#ifdef CONFIG_GD32E11X_TIMER0
      case 0:
        priv = &g_timer0_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER1
      case 1:
        priv = &g_timer1_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER2
      case 2:
        priv = &g_timer2_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER3
      case 3:
        priv = &g_timer3_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER4
      case 4:
        priv = &g_timer4_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER5
      case 5:
        priv = &g_timer5_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER6
      case 6:
        priv = &g_timer6_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER7
      case 7:
        priv = &g_timer7_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER8
      case 8:
        priv = &g_timer8_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER9
      case 9:
        priv = &g_timer9_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER10
      case 10:
        priv = &g_timer10_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER11
      case 11:
        priv = &g_timer11_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER12
      case 12:
        priv = &g_timer12_priv;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER13
      case 13:
        priv = &g_timer13_priv;
        break;
#endif

      default:
        return NULL;
    }

  /* Check if timer is already in use */

  if (priv->mode != GD32_TIMER_MODE_UNUSED)
    {
      return NULL;
    }

  /* Enable peripheral clock */

  gd32_timer_clock_enable(priv);

  /* Reset timer to default state */

  gd32_timer_reset(priv);

  return (struct gd32_timer_dev_s *)priv;
}

/****************************************************************************
 * Name: gd32_timer_deinit
 *
 * Description:
 *   Deinitialize a timer device
 *
 ****************************************************************************/

int gd32_timer_deinit(struct gd32_timer_dev_s *dev)
{
  struct gd32_timer_priv_s *priv = (struct gd32_timer_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  /* Disable timer */

  gd32_timer_setmode(dev, GD32_TIMER_MODE_DISABLED);

  /* Disable all interrupts */

  gd32_timer_putreg16(priv, GD32_TIMER_DMAINTEN_OFFSET, 0);

  /* Disable peripheral clock */

  gd32_timer_clock_disable(priv);

  /* Mark timer as unused */

  priv->mode = GD32_TIMER_MODE_UNUSED;

  return OK;
}
