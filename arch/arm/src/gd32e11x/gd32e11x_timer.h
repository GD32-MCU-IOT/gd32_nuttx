/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_timer.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_TIMER_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"
#include "hardware/gd32e11x_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define GD32_TIMER_SETMODE(d, mode)       ((d)->ops->setmode(d, mode))
#define GD32_TIMER_SETCLOCK(d, freq)      ((d)->ops->setclock(d, freq))
#define GD32_TIMER_GETCLOCK(d)            ((d)->ops->getclock(d))
#define GD32_TIMER_SETPERIOD(d, period)   ((d)->ops->setperiod(d, period))
#define GD32_TIMER_GETPERIOD(d)           ((d)->ops->getperiod(d))
#define GD32_TIMER_GETCOUNTER(d)          ((d)->ops->getcounter(d))
#define GD32_TIMER_SETCOUNTER(d, cnt)      ((d)->ops->setcounter(d, cnt))
#define GD32_TIMER_RELOAD_COUNTER(d)       ((d)->ops->reload_counter(d))
#define GD32_TIMER_ENABLE(d)              ((d)->ops->enable(d))
#define GD32_TIMER_DISABLE(d)             ((d)->ops->disable(d))
#define GD32_TIMER_SETCHANNEL(d, ch, mode) ((d)->ops->setchannel(d, ch, mode))
#define GD32_TIMER_SETCOMPARE(d, ch, comp) ((d)->ops->setcompare(d, ch, comp))
#define GD32_TIMER_GETCAPTURE(d, ch)       ((d)->ops->getcapture(d, ch))
#define GD32_TIMER_SETISR(d, hnd, arg, s) ((d)->ops->setisr(d, hnd, arg, s))
#define GD32_TIMER_ENABLEINT(d, s)        ((d)->ops->enableint(d, s))
#define GD32_TIMER_DISABLEINT(d, s)       ((d)->ops->disableint(d, s))
#define GD32_TIMER_ACKINT(d, s)           ((d)->ops->ackint(d, s))
#define GD32_TIMER_CHECKINT(d, s)         ((d)->ops->checkint(d, s))

/* Note: GPIO configuration is handled internally by setchannel().
 * When setchannel() is called, it automatically configures the
 * corresponding GPIO pin for the selected channel mode (output or input).
 * Disabling a channel (GD32_TIMER_CH_DISABLED) unconfigures the GPIO pin.
 * The pin mapping is determined by the remap selection aliases in pinmap.h
 * (the _0 suffix macros, which resolve based on CONFIG_GD32E11X_TIMERx_*
 * remap Kconfig options).
 */

/****************************************************************************
 * Public Types
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

/* Timer device structure */

struct gd32_timer_dev_s;

/* Timer counter modes - follows GD32 naming convention */

typedef enum
{
  GD32_TIMER_MODE_UNUSED       = -1,

  /* Counter direction/alignment modes */

  GD32_TIMER_MODE_MASK         = 0x000f,
  GD32_TIMER_MODE_DISABLED     = 0x0000,
  GD32_TIMER_COUNTER_UP        = 0x0001,  /* Counter up direction */
  GD32_TIMER_COUNTER_DOWN      = 0x0002,  /* Counter down direction */
  GD32_TIMER_COUNTER_CENTER    = 0x0003,  /* Center-aligned mode */
  GD32_TIMER_SP_MODE_SINGLE    = 0x0004,  /* Single pulse mode */

  /* Timer counter clock source selection */

  GD32_TIMER_SMCFG_DISABLE     = 0x0000,  /* Clock source internal */
  GD32_TIMER_SMCFG_RESTART     = 0x0040,  /* Restart mode */
  GD32_TIMER_SMCFG_EXTERNAL0   = 0x0050,  /* External clock mode 0 */
  GD32_TIMER_SMCFG_EXTERNAL1   = 0x0060,  /* External clock mode 1 */

  /* External clock prescaler */

  GD32_TIMER_EXT_TRI_PSC_MASK  = 0x0300,
  GD32_TIMER_EXT_TRI_PSC_OFF   = 0x0000,
  GD32_TIMER_EXT_TRI_PSC_DIV2  = 0x0100,
  GD32_TIMER_EXT_TRI_PSC_DIV4  = 0x0200,
  GD32_TIMER_EXT_TRI_PSC_DIV8  = 0x0300,

  /* External clock filter */

  GD32_TIMER_EXT_FILTER_MASK   = 0xf000,
  GD32_TIMER_EXT_FILTER_0      = 0x0000,
  GD32_TIMER_EXT_FILTER_1      = 0x1000,
  GD32_TIMER_EXT_FILTER_2      = 0x2000,
  GD32_TIMER_EXT_FILTER_3      = 0x3000,
  GD32_TIMER_EXT_FILTER_4      = 0x4000,
  GD32_TIMER_EXT_FILTER_5      = 0x5000,
  GD32_TIMER_EXT_FILTER_6      = 0x6000,
  GD32_TIMER_EXT_FILTER_7      = 0x7000,
  GD32_TIMER_EXT_FILTER_8      = 0x8000,
  GD32_TIMER_EXT_FILTER_9      = 0x9000,
  GD32_TIMER_EXT_FILTER_10     = 0xa000,
  GD32_TIMER_EXT_FILTER_11     = 0xb000,
  GD32_TIMER_EXT_FILTER_12     = 0xc000,
  GD32_TIMER_EXT_FILTER_13     = 0xd000,
  GD32_TIMER_EXT_FILTER_14     = 0xe000,
  GD32_TIMER_EXT_FILTER_15     = 0xf000,
} gd32_timer_mode_t;

/* Timer channel modes - follows GD32 naming convention */

typedef enum
{
  GD32_TIMER_CH_DISABLED       = 0x00,

  /* Common configuration - polarity */

  GD32_TIMER_OC_POLARITY_HIGH  = 0x00,    /* Channel output polarity high */
  GD32_TIMER_OC_POLARITY_LOW   = 0x01,    /* Channel output polarity low */

  /* Timer channel output compare modes */

  GD32_TIMER_OC_MODE_PWM0      = 0x04,    /* PWM mode 0 */
  GD32_TIMER_OC_MODE_PWM1      = 0x08,    /* PWM mode 1 */

  /* Timer channel input capture modes */

  GD32_TIMER_IC_SELECTION      = 0x10,    /* Input capture mode */
  GD32_TIMER_IC_PWM            = 0x20,    /* Input PWM capture mode */

  /* Input capture polarity */

  GD32_TIMER_IC_POLARITY_RISING   = 0x00,    /* Rising edge */
  GD32_TIMER_IC_POLARITY_FALLING  = 0x40,    /* Falling edge */
  GD32_TIMER_IC_POLARITY_BOTH     = 0x80,    /* Both edges */

  /* Filter options for input capture */

  GD32_TIMER_IC_FILTER_MASK    = 0xf000,
  GD32_TIMER_IC_FILTER_0       = 0x0000,
  GD32_TIMER_IC_FILTER_1       = 0x1000,
  GD32_TIMER_IC_FILTER_2       = 0x2000,
  GD32_TIMER_IC_FILTER_3       = 0x3000,
  GD32_TIMER_IC_FILTER_4       = 0x4000,
  GD32_TIMER_IC_FILTER_5       = 0x5000,
  GD32_TIMER_IC_FILTER_6       = 0x6000,
  GD32_TIMER_IC_FILTER_7       = 0x7000,
  GD32_TIMER_IC_FILTER_8       = 0x8000,
  GD32_TIMER_IC_FILTER_9       = 0x9000,
  GD32_TIMER_IC_FILTER_10      = 0xa000,
  GD32_TIMER_IC_FILTER_11      = 0xb000,
  GD32_TIMER_IC_FILTER_12      = 0xc000,
  GD32_TIMER_IC_FILTER_13      = 0xd000,
  GD32_TIMER_IC_FILTER_14      = 0xe000,
  GD32_TIMER_IC_FILTER_15      = 0xf000,
} gd32_timer_channel_t;

/* Timer operations */

struct gd32_timer_ops_s
{
  /* Basic timer operations */

  int  (*setmode)(struct gd32_timer_dev_s *dev, gd32_timer_mode_t mode);
  int  (*setclock)(struct gd32_timer_dev_s *dev, uint32_t freq);
  uint32_t (*getclock)(struct gd32_timer_dev_s *dev);
  void (*setperiod)(struct gd32_timer_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(struct gd32_timer_dev_s *dev);
  uint32_t (*getcounter)(struct gd32_timer_dev_s *dev);
  void (*setcounter)(struct gd32_timer_dev_s *dev, uint32_t count);
  void (*reload_counter)(struct gd32_timer_dev_s *dev);

  /* Enable/Disable */

  void (*enable)(struct gd32_timer_dev_s *dev);
  void (*disable)(struct gd32_timer_dev_s *dev);

  /* Channel management */

  int  (*setchannel)(struct gd32_timer_dev_s *dev, uint8_t channel,
                     gd32_timer_channel_t mode);
  int  (*setcompare)(struct gd32_timer_dev_s *dev, uint8_t channel,
                     uint32_t compare);
  int  (*getcapture)(struct gd32_timer_dev_s *dev, uint8_t channel);

  /* Interrupt operations */

  int  (*setisr)(struct gd32_timer_dev_s *dev, xcpt_t handler, void *arg,
                 int source);
  void (*enableint)(struct gd32_timer_dev_s *dev, int source);
  void (*disableint)(struct gd32_timer_dev_s *dev, int source);
  void (*ackint)(struct gd32_timer_dev_s *dev, int source);
  int  (*checkint)(struct gd32_timer_dev_s *dev, int source);
};

/* Timer device structure */

struct gd32_timer_dev_s
{
  const struct gd32_timer_ops_s *ops;
};

/* Timer interrupt sources - follows GD32 naming (TIMER_INT_xxx) */

typedef enum
{
  GD32_TIMER_INT_UP   = 0,        /* Update interrupt */
  GD32_TIMER_INT_CH0  = 1,        /* Channel 0 capture/compare interrupt */
  GD32_TIMER_INT_CH1  = 2,        /* Channel 1 capture/compare interrupt */
  GD32_TIMER_INT_CH2  = 3,        /* Channel 2 capture/compare interrupt */
  GD32_TIMER_INT_CH3  = 4,        /* Channel 3 capture/compare interrupt */
  GD32_TIMER_INT_CMT  = 5,        /* Commutation interrupt (advanced timers) */
  GD32_TIMER_INT_TRG  = 6,        /* Trigger interrupt */
  GD32_TIMER_INT_BRK  = 7,        /* Break interrupt (advanced timers) */
} gd32_timer_int_t;

/* Timer channel numbers - follows GD32 naming (TIMER_CH_x) */

#define GD32_TIMER_CH_0    0      /* TIMER channel 0 */
#define GD32_TIMER_CH_1    1      /* TIMER channel 1 */
#define GD32_TIMER_CH_2    2      /* TIMER channel 2 */
#define GD32_TIMER_CH_3    3      /* TIMER channel 3 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_timer_init
 *
 * Description:
 *   Power-up the timer and get its structure.
 *
 * Input Parameters:
 *   timer - The timer number (0-13 for GD32E11X)
 *
 * Returned Value:
 *   On success, a pointer to timer device structure; NULL on failure.
 *
 ****************************************************************************/

struct gd32_timer_dev_s *gd32_timer_init(int timer);

/****************************************************************************
 * Name: gd32_timer_deinit
 *
 * Description:
 *   Power-off the timer.
 *
 * Input Parameters:
 *   dev - The timer device to deinitialize
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gd32_timer_deinit(struct gd32_timer_dev_s *dev);

/****************************************************************************
 * Name: gd32_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'.  This creates a NuttX timer
 *   character device (e.g., /dev/timer0) that can be used with the
 *   standard timer test applications (apps/examples/timer).
 *
 * Input Parameters:
 *   devpath - The full path to the timer device (e.g., /dev/timer0)
 *   timer   - The timer number (0-13 for GD32E11X)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int gd32_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_TIMER_H */
