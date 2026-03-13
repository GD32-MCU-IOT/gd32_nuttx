/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_pwm.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32e11x.h"
#include "gd32e11x_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Advanced timer support */

#if defined(CONFIG_GD32E11X_TIMER0_PWM) || defined(CONFIG_GD32E11X_TIMER7_PWM)
#  define HAVE_ADVANCED_TIMER
#endif

/* Pulsecount support */

#ifdef CONFIG_PWM_PULSECOUNT
#  ifndef HAVE_ADVANCED_TIMER
#    error "PWM_PULSECOUNT requires HAVE_ADVANCED_TIMER"
#  endif
#  if defined(CONFIG_GD32E11X_TIMER0_PWM) || \
      defined(CONFIG_GD32E11X_TIMER7_PWM)
#    define HAVE_PWM_INTERRUPT
#  endif
#endif

/* Complementary output support (defined in header, ensure available) */

#ifndef HAVE_PWM_COMPLEMENTARY
#  if defined(CONFIG_GD32E11X_TIMER0_CH0ON) || \
      defined(CONFIG_GD32E11X_TIMER0_CH1ON) || \
      defined(CONFIG_GD32E11X_TIMER0_CH2ON) || \
      defined(CONFIG_GD32E11X_TIMER7_CH0ON) || \
      defined(CONFIG_GD32E11X_TIMER7_CH1ON) || \
      defined(CONFIG_GD32E11X_TIMER7_CH2ON)
#    define HAVE_PWM_COMPLEMENTARY
#  endif
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) gd32_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PWM timer device structure - represents one timer configured for PWM */

struct gd32_pwmtimer_s
{
  const struct pwm_ops_s    *ops;        /* PWM operations */
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  const struct gd32_pwm_ops_s *llops;    /* Low-level operations */
#endif
  struct gd32_pwmchan_s     *channels;   /* Channel configurations */
  uint8_t                    timer_id;   /* Timer ID (0-13) */
  uint8_t                    chan_num;   /* Num of configured chans */
  uint8_t                    timer_type; /* Timer type (TIMERx_TYPE) */
  uint8_t                    mode;       /* Timer counting mode */
  uint8_t                    t_dts;      /* Dead-time clock div */
  uint8_t                    lock;       /* Register lock level (0-3) */
#ifdef HAVE_BREAK
  struct gd32_pwm_break_s    brk;        /* Break configuration */
#endif
#ifdef HAVE_PWM_COMPLEMENTARY
  uint8_t                    deadtime;   /* Dead-time value (CCHP.DTCFG) */
#endif
#ifdef HAVE_TRGO
  uint8_t                    trgo;       /* TRGO output source (CTL1.MMC) */
#endif
  uint32_t                   base;       /* Timer base address */
  uint32_t                   pclk;       /* Timer clock frequency */
  uint32_t                   frequency;  /* Current frequency setting */
  uint32_t                   rcuen;      /* RCU clock enable bit */
  uint32_t                   rcurst;     /* RCU reset bit */
  uint32_t                   rcureg;     /* RCU enable register */
  uint32_t                   rcurstreg;  /* RCU reset register */
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t                    irq;        /* Update IRQ */
  uint8_t                    prev;       /* The previous value of the CREP */
  uint8_t                    curr;       /* The current value of the CREP */
  uint32_t                   count;      /* Remaining pulse count */
  void                      *handle;     /* Pulse count callback handle */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t pwm_getreg(struct gd32_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct gd32_pwmtimer_s *priv, int offset,
                       uint32_t value);
static void pwm_modifyreg(struct gd32_pwmtimer_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev, const char *msg);
#else
#  define pwm_dumpregs(dev,msg)
#endif

/* Timer management */

static int pwm_frequency_update(struct pwm_lowerhalf_s *dev,
                                uint32_t frequency);
static int pwm_timer_configure(struct gd32_pwmtimer_s *priv);
static int pwm_mode_configure(struct pwm_lowerhalf_s *dev,
                              uint8_t channel, uint32_t mode);
static int pwm_output_configure(struct gd32_pwmtimer_s *priv,
                                struct gd32_pwmchan_s *chan);
static int pwm_outputs_enable(struct pwm_lowerhalf_s *dev,
                              uint16_t outputs, bool state);
static int pwm_soft_update(struct pwm_lowerhalf_s *dev);
static int pwm_soft_break(struct pwm_lowerhalf_s *dev, bool state);
static int pwm_chcv_update(struct pwm_lowerhalf_s *dev, uint8_t index,
                           uint32_t chcv);
static int pwm_car_update(struct pwm_lowerhalf_s *dev, uint32_t car);
static uint32_t pwm_car_get(struct pwm_lowerhalf_s *dev);
static int pwm_duty_update(struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty);
static int pwm_timer_enable(struct pwm_lowerhalf_s *dev, bool state);

#ifdef HAVE_ADVANCED_TIMER
static int pwm_break_dt_configure(struct gd32_pwmtimer_s *priv);
#endif
#ifdef HAVE_TRGO
static int pwm_trgo_configure(struct pwm_lowerhalf_s *dev, uint8_t trgo);
#endif
#if defined(HAVE_PWM_COMPLEMENTARY) && defined(CONFIG_GD32E11X_PWM_LL_OPS)
static int pwm_deadtime_update(struct pwm_lowerhalf_s *dev, uint8_t dt);
#endif
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
static uint32_t pwm_chcv_get(struct pwm_lowerhalf_s *dev, uint8_t index);
#endif
#ifdef HAVE_ADVANCED_TIMER
static int pwm_crep_update(struct pwm_lowerhalf_s *dev, uint16_t crep);
static uint16_t pwm_crep_get(struct pwm_lowerhalf_s *dev);
#endif
static uint16_t pwm_outputs_from_channels(struct gd32_pwmtimer_s *priv);

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_pulsecount_configure(struct pwm_lowerhalf_s *dev);
static int pwm_pulsecount_timer(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info);
#endif
static int pwm_configure(struct pwm_lowerhalf_s *dev);
static int pwm_duty_channels_update(struct pwm_lowerhalf_s *dev,
                                    const struct pwm_info_s *info);
static int pwm_timer(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);

#ifdef HAVE_PWM_INTERRUPT
static int pwm_interrupt(struct gd32_pwmtimer_s *priv);
#  ifdef CONFIG_GD32E11X_TIMER0_PWM
static int pwm_timer0_interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_GD32E11X_TIMER7_PWM
static int pwm_timer7_interrupt(int irq, void *context, void *arg);
#  endif
static uint8_t pwm_pulsecount(uint32_t count);
#endif

static int pwm_set_apb_clock(struct gd32_pwmtimer_s *priv, bool on);
static void pwm_timer_reset(struct gd32_pwmtimer_s *priv);

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start_pulsecount(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info,
                                void *handle);
#endif
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup    = pwm_setup,
  .shutdown = pwm_shutdown,
#ifdef CONFIG_PWM_PULSECOUNT
  .start    = pwm_start_pulsecount,
#else
  .start    = pwm_start,
#endif
  .stop     = pwm_stop,
  .ioctl    = pwm_ioctl,
};

#ifdef CONFIG_GD32E11X_PWM_LL_OPS
static const struct gd32_pwm_ops_s g_llpwmops =
{
  .configure       = pwm_configure,
  .soft_break      = pwm_soft_break,
  .chcv_update     = pwm_chcv_update,
  .mode_update     = pwm_mode_configure,
  .chcv_get        = pwm_chcv_get,
  .car_update      = pwm_car_update,
  .car_get         = pwm_car_get,
#ifdef HAVE_ADVANCED_TIMER
  .crep_update     = pwm_crep_update,
  .crep_get        = pwm_crep_get,
#endif
#ifdef HAVE_TRGO
  .trgo_set        = pwm_trgo_configure,
#endif
  .outputs_enable  = pwm_outputs_enable,
  .soft_update     = pwm_soft_update,
  .freq_update     = pwm_frequency_update,
  .enable          = pwm_timer_enable,
#ifdef CONFIG_DEBUG_PWM_INFO
  .dumpregs        = pwm_dumpregs,
#endif
#ifdef HAVE_PWM_COMPLEMENTARY
  .dt_update       = pwm_deadtime_update,
#endif
};
#endif /* CONFIG_GD32E11X_PWM_LL_OPS */

/* TIMER0 - Advanced Timer **************************************************/

#ifdef CONFIG_GD32E11X_TIMER0_PWM

static struct gd32_pwmchan_s g_pwm0channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER0_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER0_CH0P,
      .idle   = CONFIG_GD32E11X_TIMER0_CH0IDLE,
      .pincfg = PWM_TIMER0_CH0CFG,
    },
#ifdef HAVE_PWM_COMPLEMENTARY
    .out2    =
    {
#ifdef CONFIG_GD32E11X_TIMER0_CH0ON
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER0_CH0NP,
      .idle   = CONFIG_GD32E11X_TIMER0_CH0NIDLE,
      .pincfg = PWM_TIMER0_CH0NCFG,
#else
      .in_use = 0,
#endif
    },
#endif
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER0_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER0_CH1P,
      .idle   = CONFIG_GD32E11X_TIMER0_CH1IDLE,
      .pincfg = PWM_TIMER0_CH1CFG,
    },
#ifdef HAVE_PWM_COMPLEMENTARY
    .out2    =
    {
#ifdef CONFIG_GD32E11X_TIMER0_CH1ON
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER0_CH1NP,
      .idle   = CONFIG_GD32E11X_TIMER0_CH1NIDLE,
      .pincfg = PWM_TIMER0_CH1NCFG,
#else
      .in_use = 0,
#endif
    },
#endif
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_GD32E11X_TIMER0_CH2MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER0_CH2P,
      .idle   = CONFIG_GD32E11X_TIMER0_CH2IDLE,
      .pincfg = PWM_TIMER0_CH2CFG,
    },
#ifdef HAVE_PWM_COMPLEMENTARY
    .out2    =
    {
#ifdef CONFIG_GD32E11X_TIMER0_CH2ON
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER0_CH2NP,
      .idle   = CONFIG_GD32E11X_TIMER0_CH2NIDLE,
      .pincfg = PWM_TIMER0_CH2NCFG,
#else
      .in_use = 0,
#endif
    },
#endif
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_GD32E11X_TIMER0_CH3MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER0_CH3P,
      .idle   = CONFIG_GD32E11X_TIMER0_CH3IDLE,
      .pincfg = PWM_TIMER0_CH3CFG,
    },

    /* CH3 has no complementary output */
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm0dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm0channels,
  .timer_id   = 0,
  .chan_num   = PWM_TIMER0_NCHANNELS,
  .timer_type = GD32_TIMER0_TYPE,
  .mode       = CONFIG_GD32E11X_TIMER0_MODE,
  .t_dts      = CONFIG_GD32E11X_TIMER0_FDTS,
  .lock       = CONFIG_GD32E11X_TIMER0_LOCK,
#ifdef HAVE_BREAK
  .brk        =
  {
#ifdef CONFIG_GD32E11X_TIMER0_BREAK
    .en      = 1,
    .pol     = CONFIG_GD32E11X_TIMER0_BRKP,
#else
    .en      = 0,
    .pol     = 0,
#endif
  },
#endif
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime   = CONFIG_GD32E11X_TIMER0_DEADTIME,
#endif
#ifdef HAVE_TRGO
  .trgo       = CONFIG_GD32E11X_TIMER0_TRGO,
#endif
  .base       = GD32_TIMER0_BASE,
  .pclk       = GD32_TIMER0_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB2EN_TIMER0EN,
  .rcurst     = RCU_APB2RST_TIMER0RST,
  .rcureg     = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER0_UP_TIMER9,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER0_PWM */

/* TIMER1 - General Timer ***************************************************/

#ifdef CONFIG_GD32E11X_TIMER1_PWM

static struct gd32_pwmchan_s g_pwm1channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER1_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER1_CH0P,
      .pincfg = PWM_TIMER1_CH0CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER1_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER1_CH1P,
      .pincfg = PWM_TIMER1_CH1CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_GD32E11X_TIMER1_CH2MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER1_CH2P,
      .pincfg = PWM_TIMER1_CH2CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_GD32E11X_TIMER1_CH3MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER1_CH3P,
      .pincfg = PWM_TIMER1_CH3CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm1channels,
  .timer_id   = 1,
  .chan_num   = PWM_TIMER1_NCHANNELS,
  .timer_type = GD32_TIMER1_TYPE,
  .mode       = CONFIG_GD32E11X_TIMER1_MODE,
  .base       = GD32_TIMER1_BASE,
  .pclk       = GD32_TIMER1_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB1EN_TIMER1EN,
  .rcurst     = RCU_APB1RST_TIMER1RST,
  .rcureg     = GD32_RCU_APB1EN,
  .rcurstreg  = GD32_RCU_APB1RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER1,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER1_PWM */

/* TIMER2 - General Timer ***************************************************/

#ifdef CONFIG_GD32E11X_TIMER2_PWM

static struct gd32_pwmchan_s g_pwm2channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER2_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER2_CH0P,
      .pincfg = PWM_TIMER2_CH0CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER2_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER2_CH1P,
      .pincfg = PWM_TIMER2_CH1CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_GD32E11X_TIMER2_CH2MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER2_CH2P,
      .pincfg = PWM_TIMER2_CH2CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_GD32E11X_TIMER2_CH3MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER2_CH3P,
      .pincfg = PWM_TIMER2_CH3CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm2dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm2channels,
  .timer_id   = 2,
  .chan_num   = PWM_TIMER2_NCHANNELS,
  .timer_type = GD32_TIMER2_TYPE,
  .mode       = CONFIG_GD32E11X_TIMER2_MODE,
  .base       = GD32_TIMER2_BASE,
  .pclk       = GD32_TIMER2_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB1EN_TIMER2EN,
  .rcurst     = RCU_APB1RST_TIMER2RST,
  .rcureg     = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER2,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER2_PWM */

/* TIMER3 - General Timer ***************************************************/

#ifdef CONFIG_GD32E11X_TIMER3_PWM

static struct gd32_pwmchan_s g_pwm3channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER3_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER3_CH0P,
      .pincfg = PWM_TIMER3_CH0CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER3_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER3_CH1P,
      .pincfg = PWM_TIMER3_CH1CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_GD32E11X_TIMER3_CH2MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER3_CH2P,
      .pincfg = PWM_TIMER3_CH2CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_GD32E11X_TIMER3_CH3MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER3_CH3P,
      .pincfg = PWM_TIMER3_CH3CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm3dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm3channels,
  .timer_id   = 3,
  .chan_num   = PWM_TIMER3_NCHANNELS,
  .timer_type = GD32_TIMER3_TYPE,
  .mode       = CONFIG_GD32E11X_TIMER3_MODE,
  .base       = GD32_TIMER3_BASE,
  .pclk       = GD32_TIMER3_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB1EN_TIMER3EN,
  .rcurst     = RCU_APB1RST_TIMER3RST,
  .rcureg     = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER3,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER3_PWM */

/* TIMER4 - General Timer ***************************************************/

#ifdef CONFIG_GD32E11X_TIMER4_PWM

static struct gd32_pwmchan_s g_pwm4channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER4_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER4_CH0P,
      .pincfg = PWM_TIMER4_CH0CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER4_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER4_CH1P,
      .pincfg = PWM_TIMER4_CH1CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_GD32E11X_TIMER4_CH2MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER4_CH2P,
      .pincfg = PWM_TIMER4_CH2CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_GD32E11X_TIMER4_CH3MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER4_CH3P,
      .pincfg = PWM_TIMER4_CH3CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm4dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm4channels,
  .timer_id   = 4,
  .chan_num   = PWM_TIMER4_NCHANNELS,
  .timer_type = GD32_TIMER4_TYPE,
  .mode       = CONFIG_GD32E11X_TIMER4_MODE,
  .base       = GD32_TIMER4_BASE,
  .pclk       = GD32_TIMER4_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB1EN_TIMER4EN,
  .rcurst     = RCU_APB1RST_TIMER4RST,
  .rcureg     = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER4,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER4_PWM */

/* TIMER7 - Advanced Timer **************************************************/

#ifdef CONFIG_GD32E11X_TIMER7_PWM

static struct gd32_pwmchan_s g_pwm7channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER7_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER7_CH0P,
      .idle   = CONFIG_GD32E11X_TIMER7_CH0IDLE,
      .pincfg = PWM_TIMER7_CH0CFG,
    },
#ifdef HAVE_PWM_COMPLEMENTARY
    .out2    =
    {
#ifdef CONFIG_GD32E11X_TIMER7_CH0ON
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER7_CH0NP,
      .idle   = CONFIG_GD32E11X_TIMER7_CH0NIDLE,
      .pincfg = PWM_TIMER7_CH0NCFG,
#else
      .in_use = 0,
#endif
    },
#endif
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER7_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER7_CH1P,
      .idle   = CONFIG_GD32E11X_TIMER7_CH1IDLE,
      .pincfg = PWM_TIMER7_CH1CFG,
    },
#ifdef HAVE_PWM_COMPLEMENTARY
    .out2    =
    {
#ifdef CONFIG_GD32E11X_TIMER7_CH1ON
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER7_CH1NP,
      .idle   = CONFIG_GD32E11X_TIMER7_CH1NIDLE,
      .pincfg = PWM_TIMER7_CH1NCFG,
#else
      .in_use = 0,
#endif
    },
#endif
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_GD32E11X_TIMER7_CH2MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER7_CH2P,
      .idle   = CONFIG_GD32E11X_TIMER7_CH2IDLE,
      .pincfg = PWM_TIMER7_CH2CFG,
    },
#ifdef HAVE_PWM_COMPLEMENTARY
    .out2    =
    {
#ifdef CONFIG_GD32E11X_TIMER7_CH2ON
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER7_CH2NP,
      .idle   = CONFIG_GD32E11X_TIMER7_CH2NIDLE,
      .pincfg = PWM_TIMER7_CH2NCFG,
#else
      .in_use = 0,
#endif
    },
#endif
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_GD32E11X_TIMER7_CH3MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER7_CH3P,
      .idle   = CONFIG_GD32E11X_TIMER7_CH3IDLE,
      .pincfg = PWM_TIMER7_CH3CFG,
    },

    /* CH3 has no complementary output */
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm7dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm7channels,
  .timer_id   = 7,
  .chan_num   = PWM_TIMER7_NCHANNELS,
  .timer_type = GD32_TIMER7_TYPE,
  .mode       = CONFIG_GD32E11X_TIMER7_MODE,
  .t_dts      = CONFIG_GD32E11X_TIMER7_FDTS,
  .lock       = CONFIG_GD32E11X_TIMER7_LOCK,
#ifdef HAVE_BREAK
  .brk        =
  {
#ifdef CONFIG_GD32E11X_TIMER7_BREAK
    .en      = 1,
    .pol     = CONFIG_GD32E11X_TIMER7_BRKP,
#else
    .en      = 0,
    .pol     = 0,
#endif
  },
#endif
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime   = CONFIG_GD32E11X_TIMER7_DEADTIME,
#endif
#ifdef HAVE_TRGO
  .trgo       = CONFIG_GD32E11X_TIMER7_TRGO,
#endif
  .base       = GD32_TIMER7_BASE,
  .pclk       = GD32_TIMER7_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB2EN_TIMER7EN,
  .rcurst     = RCU_APB2RST_TIMER7RST,
  .rcureg     = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER7_UP_TIMER12,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER7_PWM */

/* TIMER8 - General Timer (2 channels) **************************************/

#ifdef CONFIG_GD32E11X_TIMER8_PWM

static struct gd32_pwmchan_s g_pwm8channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER8_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER8_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER8_CH0P,
      .pincfg = PWM_TIMER8_CH0CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER8_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER8_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER8_CH1P,
      .pincfg = PWM_TIMER8_CH1CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm8dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm8channels,
  .timer_id   = 8,
  .chan_num   = PWM_TIMER8_NCHANNELS,
  .timer_type = GD32_TIMER8_TYPE,
  .base       = GD32_TIMER8_BASE,
  .pclk       = GD32_TIMER8_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB2EN_TIMER8EN,
  .rcurst     = RCU_APB2RST_TIMER8RST,
  .rcureg     = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER0_BRK_TIMER8,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER8_PWM */

/* TIMER9 - General Timer (1 channel) ***************************************/

#ifdef CONFIG_GD32E11X_TIMER9_PWM

static struct gd32_pwmchan_s g_pwm9channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER9_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER9_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER9_CH0P,
      .pincfg = PWM_TIMER9_CH0CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm9dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm9channels,
  .timer_id   = 9,
  .chan_num   = PWM_TIMER9_NCHANNELS,
  .timer_type = GD32_TIMER9_TYPE,
  .base       = GD32_TIMER9_BASE,
  .pclk       = GD32_TIMER9_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB2EN_TIMER9EN,
  .rcurst     = RCU_APB2RST_TIMER9RST,
  .rcureg     = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER0_UP_TIMER9,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER9_PWM */

/* TIMER10 - General Timer (1 channel) **************************************/

#ifdef CONFIG_GD32E11X_TIMER10_PWM

static struct gd32_pwmchan_s g_pwm10channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER10_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER10_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER10_CH0P,
      .pincfg = PWM_TIMER10_CH0CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm10dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm10channels,
  .timer_id   = 10,
  .chan_num   = PWM_TIMER10_NCHANNELS,
  .timer_type = GD32_TIMER10_TYPE,
  .base       = GD32_TIMER10_BASE,
  .pclk       = GD32_TIMER10_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB2EN_TIMER10EN,
  .rcurst     = RCU_APB2RST_TIMER10RST,
  .rcureg     = GD32_RCU_APB2EN,
  .rcurstreg = GD32_RCU_APB2RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER0_TRG_CMT_TIMER10,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER10_PWM */

/* TIMER11 - General Timer (2 channels) *************************************/

#ifdef CONFIG_GD32E11X_TIMER11_PWM

static struct gd32_pwmchan_s g_pwm11channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER11_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER11_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER11_CH0P,
      .pincfg = PWM_TIMER11_CH0CFG,
    },
  },
#endif
#ifdef CONFIG_GD32E11X_TIMER11_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_GD32E11X_TIMER11_CH1MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER11_CH1P,
      .pincfg = PWM_TIMER11_CH1CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm11dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm11channels,
  .timer_id   = 11,
  .chan_num   = PWM_TIMER11_NCHANNELS,
  .timer_type = GD32_TIMER11_TYPE,
  .base       = GD32_TIMER11_BASE,
  .pclk       = GD32_TIMER11_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB1EN_TIMER11EN,
  .rcurst     = RCU_APB1RST_TIMER11RST,
  .rcureg     = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER7_BRK_TIMER11,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER11_PWM */

/* TIMER12 - General Timer (1 channel) **************************************/

#ifdef CONFIG_GD32E11X_TIMER12_PWM

static struct gd32_pwmchan_s g_pwm12channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER12_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER12_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER12_CH0P,
      .pincfg = PWM_TIMER12_CH0CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm12dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm12channels,
  .timer_id   = 12,
  .chan_num   = PWM_TIMER12_NCHANNELS,
  .timer_type = GD32_TIMER12_TYPE,
  .base       = GD32_TIMER12_BASE,
  .pclk       = GD32_TIMER12_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB1EN_TIMER12EN,
  .rcurst     = RCU_APB1RST_TIMER12RST,
  .rcureg     = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER7_UP_TIMER12,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER12_PWM */

/* TIMER13 - General Timer (1 channel) **************************************/

#ifdef CONFIG_GD32E11X_TIMER13_PWM

static struct gd32_pwmchan_s g_pwm13channels[] =
{
#ifdef CONFIG_GD32E11X_TIMER13_CHANNEL0
  {
    .channel = 0,
    .mode    = CONFIG_GD32E11X_TIMER13_CH0MODE,
    .out1    =
    {
      .in_use = 1,
      .pol    = CONFIG_GD32E11X_TIMER13_CH0P,
      .pincfg = PWM_TIMER13_CH0CFG,
    },
  },
#endif
};

static struct gd32_pwmtimer_s g_pwm13dev =
{
  .ops        = &g_pwmops,
#ifdef CONFIG_GD32E11X_PWM_LL_OPS
  .llops      = &g_llpwmops,
#endif
  .channels   = g_pwm13channels,
  .timer_id   = 13,
  .chan_num   = PWM_TIMER13_NCHANNELS,
  .timer_type = GD32_TIMER13_TYPE,
  .base       = GD32_TIMER13_BASE,
  .pclk       = GD32_TIMER13_CLKIN,
  .frequency  = 0,
  .rcuen      = RCU_APB1EN_TIMER13EN,
  .rcurst     = RCU_APB1RST_TIMER13RST,
  .rcureg     = GD32_RCU_APB1EN,
  .rcurstreg = GD32_RCU_APB1RST,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = GD32_IRQ_TIMER7_TRG_CMT_TIMER13,
#endif
};
#endif /* CONFIG_GD32E11X_TIMER13_PWM */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of a PWM timer register
 *
 * Input Parameters:
 *   priv   - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t pwm_getreg(struct gd32_pwmtimer_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Write a value to a PWM timer register
 *
 * Input Parameters:
 *   priv   - A reference to the PWM block status
 *   offset - The offset to the register to write
 *   value  - The value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_putreg(struct gd32_pwmtimer_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: pwm_modifyreg
 *
 * Description:
 *   Modify a PWM timer register
 *
 * Input Parameters:
 *   priv      - A reference to the PWM block status
 *   offset    - The offset to the register to modify
 *   clearbits - The bits to clear
 *   setbits   - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_modifyreg(struct gd32_pwmtimer_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   msg - Message to print with the register dump
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev, const char *msg)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  pwminfo("%s:\n", msg);

  /* CTL0 is present on all timers.
   * CTL1 and SMCFG are not available on 1-channel timers (TIMER9-10,12-13).
   */

  if (priv->timer_type == GD32_TIMER_TYPE_GENERAL16_1CH)
    {
      pwminfo("  CTL0: %04" PRIx32 " DMAINTEN: %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_CTL0_OFFSET),
              pwm_getreg(priv, GD32_TIMER_DMAINTEN_OFFSET));
    }
  else
    {
      pwminfo("  CTL0: %04" PRIx32 " CTL1:   %04" PRIx32
              " SMCFG: %04" PRIx32 " DMAINTEN: %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_CTL0_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CTL1_OFFSET),
              pwm_getreg(priv, GD32_TIMER_SMCFG_OFFSET),
              pwm_getreg(priv, GD32_TIMER_DMAINTEN_OFFSET));
    }

  /* CHCTL1 is only available on 4-channel timers (advanced + general) */

  if (priv->timer_type == GD32_TIMER_TYPE_GENERAL16_1CH ||
      priv->timer_type == GD32_TIMER_TYPE_GENERAL16_2CH)
    {
      pwminfo("  INTF: %04" PRIx32 " SWEVG:  %04" PRIx32
              " CHCTL0: %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_INTF_OFFSET),
              pwm_getreg(priv, GD32_TIMER_SWEVG_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CHCTL0_OFFSET));
    }
  else
    {
      pwminfo("  INTF: %04" PRIx32 " SWEVG:  %04" PRIx32
              " CHCTL0: %04" PRIx32 " CHCTL1: %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_INTF_OFFSET),
              pwm_getreg(priv, GD32_TIMER_SWEVG_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CHCTL0_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CHCTL1_OFFSET));
    }

  pwminfo("  CHCTL2: %04" PRIx32 " CNT:   %04" PRIx32
          " PSC:    %04" PRIx32 " CAR:    %04" PRIx32 "\n",
          pwm_getreg(priv, GD32_TIMER_CHCTL2_OFFSET),
          pwm_getreg(priv, GD32_TIMER_CNT_OFFSET),
          pwm_getreg(priv, GD32_TIMER_PSC_OFFSET),
          pwm_getreg(priv, GD32_TIMER_CAR_OFFSET));

  /* CREP and CCHP are only available on advanced timers */

  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      pwminfo("  CREP: %04" PRIx32 " CCHP:  %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_CREP_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CCHP_OFFSET));
    }

  /* CHxCV registers based on number of channels */

  if (priv->timer_type == GD32_TIMER_TYPE_GENERAL16_1CH)
    {
      pwminfo("  CH0CV: %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_CH0CV_OFFSET));
    }
  else if (priv->timer_type == GD32_TIMER_TYPE_GENERAL16_2CH)
    {
      pwminfo("  CH0CV: %04" PRIx32 " CH1CV: %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_CH0CV_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CH1CV_OFFSET));
    }
  else
    {
      pwminfo("  CH0CV: %04" PRIx32 " CH1CV: %04" PRIx32
              " CH2CV: %04" PRIx32 " CH3CV: %04" PRIx32 "\n",
              pwm_getreg(priv, GD32_TIMER_CH0CV_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CH1CV_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CH2CV_OFFSET),
              pwm_getreg(priv, GD32_TIMER_CH3CV_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: pwm_chcv_update
 *
 * Description:
 *   Update channel compare value register (CHxCV)
 *
 ****************************************************************************/

static int pwm_chcv_update(struct pwm_lowerhalf_s *dev, uint8_t index,
                           uint32_t chcv)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t offset = 0;

  /* GD32 uses 0-based channel indexing */

  switch (index)
    {
      case GD32_PWM_CHAN0:
        offset = GD32_TIMER_CH0CV_OFFSET;
        break;

      case GD32_PWM_CHAN1:
        offset = GD32_TIMER_CH1CV_OFFSET;
        break;

      case GD32_PWM_CHAN2:
        offset = GD32_TIMER_CH2CV_OFFSET;
        break;

      case GD32_PWM_CHAN3:
        offset = GD32_TIMER_CH3CV_OFFSET;
        break;

      default:
        pwmerr("ERROR: No such CHxCV: %u\n", index);
        return -EINVAL;
    }

  /* Update CHxCV register */

  pwm_putreg(priv, offset, chcv);

  return OK;
}

/****************************************************************************
 * Name: pwm_chcv_get
 *
 * Description:
 *   Get channel compare value register (CHxCV)
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_PWM_LL_OPS
static uint32_t pwm_chcv_get(struct pwm_lowerhalf_s *dev, uint8_t index)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t offset = 0;

  switch (index)
    {
      case GD32_PWM_CHAN0:
        offset = GD32_TIMER_CH0CV_OFFSET;
        break;

      case GD32_PWM_CHAN1:
        offset = GD32_TIMER_CH1CV_OFFSET;
        break;

      case GD32_PWM_CHAN2:
        offset = GD32_TIMER_CH2CV_OFFSET;
        break;

      case GD32_PWM_CHAN3:
        offset = GD32_TIMER_CH3CV_OFFSET;
        break;

      default:
        pwmerr("ERROR: No such CHxCV: %u\n", index);
        return 0;
    }

  return pwm_getreg(priv, offset);
}
#endif

/****************************************************************************
 * Name: pwm_car_update
 *
 * Description:
 *   Update counter auto-reload register (CAR)
 *
 ****************************************************************************/

static int pwm_car_update(struct pwm_lowerhalf_s *dev, uint32_t car)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  /* Update CAR register */

  pwm_putreg(priv, GD32_TIMER_CAR_OFFSET, car);
  return OK;
}

/****************************************************************************
 * Name: pwm_car_get
 *
 * Description:
 *   Get counter auto-reload register value (CAR)
 *
 ****************************************************************************/

static uint32_t pwm_car_get(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  return pwm_getreg(priv, GD32_TIMER_CAR_OFFSET);
}

#ifdef HAVE_ADVANCED_TIMER
/****************************************************************************
 * Name: pwm_crep_update
 *
 * Description:
 *   Update counter repetition register (CREP)
 *
 ****************************************************************************/

static int pwm_crep_update(struct pwm_lowerhalf_s *dev, uint16_t crep)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  pwm_putreg(priv, GD32_TIMER_CREP_OFFSET, crep);
  return OK;
}

/****************************************************************************
 * Name: pwm_crep_get
 *
 * Description:
 *   Get counter repetition register value (CREP)
 *
 ****************************************************************************/

static uint16_t pwm_crep_get(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  return (uint16_t)pwm_getreg(priv, GD32_TIMER_CREP_OFFSET);
}
#endif

/****************************************************************************
 * Name: pwm_duty_update
 *
 * Description:
 *   Update the duty cycle for a PWM channel.
 *
 *   The duty cycle is specified as a ub16_t (unsigned 16.16 fixed-point):
 *     0x00000000 = 0%
 *     0x00008000 = 50%
 *     0x0000ffff = 99.998%
 *
 ****************************************************************************/

static int pwm_duty_update(struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t reload = 0;
  uint32_t chcv   = 0;

  /* We don't want compilation warnings if no DEBUGASSERT */

  UNUSED(priv);

  DEBUGASSERT(priv != NULL);

    pwminfo("TIMER%d channel: %u duty: %08" PRIx32 "\n",
      priv->timer_id, channel, (uint32_t)duty);

  /* Get the reload value */

  reload = pwm_car_get(dev);

  /* Duty cycle:
   * chcv = duty * reload / 65536
   * Use 64-bit intermediate to avoid overflow
   */

  chcv = (uint32_t)(((uint64_t)duty * (reload + 1) + 0x8000) >> 16);
  if (chcv > reload + 1)
    {
      chcv = reload + 1;
    }

  pwminfo("chcv: %" PRIu32 "\n", chcv);

  /* Write CHxCV register */

  return pwm_chcv_update(dev, channel, chcv);
}

/****************************************************************************
 * Name: pwm_timer_enable
 *
 * Description:
 *   Enable or disable the timer counter (CTL0.CEN)
 *
 ****************************************************************************/

static int pwm_timer_enable(struct pwm_lowerhalf_s *dev, bool state)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  if (state)
    {
      pwm_modifyreg(priv, GD32_TIMER_CTL0_OFFSET, 0, TIMER_CTL0_CEN);
    }
  else
    {
      pwm_modifyreg(priv, GD32_TIMER_CTL0_OFFSET, TIMER_CTL0_CEN, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_frequency_update
 *
 * Description:
 *   Update the PWM frequency by setting the prescaler (PSC) and
 *   auto-reload (CAR) register values.
 *
 ****************************************************************************/

static int pwm_frequency_update(struct pwm_lowerhalf_s *dev,
                                uint32_t frequency)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t prescaler = 0;
  uint32_t timer_clk = 0;
  uint32_t reload    = 0;

  /* Calculate optimal values for the timer prescaler and reload.
   *
   *   reload = timer_clk / frequency
   *   timer_clk = pclk / presc
   *
   * Or:
   *   reload = pclk / presc / frequency
   *
   * The best solution has the largest reload value and smallest prescaler.
   * Subject to:
   *   1 <= presc  <= 65536
   *   1 <= reload <= 65536
   */

  prescaler = (priv->pclk / frequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timer_clk = priv->pclk / prescaler;

  reload = timer_clk / frequency;
  if (reload < 2)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }
  else
    {
      reload--;
    }

  pwminfo("TIMER%d PCLK: %" PRIu32 " prescaler: %" PRIu32
          " reload: %" PRIu32 "\n",
          priv->timer_id, priv->pclk, prescaler, reload);

  /* Set the reload and prescaler values */

  pwm_car_update(dev, reload);
  pwm_putreg(priv, GD32_TIMER_PSC_OFFSET, (uint16_t)(prescaler - 1));

  return OK;
}

/****************************************************************************
 * Name: pwm_timer_configure
 *
 * Description:
 *   Initial timer configuration for PWM.
 *
 *   Set up the timer CTL0 register:
 *
 *   TIMER0,7   CKDIV[1:0] ARSE CAM[1:0] DIR SPM UPS UPDIS CEN
 *   TIMER1-4   CKDIV[1:0] ARSE CAM[1:0] DIR SPM UPS UPDIS CEN
 *   TIMER5-6              ARSE               SPM UPS UPDIS CEN
 *   TIMER8,11  CKDIV[1:0] ARSE                   UPS UPDIS CEN
 *   TIMER9-10,12-13
 *              CKDIV[1:0] ARSE                   UPS UPDIS CEN
 *
 ****************************************************************************/

static int pwm_timer_configure(struct gd32_pwmtimer_s *priv)
{
  uint32_t ctl0;

  /* Get current CTL0 value */

  ctl0 = pwm_getreg(priv, GD32_TIMER_CTL0_OFFSET);

  /* Clear counter mode bits (DIR, CAM) */

  ctl0 &= ~(TIMER_CTL0_DIR | TIMER_CTL0_CAM_MASK);

  /* Configure counter mode based on priv->mode:
   *   0 = Edge-aligned, up counting
   *   1 = Edge-aligned, down counting
   *   2 = Center-aligned, down counting trigger
   *   3 = Center-aligned, up counting trigger
   *   4 = Center-aligned, both direction trigger
   */

  switch (priv->mode)
    {
      default:
      case GD32_PWM_TIMER_MODE_COUNTUP:

        /* DIR=0, CAM=00 - already cleared */

        break;

      case GD32_PWM_TIMER_MODE_COUNTDOWN:
        ctl0 |= TIMER_CTL0_CAM_EDGE | TIMER_CTL0_DIR;
        break;

      case GD32_PWM_TIMER_MODE_CENTER_DOWN:
        ctl0 |= TIMER_CTL0_CAM_CENTER_DOWN;
        break;

      case GD32_PWM_TIMER_MODE_CENTER_UP:
        ctl0 |= TIMER_CTL0_CAM_CENTER_UP;
        break;

      case GD32_PWM_TIMER_MODE_CENTER_BOTH:
        ctl0 |= TIMER_CTL0_CAM_CENTER_BOTH;
        break;
    }

  /* Configure dead-time clock division (CKDIV) for timers that support it.
   * CKDIV is available on advanced timers (TIMER0/7) and general timers
   * (TIMER1-4, TIMER8-13).  Only basic timers (TIMER5/6) lack CKDIV.
   * For advanced timers, CKDIV affects dead-time generator.
   * For other timers, CKDIV affects digital filter sampling clock.
   */

  if (priv->timer_type != GD32_TIMER_TYPE_BASIC)
    {
      ctl0 &= ~TIMER_CTL0_CKDIV_MASK;
      ctl0 |= ((uint32_t)priv->t_dts << TIMER_CTL0_CKDIV_SHIFT)
               & TIMER_CTL0_CKDIV_MASK;
    }

  /* Enable auto-reload preload (ARSE) */

  ctl0 |= TIMER_CTL0_ARSE;

  /* Write CTL0 */

  pwm_putreg(priv, GD32_TIMER_CTL0_OFFSET, ctl0);

  return OK;
}

/****************************************************************************
 * Name: pwm_mode_configure
 *
 * Description:
 *   Configure the PWM mode for a channel in the CHCTL0 or CHCTL1 register.
 *
 ****************************************************************************/

static int pwm_mode_configure(struct pwm_lowerhalf_s *dev,
                              uint8_t channel, uint32_t mode)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t chctl_offset;
  uint32_t chctl_val;
  uint32_t chctl_mask;
  uint32_t ocmode;
  uint8_t  shift;
  int      ret = OK;

  /* Determine register and bit position */

  if (channel < 2)
    {
      chctl_offset = GD32_TIMER_CHCTL0_OFFSET;
      shift = (channel == 0) ? 0 : 8;
    }
  else if (channel < 4)
    {
      chctl_offset = GD32_TIMER_CHCTL1_OFFSET;
      shift = (channel == 2) ? 0 : 8;
    }
  else
    {
      pwmerr("ERROR: No such channel: %u\n", channel);
      ret = -EINVAL;
      goto errout;
    }

  /* Get channel output compare mode value.
   * Reference: GD32 TIMER_OC_MODE definitions.
   * Maps to CHxCOMCTL[6:4] bits in CHCTLx register.
   */

  switch (mode)
    {
      case GD32_CHANMODE_TIMING:
        ocmode = 0x00;  /* Timing - frozen, no effect on output */
        break;

      case GD32_CHANMODE_ACTIVE:
        ocmode = 0x01;  /* Active on match */
        break;

      case GD32_CHANMODE_INACTIVE:
        ocmode = 0x02;  /* Inactive on match */
        break;

      case GD32_CHANMODE_TOGGLE:
        ocmode = 0x03;  /* Toggle on match */
        break;

      case GD32_CHANMODE_FORCE_LO:
        ocmode = 0x04;  /* Force low */
        break;

      case GD32_CHANMODE_FORCE_HI:
        ocmode = 0x05;  /* Force high */
        break;

      case GD32_CHANMODE_PWM0:
        ocmode = 0x06;  /* PWM mode 0 */
        break;

      case GD32_CHANMODE_PWM1:
        ocmode = 0x07;  /* PWM mode 1 */
        break;

      default:
        pwmerr("ERROR: No such mode: %" PRIu32 "\n", mode);
        ret = -EINVAL;
        goto errout;
    }

  /* Build the mask and value.
   * Bit layout for each channel (in its 8-bit field):
   *   Bit 0-1: CHxMS    (mode selection, 00 = output)
   *   Bit 2:   CHxCOMFEN (fast enable)
   *   Bit 3:   CHxCOMSEN (shadow enable / preload)
   *   Bit 4-6: CHxCOMCTL (output compare mode)
   *   Bit 7:   CHxCOMCEN (clear enable)
   */

  chctl_mask = (uint32_t)(0xff << shift);

  /* CHxMS = 00 (output), CHxCOMCTL = mode, CHxCOMSEN = 1 (preload) */

  chctl_val = (uint32_t)(((ocmode & 0x07) << 4) | (1 << 3)) << shift;

  pwm_modifyreg(priv, chctl_offset, chctl_mask, chctl_val);

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_output_configure
 *
 * Description:
 *   Configure the output polarity, complementary output, and idle states
 *   for a PWM channel.
 *
 ****************************************************************************/

static int pwm_output_configure(struct gd32_pwmtimer_s *priv,
                                struct gd32_pwmchan_s *chan)
{
  uint8_t  channel = chan->channel;
  uint32_t chctl2 = 0;
  uint32_t ctl1 = 0;
  uint8_t  chctl2_shift;

  /* Get current registers */

  chctl2 = pwm_getreg(priv, GD32_TIMER_CHCTL2_OFFSET);
  ctl1 = pwm_getreg(priv, GD32_TIMER_CTL1_OFFSET);

  /* Each channel occupies 4 bits in CHCTL2:
   *   CH0: bits 0-3, CH1: bits 4-7, CH2: bits 8-11, CH3: bits 12-15
   */

  chctl2_shift = channel * 4;

  /* Clear all bits for this channel */

  chctl2 &= ~(uint32_t)(0x0f << chctl2_shift);

  /* Main output configuration */

  if (chan->out1.in_use)
    {
      /* Set polarity */

      if (chan->out1.pol == GD32_POL_ACTIVE_LOW)
        {
          chctl2 |= (uint32_t)(1 << (chctl2_shift + 1));  /* CHxP */
        }
    }

#ifdef HAVE_PWM_COMPLEMENTARY
  /* Complementary output configuration (advanced timers only) */

  if (GD32_TIMER_IS_ADVANCED(priv->timer_type) && chan->out2.in_use &&
      channel < 3)
    {
      /* Set complementary polarity */

      if (chan->out2.pol == GD32_POL_ACTIVE_LOW)
        {
          chctl2 |= (uint32_t)(1 << (chctl2_shift + 3));  /* CHxNP */
        }
    }
#endif

  /* Idle state configuration (advanced timers only) */

  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      /* ISO bits start at bit 8 in CTL1, each channel uses 2 bits */

      uint8_t iso_shift = 8 + channel * 2;

      ctl1 &= ~(uint32_t)((channel < 3) ?
                          (3 << iso_shift) : (1 << iso_shift));

      if (chan->out1.idle == GD32_IDLE_ACTIVE)
        {
          ctl1 |= (uint32_t)(1 << iso_shift);
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      if (channel < 3 && chan->out2.in_use &&
          chan->out2.idle == GD32_IDLE_ACTIVE)
        {
          ctl1 |= (uint32_t)(1 << (iso_shift + 1));
        }
#endif
    }

  /* Write registers */

  pwm_putreg(priv, GD32_TIMER_CHCTL2_OFFSET, chctl2);

  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      pwm_putreg(priv, GD32_TIMER_CTL1_OFFSET, ctl1);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_enable
 *
 * Description:
 *   Enable/disable given timer PWM outputs.
 *
 ****************************************************************************/

static int pwm_outputs_enable(struct pwm_lowerhalf_s *dev,
                              uint16_t outputs, bool state)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t chctl2;
  uint32_t regval = 0;

  /* Get current register state */

  chctl2 = pwm_getreg(priv, GD32_TIMER_CHCTL2_OFFSET);

  /* Build outputs configuration.
   * Output bits in CHCTL2:
   *   CH0EN = bit 0, CH0NEN = bit 2
   *   CH1EN = bit 4, CH1NEN = bit 6
   *   CH2EN = bit 8, CH2NEN = bit 10
   *   CH3EN = bit 12
   */

  regval |= ((outputs & GD32_PWM_OUT0)  ? TIMER_CHCTL2_CH0EN  : 0);
  regval |= ((outputs & GD32_PWM_OUT0N) ? TIMER_CHCTL2_CH0NEN : 0);
  regval |= ((outputs & GD32_PWM_OUT1)  ? TIMER_CHCTL2_CH1EN  : 0);
  regval |= ((outputs & GD32_PWM_OUT1N) ? TIMER_CHCTL2_CH1NEN : 0);
  regval |= ((outputs & GD32_PWM_OUT2)  ? TIMER_CHCTL2_CH2EN  : 0);
  regval |= ((outputs & GD32_PWM_OUT2N) ? TIMER_CHCTL2_CH2NEN : 0);
  regval |= ((outputs & GD32_PWM_OUT3)  ? TIMER_CHCTL2_CH3EN  : 0);

  if (state)
    {
      chctl2 |= regval;
    }
  else
    {
      chctl2 &= ~regval;
    }

  /* Write register */

  pwm_putreg(priv, GD32_TIMER_CHCTL2_OFFSET, chctl2);

  return OK;
}

#if defined(HAVE_PWM_COMPLEMENTARY) && defined(CONFIG_GD32E11X_PWM_LL_OPS)
/****************************************************************************
 * Name: pwm_deadtime_update
 *
 * Description:
 *   Update dead-time value
 *
 ****************************************************************************/

static int pwm_deadtime_update(struct pwm_lowerhalf_s *dev, uint8_t dt)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t cchp;
  int      ret = OK;

  /* Check if locked */

  if (priv->lock > 0)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Get current register state */

  cchp = pwm_getreg(priv, GD32_TIMER_CCHP_OFFSET);

  /* Update deadtime */

  cchp &= ~TIMER_CCHP_DTCFG_MASK;
  cchp |= (dt << TIMER_CCHP_DTCFG_SHIFT);

  /* Write register */

  pwm_putreg(priv, GD32_TIMER_CCHP_OFFSET, cchp);

errout:
  return ret;
}
#endif

#ifdef HAVE_TRGO
/****************************************************************************
 * Name: pwm_trgo_configure
 *
 * Description:
 *   Configure TRGO (trigger output) synchronization.
 *   This sets the master mode control (MMC) field in CTL1 register.
 *
 *   TRGO source values (CTL1.MMC):
 *     0 = Reset
 *     1 = Enable (CEN)
 *     2 = Update event
 *     3 = Compare pulse (CH0IF set)
 *     4 = CH0CV compare match
 *     5 = CH1CV compare match
 *     6 = CH2CV compare match
 *     7 = CH3CV compare match
 *
 ****************************************************************************/

static int pwm_trgo_configure(struct pwm_lowerhalf_s *dev, uint8_t trgo)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t ctl1 = 0;

  /* Configure TRGO */

  ctl1 |= (((trgo >> 0) & 0x07) << TIMER_CTL1_MMC_SHIFT)
         & TIMER_CTL1_MMC_MASK;

  /* Write register */

  pwm_modifyreg(priv, GD32_TIMER_CTL1_OFFSET, TIMER_CTL1_MMC_MASK, ctl1);

  return OK;
}
#endif

/****************************************************************************
 * Name: pwm_soft_update
 *
 * Description:
 *   Generate a software update event (SWEVG.UPG)
 *
 ****************************************************************************/

static int pwm_soft_update(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  pwm_putreg(priv, GD32_TIMER_SWEVG_OFFSET, TIMER_SWEVG_UPG);
  return OK;
}

/****************************************************************************
 * Name: pwm_soft_break
 *
 * Description:
 *   Enable or disable the primary output
 *   (primary output enable bit in CCHP register).
 *
 *   For advanced timers, outputs are enabled if state is false.
 *
 ****************************************************************************/

static int pwm_soft_break(struct pwm_lowerhalf_s *dev, bool state)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      if (state)
        {
          /* Disable primary output (soft break) */

          pwm_modifyreg(priv, GD32_TIMER_CCHP_OFFSET, TIMER_CCHP_POEN, 0);
        }
      else
        {
          /* Enable primary output */

          pwm_modifyreg(priv, GD32_TIMER_CCHP_OFFSET, 0, TIMER_CCHP_POEN);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_from_channels
 *
 * Description:
 *   Get enabled outputs configuration from the PWM timer state
 *
 ****************************************************************************/

static uint16_t pwm_outputs_from_channels(struct gd32_pwmtimer_s *priv)
{
  uint16_t outputs = 0;
  uint8_t  channel;
  int      i;

  for (i = 0; i < priv->chan_num; i++)
    {
      channel = priv->channels[i].channel;

      /* Set outputs if channel configured.
       * Output bit mapping: OUT0=0, OUT0N=1, OUT1=2, OUT1N=3, etc.
       */

      if (priv->channels[i].out1.in_use)
        {
          outputs |= (GD32_PWM_OUT0 << (channel * 2));  /* Main output */
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      if (priv->channels[i].out2.in_use)
        {
          outputs |= (GD32_PWM_OUT0N << (channel * 2)); /* Complementary */
        }
#endif
    }

  return outputs;
}

#ifdef HAVE_ADVANCED_TIMER
/****************************************************************************
 * Name: pwm_break_dt_configure
 *
 * Description:
 *   Configure break and dead-time for advanced timers
 *
 ****************************************************************************/

static int pwm_break_dt_configure(struct gd32_pwmtimer_s *priv)
{
  uint32_t cchp = 0;

#ifdef HAVE_PWM_COMPLEMENTARY
  /* Configure deadtime value (DTCFG field) */

  cchp |= ((uint32_t)priv->deadtime << TIMER_CCHP_DTCFG_SHIFT)
           & TIMER_CCHP_DTCFG_MASK;
#endif

#ifdef HAVE_BREAK
  /* Configure break input */

  if (priv->brk.en)
    {
      cchp |= TIMER_CCHP_BRKEN;

      if (priv->brk.pol)
        {
          cchp |= TIMER_CCHP_BRKP;
        }
    }
#endif

  /* Configure register lock level (PROT field) */

  cchp |= ((uint32_t)priv->lock << TIMER_CCHP_PROT_SHIFT)
           & TIMER_CCHP_PROT_MASK;

  /* Set OSSI and OSSR bits for proper idle/run behavior */

  cchp |= (TIMER_CCHP_IOS | TIMER_CCHP_ROS);

  /* Write CCHP register */

  pwm_putreg(priv, GD32_TIMER_CCHP_OFFSET, cchp);

  return OK;
}
#endif

#ifdef CONFIG_PWM_PULSECOUNT
/****************************************************************************
 * Name: pwm_pulsecount_configure
 *
 * Description:
 *   Configure PWM timer in PULSECOUNT mode
 *
 ****************************************************************************/

static int pwm_pulsecount_configure(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint16_t outputs;
  int      ret = OK;
  int      i;

  /* Disable timer and all outputs */

  pwm_timer_enable(dev, false);

  outputs = pwm_outputs_from_channels(priv);
  ret = pwm_outputs_enable(dev, outputs, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Initial timer configuration */

  ret = pwm_timer_configure(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure break and deadtime */

  ret = pwm_break_dt_configure(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure TRGO synchronization output */

#ifdef HAVE_TRGO
  ret = pwm_trgo_configure(dev, priv->trgo);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Disable software break (enable outputs) */

  ret = pwm_soft_break(dev, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure each channel */

  for (i = 0; i < priv->chan_num; i++)
    {
      struct gd32_pwmchan_s *chan = &priv->channels[i];

      /* Update PWM mode */

      pwm_mode_configure(dev, chan->channel, chan->mode);

      /* PWM outputs configuration */

      pwm_output_configure(priv, chan);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_pulsecount_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 ****************************************************************************/

static int pwm_pulsecount_timer(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  ub16_t   duty;
  uint8_t  channel;
  uint16_t outputs;
  int      ret = OK;

  /* We don't want compilation warnings if no DEBUGASSERT */

  UNUSED(priv);

  DEBUGASSERT(priv != NULL && info != NULL);

    pwminfo("TIMER%d frequency: %" PRIu32 " duty: %08" PRIx32
      " count: %" PRIu32 "\n",
      priv->timer_id, info->frequency, info->duty, info->count);

  DEBUGASSERT(info->frequency > 0);

  /* Channel specific setup */

  duty = info->duty;
  channel = priv->channels[0].channel;

  /* Disable all interrupts and DMA requests, clear all pending status */

  pwm_putreg(priv, GD32_TIMER_DMAINTEN_OFFSET, 0);
  pwm_putreg(priv, GD32_TIMER_INTF_OFFSET, 0);

  /* Set timer frequency */

  ret = pwm_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      goto errout;
    }

  /* Update duty cycle */

  ret = pwm_duty_update(dev, channel, duty);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure repetition counter */

  if (info->count > 0)
    {
      priv->prev = pwm_pulsecount(info->count);
      pwm_crep_update(dev, priv->prev - 1);

      /* Generate update event to reload prescaler */

      pwm_soft_update(dev);

      /* Set up next CREP value */

      priv->count = info->count;
      priv->curr = pwm_pulsecount(info->count - priv->prev);
      pwm_crep_update(dev, priv->curr - 1);
    }
  else
    {
      pwm_crep_update(dev, 0);
      pwm_soft_update(dev);
    }

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Enable outputs */

  ret = pwm_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Setup update interrupt */

  if (info->count > 0)
    {
      pwm_putreg(priv, GD32_TIMER_INTF_OFFSET, 0);
      pwm_putreg(priv, GD32_TIMER_DMAINTEN_OFFSET, TIMER_DMAINTEN_UPIE);

      /* Enable timer */

      pwm_timer_enable(dev, true);

      /* Enable interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }

  pwm_dumpregs(dev, "After pulsecount start");

errout:
  return ret;
}
#endif /* CONFIG_PWM_PULSECOUNT */

/****************************************************************************
 * Name: pwm_configure
 *
 * Description:
 *   Configure PWM timer in normal mode (no PULSECOUNT)
 *
 ****************************************************************************/

static int pwm_configure(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint16_t outputs;
  int      ret = OK;
  int      i;

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Disable outputs */

  ret = pwm_outputs_enable(dev, outputs, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Disable timer */

  pwm_timer_enable(dev, false);

  /* Initial timer configuration */

  ret = pwm_timer_configure(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure break and deadtime for advanced timers */

#ifdef HAVE_ADVANCED_TIMER
  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      ret = pwm_break_dt_configure(priv);
      if (ret < 0)
        {
          goto errout;
        }
    }
#endif

  /* Configure TRGO synchronization output */

#ifdef HAVE_TRGO
  ret = pwm_trgo_configure(dev, priv->trgo);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure each channel */

  for (i = 0; i < priv->chan_num; i++)
    {
      struct gd32_pwmchan_s *chan = &priv->channels[i];

      /* Update PWM mode */

      ret = pwm_mode_configure(dev, chan->channel, chan->mode);
      if (ret < 0)
        {
          goto errout;
        }

      /* PWM outputs configuration */

      ret = pwm_output_configure(priv, chan);
      if (ret < 0)
        {
          goto errout;
        }
    }

  /* Enable main output for advanced timers */

  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      ret = pwm_soft_break(dev, false);
      if (ret < 0)
        {
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_duty_channels_update
 *
 * Description:
 *   Update duty cycle for given channels
 *
 ****************************************************************************/

static int pwm_duty_channels_update(struct pwm_lowerhalf_s *dev,
                                    const struct pwm_info_s *info)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint8_t channel;
  ub16_t  duty;
  int     ret = OK;
#ifdef CONFIG_PWM_MULTICHAN
  int     i;
  int     j;
#endif

#ifdef CONFIG_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      /* Break if all following channels are not configured */

      if (info->channels[i].channel == -1)
        {
          break;
        }

      duty = info->channels[i].duty;
      channel = info->channels[i].channel;

      /* In NuttX multichan framework, channel == 0 means "skip".
       * NuttX uses 1-based channel numbering in the pwm_info_s
       * structure, whereas GD32 uses 0-based channel indexing.
       * Convert from 1-based (NuttX) to 0-based (GD32).
       */

      if (channel != 0)
        {
          /* Convert from 1-based NuttX channel to 0-based GD32 channel */

          uint8_t hwchannel = channel - 1;

          /* Find matching channel configuration */

          for (j = 0; j < priv->chan_num; j++)
            {
              if (priv->channels[j].channel == hwchannel)
                {
                  break;
                }
            }

          if (j >= priv->chan_num)
            {
              pwmerr("ERROR: No such channel: %u\n", hwchannel);
              ret = -EINVAL;
              goto errout;
            }

          /* Update duty cycle using 0-based hw channel */

          ret = pwm_duty_update(dev, hwchannel, duty);
          if (ret < 0)
            {
              goto errout;
            }
        }
    }
#else
  duty = info->duty;
  channel = priv->channels[0].channel;

  /* Update duty cycle */

  ret = pwm_duty_update(dev, channel, duty);
  if (ret < 0)
    {
      goto errout;
    }
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 ****************************************************************************/

static int pwm_timer(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint16_t outputs;
  int      ret = OK;

  UNUSED(priv);
  DEBUGASSERT(priv != NULL && info != NULL);

#ifdef CONFIG_PWM_MULTICHAN
  pwminfo("TIMER%d frequency: %" PRIu32 "\n",
          priv->timer_id, info->frequency);
#else
  pwminfo("TIMER%d channel: %u frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
          priv->timer_id, priv->channels[0].channel,
          info->frequency, (uint32_t)info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);

  /* Set timer frequency */

  ret = pwm_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      goto errout;
    }

  /* Channel specific configuration */

  ret = pwm_duty_channels_update(dev, info);
  if (ret < 0)
    {
      goto errout;
    }

  /* Set repetition counter for advanced timers */

#ifdef HAVE_ADVANCED_TIMER
  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      pwm_crep_update(dev, 0);
      pwm_soft_update(dev);
    }
  else
#endif
    {
      pwm_soft_update(dev);
    }

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Enable outputs */

  ret = pwm_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Enable timer */

  pwm_timer_enable(dev, true);

  pwm_dumpregs(dev, "After starting");

errout:
  return ret;
}

#ifdef HAVE_PWM_INTERRUPT
/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 ****************************************************************************/

static int pwm_interrupt(struct gd32_pwmtimer_s *priv)
{
  uint16_t regval;

  /* Verify this is an update interrupt.
   * This check is critical for shared IRQ vectors (e.g., TIMER0_UP shares
   * with TIMER9, TIMER7_UP shares with TIMER12).  In release builds,
   * DEBUGASSERT is a no-op, so we must explicitly check and return.
   */

  regval = (uint16_t)pwm_getreg(priv, GD32_TIMER_INTF_OFFSET);
  if ((regval & TIMER_INTF_UPIF) == 0)
    {
      /* Not our interrupt - shared IRQ triggered by another timer */

      return OK;
    }

  /* Clear the update interrupt flag */

  pwm_putreg(priv, GD32_TIMER_INTF_OFFSET, regval & ~TIMER_INTF_UPIF);

  /* Calculate new count */

  if (priv->count <= priv->prev)
    {
      /* Finished - disable output */

      pwm_soft_break((struct pwm_lowerhalf_s *)priv, true);

      /* Disable interrupts and stop the timer */

      pwm_stop((struct pwm_lowerhalf_s *)priv);

      /* Callback to upper half driver */

      pwm_expired(priv->handle);

      priv->handle = NULL;
      priv->count  = 0;
      priv->prev   = 0;
      priv->curr   = 0;
    }
  else
    {
      /* Decrement pulse count */

      priv->count -= priv->prev;

      /* Set up next CREP */

      priv->prev = priv->curr;
      priv->curr = pwm_pulsecount(priv->count - priv->prev);
      pwm_crep_update((struct pwm_lowerhalf_s *)priv, priv->curr - 1);
    }

  pwminfo("Update interrupt: prev=%u curr=%u count=%" PRIu32 "\n",
          priv->prev, priv->curr, priv->count);

  return OK;
}

#ifdef CONFIG_GD32E11X_TIMER0_PWM
static int pwm_timer0_interrupt(int irq, void *context, void *arg)
{
  return pwm_interrupt(&g_pwm0dev);
}
#endif

#ifdef CONFIG_GD32E11X_TIMER7_PWM
static int pwm_timer7_interrupt(int irq, void *context, void *arg)
{
  return pwm_interrupt(&g_pwm7dev);
}
#endif

/****************************************************************************
 * Name: pwm_pulsecount
 *
 * Description:
 *   Pick an optimal pulse count to program the CREP.
 *
 ****************************************************************************/

static uint8_t pwm_pulsecount(uint32_t count)
{
  /* The remaining pulse count is less than or equal to the maximum, then
   * just return the count.
   */

  if (count <= TIMER_CREP_REP_MAX)
    {
      return (uint8_t)count;
    }

  /* Otherwise, we have to be careful.  We do not want a small number of
   * counts at the end because we might have trouble responding fast enough.
   * If the remaining count is less than 150% of the maximum, then return
   * half of the maximum.  In this case the final sequence will be between
   * 64 and 128.
   */

  else if (count < (3 * TIMER_CREP_REP_MAX / 2))
    {
      return (uint8_t)((TIMER_CREP_REP_MAX + 1) >> 1);
    }

  /* Otherwise, return the maximum.  The final count will be 64 or more */

  else
    {
      return (uint8_t)TIMER_CREP_REP_MAX;
    }
}
#endif /* HAVE_PWM_INTERRUPT */

/****************************************************************************
 * Name: pwm_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 ****************************************************************************/

static int pwm_set_apb_clock(struct gd32_pwmtimer_s *priv, bool on)
{
  pwminfo("TIMER%d clock enable: %d\n", priv->timer_id, on ? 1 : 0);

  if (on)
    {
      modifyreg32(priv->rcureg, 0, priv->rcuen);
    }
  else
    {
      modifyreg32(priv->rcureg, priv->rcuen, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_timer_reset
 *
 * Description:
 *   Reset the timer to its default state via the RCU reset register.
 *
 ****************************************************************************/

static void pwm_timer_reset(struct gd32_pwmtimer_s *priv)
{
  /* Set reset bit */

  modifyreg32(priv->rcurstreg, 0, priv->rcurst);

  /* Clear reset bit */

  modifyreg32(priv->rcurstreg, priv->rcurst, 0);
}

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t pincfg;
  int      ret = OK;
  int      i;

  pwminfo("TIMER%d\n", priv->timer_id);

  /* Enable APB clock for timer */

  ret = pwm_set_apb_clock(priv, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Reset the timer */

  pwm_timer_reset(priv);

  pwm_dumpregs(dev, "Initially");

  /* Configure PWM output pins */

  for (i = 0; i < priv->chan_num; i++)
    {
      if (priv->channels[i].out1.in_use)
        {
          pincfg = priv->channels[i].out1.pincfg;
          if (pincfg != 0)
            {
              pwminfo("pincfg: %08" PRIx32 "\n", pincfg);
              gd32_gpio_config(pincfg | GPIO_CFG_SPEED_50MHZ);
              pwm_dumpgpio(pincfg, "PWM setup");
            }
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      if (priv->channels[i].out2.in_use)
        {
          pincfg = priv->channels[i].out2.pincfg;
          if (pincfg != 0)
            {
              pwminfo("pincfg: %08" PRIx32 "\n", pincfg);
              gd32_gpio_config(pincfg | GPIO_CFG_SPEED_50MHZ);
              pwm_dumpgpio(pincfg, "PWM setup");
            }
        }
#endif
    }

  /* Configure PWM timer */

#ifdef CONFIG_PWM_PULSECOUNT
  if (GD32_TIMER_IS_ADVANCED(priv->timer_type))
    {
      ret = pwm_pulsecount_configure(dev);
    }
  else
#endif
    {
      ret = pwm_configure(dev);
    }

  if (ret < 0)
    {
      pwmerr("Failed to configure PWM TIMER%d\n", priv->timer_id);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  uint32_t pincfg;
  int      i;
  int      ret = OK;

  pwminfo("TIMER%d\n", priv->timer_id);

  /* Stop PWM output */

  pwm_stop(dev);

  /* Disable APB clock */

  ret = pwm_set_apb_clock(priv, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Put GPIO pins back to default state */

  for (i = 0; i < priv->chan_num; i++)
    {
      pincfg = priv->channels[i].out1.pincfg;
      if (pincfg != 0)
        {
          gd32_gpio_unconfig(pincfg);
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      pincfg = priv->channels[i].out2.pincfg;
      if (pincfg != 0)
        {
          gd32_gpio_unconfig(pincfg);
        }
#endif
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  int ret = OK;

  /* If frequency unchanged, just update duty and ensure timer is running.
   * After pwm_stop() the timer is disabled and frequency reset to 0,
   * so the frequency comparison will be false and pwm_timer() will be
   * called to fully reconfigure.
   */

  if (info->frequency == priv->frequency && priv->frequency != 0)
    {
#ifdef CONFIG_PWM_MULTICHAN
      int i;

      for (i = 0; ret == OK && i < CONFIG_PWM_NCHANNELS; i++)
        {
          if (info->channels[i].channel == -1)
            {
              break;
            }

          /* NuttX multichan uses 1-based channels (0=skip),
           * convert to 0-based GD32 channel.
           */

          if (info->channels[i].channel != 0)
            {
              ret = pwm_duty_update(dev, info->channels[i].channel - 1,
                                    info->channels[i].duty);
            }
        }
#else
      ret = pwm_duty_update(dev, priv->channels[0].channel, info->duty);
#endif
    }
  else
    {
      ret = pwm_timer(dev, info);

      if (ret == OK)
        {
          priv->frequency = info->frequency;
        }
    }

  return ret;
}

#ifdef CONFIG_PWM_PULSECOUNT

/****************************************************************************
 * Name: pwm_start_pulsecount
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output with
 *   pulse counting support.
 *
 ****************************************************************************/

static int pwm_start_pulsecount(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info,
                                void *handle)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  /* Generate an indefinite number of pulses */

  if (info->count == 0)
    {
      return pwm_start(dev, info);
    }

  /* Check if a pulsecount has been selected.
   * Only the advanced timers (TIMER0, TIMER7) can support pulse counting.
   */

  if (info->count > 0)
    {
      if (!GD32_TIMER_IS_ADVANCED(priv->timer_type))
        {
          pwmerr("ERROR: TIMER%d cannot support pulse count: %"
                 PRIu32 "\n", priv->timer_id, info->count);
          return -EPERM;
        }
    }

  /* Save the handle */

  priv->handle = handle;

  /* Start the time */

  return pwm_pulsecount_timer(dev, info);
}
#endif /* CONFIG_PWM_PULSECOUNT */

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  irqstate_t flags;
  uint16_t   outputs;
  int        ret = OK;

  pwminfo("TIMER%d\n", priv->timer_id);

  /* Disable interrupts momentarily */

  flags = enter_critical_section();

  /* Reset frequency */

  priv->frequency = 0;

  /* Disable further interrupts and stop timer */

  pwm_putreg(priv, GD32_TIMER_DMAINTEN_OFFSET, 0);
  pwm_putreg(priv, GD32_TIMER_INTF_OFFSET, 0);

  /* Disable timer */

  pwm_timer_enable(dev, false);

  /* Disable outputs */

  outputs = pwm_outputs_from_channels(priv);
  ret = pwm_outputs_enable(dev, outputs, false);

  /* Enable software break for advanced timers */

  pwm_soft_break(dev, true);

  /* Clear CHxCV registers only for channels that this timer actually has.
   * Writing to non-existent channel registers (reserved addresses) should
   * be avoided.
   */

  pwm_putreg(priv, GD32_TIMER_CH0CV_OFFSET, 0);

  if (priv->timer_type == GD32_TIMER_TYPE_GENERAL16 ||
      priv->timer_type == GD32_TIMER_TYPE_GENERAL16_2CH ||
      priv->timer_type == GD32_TIMER_TYPE_ADVANCED)
    {
      pwm_putreg(priv, GD32_TIMER_CH1CV_OFFSET, 0);
    }

  if (priv->timer_type == GD32_TIMER_TYPE_GENERAL16 ||
      priv->timer_type == GD32_TIMER_TYPE_ADVANCED)
    {
      pwm_putreg(priv, GD32_TIMER_CH2CV_OFFSET, 0);
      pwm_putreg(priv, GD32_TIMER_CH3CV_OFFSET, 0);
    }

  leave_critical_section(flags);

  pwm_dumpregs(dev, "After stop");

  return ret;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  pwminfo("TIMER%d cmd=%d\n", priv->timer_id, cmd);
#endif

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the GD32 MCU (TIMER0..TIMER4, TIMER7..TIMER13).
 *     Note: TIMER5 and TIMER6 are basic timers without PWM capability.
 *
 * Returned Value:
 *   On success, a pointer to the GD32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *gd32_pwminitialize(int timer)
{
  struct gd32_pwmtimer_s *lower = NULL;

  pwminfo("Initialize TIMER%d for PWM\n", timer);

  switch (timer)
    {
#ifdef CONFIG_GD32E11X_TIMER0_PWM
      case 0:
        lower = &g_pwm0dev;
#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_timer0_interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER1_PWM
      case 1:
        lower = &g_pwm1dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER2_PWM
      case 2:
        lower = &g_pwm2dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER3_PWM
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER4_PWM
      case 4:
        lower = &g_pwm4dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER7_PWM
      case 7:
        lower = &g_pwm7dev;
#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_timer7_interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER8_PWM
      case 8:
        lower = &g_pwm8dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER9_PWM
      case 9:
        lower = &g_pwm9dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER10_PWM
      case 10:
        lower = &g_pwm10dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER11_PWM
      case 11:
        lower = &g_pwm11dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER12_PWM
      case 12:
        lower = &g_pwm12dev;
        break;
#endif

#ifdef CONFIG_GD32E11X_TIMER13_PWM
      case 13:
        lower = &g_pwm13dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such TIMER%d configured for PWM\n", timer);
        lower = NULL;
        break;
    }

  return (struct pwm_lowerhalf_s *)lower;
}
