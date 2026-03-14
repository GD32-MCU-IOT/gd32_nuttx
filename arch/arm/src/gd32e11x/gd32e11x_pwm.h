/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_pwm.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_PWM_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/pwm.h>

#include "chip.h"
#include "hardware/gd32e11x_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  Such special purposes
 * include use as a PWM output.  Each timer that is used for PWM must be
 * enabled and a Kconfig option must exist for each timer to select its use
 * for PWM.
 *
 * CONFIG_GD32E11X_TIMERn_PWM      - Enables PWM for TIMERn
 * CONFIG_GD32E11X_TIMERn_CHANNELx - Enables channel x for TIMERn
 * CONFIG_GD32E11X_TIMERn_CHxO     - Enables main output for channel x
 * CONFIG_GD32E11X_TIMERn_CHxON    - Enables complementary output for ch x
 * CONFIG_GD32E11X_TIMERn_CHxMODE  - Channel x compare output mode
 * CONFIG_GD32E11X_TIMERn_CHxP     - Output polarity for channel x
 * CONFIG_GD32E11X_TIMERn_CHxIDLE  - Idle state for channel x
 * CONFIG_GD32E11X_TIMERn_CHxNP    - Complementary output polarity
 * CONFIG_GD32E11X_TIMERn_CHxNIDLE - Complementary idle state
 * CONFIG_GD32E11X_TIMERn_BREAK    - Break input enable
 * CONFIG_GD32E11X_TIMERn_BRKP     - Break polarity
 * CONFIG_GD32E11X_TIMERn_DEADTIME - Dead-time value
 * CONFIG_GD32E11X_TIMERn_FDTS     - Dead-time clock division (CKDIV)
 * CONFIG_GD32E11X_TIMERn_MODE     - Timer counting mode
 * CONFIG_GD32E11X_TIMERn_LOCK     - Register lock level
 * CONFIG_GD32E11X_PWM_TRGO        - TRGO synchronization support
 * CONFIG_GD32E11X_PWM_LL_OPS      - Low-level operations support
 */

/* Timer types **************************************************************/

#define GD32_TIMER_TYPE_BASIC         0  /* Basic timer (TIMER5-6, no channels) */
#define GD32_TIMER_TYPE_GENERAL16     1  /* General 16-bit timer (TIMER1-4) */
#define GD32_TIMER_TYPE_GENERAL16_2CH 2  /* General 16-bit timer with 2 channels (TIMER8, TIMER11) */
#define GD32_TIMER_TYPE_GENERAL16_1CH 3  /* General 16-bit timer with 1 channel (TIMER9-10, TIMER12-13) */
#define GD32_TIMER_TYPE_ADVANCED      4  /* Advanced timer (TIMER0, TIMER7) */

/* Map timer types to specific timers */

#define GD32_TIMER0_TYPE         GD32_TIMER_TYPE_ADVANCED
#define GD32_TIMER1_TYPE         GD32_TIMER_TYPE_GENERAL16
#define GD32_TIMER2_TYPE         GD32_TIMER_TYPE_GENERAL16
#define GD32_TIMER3_TYPE         GD32_TIMER_TYPE_GENERAL16
#define GD32_TIMER4_TYPE         GD32_TIMER_TYPE_GENERAL16
#define GD32_TIMER5_TYPE         GD32_TIMER_TYPE_BASIC
#define GD32_TIMER6_TYPE         GD32_TIMER_TYPE_BASIC
#define GD32_TIMER7_TYPE         GD32_TIMER_TYPE_ADVANCED
#define GD32_TIMER8_TYPE         GD32_TIMER_TYPE_GENERAL16_2CH
#define GD32_TIMER9_TYPE         GD32_TIMER_TYPE_GENERAL16_1CH
#define GD32_TIMER10_TYPE        GD32_TIMER_TYPE_GENERAL16_1CH
#define GD32_TIMER11_TYPE        GD32_TIMER_TYPE_GENERAL16_2CH
#define GD32_TIMER12_TYPE        GD32_TIMER_TYPE_GENERAL16_1CH
#define GD32_TIMER13_TYPE        GD32_TIMER_TYPE_GENERAL16_1CH

/* Check if timer is an advanced timer */

#define GD32_TIMER_IS_ADVANCED(timer_type) \
        ((timer_type) == GD32_TIMER_TYPE_ADVANCED)

/* Maximum number of PWM channels per timer */

#define PWM_MAX_CHANNELS          4

/* Timer clock input ********************************************************/

/* When APB prescaler is > 1, timer clock is 2x the APB clock */

#ifndef GD32_APB1_TIMER_CLKIN
#  define GD32_APB1_TIMER_CLKIN   (2 * GD32_PCLK1_FREQUENCY)
#endif

#ifndef GD32_APB2_TIMER_CLKIN
#  define GD32_APB2_TIMER_CLKIN   (2 * GD32_PCLK2_FREQUENCY)
#endif

/* Per-timer clock frequency.
 * APB2: TIMER0, 7-10
 * APB1: TIMER1-6, 11-13
 */

#define GD32_TIMER0_CLKIN          GD32_APB2_TIMER_CLKIN
#define GD32_TIMER1_CLKIN          GD32_APB1_TIMER_CLKIN
#define GD32_TIMER2_CLKIN          GD32_APB1_TIMER_CLKIN
#define GD32_TIMER3_CLKIN          GD32_APB1_TIMER_CLKIN
#define GD32_TIMER4_CLKIN          GD32_APB1_TIMER_CLKIN
#define GD32_TIMER5_CLKIN          GD32_APB1_TIMER_CLKIN
#define GD32_TIMER6_CLKIN          GD32_APB1_TIMER_CLKIN
#define GD32_TIMER7_CLKIN          GD32_APB2_TIMER_CLKIN
#define GD32_TIMER8_CLKIN          GD32_APB2_TIMER_CLKIN
#define GD32_TIMER9_CLKIN          GD32_APB2_TIMER_CLKIN
#define GD32_TIMER10_CLKIN         GD32_APB2_TIMER_CLKIN
#define GD32_TIMER11_CLKIN         GD32_APB1_TIMER_CLKIN
#define GD32_TIMER12_CLKIN         GD32_APB1_TIMER_CLKIN
#define GD32_TIMER13_CLKIN         GD32_APB1_TIMER_CLKIN

/* Complementary output support *********************************************/

/* TIMER0 complementary outputs */

#if defined(CONFIG_GD32E11X_TIMER0_CH0ON) || \
    defined(CONFIG_GD32E11X_TIMER0_CH1ON) || \
    defined(CONFIG_GD32E11X_TIMER0_CH2ON)
#  define HAVE_TIMER0_COMPLEMENTARY
#endif

/* TIMER7 complementary outputs */

#if defined(CONFIG_GD32E11X_TIMER7_CH0ON) || \
    defined(CONFIG_GD32E11X_TIMER7_CH1ON) || \
    defined(CONFIG_GD32E11X_TIMER7_CH2ON)
#  define HAVE_TIMER7_COMPLEMENTARY
#endif

#if defined(HAVE_TIMER0_COMPLEMENTARY) || defined(HAVE_TIMER7_COMPLEMENTARY)
#  define HAVE_PWM_COMPLEMENTARY
#endif

/* Break support ************************************************************/

#if defined(CONFIG_GD32E11X_TIMER0_BREAK) || \
    defined(CONFIG_GD32E11X_TIMER7_BREAK)
#  define HAVE_BREAK
#endif

/* TRGO synchronization support *********************************************/

#ifdef CONFIG_GD32E11X_PWM_TRGO
#  define HAVE_TRGO
#endif

/* PWM channel GPIO configuration macros ************************************
 *
 * These macros map Kconfig output enables to GPIO pin configurations.
 * If the output is enabled in Kconfig, the corresponding GPIO pin
 * definition from the pinmap header is used; otherwise 0.
 */

/* TIMER0 channel output configurations */

#ifdef CONFIG_GD32E11X_TIMER0_CH0O
#  define PWM_TIMER0_CH0CFG  GPIO_TIMER0_CH0OUT_0
#else
#  define PWM_TIMER0_CH0CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CH0ON
#  define PWM_TIMER0_CH0NCFG GPIO_TIMER0_CH0OUTN_0
#else
#  define PWM_TIMER0_CH0NCFG 0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CH1O
#  define PWM_TIMER0_CH1CFG  GPIO_TIMER0_CH1OUT_0
#else
#  define PWM_TIMER0_CH1CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CH1ON
#  define PWM_TIMER0_CH1NCFG GPIO_TIMER0_CH1OUTN_0
#else
#  define PWM_TIMER0_CH1NCFG 0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CH2O
#  define PWM_TIMER0_CH2CFG  GPIO_TIMER0_CH2OUT_0
#else
#  define PWM_TIMER0_CH2CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CH2ON
#  define PWM_TIMER0_CH2NCFG GPIO_TIMER0_CH2OUTN_0
#else
#  define PWM_TIMER0_CH2NCFG 0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CH3O
#  define PWM_TIMER0_CH3CFG  GPIO_TIMER0_CH3OUT_0
#else
#  define PWM_TIMER0_CH3CFG  0
#endif

/* TIMER1 channel output configurations */

#ifdef CONFIG_GD32E11X_TIMER1_CH0O
#  define PWM_TIMER1_CH0CFG  GPIO_TIMER1_CH0OUT_0
#else
#  define PWM_TIMER1_CH0CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CH1O
#  define PWM_TIMER1_CH1CFG  GPIO_TIMER1_CH1OUT_0
#else
#  define PWM_TIMER1_CH1CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CH2O
#  define PWM_TIMER1_CH2CFG  GPIO_TIMER1_CH2OUT_0
#else
#  define PWM_TIMER1_CH2CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CH3O
#  define PWM_TIMER1_CH3CFG  GPIO_TIMER1_CH3OUT_0
#else
#  define PWM_TIMER1_CH3CFG  0
#endif

/* TIMER2 channel output configurations */

#ifdef CONFIG_GD32E11X_TIMER2_CH0O
#  define PWM_TIMER2_CH0CFG  GPIO_TIMER2_CH0OUT_0
#else
#  define PWM_TIMER2_CH0CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CH1O
#  define PWM_TIMER2_CH1CFG  GPIO_TIMER2_CH1OUT_0
#else
#  define PWM_TIMER2_CH1CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CH2O
#  define PWM_TIMER2_CH2CFG  GPIO_TIMER2_CH2OUT_0
#else
#  define PWM_TIMER2_CH2CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CH3O
#  define PWM_TIMER2_CH3CFG  GPIO_TIMER2_CH3OUT_0
#else
#  define PWM_TIMER2_CH3CFG  0
#endif

/* TIMER3 channel output configurations */

#ifdef CONFIG_GD32E11X_TIMER3_CH0O
#  define PWM_TIMER3_CH0CFG  GPIO_TIMER3_CH0OUT_0
#else
#  define PWM_TIMER3_CH0CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CH1O
#  define PWM_TIMER3_CH1CFG  GPIO_TIMER3_CH1OUT_0
#else
#  define PWM_TIMER3_CH1CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CH2O
#  define PWM_TIMER3_CH2CFG  GPIO_TIMER3_CH2OUT_0
#else
#  define PWM_TIMER3_CH2CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CH3O
#  define PWM_TIMER3_CH3CFG  GPIO_TIMER3_CH3OUT_0
#else
#  define PWM_TIMER3_CH3CFG  0
#endif

/* TIMER4 channel output configurations */

#ifdef CONFIG_GD32E11X_TIMER4_CH0O
#  define PWM_TIMER4_CH0CFG  GPIO_TIMER4_CH0OUT_0
#else
#  define PWM_TIMER4_CH0CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CH1O
#  define PWM_TIMER4_CH1CFG  GPIO_TIMER4_CH1OUT_0
#else
#  define PWM_TIMER4_CH1CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CH2O
#  define PWM_TIMER4_CH2CFG  GPIO_TIMER4_CH2OUT_0
#else
#  define PWM_TIMER4_CH2CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CH3O
#  define PWM_TIMER4_CH3CFG  GPIO_TIMER4_CH3OUT_0
#else
#  define PWM_TIMER4_CH3CFG  0
#endif

/* TIMER7 channel output configurations */

#ifdef CONFIG_GD32E11X_TIMER7_CH0O
#  define PWM_TIMER7_CH0CFG  GPIO_TIMER7_CH0OUT_0
#else
#  define PWM_TIMER7_CH0CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CH0ON
#  define PWM_TIMER7_CH0NCFG GPIO_TIMER7_CH0OUTN_0
#else
#  define PWM_TIMER7_CH0NCFG 0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CH1O
#  define PWM_TIMER7_CH1CFG  GPIO_TIMER7_CH1OUT_0
#else
#  define PWM_TIMER7_CH1CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CH1ON
#  define PWM_TIMER7_CH1NCFG GPIO_TIMER7_CH1OUTN_0
#else
#  define PWM_TIMER7_CH1NCFG 0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CH2O
#  define PWM_TIMER7_CH2CFG  GPIO_TIMER7_CH2OUT_0
#else
#  define PWM_TIMER7_CH2CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CH2ON
#  define PWM_TIMER7_CH2NCFG GPIO_TIMER7_CH2OUTN_0
#else
#  define PWM_TIMER7_CH2NCFG 0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CH3O
#  define PWM_TIMER7_CH3CFG  GPIO_TIMER7_CH3OUT_0
#else
#  define PWM_TIMER7_CH3CFG  0
#endif

/* TIMER8 channel output configurations (2 channels) */

#ifdef CONFIG_GD32E11X_TIMER8_CH0O
#  define PWM_TIMER8_CH0CFG  GPIO_TIMER8_CH0OUT_0
#else
#  define PWM_TIMER8_CH0CFG  0
#endif
#ifdef CONFIG_GD32E11X_TIMER8_CH1O
#  define PWM_TIMER8_CH1CFG  GPIO_TIMER8_CH1OUT_0
#else
#  define PWM_TIMER8_CH1CFG  0
#endif

/* TIMER9 channel output configurations (1 channel) */

#ifdef CONFIG_GD32E11X_TIMER9_CH0O
#  define PWM_TIMER9_CH0CFG  GPIO_TIMER9_CH0OUT_0
#else
#  define PWM_TIMER9_CH0CFG  0
#endif

/* TIMER10 channel output configurations (1 channel) */

#ifdef CONFIG_GD32E11X_TIMER10_CH0O
#  define PWM_TIMER10_CH0CFG GPIO_TIMER10_CH0OUT_0
#else
#  define PWM_TIMER10_CH0CFG 0
#endif

/* TIMER11 channel output configurations (2 channels) */

#ifdef CONFIG_GD32E11X_TIMER11_CH0O
#  define PWM_TIMER11_CH0CFG GPIO_TIMER11_CH0OUT_0
#else
#  define PWM_TIMER11_CH0CFG 0
#endif
#ifdef CONFIG_GD32E11X_TIMER11_CH1O
#  define PWM_TIMER11_CH1CFG GPIO_TIMER11_CH1OUT_0
#else
#  define PWM_TIMER11_CH1CFG 0
#endif

/* TIMER12 channel output configurations (1 channel) */

#ifdef CONFIG_GD32E11X_TIMER12_CH0O
#  define PWM_TIMER12_CH0CFG GPIO_TIMER12_CH0OUT_0
#else
#  define PWM_TIMER12_CH0CFG 0
#endif

/* TIMER13 channel output configurations (1 channel) */

#ifdef CONFIG_GD32E11X_TIMER13_CH0O
#  define PWM_TIMER13_CH0CFG GPIO_TIMER13_CH0OUT_0
#else
#  define PWM_TIMER13_CH0CFG 0
#endif

/****************************************************************************
 * PWM channel count macros
 *
 * These macros count the number of configured channels per timer.
 * Each CONFIG_GD32E11X_TIMERn_CHANNELx maps to 0 or 1 and the sum
 * produces the total number of configured channels.
 ****************************************************************************/

/* TIMER0 - Advanced, 4 channels */

#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL0
#  define PWM_TIMER0_CHANNEL0 1
#else
#  define PWM_TIMER0_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL1
#  define PWM_TIMER0_CHANNEL1 1
#else
#  define PWM_TIMER0_CHANNEL1 0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL2
#  define PWM_TIMER0_CHANNEL2 1
#else
#  define PWM_TIMER0_CHANNEL2 0
#endif
#ifdef CONFIG_GD32E11X_TIMER0_CHANNEL3
#  define PWM_TIMER0_CHANNEL3 1
#else
#  define PWM_TIMER0_CHANNEL3 0
#endif
#define PWM_TIMER0_NCHANNELS (PWM_TIMER0_CHANNEL0 + PWM_TIMER0_CHANNEL1 + \
                              PWM_TIMER0_CHANNEL2 + PWM_TIMER0_CHANNEL3)

/* TIMER1 - General, 4 channels */

#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL0
#  define PWM_TIMER1_CHANNEL0 1
#else
#  define PWM_TIMER1_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL1
#  define PWM_TIMER1_CHANNEL1 1
#else
#  define PWM_TIMER1_CHANNEL1 0
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL2
#  define PWM_TIMER1_CHANNEL2 1
#else
#  define PWM_TIMER1_CHANNEL2 0
#endif
#ifdef CONFIG_GD32E11X_TIMER1_CHANNEL3
#  define PWM_TIMER1_CHANNEL3 1
#else
#  define PWM_TIMER1_CHANNEL3 0
#endif
#define PWM_TIMER1_NCHANNELS (PWM_TIMER1_CHANNEL0 + PWM_TIMER1_CHANNEL1 + \
                              PWM_TIMER1_CHANNEL2 + PWM_TIMER1_CHANNEL3)

/* TIMER2 - General, 4 channels */

#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL0
#  define PWM_TIMER2_CHANNEL0 1
#else
#  define PWM_TIMER2_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL1
#  define PWM_TIMER2_CHANNEL1 1
#else
#  define PWM_TIMER2_CHANNEL1 0
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL2
#  define PWM_TIMER2_CHANNEL2 1
#else
#  define PWM_TIMER2_CHANNEL2 0
#endif
#ifdef CONFIG_GD32E11X_TIMER2_CHANNEL3
#  define PWM_TIMER2_CHANNEL3 1
#else
#  define PWM_TIMER2_CHANNEL3 0
#endif
#define PWM_TIMER2_NCHANNELS (PWM_TIMER2_CHANNEL0 + PWM_TIMER2_CHANNEL1 + \
                              PWM_TIMER2_CHANNEL2 + PWM_TIMER2_CHANNEL3)

/* TIMER3 - General, 4 channels */

#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL0
#  define PWM_TIMER3_CHANNEL0 1
#else
#  define PWM_TIMER3_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL1
#  define PWM_TIMER3_CHANNEL1 1
#else
#  define PWM_TIMER3_CHANNEL1 0
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL2
#  define PWM_TIMER3_CHANNEL2 1
#else
#  define PWM_TIMER3_CHANNEL2 0
#endif
#ifdef CONFIG_GD32E11X_TIMER3_CHANNEL3
#  define PWM_TIMER3_CHANNEL3 1
#else
#  define PWM_TIMER3_CHANNEL3 0
#endif
#define PWM_TIMER3_NCHANNELS (PWM_TIMER3_CHANNEL0 + PWM_TIMER3_CHANNEL1 + \
                              PWM_TIMER3_CHANNEL2 + PWM_TIMER3_CHANNEL3)

/* TIMER4 - General, 4 channels */

#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL0
#  define PWM_TIMER4_CHANNEL0 1
#else
#  define PWM_TIMER4_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL1
#  define PWM_TIMER4_CHANNEL1 1
#else
#  define PWM_TIMER4_CHANNEL1 0
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL2
#  define PWM_TIMER4_CHANNEL2 1
#else
#  define PWM_TIMER4_CHANNEL2 0
#endif
#ifdef CONFIG_GD32E11X_TIMER4_CHANNEL3
#  define PWM_TIMER4_CHANNEL3 1
#else
#  define PWM_TIMER4_CHANNEL3 0
#endif
#define PWM_TIMER4_NCHANNELS (PWM_TIMER4_CHANNEL0 + PWM_TIMER4_CHANNEL1 + \
                              PWM_TIMER4_CHANNEL2 + PWM_TIMER4_CHANNEL3)

/* TIMER7 - Advanced, 4 channels */

#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL0
#  define PWM_TIMER7_CHANNEL0 1
#else
#  define PWM_TIMER7_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL1
#  define PWM_TIMER7_CHANNEL1 1
#else
#  define PWM_TIMER7_CHANNEL1 0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL2
#  define PWM_TIMER7_CHANNEL2 1
#else
#  define PWM_TIMER7_CHANNEL2 0
#endif
#ifdef CONFIG_GD32E11X_TIMER7_CHANNEL3
#  define PWM_TIMER7_CHANNEL3 1
#else
#  define PWM_TIMER7_CHANNEL3 0
#endif
#define PWM_TIMER7_NCHANNELS (PWM_TIMER7_CHANNEL0 + PWM_TIMER7_CHANNEL1 + \
                              PWM_TIMER7_CHANNEL2 + PWM_TIMER7_CHANNEL3)

/* TIMER8 - General, 2 channels */

#ifdef CONFIG_GD32E11X_TIMER8_CHANNEL0
#  define PWM_TIMER8_CHANNEL0 1
#else
#  define PWM_TIMER8_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER8_CHANNEL1
#  define PWM_TIMER8_CHANNEL1 1
#else
#  define PWM_TIMER8_CHANNEL1 0
#endif
#define PWM_TIMER8_NCHANNELS (PWM_TIMER8_CHANNEL0 + PWM_TIMER8_CHANNEL1)

/* TIMER9 - General, 1 channel */

#ifdef CONFIG_GD32E11X_TIMER9_CHANNEL0
#  define PWM_TIMER9_CHANNEL0 1
#else
#  define PWM_TIMER9_CHANNEL0 0
#endif
#define PWM_TIMER9_NCHANNELS  (PWM_TIMER9_CHANNEL0)

/* TIMER10 - General, 1 channel */

#ifdef CONFIG_GD32E11X_TIMER10_CHANNEL0
#  define PWM_TIMER10_CHANNEL0 1
#else
#  define PWM_TIMER10_CHANNEL0 0
#endif
#define PWM_TIMER10_NCHANNELS (PWM_TIMER10_CHANNEL0)

/* TIMER11 - General, 2 channels */

#ifdef CONFIG_GD32E11X_TIMER11_CHANNEL0
#  define PWM_TIMER11_CHANNEL0 1
#else
#  define PWM_TIMER11_CHANNEL0 0
#endif
#ifdef CONFIG_GD32E11X_TIMER11_CHANNEL1
#  define PWM_TIMER11_CHANNEL1 1
#else
#  define PWM_TIMER11_CHANNEL1 0
#endif
#define PWM_TIMER11_NCHANNELS (PWM_TIMER11_CHANNEL0 + PWM_TIMER11_CHANNEL1)

/* TIMER12 - General, 1 channel */

#ifdef CONFIG_GD32E11X_TIMER12_CHANNEL0
#  define PWM_TIMER12_CHANNEL0 1
#else
#  define PWM_TIMER12_CHANNEL0 0
#endif
#define PWM_TIMER12_NCHANNELS (PWM_TIMER12_CHANNEL0)

/* TIMER13 - General, 1 channel */

#ifdef CONFIG_GD32E11X_TIMER13_CHANNEL0
#  define PWM_TIMER13_CHANNEL0 1
#else
#  define PWM_TIMER13_CHANNEL0 0
#endif
#define PWM_TIMER13_NCHANNELS (PWM_TIMER13_CHANNEL0)

/* Low-level ops helpers ****************************************************/

#ifdef CONFIG_GD32E11X_PWM_LL_OPS

/* NOTE:
 * Low-level ops accept pwm_lowerhalf_s as first argument, but llops access
 * can be found in gd32_pwm_dev_s.
 * These macros use GD32 register naming conventions:
 *   CHxCV - Channel Compare Value
 *   CAR   - Counter Auto-Reload
 *   CREP  - Counter Repetition
 */

#define PWM_SETUP(dev)                                                       \
        (dev)->ops->setup((struct pwm_lowerhalf_s *)dev)
#define PWM_SHUTDOWN(dev)                                                    \
        (dev)->ops->shutdown((struct pwm_lowerhalf_s *)dev)
#define PWM_CHCV_UPDATE(dev, index, chcv)                                    \
        ((struct gd32_pwm_dev_s *)dev)->llops->chcv_update(                  \
                (struct pwm_lowerhalf_s *)dev, index, chcv)
#define PWM_MODE_UPDATE(dev, index, mode)                                    \
        ((struct gd32_pwm_dev_s *)dev)->llops->mode_update(                  \
                (struct pwm_lowerhalf_s *)dev, index, mode)
#define PWM_CHCV_GET(dev, index)                                             \
        ((struct gd32_pwm_dev_s *)dev)->llops->chcv_get(                     \
                (struct pwm_lowerhalf_s *)dev, index)
#define PWM_CAR_UPDATE(dev, car)                                             \
        ((struct gd32_pwm_dev_s *)dev)->llops->car_update(                   \
                (struct pwm_lowerhalf_s *)dev, car)
#define PWM_CAR_GET(dev)                                                     \
        ((struct gd32_pwm_dev_s *)dev)->llops->car_get(                      \
                (struct pwm_lowerhalf_s *)dev)
#define PWM_CREP_UPDATE(dev, crep)                                           \
        ((struct gd32_pwm_dev_s *)dev)->llops->crep_update(                  \
                (struct pwm_lowerhalf_s *)dev, crep)
#define PWM_CREP_GET(dev)                                                    \
        ((struct gd32_pwm_dev_s *)dev)->llops->crep_get(                     \
                (struct pwm_lowerhalf_s *)dev)
#ifdef HAVE_TRGO
#  define PWM_TRGO_SET(dev, trgo)                                            \
        ((struct gd32_pwm_dev_s *)dev)->llops->trgo_set(                     \
                (struct pwm_lowerhalf_s *)dev, trgo)
#endif
#define PWM_OUTPUTS_ENABLE(dev, out, state)                                  \
        ((struct gd32_pwm_dev_s *)dev)->llops->outputs_enable(               \
                (struct pwm_lowerhalf_s *)dev, out, state)
#define PWM_SOFT_UPDATE(dev)                                                 \
        ((struct gd32_pwm_dev_s *)dev)->llops->soft_update(                  \
                (struct pwm_lowerhalf_s *)dev)
#define PWM_CONFIGURE(dev)                                                   \
        ((struct gd32_pwm_dev_s *)dev)->llops->configure(                    \
                (struct pwm_lowerhalf_s *)dev)
#define PWM_SOFT_BREAK(dev, state)                                           \
        ((struct gd32_pwm_dev_s *)dev)->llops->soft_break(                   \
                (struct pwm_lowerhalf_s *)dev, state)
#define PWM_FREQ_UPDATE(dev, freq)                                           \
        ((struct gd32_pwm_dev_s *)dev)->llops->freq_update(                  \
                (struct pwm_lowerhalf_s *)dev, freq)
#define PWM_TIMER_ENABLE(dev, state)                                         \
        ((struct gd32_pwm_dev_s *)dev)->llops->enable(                       \
                (struct pwm_lowerhalf_s *)dev, state)
#ifdef CONFIG_DEBUG_PWM_INFO
#  define PWM_DUMP_REGS(dev, msg)                                            \
        ((struct gd32_pwm_dev_s *)dev)->llops->dumpregs(                     \
                (struct pwm_lowerhalf_s *)dev, msg)
#else
#  define PWM_DUMP_REGS(dev, msg)
#endif
#ifdef HAVE_PWM_COMPLEMENTARY
#  define PWM_DT_UPDATE(dev, dt)                                             \
        ((struct gd32_pwm_dev_s *)dev)->llops->dt_update(                    \
                (struct pwm_lowerhalf_s *)dev, dt)
#endif

#endif /* CONFIG_GD32E11X_PWM_LL_OPS */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Timer mode values (counting direction) */

enum gd32_pwm_timer_mode_e
{
        GD32_PWM_TIMER_MODE_COUNTUP      = 0, /* Edge-aligned, up counting */
        GD32_PWM_TIMER_MODE_COUNTDOWN    = 1, /* Edge-aligned, down counting */
        GD32_PWM_TIMER_MODE_CENTER_DOWN  = 2, /* Center-aligned, down counting trigger */
        GD32_PWM_TIMER_MODE_CENTER_UP    = 3, /* Center-aligned, up counting trigger */
        GD32_PWM_TIMER_MODE_CENTER_BOTH  = 4, /* Center-aligned, both direction trigger */
};

/* Channel mode values (output compare mode)
 * Reference: GD32 TIMER_OC_MODE definitions in gd32e113_timer.h
 * Maps to CHxCOMCTL[6:4] bits in CHCTLx register (values 0-7)
 */

enum gd32_pwm_chanmode_e
{
  GD32_CHANMODE_TIMING    = 0,  /* Timing mode - no effect on outputs */
  GD32_CHANMODE_ACTIVE    = 1,  /* Active on match */
  GD32_CHANMODE_INACTIVE  = 2,  /* Inactive on match */
  GD32_CHANMODE_TOGGLE    = 3,  /* Toggle on match */
  GD32_CHANMODE_FORCE_LO  = 4,  /* Force low */
  GD32_CHANMODE_FORCE_HI  = 5,  /* Force high */
  GD32_CHANMODE_PWM0      = 6,  /* PWM mode 0 (active when CNT < CHxCV) */
  GD32_CHANMODE_PWM1      = 7,  /* PWM mode 1 (active when CNT >= CHxCV) */
};

/* Output polarity */

enum gd32_pwm_pol_e
{
  GD32_POL_ACTIVE_HIGH    = 0,  /* Active high polarity */
  GD32_POL_ACTIVE_LOW     = 1,  /* Active low polarity */
};

/* Idle state */

enum gd32_pwm_idle_e
{
  GD32_IDLE_INACTIVE      = 0,  /* Inactive during idle */
  GD32_IDLE_ACTIVE        = 1,  /* Active during idle */
};

/* PWM timer channel numbers (0-based indexing per GD32 convention) */

enum gd32_pwm_chan_e
{
  GD32_PWM_CHAN0  = 0,
  GD32_PWM_CHAN1  = 1,
  GD32_PWM_CHAN2  = 2,
  GD32_PWM_CHAN3  = 3,
};

/* PWM timer channel output bit flags.
 * Used with pwm_outputs_enable() to control specific outputs.
 * For advanced timers (TIMER0/7), complementary outputs (CHxON) exist
 * only for channels 0-2.
 */

enum gd32_pwm_output_e
{
  GD32_PWM_OUT0   = (1 << 0),   /* CH0 output */
  GD32_PWM_OUT0N  = (1 << 1),   /* CH0 complementary output (advanced only) */
  GD32_PWM_OUT1   = (1 << 2),   /* CH1 output */
  GD32_PWM_OUT1N  = (1 << 3),   /* CH1 complementary output (advanced only) */
  GD32_PWM_OUT2   = (1 << 4),   /* CH2 output */
  GD32_PWM_OUT2N  = (1 << 5),   /* CH2 complementary output (advanced only) */
  GD32_PWM_OUT3   = (1 << 6),   /* CH3 output (no complementary) */

  /* Bit 7 reserved - CH3 has no complementary output */
};

/* PWM output configuration */

struct gd32_pwm_out_s
{
  uint8_t  in_use : 1;          /* Output is in use */
  uint8_t  pol    : 1;          /* Output polarity */
  uint8_t  idle   : 1;          /* Idle state */
  uint8_t  _res   : 5;          /* Reserved */
  uint32_t pincfg;              /* GPIO pin configuration */
};

/* PWM break configuration (advanced timers only) */

#ifdef HAVE_BREAK
struct gd32_pwm_break_s
{
  uint8_t  en   : 1;            /* Break enable */
  uint8_t  pol  : 1;            /* Break polarity (0=active low, 1=active high) */
  uint8_t  _res : 6;            /* Reserved */
};
#endif

/* PWM channel configuration */

struct gd32_pwmchan_s
{
  uint8_t                channel : 4;  /* Timer channel number: {0..3} */
  uint8_t                mode    : 4;  /* Channel mode (see chanmode_e) */
  struct gd32_pwm_out_s  out1;         /* PWM main output */
#ifdef HAVE_PWM_COMPLEMENTARY
  struct gd32_pwm_out_s  out2;         /* PWM complementary output */
#endif
};

/* PWM timer state structure (opaque - defined in .c file) */

struct gd32_pwmtimer_s;

#ifdef CONFIG_GD32E11X_PWM_LL_OPS
/****************************************************************************
 * Low-level Operations
 *
 * This structure provides low-level access to the timer registers for
 * advanced usage scenarios.
 ****************************************************************************/

struct gd32_pwm_ops_s
{
  /* Update CHxCV (channel compare value) register */

  int (*chcv_update)(struct pwm_lowerhalf_s *dev, uint8_t index,
                     uint32_t chcv);

  /* Update channel output compare mode */

  int (*mode_update)(struct pwm_lowerhalf_s *dev, uint8_t index,
                     uint32_t mode);

  /* Get current CHxCV value */

  uint32_t (*chcv_get)(struct pwm_lowerhalf_s *dev, uint8_t index);

  /* Update CAR (counter auto-reload) register */

  int (*car_update)(struct pwm_lowerhalf_s *dev, uint32_t car);

  /* Get current CAR value */

  uint32_t (*car_get)(struct pwm_lowerhalf_s *dev);

  /* Update CREP (counter repetition) register - advanced timers only */

  int (*crep_update)(struct pwm_lowerhalf_s *dev, uint16_t crep);

  /* Get current CREP value */

  uint16_t (*crep_get)(struct pwm_lowerhalf_s *dev);

#ifdef HAVE_TRGO
  /* Set TRGO output source */

  int (*trgo_set)(struct pwm_lowerhalf_s *dev, uint8_t trgo);
#endif

  /* Enable/disable outputs */

  int (*outputs_enable)(struct pwm_lowerhalf_s *dev, uint16_t outputs,
                        bool state);

  /* Generate software update event */

  int (*soft_update)(struct pwm_lowerhalf_s *dev);

  /* Configure timer (initial setup) */

  int (*configure)(struct pwm_lowerhalf_s *dev);

  /* Enable/disable software break (primary output enable control) */

  int (*soft_break)(struct pwm_lowerhalf_s *dev, bool state);

  /* Update PWM frequency */

  int (*freq_update)(struct pwm_lowerhalf_s *dev, uint32_t frequency);

  /* Enable/disable timer counter */

  int (*enable)(struct pwm_lowerhalf_s *dev, bool state);

#ifdef CONFIG_DEBUG_PWM_INFO
  /* Dump timer registers */

  void (*dumpregs)(struct pwm_lowerhalf_s *dev, const char *msg);
#endif

#ifdef HAVE_PWM_COMPLEMENTARY
  /* Update dead-time value */

  int (*dt_update)(struct pwm_lowerhalf_s *dev, uint8_t dt);
#endif
};

/* This structure provides the publicly visible representation of the
 * "lower-half" PWM driver structure for low-level operations access.
 */

struct gd32_pwm_dev_s
{
  /* The first field of this state structure must be a pointer to the PWM
   * callback structure to be consistent with upper-half PWM driver.
   */

  const struct pwm_ops_s *ops;

  /* Publicly visible portion of the "lower-half" PWM driver structure.
   * This provides access to low-level timer register operations.
   */

  const struct gd32_pwm_ops_s *llops;

  /* Require cast-compatibility with private gd32_pwmtimer_s structure */
};

#endif /* CONFIG_GD32E11X_PWM_LL_OPS */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper-half PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the GD32 MCU (TIMER0..TIMER4, TIMER7..TIMER13).
 *     Note: TIMER5 and TIMER6 are basic timers without PWM capability.
 *
 * Returned Value:
 *   On success, a pointer to the GD32 lower-half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *gd32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_PWM_H */
