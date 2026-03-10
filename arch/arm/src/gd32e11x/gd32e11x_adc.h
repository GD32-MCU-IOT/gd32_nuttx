/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_adc.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_ADC_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "chip.h"
#include "hardware/gd32e11x_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Up to 2 ADC interfaces are supported on GD32E11x */

#if GD32_NADC < 2
#  undef CONFIG_GD32E11X_ADC1
#endif

#if GD32_NADC < 1
#  undef CONFIG_GD32E11X_ADC0
#endif

#if defined(CONFIG_GD32E11X_ADC0) || defined(CONFIG_GD32E11X_ADC1)

/* DMA support */

#undef ADC_HAVE_DMA
#if defined(CONFIG_GD32E11X_ADC0_DMA) || defined(CONFIG_GD32E11X_ADC1_DMA)
#  define ADC_HAVE_DMA  1
#endif

#ifdef CONFIG_GD32E11X_ADC0_DMA
#  define ADC0_HAVE_DMA 1
#else
#  undef  ADC0_HAVE_DMA
#endif

#ifdef CONFIG_GD32E11X_ADC1_DMA
#  define ADC1_HAVE_DMA 1
#else
#  undef  ADC1_HAVE_DMA
#endif

/* Injected channels support */

#if (defined(CONFIG_GD32E11X_ADC0) && defined(CONFIG_GD32E11X_ADC0_INJECTED_CHAN) && (CONFIG_GD32E11X_ADC0_INJECTED_CHAN > 0)) || \
    (defined(CONFIG_GD32E11X_ADC1) && defined(CONFIG_GD32E11X_ADC1_INJECTED_CHAN) && (CONFIG_GD32E11X_ADC1_INJECTED_CHAN > 0))
#  define ADC_HAVE_INJECTED
#endif

/* Timer configuration: If a timer trigger is specified, get info about it */

#if defined(CONFIG_GD32E11X_TIMER0_ADC0)
#  define ADC0_HAVE_TIMER           1
#  define ADC0_TIMER_BASE           GD32_TIMER0_BASE
#  define ADC0_TIMER_PCLK_FREQUENCY GD32_APB2_TIM0_CLKIN
#elif defined(CONFIG_GD32E11X_TIMER1_ADC0)
#  define ADC0_HAVE_TIMER           1
#  define ADC0_TIMER_BASE           GD32_TIMER1_BASE
#  define ADC0_TIMER_PCLK_FREQUENCY GD32_APB1_TIM1_CLKIN
#elif defined(CONFIG_GD32E11X_TIMER2_ADC0)
#  define ADC0_HAVE_TIMER           1
#  define ADC0_TIMER_BASE           GD32_TIMER2_BASE
#  define ADC0_TIMER_PCLK_FREQUENCY GD32_APB1_TIM2_CLKIN
#elif defined(CONFIG_GD32E11X_TIMER3_ADC0)
#  define ADC0_HAVE_TIMER           1
#  define ADC0_TIMER_BASE           GD32_TIMER3_BASE
#  define ADC0_TIMER_PCLK_FREQUENCY GD32_APB1_TIM3_CLKIN
#else
#  undef  ADC0_HAVE_TIMER
#endif

#if defined(CONFIG_GD32E11X_TIMER0_ADC1)
#  define ADC1_HAVE_TIMER           1
#  define ADC1_TIMER_BASE           GD32_TIMER0_BASE
#  define ADC1_TIMER_PCLK_FREQUENCY GD32_APB2_TIM0_CLKIN
#elif defined(CONFIG_GD32E11X_TIMER1_ADC1)
#  define ADC1_HAVE_TIMER           1
#  define ADC1_TIMER_BASE           GD32_TIMER1_BASE
#  define ADC1_TIMER_PCLK_FREQUENCY GD32_APB1_TIM1_CLKIN
#elif defined(CONFIG_GD32E11X_TIMER2_ADC1)
#  define ADC1_HAVE_TIMER           1
#  define ADC1_TIMER_BASE           GD32_TIMER2_BASE
#  define ADC1_TIMER_PCLK_FREQUENCY GD32_APB1_TIM2_CLKIN
#elif defined(CONFIG_GD32E11X_TIMER3_ADC1)
#  define ADC1_HAVE_TIMER           1
#  define ADC1_TIMER_BASE           GD32_TIMER3_BASE
#  define ADC1_TIMER_PCLK_FREQUENCY GD32_APB1_TIM3_CLKIN
#else
#  undef  ADC1_HAVE_TIMER
#endif

#if defined(ADC0_HAVE_TIMER) || defined(ADC1_HAVE_TIMER)
#  define ADC_HAVE_TIMER 1
#else
#  undef ADC_HAVE_TIMER
#endif

/* Timer trigger for regular channels:
 * ADC_CTL1 ETSRC[2:0] encodes the external trigger source.
 * Values map from trigger select index to ETSRC value.
 * 0=T0CH0, 1=T0CH1, 2=T0CH2, 3=T1CH1, 4=T2TRGO, 5=T3CH3, 6=EXTI11, 7=SW
 */

#define ADC_EXTSEL_T0CH0             ADC_CTL1_ETSRC_T0CH0
#define ADC_EXTSEL_T0CH1             ADC_CTL1_ETSRC_T0CH1
#define ADC_EXTSEL_T0CH2             ADC_CTL1_ETSRC_T0CH2
#define ADC_EXTSEL_T1CH1             ADC_CTL1_ETSRC_T1CH1
#define ADC_EXTSEL_T2TRGO            ADC_CTL1_ETSRC_T2TRGO
#define ADC_EXTSEL_T3CH3             ADC_CTL1_ETSRC_T3CH3
#define ADC_EXTSEL_EXTI11            ADC_CTL1_ETSRC_EXTI11
#define ADC_EXTSEL_NONE              ADC_CTL1_ETSRC_NONE

/* ADC0 external trigger source based on timer configuration */

#if defined(ADC0_HAVE_TIMER)
#  if defined(CONFIG_GD32E11X_TIMER0_ADC0)
#    if CONFIG_GD32E11X_ADC0_TIMERTRIG == 0
#      define ADC0_EXTSEL_VALUE       ADC_CTL1_ETSRC_T0CH0
#    elif CONFIG_GD32E11X_ADC0_TIMERTRIG == 1
#      define ADC0_EXTSEL_VALUE       ADC_CTL1_ETSRC_T0CH1
#    elif CONFIG_GD32E11X_ADC0_TIMERTRIG == 2
#      define ADC0_EXTSEL_VALUE       ADC_CTL1_ETSRC_T0CH2
#    else
#      error "Invalid ADC0 timer trigger selection for TIMER0"
#    endif
#  elif defined(CONFIG_GD32E11X_TIMER1_ADC0)
#    define ADC0_EXTSEL_VALUE         ADC_CTL1_ETSRC_T1CH1
#  elif defined(CONFIG_GD32E11X_TIMER2_ADC0)
#    define ADC0_EXTSEL_VALUE         ADC_CTL1_ETSRC_T2TRGO
#  elif defined(CONFIG_GD32E11X_TIMER3_ADC0)
#    define ADC0_EXTSEL_VALUE         ADC_CTL1_ETSRC_T3CH3
#  else
#    define ADC0_EXTSEL_VALUE         ADC_CTL1_ETSRC_NONE
#  endif
#else
#  define ADC0_EXTSEL_VALUE           ADC_CTL1_ETSRC_NONE
#endif

/* ADC1 external trigger source based on timer configuration */

#if defined(ADC1_HAVE_TIMER)
#  if defined(CONFIG_GD32E11X_TIMER0_ADC1)
#    if CONFIG_GD32E11X_ADC1_TIMERTRIG == 0
#      define ADC1_EXTSEL_VALUE       ADC_CTL1_ETSRC_T0CH0
#    elif CONFIG_GD32E11X_ADC1_TIMERTRIG == 1
#      define ADC1_EXTSEL_VALUE       ADC_CTL1_ETSRC_T0CH1
#    elif CONFIG_GD32E11X_ADC1_TIMERTRIG == 2
#      define ADC1_EXTSEL_VALUE       ADC_CTL1_ETSRC_T0CH2
#    else
#      error "Invalid ADC1 timer trigger selection for TIMER0"
#    endif
#  elif defined(CONFIG_GD32E11X_TIMER1_ADC1)
#    define ADC1_EXTSEL_VALUE         ADC_CTL1_ETSRC_T1CH1
#  elif defined(CONFIG_GD32E11X_TIMER2_ADC1)
#    define ADC1_EXTSEL_VALUE         ADC_CTL1_ETSRC_T2TRGO
#  elif defined(CONFIG_GD32E11X_TIMER3_ADC1)
#    define ADC1_EXTSEL_VALUE         ADC_CTL1_ETSRC_T3CH3
#  else
#    define ADC1_EXTSEL_VALUE         ADC_CTL1_ETSRC_NONE
#  endif
#else
#  define ADC1_EXTSEL_VALUE           ADC_CTL1_ETSRC_NONE
#endif

/* ADC DMA channel mappings (GD32E11x):
 *   ADC0 -> DMA0 Channel 0 (DMA_REQ_ADC0)
 *   ADC1 -> No dedicated DMA channel (must use ADC0 dual mode RDATA upper
 *           half for ADC1 data, or use ADC0 DMA)
 */

#ifdef ADC0_HAVE_DMA
#  define ADC0_DMA_CHAN              DMA_REQ_ADC0
#endif

/* ADC resolution is configurable via OVSAMPCTL on GD32E11x */

#define HAVE_ADC_RESOLUTION  1

/* Max samples per ADC conversion sequence */

#ifndef CONFIG_GD32E11X_ADC_MAX_SAMPLES
#  define CONFIG_GD32E11X_ADC_MAX_SAMPLES 16
#endif

/* IRQ: GD32E11x shares one IRQ for ADC0 and ADC1 */

#define GD32_ADC_IRQ                 GD32_IRQ_ADC0_1

/* ADC scan mode: always supported on GD32E11x */

#define ADC_HAVE_SCAN  1

#ifndef CONFIG_GD32E11X_ADC0_SCAN
#  define CONFIG_GD32E11X_ADC0_SCAN 0
#endif

#ifndef CONFIG_GD32E11X_ADC1_SCAN
#  define CONFIG_GD32E11X_ADC1_SCAN 0
#endif

/* ADC injected channels max count */

#define ADC_INJ_MAX_SAMPLES          4

/* Number of channels per ADC on GD32E11x */

#define ADC_CHANNELS_NUMBER          GD32_ADC_CHANNELS_NUMBER

/* GD32E11x ADC driver-specific ioctl commands.
 * These use the platform-specific ANIOC range to avoid conflicts
 * with the standard NuttX analog ioctl commands.
 */

#define IO_ENABLE_TEMPER_VOLT_CH     _ANIOC(AN_FIRST + AN_NCMDS + 0)
#define IO_STOP_ADC                  _ANIOC(AN_FIRST + AN_NCMDS + 1)
#define IO_START_ADC                 _ANIOC(AN_FIRST + AN_NCMDS + 2)
#define IO_START_CONV                _ANIOC(AN_FIRST + AN_NCMDS + 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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

/****************************************************************************
 * Name: gd32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   intf      - ADC number, 0 or 1 (ADC0 or ADC1)
 *   chanlist  - Pointer to the list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *gd32_adc_initialize(int intf, const uint8_t *chanlist,
                                      int nchannels);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_GD32E11X_ADC0 || CONFIG_GD32E11X_ADC1 */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_ADC_H */
