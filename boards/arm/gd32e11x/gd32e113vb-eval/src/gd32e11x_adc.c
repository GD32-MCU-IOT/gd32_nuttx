/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_adc.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>

#include "gd32e11x.h"
#include "gd32e11x_adc.h"

#include "gd32e113v_eval.h"

#if defined(CONFIG_ADC) && \
    (defined(CONFIG_GD32E11X_ADC0) || defined(CONFIG_GD32E11X_ADC1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* 1 or 2 ADC devices (DEV1, DEV2) */

#if defined(CONFIG_GD32E11X_ADC0)
#  define DEV1_PORT 0
#endif

#if defined(CONFIG_GD32E11X_ADC1)
#  if defined(DEV1_PORT)
#    define DEV2_PORT 1
#  else
#    define DEV1_PORT 1
#  endif
#endif

/* The number of ADC channels in the conversion list */

#define ADC0_NCHANNELS 4
#define ADC1_NCHANNELS 3

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DEV 1 */

#if DEV1_PORT == 0

#define DEV1_NCHANNELS ADC0_NCHANNELS

/* Identifying number of each ADC channel.
 * GD32E11x ADC0 channels:
 *   CH0  - PA0
 *   CH1  - PA1
 *   CH4  - PA4
 *   CH13 - PC3
 */

static const uint8_t g_chanlist1[ADC0_NCHANNELS] =
{
  0,
  1,
  4,
  13
};

/* Configurations of pins used by each ADC channel.
 * Analog input mode: GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN
 */

static const uint32_t g_pinlist1[ADC0_NCHANNELS] =
{
  GPIO_ADC0_IN0,                  /* PA0 - ADC0 Channel 0 */
  GPIO_ADC0_IN1,                  /* PA1 - ADC0 Channel 1 */
  GPIO_ADC0_IN4,                  /* PA4 - ADC0 Channel 4 */
  GPIO_ADC01_IN13                 /* PC3 - ADC0 Channel 13 */
};

#elif DEV1_PORT == 1

#define DEV1_NCHANNELS ADC1_NCHANNELS

/* Identifying number of each ADC channel.
 * GD32E11x ADC1 channels:
 *   CH8  - PB0
 *   CH10 - PC0
 *   CH11 - PC1
 */

static const uint8_t g_chanlist1[ADC1_NCHANNELS] =
{
  8,
  10,
  11
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[ADC1_NCHANNELS] =
{
  GPIO_ADC1_IN8,                  /* PB0 - ADC1 Channel 8 */
  GPIO_ADC1_IN10,                 /* PC0 - ADC1 Channel 10 */
  GPIO_ADC1_IN11,                 /* PC1 - ADC1 Channel 11 */
};

#endif /* DEV1_PORT == 0 */

#ifdef DEV2_PORT

/* DEV 2 */

#if DEV2_PORT == 1

#define DEV2_NCHANNELS ADC1_NCHANNELS

/* Identifying number of each ADC channel */

static const uint8_t g_chanlist2[ADC1_NCHANNELS] =
{
  8,
  10,
  11
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist2[ADC1_NCHANNELS] =
{
  GPIO_ADC1_IN8,                  /* PB0 - ADC1 Channel 8 */
  GPIO_ADC1_IN10,                 /* PC0 - ADC1 Channel 10 */
  GPIO_ADC1_IN11,                 /* PC1 - ADC1 Channel 11 */
};

#endif /* DEV2_PORT == 1 */
#endif /* DEV2_PORT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int gd32_adc_setup(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* DEV1 */

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < DEV1_NCHANNELS; i++)
        {
          gd32_gpio_config(g_pinlist1[i]);
        }

      /* Call gd32_adc_initialize() to get an instance of the ADC
       * interface
       */

      adc = gd32_adc_initialize(DEV1_PORT, g_chanlist1, DEV1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface %d\n", DEV1_PORT);
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register /dev/adc0 failed: %d\n", ret);
          return ret;
        }

#ifdef DEV2_PORT

      /* DEV2 */

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < DEV2_NCHANNELS; i++)
        {
          gd32_gpio_config(g_pinlist2[i]);
        }

      /* Call gd32_adc_initialize() to get an instance of the ADC
       * interface
       */

      adc = gd32_adc_initialize(DEV2_PORT, g_chanlist2, DEV2_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface %d\n", DEV2_PORT);
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc1" */

      ret = adc_register("/dev/adc1", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register /dev/adc1 failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_ADC && (CONFIG_GD32E11X_ADC0 || CONFIG_GD32E11X_ADC1) */
