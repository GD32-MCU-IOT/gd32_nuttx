/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_userleds.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "gd32e11x.h"
#include "gd32e113v_eval.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED1-4 GPIOs for output */

  gd32_gpio_config(GPIO_LED1);
  gd32_gpio_config(GPIO_LED2);
  gd32_gpio_config(GPIO_LED3);
  gd32_gpio_config(GPIO_LED4);
  return BOARD_LEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  switch (led)
    {
      case BOARD_LED1:
        gd32_gpio_write(GPIO_LED1, ledon);
        break;

      case BOARD_LED2:
        gd32_gpio_write(GPIO_LED2, ledon);
        break;

      case BOARD_LED3:
        gd32_gpio_write(GPIO_LED3, ledon);
        break;

      case BOARD_LED4:
        gd32_gpio_write(GPIO_LED4, ledon);
        break;
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  gd32_gpio_write(GPIO_LED1, (ledset & BOARD_LED1_BIT) != 0);
  gd32_gpio_write(GPIO_LED2, (ledset & BOARD_LED2_BIT) != 0);
  gd32_gpio_write(GPIO_LED3, (ledset & BOARD_LED3_BIT) != 0);
  gd32_gpio_write(GPIO_LED4, (ledset & BOARD_LED4_BIT) != 0);
}

#endif /* !CONFIG_ARCH_LEDS */
