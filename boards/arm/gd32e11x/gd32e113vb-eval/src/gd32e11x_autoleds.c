/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_autoleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "gd32e11x.h"
#include "gd32e113v_eval.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definitions map the encoded LED setting to bit sets used to
 * manipulate the LEDs.
 */

#define ON_SETBITS_SHIFT  (0)
#define ON_CLRBITS_SHIFT  (4)
#define OFF_SETBITS_SHIFT (8)
#define OFF_CLRBITS_SHIFT (12)

#define ON_BITS(v)        ((v) & 0x0f)
#define OFF_BITS(v)       (((v) >> 8) & 0x0f)
#define SETBITS(b)        ((b) & 0x0f)
#define CLRBITS(b)        (((b) >> 4) & 0x0f)

#define ON_SETBITS(v)     (SETBITS(ON_BITS(v))
#define ON_CLRBITS(v)     (CLRBITS(ON_BITS(v))
#define OFF_SETBITS(v)    (SETBITS(OFF_BITS(v))
#define OFF_CLRBITS(v)    (CLRBITS(OFF_BITS(v))

#define LED_STARTED_ON_SETBITS       ((0) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((BOARD_LED1_BIT) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((BOARD_LED2_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS ((BOARD_LED2_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT) << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((BOARD_LED1_BIT) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((BOARD_LED2_BIT) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((BOARD_LED1_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((BOARD_LED1_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((BOARD_LED2_BIT) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((BOARD_LED3_BIT) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED4_BIT) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED4_BIT) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((BOARD_LED3_BIT) << OFF_CLRBITS_SHIFT)

#define LED_INIRQ_ON_SETBITS         ((BOARD_LED1_BIT | BOARD_LED2_BIT) << ON_SETBITS_SHIFT)
#define LED_INIRQ_ON_CLRBITS         ((BOARD_LED3_BIT | BOARD_LED4_BIT) << ON_CLRBITS_SHIFT)
#define LED_INIRQ_OFF_SETBITS        ((BOARD_LED3_BIT | BOARD_LED4_BIT) << OFF_SETBITS_SHIFT)
#define LED_INIRQ_OFF_CLRBITS        ((BOARD_LED1_BIT | BOARD_LED2_BIT) << OFF_CLRBITS_SHIFT)

#define LED_SIGNAL_ON_SETBITS        ((BOARD_LED1_BIT | BOARD_LED3_BIT) << ON_SETBITS_SHIFT)
#define LED_SIGNAL_ON_CLRBITS        ((BOARD_LED2_BIT | BOARD_LED4_BIT) << ON_CLRBITS_SHIFT)
#define LED_SIGNAL_OFF_SETBITS       ((BOARD_LED2_BIT | BOARD_LED4_BIT) << OFF_SETBITS_SHIFT)
#define LED_SIGNAL_OFF_CLRBITS       ((BOARD_LED1_BIT | BOARD_LED3_BIT) << OFF_CLRBITS_SHIFT)

#define LED_ASSERTION_ON_SETBITS     ((BOARD_LED2_BIT | BOARD_LED3_BIT) << ON_SETBITS_SHIFT)
#define LED_ASSERTION_ON_CLRBITS     ((BOARD_LED1_BIT | BOARD_LED4_BIT) << ON_CLRBITS_SHIFT)
#define LED_ASSERTION_OFF_SETBITS    ((BOARD_LED1_BIT | BOARD_LED4_BIT) << OFF_SETBITS_SHIFT)
#define LED_ASSERTION_OFF_CLRBITS    ((BOARD_LED2_BIT | BOARD_LED3_BIT) << OFF_CLRBITS_SHIFT)

#define LED_PANIC_ON_SETBITS         ((BOARD_LED2_BIT | BOARD_LED3_BIT | BOARD_LED4_BIT) << ON_SETBITS_SHIFT)
#define LED_PANIC_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_PANIC_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_PANIC_OFF_CLRBITS        ((BOARD_LED1_BIT) << OFF_CLRBITS_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[8] =
{
  (LED_STARTED_ON_SETBITS       | LED_STARTED_ON_CLRBITS |
   LED_STARTED_OFF_SETBITS      | LED_STARTED_OFF_CLRBITS),

  (LED_HEAPALLOCATE_ON_SETBITS  | LED_HEAPALLOCATE_ON_CLRBITS |
   LED_HEAPALLOCATE_OFF_SETBITS | LED_HEAPALLOCATE_OFF_CLRBITS),

  (LED_IRQSENABLED_ON_SETBITS   | LED_IRQSENABLED_ON_CLRBITS |
   LED_IRQSENABLED_OFF_SETBITS  | LED_IRQSENABLED_OFF_CLRBITS),

  (LED_STACKCREATED_ON_SETBITS  | LED_STACKCREATED_ON_CLRBITS |
   LED_STACKCREATED_OFF_SETBITS | LED_STACKCREATED_OFF_CLRBITS),

  (LED_INIRQ_ON_SETBITS         | LED_INIRQ_ON_CLRBITS |
   LED_INIRQ_OFF_SETBITS        | LED_INIRQ_OFF_CLRBITS),

  (LED_SIGNAL_ON_SETBITS        | LED_SIGNAL_ON_CLRBITS |
   LED_SIGNAL_OFF_SETBITS       | LED_SIGNAL_OFF_CLRBITS),

  (LED_ASSERTION_ON_SETBITS     | LED_ASSERTION_ON_CLRBITS |
   LED_ASSERTION_OFF_SETBITS    | LED_ASSERTION_OFF_CLRBITS),

  (LED_PANIC_ON_SETBITS         | LED_PANIC_ON_CLRBITS |
   LED_PANIC_OFF_SETBITS        | LED_PANIC_OFF_CLRBITS)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED1, false);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED2, false);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED3, false);
    }

  if ((clrbits & BOARD_LED4_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED4, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & BOARD_LED1_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED1, true);
    }

  if ((setbits & BOARD_LED2_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED2, true);
    }

  if ((setbits & BOARD_LED3_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED3, true);
    }

  if ((setbits & BOARD_LED4_BIT) != 0)
    {
      gd32_gpio_write(GPIO_LED4, true);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED1-4 GPIOs for output */

  gd32_gpio_config(GPIO_LED1);
  gd32_gpio_config(GPIO_LED2);
  gd32_gpio_config(GPIO_LED3);
  gd32_gpio_config(GPIO_LED4);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_clrbits(CLRBITS(ON_BITS(g_ledbits[led])));
  led_setbits(SETBITS(ON_BITS(g_ledbits[led])));
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_clrbits(CLRBITS(OFF_BITS(g_ledbits[led])));
  led_setbits(SETBITS(OFF_BITS(g_ledbits[led])));
}

#endif /* CONFIG_ARCH_LEDS */
