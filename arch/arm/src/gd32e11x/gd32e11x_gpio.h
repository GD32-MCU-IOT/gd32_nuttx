/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_gpio.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_GPIO_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_GPIO_H

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/* Bit-encoded GPIO configuration (STM32F1/GD32E11x style).
 * Encoding (16-bit):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * OFFS SX.. VPPP BBBB
 */

/* Input/output select */

#define GPIO_INPUT                    (1 << 15)                  /* 1=input mode */
#define GPIO_OUTPUT                   (0)
#define GPIO_ALT                      (0)

/* Output set/clear (also used for input pull-up/down selection) */

#define GPIO_OUTPUT_SET               (1 << 7)
#define GPIO_OUTPUT_CLEAR             (0)

/* CNF bits (function) */

#define GPIO_CNF_SHIFT                13
#define GPIO_CNF_MASK                 (3 << GPIO_CNF_SHIFT)

#  define GPIO_CNF_ANALOGIN           (0 << GPIO_CNF_SHIFT)
#  define GPIO_CNF_INFLOAT            (1 << GPIO_CNF_SHIFT)
#  define GPIO_CNF_INPULLUD           (2 << GPIO_CNF_SHIFT)
#  define GPIO_CNF_INPULLDWN          (2 << GPIO_CNF_SHIFT)
#  define GPIO_CNF_INPULLUP           ((2 << GPIO_CNF_SHIFT) | GPIO_OUTPUT_SET)

#  define GPIO_CNF_OUTPP              (0 << GPIO_CNF_SHIFT)
#  define GPIO_CNF_OUTOD              (1 << GPIO_CNF_SHIFT)
#  define GPIO_CNF_AFPP               (2 << GPIO_CNF_SHIFT)
#  define GPIO_CNF_AFOD               (3 << GPIO_CNF_SHIFT)

/* MODE bits (speed) */

#define GPIO_MODE_SHIFT               11
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INPUT             (0 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_10MHz             (1 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_2MHz              (2 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_50MHz             (3 << GPIO_MODE_SHIFT)

/* Compatibility spelling used by some pinmap headers */

#  define GPIO_MODE_50MHZ             GPIO_MODE_50MHz

/* EXTI selection */

#define GPIO_EXTI                     (1 << 10)

/* Port selection */

#define GPIO_PORT_SHIFT               4
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)

/* Pin selection */

#define GPIO_PIN_SHIFT                0
#define GPIO_PIN_MASK                 (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN0                     (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1                     (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2                     (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3                     (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4                     (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5                     (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6                     (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7                     (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8                     (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9                     (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10                    (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11                    (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12                    (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13                    (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14                    (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15                    (15 << GPIO_PIN_SHIFT)

#ifndef __ASSEMBLY__

int  gd32_configgpio(uint32_t cfgset);
int  gd32_unconfiggpio(uint32_t cfgset);
void gd32_gpiowrite(uint32_t cfgset, bool value);

/* Compatibility wrappers used by the GD32F4-style lowputc logic */

static inline int gd32_gpio_config(uint32_t cfgset)
{
  return gd32_configgpio(cfgset);
}

static inline void gd32_gpio_write(uint32_t cfgset, bool value)
{
  gd32_gpiowrite(cfgset, value);
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_GPIO_H */
