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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include "chip.h"
#include "hardware/gd32e11x_gpio.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Bit-encoded input to gd32_configgpio() */

/* Bit-encoded GPIO configuration (GD32E11x style, 16-bit).
 *
 * Encoding:    1111 1100 0000 0000
 *              5432 1098 7654 3210
 * ENCODING     OFFS SAX. VPPP BBBB
 *
 */

/* Output/Input mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * O... .... .... ....
 */

#define GPIO_CFG_INPUT                (1 << 15)               /* Bit15: 1=input mode */
#define GPIO_CFG_OUTPUT               (0)                     /*        0=output mode */
#define GPIO_CFG_AF                   (0)                     /*        0=alternate function mode */

/* Output set/clear */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.  If the pin is an input, this bit is overloaded to
 * provide the qualifier to\ distinguish input pull-up and -down:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... V... ....
 */

#define GPIO_CFG_OUTPUT_SET           (1 << 7)
#define GPIO_CFG_OUTPUT_CLEAR         (0)

/* These bits set the primary function of the pin:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .FF. .... .... ....
 */

#define GPIO_CFG_CTL_SHIFT            13                               /* Bits 13-14: GPIO pin control configuration */
#define GPIO_CFG_CTL_MASK             (3 << GPIO_CFG_CTL_SHIFT)

#  define GPIO_CFG_CTL_AIN            (0 << GPIO_CFG_CTL_SHIFT)       /* Analog input mode */
#  define GPIO_CFG_CTL_INFLOAT        (1 << GPIO_CFG_CTL_SHIFT)       /* Input floating mode */
#  define GPIO_CFG_CTL_IPUD           (2 << GPIO_CFG_CTL_SHIFT)       /* Input pull-up/down general bit, since up is composed of two parts */
#  define GPIO_CFG_CTL_IPD            (2 << GPIO_CFG_CTL_SHIFT)       /* Input pull-down mode */
#  define GPIO_CFG_CTL_IPU            ((2 << GPIO_CFG_CTL_SHIFT) \
                                       | GPIO_CFG_OUTPUT_SET)          /* Input pull-up mode */

#  define GPIO_CFG_CTL_OUTPP          (0 << GPIO_CFG_CTL_SHIFT)       /* Output push-pull mode */
#  define GPIO_CFG_CTL_OUTOD          (1 << GPIO_CFG_CTL_SHIFT)       /* Output open-drain mode */
#  define GPIO_CFG_CTL_AFPP           (2 << GPIO_CFG_CTL_SHIFT)       /* Alternate function push-pull mode */
#  define GPIO_CFG_CTL_AFOD           (3 << GPIO_CFG_CTL_SHIFT)       /* Alternate function open-drain mode */

/* Maximum frequency selection:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ...S S... .... ....
 */

#define GPIO_CFG_MODE_SHIFT           11                               /* Bits 11-12: GPIO pin speed */
#define GPIO_CFG_MODE_MASK            (3 << GPIO_CFG_MODE_SHIFT)
#  define GPIO_CFG_MODE_INPUT         (0 << GPIO_CFG_MODE_SHIFT)       /* Input mode (reset state) */
#  define GPIO_CFG_SPEED_10MHZ        (1 << GPIO_CFG_MODE_SHIFT)       /* Output mode, max speed 10 MHz */
#  define GPIO_CFG_SPEED_2MHZ         (2 << GPIO_CFG_MODE_SHIFT)       /* Output mode, max speed 2 MHz */
#  define GPIO_CFG_SPEED_50MHZ        (3 << GPIO_CFG_MODE_SHIFT)       /* Output mode, max speed 50 MHz */
#  define GPIO_CFG_SPEED_MAX          ((3 << GPIO_CFG_MODE_SHIFT) \
                                       | GPIO_CFG_SPEED_A)             /* Output mode, max speed more than 50 MHz */

#define GPIO_ADJUST_CFG_MODE(cfg, spd) (((cfg) & ~GPIO_CFG_MODE_MASK) | (spd))

/* GPIO MAX output speed (Addional definitions for GPIO speed):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .A.. .... ....
 */

#define GPIO_CFG_SPEED_A             (1 << 10)                         /* Bit 10: Additional definitions for MAX OSPEED */

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... ..X. .... ....
 */

#define GPIO_CFG_EXTI                 (1 << 9)                         /* Bit 9: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... .PPP ....
 */

#define GPIO_CFG_PORT_SHIFT           4                                /* Bit 4-6:  Port number */
#define GPIO_CFG_PORT_MASK            (7 << GPIO_CFG_PORT_SHIFT)
#  define GPIO_CFG_PORT_A             (0 << GPIO_CFG_PORT_SHIFT)       /* GPIOA */
#  define GPIO_CFG_PORT_B             (1 << GPIO_CFG_PORT_SHIFT)       /* GPIOB */
#  define GPIO_CFG_PORT_C             (2 << GPIO_CFG_PORT_SHIFT)       /* GPIOC */
#  define GPIO_CFG_PORT_D             (3 << GPIO_CFG_PORT_SHIFT)       /* GPIOD */
#  define GPIO_CFG_PORT_E             (4 << GPIO_CFG_PORT_SHIFT)       /* GPIOE */

/* This identifies the bit in the port:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... .... BBBB
 */

#define GPIO_CFG_PIN_SHIFT            0                                /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_CFG_PIN_MASK             (15 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_0                (0 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_1                (1 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_2                (2 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_3                (3 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_4                (4 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_5                (5 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_6                (6 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_7                (7 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_8                (8 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_9                (9 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_10               (10 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_11               (11 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_12               (12 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_13               (13 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_14               (14 << GPIO_CFG_PIN_SHIFT)
#define GPIO_CFG_PIN_15               (15 << GPIO_CFG_PIN_SHIFT)

/****************************************************************************
 * Public Data
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

/* Base addresses for each GPIO block */

EXTERN const uint32_t g_gpio_base[GD32_NGPIO_PORTS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Return value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 *
 ****************************************************************************/

int gd32_gpio_config(uint32_t cfgset);

/****************************************************************************
 * Name: gd32_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state.
 *
 * Returned Value:
 *  OK on success
 *  A negated errno value on invalid port or mode
 ****************************************************************************/

int gd32_gpio_unconfig(uint32_t cfgset);

/****************************************************************************
 * Name: gd32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void gd32_gpio_write(uint32_t pinset, bool value);

/****************************************************************************
 * Name: gd32_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool gd32_gpio_read(uint32_t pinset);

/****************************************************************************
 * Function:  gd32_dump_gpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int gd32_dump_gpio(uint32_t pinset, const char *msg);
#else
#  define gd32_dump_gpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_GPIO_H */
