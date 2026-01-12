/****************************************************************************
 * arch/arm/include/gd32e11x/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_GD32E11X_CHIP_H
#define __ARCH_ARM_INCLUDE_GD32E11X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Check the GD32E11x family configuration.
 *
 * NOTE: CONFIG_ARCH_CHIP_GD32E113VB is added for GD32E113x(VB) package.
 */

#if defined(CONFIG_ARCH_CHIP_GD32E113VB)
/* Resources based on GD32E113xx device feature table (VB column) */

#  define GD32_NGPIO_PORTS              5   /* GPIOA-E (80 pins / 16 pins per port) */
#  define GD32_NDMA                     2   /* DMA0-1 */
#  define GD32_NADC                     2   /* ADC0-1 */
#  define GD32_NDAC                     1   /* DAC */

#  define GD32_NATIMER                  2   /* Advanced timers: TIMER0,7 */
#  define GD32_NGTIMER                  10  /* General timers: TIMER1-4, TIMER8-13 */
#  define GD32_NBTIMER                  2   /* Basic timers: TIMER5-6 */

#  define GD32_NUSART                   5   /* USART0-2, UART3-4 */
#  define GD32_NI2C                     2   /* I2C0-1 */
#  define GD32_NSPI                     3   /* SPI0-2 */
#  define GD32_NI2S                     2   /* I2S1-2 (muxed with SPI1-2) */
#  define GD32_NUSBFS                   1   /* USBFS */
#  define GD32_NEXMC                    1   /* EXMC */

#elif defined(CONFIG_ARCH_CHIP_GD32E113CB)
/* Resources based on GD32E113xx device feature table (CB column) */

#  define GD32_NGPIO_PORTS              5   /* GPIOA-E (80 pins / 16 pins per port) */
#  define GD32_NDMA                     2   /* DMA0-1 */
#  define GD32_NADC                     2   /* ADC0-1 */
#  define GD32_NDAC                     1   /* DAC */

#  define GD32_NATIMER                  1   /* Advanced timers: TIMER0,7 */
#  define GD32_NGTIMER                  10  /* General timers: TIMER1-4, TIMER8-13 */
#  define GD32_NBTIMER                  2   /* Basic timers: TIMER5-6 */

#  define GD32_NUSART                   3   /* USART0-2 */
#  define GD32_NI2C                     2   /* I2C0-1 */
#  define GD32_NSPI                     3   /* SPI0-2 */
#  define GD32_NI2S                     2   /* I2S1-2 (muxed with SPI1-2) */
#  define GD32_NUSBFS                   1   /* USBFS */
#  define GD32_NEXMC                    1   /* EXMC */

#else
#  error "Unknown GD32E11x chip type"
#endif

/* Get customizations for each supported chip and provide alternate function
 * pin-mapping
 *
 * NOTE: Each GPIO pin may serve either for general purpose I/O or for a
 * special alternate function (such as USART, CAN, USB, SDIO, etc.).  That
 * particular pin-mapping will depend on the package and GD32 family.  If
 * you are incorporating a new GD32 chip into NuttX, you will need to add
 * the pin-mapping to a header file and to include that header file below.
 * The chip-specific pin-mapping is defined in the chip datasheet.
 */

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_GD32E11X_CHIP_H */
