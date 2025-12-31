/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_syscfg.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_SYSCFG_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Note: GD32E11x series uses AFIO (Alternate Function I/O) instead of SYSCFG
 * Register Offsets
 */

#define GD32_AFIO_EC_OFFSET              0x0000 /* Event control register */
#define GD32_AFIO_PCF0_OFFSET            0x0004 /* AFIO port configuration register 0 */
#define GD32_AFIO_EXTISS0_OFFSET         0x0008 /* EXTI sources selection register 0 */
#define GD32_AFIO_EXTISS1_OFFSET         0x000c /* EXTI sources selection register 1 */
#define GD32_AFIO_EXTISS2_OFFSET         0x0010 /* EXTI sources selection register 2 */
#define GD32_AFIO_EXTISS3_OFFSET         0x0014 /* EXTI sources selection register 3 */
#define GD32_AFIO_PCF1_OFFSET            0x001c /* AFIO port configuration register 1 */
#define GD32_AFIO_CPSCTL_OFFSET          0x0020 /* I/O compensation control register */

/* Register Addresses *******************************************************/

#define GD32_AFIO_EC                     (GD32_AFIO_BASE+GD32_AFIO_EC_OFFSET)
#define GD32_AFIO_PCF0                   (GD32_AFIO_BASE+GD32_AFIO_PCF0_OFFSET)
#define GD32_AFIO_EXTISS0                (GD32_AFIO_BASE+GD32_AFIO_EXTISS0_OFFSET)
#define GD32_AFIO_EXTISS1                (GD32_AFIO_BASE+GD32_AFIO_EXTISS1_OFFSET)
#define GD32_AFIO_EXTISS2                (GD32_AFIO_BASE+GD32_AFIO_EXTISS2_OFFSET)
#define GD32_AFIO_EXTISS3                (GD32_AFIO_BASE+GD32_AFIO_EXTISS3_OFFSET)
#define GD32_AFIO_PCF1                   (GD32_AFIO_BASE+GD32_AFIO_PCF1_OFFSET)
#define GD32_AFIO_CPSCTL                 (GD32_AFIO_BASE+GD32_AFIO_CPSCTL_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* AFIO event control register (EC) */

#define AFIO_EC_PIN_SHIFT                (0)
#define AFIO_EC_PIN_MASK                 (0xf << AFIO_EC_PIN_SHIFT)
#define AFIO_EC_PORT_SHIFT               (4)
#define AFIO_EC_PORT_MASK                (0x7 << AFIO_EC_PORT_SHIFT)
#define AFIO_EC_EOE                      (1 << 7)

/* AFIO port configuration register 0 (PCF0) */

#define AFIO_PCF0_SPI0_REMAP             (1 << 0)
#define AFIO_PCF0_I2C0_REMAP             (1 << 1)
#define AFIO_PCF0_USART0_REMAP           (1 << 2)
#define AFIO_PCF0_USART1_REMAP           (1 << 3)
#define AFIO_PCF0_USART2_REMAP_SHIFT     (4)
#define AFIO_PCF0_USART2_REMAP_MASK      (0x3 << AFIO_PCF0_USART2_REMAP_SHIFT)
#define AFIO_PCF0_USART2_REMAP(n)        ((n) << AFIO_PCF0_USART2_REMAP_SHIFT)
#define AFIO_PCF0_TIMER0_REMAP_SHIFT     (6)
#define AFIO_PCF0_TIMER0_REMAP_MASK      (0x3 << AFIO_PCF0_TIMER0_REMAP_SHIFT)
#define AFIO_PCF0_TIMER0_REMAP(n)        ((n) << AFIO_PCF0_TIMER0_REMAP_SHIFT)
#define AFIO_PCF0_TIMER1_REMAP_SHIFT     (8)
#define AFIO_PCF0_TIMER1_REMAP_MASK      (0x3 << AFIO_PCF0_TIMER1_REMAP_SHIFT)
#define AFIO_PCF0_TIMER1_REMAP(n)        ((n) << AFIO_PCF0_TIMER1_REMAP_SHIFT)
#define AFIO_PCF0_TIMER2_REMAP_SHIFT     (10)
#define AFIO_PCF0_TIMER2_REMAP_MASK      (0x3 << AFIO_PCF0_TIMER2_REMAP_SHIFT)
#define AFIO_PCF0_TIMER2_REMAP(n)        ((n) << AFIO_PCF0_TIMER2_REMAP_SHIFT)
#define AFIO_PCF0_TIMER3_REMAP           (1 << 12)
#define AFIO_PCF0_CAN0_REMAP_SHIFT       (13)
#define AFIO_PCF0_CAN0_REMAP_MASK        (0x3 << AFIO_PCF0_CAN0_REMAP_SHIFT)
#define AFIO_PCF0_CAN0_REMAP(n)          ((n) << AFIO_PCF0_CAN0_REMAP_SHIFT)
#define AFIO_PCF0_PD01_REMAP             (1 << 15)
#define AFIO_PCF0_TIMER4CH3_IREMAP       (1 << 16)
#define AFIO_PCF0_ADC0_ETRGRT_REMAP      (1 << 18)
#define AFIO_PCF0_ADC1_ETRGRT_REMAP      (1 << 20)
#define AFIO_PCF0_CAN1_REMAP             (1 << 22)
#define AFIO_PCF0_SWJ_CFG_SHIFT          (24)
#define AFIO_PCF0_SWJ_CFG_MASK           (0x7 << AFIO_PCF0_SWJ_CFG_SHIFT)
#define AFIO_PCF0_SWJ_CFG_(n)            ((n) << AFIO_PCF0_SWJ_CFG_SHIFT)
#define AFIO_PCF0_SPI2_REMAP             (1 << 28)
#define AFIO_PCF0_TIMER1ITI1_REMAP       (1 << 29)

/* AFIO EXTI sources selection registers (EXTISS0-3) */

#define AFIO_EXTISS_SHIFT(n)             (((n) & 0x3) << 2)
#define AFIO_EXTISS_MASK(n)              (0xf << AFIO_EXTISS_SHIFT(n))

#define AFIO_EXTISS_GPIOA                (0)
#define AFIO_EXTISS_GPIOB                (1)
#define AFIO_EXTISS_GPIOC                (2)
#define AFIO_EXTISS_GPIOD                (3)
#define AFIO_EXTISS_GPIOE                (4)

/* AFIO port configuration register 1 (PCF1) */

#define AFIO_PCF1_TIMER8_REMAP           (1 << 5)
#define AFIO_PCF1_EXMC_NADV              (1 << 10)
#define AFIO_PCF1_CTC_REMAP_SHIFT        (11)
#define AFIO_PCF1_CTC_REMAP_MASK         (0x3 << AFIO_PCF1_CTC_REMAP_SHIFT)
#define AFIO_PCF1_CTC_REMAP(n)           ((n) << AFIO_PCF1_CTC_REMAP_SHIFT)

/* Bit31 used for PCF1 */

#define AFIO_PCF1_TIMER8_REMAP_CFG       (0x80000000 | AFIO_PCF1_TIMER8_REMAP)
#define AFIO_PCF1_EXMC_NADV_CFG          (0x80000000 | AFIO_PCF1_EXMC_NADV)
#define AFIO_PCF1_CTC_REMAP_CFG          (0x80000000 | AFIO_PCF1_CTC_REMAP(1))

/* AFIO I/O compensation control register (CPSCTL) */

#define AFIO_CPSCTL_CPS_EN               (1 << 0)
#define AFIO_CPSCTL_CPS_RDY              (1 << 8)

/* Helper macros for remap configuration */

#define PCF0_USART2_REMAP(regval)        (AFIO_PCF0_USART2_REMAP_MASK & ((uint32_t)(regval) << AFIO_PCF0_USART2_REMAP_SHIFT))
#define PCF0_TIMER0_REMAP(regval)        (AFIO_PCF0_TIMER0_REMAP_MASK & ((uint32_t)(regval) << AFIO_PCF0_TIMER0_REMAP_SHIFT))
#define PCF0_TIMER1_REMAP(regval)        (AFIO_PCF0_TIMER1_REMAP_MASK & ((uint32_t)(regval) << AFIO_PCF0_TIMER1_REMAP_SHIFT))
#define PCF0_TIMER2_REMAP(regval)        (AFIO_PCF0_TIMER2_REMAP_MASK & ((uint32_t)(regval) << AFIO_PCF0_TIMER2_REMAP_SHIFT))
#define PCF0_SWJ_CFG(regval)             (AFIO_PCF0_SWJ_CFG_MASK & ((uint32_t)(regval) << AFIO_PCF0_SWJ_CFG_SHIFT))
#define PCF1_CTC_REMAP(regval)           (AFIO_PCF1_CTC_REMAP_MASK & ((uint32_t)(regval) << AFIO_PCF1_CTC_REMAP_SHIFT))

/* GPIO remap definitions (high byte indicates register) */

#define GPIO_REMAP_REG_MASK              (0xff000000)
#define GPIO_REMAP_REG_PCF0              (0x00000000)
#define GPIO_REMAP_REG_PCF1              (0x80000000)

#define GPIO_SPI0_REMAP                  (GPIO_REMAP_REG_PCF0 | AFIO_PCF0_SPI0_REMAP)
#define GPIO_I2C0_REMAP                  (GPIO_REMAP_REG_PCF0 | AFIO_PCF0_I2C0_REMAP)
#define GPIO_USART0_REMAP                (GPIO_REMAP_REG_PCF0 | AFIO_PCF0_USART0_REMAP)
#define GPIO_USART1_REMAP                (GPIO_REMAP_REG_PCF0 | AFIO_PCF0_USART1_REMAP)
#define GPIO_USART2_PARTIAL_REMAP        (GPIO_REMAP_REG_PCF0 | 0x00140000 | PCF0_USART2_REMAP(1))
#define GPIO_USART2_FULL_REMAP           (GPIO_REMAP_REG_PCF0 | 0x00140000 | PCF0_USART2_REMAP(3))
#define GPIO_TIMER0_PARTIAL_REMAP        (GPIO_REMAP_REG_PCF0 | 0x00160000 | PCF0_TIMER0_REMAP(1))
#define GPIO_TIMER0_FULL_REMAP           (GPIO_REMAP_REG_PCF0 | 0x00160000 | PCF0_TIMER0_REMAP(3))
#define GPIO_TIMER1_PARTIAL_REMAP0       (GPIO_REMAP_REG_PCF0 | 0x00180000 | PCF0_TIMER1_REMAP(1))
#define GPIO_TIMER1_PARTIAL_REMAP1       (GPIO_REMAP_REG_PCF0 | 0x00180000 | PCF0_TIMER1_REMAP(2))
#define GPIO_TIMER1_FULL_REMAP           (GPIO_REMAP_REG_PCF0 | 0x00180000 | PCF0_TIMER1_REMAP(3))
#define GPIO_TIMER2_PARTIAL_REMAP        (GPIO_REMAP_REG_PCF0 | 0x001a0000 | PCF0_TIMER2_REMAP(2))
#define GPIO_TIMER2_FULL_REMAP           (GPIO_REMAP_REG_PCF0 | 0x001a0000 | PCF0_TIMER2_REMAP(3))
#define GPIO_TIMER3_REMAP                (GPIO_REMAP_REG_PCF0 | AFIO_PCF0_TIMER3_REMAP)
#define GPIO_PD01_REMAP                  (GPIO_REMAP_REG_PCF0 | AFIO_PCF0_PD01_REMAP)
#define GPIO_TIMER4CH3_IREMAP            (GPIO_REMAP_REG_PCF0 | 0x00200000 | (AFIO_PCF0_TIMER4CH3_IREMAP >> 16))
#define GPIO_ADC0_ETRGRT_REMAP           (GPIO_REMAP_REG_PCF0 | 0x00200000 | (AFIO_PCF0_ADC0_ETRGRT_REMAP >> 16))
#define GPIO_ADC1_ETRGRT_REMAP           (GPIO_REMAP_REG_PCF0 | 0x00200000 | (AFIO_PCF0_ADC1_ETRGRT_REMAP >> 16))
#define GPIO_SWJ_NONJTRST_REMAP          (GPIO_REMAP_REG_PCF0 | 0x00300000 | (PCF0_SWJ_CFG(1) >> 16))
#define GPIO_SWJ_SWDPENABLE_REMAP        (GPIO_REMAP_REG_PCF0 | 0x00300000 | (PCF0_SWJ_CFG(2) >> 16))
#define GPIO_SWJ_DISABLE_REMAP           (GPIO_REMAP_REG_PCF0 | 0x00300000 | (PCF0_SWJ_CFG(4) >> 16))
#define GPIO_SPI2_REMAP                  (GPIO_REMAP_REG_PCF0 | 0x00200000 | (AFIO_PCF0_SPI2_REMAP >> 16))
#define GPIO_TIMER1ITI1_REMAP            (GPIO_REMAP_REG_PCF0 | 0x00200000 | (AFIO_PCF0_TIMER1ITI1_REMAP >> 16))
#define GPIO_TIMER8_REMAP                (GPIO_REMAP_REG_PCF1 | AFIO_PCF1_TIMER8_REMAP)
#define GPIO_EXMC_NADV_REMAP             (GPIO_REMAP_REG_PCF1 | AFIO_PCF1_EXMC_NADV)
#define GPIO_CTC_REMAP0                  (GPIO_REMAP_REG_PCF1 | 0x001b0000 | PCF1_CTC_REMAP(1))

/* I/O compensation cell enable/disable */

#define GPIO_COMPENSATION_ENABLE         (1)
#define GPIO_COMPENSATION_DISABLE        (0)

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_SYSCFG_H */
