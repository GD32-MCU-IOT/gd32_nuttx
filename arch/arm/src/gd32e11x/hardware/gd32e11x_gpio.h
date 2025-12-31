/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_gpio.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_GPIO_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIOx(x=A,B,C,D,E) definitions *******************************************/

#define GD32_GPIOA_BASE                  (GD32_GPIO_BASE + 0x00000000)
#define GD32_GPIOB_BASE                  (GD32_GPIO_BASE + 0x00000400)
#define GD32_GPIOC_BASE                  (GD32_GPIO_BASE + 0x00000800)
#define GD32_GPIOD_BASE                  (GD32_GPIO_BASE + 0x00000c00)
#define GD32_GPIOE_BASE                  (GD32_GPIO_BASE + 0x00001000)

/* Register Offsets *********************************************************/

#define GD32_GPIO_CTL0_OFFSET            0x0000 /* GPIO port control register 0 offset */
#define GD32_GPIO_CTL1_OFFSET            0x0004 /* GPIO port control register 1 offset */
#define GD32_GPIO_ISTAT_OFFSET           0x0008 /* GPIO port input status register offset */
#define GD32_GPIO_OCTL_OFFSET            0x000c /* GPIO port output control register offset */
#define GD32_GPIO_BOP_OFFSET             0x0010 /* GPIO port bit operation register offset */
#define GD32_GPIO_BC_OFFSET              0x0014 /* GPIO bit clear register offset */
#define GD32_GPIO_LOCK_OFFSET            0x0018 /* GPIO port configuration lock register offset */
#define GD32_GPIO_SPD_OFFSET             0x003c /* GPIO port bit speed register offset */

/* Register Addresses *******************************************************/

#define GD32_GPIOA                       GD32_GPIOA_BASE
#define GD32_GPIOB                       GD32_GPIOB_BASE
#define GD32_GPIOC                       GD32_GPIOC_BASE
#define GD32_GPIOD                       GD32_GPIOD_BASE
#define GD32_GPIOE                       GD32_GPIOE_BASE

#define GD32_GPIO_CTL0(gpiox)            ((gpiox)+GD32_GPIO_CTL0_OFFSET)    /* GPIO port control register 0 */
#define GD32_GPIO_CTL1(gpiox)            ((gpiox)+GD32_GPIO_CTL1_OFFSET)    /* GPIO port control register 1 */
#define GD32_GPIO_ISTAT(gpiox)           ((gpiox)+GD32_GPIO_ISTAT_OFFSET)   /* GPIO port input status register */
#define GD32_GPIO_OCTL(gpiox)            ((gpiox)+GD32_GPIO_OCTL_OFFSET)    /* GPIO port output control register */
#define GD32_GPIO_BOP(gpiox)             ((gpiox)+GD32_GPIO_BOP_OFFSET)     /* GPIO port bit operation register */
#define GD32_GPIO_BC(gpiox)              ((gpiox)+GD32_GPIO_BC_OFFSET)      /* GPIO bit clear register */
#define GD32_GPIO_LOCK(gpiox)            ((gpiox)+GD32_GPIO_LOCK_OFFSET)    /* GPIO port configuration lock register */
#define GD32_GPIO_SPD(gpiox)             ((gpiox)+GD32_GPIO_SPD_OFFSET)     /* GPIO port bit speed register */

/* Register Bitfield Definitions ********************************************/

/* GPIO port control register 0/1 (CTL0/CTL1) */

/* Pin mode (MD) - Bits [1:0] for each pin in CTL0/CTL1 */

#define GPIO_MODE_INPUT                  (0) /* Input mode */
#define GPIO_MODE_OUTPUT_10MHZ           (1) /* Output mode, max speed 10 MHz */
#define GPIO_MODE_OUTPUT_2MHZ            (2) /* Output mode, max speed 2 MHz */
#define GPIO_MODE_OUTPUT_50MHZ           (3) /* Output mode, max speed 50 MHz */

/* Pin configuration (CTL) - Bits [3:2] for each pin in CTL0/CTL1 */

/* Input mode configuration */

#define GPIO_CNF_IN_ANALOG               (0 << 2) /* Analog input mode */
#define GPIO_CNF_IN_FLOATING             (1 << 2) /* Floating input mode */
#define GPIO_CNF_IN_PUPD                 (2 << 2) /* Pull-up/pull-down input mode */

/* Output mode configuration */

#define GPIO_CNF_OUT_PP                  (0 << 2) /* GPIO output push-pull */
#define GPIO_CNF_OUT_OD                  (1 << 2) /* GPIO output open-drain */
#define GPIO_CNF_AF_PP                   (2 << 2) /* Alternate function output push-pull */
#define GPIO_CNF_AF_OD                   (3 << 2) /* Alternate function output open-drain */

/* CTL0 register bit field definitions (pins 0-7) */

#define GPIO_CTL0_MD_SHIFT(n)            ((n) << 2)
#define GPIO_CTL0_MD_MASK(n)             (0x3 << GPIO_CTL0_MD_SHIFT(n))
#define GPIO_CTL0_CTL_SHIFT(n)           (((n) << 2) + 2)
#define GPIO_CTL0_CTL_MASK(n)            (0x3 << GPIO_CTL0_CTL_SHIFT(n))

#define GPIO_CTL0_MD0_SHIFT              (0)
#define GPIO_CTL0_MD0_MASK               (0x3 << GPIO_CTL0_MD0_SHIFT)
#define GPIO_CTL0_CTL0_SHIFT             (2)
#define GPIO_CTL0_CTL0_MASK              (0x3 << GPIO_CTL0_CTL0_SHIFT)

#define GPIO_CTL0_MD1_SHIFT              (4)
#define GPIO_CTL0_MD1_MASK               (0x3 << GPIO_CTL0_MD1_SHIFT)
#define GPIO_CTL0_CTL1_SHIFT             (6)
#define GPIO_CTL0_CTL1_MASK              (0x3 << GPIO_CTL0_CTL1_SHIFT)

#define GPIO_CTL0_MD2_SHIFT              (8)
#define GPIO_CTL0_MD2_MASK               (0x3 << GPIO_CTL0_MD2_SHIFT)
#define GPIO_CTL0_CTL2_SHIFT             (10)
#define GPIO_CTL0_CTL2_MASK              (0x3 << GPIO_CTL0_CTL2_SHIFT)

#define GPIO_CTL0_MD3_SHIFT              (12)
#define GPIO_CTL0_MD3_MASK               (0x3 << GPIO_CTL0_MD3_SHIFT)
#define GPIO_CTL0_CTL3_SHIFT             (14)
#define GPIO_CTL0_CTL3_MASK              (0x3 << GPIO_CTL0_CTL3_SHIFT)

#define GPIO_CTL0_MD4_SHIFT              (16)
#define GPIO_CTL0_MD4_MASK               (0x3 << GPIO_CTL0_MD4_SHIFT)
#define GPIO_CTL0_CTL4_SHIFT             (18)
#define GPIO_CTL0_CTL4_MASK              (0x3 << GPIO_CTL0_CTL4_SHIFT)

#define GPIO_CTL0_MD5_SHIFT              (20)
#define GPIO_CTL0_MD5_MASK               (0x3 << GPIO_CTL0_MD5_SHIFT)
#define GPIO_CTL0_CTL5_SHIFT             (22)
#define GPIO_CTL0_CTL5_MASK              (0x3 << GPIO_CTL0_CTL5_SHIFT)

#define GPIO_CTL0_MD6_SHIFT              (24)
#define GPIO_CTL0_MD6_MASK               (0x3 << GPIO_CTL0_MD6_SHIFT)
#define GPIO_CTL0_CTL6_SHIFT             (26)
#define GPIO_CTL0_CTL6_MASK              (0x3 << GPIO_CTL0_CTL6_SHIFT)

#define GPIO_CTL0_MD7_SHIFT              (28)
#define GPIO_CTL0_MD7_MASK               (0x3 << GPIO_CTL0_MD7_SHIFT)
#define GPIO_CTL0_CTL7_SHIFT             (30)
#define GPIO_CTL0_CTL7_MASK              (0x3 << GPIO_CTL0_CTL7_SHIFT)

/* CTL1 register bit field definitions (pins 8-15) */

#define GPIO_CTL1_MD_SHIFT(n)            (((n) - 8) << 2)
#define GPIO_CTL1_MD_MASK(n)             (0x3 << GPIO_CTL1_MD_SHIFT(n))
#define GPIO_CTL1_CTL_SHIFT(n)           ((((n) - 8) << 2) + 2)
#define GPIO_CTL1_CTL_MASK(n)            (0x3 << GPIO_CTL1_CTL_SHIFT(n))

#define GPIO_CTL1_MD8_SHIFT              (0)
#define GPIO_CTL1_MD8_MASK               (0x3 << GPIO_CTL1_MD8_SHIFT)
#define GPIO_CTL1_CTL8_SHIFT             (2)
#define GPIO_CTL1_CTL8_MASK              (0x3 << GPIO_CTL1_CTL8_SHIFT)

#define GPIO_CTL1_MD9_SHIFT              (4)
#define GPIO_CTL1_MD9_MASK               (0x3 << GPIO_CTL1_MD9_SHIFT)
#define GPIO_CTL1_CTL9_SHIFT             (6)
#define GPIO_CTL1_CTL9_MASK              (0x3 << GPIO_CTL1_CTL9_SHIFT)

#define GPIO_CTL1_MD10_SHIFT             (8)
#define GPIO_CTL1_MD10_MASK              (0x3 << GPIO_CTL1_MD10_SHIFT)
#define GPIO_CTL1_CTL10_SHIFT            (10)
#define GPIO_CTL1_CTL10_MASK             (0x3 << GPIO_CTL1_CTL10_SHIFT)

#define GPIO_CTL1_MD11_SHIFT             (12)
#define GPIO_CTL1_MD11_MASK              (0x3 << GPIO_CTL1_MD11_SHIFT)
#define GPIO_CTL1_CTL11_SHIFT            (14)
#define GPIO_CTL1_CTL11_MASK             (0x3 << GPIO_CTL1_CTL11_SHIFT)

#define GPIO_CTL1_MD12_SHIFT             (16)
#define GPIO_CTL1_MD12_MASK              (0x3 << GPIO_CTL1_MD12_SHIFT)
#define GPIO_CTL1_CTL12_SHIFT            (18)
#define GPIO_CTL1_CTL12_MASK             (0x3 << GPIO_CTL1_CTL12_SHIFT)

#define GPIO_CTL1_MD13_SHIFT             (20)
#define GPIO_CTL1_MD13_MASK              (0x3 << GPIO_CTL1_MD13_SHIFT)
#define GPIO_CTL1_CTL13_SHIFT            (22)
#define GPIO_CTL1_CTL13_MASK             (0x3 << GPIO_CTL1_CTL13_SHIFT)

#define GPIO_CTL1_MD14_SHIFT             (24)
#define GPIO_CTL1_MD14_MASK              (0x3 << GPIO_CTL1_MD14_SHIFT)
#define GPIO_CTL1_CTL14_SHIFT            (26)
#define GPIO_CTL1_CTL14_MASK             (0x3 << GPIO_CTL1_CTL14_SHIFT)

#define GPIO_CTL1_MD15_SHIFT             (28)
#define GPIO_CTL1_MD15_MASK              (0x3 << GPIO_CTL1_MD15_SHIFT)
#define GPIO_CTL1_CTL15_SHIFT            (30)
#define GPIO_CTL1_CTL15_MASK             (0x3 << GPIO_CTL1_CTL15_SHIFT)

/* GPIO port input status register (ISTAT) */

#define GPIO_ISTAT(n)                    (1 << (n))
#define GPIO_ISTAT_SHIFT(n)              (n)
#define GPIO_ISTAT_MASK(n)               (1 << GPIO_ISTAT_SHIFT(n))

/* GPIO port output control register (OCTL) */

#define GPIO_OCTL(n)                     (1 << (n))
#define GPIO_OCTL_SHIFT(n)               (n)
#define GPIO_OCTL_MASK(n)                (1 << GPIO_OCTL_SHIFT(n))

/* GPIO port bit operation register (BOP) */

#define GPIO_BOP_SET(n)                  (1 << (n))
#define GPIO_BOP_CLEAR(n)                (1 << ((n)+16))
#define GPIO_BOP_BOP(n)                  (1 << (n))       /* Port set bit */
#define GPIO_BOP_CR(n)                   (1 << ((n)+16))  /* Port clear bit */

/* GPIO bit clear register (BC) */

#define GPIO_BC_SET(n)                   (1 << (n))
#define GPIO_BC_CR(n)                    (1 << (n))       /* Port clear bit */

/* GPIO port configuration lock register (LOCK) */

#define GPIO_LOCK(n)                     (1 << (n))
#define GPIO_LOCK_LK(n)                  (1 << (n))       /* Port lock bit */
#define GPIO_LOCK_LKK                    (1 << 16)        /* Lock key */

/* GPIO port bit speed register (SPD) */

#define GPIO_SPD_SET(n)                  (1 << (n))       /* Port speed set bit */
#define GPIO_SPD_SPD(n)                  (1 << (n))       /* Port n speed bit */

/* GPIO pin definitions */

#define GPIO_PIN(n)                      (1 << (n)) /* Bit n: Pin n, n=0-15 */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_GPIO_H */
