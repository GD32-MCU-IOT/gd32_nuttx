/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_exti.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_EXTI_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GD32_NEXTI                        19
#define GD32_EXTI_MASK                    0x000fffff

#define GD32_EXTI_BIT(n)                  (1 << (n))

/* Register Offsets *********************************************************/

#define GD32_EXTI_INTEN_OFFSET         0x0000  /* Interrupt enable register offset */
#define GD32_EXTI_EVEN_OFFSET          0x0004  /* Event enable register offset */
#define GD32_EXTI_RTEN_OFFSET          0x0008  /* Rising edge trigger enable register offset */
#define GD32_EXTI_FTEN_OFFSET          0x000c  /* Falling edge trigger enable register offset */
#define GD32_EXTI_SWIEV_OFFSET         0x0010  /* Software interrupt event register offset */
#define GD32_EXTI_PD_OFFSET            0x0014  /* Pending register offset */

/* Register Addresses *******************************************************/

#define GD32_EXTI_INTEN                (GD32_EXTI_BASE+GD32_EXTI_INTEN_OFFSET)  /* Interrupt enable register */
#define GD32_EXTI_EVEN                 (GD32_EXTI_BASE+GD32_EXTI_EVEN_OFFSET)   /* Event enable register */
#define GD32_EXTI_RTEN                 (GD32_EXTI_BASE+GD32_EXTI_RTEN_OFFSET)   /* Rising edge trigger enable register */
#define GD32_EXTI_FTEN                 (GD32_EXTI_BASE+GD32_EXTI_FTEN_OFFSET)   /* Falling edge trigger enable register */
#define GD32_EXTI_SWIEV                (GD32_EXTI_BASE+GD32_EXTI_SWIEV_OFFSET)  /* Software interrupt event register */
#define GD32_EXTI_PD                   (GD32_EXTI_BASE+GD32_EXTI_PD_OFFSET)     /* Pending register */

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 are associated with internal devices: */

#  define EXTI_PVD_LINE                  (1 << 16)   /* EXTI line 16 is connected to the PVD output */
#  define EXTI_RTC_ALARM                 (1 << 17)   /* EXTI line 17 is connected to the RTC Alarm event */
#  define EXTI_USBFS_WAKEUP              (1 << 18)   /* EXTI line 18 is connected to the USB FS Wakeup event */

/* Interrupt enable register */

#define EXTI_INTEN_BIT(n)                GD32_EXTI_BIT(n)      /* 1=Interrupt request from line x is not masked */
#define EXTI_INTEN_SHIFT                 (0)                   /* Bits 0-X: Interrupt enable for all lines */
#define EXTI_INTEN_MASK                  GD32_EXTI_MASK

/* Event enable register */

#define EXTI_EVEN_BIT(n)                 GD32_EXTI_BIT(n)      /* 1=Event request from line x is not mask */
#define EXTI_EVEN_SHIFT                  (0)                   /* Bits 0-X: Event enable for all lines */
#define EXTI_EVEN_MASK                   GD32_EXTI_MASK

/* Rising edge trigger enable register */

#define EXTI_RTEN_BIT(n)                 GD32_EXTI_BIT(n)      /* 1=Rising trigger enabled (for Event and Interrupt) for input line */
#define EXTI_RTEN_SHIFT                  (0)                   /* Bits 0-X: Rising trigger event configuration bit for all lines */
#define EXTI_RTEN_MASK                   GD32_EXTI_MASK

/* Falling edge trigger enable register */

#define EXTI_FTEN_BIT(n)                 GD32_EXTI_BIT(n)      /* 1=Falling trigger enabled (for Event and Interrupt) for input line */
#define EXTI_FTEN_SHIFT                  (0)                   /* Bits 0-X: Falling trigger event configuration bit for all lines */
#define EXTI_FTEN_MASK                   GD32_EXTI_MASK

/* Software interrupt event register  */

#define EXTI_SWIEV_BIT(n)                GD32_EXTI_BIT(n)      /* 1=Sets the corresponding pending bit in EXTI_PD */
#define EXTI_SWIEV_SHIFT                 (0)                   /* Bits 0-X: Software interrupt for all lines */
#define EXTI_SWIEV_MASK                  GD32_EXTI_MASK

/* Pending register */

#define EXTI_PD_BIT(n)                   GD32_EXTI_BIT(n)      /* 1=Selected trigger request occurred */
#define EXTI_PD_SHIFT                    (0)                   /* Bits 0-X: Pending bit for all lines */
#define EXTI_PD_MASK                     GD32_EXTI_MASK

/* EXTI lines 0-19 (for backward compatibility) */

/* EXTI line number */
#define EXTI_0                           GD32_EXTI_BIT(0)      /* EXTI line 0 */
#define EXTI_1                           GD32_EXTI_BIT(1)      /* EXTI line 1 */
#define EXTI_2                           GD32_EXTI_BIT(2)      /* EXTI line 2 */
#define EXTI_3                           GD32_EXTI_BIT(3)      /* EXTI line 3 */
#define EXTI_4                           GD32_EXTI_BIT(4)      /* EXTI line 4 */
#define EXTI_5                           GD32_EXTI_BIT(5)      /* EXTI line 5 */
#define EXTI_6                           GD32_EXTI_BIT(6)      /* EXTI line 6 */
#define EXTI_7                           GD32_EXTI_BIT(7)      /* EXTI line 7 */
#define EXTI_8                           GD32_EXTI_BIT(8)      /* EXTI line 8 */
#define EXTI_9                           GD32_EXTI_BIT(9)      /* EXTI line 9 */
#define EXTI_10                          GD32_EXTI_BIT(10)     /* EXTI line 10 */
#define EXTI_11                          GD32_EXTI_BIT(11)     /* EXTI line 11 */
#define EXTI_12                          GD32_EXTI_BIT(12)     /* EXTI line 12 */
#define EXTI_13                          GD32_EXTI_BIT(13)     /* EXTI line 13 */
#define EXTI_14                          GD32_EXTI_BIT(14)     /* EXTI line 14 */
#define EXTI_15                          GD32_EXTI_BIT(15)     /* EXTI line 15 */
#define EXTI_16                          GD32_EXTI_BIT(16)     /* EXTI line 16 */
#define EXTI_17                          GD32_EXTI_BIT(17)     /* EXTI line 17 */
#define EXTI_18                          GD32_EXTI_BIT(18)     /* EXTI line 18 */

/* External interrupt and event  */

#define EXTI_INTERRUPT                   0                     /* EXTI interrupt mode */
#define EXTI_EVENT                       1                     /* EXTI event mode */

/* Interrupt trigger mode */
#define EXTI_TRIG_RISING                 0                     /* EXTI rising edge trigger */
#define EXTI_TRIG_FALLING                1                     /* EXTI falling edge trigger */
#define EXTI_TRIG_BOTH                   2                     /* EXTI rising and falling edge trigger */
#define EXTI_TRIG_NONE                   3                     /* None EXTI edge trigger */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_EXTI_H */
