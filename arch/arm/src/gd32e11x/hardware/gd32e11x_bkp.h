/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_bkp.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_BKP_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_BKP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/gd32e11x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_BKP_DATA0_OFFSET       0x0004  /* BKP data register 0 */
#define GD32_BKP_DATA1_OFFSET       0x0008  /* BKP data register 1 */
#define GD32_BKP_DATA2_OFFSET       0x000c  /* BKP data register 2 */
#define GD32_BKP_DATA3_OFFSET       0x0010  /* BKP data register 3 */
#define GD32_BKP_DATA4_OFFSET       0x0014  /* BKP data register 4 */
#define GD32_BKP_DATA5_OFFSET       0x0018  /* BKP data register 5 */
#define GD32_BKP_DATA6_OFFSET       0x001c  /* BKP data register 6 */
#define GD32_BKP_DATA7_OFFSET       0x0020  /* BKP data register 7 */
#define GD32_BKP_DATA8_OFFSET       0x0024  /* BKP data register 8 */
#define GD32_BKP_DATA9_OFFSET       0x0028  /* BKP data register 9 */
#define GD32_BKP_OCTL_OFFSET        0x002c  /* RTC signal output control register */
#define GD32_BKP_TPCTL_OFFSET       0x0030  /* Tamper pin control register */
#define GD32_BKP_TPCS_OFFSET        0x0034  /* Tamper control and status register */
#define GD32_BKP_DATA10_OFFSET      0x0040  /* BKP data register 10 */
#define GD32_BKP_DATA11_OFFSET      0x0044  /* BKP data register 11 */
#define GD32_BKP_DATA12_OFFSET      0x0048  /* BKP data register 12 */
#define GD32_BKP_DATA13_OFFSET      0x004c  /* BKP data register 13 */
#define GD32_BKP_DATA14_OFFSET      0x0050  /* BKP data register 14 */
#define GD32_BKP_DATA15_OFFSET      0x0054  /* BKP data register 15 */
#define GD32_BKP_DATA16_OFFSET      0x0058  /* BKP data register 16 */
#define GD32_BKP_DATA17_OFFSET      0x005c  /* BKP data register 17 */
#define GD32_BKP_DATA18_OFFSET      0x0060  /* BKP data register 18 */
#define GD32_BKP_DATA19_OFFSET      0x0064  /* BKP data register 19 */
#define GD32_BKP_DATA20_OFFSET      0x0068  /* BKP data register 20 */
#define GD32_BKP_DATA21_OFFSET      0x006c  /* BKP data register 21 */
#define GD32_BKP_DATA22_OFFSET      0x0070  /* BKP data register 22 */
#define GD32_BKP_DATA23_OFFSET      0x0074  /* BKP data register 23 */
#define GD32_BKP_DATA24_OFFSET      0x0078  /* BKP data register 24 */
#define GD32_BKP_DATA25_OFFSET      0x007c  /* BKP data register 25 */
#define GD32_BKP_DATA26_OFFSET      0x0080  /* BKP data register 26 */
#define GD32_BKP_DATA27_OFFSET      0x0084  /* BKP data register 27 */
#define GD32_BKP_DATA28_OFFSET      0x0088  /* BKP data register 28 */
#define GD32_BKP_DATA29_OFFSET      0x008c  /* BKP data register 29 */
#define GD32_BKP_DATA30_OFFSET      0x0090  /* BKP data register 30 */
#define GD32_BKP_DATA31_OFFSET      0x0094  /* BKP data register 31 */
#define GD32_BKP_DATA32_OFFSET      0x0098  /* BKP data register 32 */
#define GD32_BKP_DATA33_OFFSET      0x009c  /* BKP data register 33 */
#define GD32_BKP_DATA34_OFFSET      0x00a0  /* BKP data register 34 */
#define GD32_BKP_DATA35_OFFSET      0x00a4  /* BKP data register 35 */
#define GD32_BKP_DATA36_OFFSET      0x00a8  /* BKP data register 36 */
#define GD32_BKP_DATA37_OFFSET      0x00ac  /* BKP data register 37 */
#define GD32_BKP_DATA38_OFFSET      0x00b0  /* BKP data register 38 */
#define GD32_BKP_DATA39_OFFSET      0x00b4  /* BKP data register 39 */
#define GD32_BKP_DATA40_OFFSET      0x00b8  /* BKP data register 40 */
#define GD32_BKP_DATA41_OFFSET      0x00bc  /* BKP data register 41 */

/* Register Addresses *******************************************************/

#define GD32_BKP_DATA0              (GD32_BKP_BASE+GD32_BKP_DATA0_OFFSET)
#define GD32_BKP_DATA1              (GD32_BKP_BASE+GD32_BKP_DATA1_OFFSET)
#define GD32_BKP_DATA2              (GD32_BKP_BASE+GD32_BKP_DATA2_OFFSET)
#define GD32_BKP_DATA3              (GD32_BKP_BASE+GD32_BKP_DATA3_OFFSET)
#define GD32_BKP_DATA4              (GD32_BKP_BASE+GD32_BKP_DATA4_OFFSET)
#define GD32_BKP_DATA5              (GD32_BKP_BASE+GD32_BKP_DATA5_OFFSET)
#define GD32_BKP_DATA6              (GD32_BKP_BASE+GD32_BKP_DATA6_OFFSET)
#define GD32_BKP_DATA7              (GD32_BKP_BASE+GD32_BKP_DATA7_OFFSET)
#define GD32_BKP_DATA8              (GD32_BKP_BASE+GD32_BKP_DATA8_OFFSET)
#define GD32_BKP_DATA9              (GD32_BKP_BASE+GD32_BKP_DATA9_OFFSET)
#define GD32_BKP_OCTL               (GD32_BKP_BASE+GD32_BKP_OCTL_OFFSET)
#define GD32_BKP_TPCTL              (GD32_BKP_BASE+GD32_BKP_TPCTL_OFFSET)
#define GD32_BKP_TPCS               (GD32_BKP_BASE+GD32_BKP_TPCS_OFFSET)
#define GD32_BKP_DATA10             (GD32_BKP_BASE+GD32_BKP_DATA10_OFFSET)
#define GD32_BKP_DATA11             (GD32_BKP_BASE+GD32_BKP_DATA11_OFFSET)
#define GD32_BKP_DATA12             (GD32_BKP_BASE+GD32_BKP_DATA12_OFFSET)
#define GD32_BKP_DATA13             (GD32_BKP_BASE+GD32_BKP_DATA13_OFFSET)
#define GD32_BKP_DATA14             (GD32_BKP_BASE+GD32_BKP_DATA14_OFFSET)
#define GD32_BKP_DATA15             (GD32_BKP_BASE+GD32_BKP_DATA15_OFFSET)
#define GD32_BKP_DATA16             (GD32_BKP_BASE+GD32_BKP_DATA16_OFFSET)
#define GD32_BKP_DATA17             (GD32_BKP_BASE+GD32_BKP_DATA17_OFFSET)
#define GD32_BKP_DATA18             (GD32_BKP_BASE+GD32_BKP_DATA18_OFFSET)
#define GD32_BKP_DATA19             (GD32_BKP_BASE+GD32_BKP_DATA19_OFFSET)
#define GD32_BKP_DATA20             (GD32_BKP_BASE+GD32_BKP_DATA20_OFFSET)
#define GD32_BKP_DATA21             (GD32_BKP_BASE+GD32_BKP_DATA21_OFFSET)
#define GD32_BKP_DATA22             (GD32_BKP_BASE+GD32_BKP_DATA22_OFFSET)
#define GD32_BKP_DATA23             (GD32_BKP_BASE+GD32_BKP_DATA23_OFFSET)
#define GD32_BKP_DATA24             (GD32_BKP_BASE+GD32_BKP_DATA24_OFFSET)
#define GD32_BKP_DATA25             (GD32_BKP_BASE+GD32_BKP_DATA25_OFFSET)
#define GD32_BKP_DATA26             (GD32_BKP_BASE+GD32_BKP_DATA26_OFFSET)
#define GD32_BKP_DATA27             (GD32_BKP_BASE+GD32_BKP_DATA27_OFFSET)
#define GD32_BKP_DATA28             (GD32_BKP_BASE+GD32_BKP_DATA28_OFFSET)
#define GD32_BKP_DATA29             (GD32_BKP_BASE+GD32_BKP_DATA29_OFFSET)
#define GD32_BKP_DATA30             (GD32_BKP_BASE+GD32_BKP_DATA30_OFFSET)
#define GD32_BKP_DATA31             (GD32_BKP_BASE+GD32_BKP_DATA31_OFFSET)
#define GD32_BKP_DATA32             (GD32_BKP_BASE+GD32_BKP_DATA32_OFFSET)
#define GD32_BKP_DATA33             (GD32_BKP_BASE+GD32_BKP_DATA33_OFFSET)
#define GD32_BKP_DATA34             (GD32_BKP_BASE+GD32_BKP_DATA34_OFFSET)
#define GD32_BKP_DATA35             (GD32_BKP_BASE+GD32_BKP_DATA35_OFFSET)
#define GD32_BKP_DATA36             (GD32_BKP_BASE+GD32_BKP_DATA36_OFFSET)
#define GD32_BKP_DATA37             (GD32_BKP_BASE+GD32_BKP_DATA37_OFFSET)
#define GD32_BKP_DATA38             (GD32_BKP_BASE+GD32_BKP_DATA38_OFFSET)
#define GD32_BKP_DATA39             (GD32_BKP_BASE+GD32_BKP_DATA39_OFFSET)
#define GD32_BKP_DATA40             (GD32_BKP_BASE+GD32_BKP_DATA40_OFFSET)
#define GD32_BKP_DATA41             (GD32_BKP_BASE+GD32_BKP_DATA41_OFFSET)

/* Helpers for indexed data registers */

#define GD32_BKP_DATA0_9(n)         (GD32_BKP_BASE + 0x0004 + ((n) << 2))
#define GD32_BKP_DATA10_41(n)       (GD32_BKP_BASE + 0x0040 + (((n) - 10) << 2))

/* Register Bitfield Definitions ********************************************/

/* BKP data register */

#define BKP_DATA_SHIFT              (0)         /* Bits 0-15: Backup data */
#define BKP_DATA_MASK               (0xffff << BKP_DATA_SHIFT)

/* BKP RTC signal output control register */

#define BKP_OCTL_RCCV_SHIFT         (0)         /* Bits 0-6: RTC clock calibration value */
#define BKP_OCTL_RCCV_MASK          (0x7f << BKP_OCTL_RCCV_SHIFT)
#  define BKP_OCTL_RCCV(n)          ((n) << BKP_OCTL_RCCV_SHIFT)
#define BKP_OCTL_COEN               (1 << 7)    /* Bit 7: RTC calibration clock output enable */
#define BKP_OCTL_ASOEN              (1 << 8)    /* Bit 8: RTC alarm/second signal output enable */
#define BKP_OCTL_ROSEL              (1 << 9)    /* Bit 9: RTC output selection */
#define BKP_OCTL_CCOSEL             (1 << 14)   /* Bit 14: RTC calibration clock output selection */
#define BKP_OCTL_CALDIR             (1 << 15)   /* Bit 15: RTC clock calibration direction */

/* BKP tamper pin control register */

#define BKP_TPCTL_TPEN              (1 << 0)    /* Bit 0: Tamper detection enable */
#define BKP_TPCTL_TPAL              (1 << 1)    /* Bit 1: Tamper pin active level */

/* BKP tamper control and status register */

#define BKP_TPCS_TER                (1 << 0)    /* Bit 0: Tamper event reset */
#define BKP_TPCS_TIR                (1 << 1)    /* Bit 1: Tamper interrupt reset */
#define BKP_TPCS_TPIE               (1 << 2)    /* Bit 2: Tamper interrupt enable */
#define BKP_TPCS_TEF                (1 << 8)    /* Bit 8: Tamper event flag */
#define BKP_TPCS_TIF                (1 << 9)    /* Bit 9: Tamper interrupt flag */

/* BKP data extraction */

#define BKP_DATA_GET(v)             (((v) & BKP_DATA_MASK) >> BKP_DATA_SHIFT)

/* RTC output selection */

#define RTC_OUTPUT_ALARM_PULSE      (0)
#define RTC_OUTPUT_SECOND_PULSE     BKP_OCTL_ROSEL

/* RTC clock output selection */

#define RTC_CLOCK_DIV_64            (0)
#define RTC_CLOCK_DIV_1             BKP_OCTL_CCOSEL

/* RTC clock calibration direction */

#define RTC_CLOCK_SLOW_DOWN         (0)
#define RTC_CLOCK_SPEED_UP          BKP_OCTL_CALDIR

/* Tamper pin active level */

#define TAMPER_PIN_ACTIVE_HIGH      (0)
#define TAMPER_PIN_ACTIVE_LOW       BKP_TPCTL_TPAL

/* Tamper flag definitions */

#define BKP_FLAG_TAMPER             BKP_TPCS_TEF
#define BKP_INT_FLAG_TAMPER         BKP_TPCS_TIF

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_BKP_H */
