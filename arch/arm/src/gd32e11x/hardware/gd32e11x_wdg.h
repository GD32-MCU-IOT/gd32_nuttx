/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_wdg.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_WDG_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_WDG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Free Watchdog Timer (FWDGT) Register Offsets */

#define GD32_FWDGT_CTL_OFFSET       0x0000  /* FWDGT control register */
#define GD32_FWDGT_PSC_OFFSET       0x0004  /* FWDGT prescaler register */
#define GD32_FWDGT_RLD_OFFSET       0x0008  /* FWDGT reload register */
#define GD32_FWDGT_STAT_OFFSET      0x000c  /* FWDGT status register */

/* Window Watchdog Timer (WWDGT) Register Offsets */

#define GD32_WWDGT_CTL_OFFSET       0x0000  /* WWDGT control register */
#define GD32_WWDGT_CFG_OFFSET       0x0004  /* WWDGT configuration register */
#define GD32_WWDGT_STAT_OFFSET      0x0008  /* WWDGT status register */

/* Register Addresses *******************************************************/

/* Free Watchdog Timer (FWDGT) Register Addresses */

#define GD32_FWDGT_CTL              (GD32_FWDGT_BASE + GD32_FWDGT_CTL_OFFSET)
#define GD32_FWDGT_PSC              (GD32_FWDGT_BASE + GD32_FWDGT_PSC_OFFSET)
#define GD32_FWDGT_RLD              (GD32_FWDGT_BASE + GD32_FWDGT_RLD_OFFSET)
#define GD32_FWDGT_STAT             (GD32_FWDGT_BASE + GD32_FWDGT_STAT_OFFSET)

/* Window Watchdog Timer (WWDGT) Register Addresses */

#define GD32_WWDGT_CTL              (GD32_WWDGT_BASE + GD32_WWDGT_CTL_OFFSET)
#define GD32_WWDGT_CFG              (GD32_WWDGT_BASE + GD32_WWDGT_CFG_OFFSET)
#define GD32_WWDGT_STAT             (GD32_WWDGT_BASE + GD32_WWDGT_STAT_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* FWDGT control register (FWDGT_CTL) */

#define FWDGT_CTL_CMD_SHIFT         (0)       /* Bits 0-15: Command value */
#define FWDGT_CTL_CMD_MASK          (0xffff << FWDGT_CTL_CMD_SHIFT)
#  define FWDGT_CTL_CMD_ENABLE      (0xcccc)  /* Enable FWDGT */
#  define FWDGT_CTL_CMD_RELOAD      (0xaaaa)  /* Reload counter */
#  define FWDGT_CTL_CMD_UNLOCK      (0x5555)  /* Unlock PSC and RLD registers */

/* FWDGT prescaler register (FWDGT_PSC) */

#define FWDGT_PSC_SHIFT             (0)       /* Bits 0-2: Prescaler divider */
#define FWDGT_PSC_MASK              (0x7 << FWDGT_PSC_SHIFT)
#  define FWDGT_PSC_DIV4            (0 << FWDGT_PSC_SHIFT)  /* PSC = 4 */
#  define FWDGT_PSC_DIV8            (1 << FWDGT_PSC_SHIFT)  /* PSC = 8 */
#  define FWDGT_PSC_DIV16           (2 << FWDGT_PSC_SHIFT)  /* PSC = 16 */
#  define FWDGT_PSC_DIV32           (3 << FWDGT_PSC_SHIFT)  /* PSC = 32 */
#  define FWDGT_PSC_DIV64           (4 << FWDGT_PSC_SHIFT)  /* PSC = 64 */
#  define FWDGT_PSC_DIV128          (5 << FWDGT_PSC_SHIFT)  /* PSC = 128 */
#  define FWDGT_PSC_DIV256          (6 << FWDGT_PSC_SHIFT)  /* PSC = 256 */

/* FWDGT reload register (FWDGT_RLD) */

#define FWDGT_RLD_SHIFT             (0)       /* Bits 0-11: Reload value */
#define FWDGT_RLD_MASK              (0xfff << FWDGT_RLD_SHIFT)
#define FWDGT_RLD_MAX               (0xfff)

/* FWDGT status register (FWDGT_STAT) */

#define FWDGT_STAT_PUD              (1 << 0)  /* Bit 0: PSC update flag */
#define FWDGT_STAT_RUD              (1 << 1)  /* Bit 1: RLD update flag */

/* WWDGT control register (WWDGT_CTL) */

#define WWDGT_CTL_CNT_SHIFT         (0)       /* Bits 0-6: Counter value */
#define WWDGT_CTL_CNT_MASK          (0x7f << WWDGT_CTL_CNT_SHIFT)
#define WWDGT_CTL_CNT_MAX           (0x7f)
#define WWDGT_CTL_WDGTEN            (1 << 7)  /* Bit 7: WWDGT enable */

/* WWDGT configuration register (WWDGT_CFG) */

#define WWDGT_CFG_WIN_SHIFT         (0)       /* Bits 0-6: Window value */
#define WWDGT_CFG_WIN_MASK          (0x7f << WWDGT_CFG_WIN_SHIFT)
#define WWDGT_CFG_WIN_MAX           (0x7f)

#define WWDGT_CFG_PSC_SHIFT         (7)       /* Bits 7-8: Prescaler */
#define WWDGT_CFG_PSC_MASK          (0x3 << WWDGT_CFG_PSC_SHIFT)
#  define WWDGT_CFG_PSC_DIV1        (0 << WWDGT_CFG_PSC_SHIFT)  /* PSC = 1 */
#  define WWDGT_CFG_PSC_DIV2        (1 << WWDGT_CFG_PSC_SHIFT)  /* PSC = 2 */
#  define WWDGT_CFG_PSC_DIV4        (2 << WWDGT_CFG_PSC_SHIFT)  /* PSC = 4 */
#  define WWDGT_CFG_PSC_DIV8        (3 << WWDGT_CFG_PSC_SHIFT)  /* PSC = 8 */

#define WWDGT_CFG_EWIE              (1 << 9)  /* Bit 9: Early wakeup interrupt enable */

/* WWDGT status register (WWDGT_STAT) */

#define WWDGT_STAT_EWIF             (1 << 0)  /* Bit 0: Early wakeup interrupt flag */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_WDG_H */
