/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_fmc.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_FMC_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_FMC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_FMC_WS_OFFSET               0x0000 /* FMC wait state register offset */
#define GD32_FMC_KEY_OFFSET              0x0004 /* FMC unlock key register offset */
#define GD32_FMC_OBKEY_OFFSET            0x0008 /* FMC option bytes unlock key register offset */
#define GD32_FMC_STAT_OFFSET             0x000C /* FMC status register offset */
#define GD32_FMC_CTL_OFFSET              0x0010 /* FMC control register offset */
#define GD32_FMC_ADDR_OFFSET             0x0014 /* FMC address register offset */
#define GD32_FMC_OBSTAT_OFFSET           0x001C /* FMC option bytes status register offset */
#define GD32_FMC_WP_OFFSET               0x0020 /* FMC erase/program protection register offset */
#define GD32_FMC_PID_OFFSET              0x0100 /* FMC product ID register offset */

/* Register Addresses *******************************************************/

#define GD32_FMC_WS                      (GD32_FMC_BASE+GD32_FMC_WS_OFFSET)     /* FMC wait state register */
#define GD32_FMC_KEY                     (GD32_FMC_BASE+GD32_FMC_KEY_OFFSET)    /* FMC unlock key register */
#define GD32_FMC_OBKEY                   (GD32_FMC_BASE+GD32_FMC_OBKEY_OFFSET)  /* FMC option bytes unlock key register */
#define GD32_FMC_STAT                    (GD32_FMC_BASE+GD32_FMC_STAT_OFFSET)   /* FMC status register */
#define GD32_FMC_CTL                     (GD32_FMC_BASE+GD32_FMC_CTL_OFFSET)    /* FMC control register */
#define GD32_FMC_ADDR                    (GD32_FMC_BASE+GD32_FMC_ADDR_OFFSET)   /* FMC address register */
#define GD32_FMC_OBSTAT                  (GD32_FMC_BASE+GD32_FMC_OBSTAT_OFFSET) /* FMC option bytes status register */
#define GD32_FMC_WP                      (GD32_FMC_BASE+GD32_FMC_WP_OFFSET)     /* FMC erase/program protection register */
#define GD32_FMC_PID                     (GD32_FMC_BASE+GD32_FMC_PID_OFFSET)    /* FMC product ID register */

/* Register Bitfield Definitions ********************************************/

/* FMC_WS - FMC wait state register */

#define FMC_WS_WSCNT_SHIFT               (0)       /* Bits 0-2: wait state counter */
#define FMC_WS_WSCNT_MASK                (7 << FMC_WS_WSCNT_SHIFT)
#define FMC_WS_WSCNT(n)                  ((n) << FMC_WS_WSCNT_SHIFT)
#define FMC_WS_WSCNT_0                   (0 << FMC_WS_WSCNT_SHIFT)  /* FMC 0 wait */
#define FMC_WS_WSCNT_1                   (1 << FMC_WS_WSCNT_SHIFT)  /* FMC 1 wait */
#define FMC_WS_WSCNT_2                   (2 << FMC_WS_WSCNT_SHIFT)  /* FMC 2 wait */
#define FMC_WS_WSCNT_3                   (3 << FMC_WS_WSCNT_SHIFT)  /* FMC 3 wait */
#define FMC_WS_WSCNT_4                   (4 << FMC_WS_WSCNT_SHIFT)  /* FMC 4 wait */
#define FMC_WS_WSCNT_5                   (5 << FMC_WS_WSCNT_SHIFT)  /* FMC 5 wait */
#define FMC_WS_WSCNT_6                   (6 << FMC_WS_WSCNT_SHIFT)  /* FMC 6 wait */
#define FMC_WS_WSCNT_7                   (7 << FMC_WS_WSCNT_SHIFT)  /* FMC 7 wait */
#define FMC_WS_PFEN                      (1 << 4)                   /* Bit 4: pre-fetch enable */
#define FMC_WS_ICEN                      (1 << 9)                   /* Bit 9: IBUS cache enable */
#define FMC_WS_DCEN                      (1 << 10)                  /* Bit 10: DBUS cache enable */
#define FMC_WS_ICRST                     (1 << 11)                  /* Bit 11: IBUS cache reset */
#define FMC_WS_DCRST                     (1 << 12)                  /* Bit 12: DBUS cache reset */
#define FMC_WS_PGW                       (1 << 15)                  /* Bit 15: program width to flash memory */

/* FMC_STAT - FMC status register */

#define FMC_STAT_BUSY                    (1 << 0)  /* Bit 0: flash busy flag bit */
#define FMC_STAT_PGERR                   (1 << 2)  /* Bit 2: flash program error flag bit */
#define FMC_STAT_PGAERR                  (1 << 3)  /* Bit 3: flash program alignment error flag bit */
#define FMC_STAT_WPERR                   (1 << 4)  /* Bit 4: erase/program protection error flag bit */
#define FMC_STAT_ENDF                    (1 << 5)  /* Bit 5: end of operation flag bit */

/* FMC_CTL - FMC control register */

#define FMC_CTL_PG                       (1 << 0)  /* Bit 0: main flash program command bit */
#define FMC_CTL_PER                      (1 << 1)  /* Bit 1: main flash page erase command bit */
#define FMC_CTL_MER                      (1 << 2)  /* Bit 2: main flash mass erase command bit */
#define FMC_CTL_OBPG                     (1 << 4)  /* Bit 4: option bytes program command bit */
#define FMC_CTL_OBER                     (1 << 5)  /* Bit 5: option bytes erase command bit */
#define FMC_CTL_START                    (1 << 6)  /* Bit 6: send erase command to FMC bit */
#define FMC_CTL_LK                       (1 << 7)  /* Bit 7: FMC_CTL lock bit */
#define FMC_CTL_OBWEN                    (1 << 9)  /* Bit 9: option bytes erase/program enable bit */
#define FMC_CTL_ERRIE                    (1 << 10) /* Bit 10: error interrupt enable bit */
#define FMC_CTL_ENDIE                    (1 << 12) /* Bit 12: end of operation interrupt enable bit */

/* FMC_OBSTAT - FMC option bytes status register */

#define FMC_OBSTAT_OBERR                 (1 << 0)  /* Bit 0: option bytes read error bit */
#define FMC_OBSTAT_SPC                   (1 << 1)  /* Bit 1: option bytes security protection bit */
#define FMC_OBSTAT_USER_SHIFT            (2)       /* Bits 2-9: store USER of option bytes block after system reset */
#define FMC_OBSTAT_USER_MASK             (0xff << FMC_OBSTAT_USER_SHIFT)
#define FMC_OBSTAT_DATA_SHIFT            (10)      /* Bits 10-25: store DATA of option bytes block after system reset */
#define FMC_OBSTAT_DATA_MASK             (0xffff << FMC_OBSTAT_DATA_SHIFT)

/* FMC unlock key */

#define FMC_UNLOCK_KEY0                  (0x45670123)  /* Unlock key 0 */
#define FMC_UNLOCK_KEY1                  (0xCDEF89AB)  /* Unlock key 1 */

/* FMC timeout */

#define FMC_TIMEOUT_COUNT                (0x00100000)  /* FMC timeout count value */

/* Option bytes write protection (32 sectors) */

#define FMC_OB_WP_0                      (0x00000001)  /* erase/program protection of sector 0  */
#define FMC_OB_WP_1                      (0x00000002)  /* erase/program protection of sector 1  */
#define FMC_OB_WP_2                      (0x00000004)  /* erase/program protection of sector 2  */
#define FMC_OB_WP_3                      (0x00000008)  /* erase/program protection of sector 3  */
#define FMC_OB_WP_4                      (0x00000010)  /* erase/program protection of sector 4  */
#define FMC_OB_WP_5                      (0x00000020)  /* erase/program protection of sector 5  */
#define FMC_OB_WP_6                      (0x00000040)  /* erase/program protection of sector 6  */
#define FMC_OB_WP_7                      (0x00000080)  /* erase/program protection of sector 7  */
#define FMC_OB_WP_8                      (0x00000100)  /* erase/program protection of sector 8  */
#define FMC_OB_WP_9                      (0x00000200)  /* erase/program protection of sector 9  */
#define FMC_OB_WP_10                     (0x00000400)  /* erase/program protection of sector 10 */
#define FMC_OB_WP_11                     (0x00000800)  /* erase/program protection of sector 11 */
#define FMC_OB_WP_12                     (0x00001000)  /* erase/program protection of sector 12 */
#define FMC_OB_WP_13                     (0x00002000)  /* erase/program protection of sector 13 */
#define FMC_OB_WP_14                     (0x00004000)  /* erase/program protection of sector 14 */
#define FMC_OB_WP_15                     (0x00008000)  /* erase/program protection of sector 15 */
#define FMC_OB_WP_16                     (0x00010000)  /* erase/program protection of sector 16 */
#define FMC_OB_WP_17                     (0x00020000)  /* erase/program protection of sector 17 */
#define FMC_OB_WP_18                     (0x00040000)  /* erase/program protection of sector 18 */
#define FMC_OB_WP_19                     (0x00080000)  /* erase/program protection of sector 19 */
#define FMC_OB_WP_20                     (0x00100000)  /* erase/program protection of sector 20 */
#define FMC_OB_WP_21                     (0x00200000)  /* erase/program protection of sector 21 */
#define FMC_OB_WP_22                     (0x00400000)  /* erase/program protection of sector 22 */
#define FMC_OB_WP_23                     (0x00800000)  /* erase/program protection of sector 23 */
#define FMC_OB_WP_24                     (0x01000000)  /* erase/program protection of sector 24 */
#define FMC_OB_WP_25                     (0x02000000)  /* erase/program protection of sector 25 */
#define FMC_OB_WP_26                     (0x04000000)  /* erase/program protection of sector 26 */
#define FMC_OB_WP_27                     (0x08000000)  /* erase/program protection of sector 27 */
#define FMC_OB_WP_28                     (0x10000000)  /* erase/program protection of sector 28 */
#define FMC_OB_WP_29                     (0x20000000)  /* erase/program protection of sector 29 */
#define FMC_OB_WP_30                     (0x40000000)  /* erase/program protection of sector 30 */
#define FMC_OB_WP_31                     (0x80000000)  /* erase/program protection of sector 31 */
#define FMC_OB_WP_ALL                    (0xFFFFFFFF)  /* erase/program protection of all sectors */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_FMC_H */
