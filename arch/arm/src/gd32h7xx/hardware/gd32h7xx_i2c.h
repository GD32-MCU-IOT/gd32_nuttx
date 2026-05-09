/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h7xx_i2c.h
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

#ifndef __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_I2C_H
#define __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2Cx(x=0,1,2,3) base addresses */

#define GD32_I2C0_BASE             (GD32_I2C_BASE + 0x00000000)
#define GD32_I2C1_BASE             (GD32_I2C_BASE + 0x00000400)
#define GD32_I2C2_BASE             (GD32_I2C_BASE + 0x00006C00)
#define GD32_I2C3_BASE             (GD32_I2C_BASE + 0x00000800)

/* Register Offsets *********************************************************/

#define GD32_I2C_CTL0_OFFSET       0x0000  /* I2C control register 0 */
#define GD32_I2C_CTL1_OFFSET       0x0004  /* I2C control register 1 */
#define GD32_I2C_SADDR0_OFFSET     0x0008  /* I2C slave address register 0 */
#define GD32_I2C_SADDR1_OFFSET     0x000C  /* I2C slave address register 1 */
#define GD32_I2C_TIMING_OFFSET     0x0010  /* I2C timing register */
#define GD32_I2C_TIMEOUT_OFFSET    0x0014  /* I2C timeout register */
#define GD32_I2C_STAT_OFFSET       0x0018  /* I2C status register */
#define GD32_I2C_STATC_OFFSET      0x001C  /* I2C status clear register */
#define GD32_I2C_PEC_OFFSET        0x0020  /* I2C PEC register */
#define GD32_I2C_RDATA_OFFSET      0x0024  /* I2C receive data register */
#define GD32_I2C_TDATA_OFFSET      0x0028  /* I2C transmit data register */
#define GD32_I2C_CTL2_OFFSET       0x0090  /* I2C control register 2 */

/* Register Addresses *******************************************************/

#define GD32_I2C0                  GD32_I2C0_BASE
#define GD32_I2C1                  GD32_I2C1_BASE
#define GD32_I2C2                  GD32_I2C2_BASE
#define GD32_I2C3                  GD32_I2C3_BASE

/* I2C register definitions */

#define GD32_I2C_CTL0(i2cx)        ((i2cx) + GD32_I2C_CTL0_OFFSET)
#define GD32_I2C_CTL1(i2cx)        ((i2cx) + GD32_I2C_CTL1_OFFSET)
#define GD32_I2C_SADDR0(i2cx)      ((i2cx) + GD32_I2C_SADDR0_OFFSET)
#define GD32_I2C_SADDR1(i2cx)      ((i2cx) + GD32_I2C_SADDR1_OFFSET)
#define GD32_I2C_TIMING(i2cx)      ((i2cx) + GD32_I2C_TIMING_OFFSET)
#define GD32_I2C_TIMEOUT(i2cx)     ((i2cx) + GD32_I2C_TIMEOUT_OFFSET)
#define GD32_I2C_STAT(i2cx)        ((i2cx) + GD32_I2C_STAT_OFFSET)
#define GD32_I2C_STATC(i2cx)       ((i2cx) + GD32_I2C_STATC_OFFSET)
#define GD32_I2C_PEC(i2cx)         ((i2cx) + GD32_I2C_PEC_OFFSET)
#define GD32_I2C_RDATA(i2cx)       ((i2cx) + GD32_I2C_RDATA_OFFSET)
#define GD32_I2C_TDATA(i2cx)       ((i2cx) + GD32_I2C_TDATA_OFFSET)
#define GD32_I2C_CTL2(i2cx)        ((i2cx) + GD32_I2C_CTL2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* I2C_CTL0 - Control register 0 */

#define I2C_CTL0_I2CEN             (1 << 0)   /* Bit 0: I2C peripheral enable */
#define I2C_CTL0_TIE               (1 << 1)   /* Bit 1: Transmit interrupt enable */
#define I2C_CTL0_RBNEIE            (1 << 2)   /* Bit 2: Receive interrupt enable */
#define I2C_CTL0_ADDMIE            (1 << 3)   /* Bit 3: Address match interrupt enable */
#define I2C_CTL0_NACKIE            (1 << 4)   /* Bit 4: NACK interrupt enable */
#define I2C_CTL0_STPDETIE          (1 << 5)   /* Bit 5: Stop detection interrupt enable */
#define I2C_CTL0_TCIE              (1 << 6)   /* Bit 6: Transfer complete interrupt enable */
#define I2C_CTL0_ERRIE             (1 << 7)   /* Bit 7: Error interrupt enable */
#define I2C_CTL0_DNF_SHIFT         (8)        /* Bits 11-8: Digital noise filter */
#define I2C_CTL0_DNF_MASK          (0x0f << I2C_CTL0_DNF_SHIFT)
#define I2C_CTL0_DNF(n)            ((n) << I2C_CTL0_DNF_SHIFT)
#define I2C_CTL0_ANOFF             (1 << 12)  /* Bit 12: Analog noise filter off */
#define I2C_CTL0_DENT              (1 << 14)  /* Bit 14: DMA enable for transmission */
#define I2C_CTL0_DENR              (1 << 15)  /* Bit 15: DMA enable for reception */
#define I2C_CTL0_SBCTL             (1 << 16)  /* Bit 16: Slave byte control */
#define I2C_CTL0_SS                (1 << 17)  /* Bit 17: Clock stretching disable */
#define I2C_CTL0_WUEN              (1 << 18)  /* Bit 18: Wakeup from deep-sleep enable */
#define I2C_CTL0_GCEN              (1 << 19)  /* Bit 19: General call enable */
#define I2C_CTL0_SMBHAEN           (1 << 20)  /* Bit 20: SMBus host address enable */
#define I2C_CTL0_SMBDAEN           (1 << 21)  /* Bit 21: SMBus device default address enable */
#define I2C_CTL0_SMBALTEN          (1 << 22)  /* Bit 22: SMBus alert enable */
#define I2C_CTL0_PECEN             (1 << 23)  /* Bit 23: PEC enable */

#define I2C_CTL0_INT_MASK          (I2C_CTL0_ERRIE | I2C_CTL0_TCIE | \
                                    I2C_CTL0_STPDETIE | I2C_CTL0_NACKIE | \
                                    I2C_CTL0_ADDMIE | I2C_CTL0_RBNEIE | \
                                    I2C_CTL0_TIE)

/* I2C_CTL1 - Control register 1 */

#define I2C_CTL1_SADDRESS_SHIFT    (0)        /* Bits 9-0: Slave address */
#define I2C_CTL1_SADDRESS_MASK     (0x3ff << I2C_CTL1_SADDRESS_SHIFT)
#define I2C_CTL1_SADDRESS(n)       ((n) << I2C_CTL1_SADDRESS_SHIFT)
#define I2C_CTL1_TRDIR             (1 << 10)  /* Bit 10: Transfer direction */
#define I2C_CTL1_ADD10EN           (1 << 11)  /* Bit 11: 10-bit addressing enable */
#define I2C_CTL1_HEAD10R           (1 << 12)  /* Bit 12: 10-bit header read */
#define I2C_CTL1_START             (1 << 13)  /* Bit 13: Start generation */
#define I2C_CTL1_STOP              (1 << 14)  /* Bit 14: Stop generation */
#define I2C_CTL1_NACKEN            (1 << 15)  /* Bit 15: NACK generation */
#define I2C_CTL1_BYTENUM_SHIFT    (16)        /* Bits 23-16: Number of bytes */
#define I2C_CTL1_BYTENUM_MASK     (0xff << I2C_CTL1_BYTENUM_SHIFT)
#define I2C_CTL1_BYTENUM(n)       ((n) << I2C_CTL1_BYTENUM_SHIFT)
#define I2C_CTL1_RELOAD            (1 << 24)  /* Bit 24: Reload mode */
#define I2C_CTL1_AUTOEND           (1 << 25)  /* Bit 25: Automatic end mode */
#define I2C_CTL1_PECTRANS          (1 << 26)  /* Bit 26: PEC transfer */

/* Transfer direction */

#define I2C_MASTER_TRANSMIT        (0)
#define I2C_MASTER_RECEIVE         I2C_CTL1_TRDIR

/* I2C_SADDR0 - Slave address register 0 */

#define I2C_SADDR0_ADDRESS0        (1 << 0)   /* Bit 0: Bit 0 of 10-bit address */
#define I2C_SADDR0_ADDRESS_SHIFT   (1)        /* Bits 7-1: 7-bit address */
#define I2C_SADDR0_ADDRESS_MASK    (0x7f << I2C_SADDR0_ADDRESS_SHIFT)
#define I2C_SADDR0_ADDRESS_H_SHIFT (8)        /* Bits 9-8: High bits of 10-bit address */
#define I2C_SADDR0_ADDRESS_H_MASK  (0x3 << I2C_SADDR0_ADDRESS_H_SHIFT)
#define I2C_SADDR0_ADDFORMAT       (1 << 10)  /* Bit 10: Address mode (0=7bit, 1=10bit) */
#define I2C_SADDR0_ADDRESSEN       (1 << 15)  /* Bit 15: Address enable */

/* I2C_SADDR1 - Slave address register 1 */

#define I2C_SADDR1_ADDRESS2_SHIFT  (1)
#define I2C_SADDR1_ADDRESS2_MASK   (0x7f << I2C_SADDR1_ADDRESS2_SHIFT)
#define I2C_SADDR1_ADDMSK2_SHIFT   (8)
#define I2C_SADDR1_ADDMSK2_MASK    (0x7 << I2C_SADDR1_ADDMSK2_SHIFT)
#define I2C_SADDR1_ADDRESS2EN      (1 << 15)  /* Bit 15: Second address enable */

/* I2C_TIMING - Timing register */

#define I2C_TIMING_SCLL_SHIFT      (0)        /* Bits 7-0: SCL low period */
#define I2C_TIMING_SCLL_MASK       (0xff << I2C_TIMING_SCLL_SHIFT)
#define I2C_TIMING_SCLH_SHIFT      (8)        /* Bits 15-8: SCL high period */
#define I2C_TIMING_SCLH_MASK       (0xff << I2C_TIMING_SCLH_SHIFT)
#define I2C_TIMING_SDADELY_SHIFT   (16)       /* Bits 19-16: Data hold time */
#define I2C_TIMING_SDADELY_MASK    (0x0f << I2C_TIMING_SDADELY_SHIFT)
#define I2C_TIMING_SCLDELY_SHIFT   (20)       /* Bits 23-20: Data setup time */
#define I2C_TIMING_SCLDELY_MASK    (0x0f << I2C_TIMING_SCLDELY_SHIFT)
#define I2C_TIMING_PSC_SHIFT       (28)       /* Bits 31-28: Timing prescaler */
#define I2C_TIMING_PSC_MASK        (0x0f << I2C_TIMING_PSC_SHIFT)

/* I2C_TIMEOUT - Timeout register */

#define I2C_TIMEOUT_BUSTOA_SHIFT   (0)        /* Bits 11-0: Bus timeout A */
#define I2C_TIMEOUT_BUSTOA_MASK    (0xfff << I2C_TIMEOUT_BUSTOA_SHIFT)
#define I2C_TIMEOUT_TOIDLE         (1 << 12)  /* Bit 12: Idle clock timeout detection */
#define I2C_TIMEOUT_TOEN           (1 << 15)  /* Bit 15: Clock timeout enable */
#define I2C_TIMEOUT_BUSTOB_SHIFT   (16)       /* Bits 27-16: Bus timeout B */
#define I2C_TIMEOUT_BUSTOB_MASK    (0xfff << I2C_TIMEOUT_BUSTOB_SHIFT)
#define I2C_TIMEOUT_EXTOEN         (1 << 31)  /* Bit 31: Extended clock timeout enable */

/* I2C_STAT - Status register */

#define I2C_STAT_TBE               (1 << 0)   /* Bit 0: Transmit data empty */
#define I2C_STAT_TI                (1 << 1)   /* Bit 1: Transmit interrupt */
#define I2C_STAT_RBNE              (1 << 2)   /* Bit 2: Receive data not empty */
#define I2C_STAT_ADDSEND           (1 << 3)   /* Bit 3: Address matched (slave) */
#define I2C_STAT_NACK              (1 << 4)   /* Bit 4: NACK received */
#define I2C_STAT_STPDET            (1 << 5)   /* Bit 5: Stop detected (slave) */
#define I2C_STAT_TC                (1 << 6)   /* Bit 6: Transfer complete (master) */
#define I2C_STAT_TCR               (1 << 7)   /* Bit 7: Transfer complete reload */
#define I2C_STAT_BERR              (1 << 8)   /* Bit 8: Bus error */
#define I2C_STAT_LOSTARB           (1 << 9)   /* Bit 9: Arbitration lost */
#define I2C_STAT_OUERR             (1 << 10)  /* Bit 10: Overrun/underrun error */
#define I2C_STAT_PECERR            (1 << 11)  /* Bit 11: PEC error */
#define I2C_STAT_TIMEOUT           (1 << 12)  /* Bit 12: Timeout */
#define I2C_STAT_SMBALT            (1 << 13)  /* Bit 13: SMBus alert */
#define I2C_STAT_I2CBSY            (1 << 15)  /* Bit 15: Bus busy */
#define I2C_STAT_TR                (1 << 16)  /* Bit 16: Transmitter/receiver */
#define I2C_STAT_READDR_SHIFT      (17)       /* Bits 23-17: Received address */
#define I2C_STAT_READDR_MASK       (0x7f << I2C_STAT_READDR_SHIFT)

#define I2C_STAT_ERROR_MASK        (I2C_STAT_BERR | I2C_STAT_LOSTARB | \
                                    I2C_STAT_OUERR | I2C_STAT_PECERR | \
                                    I2C_STAT_TIMEOUT | I2C_STAT_SMBALT)

/* I2C_STATC - Status clear register */

#define I2C_STATC_ADDSENDC         (1 << 3)   /* Bit 3: Clear ADDSEND */
#define I2C_STATC_NACKC            (1 << 4)   /* Bit 4: Clear NACK */
#define I2C_STATC_STPDETC          (1 << 5)   /* Bit 5: Clear STPDET */
#define I2C_STATC_BERRC            (1 << 8)   /* Bit 8: Clear BERR */
#define I2C_STATC_LOSTARBC         (1 << 9)   /* Bit 9: Clear LOSTARB */
#define I2C_STATC_OUERRC           (1 << 10)  /* Bit 10: Clear OUERR */
#define I2C_STATC_PECERRC          (1 << 11)  /* Bit 11: Clear PECERR */
#define I2C_STATC_TIMEOUTC         (1 << 12)  /* Bit 12: Clear TIMEOUT */
#define I2C_STATC_SMBALTC          (1 << 13)  /* Bit 13: Clear SMBALT */

#define I2C_STATC_ALL              (I2C_STATC_ADDSENDC | I2C_STATC_NACKC | \
                                    I2C_STATC_STPDETC | I2C_STATC_BERRC | \
                                    I2C_STATC_LOSTARBC | I2C_STATC_OUERRC | \
                                    I2C_STATC_PECERRC | I2C_STATC_TIMEOUTC | \
                                    I2C_STATC_SMBALTC)

/* I2C_PEC */

#define I2C_PEC_PECV_SHIFT         (0)
#define I2C_PEC_PECV_MASK          (0xff << I2C_PEC_PECV_SHIFT)

/* I2C_RDATA */

#define I2C_RDATA_RDATA_SHIFT      (0)
#define I2C_RDATA_RDATA_MASK       (0xff << I2C_RDATA_RDATA_SHIFT)

/* I2C_TDATA */

#define I2C_TDATA_TDATA_SHIFT      (0)
#define I2C_TDATA_TDATA_MASK       (0xff << I2C_TDATA_TDATA_SHIFT)

/* Maximum reload size */

#define I2C_MAX_RELOAD_SIZE        255

#endif /* __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_I2C_H */
