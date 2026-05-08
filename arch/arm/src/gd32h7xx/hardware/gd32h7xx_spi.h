/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h7xx_spi.h
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

#ifndef __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_SPI_H
#define __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI Base Addresses *******************************************************/

#define GD32_SPI0_BASE          (GD32_APB2_BUS_BASE + 0x00003000)  /* 0x40013000 */
#define GD32_SPI1_BASE          (GD32_APB1_BUS_BASE + 0x00003800)  /* 0x40003800 */
#define GD32_SPI2_BASE          (GD32_APB1_BUS_BASE + 0x00003C00)  /* 0x40003C00 */
#define GD32_SPI3_BASE          (GD32_APB2_BUS_BASE + 0x00003400)  /* 0x40013400 */
#define GD32_SPI4_BASE          (GD32_APB2_BUS_BASE + 0x00005000)  /* 0x40015000 */
#define GD32_SPI5_BASE          (GD32_APB2_BUS_BASE + 0x00003800)  /* 0x40013800 */

/* SPI Register Offsets *****************************************************/

#define GD32_SPI_CTL0_OFFSET    0x0000  /* SPI control register 0 */
#define GD32_SPI_CTL1_OFFSET    0x0004  /* SPI control register 1 */
#define GD32_SPI_CFG0_OFFSET    0x0008  /* SPI configuration register 0 */
#define GD32_SPI_CFG1_OFFSET    0x000C  /* SPI configuration register 1 */
#define GD32_SPI_INT_OFFSET     0x0010  /* SPI interrupt register */
#define GD32_SPI_STAT_OFFSET    0x0014  /* SPI status register */
#define GD32_SPI_STATC_OFFSET   0x0018  /* SPI status clear register */
#define GD32_SPI_TDATA_OFFSET   0x0020  /* SPI transmit data register */
#define GD32_SPI_RDATA_OFFSET   0x0030  /* SPI receive data register */
#define GD32_SPI_CRCPOLY_OFFSET 0x0040  /* SPI CRC polynomial register */
#define GD32_SPI_TCRC_OFFSET    0x0044  /* SPI transmit CRC register */
#define GD32_SPI_RCRC_OFFSET    0x0048  /* SPI receive CRC register */
#define GD32_SPI_URDATA_OFFSET  0x004C  /* SPI underrun data register */
#define GD32_SPI_I2SCTL_OFFSET  0x0050  /* SPI I2S control register */
#define GD32_SPI_QCTL_OFFSET    0x0080  /* SPI quad mode control register */
#define GD32_SPI_RXDLYCK_OFFSET 0x00FC  /* SPI receive clock delay register */

/* SPI Register Addresses ***************************************************/

#define GD32_SPI0               GD32_SPI0_BASE
#define GD32_SPI1               GD32_SPI1_BASE
#define GD32_SPI2               GD32_SPI2_BASE
#define GD32_SPI3               GD32_SPI3_BASE
#define GD32_SPI4               GD32_SPI4_BASE
#define GD32_SPI5               GD32_SPI5_BASE

/* SPI register addresses */

#define GD32_SPI_CTL0(spi)      ((spi) + GD32_SPI_CTL0_OFFSET)
#define GD32_SPI_CTL1(spi)      ((spi) + GD32_SPI_CTL1_OFFSET)
#define GD32_SPI_CFG0(spi)      ((spi) + GD32_SPI_CFG0_OFFSET)
#define GD32_SPI_CFG1(spi)      ((spi) + GD32_SPI_CFG1_OFFSET)
#define GD32_SPI_INT(spi)       ((spi) + GD32_SPI_INT_OFFSET)
#define GD32_SPI_STAT(spi)      ((spi) + GD32_SPI_STAT_OFFSET)
#define GD32_SPI_STATC(spi)     ((spi) + GD32_SPI_STATC_OFFSET)
#define GD32_SPI_TDATA(spi)     ((spi) + GD32_SPI_TDATA_OFFSET)
#define GD32_SPI_RDATA(spi)     ((spi) + GD32_SPI_RDATA_OFFSET)
#define GD32_SPI_CRCPOLY(spi)   ((spi) + GD32_SPI_CRCPOLY_OFFSET)
#define GD32_SPI_TCRC(spi)      ((spi) + GD32_SPI_TCRC_OFFSET)
#define GD32_SPI_RCRC(spi)      ((spi) + GD32_SPI_RCRC_OFFSET)
#define GD32_SPI_URDATA(spi)    ((spi) + GD32_SPI_URDATA_OFFSET)
#define GD32_SPI_I2SCTL(spi)    ((spi) + GD32_SPI_I2SCTL_OFFSET)
#define GD32_SPI_QCTL(spi)      ((spi) + GD32_SPI_QCTL_OFFSET)
#define GD32_SPI_RXDLYCK(spi)   ((spi) + GD32_SPI_RXDLYCK_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SPI Control Register 0 (SPI_CTL0) */

#define SPI_CTL0_SPIEN          (1u << 0)   /* Bit 0: SPI enable */
#define SPI_CTL0_MASP           (1u << 8)   /* Bit 8: Master auto suspend in receive mode */
#define SPI_CTL0_MSTART         (1u << 9)   /* Bit 9: Master start transfer */
#define SPI_CTL0_MSPDR          (1u << 10)  /* Bit 10: Master suspend request */
#define SPI_CTL0_NSSI           (1u << 12)  /* Bit 12: Internal NSS signal level */
#define SPI_CTL0_CRCFS          (1u << 13)  /* Bit 13: Full-size CRC polynomial */
#define SPI_CTL0_RXCRCI         (1u << 14)  /* Bit 14: RX CRC initialization pattern */
#define SPI_CTL0_TXCRCI         (1u << 15)  /* Bit 15: TX CRC initialization pattern */
#define SPI_CTL0_IOAFEN         (1u << 16)  /* Bit 16: IO AF enable */

/* SPI Control Register 1 (SPI_CTL1) */

#define SPI_CTL1_TXSIZE_SHIFT   (0)        /* Bits 0-15: Transfer size */
#define SPI_CTL1_TXSIZE_MASK    (0xffffu << SPI_CTL1_TXSIZE_SHIFT)
#define SPI_CTL1_TXSIZE(n)      ((n) << SPI_CTL1_TXSIZE_SHIFT)

#define SPI_CTL1_TXSER_SHIFT    (16)       /* Bits 16-31: Transfer serial reload */
#define SPI_CTL1_TXSER_MASK     (0xffffu << SPI_CTL1_TXSER_SHIFT)
#define SPI_CTL1_TXSER(n)       ((n) << SPI_CTL1_TXSER_SHIFT)

/* SPI Configuration Register 0 (SPI_CFG0) */

#define SPI_CFG0_DZ_SHIFT       (0)        /* Bits 0-4: Data size */
#define SPI_CFG0_DZ_MASK        (0x1fu << SPI_CFG0_DZ_SHIFT)
#define SPI_CFG0_DZ(n)          (((n) - 1) << SPI_CFG0_DZ_SHIFT)  /* n = 4..32 bits */
#define SPI_CFG0_DZ_4BIT        SPI_CFG0_DZ(4)
#define SPI_CFG0_DZ_5BIT        SPI_CFG0_DZ(5)
#define SPI_CFG0_DZ_6BIT        SPI_CFG0_DZ(6)
#define SPI_CFG0_DZ_7BIT        SPI_CFG0_DZ(7)
#define SPI_CFG0_DZ_8BIT        SPI_CFG0_DZ(8)
#define SPI_CFG0_DZ_9BIT        SPI_CFG0_DZ(9)
#define SPI_CFG0_DZ_10BIT       SPI_CFG0_DZ(10)
#define SPI_CFG0_DZ_11BIT       SPI_CFG0_DZ(11)
#define SPI_CFG0_DZ_12BIT       SPI_CFG0_DZ(12)
#define SPI_CFG0_DZ_13BIT       SPI_CFG0_DZ(13)
#define SPI_CFG0_DZ_14BIT       SPI_CFG0_DZ(14)
#define SPI_CFG0_DZ_15BIT       SPI_CFG0_DZ(15)
#define SPI_CFG0_DZ_16BIT       SPI_CFG0_DZ(16)
#define SPI_CFG0_DZ_32BIT       SPI_CFG0_DZ(32)

#define SPI_CFG0_FIFOLVL_SHIFT  (5)        /* Bits 5-8: FIFO threshold level */
#define SPI_CFG0_FIFOLVL_MASK   (0x0fu << SPI_CFG0_FIFOLVL_SHIFT)
#define SPI_CFG0_FIFOLVL(n)     (((n) - 1) << SPI_CFG0_FIFOLVL_SHIFT)  /* n = 1..16 */
#define SPI_CFG0_FIFOLVL_1      SPI_CFG0_FIFOLVL(1)

#define SPI_CFG0_TXUROP_SHIFT   (9)        /* Bits 9-10: Underrun operation */
#define SPI_CFG0_TXUROP_MASK    (0x03u << SPI_CFG0_TXUROP_SHIFT)
#define SPI_CFG0_TXUROP_CONST   (0u << SPI_CFG0_TXUROP_SHIFT)  /* Send constant from URDATA */
#define SPI_CFG0_TXUROP_LAST    (1u << SPI_CFG0_TXUROP_SHIFT)  /* Send last received data */
#define SPI_CFG0_TXUROP_LASTTX  (2u << SPI_CFG0_TXUROP_SHIFT)  /* Send last transmitted data */

#define SPI_CFG0_TXURDT_SHIFT   (11)       /* Bits 11-12: Underrun detection */
#define SPI_CFG0_TXURDT_MASK    (0x03u << SPI_CFG0_TXURDT_SHIFT)
#define SPI_CFG0_TXURDT_START   (0u << SPI_CFG0_TXURDT_SHIFT)  /* At frame start */
#define SPI_CFG0_TXURDT_END     (1u << SPI_CFG0_TXURDT_SHIFT)  /* At last frame end */
#define SPI_CFG0_TXURDT_NSS     (2u << SPI_CFG0_TXURDT_SHIFT)  /* At NSS start */

#define SPI_CFG0_DMAREN         (1u << 14)  /* Bit 14: RX DMA enable */
#define SPI_CFG0_DMATEN         (1u << 15)  /* Bit 15: TX DMA enable */

#define SPI_CFG0_CRCSZ_SHIFT    (16)       /* Bits 16-20: CRC size */
#define SPI_CFG0_CRCSZ_MASK     (0x1fu << SPI_CFG0_CRCSZ_SHIFT)
#define SPI_CFG0_CRCSZ(n)       (((n) - 1) << SPI_CFG0_CRCSZ_SHIFT)

#define SPI_CFG0_CRCEN          (1u << 22)  /* Bit 22: CRC enable */
#define SPI_CFG0_BYTEN          (1u << 23)  /* Bit 23: Byte access enable */
#define SPI_CFG0_WORDEN         (1u << 24)  /* Bit 24: Word access enable */

#define SPI_CFG0_PSC_SHIFT      (28)       /* Bits 28-30: Prescaler */
#define SPI_CFG0_PSC_MASK       (0x07u << SPI_CFG0_PSC_SHIFT)
#define SPI_CFG0_PSC_2          (0u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/2 */
#define SPI_CFG0_PSC_4          (1u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/4 */
#define SPI_CFG0_PSC_8          (2u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/8 */
#define SPI_CFG0_PSC_16         (3u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/16 */
#define SPI_CFG0_PSC_32         (4u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/32 */
#define SPI_CFG0_PSC_64         (5u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/64 */
#define SPI_CFG0_PSC_128        (6u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/128 */
#define SPI_CFG0_PSC_256        (7u << SPI_CFG0_PSC_SHIFT)  /* KERCLK/256 */

/* SPI Configuration Register 1 (SPI_CFG1) */

#define SPI_CFG1_MSSD_SHIFT     (0)        /* Bits 0-3: NSS to data delay */
#define SPI_CFG1_MSSD_MASK      (0x0fu << SPI_CFG1_MSSD_SHIFT)
#define SPI_CFG1_MSSD(n)        ((n) << SPI_CFG1_MSSD_SHIFT)

#define SPI_CFG1_MDFD_SHIFT     (4)        /* Bits 4-7: Data frame delay */
#define SPI_CFG1_MDFD_MASK      (0x0fu << SPI_CFG1_MDFD_SHIFT)
#define SPI_CFG1_MDFD(n)        ((n) << SPI_CFG1_MDFD_SHIFT)

#define SPI_CFG1_SWPMIO         (1u << 15)  /* Bit 15: Swap MISO/MOSI */
#define SPI_CFG1_RO             (1u << 16)  /* Bit 16: Receive only mode */
#define SPI_CFG1_BDOEN          (1u << 17)  /* Bit 17: Bidirectional output enable */
#define SPI_CFG1_BDEN           (1u << 18)  /* Bit 18: Bidirectional mode enable */
#define SPI_CFG1_TMOD           (1u << 21)  /* Bit 21: TI mode enable */
#define SPI_CFG1_MSTMOD         (1u << 22)  /* Bit 22: Master mode */
#define SPI_CFG1_LF             (1u << 23)  /* Bit 23: LSB first */
#define SPI_CFG1_CKPH           (1u << 24)  /* Bit 24: Clock phase */
#define SPI_CFG1_CKPL           (1u << 25)  /* Bit 25: Clock polarity */
#define SPI_CFG1_NSSIM          (1u << 26)  /* Bit 26: NSS input management */
#define SPI_CFG1_NSSIOPL        (1u << 28)  /* Bit 28: NSS I/O polarity */
#define SPI_CFG1_NSSDRV         (1u << 29)  /* Bit 29: NSS output drive enable */
#define SPI_CFG1_NSSCTL         (1u << 30)  /* Bit 30: NSS control */
#define SPI_CFG1_AFCTL          (1u << 31)  /* Bit 31: AF GPIO control */

/* SPI Interrupt Register (SPI_INT) */

#define SPI_INT_RPIE            (1u << 0)   /* Bit 0: RX packet interrupt enable */
#define SPI_INT_TPIE            (1u << 1)   /* Bit 1: TX packet interrupt enable */
#define SPI_INT_DPIE            (1u << 2)   /* Bit 2: Duplex packet interrupt enable */
#define SPI_INT_ESTCIE          (1u << 3)   /* Bit 3: End/suspend/TC interrupt enable */
#define SPI_INT_TXFIE           (1u << 4)   /* Bit 4: TX filled interrupt enable */
#define SPI_INT_TXUREIE         (1u << 5)   /* Bit 5: TX underrun interrupt enable */
#define SPI_INT_RXOREIE         (1u << 6)   /* Bit 6: RX overrun interrupt enable */
#define SPI_INT_CRCERIE         (1u << 7)   /* Bit 7: CRC error interrupt enable */
#define SPI_INT_FEIE            (1u << 8)   /* Bit 8: Frame error interrupt enable */
#define SPI_INT_CONFEIE         (1u << 9)   /* Bit 9: Config error interrupt enable */
#define SPI_INT_TXSERFIE        (1u << 10)  /* Bit 10: TXSER reload interrupt enable */

/* SPI Status Register (SPI_STAT) */

#define SPI_STAT_RP             (1u << 0)   /* Bit 0: RX packet available */
#define SPI_STAT_TP             (1u << 1)   /* Bit 1: TX packet space available */
#define SPI_STAT_DP             (1u << 2)   /* Bit 2: Duplex packet */
#define SPI_STAT_ET             (1u << 3)   /* Bit 3: End of transfer */
#define SPI_STAT_TXF            (1u << 4)   /* Bit 4: TX FIFO filled */
#define SPI_STAT_TXURERR        (1u << 5)   /* Bit 5: TX underrun error */
#define SPI_STAT_RXORERR        (1u << 6)   /* Bit 6: RX overrun error */
#define SPI_STAT_CRCERR         (1u << 7)   /* Bit 7: CRC error */
#define SPI_STAT_FERR           (1u << 8)   /* Bit 8: Frame format error */
#define SPI_STAT_CONFERR        (1u << 9)   /* Bit 9: Configuration error */
#define SPI_STAT_TXSERF         (1u << 10)  /* Bit 10: TXSER reloaded */
#define SPI_STAT_SPD            (1u << 11)  /* Bit 11: Suspend flag */
#define SPI_STAT_TC             (1u << 12)  /* Bit 12: Transfer complete */

#define SPI_STAT_RPLVL_SHIFT    (13)       /* Bits 13-14: RX FIFO packing level */
#define SPI_STAT_RPLVL_MASK     (0x03u << SPI_STAT_RPLVL_SHIFT)

#define SPI_STAT_RWNE           (1u << 15)  /* Bit 15: RX word not empty */

#define SPI_STAT_CTXSIZE_SHIFT  (16)       /* Bits 16-31: Current transfer size */
#define SPI_STAT_CTXSIZE_MASK   (0xffffu << SPI_STAT_CTXSIZE_SHIFT)

/* SPI Status Clear Register (SPI_STATC) */

#define SPI_STATC_ETC           (1u << 3)   /* Bit 3: Clear ET flag */
#define SPI_STATC_TXFC          (1u << 4)   /* Bit 4: Clear TXF flag */
#define SPI_STATC_TXURERRC      (1u << 5)   /* Bit 5: Clear TX underrun error */
#define SPI_STATC_RXORERRC      (1u << 6)   /* Bit 6: Clear RX overrun error */
#define SPI_STATC_CRCERRC       (1u << 7)   /* Bit 7: Clear CRC error */
#define SPI_STATC_FERRC         (1u << 8)   /* Bit 8: Clear frame error */
#define SPI_STATC_CONFERRC      (1u << 9)   /* Bit 9: Clear config error */
#define SPI_STATC_TXSERFC       (1u << 10)  /* Bit 10: Clear TXSER flag */
#define SPI_STATC_SPDC          (1u << 11)  /* Bit 11: Clear suspend flag */

/* SPI I2S Control Register (SPI_I2SCTL) */

#define SPI_I2SCTL_CHLEN        (1u << 0)   /* Bit 0: Channel length */
#define SPI_I2SCTL_DTLEN_SHIFT  (1)         /* Bits 1-2: Data length */
#define SPI_I2SCTL_DTLEN_MASK   (0x03u << SPI_I2SCTL_DTLEN_SHIFT)
#define SPI_I2SCTL_DTLEN_16     (0u << SPI_I2SCTL_DTLEN_SHIFT)
#define SPI_I2SCTL_DTLEN_24     (1u << SPI_I2SCTL_DTLEN_SHIFT)
#define SPI_I2SCTL_DTLEN_32     (2u << SPI_I2SCTL_DTLEN_SHIFT)
#define SPI_I2SCTL_CKPL         (1u << 3)   /* Bit 3: Clock polarity */
#define SPI_I2SCTL_I2SSTD_SHIFT (4)         /* Bits 4-5: I2S standard */
#define SPI_I2SCTL_I2SSTD_MASK  (0x03u << SPI_I2SCTL_I2SSTD_SHIFT)
#define SPI_I2SCTL_PCMSMOD      (1u << 7)   /* Bit 7: PCM frame sync mode */
#define SPI_I2SCTL_I2SOPMOD_SHIFT (8)       /* Bits 8-9: I2S operation mode */
#define SPI_I2SCTL_I2SOPMOD_MASK  (0x03u << SPI_I2SCTL_I2SOPMOD_SHIFT)
#define SPI_I2SCTL_I2SEN        (1u << 10)  /* Bit 10: I2S enable */
#define SPI_I2SCTL_I2SSEL       (1u << 11)  /* Bit 11: I2S mode select */
#define SPI_I2SCTL_DIV_SHIFT    (16)        /* Bits 16-23: Divider */
#define SPI_I2SCTL_DIV_MASK     (0xffu << SPI_I2SCTL_DIV_SHIFT)
#define SPI_I2SCTL_OF           (1u << 24)  /* Bit 24: Odd factor */
#define SPI_I2SCTL_MCKOEN       (1u << 25)  /* Bit 25: MCK output enable */
#define SPI_I2SCTL_I2SCH        (1u << 31)  /* Bit 31: I2S channel */

/* SPI Quad Mode Control Register (SPI_QCTL) - SPI3/4 only */

#define SPI_QCTL_QMOD           (1u << 0)   /* Bit 0: Quad SPI mode enable */
#define SPI_QCTL_QRD            (1u << 1)   /* Bit 1: Quad SPI read select */
#define SPI_QCTL_IO23_DRV       (1u << 2)   /* Bit 2: IO2/IO3 output enable */

/* SPI Receive Clock Delay Register (SPI_RXDLYCK) */

#define SPI_RXDLYCK_SRXD_SHIFT  (0)        /* Bits 0-4: Slave RX delay */
#define SPI_RXDLYCK_SRXD_MASK   (0x1fu << SPI_RXDLYCK_SRXD_SHIFT)
#define SPI_RXDLYCK_SRXDEN      (1u << 5)   /* Bit 5: Slave RX delay enable */
#define SPI_RXDLYCK_MRXD_SHIFT  (6)         /* Bits 6-10: Master RX delay */
#define SPI_RXDLYCK_MRXD_MASK   (0x1fu << SPI_RXDLYCK_MRXD_SHIFT)
#define SPI_RXDLYCK_MRXDEN      (1u << 11)  /* Bit 11: Master RX delay enable */

#endif /* __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_SPI_H */
