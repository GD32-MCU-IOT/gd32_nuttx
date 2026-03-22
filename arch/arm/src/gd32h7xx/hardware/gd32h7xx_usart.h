/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h7xx_usart.h
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

#ifndef __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_USART_H
#define __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_USART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USARTx(x=0,1,2,5)/UARTx(x=3,4,6,7) base addresses */

#define GD32_USART1_BASE           (GD32_USART_BASE+0x00000000)    /* USART1 base address */
#define GD32_USART2_BASE           (GD32_USART_BASE+0x00000400)    /* USART2 base address */
#define GD32_UART3_BASE            (GD32_USART_BASE+0x00000800)    /* UART3 base address */
#define GD32_UART4_BASE            (GD32_USART_BASE+0x00000C00)    /* UART4 base address */
#define GD32_UART6_BASE            (GD32_USART_BASE+0x00003400)    /* UART6 base address */
#define GD32_UART7_BASE            (GD32_USART_BASE+0x00003800)    /* UART7 base address */
#define GD32_USART0_BASE           (GD32_USART_BASE+0x0000CC00)    /* USART0 base address */
#define GD32_USART5_BASE           (GD32_USART_BASE+0x0000D000)    /* USART5 base address */

/* Register Offsets *********************************************************/

#define GD32_USART_CTL0_OFFSET     0x0000  /* USART control register 0 offset */
#define GD32_USART_CTL1_OFFSET     0x0004  /* USART control register 1 offset */
#define GD32_USART_CTL2_OFFSET     0x0008  /* USART control register 2 offset */
#define GD32_USART_BAUD_OFFSET     0x000C  /* USART baud rate register offset */
#define GD32_USART_GP_OFFSET       0x0010  /* USART guard time and prescaler register offset */
#define GD32_USART_RT_OFFSET       0x0014  /* USART receiver timeout register offset */
#define GD32_USART_CMD_OFFSET      0x0018  /* USART command register offset */
#define GD32_USART_STAT_OFFSET     0x001C  /* USART status register offset */
#define GD32_USART_INTC_OFFSET     0x0020  /* USART interrupt status clear register offset */
#define GD32_USART_RDATA_OFFSET    0x0024  /* USART receive data register offset */
#define GD32_USART_TDATA_OFFSET    0x0028  /* USART transmit data register offset */
#define GD32_USART_CHC_OFFSET      0x00C0  /* USART coherence control register offset */
#define GD32_USART_FCS_OFFSET      0x00D0  /* USART FIFO control and status register offset */

/* Register Addresses *******************************************************/

#define GD32_USART0                GD32_USART0_BASE
#define GD32_USART1                GD32_USART1_BASE
#define GD32_USART2                GD32_USART2_BASE
#define GD32_UART3                 GD32_UART3_BASE
#define GD32_UART4                 GD32_UART4_BASE
#define GD32_USART5                GD32_USART5_BASE
#define GD32_UART6                 GD32_UART6_BASE
#define GD32_UART7                 GD32_UART7_BASE

/* USART0,1,2,5 Register ****************************************************/

#define GD32_USART_CTL0(usartx)    ((usartx)+GD32_USART_CTL0_OFFSET)     /* USART control register 0 */
#define GD32_USART_CTL1(usartx)    ((usartx)+GD32_USART_CTL1_OFFSET)     /* USART control register 1 */
#define GD32_USART_CTL2(usartx)    ((usartx)+GD32_USART_CTL2_OFFSET)     /* USART control register 2 */
#define GD32_USART_BAUD(usartx)    ((usartx)+GD32_USART_BAUD_OFFSET)     /* USART baud rate register */
#define GD32_USART_GP(usartx)      ((usartx)+GD32_USART_GP_OFFSET)       /* USART guard time and prescaler register */
#define GD32_USART_RT(usartx)      ((usartx)+GD32_USART_RT_OFFSET)       /* USART receiver timeout register */
#define GD32_USART_CMD(usartx)     ((usartx)+GD32_USART_CMD_OFFSET)      /* USART command register */
#define GD32_USART_STAT(usartx)    ((usartx)+GD32_USART_STAT_OFFSET)     /* USART status register */
#define GD32_USART_INTC(usartx)    ((usartx)+GD32_USART_INTC_OFFSET)     /* USART interrupt status clear register */
#define GD32_USART_RDATA(usartx)   ((usartx)+GD32_USART_RDATA_OFFSET)    /* USART receive data register */
#define GD32_USART_TDATA(usartx)   ((usartx)+GD32_USART_TDATA_OFFSET)    /* USART transmit data register */
#define GD32_USART_CHC(usartx)     ((usartx)+GD32_USART_CHC_OFFSET)      /* USART coherence control register */
#define GD32_USART_FCS(usartx)     ((usartx)+GD32_USART_FCS_OFFSET)      /* USART FIFO control and status register */

/* UART3,4,6,7 Register *****************************************************/

#define GD32_UART_CTL0(uartx)      ((uartx)+GD32_USART_CTL0_OFFSET)      /* UART control register 0 */
#define GD32_UART_CTL1(uartx)      ((uartx)+GD32_USART_CTL1_OFFSET)      /* UART control register 1 */
#define GD32_UART_CTL2(uartx)      ((uartx)+GD32_USART_CTL2_OFFSET)      /* UART control register 2 */
#define GD32_UART_BAUD(uartx)      ((uartx)+GD32_USART_BAUD_OFFSET)      /* UART baud rate register */
#define GD32_UART_GP(uartx)        ((uartx)+GD32_USART_GP_OFFSET)        /* UART guard time and prescaler register */
#define GD32_UART_RT(uartx)        ((uartx)+GD32_USART_RT_OFFSET)        /* UART receiver timeout register */
#define GD32_UART_CMD(uartx)       ((uartx)+GD32_USART_CMD_OFFSET)       /* UART command register */
#define GD32_UART_STAT(uartx)      ((uartx)+GD32_USART_STAT_OFFSET)      /* UART status register */
#define GD32_UART_INTC(uartx)      ((uartx)+GD32_USART_INTC_OFFSET)      /* UART interrupt status clear register */
#define GD32_UART_RDATA(uartx)     ((uartx)+GD32_USART_RDATA_OFFSET)     /* UART receive data register */
#define GD32_UART_TDATA(uartx)     ((uartx)+GD32_USART_TDATA_OFFSET)     /* UART transmit data register */
#define GD32_UART_CHC(uartx)       ((uartx)+GD32_USART_CHC_OFFSET)       /* UART coherence control register */
#define GD32_UART_FCS(uartx)       ((uartx)+GD32_USART_FCS_OFFSET)       /* UART FIFO control and status register */

/* Register Bitfield Definitions ********************************************/

/* Control register 0 (CTL0) */

#define USART_CTL0_UEN             (1 << 0)    /* Bit 0: USART enable */
#define USART_CTL0_UESM            (1 << 1)    /* Bit 1: USART enable in Deep-sleep mode */
#define USART_CTL0_REN             (1 << 2)    /* Bit 2: Receiver enable */
#define USART_CTL0_TEN             (1 << 3)    /* Bit 3: Transmitter enable */
#define USART_CTL0_IDLEIE          (1 << 4)    /* Bit 4: IDLE line detected interrupt enable */
#define USART_CTL0_RBNEIE          (1 << 5)    /* Bit 5: Read data buffer not empty interrupt enable */
#define USART_CTL0_RFNEIE          (1 << 5)    /* Bit 5: RXFIFO not empty interrupt enable */
#define USART_CTL0_TCIE            (1 << 6)    /* Bit 6: Transmission complete interrupt enable */
#define USART_CTL0_TBEIE           (1 << 7)    /* Bit 7: Transmitter buffer empty interrupt enable */
#define USART_CTL0_TFNFIE          (1 << 7)    /* Bit 7: TXFIFO not full interrupt enable */
#define USART_CTL0_PERRIE          (1 << 8)    /* Bit 8: Parity error interrupt enable */
#define USART_CTL0_PM              (1 << 9)    /* Bit 9: Parity mode */
#define USART_CTL0_PCEN            (1 << 10)   /* Bit 10: Parity check function enable */
#define USART_CTL0_WM              (1 << 11)   /* Bit 11: Wakeup method in mute mode */
#define USART_CTL0_WL0             (1 << 12)   /* Bit 12: Word length bit 0 */
#define USART_CTL0_MEN             (1 << 13)   /* Bit 13: Mute mode enable */
#define USART_CTL0_AMIE0           (1 << 14)   /* Bit 14: Address match interrupt enable in ADDR0 */
#define USART_CTL0_OVSMOD          (1 << 15)   /* Bit 15: Oversample mode */
#define USART_CTL0_DED_SHIFT       (16)        /* Bits 16-20: Driver Enable de-assertion time */
#define USART_CTL0_DED_MASK        (0x1f << USART_CTL0_DED_SHIFT)
#define USART_CTL0_DEA_SHIFT       (21)        /* Bits 21-25: Driver Enable assertion time */
#define USART_CTL0_DEA_MASK        (0x1f << USART_CTL0_DEA_SHIFT)
#define USART_CTL0_RTIE            (1 << 26)   /* Bit 26: Receiver timeout interrupt enable */
#define USART_CTL0_EBIE            (1 << 27)   /* Bit 27: End of block interrupt enable */
#define USART_CTL0_WL1             (1 << 28)   /* Bit 28: Word length bit 1 */
#define USART_CTL0_AMIE1           (1 << 31)   /* Bit 31: Address match interrupt enable in ADDR1 */

#define USART_CTL0_PM_SHIFT        (9)
#  define USART_CTL0_PM_MASK       (0x3 << USART_CTL0_PM_SHIFT)
#  define USART_CTL0_PMEN(n)       ((n) << USART_CTL0_PM_SHIFT)
#  define USART_CTL0_PM_NONE       USART_CTL0_PMEN(0)
#  define USART_CTL0_PM_EVEN       USART_CTL0_PMEN(2)
#  define USART_CTL0_PM_ODD        USART_CTL0_PMEN(3)

#define USART_WL_8BIT              (0)
#define USART_WL_9BIT              USART_CTL0_WL0
#define USART_WL_7BIT              USART_CTL0_WL1

#define USART_CTL0_INT_SHIFT       (4)
#define USART_CTL0_INT_MASK        (0x1f << USART_CTL0_INT_SHIFT)

/* Control register 1 (CTL1) */

#define USART_CTL1_AMEN0           (1 << 0)    /* Bit 0: Address match mode enable in ADDR0 */
#define USART_CTL1_ADDM0           (1 << 4)    /* Bit 4: Address detection mode in ADDR0 */
#define USART_CTL1_LBLEN           (1 << 5)    /* Bit 5: LIN break frame length */
#define USART_CTL1_LBDIE           (1 << 6)    /* Bit 6: LIN break detected interrupt enable */
#define USART_CTL1_CLEN            (1 << 8)    /* Bit 8: CK length */
#define USART_CTL1_CPH             (1 << 9)    /* Bit 9: CK phase */
#define USART_CTL1_CPL             (1 << 10)   /* Bit 10: CK polarity */
#define USART_CTL1_CKEN            (1 << 11)   /* Bit 11: CK pin enable */
#define USART_CTL1_STB_SHIFT       (12)        /* Bits 12-13: STOP bits length */
#define USART_CTL1_STB_MASK        (3 << USART_CTL1_STB_SHIFT)
#  define USART_CTL1_STB(n)        (n << USART_CTL1_STB_SHIFT)
#  define USART_CTL1_STB1BIT       USART_CTL1_STB(0) /* 00: 1 bit */
#  define USART_CTL1_STB0_5BIT     USART_CTL1_STB(1) /* 01: 0.5 bit */
#  define USART_CTL1_STB2BIT       USART_CTL1_STB(2) /* 10: 2 bits */
#  define USART_CTL1_STB1_5BIT     USART_CTL1_STB(3) /* 11: 1.5 bit */
#define USART_CTL1_LMEN            (1 << 14)         /* Bit 14: LIN mode enable */
#define USART_CTL1_STRP            (1 << 15)         /* Bit 15: Swap TX/RX pins */
#define USART_CTL1_RINV            (1 << 16)         /* Bit 16: RX pin level inversion */
#define USART_CTL1_TINV            (1 << 17)         /* Bit 17: TX pin level inversion */
#define USART_CTL1_DINV            (1 << 18)         /* Bit 18: Data bit level inversion */
#define USART_CTL1_MSBF            (1 << 19)         /* Bit 19: Most significant bit first */
#define USART_CTL1_RTEN            (1 << 23)         /* Bit 23: Receiver timeout enable */
#define USART_CTL1_ADDR0_SHIFT     (24)              /* Bits 24-31: Address of USART in ADDR0 */
#define USART_CTL1_ADDR0_MASK      (0xff << USART_CTL1_ADDR0_SHIFT)
#define USART_CTL1_ADDR0(n)        ((n) << USART_CTL1_ADDR0_SHIFT)

#define USART_CTL1_INT_MASK        USART_CTL1_LBDIE

/* Control register 2 (CTL2) */

#define USART_CTL2_ERRIE           (1 << 0)    /* Bit 0: Error interrupt enable */
#define USART_CTL2_IREN            (1 << 1)    /* Bit 1: IrDA mode enable */
#define USART_CTL2_IRLP            (1 << 2)    /* Bit 2: IrDA low-power */
#define USART_CTL2_HDEN            (1 << 3)    /* Bit 3: Half-duplex enable */
#define USART_CTL2_NKEN            (1 << 4)    /* Bit 4: NACK enable in smartcard mode */
#define USART_CTL2_SCEN            (1 << 5)    /* Bit 5: Smartcard mode enable */
#define USART_CTL2_DENR            (1 << 6)    /* Bit 6: DMA request enable for reception */
#define USART_CTL2_DENT            (1 << 7)    /* Bit 7: DMA request enable for transmission */
#define USART_CTL2_RTSEN           (1 << 8)    /* Bit 8: RTS enable */
#define USART_CTL2_CTSEN           (1 << 9)    /* Bit 9: CTS enable */
#define USART_CTL2_CTSIE           (1 << 10)   /* Bit 10: CTS interrupt enable */
#define USART_CTL2_OSB             (1 << 11)   /* Bit 11: One sample bit method */
#define USART_CTL2_OVRD            (1 << 12)   /* Bit 12: Overrun disable */
#define USART_CTL2_DDRE            (1 << 13)   /* Bit 13: DMA disable on reception error */
#define USART_CTL2_DEM             (1 << 14)   /* Bit 14: Driver enable mode */
#define USART_CTL2_DEP             (1 << 15)   /* Bit 15: Driver enable polarity */
#define USART_CTL2_AMEN1           (1 << 16)   /* Bit 16: Address match mode enable in ADDR1 */
#define USART_CTL2_SCRTNUM_SHIFT   (17)        /* Bits 17-19: Smartcard auto-retry number */
#define USART_CTL2_SCRTNUM_MASK    (0x07 << USART_CTL2_SCRTNUM_SHIFT)
#  define USART_CTL2_SCRTNUM(n)    ((n) << USART_CTL2_SCRTNUM_SHIFT)
#define USART_CTL2_WUM_SHIFT       (20)        /* Bits 20-21: Wakeup mode from Deep-sleep mode */
#define USART_CTL2_WUM_MASK        (0x03 << USART_CTL2_WUM_SHIFT)
#  define USART_CTL2_WUM(n)        ((n) << USART_CTL2_WUM_SHIFT)
#define USART_CTL2_WUIE            (1 << 22)   /* Bit 22: Wakeup from Deep-sleep mode interrupt enable */
#define USART_CTL2_ADDM1           (1 << 23)   /* Bit 23: Address detection mode in ADDR1 */
#define USART_CTL2_ADDR1_SHIFT     (24)        /* Bits 24-31: Address of USART in ADDR1 */
#define USART_CTL2_ADDR1_MASK      (0xff << USART_CTL2_ADDR1_SHIFT)
#define USART_CTL2_ADDR1(n)        ((n) << USART_CTL2_ADDR1_SHIFT)

#define USART_CTL2_INT_MASK        (USART_CTL2_ERRIE | USART_CTL2_CTSIE | USART_CTL2_WUIE)

/* Baud Rate Register (BAUD) */

#define USART_BAUD_FRADIV_SHIFT    (0)         /* Bits 0-3: Fraction part of baud-rate divider */
#define USART_BAUD_FRADIV_MASK     (0x0f << USART_BAUD_FRADIV_SHIFT)
#  define USART_BAUD_FRADIV(n)     ((n) << USART_BAUD_FRADIV_SHIFT)
#define USART_BAUD_INTDIV_SHIFT    (4)         /* Bits 4-15: Integer part of baud-rate divider */
#define USART_BAUD_INTDIV_MASK     (0xfff << USART_BAUD_INTDIV_SHIFT)
#  define USART_BAUD_INTDIV(n)     ((n) << USART_BAUD_INTDIV_SHIFT)

/* Guard time and prescaler register (GP) */

#define USART_GP_PSC_SHIFT         (0)         /* Bits 0-7: Prescaler value for dividing the system clock */
#define USART_GP_PSC_MASK          (0xff << USART_GP_PSC_SHIFT)
#  define USART_GP_PSC(n)          ((n) << USART_GP_PSC_SHIFT)
#define USART_GP_GUAT_SHIFT        (8)         /* Bits 8-15: Guard time value in smartcard mode */
#define USART_GP_GUAT_MASK         (0xff << USART_GP_GUAT_SHIFT)
#  define USART_GP_GUAT(n)         ((n) << USART_GP_GUAT_SHIFT)

/* Receiver timeout register (RT) */

#define USART_RT_RT_SHIFT          (0)        /* Bits 0-23: Receiver timeout threshold */
#define USART_RT_RT_MASK           (0xffffff << USART_RT_RT_SHIFT)
#  define USART_RT_RT(n)           ((n) << USART_RT_RT_SHIFT)
#define USART_RT_BL_SHIFT          (24)       /* Bits 24-31: Block length */
#define USART_RT_BL_MASK           (0xff << USART_RT_BL_SHIFT)
#  define USART_RT_BL(n)           ((n) << USART_RT_BL_SHIFT)

/* Command register (CMD) */

#define USART_CMD_SBKCMD           (1 << 1)    /* Bit 1: Send break command */
#define USART_CMD_MMCMD            (1 << 2)    /* Bit 2: Mute mode command */
#define USART_CMD_RXFCMD           (1 << 3)    /* Bit 3: Receive data flush command */
#define USART_CMD_TXFCMD           (1 << 4)    /* Bit 4: Transmit data flush command */

/* Status register (STAT) */

#define USART_STAT_PERR            (1 << 0)    /* Bit 0: Parity error flag */
#define USART_STAT_FERR            (1 << 1)    /* Bit 1: Frame error flag */
#define USART_STAT_NERR            (1 << 2)    /* Bit 2: Noise error flag */
#define USART_STAT_ORERR           (1 << 3)    /* Bit 3: Overrun error */
#define USART_STAT_IDLEF           (1 << 4)    /* Bit 4: IDLE frame detected flag */
#define USART_STAT_RBNE            (1 << 5)    /* Bit 5: Read data buffer not empty */
#define USART_STAT_RFNE            (1 << 5)    /* Bit 5: RXFIFO not empty */
#define USART_STAT_TC              (1 << 6)    /* Bit 6: Transmission complete */
#define USART_STAT_TBE             (1 << 7)    /* Bit 7: Transmit data buffer empty */
#define USART_STAT_TFNF            (1 << 7)    /* Bit 7: TXFIFO not full */
#define USART_STAT_LBDF            (1 << 8)    /* Bit 8: LIN break detected flag */
#define USART_STAT_CTSF            (1 << 9)    /* Bit 9: CTS change flag */
#define USART_STAT_CTS             (1 << 10)   /* Bit 10: CTS level */
#define USART_STAT_RTF             (1 << 11)   /* Bit 11: Receiver timeout flag */
#define USART_STAT_EBF             (1 << 12)   /* Bit 12: End of block flag */
#define USART_STAT_AMF1            (1 << 13)   /* Bit 13: Address match flag in ADDR1 */
#define USART_STAT_BSY             (1 << 16)   /* Bit 16: Busy flag */
#define USART_STAT_AMF0            (1 << 17)   /* Bit 17: Address match flag in ADDR0 */
#define USART_STAT_SBF             (1 << 18)   /* Bit 18: Send break flag */
#define USART_STAT_RWU             (1 << 19)   /* Bit 19: Receiver wakeup from mute mode */
#define USART_STAT_WUF             (1 << 20)   /* Bit 20: Wakeup from Deep-sleep mode flag */
#define USART_STAT_TEA             (1 << 21)   /* Bit 21: Transmit enable acknowledge flag */
#define USART_STAT_REA             (1 << 22)   /* Bit 22: Receive enable acknowledge flag */

/* Interrupt status clear register (INTC) */

#define USART_INTC_PEC             (1 << 0)    /* Bit 0: Parity error clear */
#define USART_INTC_FEC             (1 << 1)    /* Bit 1: Frame error clear */
#define USART_INTC_NEC             (1 << 2)    /* Bit 2: Noise error clear */
#define USART_INTC_OREC            (1 << 3)    /* Bit 3: Overrun error clear */
#define USART_INTC_IDLEC           (1 << 4)    /* Bit 4: IDLE frame detected clear */
#define USART_INTC_TCC             (1 << 6)    /* Bit 6: Transmission complete clear */
#define USART_INTC_LBDC            (1 << 8)    /* Bit 8: LIN break detected clear */
#define USART_INTC_CTSC            (1 << 9)    /* Bit 9: CTS change clear */
#define USART_INTC_RTC             (1 << 11)   /* Bit 11: Receiver timeout clear */
#define USART_INTC_EBC             (1 << 12)   /* Bit 12: End of block clear */
#define USART_INTC_AMC1            (1 << 16)   /* Bit 16: Address match clear in ADDR1 */
#define USART_INTC_AMC0            (1 << 17)   /* Bit 17: Address match clear in ADDR0 */
#define USART_INTC_WUC             (1 << 20)   /* Bit 20: Wakeup from Deep-sleep mode clear */

/* Receive data register (RDATA) */

#define USART_RDATA_RDATA_SHIFT    (0)         /* Bits 0-9: Receive data value */
#define USART_RDATA_RDATA_MASK     (0x3ff << USART_RDATA_RDATA_SHIFT)

/* Transmit data register (TDATA) */

#define USART_TDATA_TDATA_SHIFT    (0)         /* Bits 0-9: Transmit data value */
#define USART_TDATA_TDATA_MASK     (0x3ff << USART_TDATA_TDATA_SHIFT)

#define USART_DATA_MASK            (0x3ff << 0)

/* Coherence control register (CHC) */

#define USART_CHC_HCM              (1 << 0)    /* Bit 0: Hardware flow control coherence mode */
#define USART_CHC_EPERR            (1 << 8)    /* Bit 8: Early parity error flag */

/* FIFO control and status register (FCS) */

#define USART_FCS_ELNACK           (1 << 0)    /* Bit 0: Early NACK */
#define USART_FCS_RFCNT3_4_SHIFT   (1)         /* Bits 1-2: Receive FIFO counter bits 3-4 */
#define USART_FCS_RFCNT3_4_MASK    (0x03 << USART_FCS_RFCNT3_4_SHIFT)
#define USART_FCS_RFT              (1 << 4)    /* Bit 4: Receive FIFO threshold flag */
#define USART_FCS_TFT              (1 << 5)    /* Bit 5: Transmit FIFO threshold flag */
#define USART_FCS_TFE              (1 << 6)    /* Bit 6: Transmit FIFO empty flag */
#define USART_FCS_TFF              (1 << 7)    /* Bit 7: Transmit FIFO full flag */
#define USART_FCS_FEN              (1 << 8)    /* Bit 8: FIFO enable */
#define USART_FCS_RFFIE            (1 << 9)    /* Bit 9: Receive FIFO full interrupt enable */
#define USART_FCS_RFE              (1 << 10)   /* Bit 10: Receive FIFO empty flag */
#define USART_FCS_RFF              (1 << 11)   /* Bit 11: Receive FIFO full flag */
#define USART_FCS_RFCNT0_2_SHIFT   (12)        /* Bits 12-14: Receive FIFO counter bits 0-2 */
#define USART_FCS_RFCNT0_2_MASK    (0x07 << USART_FCS_RFCNT0_2_SHIFT)
#define USART_FCS_RFFIF            (1 << 15)   /* Bit 15: Receive FIFO full interrupt flag */
#define USART_FCS_RFTCFG_SHIFT     (16)        /* Bits 16-18: Receive FIFO threshold configuration */
#define USART_FCS_RFTCFG_MASK      (0x07 << USART_FCS_RFTCFG_SHIFT)
#  define USART_FCS_RFTCFG(n)      ((n) << USART_FCS_RFTCFG_SHIFT)
#define USART_FCS_TFTCFG_SHIFT     (19)        /* Bits 19-21: Transmit FIFO threshold configuration */
#define USART_FCS_TFTCFG_MASK      (0x07 << USART_FCS_TFTCFG_SHIFT)
#  define USART_FCS_TFTCFG(n)      ((n) << USART_FCS_TFTCFG_SHIFT)
#define USART_FCS_RFTIF            (1 << 22)   /* Bit 22: Receive FIFO threshold interrupt flag */
#define USART_FCS_TFEIF            (1 << 24)   /* Bit 24: Transmit FIFO empty interrupt flag */
#define USART_FCS_TFTIF            (1 << 25)   /* Bit 25: Transmit FIFO threshold interrupt flag */
#define USART_FCS_TFEC             (1 << 26)   /* Bit 26: Transmit FIFO empty interrupt flag clear */
#define USART_FCS_RFTIE            (1 << 27)   /* Bit 27: Receive FIFO threshold interrupt enable */
#define USART_FCS_TFTIE            (1 << 29)   /* Bit 29: Transmit FIFO threshold interrupt enable */
#define USART_FCS_TFEIE            (1 << 31)   /* Bit 31: Transmit FIFO empty interrupt enable */

#endif /* __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_USART_H */
