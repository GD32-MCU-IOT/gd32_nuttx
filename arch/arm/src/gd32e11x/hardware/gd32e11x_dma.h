/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_dma.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DMA_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA definitions */
#define GD32_DMA0_BASE               (GD32_DMA_BASE+0x00000000)          /* DMA0 base address */
#define GD32_DMA1_BASE               (GD32_DMA_BASE+0x00000400)          /* DMA1 base address */

/* Register Offsets *********************************************************/

#define GD32_DMA_INTF_OFFSET           0x0000            /* DMA interrupt flag register offset */
#define GD32_DMA_INTC_OFFSET           0x0004            /* DMA interrupt flag clear register offset */

#define GD32_DMA_CH0CTL_OFFSET         0x0008            /* DMA channel 0 control register offset */
#define GD32_DMA_CH0CNT_OFFSET         0x000C            /* DMA channel 0 counter register offset */
#define GD32_DMA_CH0PADDR_OFFSET       0x0010            /* DMA channel 0 peripheral base address register offset */
#define GD32_DMA_CH0MADDR_OFFSET       0x0014            /* DMA channel 0 memory base address register offset */

#define GD32_DMA_CH1CTL_OFFSET         0x001C            /* DMA channel 1 control register offset */
#define GD32_DMA_CH1CNT_OFFSET         0x0020            /* DMA channel 1 counter register offset */
#define GD32_DMA_CH1PADDR_OFFSET       0x0024            /* DMA channel 1 peripheral base address register offset */
#define GD32_DMA_CH1MADDR_OFFSET       0x0028            /* DMA channel 1 memory base address register offset */

#define GD32_DMA_CH2CTL_OFFSET         0x0030            /* DMA channel 2 control register offset */
#define GD32_DMA_CH2CNT_OFFSET         0x0034            /* DMA channel 2 counter register offset */
#define GD32_DMA_CH2PADDR_OFFSET       0x0038            /* DMA channel 2 peripheral base address register offset */
#define GD32_DMA_CH2MADDR_OFFSET       0x003C            /* DMA channel 2 memory base address register offset */

#define GD32_DMA_CH3CTL_OFFSET         0x0044            /* DMA channel 3 control register offset */
#define GD32_DMA_CH3CNT_OFFSET         0x0048            /* DMA channel 3 counter register offset */
#define GD32_DMA_CH3PADDR_OFFSET       0x004C            /* DMA channel 3 peripheral base address register offset */
#define GD32_DMA_CH3MADDR_OFFSET       0x0050            /* DMA channel 3 memory base address register offset */

#define GD32_DMA_CH4CTL_OFFSET         0x0058            /* DMA channel 4 control register offset */
#define GD32_DMA_CH4CNT_OFFSET         0x005C            /* DMA channel 4 counter register offset */
#define GD32_DMA_CH4PADDR_OFFSET       0x0060            /* DMA channel 4 peripheral base address register offset */
#define GD32_DMA_CH4MADDR_OFFSET       0x0064            /* DMA channel 4 memory base address register offset */

#define GD32_DMA_CH5CTL_OFFSET         0x006C            /* DMA channel 5 control register offset */
#define GD32_DMA_CH5CNT_OFFSET         0x0070            /* DMA channel 5 counter register offset */
#define GD32_DMA_CH5PADDR_OFFSET       0x0074            /* DMA channel 5 peripheral base address register offset */
#define GD32_DMA_CH5MADDR_OFFSET       0x0078            /* DMA channel 5 memory base address register offset */

#define GD32_DMA_CH6CTL_OFFSET         0x0080            /* DMA channel 6 control register offset */
#define GD32_DMA_CH6CNT_OFFSET         0x0084            /* DMA channel 6 counter register offset */
#define GD32_DMA_CH6PADDR_OFFSET       0x0088            /* DMA channel 6 peripheral base address register offset */
#define GD32_DMA_CH6MADDR_OFFSET       0x008C            /* DMA channel 6 memory base address register offset */

/* Register Addresses *******************************************************/

#define GD32_DMA_INTF(dmax)            ((dmax)+GD32_DMA_INTF_OFFSET)            /* DMA interrupt flag register */
#define GD32_DMA_INTC(dmax)            ((dmax)+GD32_DMA_INTC_OFFSET)            /* DMA interrupt flag clear register */

#define GD32_DMA_CH0CTL(dmax)          ((dmax)+GD32_DMA_CH0CTL_OFFSET)          /* DMA channel 0 control register */
#define GD32_DMA_CH0CNT(dmax)          ((dmax)+GD32_DMA_CH0CNT_OFFSET)          /* DMA channel 0 counter register */
#define GD32_DMA_CH0PADDR(dmax)        ((dmax)+GD32_DMA_CH0PADDR_OFFSET)        /* DMA channel 0 peripheral base address register */
#define GD32_DMA_CH0MADDR(dmax)        ((dmax)+GD32_DMA_CH0MADDR_OFFSET)        /* DMA channel 0 memory base address register */

#define GD32_DMA_CH1CTL(dmax)          ((dmax)+GD32_DMA_CH1CTL_OFFSET)          /* DMA channel 1 control register */
#define GD32_DMA_CH1CNT(dmax)          ((dmax)+GD32_DMA_CH1CNT_OFFSET)          /* DMA channel 1 counter register */
#define GD32_DMA_CH1PADDR(dmax)        ((dmax)+GD32_DMA_CH1PADDR_OFFSET)        /* DMA channel 1 peripheral base address register */
#define GD32_DMA_CH1MADDR(dmax)        ((dmax)+GD32_DMA_CH1MADDR_OFFSET)        /* DMA channel 1 memory base address register */

#define GD32_DMA_CH2CTL(dmax)          ((dmax)+GD32_DMA_CH2CTL_OFFSET)          /* DMA channel 2 control register */
#define GD32_DMA_CH2CNT(dmax)          ((dmax)+GD32_DMA_CH2CNT_OFFSET)          /* DMA channel 2 counter register */
#define GD32_DMA_CH2PADDR(dmax)        ((dmax)+GD32_DMA_CH2PADDR_OFFSET)        /* DMA channel 2 peripheral base address register */
#define GD32_DMA_CH2MADDR(dmax)        ((dmax)+GD32_DMA_CH2MADDR_OFFSET)        /* DMA channel 2 memory base address register */

#define GD32_DMA_CH3CTL(dmax)          ((dmax)+GD32_DMA_CH3CTL_OFFSET)          /* DMA channel 3 control register */
#define GD32_DMA_CH3CNT(dmax)          ((dmax)+GD32_DMA_CH3CNT_OFFSET)          /* DMA channel 3 counter register */
#define GD32_DMA_CH3PADDR(dmax)        ((dmax)+GD32_DMA_CH3PADDR_OFFSET)        /* DMA channel 3 peripheral base address register */
#define GD32_DMA_CH3MADDR(dmax)        ((dmax)+GD32_DMA_CH3MADDR_OFFSET)        /* DMA channel 3 memory base address register */

#define GD32_DMA_CH4CTL(dmax)          ((dmax)+GD32_DMA_CH4CTL_OFFSET)          /* DMA channel 4 control register */
#define GD32_DMA_CH4CNT(dmax)          ((dmax)+GD32_DMA_CH4CNT_OFFSET)          /* DMA channel 4 counter register */
#define GD32_DMA_CH4PADDR(dmax)        ((dmax)+GD32_DMA_CH4PADDR_OFFSET)        /* DMA channel 4 peripheral base address register */
#define GD32_DMA_CH4MADDR(dmax)        ((dmax)+GD32_DMA_CH4MADDR_OFFSET)        /* DMA channel 4 memory base address register */

#define GD32_DMA_CH5CTL(dmax)          ((dmax)+GD32_DMA_CH5CTL_OFFSET)          /* DMA channel 5 control register */
#define GD32_DMA_CH5CNT(dmax)          ((dmax)+GD32_DMA_CH5CNT_OFFSET)          /* DMA channel 5 counter register */
#define GD32_DMA_CH5PADDR(dmax)        ((dmax)+GD32_DMA_CH5PADDR_OFFSET)        /* DMA channel 5 peripheral base address register */
#define GD32_DMA_CH5MADDR(dmax)        ((dmax)+GD32_DMA_CH5MADDR_OFFSET)        /* DMA channel 5 memory base address register */

#define GD32_DMA_CH6CTL(dmax)          ((dmax)+GD32_DMA_CH6CTL_OFFSET)          /* DMA channel 6 control register */
#define GD32_DMA_CH6CNT(dmax)          ((dmax)+GD32_DMA_CH6CNT_OFFSET)          /* DMA channel 6 counter register */
#define GD32_DMA_CH6PADDR(dmax)        ((dmax)+GD32_DMA_CH6PADDR_OFFSET)        /* DMA channel 6 peripheral base address register */
#define GD32_DMA_CH6MADDR(dmax)        ((dmax)+GD32_DMA_CH6MADDR_OFFSET)        /* DMA channel 6 memory base address register */

/* DMA channelx register address */
#define GD32_DMA_CHCTL(dma, channelx)            ((dma+0x08) + 0x14*(channelx))  /* The address of DMA channel CHXCTL register  */
#define GD32_DMA_CHCNT(dma, channelx)            ((dma+0x0C) + 0x14*(channelx))  /* The address of DMA channel CHXCNT register */
#define GD32_DMA_CHPADDR(dma, channelx)          ((dma+0x10) + 0x14*(channelx))  /* The address of DMA channel CHXPADDR register */
#define GD32_DMA_CHMADDR(dma, channelx)          ((dma+0x14) + 0x14*(channelx))  /* The address of DMA channel CHXMADDR register */

/* bits definitions */

/* DMA_INTF */
#define DMA_INTF_GIF                   (1 << 0)                        /* Bit 0: global interrupt flag of channel */
#define DMA_INTF_FTFIF                 (1 << 1)                        /* Bit 1: full transfer finish flag of channel */
#define DMA_INTF_HTFIF                 (1 << 2)                        /* Bit 2: half transfer finish flag of channel */
#define DMA_INTF_ERRIF                 (1 << 3)                        /* Bit 3: error flag of channel */

/* DMA_INTC */
#define DMA_INTC_GIFC                  (1 << 0)                        /* Bit 0: clear global interrupt flag of channel */
#define DMA_INTC_FTFIFC                (1 << 1)                        /* Bit 1: clear transfer finish flag of channel */
#define DMA_INTC_HTFIFC                (1 << 2)                        /* Bit 2: clear half transfer finish flag of channel */
#define DMA_INTC_ERRIFC                (1 << 3)                        /* Bit 3: clear error flag of channel */

/* DMA_CHxCTL, x=0..6 */
#define DMA_CHXCTL_CHEN                (1 << 0)                        /* Bit 0: channel enable */
#define DMA_CHXCTL_FTFIE               (1 << 1)                        /* Bit 1: enable bit for channel full transfer finish interrupt */
#define DMA_CHXCTL_HTFIE               (1 << 2)                        /* Bit 2: enable bit for channel half transfer finish interrupt */
#define DMA_CHXCTL_ERRIE               (1 << 3)                        /* Bit 3: enable bit for channel error interrupt */
#define DMA_CHXCTL_DIR                 (1 << 4)                        /* Bit 4: transfer direction */
#define DMA_CHXCTL_CMEN                (1 << 5)                        /* Bit 5: circular mode enable */
#define DMA_CHXCTL_PNAGA               (1 << 6)                        /* Bit 6: next address generation algorithm of peripheral */
#define DMA_CHXCTL_MNAGA               (1 << 7)                        /* Bit 7: next address generation algorithm of memory */

#define DMA_CHXCTL_PWIDTH_SHIFT        (8)                             /* Bit 8-9: transfer data width of peripheral */
#define DMA_CHXCTL_PWIDTH_MASK         (3 << DMA_CHXCTL_PWIDTH_SHIFT)
#define DMA_CHXCTL_PWIDTH(n)           ((n) << DMA_CHXCTL_PWIDTH_SHIFT)
#  define DMA_PERIPH_WIDTH_8BIT        DMA_CHXCTL_PWIDTH(0)            /* 00: transfer data width of peripheral is 8-bit */
#  define DMA_PERIPH_WIDTH_16BIT       DMA_CHXCTL_PWIDTH(1)            /* 01: transfer data width of peripheral is 16-bit */
#  define DMA_PERIPH_WIDTH_32BIT       DMA_CHXCTL_PWIDTH(2)            /* 10: transfer data width of peripheral is 32-bit */

#define DMA_CHXCTL_MWIDTH_SHIFT        (10)                            /* Bit 10-11: transfer data width of memory */
#define DMA_CHXCTL_MWIDTH_MASK         (3 << DMA_CHXCTL_MWIDTH_SHIFT)
#define DMA_CHXCTL_MWIDTH(n)           ((n) << DMA_CHXCTL_MWIDTH_SHIFT)
#  define DMA_MEMORY_WIDTH_8BIT        DMA_CHXCTL_MWIDTH(0)            /* 00: transfer data width of memory is 8-bit */
#  define DMA_MEMORY_WIDTH_16BIT       DMA_CHXCTL_MWIDTH(1)            /* 01: transfer data width of memory is 16-bit */
#  define DMA_MEMORY_WIDTH_32BIT       DMA_CHXCTL_MWIDTH(2)            /* 10: transfer data width of memory is 32-bit */

#define DMA_CHXCTL_PRIO_SHIFT          (12)                            /* Bit 12-13: priority level */
#define DMA_CHXCTL_PRIO_MASK           (3 << DMA_CHXCTL_PRIO_SHIFT)
#define DMA_CHXCTL_PRIO(n)             ((n) << DMA_CHXCTL_PRIO_SHIFT)
#  define DMA_PRIORITY_LOW             DMA_CHXCTL_PRIO(0)              /* 00: low priority */
#  define DMA_PRIORITY_MEDIUM          DMA_CHXCTL_PRIO(1)              /* 01: medium priority */
#  define DMA_PRIORITY_HIGH            DMA_CHXCTL_PRIO(2)              /* 10: high priority */
#  define DMA_PRIORITY_ULTRA_HIGH      DMA_CHXCTL_PRIO(3)              /* 11: ultra high priority */

#define DMA_CHXCTL_M2M                 (1 << 14)                       /* Bit 14: memory to memory mode */

/* DMA_CHxCNT, x=0..6 */
#define DMA_CHXCNT_CNT_MASK            (0xffff << 0)                   /* Bit 0-15: transfer counter */

/* DMA_CHxPADDR,x=0..6 */
#define DMA_CHXPADDR_PADDR_MASK        (0xffffffff << 0)               /* Bit 0-31: peripheral base address */

/* DMA_CHxMADDR,x=0..6 */
#define DMA_CHXMADDR_MADDR_MASK        (0xffffffff << 0)               /* Bit 0-31: memory base address */

/* DMA flag shift calculation */
#define GD32_DMA_FLAG_ADD(flag, channel)       ((uint32_t)((flag) << ((((uint32_t)(channel)) * 4U))))   /* DMA channel flag shift */

/* peripheral increasing mode */
#define DMA_PERIPH_INCREASE_ENABLE       (0x00000000)          /* Next address of peripheral is increasing address mode */
#define DMA_PERIPH_INCREASE_DISABLE      (0x00000001)          /* Next address of peripheral is fixed address mode */
#define DMA_PERIPH_INCREASE_FIX          (0x00000002)          /* Next address of peripheral is increasing fixed */

/* memory increasing mode */
#define DMA_MEMORY_INCREASE_ENABLE       (0x00000000)          /* Next address of memory is increasing address mode */
#define DMA_MEMORY_INCREASE_DISABLE      (0x00000001)          /* Next address of memory is fixed address mode */

/* DMA circular mode */
#define DMA_CIRCULAR_MODE_ENABLE         (0x00000000)          /* Circular mode enable */
#define DMA_CIRCULAR_MODE_DISABLE        (0x00000001)          /* Circular mode disable */

/* DMA width selection */
#define DMA_WIDTH_8BITS_SELECT           (0x00000000)          /* Select 8 bits width */
#define DMA_WIDTH_16BITS_SELECT          (0x00000001)          /* Select 16 bits width */
#define DMA_WIDTH_32BITS_SELECT          (0x00000002)          /* Select 16 bits width */

/* DMA priority level selection */
#define DMA_PRIO_LOW_SELECT              (0x00000000)          /* Select low priority level */
#define DMA_PRIO_MEDIUM_SELECT           (0x00000001)          /* Select medium priority level */
#define DMA_PRIO_HIGH_SELECT             (0x00000002)          /* Select high priority level */
#define DMA_PRIO_ULTRA_HIGHSELECT        (0x00000003)          /* Select ultra high priority level */

/* DMA channel select - GD32E11X has 12 channels total */

/* DMA0: 7 channels (CH0-CH6), DMA1: 5 channels (CH0-CH4) */

#define GD32_DMA_CH0                 (0)        /* DMA Channel 0 */
#define GD32_DMA_CH1                 (1)        /* DMA Channel 1 */
#define GD32_DMA_CH2                 (2)        /* DMA Channel 2 */
#define GD32_DMA_CH3                 (3)        /* DMA Channel 3 */
#define GD32_DMA_CH4                 (4)        /* DMA Channel 4 */
#define GD32_DMA_CH5                 (5)        /* DMA Channel 5 */
#define GD32_DMA_CH6                 (6)        /* DMA Channel 6 */

/* DMA channel mapping helpers */

#define DMAMAP_DMA0_MASK               0x000000FF
#define DMAMAP_DMA0_SHIFT              (0)
#define DMAMAP_DMA1_MASK               0x0000FF00
#define DMAMAP_DMA1_SHIFT              (8)

#define DMAMAP_MAP(d,c)                ( (((d) & 1) << 8*((d) & 1)) | (((c) & 0x0F) << 8*((d) & 1)) )

/* Peripheral with DMA to channel mapping based on GD32E11X User Manual */

/* DMA0 Channel 0 */

#define DMA_REQ_TIMER1_CH2              DMAMAP_MAP(0, GD32_DMA_CH0)            /* TIMER1_CH2 */
#define DMA_REQ_TIMER3_CH0              DMAMAP_MAP(0, GD32_DMA_CH0)            /* TIMER3_CH0 */
#define DMA_REQ_ADC0                    DMAMAP_MAP(0, GD32_DMA_CH0)            /* ADC0 */

/* DMA0 Channel 1 */

#define DMA_REQ_TIMER0_CH0              DMAMAP_MAP(0, GD32_DMA_CH1)            /* TIMER0_CH0 */
#define DMA_REQ_TIMER1_UP               DMAMAP_MAP(0, GD32_DMA_CH1)            /* TIMER1_UP */
#define DMA_REQ_TIMER2_CH2              DMAMAP_MAP(0, GD32_DMA_CH1)            /* TIMER2_CH2 */
#define DMA_REQ_SPI0_RX                 DMAMAP_MAP(0, GD32_DMA_CH1)            /* SPI0_RX */
#define DMA_REQ_USART2_TX               DMAMAP_MAP(0, GD32_DMA_CH1)            /* USART2_TX */

/* DMA0 Channel 2 */

#define DMA_REQ_TIMER0_CH1              DMAMAP_MAP(0, GD32_DMA_CH2)            /* TIMER0_CH1 */
#define DMA_REQ_TIMER2_CH3              DMAMAP_MAP(0, GD32_DMA_CH2)            /* TIMER2_CH3 */
#define DMA_REQ_TIMER2_UP               DMAMAP_MAP(0, GD32_DMA_CH2)            /* TIMER2_UP */
#define DMA_REQ_SPI0_TX                 DMAMAP_MAP(0, GD32_DMA_CH2)            /* SPI0_TX */
#define DMA_REQ_USART2_RX               DMAMAP_MAP(0, GD32_DMA_CH2)            /* USART2_RX */

/* DMA0 Channel 3 */

#define DMA_REQ_TIMER0_CH3              DMAMAP_MAP(0, GD32_DMA_CH3)            /* TIMER0_CH3 */
#define DMA_REQ_TIMER0_TG               DMAMAP_MAP(0, GD32_DMA_CH3)            /* TIMER0_TG */
#define DMA_REQ_TIMER0_CMT              DMAMAP_MAP(0, GD32_DMA_CH3)            /* TIMER0_CMT */
#define DMA_REQ_TIMER3_CH1              DMAMAP_MAP(0, GD32_DMA_CH3)            /* TIMER3_CH1 */
#define DMA_REQ_SPI1_RX                 DMAMAP_MAP(0, GD32_DMA_CH3)            /* SPI1_RX */
#define DMA_REQ_I2S1_RX                 DMAMAP_MAP(0, GD32_DMA_CH3)            /* I2S1_RX */
#define DMA_REQ_USART0_TX               DMAMAP_MAP(0, GD32_DMA_CH3)            /* USART0_TX */
#define DMA_REQ_I2C1_TX                 DMAMAP_MAP(0, GD32_DMA_CH3)            /* I2C1_TX */

/* DMA0 Channel 4 */

#define DMA_REQ_TIMER0_UP               DMAMAP_MAP(0, GD32_DMA_CH4)            /* TIMER0_UP */
#define DMA_REQ_TIMER1_CH0              DMAMAP_MAP(0, GD32_DMA_CH4)            /* TIMER1_CH0 */
#define DMA_REQ_SPI1_TX                 DMAMAP_MAP(0, GD32_DMA_CH4)            /* SPI1_TX */
#define DMA_REQ_I2S1_TX                 DMAMAP_MAP(0, GD32_DMA_CH4)            /* I2S1_TX */
#define DMA_REQ_USART0_RX               DMAMAP_MAP(0, GD32_DMA_CH4)            /* USART0_RX */
#define DMA_REQ_I2C1_RX                 DMAMAP_MAP(0, GD32_DMA_CH4)            /* I2C1_RX */

/* DMA0 Channel 5 */

#define DMA_REQ_TIMER0_CH2              DMAMAP_MAP(0, GD32_DMA_CH5)            /* TIMER0_CH2 */
#define DMA_REQ_TIMER2_CH0              DMAMAP_MAP(0, GD32_DMA_CH5)            /* TIMER2_CH0 */
#define DMA_REQ_TIMER2_TG               DMAMAP_MAP(0, GD32_DMA_CH5)            /* TIMER2_TG */
#define DMA_REQ_USART1_RX               DMAMAP_MAP(0, GD32_DMA_CH5)            /* USART1_RX */
#define DMA_REQ_I2C0_TX                 DMAMAP_MAP(0, GD32_DMA_CH5)            /* I2C0_TX */

/* DMA0 Channel 6 */

#define DMA_REQ_TIMER1_CH1              DMAMAP_MAP(0, GD32_DMA_CH6)            /* TIMER1_CH1 */
#define DMA_REQ_TIMER1_CH3              DMAMAP_MAP(0, GD32_DMA_CH6)            /* TIMER1_CH3 */
#define DMA_REQ_TIMER3_UP               DMAMAP_MAP(0, GD32_DMA_CH6)            /* TIMER3_UP */
#define DMA_REQ_USART1_TX               DMAMAP_MAP(0, GD32_DMA_CH6)            /* USART1_TX */
#define DMA_REQ_I2C0_RX                 DMAMAP_MAP(0, GD32_DMA_CH6)            /* I2C0_RX */

/* DMA1 Channel 0 */

#define DMA_REQ_TIMER4_CH3              DMAMAP_MAP(1, GD32_DMA_CH0)            /* TIMER4_CH3 */
#define DMA_REQ_TIMER4_TG               DMAMAP_MAP(1, GD32_DMA_CH0)            /* TIMER4_TG */
#define DMA_REQ_TIMER7_CH2              DMAMAP_MAP(1, GD32_DMA_CH0)            /* TIMER7_CH2 */
#define DMA_REQ_TIMER7_UP               DMAMAP_MAP(1, GD32_DMA_CH0)            /* TIMER7_UP */
#define DMA_REQ_SPI2_RX                 DMAMAP_MAP(1, GD32_DMA_CH0)            /* SPI2_RX */
#define DMA_REQ_I2S2_RX                 DMAMAP_MAP(1, GD32_DMA_CH0)            /* I2S2_RX */

/* DMA1 Channel 1 */

#define DMA_REQ_TIMER4_CH2              DMAMAP_MAP(1, GD32_DMA_CH1)            /* TIMER4_CH2 */
#define DMA_REQ_TIMER4_UP               DMAMAP_MAP(1, GD32_DMA_CH1)            /* TIMER4_UP */
#define DMA_REQ_TIMER7_CH3              DMAMAP_MAP(1, GD32_DMA_CH1)            /* TIMER7_CH3 */
#define DMA_REQ_TIMER7_TG               DMAMAP_MAP(1, GD32_DMA_CH1)            /* TIMER7_TG */
#define DMA_REQ_TIMER7_CMT              DMAMAP_MAP(1, GD32_DMA_CH1)            /* TIMER7_CMT */
#define DMA_REQ_SPI2_TX                 DMAMAP_MAP(1, GD32_DMA_CH1)            /* SPI2_TX */
#define DMA_REQ_I2S2_TX                 DMAMAP_MAP(1, GD32_DMA_CH1)            /* I2S2_TX */

/* DMA1 Channel 2 */

#define DMA_REQ_TIMER5_UP               DMAMAP_MAP(1, GD32_DMA_CH2)            /* TIMER5_UP */
#define DMA_REQ_TIMER7_CH0              DMAMAP_MAP(1, GD32_DMA_CH2)            /* TIMER7_CH0 */
#define DMA_REQ_DAC_CH0                 DMAMAP_MAP(1, GD32_DMA_CH2)            /* DAC_CH0 */
#define DMA_REQ_UART3_RX                DMAMAP_MAP(1, GD32_DMA_CH2)            /* UART3_RX */

/* DMA1 Channel 3 */

#define DMA_REQ_TIMER4_CH1              DMAMAP_MAP(1, GD32_DMA_CH3)            /* TIMER4_CH1 */
#define DMA_REQ_TIMER6_UP               DMAMAP_MAP(1, GD32_DMA_CH3)            /* TIMER6_UP */
#define DMA_REQ_DAC_CH1                 DMAMAP_MAP(1, GD32_DMA_CH3)            /* DAC_CH1 */

/* DMA1 Channel 4 */

#define DMA_REQ_TIMER4_CH0              DMAMAP_MAP(1, GD32_DMA_CH4)            /* TIMER4_CH0 */
#define DMA_REQ_TIMER7_CH1              DMAMAP_MAP(1, GD32_DMA_CH4)            /* TIMER7_CH1 */
#define DMA_REQ_UART3_TX                DMAMAP_MAP(1, GD32_DMA_CH4)            /* UART3_TX */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DMA_H */
