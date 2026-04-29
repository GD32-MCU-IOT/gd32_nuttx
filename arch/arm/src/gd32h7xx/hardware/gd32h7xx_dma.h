/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h7xx_dma.h
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

#ifndef __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_DMA_H
#define __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA / DMAMUX base addresses */

#define GD32_DMA0_BASE              (GD32_DMA_BASE + 0x00000000)
#define GD32_DMA1_BASE              (GD32_DMA_BASE + 0x00000400)
#define GD32_DMAMUX0_BASE           (GD32_DMAMUX_BASE)

/* DMA register offsets */

#define GD32_DMA_INTF0_OFFSET       0x0000
#define GD32_DMA_INTF1_OFFSET       0x0004
#define GD32_DMA_INTC0_OFFSET       0x0008
#define GD32_DMA_INTC1_OFFSET       0x000c

#define GD32_DMA_CH0CTL_OFFSET      0x0010
#define GD32_DMA_CH0CNT_OFFSET      0x0014
#define GD32_DMA_CH0PADDR_OFFSET    0x0018
#define GD32_DMA_CH0M0ADDR_OFFSET   0x001c
#define GD32_DMA_CH0M1ADDR_OFFSET   0x0020
#define GD32_DMA_CH0FCTL_OFFSET     0x0024

/* Generic channel register addressing */

#define GD32_DMA_CHCTL(dma, ch)     (((dma) + GD32_DMA_CH0CTL_OFFSET) + \
                                     0x18 * (ch))
#define GD32_DMA_CHCNT(dma, ch)     (((dma) + GD32_DMA_CH0CNT_OFFSET) + \
                                     0x18 * (ch))
#define GD32_DMA_CHPADDR(dma, ch)   (((dma) + GD32_DMA_CH0PADDR_OFFSET) + \
                                     0x18 * (ch))
#define GD32_DMA_CHM0ADDR(dma, ch)  (((dma) + GD32_DMA_CH0M0ADDR_OFFSET) + \
                                     0x18 * (ch))
#define GD32_DMA_CHM1ADDR(dma, ch)  (((dma) + GD32_DMA_CH0M1ADDR_OFFSET) + \
                                     0x18 * (ch))
#define GD32_DMA_CHFCTL(dma, ch)    (((dma) + GD32_DMA_CH0FCTL_OFFSET) + \
                                     0x18 * (ch))

/* DMAMUX request-router register offsets */

#define GD32_DMAMUX_RM_CH0CFG_OFFSET  0x0000
#define GD32_DMAMUX_RM_INTF_OFFSET    0x0080
#define GD32_DMAMUX_RM_INTC_OFFSET    0x0084

#define GD32_DMAMUX_RM_CHCFG(base, ch) \
  ((base) + GD32_DMAMUX_RM_CH0CFG_OFFSET + 4 * (ch))

/* DMA interrupt flags */

#define DMA_INTF_FEEIF              (1 << 0)
#define DMA_INTF_SDEIF              (1 << 2)
#define DMA_INTF_TAEIF              (1 << 3)
#define DMA_INTF_HTFIF              (1 << 4)
#define DMA_INTF_FTFIF              (1 << 5)

#define DMA_INTF_MASK               (DMA_INTF_FEEIF | DMA_INTF_SDEIF | \
                                     DMA_INTF_TAEIF | DMA_INTF_HTFIF | \
                                     DMA_INTF_FTFIF)
#define DMA_INTF_ERRIF              (DMA_INTF_FEEIF | DMA_INTF_SDEIF | \
                                     DMA_INTF_TAEIF)

/* DMA interrupt flag clear bits */

#define DMA_INTC_FEEIFC             (1 << 0)
#define DMA_INTC_SDEIFC             (1 << 2)
#define DMA_INTC_TAEIFC             (1 << 3)
#define DMA_INTC_HTFIFC             (1 << 4)
#define DMA_INTC_FTFIFC             (1 << 5)

#define DMA_INTC_ALL                (DMA_INTC_FEEIFC | DMA_INTC_SDEIFC | \
                                     DMA_INTC_TAEIFC | DMA_INTC_HTFIFC | \
                                     DMA_INTC_FTFIFC)

/* DMA control register bits */

#define DMA_CHXCTL_CHEN             (1 << 0)
#define DMA_CHXCTL_SDEIE            (1 << 1)
#define DMA_CHXCTL_TAEIE            (1 << 2)
#define DMA_CHXCTL_HTFIE            (1 << 3)
#define DMA_CHXCTL_FTFIE            (1 << 4)

#define DMA_CHXCTL_TM_SHIFT         (6)
#define DMA_CHXCTL_TM_MASK          (3 << DMA_CHXCTL_TM_SHIFT)
#define DMA_CHXCTL_TM(n)            ((n) << DMA_CHXCTL_TM_SHIFT)
#  define DMA_PERIPH_TO_MEMORY      DMA_CHXCTL_TM(0)
#  define DMA_MEMORY_TO_PERIPH      DMA_CHXCTL_TM(1)
#  define DMA_MEMORY_TO_MEMORY      DMA_CHXCTL_TM(2)

#define DMA_CHXCTL_CMEN             (1 << 8)
#define DMA_CHXCTL_PNAGA            (1 << 9)
#define DMA_CHXCTL_MNAGA            (1 << 10)

#define DMA_CHXCTL_PWIDTH_SHIFT     (11)
#define DMA_CHXCTL_PWIDTH_MASK      (3 << DMA_CHXCTL_PWIDTH_SHIFT)
#define DMA_CHXCTL_PWIDTH(n)        ((n) << DMA_CHXCTL_PWIDTH_SHIFT)
#  define DMA_PERIPH_WIDTH_8BIT     DMA_CHXCTL_PWIDTH(0)
#  define DMA_PERIPH_WIDTH_16BIT    DMA_CHXCTL_PWIDTH(1)
#  define DMA_PERIPH_WIDTH_32BIT    DMA_CHXCTL_PWIDTH(2)

#define DMA_CHXCTL_MWIDTH_SHIFT     (13)
#define DMA_CHXCTL_MWIDTH_MASK      (3 << DMA_CHXCTL_MWIDTH_SHIFT)
#define DMA_CHXCTL_MWIDTH(n)        ((n) << DMA_CHXCTL_MWIDTH_SHIFT)
#  define DMA_MEMORY_WIDTH_8BIT     DMA_CHXCTL_MWIDTH(0)
#  define DMA_MEMORY_WIDTH_16BIT    DMA_CHXCTL_MWIDTH(1)
#  define DMA_MEMORY_WIDTH_32BIT    DMA_CHXCTL_MWIDTH(2)

#define DMA_CHXCTL_PAIF             (1 << 15)

#define DMA_CHXCTL_PRIO_SHIFT       (16)
#define DMA_CHXCTL_PRIO_MASK        (3 << DMA_CHXCTL_PRIO_SHIFT)
#define DMA_CHXCTL_PRIO(n)          ((n) << DMA_CHXCTL_PRIO_SHIFT)
#  define DMA_PRIORITY_LOW          DMA_CHXCTL_PRIO(0)
#  define DMA_PRIORITY_MEDIUM       DMA_CHXCTL_PRIO(1)
#  define DMA_PRIORITY_HIGH         DMA_CHXCTL_PRIO(2)
#  define DMA_PRIORITY_ULTRA_HIGH   DMA_CHXCTL_PRIO(3)

#define DMA_CHXCTL_SBMEN            (1 << 18)  /* Switch-buffer mode enable */
#define DMA_CHXCTL_MBS              (1 << 19)  /* Memory buffer select */

#define DMA_CHXCTL_PBURST_SHIFT     (21)
#define DMA_CHXCTL_PBURST_MASK      (3 << DMA_CHXCTL_PBURST_SHIFT)
#define DMA_CHXCTL_PBURST(n)        ((n) << DMA_CHXCTL_PBURST_SHIFT)
#  define DMA_PBURST_SINGLE         DMA_CHXCTL_PBURST(0)
#  define DMA_PBURST_INCR4          DMA_CHXCTL_PBURST(1)
#  define DMA_PBURST_INCR8          DMA_CHXCTL_PBURST(2)
#  define DMA_PBURST_INCR16         DMA_CHXCTL_PBURST(3)

#define DMA_CHXCTL_MBURST_SHIFT     (23)
#define DMA_CHXCTL_MBURST_MASK      (3 << DMA_CHXCTL_MBURST_SHIFT)
#define DMA_CHXCTL_MBURST(n)        ((n) << DMA_CHXCTL_MBURST_SHIFT)
#  define DMA_MBURST_SINGLE         DMA_CHXCTL_MBURST(0)
#  define DMA_MBURST_INCR4          DMA_CHXCTL_MBURST(1)
#  define DMA_MBURST_INCR8          DMA_CHXCTL_MBURST(2)
#  define DMA_MBURST_INCR16         DMA_CHXCTL_MBURST(3)

/* DMA interrupt enable mask for gd32_dma_start */

#define DMA_INT_MASK                (DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE | \
                                     DMA_CHXCTL_HTFIE | DMA_CHXCTL_FTFIE)

/* DMA_CHxFCTL bits */

#define DMA_CHXFCTL_FCCV_SHIFT      (0)
#define DMA_CHXFCTL_FCCV_MASK       (3 << DMA_CHXFCTL_FCCV_SHIFT)
#define DMA_CHXFCTL_MDMEN           (1 << 2)
#define DMA_CHXFCTL_FCNT_SHIFT      (3)
#define DMA_CHXFCTL_FCNT_MASK       (7 << DMA_CHXFCTL_FCNT_SHIFT)
#define DMA_CHXFCTL_FEEIE           (1 << 7)   /* FIFO exception IE */

/* DMAMUX request-router channel configuration register */

#define DMAMUX_RM_CHXCFG_MUXID_MASK  0x000000ff

/* Increment mode helpers (same as E113) */

#define DMA_PERIPH_INCREASE_ENABLE  (0x00000000)
#define DMA_PERIPH_INCREASE_DISABLE (0x00000001)
#define DMA_PERIPH_INCREASE_FIX     (0x00000002)

#define DMA_MEMORY_INCREASE_ENABLE  (0x00000000)
#define DMA_MEMORY_INCREASE_DISABLE (0x00000001)

/* Circular mode helpers */

#define DMA_CIRCULAR_MODE_ENABLE    (0x00000000)
#define DMA_CIRCULAR_MODE_DISABLE   (0x00000001)

/****************************************************************************
 * DMAMUX Request IDs (from GD32H7xx Firmware Library V1.4.0)
 ****************************************************************************/

/* Memory to Memory */

#define DMA_REQUEST_M2M             0

/* Request Generator */

#define DMA_REQUEST_GENERATOR0      1
#define DMA_REQUEST_GENERATOR1      2
#define DMA_REQUEST_GENERATOR2      3
#define DMA_REQUEST_GENERATOR3      4
#define DMA_REQUEST_GENERATOR4      5
#define DMA_REQUEST_GENERATOR5      6
#define DMA_REQUEST_GENERATOR6      7
#define DMA_REQUEST_GENERATOR7      8

/* ADC */

#define DMA_REQUEST_ADC0            9
#define DMA_REQUEST_ADC1            10
#define DMA_REQUEST_ADC2            123

/* TIMER0 */

#define DMA_REQUEST_TIMER0_CH0      11
#define DMA_REQUEST_TIMER0_CH1      12
#define DMA_REQUEST_TIMER0_CH2      13
#define DMA_REQUEST_TIMER0_CH3      14
#define DMA_REQUEST_TIMER0_MCH0     15
#define DMA_REQUEST_TIMER0_MCH1     16
#define DMA_REQUEST_TIMER0_MCH2     17
#define DMA_REQUEST_TIMER0_MCH3     18
#define DMA_REQUEST_TIMER0_UP       19
#define DMA_REQUEST_TIMER0_TRG      20
#define DMA_REQUEST_TIMER0_CMT      21

/* TIMER1 */

#define DMA_REQUEST_TIMER1_CH0      22
#define DMA_REQUEST_TIMER1_CH1      23
#define DMA_REQUEST_TIMER1_CH2      24
#define DMA_REQUEST_TIMER1_CH3      25
#define DMA_REQUEST_TIMER1_UP       26
#define DMA_REQUEST_TIMER1_TRG      27

/* TIMER2 */

#define DMA_REQUEST_TIMER2_CH0      29
#define DMA_REQUEST_TIMER2_CH1      30
#define DMA_REQUEST_TIMER2_CH2      31
#define DMA_REQUEST_TIMER2_CH3      32
#define DMA_REQUEST_TIMER2_UP       33
#define DMA_REQUEST_TIMER2_TRG      35

/* TIMER3 */

#define DMA_REQUEST_TIMER3_CH0      36
#define DMA_REQUEST_TIMER3_CH1      37
#define DMA_REQUEST_TIMER3_CH2      38
#define DMA_REQUEST_TIMER3_CH3      39
#define DMA_REQUEST_TIMER3_TRG      41
#define DMA_REQUEST_TIMER3_UP       42

/* I2C */

#define DMA_REQUEST_I2C0_RX         43
#define DMA_REQUEST_I2C0_TX         44
#define DMA_REQUEST_I2C1_RX         45
#define DMA_REQUEST_I2C1_TX         46
#define DMA_REQUEST_I2C2_RX         87
#define DMA_REQUEST_I2C2_TX         88
#define DMA_REQUEST_I2C3_RX         184
#define DMA_REQUEST_I2C3_TX         185

/* SPI */

#define DMA_REQUEST_SPI0_RX         47
#define DMA_REQUEST_SPI0_TX         48
#define DMA_REQUEST_SPI1_RX         49
#define DMA_REQUEST_SPI1_TX         50
#define DMA_REQUEST_SPI2_RX         75
#define DMA_REQUEST_SPI2_TX         76
#define DMA_REQUEST_SPI3_RX         97
#define DMA_REQUEST_SPI3_TX         98
#define DMA_REQUEST_SPI4_RX         99
#define DMA_REQUEST_SPI4_TX         100
#define DMA_REQUEST_SPI5_RX         182
#define DMA_REQUEST_SPI5_TX         183

/* USART/UART */

#define DMA_REQUEST_USART0_RX       51
#define DMA_REQUEST_USART0_TX       52
#define DMA_REQUEST_USART1_RX       53
#define DMA_REQUEST_USART1_TX       54
#define DMA_REQUEST_USART2_RX       55
#define DMA_REQUEST_USART2_TX       56
#define DMA_REQUEST_UART3_RX        77
#define DMA_REQUEST_UART3_TX        78
#define DMA_REQUEST_UART4_RX        79
#define DMA_REQUEST_UART4_TX        80
#define DMA_REQUEST_USART5_RX       85
#define DMA_REQUEST_USART5_TX       86
#define DMA_REQUEST_UART6_RX        93
#define DMA_REQUEST_UART6_TX        94
#define DMA_REQUEST_UART7_RX        95
#define DMA_REQUEST_UART7_TX        96

/* TIMER4-7 */

#define DMA_REQUEST_TIMER7_CH0      57
#define DMA_REQUEST_TIMER7_CH1      58
#define DMA_REQUEST_TIMER7_CH2      59
#define DMA_REQUEST_TIMER7_CH3      60
#define DMA_REQUEST_TIMER7_MCH0     61
#define DMA_REQUEST_TIMER7_MCH1     62
#define DMA_REQUEST_TIMER7_MCH2     63
#define DMA_REQUEST_TIMER7_MCH3     64
#define DMA_REQUEST_TIMER7_UP       65
#define DMA_REQUEST_TIMER7_TRG      66
#define DMA_REQUEST_TIMER7_CMT      67
#define DMA_REQUEST_TIMER4_CH0      68
#define DMA_REQUEST_TIMER4_CH1      69
#define DMA_REQUEST_TIMER4_CH2      70
#define DMA_REQUEST_TIMER4_CH3      71
#define DMA_REQUEST_TIMER4_UP       72
#define DMA_REQUEST_TIMER4_TRG      74
#define DMA_REQUEST_TIMER5_UP       83
#define DMA_REQUEST_TIMER6_UP       84

/* DAC */

#define DMA_REQUEST_DAC_CH0         81
#define DMA_REQUEST_DAC_CH1         82

/* DCI/CAU/HAU */

#define DMA_REQUEST_DCI             89
#define DMA_REQUEST_CAU_IN          90
#define DMA_REQUEST_CAU_OUT         91
#define DMA_REQUEST_HAU_IN          92

/* SAI */

#define DMA_REQUEST_SAI0_B0         101
#define DMA_REQUEST_SAI0_B1         102
#define DMA_REQUEST_SAI1_B0         178
#define DMA_REQUEST_SAI1_B1         179
#define DMA_REQUEST_SAI2_B0         180
#define DMA_REQUEST_SAI2_B1         181

/* RSPDIF */

#define DMA_REQUEST_RSPDIF_DATA     103
#define DMA_REQUEST_RSPDIF_CS       104

/* HPDF */

#define DMA_REQUEST_HPDF_FLT0       105
#define DMA_REQUEST_HPDF_FLT1       106
#define DMA_REQUEST_HPDF_FLT2       107
#define DMA_REQUEST_HPDF_FLT3       108

/* FAC/TMU */

#define DMA_REQUEST_FAC_READ        124
#define DMA_REQUEST_FAC_WRITE       125
#define DMA_REQUEST_TMU_INPUT       126
#define DMA_REQUEST_TMU_OUTPUT      127

/* CAN */

#define DMA_REQUEST_CAN0            186
#define DMA_REQUEST_CAN1            187
#define DMA_REQUEST_CAN2            188

#endif /* __ARCH_ARM_SRC_GD32H7XX_HARDWARE_GD32H7XX_DMA_H */
