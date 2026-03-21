/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_dac.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DAC_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DAC base address *********************************************************/

#define GD32_DAC0_BASE              GD32_DAC_BASE

/* Register Offsets *********************************************************/

#define GD32_DAC_CTL0_OFFSET        0x0000      /* DAC control register 0 */
#define GD32_DAC_SWT_OFFSET         0x0004      /* DAC software trigger register */
#define GD32_DAC_OUT0_R12DH_OFFSET  0x0008      /* DAC OUT0 12-bit right-aligned data holding register */
#define GD32_DAC_OUT0_L12DH_OFFSET  0x000c      /* DAC OUT0 12-bit left-aligned data holding register */
#define GD32_DAC_OUT0_R8DH_OFFSET   0x0010      /* DAC OUT0 8-bit right-aligned data holding register */
#define GD32_DAC_OUT1_R12DH_OFFSET  0x0014      /* DAC OUT1 12-bit right-aligned data holding register */
#define GD32_DAC_OUT1_L12DH_OFFSET  0x0018      /* DAC OUT1 12-bit left-aligned data holding register */
#define GD32_DAC_OUT1_R8DH_OFFSET   0x001c      /* DAC OUT1 8-bit right-aligned data holding register */
#define GD32_DAC_CONC_R12DH_OFFSET  0x0020      /* DAC concurrent mode 12-bit right-aligned data holding register */
#define GD32_DAC_CONC_L12DH_OFFSET  0x0024      /* DAC concurrent mode 12-bit left-aligned data holding register */
#define GD32_DAC_CONC_R8DH_OFFSET   0x0028      /* DAC concurrent mode 8-bit right-aligned data holding register */
#define GD32_DAC_OUT0_DO_OFFSET     0x002c      /* DAC OUT0 data output register */
#define GD32_DAC_OUT1_DO_OFFSET     0x0030      /* DAC OUT1 data output register */

/* Register Addresses *******************************************************/

#define GD32_DAC_CTL0               (GD32_DAC0_BASE + GD32_DAC_CTL0_OFFSET)
#define GD32_DAC_SWT                (GD32_DAC0_BASE + GD32_DAC_SWT_OFFSET)
#define GD32_DAC_OUT0_R12DH         (GD32_DAC0_BASE + GD32_DAC_OUT0_R12DH_OFFSET)
#define GD32_DAC_OUT0_L12DH         (GD32_DAC0_BASE + GD32_DAC_OUT0_L12DH_OFFSET)
#define GD32_DAC_OUT0_R8DH          (GD32_DAC0_BASE + GD32_DAC_OUT0_R8DH_OFFSET)
#define GD32_DAC_OUT1_R12DH         (GD32_DAC0_BASE + GD32_DAC_OUT1_R12DH_OFFSET)
#define GD32_DAC_OUT1_L12DH         (GD32_DAC0_BASE + GD32_DAC_OUT1_L12DH_OFFSET)
#define GD32_DAC_OUT1_R8DH          (GD32_DAC0_BASE + GD32_DAC_OUT1_R8DH_OFFSET)
#define GD32_DAC_CONC_R12DH         (GD32_DAC0_BASE + GD32_DAC_CONC_R12DH_OFFSET)
#define GD32_DAC_CONC_L12DH         (GD32_DAC0_BASE + GD32_DAC_CONC_L12DH_OFFSET)
#define GD32_DAC_CONC_R8DH          (GD32_DAC0_BASE + GD32_DAC_CONC_R8DH_OFFSET)
#define GD32_DAC_OUT0_DO            (GD32_DAC0_BASE + GD32_DAC_OUT0_DO_OFFSET)
#define GD32_DAC_OUT1_DO            (GD32_DAC0_BASE + GD32_DAC_OUT1_DO_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* DAC control register 0 (DAC_CTL0) */

#define DAC_CTL0_DEN0               (1 << 0)   /* Bit 0:  DAC OUT0 enable */
#define DAC_CTL0_DBOFF0             (1 << 1)   /* Bit 1:  DAC OUT0 output buffer turn off */
#define DAC_CTL0_DTEN0              (1 << 2)   /* Bit 2:  DAC OUT0 trigger enable */
#define DAC_CTL0_DTSEL0_SHIFT      (3)         /* Bits 5-3: DAC OUT0 trigger selection */
#define DAC_CTL0_DTSEL0_MASK       (7 << DAC_CTL0_DTSEL0_SHIFT)
#define DAC_CTL0_DTSEL0(n)         ((uint32_t)(n) << DAC_CTL0_DTSEL0_SHIFT)
#define DAC_CTL0_DWM0_SHIFT        (6)         /* Bits 7-6: DAC OUT0 noise wave mode */
#define DAC_CTL0_DWM0_MASK         (3 << DAC_CTL0_DWM0_SHIFT)
#define DAC_CTL0_DWBW0_SHIFT       (8)         /* Bits 11-8: DAC OUT0 noise wave bit width */
#define DAC_CTL0_DWBW0_MASK        (0xf << DAC_CTL0_DWBW0_SHIFT)
#define DAC_CTL0_DDMAEN0            (1 << 12)  /* Bit 12: DAC OUT0 DMA enable */

/* Bits 13-15: Reserved */

#define DAC_CTL0_DEN1               (1 << 16)  /* Bit 16: DAC OUT1 enable */
#define DAC_CTL0_DBOFF1             (1 << 17)  /* Bit 17: DAC OUT1 output buffer turn off */
#define DAC_CTL0_DTEN1              (1 << 18)  /* Bit 18: DAC OUT1 trigger enable */
#define DAC_CTL0_DTSEL1_SHIFT      (19)        /* Bits 21-19: DAC OUT1 trigger selection */
#define DAC_CTL0_DTSEL1_MASK       (7 << DAC_CTL0_DTSEL1_SHIFT)
#define DAC_CTL0_DTSEL1(n)         ((uint32_t)(n) << DAC_CTL0_DTSEL1_SHIFT)
#define DAC_CTL0_DWM1_SHIFT        (22)        /* Bits 23-22: DAC OUT1 noise wave mode */
#define DAC_CTL0_DWM1_MASK         (3 << DAC_CTL0_DWM1_SHIFT)
#define DAC_CTL0_DWBW1_SHIFT       (24)        /* Bits 27-24: DAC OUT1 noise wave bit width */
#define DAC_CTL0_DWBW1_MASK        (0xf << DAC_CTL0_DWBW1_SHIFT)
#define DAC_CTL0_DDMAEN1            (1 << 28)  /* Bit 28: DAC OUT1 DMA enable */

/* Bits 29-31: Reserved */

/* DAC CR bits for generic channel access (shift by channel * 16) */

#define DAC_CR_EN                   (1 << 0)   /* Bit 0: DAC channel enable */
#define DAC_CR_BOFF                 (1 << 1)   /* Bit 1: DAC output buffer off */
#define DAC_CR_TEN                  (1 << 2)   /* Bit 2: DAC trigger enable */
#define DAC_CR_TSEL_SHIFT           (3)        /* Bits 5-3: DAC trigger selection */
#define DAC_CR_TSEL_MASK            (7 << DAC_CR_TSEL_SHIFT)
#define DAC_CR_TSEL(n)              ((uint32_t)(n) << DAC_CR_TSEL_SHIFT)
#define DAC_CR_WAVE_SHIFT           (6)        /* Bits 7-6: DAC noise wave mode */
#define DAC_CR_WAVE_MASK            (3 << DAC_CR_WAVE_SHIFT)
#define DAC_CR_WAVE_DISABLED        (0 << DAC_CR_WAVE_SHIFT)  /* Wave disabled */
#define DAC_CR_WAVE_LFSR            (1 << DAC_CR_WAVE_SHIFT)  /* LFSR noise mode */
#define DAC_CR_WAVE_TRIANGLE        (2 << DAC_CR_WAVE_SHIFT)  /* Triangle noise mode */
#define DAC_CR_MAMP_SHIFT           (8)                       /* Bits 11-8: DAC noise wave bit width / amplitude */
#define DAC_CR_MAMP_MASK            (0xf << DAC_CR_MAMP_SHIFT)
#define DAC_CR_MAMP_AMP1            (0 << DAC_CR_MAMP_SHIFT)   /* Unmask bit 0 / amplitude 1 */
#define DAC_CR_MAMP_AMP3            (1 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 1:0 / amplitude 3 */
#define DAC_CR_MAMP_AMP7            (2 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 2:0 / amplitude 7 */
#define DAC_CR_MAMP_AMP15           (3 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 3:0 / amplitude 15 */
#define DAC_CR_MAMP_AMP31           (4 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 4:0 / amplitude 31 */
#define DAC_CR_MAMP_AMP63           (5 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 5:0 / amplitude 63 */
#define DAC_CR_MAMP_AMP127          (6 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 6:0 / amplitude 127 */
#define DAC_CR_MAMP_AMP255          (7 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 7:0 / amplitude 255 */
#define DAC_CR_MAMP_AMP511          (8 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 8:0 / amplitude 511 */
#define DAC_CR_MAMP_AMP1023         (9 << DAC_CR_MAMP_SHIFT)   /* Unmask bits 9:0 / amplitude 1023 */
#define DAC_CR_MAMP_AMP2047         (10 << DAC_CR_MAMP_SHIFT)  /* Unmask bits 10:0 / amplitude 2047 */
#define DAC_CR_MAMP_AMP4095         (11 << DAC_CR_MAMP_SHIFT)  /* Unmask bits 11:0 / amplitude 4095 */
#define DAC_CR_DMAEN                (1 << 12)                  /* Bit 12: DAC DMA enable */

/* DAC trigger selection values
 *
 * GD32E11X DAC trigger sources:
 *   TSEL Value  Source
 *   ----------  ----------------------
 *   000 (0)     TIMER5 TRGO
 *   001 (1)     TIMER2 TRGO
 *   010 (2)     TIMER6 TRGO
 *   011 (3)     TIMER4 TRGO
 *   100 (4)     TIMER1 TRGO
 *   101 (5)     TIMER3 TRGO
 *   110 (6)     EXTI line 9
 *   111 (7)     Software trigger
 */

#define DAC_CR_TSEL_TIM5            DAC_CR_TSEL(0)  /* TIMER5 TRGO */
#define DAC_CR_TSEL_TIM2            DAC_CR_TSEL(1)  /* TIMER2 TRGO */
#define DAC_CR_TSEL_TIM6            DAC_CR_TSEL(2)  /* TIMER6 TRGO */
#define DAC_CR_TSEL_TIM4            DAC_CR_TSEL(3)  /* TIMER4 TRGO */
#define DAC_CR_TSEL_TIM1            DAC_CR_TSEL(4)  /* TIMER1 TRGO */
#define DAC_CR_TSEL_TIM3            DAC_CR_TSEL(5)  /* TIMER3 TRGO */
#define DAC_CR_TSEL_EXTI9           DAC_CR_TSEL(6)  /* EXTI line 9 */
#define DAC_CR_TSEL_SW              DAC_CR_TSEL(7)  /* Software trigger */

/* DAC software trigger register (DAC_SWT) */

#define DAC_SWT_SWTR0               (1 << 0)   /* Bit 0: DAC OUT0 software trigger */
#define DAC_SWT_SWTR1               (1 << 1)   /* Bit 1: DAC OUT1 software trigger */

/* DAC OUT0 data output register (DAC_OUT0_DO) */

#define DAC_OUT0_DO_MASK            (0xfff)     /* Bits 11-0: DAC OUT0 12-bit output data */

/* DAC OUT1 data output register (DAC_OUT1_DO) */

#define DAC_OUT1_DO_MASK            (0xfff)     /* Bits 11-0: DAC OUT1 12-bit output data */

/* DAC OUT0 data holding register bit field definitions */

#define DAC_OUT0_R12DH_MASK         (0xfff)     /* Bits 11-0: 12-bit right-aligned data */
#define DAC_OUT0_L12DH_MASK         (0xfff0)    /* Bits 15-4: 12-bit left-aligned data */
#define DAC_OUT0_R8DH_MASK          (0xff)      /* Bits 7-0: 8-bit right-aligned data */

/* DAC OUT1 data holding register bit field definitions */

#define DAC_OUT1_R12DH_MASK         (0xfff)     /* Bits 11-0: 12-bit right-aligned data */
#define DAC_OUT1_L12DH_MASK         (0xfff0)    /* Bits 15-4: 12-bit left-aligned data */
#define DAC_OUT1_R8DH_MASK          (0xff)      /* Bits 7-0: 8-bit right-aligned data */

/* DAC concurrent mode data holding register bit field definitions */

#define DAC_CONC_R12DH_OUT0_SHIFT   (0)         /* Bits 11-0: OUT0 12-bit right-aligned */
#define DAC_CONC_R12DH_OUT0_MASK    (0xfff << DAC_CONC_R12DH_OUT0_SHIFT)
#define DAC_CONC_R12DH_OUT1_SHIFT   (16)        /* Bits 27-16: OUT1 12-bit right-aligned */
#define DAC_CONC_R12DH_OUT1_MASK    (0xfff << DAC_CONC_R12DH_OUT1_SHIFT)
#define DAC_CONC_L12DH_OUT0_SHIFT   (4)         /* Bits 15-4: OUT0 12-bit left-aligned */
#define DAC_CONC_L12DH_OUT0_MASK    (0xfff << DAC_CONC_L12DH_OUT0_SHIFT)
#define DAC_CONC_L12DH_OUT1_SHIFT   (20)        /* Bits 31-20: OUT1 12-bit left-aligned */
#define DAC_CONC_L12DH_OUT1_MASK    (0xfff << DAC_CONC_L12DH_OUT1_SHIFT)
#define DAC_CONC_R8DH_OUT0_SHIFT    (0)         /* Bits 7-0: OUT0 8-bit right-aligned */
#define DAC_CONC_R8DH_OUT0_MASK     (0xff << DAC_CONC_R8DH_OUT0_SHIFT)
#define DAC_CONC_R8DH_OUT1_SHIFT    (8)         /* Bits 15-8: OUT1 8-bit right-aligned */
#define DAC_CONC_R8DH_OUT1_MASK     (0xff << DAC_CONC_R8DH_OUT1_SHIFT)

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DAC_H */
