/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_adc.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_ADC_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Base Addresses ***********************************************************/

/* GD32E11x has 2 separate ADC units, each with its own base.
 * GD32_ADC_BASE is defined in gd32e113_memorymap.h as APB2 + 0x2400.
 * ADC0: GD32_ADC_BASE + 0x000
 * ADC1: GD32_ADC_BASE + 0x400
 */

#define GD32_ADC0_BASE               (GD32_ADC_BASE + 0x0000U)  /* ADC0 base */
#define GD32_ADC1_BASE               (GD32_ADC_BASE + 0x0400U)  /* ADC1 base */

/* Number of ADC units */

#define GD32_NADC                    2

/* Register Offsets *********************************************************/

#define GD32_ADC_STAT_OFFSET         0x0000  /* ADC status register */
#define GD32_ADC_CTL0_OFFSET         0x0004  /* ADC control register 0 */
#define GD32_ADC_CTL1_OFFSET         0x0008  /* ADC control register 1 */
#define GD32_ADC_SAMPT0_OFFSET       0x000c  /* ADC sampling time register 0 (channels 10..17) */
#define GD32_ADC_SAMPT1_OFFSET       0x0010  /* ADC sampling time register 1 (channels 0..9) */
#define GD32_ADC_IOFF0_OFFSET        0x0014  /* ADC inserted channel data offset register 0 */
#define GD32_ADC_IOFF1_OFFSET        0x0018  /* ADC inserted channel data offset register 1 */
#define GD32_ADC_IOFF2_OFFSET        0x001c  /* ADC inserted channel data offset register 2 */
#define GD32_ADC_IOFF3_OFFSET        0x0020  /* ADC inserted channel data offset register 3 */
#define GD32_ADC_WDHT_OFFSET         0x0024  /* ADC watchdog high threshold register */
#define GD32_ADC_WDLT_OFFSET         0x0028  /* ADC watchdog low threshold register */
#define GD32_ADC_RSQ0_OFFSET         0x002c  /* ADC regular sequence register 0 (contains length) */
#define GD32_ADC_RSQ1_OFFSET         0x0030  /* ADC regular sequence register 1 */
#define GD32_ADC_RSQ2_OFFSET         0x0034  /* ADC regular sequence register 2 */
#define GD32_ADC_ISQ_OFFSET          0x0038  /* ADC inserted sequence register */
#define GD32_ADC_IDATA0_OFFSET       0x003c  /* ADC inserted data register 0 */
#define GD32_ADC_IDATA1_OFFSET       0x0040  /* ADC inserted data register 1 */
#define GD32_ADC_IDATA2_OFFSET       0x0044  /* ADC inserted data register 2 */
#define GD32_ADC_IDATA3_OFFSET       0x0048  /* ADC inserted data register 3 */
#define GD32_ADC_RDATA_OFFSET        0x004c  /* ADC regular data register */
#define GD32_ADC_OVSAMPCTL_OFFSET    0x0080  /* ADC oversampling control register (GD32E11x only) */

/* Register Addresses *******************************************************/

/* ADC0 registers */

#define GD32_ADC0_STAT               (GD32_ADC0_BASE + GD32_ADC_STAT_OFFSET)
#define GD32_ADC0_CTL0               (GD32_ADC0_BASE + GD32_ADC_CTL0_OFFSET)
#define GD32_ADC0_CTL1               (GD32_ADC0_BASE + GD32_ADC_CTL1_OFFSET)
#define GD32_ADC0_SAMPT0             (GD32_ADC0_BASE + GD32_ADC_SAMPT0_OFFSET)
#define GD32_ADC0_SAMPT1             (GD32_ADC0_BASE + GD32_ADC_SAMPT1_OFFSET)
#define GD32_ADC0_IOFF0              (GD32_ADC0_BASE + GD32_ADC_IOFF0_OFFSET)
#define GD32_ADC0_IOFF1              (GD32_ADC0_BASE + GD32_ADC_IOFF1_OFFSET)
#define GD32_ADC0_IOFF2              (GD32_ADC0_BASE + GD32_ADC_IOFF2_OFFSET)
#define GD32_ADC0_IOFF3              (GD32_ADC0_BASE + GD32_ADC_IOFF3_OFFSET)
#define GD32_ADC0_WDHT               (GD32_ADC0_BASE + GD32_ADC_WDHT_OFFSET)
#define GD32_ADC0_WDLT               (GD32_ADC0_BASE + GD32_ADC_WDLT_OFFSET)
#define GD32_ADC0_RSQ0               (GD32_ADC0_BASE + GD32_ADC_RSQ0_OFFSET)
#define GD32_ADC0_RSQ1               (GD32_ADC0_BASE + GD32_ADC_RSQ1_OFFSET)
#define GD32_ADC0_RSQ2               (GD32_ADC0_BASE + GD32_ADC_RSQ2_OFFSET)
#define GD32_ADC0_ISQ                (GD32_ADC0_BASE + GD32_ADC_ISQ_OFFSET)
#define GD32_ADC0_IDATA0             (GD32_ADC0_BASE + GD32_ADC_IDATA0_OFFSET)
#define GD32_ADC0_IDATA1             (GD32_ADC0_BASE + GD32_ADC_IDATA1_OFFSET)
#define GD32_ADC0_IDATA2             (GD32_ADC0_BASE + GD32_ADC_IDATA2_OFFSET)
#define GD32_ADC0_IDATA3             (GD32_ADC0_BASE + GD32_ADC_IDATA3_OFFSET)
#define GD32_ADC0_RDATA              (GD32_ADC0_BASE + GD32_ADC_RDATA_OFFSET)
#define GD32_ADC0_OVSAMPCTL          (GD32_ADC0_BASE + GD32_ADC_OVSAMPCTL_OFFSET)

/* ADC1 registers */

#define GD32_ADC1_STAT               (GD32_ADC1_BASE + GD32_ADC_STAT_OFFSET)
#define GD32_ADC1_CTL0               (GD32_ADC1_BASE + GD32_ADC_CTL0_OFFSET)
#define GD32_ADC1_CTL1               (GD32_ADC1_BASE + GD32_ADC_CTL1_OFFSET)
#define GD32_ADC1_SAMPT0             (GD32_ADC1_BASE + GD32_ADC_SAMPT0_OFFSET)
#define GD32_ADC1_SAMPT1             (GD32_ADC1_BASE + GD32_ADC_SAMPT1_OFFSET)
#define GD32_ADC1_IOFF0              (GD32_ADC1_BASE + GD32_ADC_IOFF0_OFFSET)
#define GD32_ADC1_IOFF1              (GD32_ADC1_BASE + GD32_ADC_IOFF1_OFFSET)
#define GD32_ADC1_IOFF2              (GD32_ADC1_BASE + GD32_ADC_IOFF2_OFFSET)
#define GD32_ADC1_IOFF3              (GD32_ADC1_BASE + GD32_ADC_IOFF3_OFFSET)
#define GD32_ADC1_WDHT               (GD32_ADC1_BASE + GD32_ADC_WDHT_OFFSET)
#define GD32_ADC1_WDLT               (GD32_ADC1_BASE + GD32_ADC_WDLT_OFFSET)
#define GD32_ADC1_RSQ0               (GD32_ADC1_BASE + GD32_ADC_RSQ0_OFFSET)
#define GD32_ADC1_RSQ1               (GD32_ADC1_BASE + GD32_ADC_RSQ1_OFFSET)
#define GD32_ADC1_RSQ2               (GD32_ADC1_BASE + GD32_ADC_RSQ2_OFFSET)
#define GD32_ADC1_ISQ                (GD32_ADC1_BASE + GD32_ADC_ISQ_OFFSET)
#define GD32_ADC1_IDATA0             (GD32_ADC1_BASE + GD32_ADC_IDATA0_OFFSET)
#define GD32_ADC1_IDATA1             (GD32_ADC1_BASE + GD32_ADC_IDATA1_OFFSET)
#define GD32_ADC1_IDATA2             (GD32_ADC1_BASE + GD32_ADC_IDATA2_OFFSET)
#define GD32_ADC1_IDATA3             (GD32_ADC1_BASE + GD32_ADC_IDATA3_OFFSET)
#define GD32_ADC1_RDATA              (GD32_ADC1_BASE + GD32_ADC_RDATA_OFFSET)
#define GD32_ADC1_OVSAMPCTL          (GD32_ADC1_BASE + GD32_ADC_OVSAMPCTL_OFFSET)

/* Register Bit Definitions *************************************************/

/* ADC status register (ADC_STAT) */

#define ADC_STAT_WDE                 (1 << 0)  /* Bit 0: Analog watchdog event flag */
#define ADC_STAT_EOC                 (1 << 1)  /* Bit 1: End of conversion */
#define ADC_STAT_EOIC                (1 << 2)  /* Bit 2: Inserted channel end of conversion */
#define ADC_STAT_STIC                (1 << 3)  /* Bit 3: Inserted channel start flag */
#define ADC_STAT_STRC                (1 << 4)  /* Bit 4: Regular channel start flag */

/* ADC control register 0 (ADC_CTL0) */

#define ADC_CTL0_WDCHSEL_SHIFT       (0)       /* Bits 4-0: Analog watchdog channel select */
#define ADC_CTL0_WDCHSEL_MASK        (0x1f << ADC_CTL0_WDCHSEL_SHIFT)
#define ADC_CTL0_EOCIE               (1 << 5)  /* Bit 5: Interrupt enable for EOC */
#define ADC_CTL0_WDEIE               (1 << 6)  /* Bit 6: Analog watchdog interrupt enable */
#define ADC_CTL0_EOICIE              (1 << 7)  /* Bit 7: Interrupt enable for inserted channels */
#define ADC_CTL0_SM                  (1 << 8)  /* Bit 8: Scan mode */
#define ADC_CTL0_WDSC                (1 << 9)  /* Bit 9: Watchdog on single channel in scan mode */
#define ADC_CTL0_ICA                 (1 << 10) /* Bit 10: Automatic inserted group conversion */
#define ADC_CTL0_DISRC               (1 << 11) /* Bit 11: Discontinuous mode on regular channels */
#define ADC_CTL0_DISIC               (1 << 12) /* Bit 12: Discontinuous mode on inserted channels */
#define ADC_CTL0_DISNUM_SHIFT        (13)      /* Bits 15-13: Discontinuous mode channel count */
#define ADC_CTL0_DISNUM_MASK         (0x07 << ADC_CTL0_DISNUM_SHIFT)
#define ADC_CTL0_SYNCM_SHIFT         (16)
#define ADC_CTL0_SYNCM_MASK          (0x0f << ADC_CTL0_SYNCM_SHIFT)
#define ADC_CTL0_IWDEN               (1 << 22) /* Bit 22: Analog watchdog enable on inserted channels */
#define ADC_CTL0_RWDEN               (1 << 23) /* Bit 23: Analog watchdog enable on regular channels */

/* Sync mode definitions (ADC_CTL0 SYNCM bits 19-16) */

#define ADC_CTL0_SYNCM_FREE          (0 << ADC_CTL0_SYNCM_SHIFT)  /* Independent mode */
#define ADC_CTL0_SYNCM_RPAR_IPAR     (1 << ADC_CTL0_SYNCM_SHIFT)  /* Regular parallel + inserted parallel */
#define ADC_CTL0_SYNCM_RPAR_IROT     (2 << ADC_CTL0_SYNCM_SHIFT)  /* Regular parallel + trigger rotation */
#define ADC_CTL0_SYNCM_IPAR_RFF      (3 << ADC_CTL0_SYNCM_SHIFT)  /* Inserted parallel + follow-up fast */
#define ADC_CTL0_SYNCM_IPAR_RFS      (4 << ADC_CTL0_SYNCM_SHIFT)  /* Inserted parallel + follow-up slow */
#define ADC_CTL0_SYNCM_IPAR          (5 << ADC_CTL0_SYNCM_SHIFT)  /* Inserted parallel only */
#define ADC_CTL0_SYNCM_RPAR          (6 << ADC_CTL0_SYNCM_SHIFT)  /* Regular parallel only */
#define ADC_CTL0_SYNCM_RFF           (7 << ADC_CTL0_SYNCM_SHIFT)  /* Follow-up fast only */
#define ADC_CTL0_SYNCM_RFS           (8 << ADC_CTL0_SYNCM_SHIFT)  /* Follow-up slow only */
#define ADC_CTL0_SYNCM_IROT          (9 << ADC_CTL0_SYNCM_SHIFT)  /* Trigger rotation only */

/* ADC control register 1 (ADC_CTL1) */

#define ADC_CTL1_ADCON               (1 << 0)  /* Bit 0: ADC converter on */
#define ADC_CTL1_CTN                 (1 << 1)  /* Bit 1: Continuous conversion */
#define ADC_CTL1_CLB                 (1 << 2)  /* Bit 2: ADC calibration */
#define ADC_CTL1_RSTCLB              (1 << 3)  /* Bit 3: Reset calibration */
#define ADC_CTL1_DMA                 (1 << 8)  /* Bit 8: Direct memory access mode */
#define ADC_CTL1_DAL                 (1 << 11) /* Bit 11: Data alignment (0=right, 1=left) */
#define ADC_CTL1_ETSIC_SHIFT         (12)      /* Bits 14-12: External trigger select for inserted channel */
#define ADC_CTL1_ETSIC_MASK          (7 << ADC_CTL1_ETSIC_SHIFT)
#define ADC_CTL1_ETEIC               (1 << 15) /* Bit 15: External trigger enable for inserted channel */
#define ADC_CTL1_ETSRC_SHIFT         (17)      /* Bits 19-17: External trigger select for regular channel */
#define ADC_CTL1_ETSRC_MASK          (7 << ADC_CTL1_ETSRC_SHIFT)
#define ADC_CTL1_ETERC               (1 << 20) /* Bit 20: External trigger enable for regular channels */
#define ADC_CTL1_SWICST              (1 << 21) /* Bit 21: Start on inserted channel */
#define ADC_CTL1_SWRCST              (1 << 22) /* Bit 22: Start on regular channel */
#define ADC_CTL1_TSVREN              (1 << 23) /* Bit 23: Channel 16 and 17 enable (temp sensor/Vrefint) */

/* External trigger select for inserted channel (ADC_CTL1 ETSIC bits 14-12) */

#define ADC_CTL1_ETSIC_T0TRGO        (0 << ADC_CTL1_ETSIC_SHIFT)  /* Timer 0 TRGO event */
#define ADC_CTL1_ETSIC_T0CH3         (1 << ADC_CTL1_ETSIC_SHIFT)  /* Timer 0 CC3 event */
#define ADC_CTL1_ETSIC_T1TRGO        (2 << ADC_CTL1_ETSIC_SHIFT)  /* Timer 1 TRGO event */
#define ADC_CTL1_ETSIC_T1CH0         (3 << ADC_CTL1_ETSIC_SHIFT)  /* Timer 1 CC0 event */
#define ADC_CTL1_ETSIC_T2CH3         (4 << ADC_CTL1_ETSIC_SHIFT)  /* Timer 2 CC3 event */
#define ADC_CTL1_ETSIC_T3TRGO        (5 << ADC_CTL1_ETSIC_SHIFT)  /* Timer 3 TRGO event */
#define ADC_CTL1_ETSIC_EXTI15        (6 << ADC_CTL1_ETSIC_SHIFT)  /* EXTI line 15 */
#define ADC_CTL1_ETSIC_NONE          (7 << ADC_CTL1_ETSIC_SHIFT)  /* Software trigger */

/* External trigger select for regular channel (ADC_CTL1 ETSRC bits 19-17) */

#define ADC_CTL1_ETSRC_T0CH0         (0 << ADC_CTL1_ETSRC_SHIFT)  /* Timer 0 CC0 event */
#define ADC_CTL1_ETSRC_T0CH1         (1 << ADC_CTL1_ETSRC_SHIFT)  /* Timer 0 CC1 event */
#define ADC_CTL1_ETSRC_T0CH2         (2 << ADC_CTL1_ETSRC_SHIFT)  /* Timer 0 CC2 event */
#define ADC_CTL1_ETSRC_T1CH1         (3 << ADC_CTL1_ETSRC_SHIFT)  /* Timer 1 CC1 event */
#define ADC_CTL1_ETSRC_T2TRGO        (4 << ADC_CTL1_ETSRC_SHIFT)  /* Timer 2 TRGO event */
#define ADC_CTL1_ETSRC_T3CH3         (5 << ADC_CTL1_ETSRC_SHIFT)  /* Timer 3 CC3 event */
#define ADC_CTL1_ETSRC_EXTI11        (6 << ADC_CTL1_ETSRC_SHIFT)  /* EXTI line 11 / Timer 7 TRGO */
#define ADC_CTL1_ETSRC_NONE          (7 << ADC_CTL1_ETSRC_SHIFT)  /* Software trigger */

/* ADC sampling time register 0 (ADC_SAMPT0) - channels 10..17 */

#define ADC_SAMPT0_SMP10_SHIFT       (0)       /* Bits 2-0: Channel 10 sample time */
#define ADC_SAMPT0_SMP10_MASK        (7 << ADC_SAMPT0_SMP10_SHIFT)
#define ADC_SAMPT0_SMP11_SHIFT       (3)       /* Bits 5-3: Channel 11 sample time */
#define ADC_SAMPT0_SMP11_MASK        (7 << ADC_SAMPT0_SMP11_SHIFT)
#define ADC_SAMPT0_SMP12_SHIFT       (6)       /* Bits 8-6: Channel 12 sample time */
#define ADC_SAMPT0_SMP12_MASK        (7 << ADC_SAMPT0_SMP12_SHIFT)
#define ADC_SAMPT0_SMP13_SHIFT       (9)       /* Bits 11-9: Channel 13 sample time */
#define ADC_SAMPT0_SMP13_MASK        (7 << ADC_SAMPT0_SMP13_SHIFT)
#define ADC_SAMPT0_SMP14_SHIFT       (12)      /* Bits 14-12: Channel 14 sample time */
#define ADC_SAMPT0_SMP14_MASK        (7 << ADC_SAMPT0_SMP14_SHIFT)
#define ADC_SAMPT0_SMP15_SHIFT       (15)      /* Bits 17-15: Channel 15 sample time */
#define ADC_SAMPT0_SMP15_MASK        (7 << ADC_SAMPT0_SMP15_SHIFT)
#define ADC_SAMPT0_SMP16_SHIFT       (18)      /* Bits 20-18: Channel 16 sample time */
#define ADC_SAMPT0_SMP16_MASK        (7 << ADC_SAMPT0_SMP16_SHIFT)
#define ADC_SAMPT0_SMP17_SHIFT       (21)      /* Bits 23-21: Channel 17 sample time */
#define ADC_SAMPT0_SMP17_MASK        (7 << ADC_SAMPT0_SMP17_SHIFT)

/* ADC sampling time register 1 (ADC_SAMPT1) - channels 0..9 */

#define ADC_SAMPT1_SMP0_SHIFT        (0)       /* Bits 2-0: Channel 0 sample time */
#define ADC_SAMPT1_SMP0_MASK         (7 << ADC_SAMPT1_SMP0_SHIFT)
#define ADC_SAMPT1_SMP1_SHIFT        (3)       /* Bits 5-3: Channel 1 sample time */
#define ADC_SAMPT1_SMP1_MASK         (7 << ADC_SAMPT1_SMP1_SHIFT)
#define ADC_SAMPT1_SMP2_SHIFT        (6)       /* Bits 8-6: Channel 2 sample time */
#define ADC_SAMPT1_SMP2_MASK         (7 << ADC_SAMPT1_SMP2_SHIFT)
#define ADC_SAMPT1_SMP3_SHIFT        (9)       /* Bits 11-9: Channel 3 sample time */
#define ADC_SAMPT1_SMP3_MASK         (7 << ADC_SAMPT1_SMP3_SHIFT)
#define ADC_SAMPT1_SMP4_SHIFT        (12)      /* Bits 14-12: Channel 4 sample time */
#define ADC_SAMPT1_SMP4_MASK         (7 << ADC_SAMPT1_SMP4_SHIFT)
#define ADC_SAMPT1_SMP5_SHIFT        (15)      /* Bits 17-15: Channel 5 sample time */
#define ADC_SAMPT1_SMP5_MASK         (7 << ADC_SAMPT1_SMP5_SHIFT)
#define ADC_SAMPT1_SMP6_SHIFT        (18)      /* Bits 20-18: Channel 6 sample time */
#define ADC_SAMPT1_SMP6_MASK         (7 << ADC_SAMPT1_SMP6_SHIFT)
#define ADC_SAMPT1_SMP7_SHIFT        (21)      /* Bits 23-21: Channel 7 sample time */
#define ADC_SAMPT1_SMP7_MASK         (7 << ADC_SAMPT1_SMP7_SHIFT)
#define ADC_SAMPT1_SMP8_SHIFT        (24)      /* Bits 26-24: Channel 8 sample time */
#define ADC_SAMPT1_SMP8_MASK         (7 << ADC_SAMPT1_SMP8_SHIFT)
#define ADC_SAMPT1_SMP9_SHIFT        (27)      /* Bits 29-27: Channel 9 sample time */
#define ADC_SAMPT1_SMP9_MASK         (7 << ADC_SAMPT1_SMP9_SHIFT)

/* Sample time values (used in both SAMPT0 and SAMPT1) */

#define ADC_SMPR_1p5                 0         /* 1.5 cycles */
#define ADC_SMPR_7p5                 1         /* 7.5 cycles */
#define ADC_SMPR_13p5                2         /* 13.5 cycles */
#define ADC_SMPR_28p5                3         /* 28.5 cycles */
#define ADC_SMPR_41p5                4         /* 41.5 cycles */
#define ADC_SMPR_55p5                5         /* 55.5 cycles */
#define ADC_SMPR_71p5                6         /* 71.5 cycles */
#define ADC_SMPR_239p5               7         /* 239.5 cycles */

/* ADC injected/inserted channel data offset registers (ADC_IOFFx) */

#define ADC_IOFF_SHIFT               (0)       /* Bits 11-0: Data offset for inserted channel x */
#define ADC_IOFF_MASK                (0x0fff << ADC_IOFF_SHIFT)

/* ADC watchdog high threshold register (ADC_WDHT) */

#define ADC_WDHT_SHIFT               (0)       /* Bits 11-0: Analog watchdog high threshold */
#define ADC_WDHT_MASK                (0x0fff << ADC_WDHT_SHIFT)

/* ADC watchdog low threshold register (ADC_WDLT) */

#define ADC_WDLT_SHIFT               (0)       /* Bits 11-0: Analog watchdog low threshold */
#define ADC_WDLT_MASK                (0x0fff << ADC_WDLT_SHIFT)

/* ADC regular sequence register 0 (ADC_RSQ0)
 * Contains regular sequence length and 13th..16th conversions.
 * GD32E11x maps channels in ascending order: RSQ2 holds 1st..6th,
 * RSQ1 holds 7th..12th, RSQ0 holds 13th..16th + length.
 */

#define ADC_RSQ0_RSQ12_SHIFT         (0)       /* Bits 4-0: 13th conversion in regular sequence */
#define ADC_RSQ0_RSQ12_MASK          (0x1f << ADC_RSQ0_RSQ12_SHIFT)
#define ADC_RSQ0_RSQ13_SHIFT         (5)       /* Bits 9-5: 14th conversion in regular sequence */
#define ADC_RSQ0_RSQ13_MASK          (0x1f << ADC_RSQ0_RSQ13_SHIFT)
#define ADC_RSQ0_RSQ14_SHIFT         (10)      /* Bits 14-10: 15th conversion in regular sequence */
#define ADC_RSQ0_RSQ14_MASK          (0x1f << ADC_RSQ0_RSQ14_SHIFT)
#define ADC_RSQ0_RSQ15_SHIFT         (15)      /* Bits 19-15: 16th conversion in regular sequence */
#define ADC_RSQ0_RSQ15_MASK          (0x1f << ADC_RSQ0_RSQ15_SHIFT)
#define ADC_RSQ0_RL_SHIFT            (20)      /* Bits 23-20: Regular channel sequence length */
#define ADC_RSQ0_RL_MASK             (0x0f << ADC_RSQ0_RL_SHIFT)
#define ADC_RSQ0_RESERVED            (0xff000000)
#define ADC_RSQ0_FIRST               (13)
#define ADC_RSQ0_LAST                (16)
#define ADC_RSQ0_SQ_OFFSET           (0)

/* ADC regular sequence register 1 (ADC_RSQ1)
 * Contains 7th..12th conversions.
 */

#define ADC_RSQ1_RSQ6_SHIFT          (0)       /* Bits 4-0: 7th conversion in regular sequence */
#define ADC_RSQ1_RSQ6_MASK           (0x1f << ADC_RSQ1_RSQ6_SHIFT)
#define ADC_RSQ1_RSQ7_SHIFT          (5)       /* Bits 9-5: 8th conversion in regular sequence */
#define ADC_RSQ1_RSQ7_MASK           (0x1f << ADC_RSQ1_RSQ7_SHIFT)
#define ADC_RSQ1_RSQ8_SHIFT          (10)      /* Bits 14-10: 9th conversion in regular sequence */
#define ADC_RSQ1_RSQ8_MASK           (0x1f << ADC_RSQ1_RSQ8_SHIFT)
#define ADC_RSQ1_RSQ9_SHIFT          (15)      /* Bits 19-15: 10th conversion in regular sequence */
#define ADC_RSQ1_RSQ9_MASK           (0x1f << ADC_RSQ1_RSQ9_SHIFT)
#define ADC_RSQ1_RSQ10_SHIFT         (20)      /* Bits 24-20: 11th conversion in regular sequence */
#define ADC_RSQ1_RSQ10_MASK          (0x1f << ADC_RSQ1_RSQ10_SHIFT)
#define ADC_RSQ1_RSQ11_SHIFT         (25)      /* Bits 29-25: 12th conversion in regular sequence */
#define ADC_RSQ1_RSQ11_MASK          (0x1f << ADC_RSQ1_RSQ11_SHIFT)
#define ADC_RSQ1_RESERVED            (0xc0000000)
#define ADC_RSQ1_FIRST               (7)
#define ADC_RSQ1_LAST                (12)
#define ADC_RSQ1_SQ_OFFSET           (0)

/* ADC regular sequence register 2 (ADC_RSQ2)
 * Contains 1st..6th conversions.
 */

#define ADC_RSQ2_RSQ0_SHIFT          (0)       /* Bits 4-0: 1st conversion in regular sequence */
#define ADC_RSQ2_RSQ0_MASK           (0x1f << ADC_RSQ2_RSQ0_SHIFT)
#define ADC_RSQ2_RSQ1_SHIFT          (5)       /* Bits 9-5: 2nd conversion in regular sequence */
#define ADC_RSQ2_RSQ1_MASK           (0x1f << ADC_RSQ2_RSQ1_SHIFT)
#define ADC_RSQ2_RSQ2_SHIFT          (10)      /* Bits 14-10:3rd conversion in regular sequence */
#define ADC_RSQ2_RSQ2_MASK           (0x1f << ADC_RSQ2_RSQ2_SHIFT)
#define ADC_RSQ2_RSQ3_SHIFT          (15)      /* Bits 19-15: 4th conversion in regular sequence */
#define ADC_RSQ2_RSQ3_MASK           (0x1f << ADC_RSQ2_RSQ3_SHIFT)
#define ADC_RSQ2_RSQ4_SHIFT          (20)      /* Bits 24-20: 5th conversion in regular sequence */
#define ADC_RSQ2_RSQ4_MASK           (0x1f << ADC_RSQ2_RSQ4_SHIFT)
#define ADC_RSQ2_RSQ5_SHIFT          (25)      /* Bits 29-25: 6th conversion in regular sequence */
#define ADC_RSQ2_RSQ5_MASK           (0x1f << ADC_RSQ2_RSQ5_SHIFT)
#define ADC_RSQ2_RESERVED            (0xc0000000)
#define ADC_RSQ2_FIRST               (1)
#define ADC_RSQ2_LAST                (6)
#define ADC_RSQ2_SQ_OFFSET           (0)

/* Offset between SQ bits in RSQx registers */

#define ADC_SQ_OFFSET                (5)

/* ADC inserted sequence register (ADC_ISQ) */

#define ADC_ISQ_ISQ0_SHIFT           (0)       /* Bits 4-0: 1st conversion in inserted sequence */
#define ADC_ISQ_ISQ0_MASK            (0x1f << ADC_ISQ_ISQ0_SHIFT)
#define ADC_ISQ_ISQ1_SHIFT           (5)       /* Bits 9-5: 2nd conversion in inserted sequence */
#define ADC_ISQ_ISQ1_MASK            (0x1f << ADC_ISQ_ISQ1_SHIFT)
#define ADC_ISQ_ISQ2_SHIFT           (10)      /* Bits 14-10: 3rd conversion in inserted sequence */
#define ADC_ISQ_ISQ2_MASK            (0x1f << ADC_ISQ_ISQ2_SHIFT)
#define ADC_ISQ_ISQ3_SHIFT           (15)      /* Bits 19-15: 4th conversion in inserted sequence */
#define ADC_ISQ_ISQ3_MASK            (0x1f << ADC_ISQ_ISQ3_SHIFT)
#define ADC_ISQ_ISQ_SHIFT            (5)       /* Shift between ISQx bits */
#define ADC_ISQ_IL_SHIFT             (20)      /* Bits 21-20: Inserted sequence length */
#define ADC_ISQ_IL_MASK              (3 << ADC_ISQ_IL_SHIFT)
#define ADC_ISQ_IL(n)                (((n) - 1) << ADC_ISQ_IL_SHIFT) /* n=1..4 */

/* ADC inserted data registers (ADC_IDATAx) */

#define ADC_IDATA_SHIFT              (0)       /* Bits 15-0: Inserted data */
#define ADC_IDATA_MASK               (0xffff << ADC_IDATA_SHIFT)

/* ADC regular data register (ADC_RDATA) */

#define ADC_RDATA_RDATA_SHIFT        (0)       /* Bits 15-0: Regular channel conversion data */
#define ADC_RDATA_RDATA_MASK         (0xffff << ADC_RDATA_RDATA_SHIFT)
#define ADC_RDATA_ADC1RDTR_SHIFT     (16)      /* Bits 31-16: ADC1 regular channel data (dual mode) */
#define ADC_RDATA_ADC1RDTR_MASK      (0xffff << ADC_RDATA_ADC1RDTR_SHIFT)

/* ADC oversampling control register (ADC_OVSAMPCTL) */

#define ADC_OVSAMPCTL_OVSEN          (1 << 0)  /* Bit 0: Oversampling enable */
#define ADC_OVSAMPCTL_OVSR_SHIFT     (2)       /* Bits 4-2: Oversampling ratio */
#define ADC_OVSAMPCTL_OVSR_MASK      (7 << ADC_OVSAMPCTL_OVSR_SHIFT)
#define ADC_OVSAMPCTL_OVSS_SHIFT     (5)       /* Bits 8-5: Oversampling shift */
#define ADC_OVSAMPCTL_OVSS_MASK      (0x0f << ADC_OVSAMPCTL_OVSS_SHIFT)
#define ADC_OVSAMPCTL_TOVS           (1 << 9)  /* Bit 9: Triggered oversampling */
#define ADC_OVSAMPCTL_DRES_SHIFT     (12)      /* Bits 13-12: ADC resolution */
#define ADC_OVSAMPCTL_DRES_MASK      (3 << ADC_OVSAMPCTL_DRES_SHIFT)

/* Oversampling ratio values */

#define ADC_OVSR_MUL2                (0 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x2 */
#define ADC_OVSR_MUL4                (1 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x4 */
#define ADC_OVSR_MUL8                (2 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x8 */
#define ADC_OVSR_MUL16               (3 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x16 */
#define ADC_OVSR_MUL32               (4 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x32 */
#define ADC_OVSR_MUL64               (5 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x64 */
#define ADC_OVSR_MUL128              (6 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x128 */
#define ADC_OVSR_MUL256              (7 << ADC_OVSAMPCTL_OVSR_SHIFT)  /* Ratio x256 */

/* Oversampling shift values */

#define ADC_OVSS_NONE                (0 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* No shift */
#define ADC_OVSS_1B                  (1 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 1 bit */
#define ADC_OVSS_2B                  (2 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 2 bits */
#define ADC_OVSS_3B                  (3 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 3 bits */
#define ADC_OVSS_4B                  (4 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 4 bits */
#define ADC_OVSS_5B                  (5 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 5 bits */
#define ADC_OVSS_6B                  (6 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 6 bits */
#define ADC_OVSS_7B                  (7 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 7 bits */
#define ADC_OVSS_8B                  (8 << ADC_OVSAMPCTL_OVSS_SHIFT)  /* Shift 8 bits */

/* ADC resolution values (OVSAMPCTL DRES bits 13-12) */

#define ADC_DRES_12BIT               (0 << ADC_OVSAMPCTL_DRES_SHIFT)  /* 12-bit resolution */
#define ADC_DRES_10BIT               (1 << ADC_OVSAMPCTL_DRES_SHIFT)  /* 10-bit resolution */
#define ADC_DRES_8BIT                (2 << ADC_OVSAMPCTL_DRES_SHIFT)  /* 8-bit resolution */
#define ADC_DRES_6BIT                (3 << ADC_OVSAMPCTL_DRES_SHIFT)  /* 6-bit resolution */

/* Generalized definitions for the GD32E11x ADC driver.
 * These aliases allow the driver code to use common names similar to the
 */

/* Status register aliases */

#define ADC_SR_AWD                   ADC_STAT_WDE   /* Analog watchdog flag */
#define ADC_SR_EOC                   ADC_STAT_EOC   /* End of conversion */
#define ADC_SR_JEOC                  ADC_STAT_EOIC  /* Inserted end of conversion */
#define ADC_SR_JSTRT                 ADC_STAT_STIC  /* Inserted channel start flag */
#define ADC_SR_STRT                  ADC_STAT_STRC  /* Regular channel start flag */

/* Register offset aliases (for use in gd32e11x_adc.c driver) */

#define GD32_ADC_SR_OFFSET           GD32_ADC_STAT_OFFSET
#define GD32_ADC_CR1_OFFSET          GD32_ADC_CTL0_OFFSET
#define GD32_ADC_CR2_OFFSET          GD32_ADC_CTL1_OFFSET
#define GD32_ADC_SMPR1_OFFSET        GD32_ADC_SAMPT0_OFFSET
#define GD32_ADC_SMPR2_OFFSET        GD32_ADC_SAMPT1_OFFSET
#define GD32_ADC_JOFR1_OFFSET        GD32_ADC_IOFF0_OFFSET
#define GD32_ADC_JOFR2_OFFSET        GD32_ADC_IOFF1_OFFSET
#define GD32_ADC_JOFR3_OFFSET        GD32_ADC_IOFF2_OFFSET
#define GD32_ADC_JOFR4_OFFSET        GD32_ADC_IOFF3_OFFSET
#define GD32_ADC_HTR_OFFSET          GD32_ADC_WDHT_OFFSET
#define GD32_ADC_LTR_OFFSET          GD32_ADC_WDLT_OFFSET
#define GD32_ADC_SQR1_OFFSET         GD32_ADC_RSQ0_OFFSET
#define GD32_ADC_SQR2_OFFSET         GD32_ADC_RSQ1_OFFSET
#define GD32_ADC_SQR3_OFFSET         GD32_ADC_RSQ2_OFFSET
#define GD32_ADC_JSQR_OFFSET         GD32_ADC_ISQ_OFFSET
#define GD32_ADC_JDR1_OFFSET         GD32_ADC_IDATA0_OFFSET
#define GD32_ADC_JDR2_OFFSET         GD32_ADC_IDATA1_OFFSET
#define GD32_ADC_JDR3_OFFSET         GD32_ADC_IDATA2_OFFSET
#define GD32_ADC_JDR4_OFFSET         GD32_ADC_IDATA3_OFFSET
#define GD32_ADC_DR_OFFSET           GD32_ADC_RDATA_OFFSET

/* CR1 (CTL0) register bit aliases */

#define ADC_CR1_AWDCH_SHIFT          ADC_CTL0_WDCHSEL_SHIFT
#define ADC_CR1_AWDCH_MASK           ADC_CTL0_WDCHSEL_MASK
#define ADC_CR1_EOCIE                ADC_CTL0_EOCIE
#define ADC_CR1_AWDIE                ADC_CTL0_WDEIE
#define ADC_CR1_JEOCIE               ADC_CTL0_EOICIE
#define ADC_CR1_SCAN                 ADC_CTL0_SM
#define ADC_CR1_AWDSGL               ADC_CTL0_WDSC
#define ADC_CR1_JAUTO                ADC_CTL0_ICA
#define ADC_CR1_DISCEN               ADC_CTL0_DISRC
#define ADC_CR1_JDISCEN              ADC_CTL0_DISIC
#define ADC_CR1_DISCNUM_SHIFT        ADC_CTL0_DISNUM_SHIFT
#define ADC_CR1_DISCNUM_MASK         ADC_CTL0_DISNUM_MASK
#define ADC_CR1_DUALMOD_SHIFT        ADC_CTL0_SYNCM_SHIFT
#define ADC_CR1_DUALMOD_MASK         ADC_CTL0_SYNCM_MASK
#define ADC_CR1_IND                  ADC_CTL0_SYNCM_FREE
#define ADC_CR1_JAWDEN               ADC_CTL0_IWDEN
#define ADC_CR1_AWDEN                ADC_CTL0_RWDEN

/* CR2 (CTL1) register bit aliases */

#define ADC_CR2_ADON                 ADC_CTL1_ADCON
#define ADC_CR2_CONT                 ADC_CTL1_CTN
#define ADC_CR2_CAL                  ADC_CTL1_CLB
#define ADC_CR2_RSTCAL               ADC_CTL1_RSTCLB
#define ADC_CR2_DMA                  ADC_CTL1_DMA
#define ADC_CR2_ALIGN                ADC_CTL1_DAL
#define ADC_CR2_JEXTSEL_SHIFT        ADC_CTL1_ETSIC_SHIFT
#define ADC_CR2_JEXTSEL_MASK         ADC_CTL1_ETSIC_MASK
#define ADC_CR2_JEXTTRIG             ADC_CTL1_ETEIC
#define ADC_CR2_EXTSEL_SHIFT         ADC_CTL1_ETSRC_SHIFT
#define ADC_CR2_EXTSEL_MASK          ADC_CTL1_ETSRC_MASK
#define ADC_CR2_EXTTRIG              ADC_CTL1_ETERC
#define ADC_CR2_JSWSTART             ADC_CTL1_SWICST
#define ADC_CR2_SWSTART              ADC_CTL1_SWRCST
#define ADC_CR2_TSVREFE              ADC_CTL1_TSVREN

/* SMPR1 (SAMPT0) aliases */

#define ADC_SMPR1_SMP10_SHIFT        ADC_SAMPT0_SMP10_SHIFT
#define ADC_SMPR1_SMP10_MASK         ADC_SAMPT0_SMP10_MASK
#define ADC_SMPR1_SMP11_SHIFT        ADC_SAMPT0_SMP11_SHIFT
#define ADC_SMPR1_SMP11_MASK         ADC_SAMPT0_SMP11_MASK
#define ADC_SMPR1_SMP12_SHIFT        ADC_SAMPT0_SMP12_SHIFT
#define ADC_SMPR1_SMP12_MASK         ADC_SAMPT0_SMP12_MASK
#define ADC_SMPR1_SMP13_SHIFT        ADC_SAMPT0_SMP13_SHIFT
#define ADC_SMPR1_SMP13_MASK         ADC_SAMPT0_SMP13_MASK
#define ADC_SMPR1_SMP14_SHIFT        ADC_SAMPT0_SMP14_SHIFT
#define ADC_SMPR1_SMP14_MASK         ADC_SAMPT0_SMP14_MASK
#define ADC_SMPR1_SMP15_SHIFT        ADC_SAMPT0_SMP15_SHIFT
#define ADC_SMPR1_SMP15_MASK         ADC_SAMPT0_SMP15_MASK
#define ADC_SMPR1_SMP16_SHIFT        ADC_SAMPT0_SMP16_SHIFT
#define ADC_SMPR1_SMP16_MASK         ADC_SAMPT0_SMP16_MASK
#define ADC_SMPR1_SMP17_SHIFT        ADC_SAMPT0_SMP17_SHIFT
#define ADC_SMPR1_SMP17_MASK         ADC_SAMPT0_SMP17_MASK

/* SMPR2 (SAMPT1) aliases */

#define ADC_SMPR2_SMP0_SHIFT         ADC_SAMPT1_SMP0_SHIFT
#define ADC_SMPR2_SMP0_MASK          ADC_SAMPT1_SMP0_MASK
#define ADC_SMPR2_SMP1_SHIFT         ADC_SAMPT1_SMP1_SHIFT
#define ADC_SMPR2_SMP1_MASK          ADC_SAMPT1_SMP1_MASK
#define ADC_SMPR2_SMP2_SHIFT         ADC_SAMPT1_SMP2_SHIFT
#define ADC_SMPR2_SMP2_MASK          ADC_SAMPT1_SMP2_MASK
#define ADC_SMPR2_SMP3_SHIFT         ADC_SAMPT1_SMP3_SHIFT
#define ADC_SMPR2_SMP3_MASK          ADC_SAMPT1_SMP3_MASK
#define ADC_SMPR2_SMP4_SHIFT         ADC_SAMPT1_SMP4_SHIFT
#define ADC_SMPR2_SMP4_MASK          ADC_SAMPT1_SMP4_MASK
#define ADC_SMPR2_SMP5_SHIFT         ADC_SAMPT1_SMP5_SHIFT
#define ADC_SMPR2_SMP5_MASK          ADC_SAMPT1_SMP5_MASK
#define ADC_SMPR2_SMP6_SHIFT         ADC_SAMPT1_SMP6_SHIFT
#define ADC_SMPR2_SMP6_MASK          ADC_SAMPT1_SMP6_MASK
#define ADC_SMPR2_SMP7_SHIFT         ADC_SAMPT1_SMP7_SHIFT
#define ADC_SMPR2_SMP7_MASK          ADC_SAMPT1_SMP7_MASK
#define ADC_SMPR2_SMP8_SHIFT         ADC_SAMPT1_SMP8_SHIFT
#define ADC_SMPR2_SMP8_MASK          ADC_SAMPT1_SMP8_MASK
#define ADC_SMPR2_SMP9_SHIFT         ADC_SAMPT1_SMP9_SHIFT
#define ADC_SMPR2_SMP9_MASK          ADC_SAMPT1_SMP9_MASK

/* SQR1 (RSQ0) aliases */

#define ADC_SQR1_SQ13_SHIFT          ADC_RSQ0_RSQ12_SHIFT
#define ADC_SQR1_SQ13_MASK           ADC_RSQ0_RSQ12_MASK
#define ADC_SQR1_SQ14_SHIFT          ADC_RSQ0_RSQ13_SHIFT
#define ADC_SQR1_SQ14_MASK           ADC_RSQ0_RSQ13_MASK
#define ADC_SQR1_SQ15_SHIFT          ADC_RSQ0_RSQ14_SHIFT
#define ADC_SQR1_SQ15_MASK           ADC_RSQ0_RSQ14_MASK
#define ADC_SQR1_SQ16_SHIFT          ADC_RSQ0_RSQ15_SHIFT
#define ADC_SQR1_SQ16_MASK           ADC_RSQ0_RSQ15_MASK
#define ADC_SQR1_L_SHIFT             ADC_RSQ0_RL_SHIFT
#define ADC_SQR1_L_MASK              ADC_RSQ0_RL_MASK
#define ADC_SQR1_RESERVED            ADC_RSQ0_RESERVED
#define ADC_SQR1_FIRST               ADC_RSQ0_FIRST
#define ADC_SQR1_LAST                ADC_RSQ0_LAST
#define ADC_SQR1_SQ_OFFSET           ADC_RSQ0_SQ_OFFSET

/* SQR2 (RSQ1) aliases */

#define ADC_SQR2_SQ7_SHIFT           ADC_RSQ1_RSQ6_SHIFT
#define ADC_SQR2_SQ7_MASK            ADC_RSQ1_RSQ6_MASK
#define ADC_SQR2_SQ8_SHIFT           ADC_RSQ1_RSQ7_SHIFT
#define ADC_SQR2_SQ8_MASK            ADC_RSQ1_RSQ7_MASK
#define ADC_SQR2_SQ9_SHIFT           ADC_RSQ1_RSQ8_SHIFT
#define ADC_SQR2_SQ9_MASK            ADC_RSQ1_RSQ8_MASK
#define ADC_SQR2_SQ10_SHIFT          ADC_RSQ1_RSQ9_SHIFT
#define ADC_SQR2_SQ10_MASK           ADC_RSQ1_RSQ9_MASK
#define ADC_SQR2_SQ11_SHIFT          ADC_RSQ1_RSQ10_SHIFT
#define ADC_SQR2_SQ11_MASK           ADC_RSQ1_RSQ10_MASK
#define ADC_SQR2_SQ12_SHIFT          ADC_RSQ1_RSQ11_SHIFT
#define ADC_SQR2_SQ12_MASK           ADC_RSQ1_RSQ11_MASK
#define ADC_SQR2_RESERVED            ADC_RSQ1_RESERVED
#define ADC_SQR2_FIRST               ADC_RSQ1_FIRST
#define ADC_SQR2_LAST                ADC_RSQ1_LAST
#define ADC_SQR2_SQ_OFFSET           ADC_RSQ1_SQ_OFFSET

/* SQR3 (RSQ2) aliases */

#define ADC_SQR3_SQ1_SHIFT           ADC_RSQ2_RSQ0_SHIFT
#define ADC_SQR3_SQ1_MASK            ADC_RSQ2_RSQ0_MASK
#define ADC_SQR3_SQ2_SHIFT           ADC_RSQ2_RSQ1_SHIFT
#define ADC_SQR3_SQ2_MASK            ADC_RSQ2_RSQ1_MASK
#define ADC_SQR3_SQ3_SHIFT           ADC_RSQ2_RSQ2_SHIFT
#define ADC_SQR3_SQ3_MASK            ADC_RSQ2_RSQ2_MASK
#define ADC_SQR3_SQ4_SHIFT           ADC_RSQ2_RSQ3_SHIFT
#define ADC_SQR3_SQ4_MASK            ADC_RSQ2_RSQ3_MASK
#define ADC_SQR3_SQ5_SHIFT           ADC_RSQ2_RSQ4_SHIFT
#define ADC_SQR3_SQ5_MASK            ADC_RSQ2_RSQ4_MASK
#define ADC_SQR3_SQ6_SHIFT           ADC_RSQ2_RSQ5_SHIFT
#define ADC_SQR3_SQ6_MASK            ADC_RSQ2_RSQ5_MASK
#define ADC_SQR3_RESERVED            ADC_RSQ2_RESERVED
#define ADC_SQR3_FIRST               ADC_RSQ2_FIRST
#define ADC_SQR3_LAST                ADC_RSQ2_LAST
#define ADC_SQR3_SQ_OFFSET           ADC_RSQ2_SQ_OFFSET

/* JSQR (ISQ) aliases */

#define ADC_JSQR_JSQ1_SHIFT          ADC_ISQ_ISQ0_SHIFT
#define ADC_JSQR_JSQ1_MASK           ADC_ISQ_ISQ0_MASK
#define ADC_JSQR_JSQ2_SHIFT          ADC_ISQ_ISQ1_SHIFT
#define ADC_JSQR_JSQ2_MASK           ADC_ISQ_ISQ1_MASK
#define ADC_JSQR_JSQ3_SHIFT          ADC_ISQ_ISQ2_SHIFT
#define ADC_JSQR_JSQ3_MASK           ADC_ISQ_ISQ2_MASK
#define ADC_JSQR_JSQ4_SHIFT          ADC_ISQ_ISQ3_SHIFT
#define ADC_JSQR_JSQ4_MASK           ADC_ISQ_ISQ3_MASK
#define ADC_JSQR_JSQ_SHIFT           ADC_ISQ_ISQ_SHIFT
#define ADC_JSQR_JL_SHIFT            ADC_ISQ_IL_SHIFT
#define ADC_JSQR_JL_MASK             ADC_ISQ_IL_MASK
#define ADC_JSQR_JL(n)               ADC_ISQ_IL(n)

/* DR (RDATA) aliases */

#define ADC_DR_RDATA_SHIFT           ADC_RDATA_RDATA_SHIFT
#define ADC_DR_RDATA_MASK            ADC_RDATA_RDATA_MASK
#define ADC_DR_ADC2DATA_SHIFT        ADC_RDATA_ADC1RDTR_SHIFT
#define ADC_DR_ADC2DATA_MASK         ADC_RDATA_ADC1RDTR_MASK

/* GD32E11x ADC channel count: 18 channels (0..17) per ADC unit */

#define GD32_ADC_CHANNELS_NUMBER     18

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_ADC_H */
