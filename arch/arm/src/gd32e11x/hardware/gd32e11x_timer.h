/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_timer.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_TIMER_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIMERx(x=0..13) base addresses */

#define GD32_TIMER0_BASE     (GD32_TIMER_BASE + 0x00012C00U)     /* TIMER0 base address (APB2) */
#define GD32_TIMER1_BASE     (GD32_TIMER_BASE + 0x00000000U)     /* TIMER1 base address (APB1) */
#define GD32_TIMER2_BASE     (GD32_TIMER_BASE + 0x00000400U)     /* TIMER2 base address (APB1) */
#define GD32_TIMER3_BASE     (GD32_TIMER_BASE + 0x00000800U)     /* TIMER3 base address (APB1) */
#define GD32_TIMER4_BASE     (GD32_TIMER_BASE + 0x00000C00U)     /* TIMER4 base address (APB1) */
#define GD32_TIMER5_BASE     (GD32_TIMER_BASE + 0x00001000U)     /* TIMER5 base address (APB1) */
#define GD32_TIMER6_BASE     (GD32_TIMER_BASE + 0x00001400U)     /* TIMER6 base address (APB1) */
#define GD32_TIMER7_BASE     (GD32_TIMER_BASE + 0x00013400U)     /* TIMER7 base address (APB2) */
#define GD32_TIMER8_BASE     (GD32_TIMER_BASE + 0x00014C00U)     /* TIMER8 base address (APB2) */
#define GD32_TIMER9_BASE     (GD32_TIMER_BASE + 0x00015000U)     /* TIMER9 base address (APB2) */
#define GD32_TIMER10_BASE    (GD32_TIMER_BASE + 0x00015400U)     /* TIMER10 base address (APB2) */
#define GD32_TIMER11_BASE    (GD32_TIMER_BASE + 0x00001800U)     /* TIMER11 base address (APB1) */
#define GD32_TIMER12_BASE    (GD32_TIMER_BASE + 0x00001C00U)     /* TIMER12 base address (APB1) */
#define GD32_TIMER13_BASE    (GD32_TIMER_BASE + 0x00002000U)     /* TIMER13 base address (APB1) */

/* Register Offsets *********************************************************/

#define GD32_TIMER_CTL0_OFFSET        0x0000  /* TIMER control register 0 */
#define GD32_TIMER_CTL1_OFFSET        0x0004  /* TIMER control register 1 */
#define GD32_TIMER_SMCFG_OFFSET       0x0008  /* TIMER slave mode configuration register */
#define GD32_TIMER_DMAINTEN_OFFSET    0x000c  /* TIMER DMA and interrupt enable register */
#define GD32_TIMER_INTF_OFFSET        0x0010  /* TIMER interrupt flag register */
#define GD32_TIMER_SWEVG_OFFSET       0x0014  /* TIMER software event generation register */
#define GD32_TIMER_CHCTL0_OFFSET      0x0018  /* TIMER channel control register 0 */
#define GD32_TIMER_CHCTL1_OFFSET      0x001c  /* TIMER channel control register 1 */
#define GD32_TIMER_CHCTL2_OFFSET      0x0020  /* TIMER channel control register 2 */
#define GD32_TIMER_CNT_OFFSET         0x0024  /* TIMER counter register */
#define GD32_TIMER_PSC_OFFSET         0x0028  /* TIMER prescaler register */
#define GD32_TIMER_CAR_OFFSET         0x002c  /* TIMER counter auto reload register */
#define GD32_TIMER_CREP_OFFSET        0x0030  /* TIMER counter repetition register */
#define GD32_TIMER_CH0CV_OFFSET       0x0034  /* TIMER channel 0 capture/compare value register */
#define GD32_TIMER_CH1CV_OFFSET       0x0038  /* TIMER channel 1 capture/compare value register */
#define GD32_TIMER_CH2CV_OFFSET       0x003c  /* TIMER channel 2 capture/compare value register */
#define GD32_TIMER_CH3CV_OFFSET       0x0040  /* TIMER channel 3 capture/compare value register */
#define GD32_TIMER_CCHP_OFFSET        0x0044  /* TIMER channel complementary protection register */
#define GD32_TIMER_DMACFG_OFFSET      0x0048  /* TIMER DMA configuration register */
#define GD32_TIMER_DMATB_OFFSET       0x004c  /* TIMER DMA transfer buffer register */
#define GD32_TIMER_CFG_OFFSET         0x00fc  /* TIMER configuration register */

/* Register Addresses *******************************************************/

/* TIMER0 - Advanced Timer */

#if defined(CONFIG_GD32E11X_TIMER0)
#define GD32_TIMER0_CTL0              (GD32_TIMER0_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER0_CTL1              (GD32_TIMER0_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER0_SMCFG             (GD32_TIMER0_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER0_DMAINTEN          (GD32_TIMER0_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER0_INTF              (GD32_TIMER0_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER0_SWEVG             (GD32_TIMER0_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER0_CHCTL0            (GD32_TIMER0_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER0_CHCTL1            (GD32_TIMER0_BASE + GD32_TIMER_CHCTL1_OFFSET)
#define GD32_TIMER0_CHCTL2            (GD32_TIMER0_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER0_CNT               (GD32_TIMER0_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER0_PSC               (GD32_TIMER0_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER0_CAR               (GD32_TIMER0_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER0_CREP              (GD32_TIMER0_BASE + GD32_TIMER_CREP_OFFSET)
#define GD32_TIMER0_CH0CV             (GD32_TIMER0_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER0_CH1CV             (GD32_TIMER0_BASE + GD32_TIMER_CH1CV_OFFSET)
#define GD32_TIMER0_CH2CV             (GD32_TIMER0_BASE + GD32_TIMER_CH2CV_OFFSET)
#define GD32_TIMER0_CH3CV             (GD32_TIMER0_BASE + GD32_TIMER_CH3CV_OFFSET)
#define GD32_TIMER0_CCHP              (GD32_TIMER0_BASE + GD32_TIMER_CCHP_OFFSET)
#define GD32_TIMER0_DMACFG            (GD32_TIMER0_BASE + GD32_TIMER_DMACFG_OFFSET)
#define GD32_TIMER0_DMATB             (GD32_TIMER0_BASE + GD32_TIMER_DMATB_OFFSET)
#define GD32_TIMER0_CFG               (GD32_TIMER0_BASE + GD32_TIMER_CFG_OFFSET)
#endif

/* TIMER1 - General Timer */

#if defined(CONFIG_GD32E11X_TIMER1)
#define GD32_TIMER1_CTL0              (GD32_TIMER1_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER1_CTL1              (GD32_TIMER1_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER1_SMCFG             (GD32_TIMER1_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER1_DMAINTEN          (GD32_TIMER1_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER1_INTF              (GD32_TIMER1_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER1_SWEVG             (GD32_TIMER1_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER1_CHCTL0            (GD32_TIMER1_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER1_CHCTL1            (GD32_TIMER1_BASE + GD32_TIMER_CHCTL1_OFFSET)
#define GD32_TIMER1_CHCTL2            (GD32_TIMER1_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER1_CNT               (GD32_TIMER1_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER1_PSC               (GD32_TIMER1_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER1_CAR               (GD32_TIMER1_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER1_CH0CV             (GD32_TIMER1_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER1_CH1CV             (GD32_TIMER1_BASE + GD32_TIMER_CH1CV_OFFSET)
#define GD32_TIMER1_CH2CV             (GD32_TIMER1_BASE + GD32_TIMER_CH2CV_OFFSET)
#define GD32_TIMER1_CH3CV             (GD32_TIMER1_BASE + GD32_TIMER_CH3CV_OFFSET)
#define GD32_TIMER1_DMACFG            (GD32_TIMER1_BASE + GD32_TIMER_DMACFG_OFFSET)
#define GD32_TIMER1_DMATB             (GD32_TIMER1_BASE + GD32_TIMER_DMATB_OFFSET)
#define GD32_TIMER1_CFG               (GD32_TIMER1_BASE + GD32_TIMER_CFG_OFFSET)
#endif

/* TIMER2 - General Timer */

#if defined(CONFIG_GD32E11X_TIMER2)
#define GD32_TIMER2_CTL0              (GD32_TIMER2_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER2_CTL1              (GD32_TIMER2_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER2_SMCFG             (GD32_TIMER2_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER2_DMAINTEN          (GD32_TIMER2_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER2_INTF              (GD32_TIMER2_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER2_SWEVG             (GD32_TIMER2_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER2_CHCTL0            (GD32_TIMER2_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER2_CHCTL1            (GD32_TIMER2_BASE + GD32_TIMER_CHCTL1_OFFSET)
#define GD32_TIMER2_CHCTL2            (GD32_TIMER2_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER2_CNT               (GD32_TIMER2_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER2_PSC               (GD32_TIMER2_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER2_CAR               (GD32_TIMER2_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER2_CH0CV             (GD32_TIMER2_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER2_CH1CV             (GD32_TIMER2_BASE + GD32_TIMER_CH1CV_OFFSET)
#define GD32_TIMER2_CH2CV             (GD32_TIMER2_BASE + GD32_TIMER_CH2CV_OFFSET)
#define GD32_TIMER2_CH3CV             (GD32_TIMER2_BASE + GD32_TIMER_CH3CV_OFFSET)
#define GD32_TIMER2_DMACFG            (GD32_TIMER2_BASE + GD32_TIMER_DMACFG_OFFSET)
#define GD32_TIMER2_DMATB             (GD32_TIMER2_BASE + GD32_TIMER_DMATB_OFFSET)
#define GD32_TIMER2_CFG               (GD32_TIMER2_BASE + GD32_TIMER_CFG_OFFSET)
#endif

/* TIMER3 - General Timer */

#if defined(CONFIG_GD32E11X_TIMER3)
#define GD32_TIMER3_CTL0              (GD32_TIMER3_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER3_CTL1              (GD32_TIMER3_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER3_SMCFG             (GD32_TIMER3_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER3_DMAINTEN          (GD32_TIMER3_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER3_INTF              (GD32_TIMER3_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER3_SWEVG             (GD32_TIMER3_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER3_CHCTL0            (GD32_TIMER3_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER3_CHCTL1            (GD32_TIMER3_BASE + GD32_TIMER_CHCTL1_OFFSET)
#define GD32_TIMER3_CHCTL2            (GD32_TIMER3_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER3_CNT               (GD32_TIMER3_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER3_PSC               (GD32_TIMER3_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER3_CAR               (GD32_TIMER3_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER3_CH0CV             (GD32_TIMER3_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER3_CH1CV             (GD32_TIMER3_BASE + GD32_TIMER_CH1CV_OFFSET)
#define GD32_TIMER3_CH2CV             (GD32_TIMER3_BASE + GD32_TIMER_CH2CV_OFFSET)
#define GD32_TIMER3_CH3CV             (GD32_TIMER3_BASE + GD32_TIMER_CH3CV_OFFSET)
#define GD32_TIMER3_DMACFG            (GD32_TIMER3_BASE + GD32_TIMER_DMACFG_OFFSET)
#define GD32_TIMER3_DMATB             (GD32_TIMER3_BASE + GD32_TIMER_DMATB_OFFSET)
#define GD32_TIMER3_CFG               (GD32_TIMER3_BASE + GD32_TIMER_CFG_OFFSET)
#endif

/* TIMER4 - General Timer */

#if defined(CONFIG_GD32E11X_TIMER4)
#define GD32_TIMER4_CTL0              (GD32_TIMER4_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER4_CTL1              (GD32_TIMER4_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER4_SMCFG             (GD32_TIMER4_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER4_DMAINTEN          (GD32_TIMER4_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER4_INTF              (GD32_TIMER4_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER4_SWEVG             (GD32_TIMER4_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER4_CHCTL0            (GD32_TIMER4_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER4_CHCTL1            (GD32_TIMER4_BASE + GD32_TIMER_CHCTL1_OFFSET)
#define GD32_TIMER4_CHCTL2            (GD32_TIMER4_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER4_CNT               (GD32_TIMER4_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER4_PSC               (GD32_TIMER4_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER4_CAR               (GD32_TIMER4_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER4_CH0CV             (GD32_TIMER4_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER4_CH1CV             (GD32_TIMER4_BASE + GD32_TIMER_CH1CV_OFFSET)
#define GD32_TIMER4_CH2CV             (GD32_TIMER4_BASE + GD32_TIMER_CH2CV_OFFSET)
#define GD32_TIMER4_CH3CV             (GD32_TIMER4_BASE + GD32_TIMER_CH3CV_OFFSET)
#define GD32_TIMER4_DMACFG            (GD32_TIMER4_BASE + GD32_TIMER_DMACFG_OFFSET)
#define GD32_TIMER4_DMATB             (GD32_TIMER4_BASE + GD32_TIMER_DMATB_OFFSET)
#define GD32_TIMER4_CFG               (GD32_TIMER4_BASE + GD32_TIMER_CFG_OFFSET)
#endif

/* TIMER5 - Basic Timer */

#if defined(CONFIG_GD32E11X_TIMER5)
#define GD32_TIMER5_CTL0              (GD32_TIMER5_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER5_CTL1              (GD32_TIMER5_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER5_DMAINTEN          (GD32_TIMER5_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER5_INTF              (GD32_TIMER5_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER5_SWEVG             (GD32_TIMER5_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER5_CNT               (GD32_TIMER5_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER5_PSC               (GD32_TIMER5_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER5_CAR               (GD32_TIMER5_BASE + GD32_TIMER_CAR_OFFSET)
#endif

/* TIMER6 - Basic Timer */

#if defined(CONFIG_GD32E11X_TIMER6)
#define GD32_TIMER6_CTL0              (GD32_TIMER6_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER6_CTL1              (GD32_TIMER6_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER6_DMAINTEN          (GD32_TIMER6_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER6_INTF              (GD32_TIMER6_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER6_SWEVG             (GD32_TIMER6_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER6_CNT               (GD32_TIMER6_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER6_PSC               (GD32_TIMER6_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER6_CAR               (GD32_TIMER6_BASE + GD32_TIMER_CAR_OFFSET)
#endif

/* TIMER7 - Advanced Timer */

#if defined(CONFIG_GD32E11X_TIMER7)
#define GD32_TIMER7_CTL0              (GD32_TIMER7_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER7_CTL1              (GD32_TIMER7_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER7_SMCFG             (GD32_TIMER7_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER7_DMAINTEN          (GD32_TIMER7_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER7_INTF              (GD32_TIMER7_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER7_SWEVG             (GD32_TIMER7_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER7_CHCTL0            (GD32_TIMER7_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER7_CHCTL1            (GD32_TIMER7_BASE + GD32_TIMER_CHCTL1_OFFSET)
#define GD32_TIMER7_CHCTL2            (GD32_TIMER7_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER7_CNT               (GD32_TIMER7_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER7_PSC               (GD32_TIMER7_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER7_CAR               (GD32_TIMER7_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER7_CREP              (GD32_TIMER7_BASE + GD32_TIMER_CREP_OFFSET)
#define GD32_TIMER7_CH0CV             (GD32_TIMER7_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER7_CH1CV             (GD32_TIMER7_BASE + GD32_TIMER_CH1CV_OFFSET)
#define GD32_TIMER7_CH2CV             (GD32_TIMER7_BASE + GD32_TIMER_CH2CV_OFFSET)
#define GD32_TIMER7_CH3CV             (GD32_TIMER7_BASE + GD32_TIMER_CH3CV_OFFSET)
#define GD32_TIMER7_CCHP              (GD32_TIMER7_BASE + GD32_TIMER_CCHP_OFFSET)
#define GD32_TIMER7_DMACFG            (GD32_TIMER7_BASE + GD32_TIMER_DMACFG_OFFSET)
#define GD32_TIMER7_DMATB             (GD32_TIMER7_BASE + GD32_TIMER_DMATB_OFFSET)
#define GD32_TIMER7_CFG               (GD32_TIMER7_BASE + GD32_TIMER_CFG_OFFSET)
#endif

/* TIMER8 - General Timer (2 channels) */

#if defined(CONFIG_GD32E11X_TIMER8)
#define GD32_TIMER8_CTL0              (GD32_TIMER8_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER8_CTL1              (GD32_TIMER8_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER8_SMCFG             (GD32_TIMER8_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER8_DMAINTEN          (GD32_TIMER8_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER8_INTF              (GD32_TIMER8_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER8_SWEVG             (GD32_TIMER8_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER8_CHCTL0            (GD32_TIMER8_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER8_CHCTL2            (GD32_TIMER8_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER8_CNT               (GD32_TIMER8_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER8_PSC               (GD32_TIMER8_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER8_CAR               (GD32_TIMER8_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER8_CH0CV             (GD32_TIMER8_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER8_CH1CV             (GD32_TIMER8_BASE + GD32_TIMER_CH1CV_OFFSET)
#endif

/* TIMER9 - General Timer (1 channel) */

#if defined(CONFIG_GD32E11X_TIMER9)
#define GD32_TIMER9_CTL0              (GD32_TIMER9_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER9_CTL1              (GD32_TIMER9_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER9_SMCFG             (GD32_TIMER9_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER9_DMAINTEN          (GD32_TIMER9_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER9_INTF              (GD32_TIMER9_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER9_SWEVG             (GD32_TIMER9_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER9_CHCTL0            (GD32_TIMER9_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER9_CHCTL2            (GD32_TIMER9_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER9_CNT               (GD32_TIMER9_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER9_PSC               (GD32_TIMER9_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER9_CAR               (GD32_TIMER9_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER9_CH0CV             (GD32_TIMER9_BASE + GD32_TIMER_CH0CV_OFFSET)
#endif

/* TIMER10 - General Timer (1 channel) */

#if defined(CONFIG_GD32E11X_TIMER10)
#define GD32_TIMER10_CTL0             (GD32_TIMER10_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER10_CTL1             (GD32_TIMER10_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER10_SMCFG            (GD32_TIMER10_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER10_DMAINTEN         (GD32_TIMER10_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER10_INTF             (GD32_TIMER10_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER10_SWEVG            (GD32_TIMER10_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER10_CHCTL0           (GD32_TIMER10_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER10_CHCTL2           (GD32_TIMER10_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER10_CNT              (GD32_TIMER10_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER10_PSC              (GD32_TIMER10_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER10_CAR              (GD32_TIMER10_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER10_CH0CV            (GD32_TIMER10_BASE + GD32_TIMER_CH0CV_OFFSET)
#endif

/* TIMER11 - General Timer (2 channels) */

#if defined(CONFIG_GD32E11X_TIMER11)
#define GD32_TIMER11_CTL0             (GD32_TIMER11_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER11_CTL1             (GD32_TIMER11_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER11_SMCFG            (GD32_TIMER11_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER11_DMAINTEN         (GD32_TIMER11_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER11_INTF             (GD32_TIMER11_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER11_SWEVG            (GD32_TIMER11_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER11_CHCTL0           (GD32_TIMER11_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER11_CHCTL2           (GD32_TIMER11_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER11_CNT              (GD32_TIMER11_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER11_PSC              (GD32_TIMER11_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER11_CAR              (GD32_TIMER11_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER11_CH0CV            (GD32_TIMER11_BASE + GD32_TIMER_CH0CV_OFFSET)
#define GD32_TIMER11_CH1CV            (GD32_TIMER11_BASE + GD32_TIMER_CH1CV_OFFSET)
#endif

/* TIMER12 - General Timer (1 channel) */

#if defined(CONFIG_GD32E11X_TIMER12)
#define GD32_TIMER12_CTL0             (GD32_TIMER12_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER12_CTL1             (GD32_TIMER12_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER12_SMCFG            (GD32_TIMER12_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER12_DMAINTEN         (GD32_TIMER12_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER12_INTF             (GD32_TIMER12_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER12_SWEVG            (GD32_TIMER12_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER12_CHCTL0           (GD32_TIMER12_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER12_CHCTL2           (GD32_TIMER12_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER12_CNT              (GD32_TIMER12_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER12_PSC              (GD32_TIMER12_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER12_CAR              (GD32_TIMER12_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER12_CH0CV            (GD32_TIMER12_BASE + GD32_TIMER_CH0CV_OFFSET)
#endif

/* TIMER13 - General Timer (1 channel) */

#if defined(CONFIG_GD32E11X_TIMER13)
#define GD32_TIMER13_CTL0             (GD32_TIMER13_BASE + GD32_TIMER_CTL0_OFFSET)
#define GD32_TIMER13_CTL1             (GD32_TIMER13_BASE + GD32_TIMER_CTL1_OFFSET)
#define GD32_TIMER13_SMCFG            (GD32_TIMER13_BASE + GD32_TIMER_SMCFG_OFFSET)
#define GD32_TIMER13_DMAINTEN         (GD32_TIMER13_BASE + GD32_TIMER_DMAINTEN_OFFSET)
#define GD32_TIMER13_INTF             (GD32_TIMER13_BASE + GD32_TIMER_INTF_OFFSET)
#define GD32_TIMER13_SWEVG            (GD32_TIMER13_BASE + GD32_TIMER_SWEVG_OFFSET)
#define GD32_TIMER13_CHCTL0           (GD32_TIMER13_BASE + GD32_TIMER_CHCTL0_OFFSET)
#define GD32_TIMER13_CHCTL2           (GD32_TIMER13_BASE + GD32_TIMER_CHCTL2_OFFSET)
#define GD32_TIMER13_CNT              (GD32_TIMER13_BASE + GD32_TIMER_CNT_OFFSET)
#define GD32_TIMER13_PSC              (GD32_TIMER13_BASE + GD32_TIMER_PSC_OFFSET)
#define GD32_TIMER13_CAR              (GD32_TIMER13_BASE + GD32_TIMER_CAR_OFFSET)
#define GD32_TIMER13_CH0CV            (GD32_TIMER13_BASE + GD32_TIMER_CH0CV_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Control register 0 (TIMER_CTL0) */

#define TIMER_CTL0_CEN                (1 << 0)   /* Bit 0:  Counter enable */
#define TIMER_CTL0_UPDIS              (1 << 1)   /* Bit 1:  Update disable */
#define TIMER_CTL0_UPS                (1 << 2)   /* Bit 2:  Update source */
#define TIMER_CTL0_SPM                (1 << 3)   /* Bit 3:  Single pulse mode */
#define TIMER_CTL0_DIR                (1 << 4)   /* Bit 4:  Timer counter direction */
#define TIMER_CTL0_CAM_SHIFT          (5)        /* Bits 5-6: Center-aligned mode selection */
#define TIMER_CTL0_CAM_MASK           (3 << TIMER_CTL0_CAM_SHIFT)
#  define TIMER_CTL0_CAM_EDGE         (0 << TIMER_CTL0_CAM_SHIFT)  /* Edge-aligned mode */
#  define TIMER_CTL0_CAM_CENTER_DOWN  (1 << TIMER_CTL0_CAM_SHIFT)  /* Center-aligned counting down */
#  define TIMER_CTL0_CAM_CENTER_UP    (2 << TIMER_CTL0_CAM_SHIFT)  /* Center-aligned counting up */
#  define TIMER_CTL0_CAM_CENTER_BOTH  (3 << TIMER_CTL0_CAM_SHIFT)  /* Center-aligned counting both */
#define TIMER_CTL0_ARSE               (1 << 7)                     /* Bit 7:  Auto-reload shadow enable */
#define TIMER_CTL0_CKDIV_SHIFT        (8)                          /* Bits 8-9: Clock division */
#define TIMER_CTL0_CKDIV_MASK         (3 << TIMER_CTL0_CKDIV_SHIFT)
#  define TIMER_CTL0_CKDIV_DIV1       (0 << TIMER_CTL0_CKDIV_SHIFT)  /* fDTS = fTIMER_CK */
#  define TIMER_CTL0_CKDIV_DIV2       (1 << TIMER_CTL0_CKDIV_SHIFT)  /* fDTS = fTIMER_CK/2 */
#  define TIMER_CTL0_CKDIV_DIV4       (2 << TIMER_CTL0_CKDIV_SHIFT)  /* fDTS = fTIMER_CK/4 */

/* Control register 1 (TIMER_CTL1) */

#define TIMER_CTL1_CCSE               (1 << 0)   /* Bit 0:  Commutation control shadow enable */
#define TIMER_CTL1_CCUC               (1 << 2)   /* Bit 2:  Commutation control shadow register update control */
#define TIMER_CTL1_DMAS               (1 << 3)   /* Bit 3:  DMA request source selection */
#define TIMER_CTL1_MMC_SHIFT          (4)        /* Bits 4-6: Master mode control */
#define TIMER_CTL1_MMC_MASK           (7 << TIMER_CTL1_MMC_SHIFT)
#  define TIMER_CTL1_MMC_RESET        (0 << TIMER_CTL1_MMC_SHIFT)  /* UPG bit as trigger output */
#  define TIMER_CTL1_MMC_ENABLE       (1 << TIMER_CTL1_MMC_SHIFT)  /* CEN as trigger output */
#  define TIMER_CTL1_MMC_UPDATE       (2 << TIMER_CTL1_MMC_SHIFT)  /* Update event as trigger output */
#  define TIMER_CTL1_MMC_CH0          (3 << TIMER_CTL1_MMC_SHIFT)  /* CH0 capture/compare as trigger */
#  define TIMER_CTL1_MMC_O0CPRE       (4 << TIMER_CTL1_MMC_SHIFT)  /* O0CPRE as trigger output */
#  define TIMER_CTL1_MMC_O1CPRE       (5 << TIMER_CTL1_MMC_SHIFT)  /* O1CPRE as trigger output */
#  define TIMER_CTL1_MMC_O2CPRE       (6 << TIMER_CTL1_MMC_SHIFT)  /* O2CPRE as trigger output */
#  define TIMER_CTL1_MMC_O3CPRE       (7 << TIMER_CTL1_MMC_SHIFT)  /* O3CPRE as trigger output */
#define TIMER_CTL1_TI0S               (1 << 7)                     /* Bit 7:  Channel 0 trigger input selection (hall mode) */
#define TIMER_CTL1_ISO0               (1 << 8)                     /* Bit 8:  Idle state of channel 0 output */
#define TIMER_CTL1_ISO0N              (1 << 9)                     /* Bit 9:  Idle state of channel 0 complementary output */
#define TIMER_CTL1_ISO1               (1 << 10)                    /* Bit 10: Idle state of channel 1 output */
#define TIMER_CTL1_ISO1N              (1 << 11)                    /* Bit 11: Idle state of channel 1 complementary output */
#define TIMER_CTL1_ISO2               (1 << 12)                    /* Bit 12: Idle state of channel 2 output */
#define TIMER_CTL1_ISO2N              (1 << 13)                    /* Bit 13: Idle state of channel 2 complementary output */
#define TIMER_CTL1_ISO3               (1 << 14)                    /* Bit 14: Idle state of channel 3 output */

/* Slave mode configuration register (TIMER_SMCFG) */

#define TIMER_SMCFG_SMC_SHIFT         (0)        /* Bits 0-2: Slave mode control */
#define TIMER_SMCFG_SMC_MASK          (7 << TIMER_SMCFG_SMC_SHIFT)
#  define TIMER_SMCFG_SMC_DISABLE     (0 << TIMER_SMCFG_SMC_SHIFT)  /* Slave mode disable */
#  define TIMER_SMCFG_SMC_QUAD0       (1 << TIMER_SMCFG_SMC_SHIFT)  /* Quadrature decoder mode 0 */
#  define TIMER_SMCFG_SMC_QUAD1       (2 << TIMER_SMCFG_SMC_SHIFT)  /* Quadrature decoder mode 1 */
#  define TIMER_SMCFG_SMC_QUAD2       (3 << TIMER_SMCFG_SMC_SHIFT)  /* Quadrature decoder mode 2 */
#  define TIMER_SMCFG_SMC_RESTART     (4 << TIMER_SMCFG_SMC_SHIFT)  /* Restart mode */
#  define TIMER_SMCFG_SMC_PAUSE       (5 << TIMER_SMCFG_SMC_SHIFT)  /* Pause mode */
#  define TIMER_SMCFG_SMC_EVENT       (6 << TIMER_SMCFG_SMC_SHIFT)  /* Event mode */
#  define TIMER_SMCFG_SMC_EXTERNAL0   (7 << TIMER_SMCFG_SMC_SHIFT)  /* External clock mode 0 */
#define TIMER_SMCFG_TRGS_SHIFT        (4)                           /* Bits 4-6: Trigger selection */
#define TIMER_SMCFG_TRGS_MASK         (7 << TIMER_SMCFG_TRGS_SHIFT)
#  define TIMER_SMCFG_TRGS_ITI0       (0 << TIMER_SMCFG_TRGS_SHIFT)  /* Internal trigger 0 */
#  define TIMER_SMCFG_TRGS_ITI1       (1 << TIMER_SMCFG_TRGS_SHIFT)  /* Internal trigger 1 */
#  define TIMER_SMCFG_TRGS_ITI2       (2 << TIMER_SMCFG_TRGS_SHIFT)  /* Internal trigger 2 */
#  define TIMER_SMCFG_TRGS_ITI3       (3 << TIMER_SMCFG_TRGS_SHIFT)  /* Internal trigger 3 */
#  define TIMER_SMCFG_TRGS_CI0F_ED    (4 << TIMER_SMCFG_TRGS_SHIFT)  /* TI0 edge detector */
#  define TIMER_SMCFG_TRGS_CI0FE0     (5 << TIMER_SMCFG_TRGS_SHIFT)  /* Filtered TIMER input 0 */
#  define TIMER_SMCFG_TRGS_CI1FE1     (6 << TIMER_SMCFG_TRGS_SHIFT)  /* Filtered TIMER input 1 */
#  define TIMER_SMCFG_TRGS_ETIFP      (7 << TIMER_SMCFG_TRGS_SHIFT)  /* Filtered external trigger */
#define TIMER_SMCFG_MSM               (1 << 7)                       /* Bit 7:  Master-slave mode */
#define TIMER_SMCFG_ETFC_SHIFT        (8)                            /* Bits 8-11: External trigger filter control */
#define TIMER_SMCFG_ETFC_MASK         (15 << TIMER_SMCFG_ETFC_SHIFT)
#define TIMER_SMCFG_ETPSC_SHIFT       (12)                           /* Bits 12-13: External trigger prescaler */
#define TIMER_SMCFG_ETPSC_MASK        (3 << TIMER_SMCFG_ETPSC_SHIFT)
#  define TIMER_SMCFG_ETPSC_OFF       (0 << TIMER_SMCFG_ETPSC_SHIFT)  /* No divided */
#  define TIMER_SMCFG_ETPSC_DIV2      (1 << TIMER_SMCFG_ETPSC_SHIFT)  /* Divided by 2 */
#  define TIMER_SMCFG_ETPSC_DIV4      (2 << TIMER_SMCFG_ETPSC_SHIFT)  /* Divided by 4 */
#  define TIMER_SMCFG_ETPSC_DIV8      (3 << TIMER_SMCFG_ETPSC_SHIFT)  /* Divided by 8 */
#define TIMER_SMCFG_SMC1              (1 << 14)                       /* Bit 14: External clock mode 1 enable */
#define TIMER_SMCFG_ETP               (1 << 15)                       /* Bit 15: External trigger polarity */

/* DMA and interrupt enable register (TIMER_DMAINTEN) */

#define TIMER_DMAINTEN_UPIE           (1 << 0)   /* Bit 0:  Update interrupt enable */
#define TIMER_DMAINTEN_CH0IE          (1 << 1)   /* Bit 1:  Channel 0 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH1IE          (1 << 2)   /* Bit 2:  Channel 1 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH2IE          (1 << 3)   /* Bit 3:  Channel 2 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH3IE          (1 << 4)   /* Bit 4:  Channel 3 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CMTIE          (1 << 5)   /* Bit 5:  Commutation interrupt enable */
#define TIMER_DMAINTEN_TRGIE          (1 << 6)   /* Bit 6:  Trigger interrupt enable */
#define TIMER_DMAINTEN_BRKIE          (1 << 7)   /* Bit 7:  Break interrupt enable */
#define TIMER_DMAINTEN_UPDEN          (1 << 8)   /* Bit 8:  Update DMA request enable */
#define TIMER_DMAINTEN_CH0DEN         (1 << 9)   /* Bit 9:  Channel 0 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH1DEN         (1 << 10)  /* Bit 10: Channel 1 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH2DEN         (1 << 11)  /* Bit 11: Channel 2 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH3DEN         (1 << 12)  /* Bit 12: Channel 3 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CMTDEN         (1 << 13)  /* Bit 13: Commutation DMA request enable */
#define TIMER_DMAINTEN_TRGDEN         (1 << 14)  /* Bit 14: Trigger DMA request enable */

/* Interrupt flag register (TIMER_INTF) */

#define TIMER_INTF_UPIF               (1 << 0)   /* Bit 0:  Update interrupt flag */
#define TIMER_INTF_CH0IF              (1 << 1)   /* Bit 1:  Channel 0 capture/compare interrupt flag */
#define TIMER_INTF_CH1IF              (1 << 2)   /* Bit 2:  Channel 1 capture/compare interrupt flag */
#define TIMER_INTF_CH2IF              (1 << 3)   /* Bit 3:  Channel 2 capture/compare interrupt flag */
#define TIMER_INTF_CH3IF              (1 << 4)   /* Bit 4:  Channel 3 capture/compare interrupt flag */
#define TIMER_INTF_CMTIF              (1 << 5)   /* Bit 5:  Channel commutation interrupt flag */
#define TIMER_INTF_TRGIF              (1 << 6)   /* Bit 6:  Trigger interrupt flag */
#define TIMER_INTF_BRKIF              (1 << 7)   /* Bit 7:  Break interrupt flag */
#define TIMER_INTF_CH0OF              (1 << 9)   /* Bit 9:  Channel 0 over capture flag */
#define TIMER_INTF_CH1OF              (1 << 10)  /* Bit 10: Channel 1 over capture flag */
#define TIMER_INTF_CH2OF              (1 << 11)  /* Bit 11: Channel 2 over capture flag */
#define TIMER_INTF_CH3OF              (1 << 12)  /* Bit 12: Channel 3 over capture flag */

/* Software event generation register (TIMER_SWEVG) */

#define TIMER_SWEVG_UPG               (1 << 0)   /* Bit 0:  Update event generate */
#define TIMER_SWEVG_CH0G              (1 << 1)   /* Bit 1:  Channel 0 capture or compare event generation */
#define TIMER_SWEVG_CH1G              (1 << 2)   /* Bit 2:  Channel 1 capture or compare event generation */
#define TIMER_SWEVG_CH2G              (1 << 3)   /* Bit 3:  Channel 2 capture or compare event generation */
#define TIMER_SWEVG_CH3G              (1 << 4)   /* Bit 4:  Channel 3 capture or compare event generation */
#define TIMER_SWEVG_CMTG              (1 << 5)   /* Bit 5:  Channel commutation event generation */
#define TIMER_SWEVG_TRGG              (1 << 6)   /* Bit 6:  Trigger event generation */
#define TIMER_SWEVG_BRKG              (1 << 7)   /* Bit 7:  Break event generation */

/* Channel control register 0 - Output compare mode (TIMER_CHCTL0) */

#define TIMER_CHCTL0_CH0MS_SHIFT      (0)        /* Bits 0-1: Channel 0 mode selection */
#define TIMER_CHCTL0_CH0MS_MASK       (3 << TIMER_CHCTL0_CH0MS_SHIFT)
#  define TIMER_CHCTL0_CH0MS_OUTPUT   (0 << TIMER_CHCTL0_CH0MS_SHIFT)  /* Output mode */
#  define TIMER_CHCTL0_CH0MS_CI0      (1 << TIMER_CHCTL0_CH0MS_SHIFT)  /* CI0 input */
#  define TIMER_CHCTL0_CH0MS_CI1      (2 << TIMER_CHCTL0_CH0MS_SHIFT)  /* CI1 input */
#  define TIMER_CHCTL0_CH0MS_ITS      (3 << TIMER_CHCTL0_CH0MS_SHIFT)  /* ITS input */
#define TIMER_CHCTL0_CH0COMFEN        (1 << 2)                         /* Bit 2:  Channel 0 output compare fast enable */
#define TIMER_CHCTL0_CH0COMSEN        (1 << 3)                         /* Bit 3:  Channel 0 output compare shadow enable */
#define TIMER_CHCTL0_CH0COMCTL_SHIFT  (4)                              /* Bits 4-6: Channel 0 output compare control */
#define TIMER_CHCTL0_CH0COMCTL_MASK   (7 << TIMER_CHCTL0_CH0COMCTL_SHIFT)
#  define TIMER_CHCTL0_CH0COMCTL_TIMING    (0 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* Timing mode */
#  define TIMER_CHCTL0_CH0COMCTL_ACTIVE    (1 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* Active mode */
#  define TIMER_CHCTL0_CH0COMCTL_INACTIVE  (2 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* Inactive mode */
#  define TIMER_CHCTL0_CH0COMCTL_TOGGLE    (3 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* Toggle mode */
#  define TIMER_CHCTL0_CH0COMCTL_LOW       (4 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* Force low mode */
#  define TIMER_CHCTL0_CH0COMCTL_HIGH      (5 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* Force high mode */
#  define TIMER_CHCTL0_CH0COMCTL_PWM0      (6 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* PWM mode 0 */
#  define TIMER_CHCTL0_CH0COMCTL_PWM1      (7 << TIMER_CHCTL0_CH0COMCTL_SHIFT)  /* PWM mode 1 */
#define TIMER_CHCTL0_CH0COMCEN        (1 << 7)                                  /* Bit 7:  Channel 0 output compare clear enable */

#define TIMER_CHCTL0_CH1MS_SHIFT      (8)                                       /* Bits 8-9: Channel 1 mode selection */
#define TIMER_CHCTL0_CH1MS_MASK       (3 << TIMER_CHCTL0_CH1MS_SHIFT)
#define TIMER_CHCTL0_CH1COMFEN        (1 << 10)                                 /* Bit 10: Channel 1 output compare fast enable */
#define TIMER_CHCTL0_CH1COMSEN        (1 << 11)                                 /* Bit 11: Channel 1 output compare shadow enable */
#define TIMER_CHCTL0_CH1COMCTL_SHIFT  (12)                                      /* Bits 12-14: Channel 1 output compare control */
#define TIMER_CHCTL0_CH1COMCTL_MASK   (7 << TIMER_CHCTL0_CH1COMCTL_SHIFT)
#  define TIMER_CHCTL0_CH1COMCTL_TIMING    (0 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* Timing mode */
#  define TIMER_CHCTL0_CH1COMCTL_ACTIVE    (1 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* Active mode */
#  define TIMER_CHCTL0_CH1COMCTL_INACTIVE  (2 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* Inactive mode */
#  define TIMER_CHCTL0_CH1COMCTL_TOGGLE    (3 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* Toggle mode */
#  define TIMER_CHCTL0_CH1COMCTL_LOW       (4 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* Force low mode */
#  define TIMER_CHCTL0_CH1COMCTL_HIGH      (5 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* Force high mode */
#  define TIMER_CHCTL0_CH1COMCTL_PWM0      (6 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* PWM mode 0 */
#  define TIMER_CHCTL0_CH1COMCTL_PWM1      (7 << TIMER_CHCTL0_CH1COMCTL_SHIFT)  /* PWM mode 1 */
#define TIMER_CHCTL0_CH1COMCEN        (1 << 15)                                 /* Bit 15: Channel 1 output compare clear enable */

/* Channel control register 0 - Input capture mode (TIMER_CHCTL0) */

#define TIMER_CHCTL0_CH0CAPPSC_SHIFT  (2)        /* Bits 2-3: Channel 0 input capture prescaler */
#define TIMER_CHCTL0_CH0CAPPSC_MASK   (3 << TIMER_CHCTL0_CH0CAPPSC_SHIFT)
#define TIMER_CHCTL0_CH0CAPFLT_SHIFT  (4)        /* Bits 4-7: Channel 0 input capture filter control */
#define TIMER_CHCTL0_CH0CAPFLT_MASK   (15 << TIMER_CHCTL0_CH0CAPFLT_SHIFT)
#define TIMER_CHCTL0_CH1CAPPSC_SHIFT  (10)       /* Bits 10-11: Channel 1 input capture prescaler */
#define TIMER_CHCTL0_CH1CAPPSC_MASK   (3 << TIMER_CHCTL0_CH1CAPPSC_SHIFT)
#define TIMER_CHCTL0_CH1CAPFLT_SHIFT  (12)       /* Bits 12-15: Channel 1 input capture filter control */
#define TIMER_CHCTL0_CH1CAPFLT_MASK   (15 << TIMER_CHCTL0_CH1CAPFLT_SHIFT)

/* Channel control register 1 - Output compare mode (TIMER_CHCTL1) */

#define TIMER_CHCTL1_CH2MS_SHIFT      (0)        /* Bits 0-1: Channel 2 mode selection */
#define TIMER_CHCTL1_CH2MS_MASK       (3 << TIMER_CHCTL1_CH2MS_SHIFT)
#define TIMER_CHCTL1_CH2COMFEN        (1 << 2)   /* Bit 2:  Channel 2 output compare fast enable */
#define TIMER_CHCTL1_CH2COMSEN        (1 << 3)   /* Bit 3:  Channel 2 output compare shadow enable */
#define TIMER_CHCTL1_CH2COMCTL_SHIFT  (4)        /* Bits 4-6: Channel 2 output compare control */
#define TIMER_CHCTL1_CH2COMCTL_MASK   (7 << TIMER_CHCTL1_CH2COMCTL_SHIFT)
#  define TIMER_CHCTL1_CH2COMCTL_TIMING    (0 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* Timing mode */
#  define TIMER_CHCTL1_CH2COMCTL_ACTIVE    (1 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* Active mode */
#  define TIMER_CHCTL1_CH2COMCTL_INACTIVE  (2 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* Inactive mode */
#  define TIMER_CHCTL1_CH2COMCTL_TOGGLE    (3 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* Toggle mode */
#  define TIMER_CHCTL1_CH2COMCTL_LOW       (4 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* Force low mode */
#  define TIMER_CHCTL1_CH2COMCTL_HIGH      (5 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* Force high mode */
#  define TIMER_CHCTL1_CH2COMCTL_PWM0      (6 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* PWM mode 0 */
#  define TIMER_CHCTL1_CH2COMCTL_PWM1      (7 << TIMER_CHCTL1_CH2COMCTL_SHIFT)  /* PWM mode 1 */
#define TIMER_CHCTL1_CH2COMCEN        (1 << 7)                                  /* Bit 7:  Channel 2 output compare clear enable */

#define TIMER_CHCTL1_CH3MS_SHIFT      (8)                                       /* Bits 8-9: Channel 3 mode selection */
#define TIMER_CHCTL1_CH3MS_MASK       (3 << TIMER_CHCTL1_CH3MS_SHIFT)
#define TIMER_CHCTL1_CH3COMFEN        (1 << 10)                                 /* Bit 10: Channel 3 output compare fast enable */
#define TIMER_CHCTL1_CH3COMSEN        (1 << 11)                                 /* Bit 11: Channel 3 output compare shadow enable */
#define TIMER_CHCTL1_CH3COMCTL_SHIFT  (12)                                      /* Bits 12-14: Channel 3 output compare control */
#define TIMER_CHCTL1_CH3COMCTL_MASK   (7 << TIMER_CHCTL1_CH3COMCTL_SHIFT)
#  define TIMER_CHCTL1_CH3COMCTL_TIMING    (0 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* Timing mode */
#  define TIMER_CHCTL1_CH3COMCTL_ACTIVE    (1 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* Active mode */
#  define TIMER_CHCTL1_CH3COMCTL_INACTIVE  (2 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* Inactive mode */
#  define TIMER_CHCTL1_CH3COMCTL_TOGGLE    (3 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* Toggle mode */
#  define TIMER_CHCTL1_CH3COMCTL_LOW       (4 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* Force low mode */
#  define TIMER_CHCTL1_CH3COMCTL_HIGH      (5 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* Force high mode */
#  define TIMER_CHCTL1_CH3COMCTL_PWM0      (6 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* PWM mode 0 */
#  define TIMER_CHCTL1_CH3COMCTL_PWM1      (7 << TIMER_CHCTL1_CH3COMCTL_SHIFT)  /* PWM mode 1 */
#define TIMER_CHCTL1_CH3COMCEN        (1 << 15)                                 /* Bit 15: Channel 3 output compare clear enable */

/* Channel control register 1 - Input capture mode (TIMER_CHCTL1) */

#define TIMER_CHCTL1_CH2CAPPSC_SHIFT  (2)        /* Bits 2-3: Channel 2 input capture prescaler */
#define TIMER_CHCTL1_CH2CAPPSC_MASK   (3 << TIMER_CHCTL1_CH2CAPPSC_SHIFT)
#define TIMER_CHCTL1_CH2CAPFLT_SHIFT  (4)        /* Bits 4-7: Channel 2 input capture filter control */
#define TIMER_CHCTL1_CH2CAPFLT_MASK   (15 << TIMER_CHCTL1_CH2CAPFLT_SHIFT)
#define TIMER_CHCTL1_CH3CAPPSC_SHIFT  (10)       /* Bits 10-11: Channel 3 input capture prescaler */
#define TIMER_CHCTL1_CH3CAPPSC_MASK   (3 << TIMER_CHCTL1_CH3CAPPSC_SHIFT)
#define TIMER_CHCTL1_CH3CAPFLT_SHIFT  (12)       /* Bits 12-15: Channel 3 input capture filter control */
#define TIMER_CHCTL1_CH3CAPFLT_MASK   (15 << TIMER_CHCTL1_CH3CAPFLT_SHIFT)

/* Channel control register 2 (TIMER_CHCTL2) */

#define TIMER_CHCTL2_CH0EN            (1 << 0)   /* Bit 0:  Channel 0 capture/compare function enable */
#define TIMER_CHCTL2_CH0P             (1 << 1)   /* Bit 1:  Channel 0 capture/compare function polarity */
#define TIMER_CHCTL2_CH0NEN           (1 << 2)   /* Bit 2:  Channel 0 complementary output enable */
#define TIMER_CHCTL2_CH0NP            (1 << 3)   /* Bit 3:  Channel 0 complementary output polarity */
#define TIMER_CHCTL2_CH1EN            (1 << 4)   /* Bit 4:  Channel 1 capture/compare function enable */
#define TIMER_CHCTL2_CH1P             (1 << 5)   /* Bit 5:  Channel 1 capture/compare function polarity */
#define TIMER_CHCTL2_CH1NEN           (1 << 6)   /* Bit 6:  Channel 1 complementary output enable */
#define TIMER_CHCTL2_CH1NP            (1 << 7)   /* Bit 7:  Channel 1 complementary output polarity */
#define TIMER_CHCTL2_CH2EN            (1 << 8)   /* Bit 8:  Channel 2 capture/compare function enable */
#define TIMER_CHCTL2_CH2P             (1 << 9)   /* Bit 9:  Channel 2 capture/compare function polarity */
#define TIMER_CHCTL2_CH2NEN           (1 << 10)  /* Bit 10: Channel 2 complementary output enable */
#define TIMER_CHCTL2_CH2NP            (1 << 11)  /* Bit 11: Channel 2 complementary output polarity */
#define TIMER_CHCTL2_CH3EN            (1 << 12)  /* Bit 12: Channel 3 capture/compare function enable */
#define TIMER_CHCTL2_CH3P             (1 << 13)  /* Bit 13: Channel 3 capture/compare function polarity */

/* Counter register (TIMER_CNT) */

#define TIMER_CNT_CNT_SHIFT           (0)        /* Bits 0-15: 16-bit timer counter */
#define TIMER_CNT_CNT_MASK            (0xffff << TIMER_CNT_CNT_SHIFT)

/* Prescaler register (TIMER_PSC) */

#define TIMER_PSC_PSC_SHIFT           (0)        /* Bits 0-15: Prescaler value of the counter clock */
#define TIMER_PSC_PSC_MASK            (0xffff << TIMER_PSC_PSC_SHIFT)

/* Counter auto reload register (TIMER_CAR) */

#define TIMER_CAR_CARL_SHIFT          (0)        /* Bits 0-15: 16-bit counter auto reload value */
#define TIMER_CAR_CARL_MASK           (0xffff << TIMER_CAR_CARL_SHIFT)

/* Counter repetition register (TIMER_CREP) */

#define TIMER_CREP_CREP_SHIFT         (0)        /* Bits 0-7: Counter repetition value */
#define TIMER_CREP_CREP_MASK          (0xff << TIMER_CREP_CREP_SHIFT)
#define TIMER_CREP_REP_MAX            255        /* Maximum repetition counter value */

/* Channel 0-3 capture/compare value register (TIMER_CHxCV) */

#define TIMER_CHXCV_CHXVAL_SHIFT      (0)        /* Bits 0-15: 16-bit capture/compare value */
#define TIMER_CHXCV_CHXVAL_MASK       (0xffff << TIMER_CHXCV_CHXVAL_SHIFT)

/* Channel complementary protection register (TIMER_CCHP) */

#define TIMER_CCHP_DTCFG_SHIFT        (0)        /* Bits 0-7: Dead time configure */
#define TIMER_CCHP_DTCFG_MASK         (0xff << TIMER_CCHP_DTCFG_SHIFT)
#define TIMER_CCHP_PROT_SHIFT         (8)        /* Bits 8-9: Complementary register protect control */
#define TIMER_CCHP_PROT_MASK          (3 << TIMER_CCHP_PROT_SHIFT)
#  define TIMER_CCHP_PROT_OFF         (0 << TIMER_CCHP_PROT_SHIFT)  /* Protect disable */
#  define TIMER_CCHP_PROT_0           (1 << TIMER_CCHP_PROT_SHIFT)  /* PROT mode 0 */
#  define TIMER_CCHP_PROT_1           (2 << TIMER_CCHP_PROT_SHIFT)  /* PROT mode 1 */
#  define TIMER_CCHP_PROT_2           (3 << TIMER_CCHP_PROT_SHIFT)  /* PROT mode 2 */
#define TIMER_CCHP_IOS                (1 << 10)                     /* Bit 10: Idle mode off-state configure */
#define TIMER_CCHP_ROS                (1 << 11)                     /* Bit 11: Run mode off-state configure */
#define TIMER_CCHP_BRKEN              (1 << 12)                     /* Bit 12: Break enable */
#define TIMER_CCHP_BRKP               (1 << 13)                     /* Bit 13: Break polarity */
#define TIMER_CCHP_OAEN               (1 << 14)                     /* Bit 14: Output automatic enable */
#define TIMER_CCHP_POEN               (1 << 15)                     /* Bit 15: Primary output enable */

/* DMA configuration register (TIMER_DMACFG) */

#define TIMER_DMACFG_DMATA_SHIFT      (0)        /* Bits 0-4: DMA transfer access start address */
#define TIMER_DMACFG_DMATA_MASK       (0x1f << TIMER_DMACFG_DMATA_SHIFT)
#define TIMER_DMACFG_DMATC_SHIFT      (8)        /* Bits 8-12: DMA transfer count */
#define TIMER_DMACFG_DMATC_MASK       (0x1f << TIMER_DMACFG_DMATC_SHIFT)

/* DMA transfer buffer address (TIMER_DMATB) */

#define TIMER_DMATB_DMATB_SHIFT       (0)        /* Bits 0-15: DMA transfer buffer address */
#define TIMER_DMATB_DMATB_MASK        (0xffff << TIMER_DMATB_DMATB_SHIFT)

/* Configuration register (TIMER_CFG) */

#define TIMER_CFG_OUTSEL              (1 << 0)   /* Bit 0:  The output value selection */
#define TIMER_CFG_CHVSEL              (1 << 1)   /* Bit 1:  Write CHxVAL register selection */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_TIMER_H */
