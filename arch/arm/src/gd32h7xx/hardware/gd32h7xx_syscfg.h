/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h7xx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_SYSCFG_H
#define __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_SYSCFG_PMCFG_OFFSET         0x0004                    /* Pin multiplexer configuration register */
#define GD32_SYSCFG_EXTISS0_OFFSET       0x0008                    /* EXTI sources selection register 0 */
#define GD32_SYSCFG_EXTISS1_OFFSET       0x000c                    /* EXTI sources selection register 1 */
#define GD32_SYSCFG_EXTISS2_OFFSET       0x0010                    /* EXTI sources selection register 2 */
#define GD32_SYSCFG_EXTISS3_OFFSET       0x0014                    /* EXTI sources selection register 3 */
#define GD32_SYSCFG_LKCTL_OFFSET         0x0018                    /* Lock control register */
#define GD32_SYSCFG_CPSCTL_OFFSET        0x0020                    /* I/O compensation control register */
#define GD32_SYSCFG_CPSCCCFG_OFFSET      0x0028                    /* I/O compensation cell configuration register */
#define GD32_SYSCFG_TIMERCISEL0_OFFSET   0x0034                    /* Timer channel input selection register 0 */
#define GD32_SYSCFG_TIMERCISEL1_OFFSET   0x0038                    /* Timer channel input selection register 1 */
#define GD32_SYSCFG_TIMERCISEL2_OFFSET   0x003C                    /* Timer channel input selection register 2 */
#define GD32_SYSCFG_TIMERCISEL3_OFFSET   0x0040                    /* Timer channel input selection register 3 */
#define GD32_SYSCFG_TIMERCISEL4_OFFSET   0x0044                    /* Timer channel input selection register 4 */
#define GD32_SYSCFG_TIMERCISEL5_OFFSET   0x0048                    /* Timer channel input selection register 5 */
#define GD32_SYSCFG_TIMERCISEL6_OFFSET   0x004C                    /* Timer channel input selection register 6 */
#define GD32_SYSCFG_CPUICAC_OFFSET       0x0054                    /* CPU ICACHE error status register */
#define GD32_SYSCFG_CPUDCAC_OFFSET       0x0058                    /* CPU DCACHE error status register */
#define GD32_SYSCFG_FPUINTEN_OFFSET      0x005C                    /* FPU interrupt enable register */
#define GD32_SYSCFG_SRAMCFG0_OFFSET      0x0064                    /* SRAM configuration register 0 */
#define GD32_SYSCFG_SRAMCFG1_OFFSET      0x0068                    /* SRAM configuration register 1 */
#define GD32_SYSCFG_TIMERCFG0_OFFSET     0x0100                    /* Timer trigger configuration register 0 */
#define GD32_SYSCFG_TIMERCFG1_OFFSET     0x0104                    /* Timer trigger configuration register 1 */
#define GD32_SYSCFG_TIMERCFG2_OFFSET     0x0108                    /* Timer trigger configuration register 2 */
#define GD32_SYSCFG_USERCFG_OFFSET       0x0300                    /* User configuration register */

/* Register Addresses *******************************************************/

#define GD32_SYSCFG_PMCFG                (GD32_SYSCFG_BASE+GD32_SYSCFG_PMCFG_OFFSET)
#define GD32_SYSCFG_EXTISS0              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS0_OFFSET)
#define GD32_SYSCFG_EXTISS1              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS1_OFFSET)
#define GD32_SYSCFG_EXTISS2              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS2_OFFSET)
#define GD32_SYSCFG_EXTISS3              (GD32_SYSCFG_BASE+GD32_SYSCFG_EXTISS3_OFFSET)
#define GD32_SYSCFG_LKCTL                (GD32_SYSCFG_BASE+GD32_SYSCFG_LKCTL_OFFSET)
#define GD32_SYSCFG_CPSCTL               (GD32_SYSCFG_BASE+GD32_SYSCFG_CPSCTL_OFFSET)
#define GD32_SYSCFG_CPSCCCFG             (GD32_SYSCFG_BASE+GD32_SYSCFG_CPSCCCFG_OFFSET)
#define GD32_SYSCFG_TIMERCISEL0          (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCISEL0_OFFSET)
#define GD32_SYSCFG_TIMERCISEL1          (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCISEL1_OFFSET)
#define GD32_SYSCFG_TIMERCISEL2          (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCISEL2_OFFSET)
#define GD32_SYSCFG_TIMERCISEL3          (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCISEL3_OFFSET)
#define GD32_SYSCFG_TIMERCISEL4          (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCISEL4_OFFSET)
#define GD32_SYSCFG_TIMERCISEL5          (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCISEL5_OFFSET)
#define GD32_SYSCFG_TIMERCISEL6          (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCISEL6_OFFSET)
#define GD32_SYSCFG_CPUICAC              (GD32_SYSCFG_BASE+GD32_SYSCFG_CPUICAC_OFFSET)
#define GD32_SYSCFG_CPUDCAC              (GD32_SYSCFG_BASE+GD32_SYSCFG_CPUDCAC_OFFSET)
#define GD32_SYSCFG_FPUINTEN             (GD32_SYSCFG_BASE+GD32_SYSCFG_FPUINTEN_OFFSET)
#define GD32_SYSCFG_SRAMCFG0             (GD32_SYSCFG_BASE+GD32_SYSCFG_SRAMCFG0_OFFSET)
#define GD32_SYSCFG_SRAMCFG1             (GD32_SYSCFG_BASE+GD32_SYSCFG_SRAMCFG1_OFFSET)
#define GD32_SYSCFG_TIMERCFG0            (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCFG0_OFFSET)
#define GD32_SYSCFG_TIMERCFG1            (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCFG1_OFFSET)
#define GD32_SYSCFG_TIMERCFG2            (GD32_SYSCFG_BASE+GD32_SYSCFG_TIMERCFG2_OFFSET)
#define GD32_SYSCFG_USERCFG              (GD32_SYSCFG_BASE+GD32_SYSCFG_USERCFG_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Pin multiplexer configuration register */

/* I2C fast mode plus enable */
#define SYSCFG_PMCFG_I2C0FMPEN_SHIFT     (0)                                    /* Bit 0: I2C0 fast mode plus enable */
#define SYSCFG_PMCFG_I2C0FMPEN_MASK      (1 << SYSCFG_PMCFG_I2C0FMPEN_SHIFT)
#define SYSCFG_PMCFG_I2C1FMPEN_SHIFT     (1)                                    /* Bit 1: I2C1 fast mode plus enable */
#define SYSCFG_PMCFG_I2C1FMPEN_MASK      (1 << SYSCFG_PMCFG_I2C1FMPEN_SHIFT)
#define SYSCFG_PMCFG_I2C2FMPEN_SHIFT     (2)                                    /* Bit 2: I2C2 fast mode plus enable */
#define SYSCFG_PMCFG_I2C2FMPEN_MASK      (1 << SYSCFG_PMCFG_I2C2FMPEN_SHIFT)
#define SYSCFG_PMCFG_I2C3FMPEN_SHIFT     (3)                                    /* Bit 3: I2C3 fast mode plus enable */
#define SYSCFG_PMCFG_I2C3FMPEN_MASK      (1 << SYSCFG_PMCFG_I2C3FMPEN_SHIFT)
#define SYSCFG_PMCFG_PB6FMPEN_SHIFT      (4)                                    /* Bit 4: PB6 fast mode plus enable */
#define SYSCFG_PMCFG_PB6FMPEN_MASK       (1 << SYSCFG_PMCFG_PB6FMPEN_SHIFT)
#define SYSCFG_PMCFG_PB7FMPEN_SHIFT      (5)                                    /* Bit 5: PB7 fast mode plus enable */
#define SYSCFG_PMCFG_PB7FMPEN_MASK       (1 << SYSCFG_PMCFG_PB7FMPEN_SHIFT)
#define SYSCFG_PMCFG_PB8FMPEN_SHIFT      (6)                                    /* Bit 6: PB8 fast mode plus enable */
#define SYSCFG_PMCFG_PB8FMPEN_MASK       (1 << SYSCFG_PMCFG_PB8FMPEN_SHIFT)
#define SYSCFG_PMCFG_PB9FMPEN_SHIFT      (7)                                    /* Bit 7: PB9 fast mode plus enable */
#define SYSCFG_PMCFG_PB9FMPEN_MASK       (1 << SYSCFG_PMCFG_PB9FMPEN_SHIFT)

/* Ethernet PHY selection */
#define SYSCFG_PMCFG_ENET1_PHY_SEL_SHIFT (22)                                   /* Bit 22: Ethernet1 PHY selection */
#define SYSCFG_PMCFG_ENET1_PHY_SEL_MASK  (1 << SYSCFG_PMCFG_ENET1_PHY_SEL_SHIFT)
#  define SYSCFG_ENET1_PHY_MII           (0 << SYSCFG_PMCFG_ENET1_PHY_SEL_SHIFT) /* MII is selected for the Ethernet1 MAC */
#  define SYSCFG_ENET1_PHY_RMII          (1 << SYSCFG_PMCFG_ENET1_PHY_SEL_SHIFT) /* RMII is selected for the Ethernet1 MAC */

#define SYSCFG_PMCFG_ENET0_PHY_SEL_SHIFT (23)                                   /* Bit 23: Ethernet0 PHY selection */
#define SYSCFG_PMCFG_ENET0_PHY_SEL_MASK  (1 << SYSCFG_PMCFG_ENET0_PHY_SEL_SHIFT)
#  define SYSCFG_ENET0_PHY_MII           (0 << SYSCFG_PMCFG_ENET0_PHY_SEL_SHIFT) /* MII is selected for the Ethernet0 MAC */
#  define SYSCFG_ENET0_PHY_RMII          (1 << SYSCFG_PMCFG_ENET0_PHY_SEL_SHIFT) /* RMII is selected for the Ethernet0 MAC */

/* Pin analog switch enable */
#define SYSCFG_PMCFG_PA0SWON_SHIFT       (24)                                   /* Bit 24: PA0 analog switch enable */
#define SYSCFG_PMCFG_PA0SWON_MASK        (1 << SYSCFG_PMCFG_PA0SWON_SHIFT)
#define SYSCFG_PMCFG_PA1SWON_SHIFT       (25)                                   /* Bit 25: PA1 analog switch enable */
#define SYSCFG_PMCFG_PA1SWON_MASK        (1 << SYSCFG_PMCFG_PA1SWON_SHIFT)
#define SYSCFG_PMCFG_PC2SWON_SHIFT       (26)                                   /* Bit 26: PC2 analog switch enable */
#define SYSCFG_PMCFG_PC2SWON_MASK        (1 << SYSCFG_PMCFG_PC2SWON_SHIFT)
#define SYSCFG_PMCFG_PC3SWON_SHIFT       (27)                                   /* Bit 27: PC3 analog switch enable */
#define SYSCFG_PMCFG_PC3SWON_MASK        (1 << SYSCFG_PMCFG_PC3SWON_SHIFT)

/* SYSCFG external interrupt configuration register 0-3 */

/* SYSCFG_EXTISS0 bits definitions */
#define SYSCFG_EXTISS0_EXTI0_SHIFT       (0)
#define SYSCFG_EXTISS0_EXTI0_MASK        (15 << SYSCFG_EXTISS0_EXTI0_SHIFT)
#define SYSCFG_EXTISS0_EXTI1_SHIFT       (4)
#define SYSCFG_EXTISS0_EXTI1_MASK        (15 << SYSCFG_EXTISS0_EXTI1_SHIFT)
#define SYSCFG_EXTISS0_EXTI2_SHIFT       (8)
#define SYSCFG_EXTISS0_EXTI2_MASK        (15 << SYSCFG_EXTISS0_EXTI2_SHIFT)
#define SYSCFG_EXTISS0_EXTI3_SHIFT       (12)
#define SYSCFG_EXTISS0_EXTI3_MASK        (15 << SYSCFG_EXTISS0_EXTI3_SHIFT)

/* SYSCFG_EXTISS1 bits definitions */
#define SYSCFG_EXTISS1_EXTI4_SHIFT       (0)
#define SYSCFG_EXTISS1_EXTI4_MASK        (15 << SYSCFG_EXTISS1_EXTI4_SHIFT)
#define SYSCFG_EXTISS1_EXTI5_SHIFT       (4)
#define SYSCFG_EXTISS1_EXTI5_MASK        (15 << SYSCFG_EXTISS1_EXTI5_SHIFT)
#define SYSCFG_EXTISS1_EXTI6_SHIFT       (8)
#define SYSCFG_EXTISS1_EXTI6_MASK        (15 << SYSCFG_EXTISS1_EXTI6_SHIFT)
#define SYSCFG_EXTISS1_EXTI7_SHIFT       (12)
#define SYSCFG_EXTISS1_EXTI7_MASK        (15 << SYSCFG_EXTISS1_EXTI7_SHIFT)

/* SYSCFG_EXTISS2 bits definitions */
#define SYSCFG_EXTISS2_EXTI8_SHIFT       (0)
#define SYSCFG_EXTISS2_EXTI8_MASK        (15 << SYSCFG_EXTISS2_EXTI8_SHIFT)
#define SYSCFG_EXTISS2_EXTI9_SHIFT       (4)
#define SYSCFG_EXTISS2_EXTI9_MASK        (15 << SYSCFG_EXTISS2_EXTI9_SHIFT)
#define SYSCFG_EXTISS2_EXTI10_SHIFT      (8)
#define SYSCFG_EXTISS2_EXTI10_MASK       (15 << SYSCFG_EXTISS2_EXTI10_SHIFT)
#define SYSCFG_EXTISS2_EXTI11_SHIFT      (12)
#define SYSCFG_EXTISS2_EXTI11_MASK       (15 << SYSCFG_EXTISS2_EXTI11_SHIFT)

/* SYSCFG_EXTISS3 bits definitions */
#define SYSCFG_EXTISS3_EXTI12_SHIFT      (0)
#define SYSCFG_EXTISS3_EXTI12_MASK       (15 << SYSCFG_EXTISS3_EXTI12_SHIFT)
#define SYSCFG_EXTISS3_EXTI13_SHIFT      (4)
#define SYSCFG_EXTISS3_EXTI13_MASK       (15 << SYSCFG_EXTISS3_EXTI13_SHIFT)
#define SYSCFG_EXTISS3_EXTI14_SHIFT      (8)
#define SYSCFG_EXTISS3_EXTI14_MASK       (15 << SYSCFG_EXTISS3_EXTI14_SHIFT)
#define SYSCFG_EXTISS3_EXTI15_SHIFT      (12)
#define SYSCFG_EXTISS3_EXTI15_MASK       (15 << SYSCFG_EXTISS3_EXTI15_SHIFT)

/* Lock control register */

#define SYSCFG_LKCTL_LVD_LOCK_SHIFT      (2)                                    /* Bit 2: LVD lock bit */
#define SYSCFG_LKCTL_LVD_LOCK_MASK       (1 << SYSCFG_LKCTL_LVD_LOCK_SHIFT)
#define SYSCFG_LKCTL_CPU_LOCK_SHIFT      (6)                                    /* Bit 6: CPU lock bit */
#define SYSCFG_LKCTL_CPU_LOCK_MASK       (1 << SYSCFG_LKCTL_CPU_LOCK_SHIFT)
#define SYSCFG_LKCTL_BKPRAM_LOCK_SHIFT   (7)                                    /* Bit 7: Backup RAM lock bit */
#define SYSCFG_LKCTL_BKPRAM_LOCK_MASK    (1 << SYSCFG_LKCTL_BKPRAM_LOCK_SHIFT)
#define SYSCFG_LKCTL_SRAM1_LOCK_SHIFT    (11)                                   /* Bit 11: SRAM1 lock bit */
#define SYSCFG_LKCTL_SRAM1_LOCK_MASK     (1 << SYSCFG_LKCTL_SRAM1_LOCK_SHIFT)
#define SYSCFG_LKCTL_SRAM0_LOCK_SHIFT    (12)                                   /* Bit 12: SRAM0 lock bit */
#define SYSCFG_LKCTL_SRAM0_LOCK_MASK     (1 << SYSCFG_LKCTL_SRAM0_LOCK_SHIFT)
#define SYSCFG_LKCTL_DTCM_LOCK_SHIFT     (13)                                   /* Bit 13: DTCM lock bit */
#define SYSCFG_LKCTL_DTCM_LOCK_MASK      (1 << SYSCFG_LKCTL_DTCM_LOCK_SHIFT)
#define SYSCFG_LKCTL_ITCM_LOCK_SHIFT     (14)                                   /* Bit 14: ITCM lock bit */
#define SYSCFG_LKCTL_ITCM_LOCK_MASK      (1 << SYSCFG_LKCTL_ITCM_LOCK_SHIFT)
#define SYSCFG_LKCTL_AXIRAM_LOCK_SHIFT   (15)                                   /* Bit 15: AXI RAM lock bit */
#define SYSCFG_LKCTL_AXIRAM_LOCK_MASK    (1 << SYSCFG_LKCTL_AXIRAM_LOCK_SHIFT)

/* I/O compensation control register */

#define SYSCFG_CPSCTL_CPS_EN_SHIFT       (0)                                    /* Bit 0: I/O compensation cell enable */
#define SYSCFG_CPSCTL_CPS_EN_MASK        (1 << SYSCFG_CPSCTL_CPS_EN_SHIFT)
#  define SYSCFG_COMPENSATION_ENABLE     (1 << SYSCFG_CPSCTL_CPS_EN_SHIFT)      /* I/O compensation cell enable */
#  define SYSCFG_COMPENSATION_DISABLE    (0 << SYSCFG_CPSCTL_CPS_EN_SHIFT)      /* I/O compensation cell disable */

#define SYSCFG_CPSCTL_CPS_RDY_SHIFT      (8)                                    /* Bit 8: I/O compensation cell is ready or not */
#define SYSCFG_CPSCTL_CPS_RDY_MASK       (1 << SYSCFG_CPSCTL_CPS_RDY_SHIFT)
#define SYSCFG_CPSCTL_CPS_RDY_SET        (1 << SYSCFG_CPSCTL_CPS_RDY_SHIFT)     /* Ready */
#define SYSCFG_CPSCTL_CPS_RDY_RESET      (0 << SYSCFG_CPSCTL_CPS_RDY_SHIFT)     /* Not ready */

#define SYSCFG_CPSCTL_IOSPDOP_SHIFT      (16)                                   /* Bit 16: I/O speed optimization */
#define SYSCFG_CPSCTL_IOSPDOP_MASK       (1 << SYSCFG_CPSCTL_IOSPDOP_SHIFT)
#define SYSCFG_CPSCTL_IOLV_SHIFT         (23)                                   /* Bit 23: I/O low voltage */
#define SYSCFG_CPSCTL_IOLV_MASK          (1 << SYSCFG_CPSCTL_IOLV_SHIFT)

/* I/O compensation cell configuration register */

#define SYSCFG_CPSCCCFG_NCPSCC_SHIFT     (0)                                    /* Bits 3:0: NMOS compensation cell code */
#define SYSCFG_CPSCCCFG_NCPSCC_MASK      (15 << SYSCFG_CPSCCCFG_NCPSCC_SHIFT)
#define SYSCFG_CPSCCCFG_PCPSCC_SHIFT     (4)                                    /* Bits 7:4: PMOS compensation cell code */
#define SYSCFG_CPSCCCFG_PCPSCC_MASK      (15 << SYSCFG_CPSCCCFG_PCPSCC_SHIFT)

/* Timer channel input selection register 0 */

#define SYSCFG_TIMER7_CI0_SEL_SHIFT      (0)                                    /* Bits 3:0: TIMER7 channel 0 input selection */
#define SYSCFG_TIMER7_CI0_SEL_MASK       (15 << SYSCFG_TIMER7_CI0_SEL_SHIFT)
#define SYSCFG_TIMER7_CI1_SEL_SHIFT      (4)                                    /* Bits 7:4: TIMER7 channel 1 input selection */
#define SYSCFG_TIMER7_CI1_SEL_MASK       (15 << SYSCFG_TIMER7_CI1_SEL_SHIFT)
#define SYSCFG_TIMER7_CI2_SEL_SHIFT      (8)                                    /* Bits 11:8: TIMER7 channel 2 input selection */
#define SYSCFG_TIMER7_CI2_SEL_MASK       (15 << SYSCFG_TIMER7_CI2_SEL_SHIFT)
#define SYSCFG_TIMER7_CI3_SEL_SHIFT      (12)                                   /* Bits 15:12: TIMER7 channel 3 input selection */
#define SYSCFG_TIMER7_CI3_SEL_MASK       (15 << SYSCFG_TIMER7_CI3_SEL_SHIFT)
#define SYSCFG_TIMER0_CI0_SEL_SHIFT      (16)                                   /* Bits 19:16: TIMER0 channel 0 input selection */
#define SYSCFG_TIMER0_CI0_SEL_MASK       (15 << SYSCFG_TIMER0_CI0_SEL_SHIFT)
#define SYSCFG_TIMER0_CI1_SEL_SHIFT      (20)                                   /* Bits 23:20: TIMER0 channel 1 input selection */
#define SYSCFG_TIMER0_CI1_SEL_MASK       (15 << SYSCFG_TIMER0_CI1_SEL_SHIFT)
#define SYSCFG_TIMER0_CI2_SEL_SHIFT      (24)                                   /* Bits 27:24: TIMER0 channel 2 input selection */
#define SYSCFG_TIMER0_CI2_SEL_MASK       (15 << SYSCFG_TIMER0_CI2_SEL_SHIFT)
#define SYSCFG_TIMER0_CI3_SEL_SHIFT      (28)                                   /* Bits 31:28: TIMER0 channel 3 input selection */
#define SYSCFG_TIMER0_CI3_SEL_MASK       (15 << SYSCFG_TIMER0_CI3_SEL_SHIFT)

/* Timer channel input selection register 1 */

#define SYSCFG_TIMER2_CI0_SEL_SHIFT      (0)                                    /* Bits 3:0: TIMER2 channel 0 input selection */
#define SYSCFG_TIMER2_CI0_SEL_MASK       (15 << SYSCFG_TIMER2_CI0_SEL_SHIFT)
#define SYSCFG_TIMER2_CI1_SEL_SHIFT      (4)                                    /* Bits 7:4: TIMER2 channel 1 input selection */
#define SYSCFG_TIMER2_CI1_SEL_MASK       (15 << SYSCFG_TIMER2_CI1_SEL_SHIFT)
#define SYSCFG_TIMER2_CI2_SEL_SHIFT      (8)                                    /* Bits 11:8: TIMER2 channel 2 input selection */
#define SYSCFG_TIMER2_CI2_SEL_MASK       (15 << SYSCFG_TIMER2_CI2_SEL_SHIFT)
#define SYSCFG_TIMER2_CI3_SEL_SHIFT      (12)                                   /* Bits 15:12: TIMER2 channel 3 input selection */
#define SYSCFG_TIMER2_CI3_SEL_MASK       (15 << SYSCFG_TIMER2_CI3_SEL_SHIFT)
#define SYSCFG_TIMER1_CI0_SEL_SHIFT      (16)                                   /* Bits 19:16: TIMER1 channel 0 input selection */
#define SYSCFG_TIMER1_CI0_SEL_MASK       (15 << SYSCFG_TIMER1_CI0_SEL_SHIFT)
#define SYSCFG_TIMER1_CI1_SEL_SHIFT      (20)                                   /* Bits 23:20: TIMER1 channel 1 input selection */
#define SYSCFG_TIMER1_CI1_SEL_MASK       (15 << SYSCFG_TIMER1_CI1_SEL_SHIFT)
#define SYSCFG_TIMER1_CI2_SEL_SHIFT      (24)                                   /* Bits 27:24: TIMER1 channel 2 input selection */
#define SYSCFG_TIMER1_CI2_SEL_MASK       (15 << SYSCFG_TIMER1_CI2_SEL_SHIFT)
#define SYSCFG_TIMER1_CI3_SEL_SHIFT      (28)                                   /* Bits 31:28: TIMER1 channel 3 input selection */
#define SYSCFG_TIMER1_CI3_SEL_MASK       (15 << SYSCFG_TIMER1_CI3_SEL_SHIFT)

/* Timer channel input selection register 2 */

#define SYSCFG_TIMER4_CI0_SEL_SHIFT      (0)                                    /* Bits 3:0: TIMER4 channel 0 input selection */
#define SYSCFG_TIMER4_CI0_SEL_MASK       (15 << SYSCFG_TIMER4_CI0_SEL_SHIFT)
#define SYSCFG_TIMER4_CI1_SEL_SHIFT      (4)                                    /* Bits 7:4: TIMER4 channel 1 input selection */
#define SYSCFG_TIMER4_CI1_SEL_MASK       (15 << SYSCFG_TIMER4_CI1_SEL_SHIFT)
#define SYSCFG_TIMER4_CI2_SEL_SHIFT      (8)                                    /* Bits 11:8: TIMER4 channel 2 input selection */
#define SYSCFG_TIMER4_CI2_SEL_MASK       (15 << SYSCFG_TIMER4_CI2_SEL_SHIFT)
#define SYSCFG_TIMER4_CI3_SEL_SHIFT      (12)                                   /* Bits 15:12: TIMER4 channel 3 input selection */
#define SYSCFG_TIMER4_CI3_SEL_MASK       (15 << SYSCFG_TIMER4_CI3_SEL_SHIFT)
#define SYSCFG_TIMER3_CI0_SEL_SHIFT      (16)                                   /* Bits 19:16: TIMER3 channel 0 input selection */
#define SYSCFG_TIMER3_CI0_SEL_MASK       (15 << SYSCFG_TIMER3_CI0_SEL_SHIFT)
#define SYSCFG_TIMER3_CI1_SEL_SHIFT      (20)                                   /* Bits 23:20: TIMER3 channel 1 input selection */
#define SYSCFG_TIMER3_CI1_SEL_MASK       (15 << SYSCFG_TIMER3_CI1_SEL_SHIFT)
#define SYSCFG_TIMER3_CI2_SEL_SHIFT      (24)                                   /* Bits 27:24: TIMER3 channel 2 input selection */
#define SYSCFG_TIMER3_CI2_SEL_MASK       (15 << SYSCFG_TIMER3_CI2_SEL_SHIFT)
#define SYSCFG_TIMER3_CI3_SEL_SHIFT      (28)                                   /* Bits 31:28: TIMER3 channel 3 input selection */
#define SYSCFG_TIMER3_CI3_SEL_MASK       (15 << SYSCFG_TIMER3_CI3_SEL_SHIFT)

/* Timer channel input selection register 3 */

#define SYSCFG_TIMER23_CI0_SEL_SHIFT     (0)                                    /* Bits 3:0: TIMER23 channel 0 input selection */
#define SYSCFG_TIMER23_CI0_SEL_MASK      (15 << SYSCFG_TIMER23_CI0_SEL_SHIFT)
#define SYSCFG_TIMER23_CI1_SEL_SHIFT     (4)                                    /* Bits 7:4: TIMER23 channel 1 input selection */
#define SYSCFG_TIMER23_CI1_SEL_MASK      (15 << SYSCFG_TIMER23_CI1_SEL_SHIFT)
#define SYSCFG_TIMER23_CI2_SEL_SHIFT     (8)                                    /* Bits 11:8: TIMER23 channel 2 input selection */
#define SYSCFG_TIMER23_CI2_SEL_MASK      (15 << SYSCFG_TIMER23_CI2_SEL_SHIFT)
#define SYSCFG_TIMER23_CI3_SEL_SHIFT     (12)                                   /* Bits 15:12: TIMER23 channel 3 input selection */
#define SYSCFG_TIMER23_CI3_SEL_MASK      (15 << SYSCFG_TIMER23_CI3_SEL_SHIFT)
#define SYSCFG_TIMER22_CI0_SEL_SHIFT     (16)                                   /* Bits 19:16: TIMER22 channel 0 input selection */
#define SYSCFG_TIMER22_CI0_SEL_MASK      (15 << SYSCFG_TIMER22_CI0_SEL_SHIFT)
#define SYSCFG_TIMER22_CI1_SEL_SHIFT     (20)                                   /* Bits 23:20: TIMER22 channel 1 input selection */
#define SYSCFG_TIMER22_CI1_SEL_MASK      (15 << SYSCFG_TIMER22_CI1_SEL_SHIFT)
#define SYSCFG_TIMER22_CI2_SEL_SHIFT     (24)                                   /* Bits 27:24: TIMER22 channel 2 input selection */
#define SYSCFG_TIMER22_CI2_SEL_MASK      (15 << SYSCFG_TIMER22_CI2_SEL_SHIFT)
#define SYSCFG_TIMER22_CI3_SEL_SHIFT     (28)                                   /* Bits 31:28: TIMER22 channel 3 input selection */
#define SYSCFG_TIMER22_CI3_SEL_MASK      (15 << SYSCFG_TIMER22_CI3_SEL_SHIFT)

/* Timer channel input selection register 4 */

#define SYSCFG_TIMER31_CI0_SEL_SHIFT     (0)                                    /* Bits 3:0: TIMER31 channel 0 input selection */
#define SYSCFG_TIMER31_CI0_SEL_MASK      (15 << SYSCFG_TIMER31_CI0_SEL_SHIFT)
#define SYSCFG_TIMER31_CI1_SEL_SHIFT     (4)                                    /* Bits 7:4: TIMER31 channel 1 input selection */
#define SYSCFG_TIMER31_CI1_SEL_MASK      (15 << SYSCFG_TIMER31_CI1_SEL_SHIFT)
#define SYSCFG_TIMER31_CI2_SEL_SHIFT     (8)                                    /* Bits 11:8: TIMER31 channel 2 input selection */
#define SYSCFG_TIMER31_CI2_SEL_MASK      (15 << SYSCFG_TIMER31_CI2_SEL_SHIFT)
#define SYSCFG_TIMER31_CI3_SEL_SHIFT     (12)                                   /* Bits 15:12: TIMER31 channel 3 input selection */
#define SYSCFG_TIMER31_CI3_SEL_MASK      (15 << SYSCFG_TIMER31_CI3_SEL_SHIFT)
#define SYSCFG_TIMER30_CI0_SEL_SHIFT     (16)                                   /* Bits 19:16: TIMER30 channel 0 input selection */
#define SYSCFG_TIMER30_CI0_SEL_MASK      (15 << SYSCFG_TIMER30_CI0_SEL_SHIFT)
#define SYSCFG_TIMER30_CI1_SEL_SHIFT     (20)                                   /* Bits 23:20: TIMER30 channel 1 input selection */
#define SYSCFG_TIMER30_CI1_SEL_MASK      (15 << SYSCFG_TIMER30_CI1_SEL_SHIFT)
#define SYSCFG_TIMER30_CI2_SEL_SHIFT     (24)                                   /* Bits 27:24: TIMER30 channel 2 input selection */
#define SYSCFG_TIMER30_CI2_SEL_MASK      (15 << SYSCFG_TIMER30_CI2_SEL_SHIFT)
#define SYSCFG_TIMER30_CI3_SEL_SHIFT     (28)                                   /* Bits 31:28: TIMER30 channel 3 input selection */
#define SYSCFG_TIMER30_CI3_SEL_MASK      (15 << SYSCFG_TIMER30_CI3_SEL_SHIFT)

/* Timer channel input selection register 5 */

#define SYSCFG_TIMER14_CI0_SEL_SHIFT     (0)                                    /* Bits 3:0: TIMER14 channel 0 input selection */
#define SYSCFG_TIMER14_CI0_SEL_MASK      (15 << SYSCFG_TIMER14_CI0_SEL_SHIFT)
#define SYSCFG_TIMER14_CI1_SEL_SHIFT     (4)                                    /* Bits 7:4: TIMER14 channel 1 input selection */
#define SYSCFG_TIMER14_CI1_SEL_MASK      (15 << SYSCFG_TIMER14_CI1_SEL_SHIFT)
#define SYSCFG_TIMER40_CI0_SEL_SHIFT     (8)                                    /* Bits 11:8: TIMER40 channel 0 input selection */
#define SYSCFG_TIMER40_CI0_SEL_MASK      (15 << SYSCFG_TIMER40_CI0_SEL_SHIFT)
#define SYSCFG_TIMER40_CI1_SEL_SHIFT     (12)                                   /* Bits 15:12: TIMER40 channel 1 input selection */
#define SYSCFG_TIMER40_CI1_SEL_MASK      (15 << SYSCFG_TIMER40_CI1_SEL_SHIFT)
#define SYSCFG_TIMER41_CI0_SEL_SHIFT     (16)                                   /* Bits 19:16: TIMER41 channel 0 input selection */
#define SYSCFG_TIMER41_CI0_SEL_MASK      (15 << SYSCFG_TIMER41_CI0_SEL_SHIFT)
#define SYSCFG_TIMER41_CI1_SEL_SHIFT     (20)                                   /* Bits 23:20: TIMER41 channel 1 input selection */
#define SYSCFG_TIMER41_CI1_SEL_MASK      (15 << SYSCFG_TIMER41_CI1_SEL_SHIFT)
#define SYSCFG_TIMER42_CI0_SEL_SHIFT     (24)                                   /* Bits 27:24: TIMER42 channel 0 input selection */
#define SYSCFG_TIMER42_CI0_SEL_MASK      (15 << SYSCFG_TIMER42_CI0_SEL_SHIFT)
#define SYSCFG_TIMER42_CI1_SEL_SHIFT     (28)                                   /* Bits 31:28: TIMER42 channel 1 input selection */
#define SYSCFG_TIMER42_CI1_SEL_MASK      (15 << SYSCFG_TIMER42_CI1_SEL_SHIFT)

/* Timer channel input selection register 6 */

#define SYSCFG_TIMER15_CI0_SEL_SHIFT     (0)                                    /* Bits 3:0: TIMER15 channel 0 input selection */
#define SYSCFG_TIMER15_CI0_SEL_MASK      (15 << SYSCFG_TIMER15_CI0_SEL_SHIFT)
#define SYSCFG_TIMER16_CI0_SEL_SHIFT     (4)                                    /* Bits 7:4: TIMER16 channel 0 input selection */
#define SYSCFG_TIMER16_CI0_SEL_MASK      (15 << SYSCFG_TIMER16_CI0_SEL_SHIFT)
#define SYSCFG_TIMER43_CI0_SEL_SHIFT     (8)                                    /* Bits 11:8: TIMER43 channel 0 input selection */
#define SYSCFG_TIMER43_CI0_SEL_MASK      (15 << SYSCFG_TIMER43_CI0_SEL_SHIFT)
#define SYSCFG_TIMER43_CI1_SEL_SHIFT     (12)                                   /* Bits 15:12: TIMER43 channel 1 input selection */
#define SYSCFG_TIMER43_CI1_SEL_MASK      (15 << SYSCFG_TIMER43_CI1_SEL_SHIFT)
#define SYSCFG_TIMER44_CI0_SEL_SHIFT     (16)                                   /* Bits 19:16: TIMER44 channel 0 input selection */
#define SYSCFG_TIMER44_CI0_SEL_MASK      (15 << SYSCFG_TIMER44_CI0_SEL_SHIFT)
#define SYSCFG_TIMER44_CI1_SEL_SHIFT     (20)                                   /* Bits 23:20: TIMER44 channel 1 input selection */
#define SYSCFG_TIMER44_CI1_SEL_MASK      (15 << SYSCFG_TIMER44_CI1_SEL_SHIFT)

/* CPU ICACHE error status register */

#define SYSCFG_CPUICAC_CPU_ICERR_SHIFT   (6)                                    /* Bits 27:6: CPU ICACHE error */
#define SYSCFG_CPUICAC_CPU_ICERR_MASK    (0x3fffff << SYSCFG_CPUICAC_CPU_ICERR_SHIFT)
#define SYSCFG_CPUICAC_CPU_ICDET_SHIFT   (28)                                   /* Bits 31:28: CPU ICACHE error detect */
#define SYSCFG_CPUICAC_CPU_ICDET_MASK    (15 << SYSCFG_CPUICAC_CPU_ICDET_SHIFT)

/* CPU DCACHE error status register */

#define SYSCFG_CPUDCAC_CPU_DCERR_SHIFT   (6)                                    /* Bits 27:6: CPU DCACHE error */
#define SYSCFG_CPUDCAC_CPU_DCERR_MASK    (0x3fffff << SYSCFG_CPUDCAC_CPU_DCERR_SHIFT)
#define SYSCFG_CPUDCAC_CPU_DCDET_SHIFT   (28)                                   /* Bits 31:28: CPU DCACHE error detect */
#define SYSCFG_CPUDCAC_CPU_DCDET_MASK    (15 << SYSCFG_CPUDCAC_CPU_DCDET_SHIFT)

/* FPU interrupt enable register */

#define SYSCFG_FPUINTEN_IOPIE_SHIFT      (0)                                    /* Bit 0: Invalid operation interrupt enable */
#define SYSCFG_FPUINTEN_IOPIE_MASK       (1 << SYSCFG_FPUINTEN_IOPIE_SHIFT)
#define SYSCFG_FPUINTEN_DZIE_SHIFT       (1)                                    /* Bit 1: Divide by zero interrupt enable */
#define SYSCFG_FPUINTEN_DZIE_MASK        (1 << SYSCFG_FPUINTEN_DZIE_SHIFT)
#define SYSCFG_FPUINTEN_UFIE_SHIFT       (2)                                    /* Bit 2: Underflow interrupt enable */
#define SYSCFG_FPUINTEN_UFIE_MASK        (1 << SYSCFG_FPUINTEN_UFIE_SHIFT)
#define SYSCFG_FPUINTEN_OVFIE_SHIFT      (3)                                    /* Bit 3: Overflow interrupt enable */
#define SYSCFG_FPUINTEN_OVFIE_MASK       (1 << SYSCFG_FPUINTEN_OVFIE_SHIFT)
#define SYSCFG_FPUINTEN_IDIE_SHIFT       (4)                                    /* Bit 4: Input denormal interrupt enable */
#define SYSCFG_FPUINTEN_IDIE_MASK        (1 << SYSCFG_FPUINTEN_IDIE_SHIFT)
#define SYSCFG_FPUINTEN_IXIE_SHIFT       (5)                                    /* Bit 5: Inexact interrupt enable */
#define SYSCFG_FPUINTEN_IXIE_MASK        (1 << SYSCFG_FPUINTEN_IXIE_SHIFT)

/* SRAM configuration register 0 */

#define SYSCFG_SRAMCFG0_SECURE_SRAM_SIZE_SHIFT  (0)                             /* Bits 1:0: Secure SRAM size */
#define SYSCFG_SRAMCFG0_SECURE_SRAM_SIZE_MASK   (3 << SYSCFG_SRAMCFG0_SECURE_SRAM_SIZE_SHIFT)

/* SRAM configuration register 1 */

#define SYSCFG_SRAMCFG1_TCM_WAITSTATE_SHIFT     (0)                             /* Bit 0: TCM wait state */
#define SYSCFG_SRAMCFG1_TCM_WAITSTATE           (1 << SYSCFG_SRAMCFG1_TCM_WAITSTATE_SHIFT)

/* Timer trigger configuration registers */

#define SYSCFG_TIMERCFG_TSCFG0_SHIFT     (0)                                    /* Bits 4:0: Timer trigger selection configuration 0 */
#define SYSCFG_TIMERCFG_TSCFG0_MASK      (31 << SYSCFG_TIMERCFG_TSCFG0_SHIFT)
#define SYSCFG_TIMERCFG_TSCFG1_SHIFT     (5)                                    /* Bits 9:5: Timer trigger selection configuration 1 */
#define SYSCFG_TIMERCFG_TSCFG1_MASK      (31 << SYSCFG_TIMERCFG_TSCFG1_SHIFT)
#define SYSCFG_TIMERCFG_TSCFG2_SHIFT     (10)                                   /* Bits 14:10: Timer trigger selection configuration 2 */
#define SYSCFG_TIMERCFG_TSCFG2_MASK      (31 << SYSCFG_TIMERCFG_TSCFG2_SHIFT)
#define SYSCFG_TIMERCFG_TSCFG3_SHIFT     (16)                                   /* Bits 20:16: Timer trigger selection configuration 3 */
#define SYSCFG_TIMERCFG_TSCFG3_MASK      (31 << SYSCFG_TIMERCFG_TSCFG3_SHIFT)
#define SYSCFG_TIMERCFG_TSCFG4_SHIFT     (21)                                   /* Bits 25:21: Timer trigger selection configuration 4 */
#define SYSCFG_TIMERCFG_TSCFG4_MASK      (31 << SYSCFG_TIMERCFG_TSCFG4_SHIFT)
#define SYSCFG_TIMERCFG_TSCFG5_SHIFT     (26)                                   /* Bits 30:26: Timer trigger selection configuration 5 */
#define SYSCFG_TIMERCFG_TSCFG5_MASK      (31 << SYSCFG_TIMERCFG_TSCFG5_SHIFT)

/* User configuration register */

#define SYSCFG_USERCFG_BOR_TH_SHIFT      (0)                                    /* Bits 1:0: BOR threshold */
#define SYSCFG_USERCFG_BOR_TH_MASK       (3 << SYSCFG_USERCFG_BOR_TH_SHIFT)
#define SYSCFG_USERCFG_BOOT_MODE_SHIFT   (4)                                    /* Bits 6:4: Boot mode */
#define SYSCFG_USERCFG_BOOT_MODE_MASK    (7 << SYSCFG_USERCFG_BOOT_MODE_SHIFT)

/* EXTI source select definition */
#define SYSCFG_EXTISS0                   (0x00)                                 /* EXTI source select GPIOx pin 0~3 */
#define SYSCFG_EXTISS1                   (0x01)                                 /* EXTI source select GPIOx pin 4~7 */
#define SYSCFG_EXTISS2                   (0x02)                                 /* EXTI source select GPIOx pin 8~11 */
#define SYSCFG_EXTISS3                   (0x03)                                 /* EXTI source select GPIOx pin 12~15 */

/* EXTI source select mask bits definition */
#define SYSCFG_EXTI_SS_MASK              (15)                                   /* EXTI source select mask */

/* EXTI source select jumping step definition */
#define SYSCFG_EXTI_SS_JSTEP             (0x04)                                 /* EXTI source select jumping step */

/* EXTI source select moving step definition */
#define SYSCFG_EXTI_SS_MSTEP(pin)        (SYSCFG_EXTI_SS_JSTEP*((pin)%SYSCFG_EXTI_SS_JSTEP))   /* EXTI source select moving step */

#define EXTI_SOURCE_GPIOA                (0)                                    /* EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOB                (1)                                    /* EXTI GPIOB configuration */
#define EXTI_SOURCE_GPIOC                (2)                                    /* EXTI GPIOC configuration */
#define EXTI_SOURCE_GPIOD                (3)                                    /* EXTI GPIOD configuration */
#define EXTI_SOURCE_GPIOE                (4)                                    /* EXTI GPIOE configuration */
#define EXTI_SOURCE_GPIOF                (5)                                    /* EXTI GPIOF configuration */
#define EXTI_SOURCE_GPIOG                (6)                                    /* EXTI GPIOG configuration */
#define EXTI_SOURCE_GPIOH                (7)                                    /* EXTI GPIOH configuration */
#define EXTI_SOURCE_GPIOJ                (9)                                    /* EXTI GPIOJ configuration */
#define EXTI_SOURCE_GPIOK                (10)                                   /* EXTI GPIOK configuration */

#endif /* __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_SYSCFG_H */