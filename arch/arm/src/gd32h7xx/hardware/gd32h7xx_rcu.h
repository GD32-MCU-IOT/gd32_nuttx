/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h7xx_rcu.h
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

#ifndef __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_RCU_H
#define __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_RCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_RCU_CTL_OFFSET            0x0000  /* Control register offset */
#define GD32_RCU_PLL0_OFFSET           0x0004  /* PLL0 register */
#define GD32_RCU_CFG0_OFFSET           0x0008  /* Clock configuration register offset */
#define GD32_RCU_INT_OFFSET            0x000c  /* Clock interrupt register offset */
#define GD32_RCU_AHB1RST_OFFSET        0x0010  /* AHB1 reset register offset */
#define GD32_RCU_AHB2RST_OFFSET        0x0014  /* AHB2 reset register offset */
#define GD32_RCU_AHB3RST_OFFSET        0x0018  /* AHB3 reset register offset */
#define GD32_RCU_AHB4RST_OFFSET        0x001c  /* AHB4 reset register offset */
#define GD32_RCU_APB1RST_OFFSET        0x0020  /* APB1 reset register offset */
#define GD32_RCU_APB2RST_OFFSET        0x0024  /* APB2 reset register offset */
#define GD32_RCU_APB3RST_OFFSET        0x0028  /* APB3 reset register offset */
#define GD32_RCU_APB4RST_OFFSET        0x002c  /* APB4 reset register offset */
#define GD32_RCU_AHB1EN_OFFSET         0x0030  /* AHB1 enable register offset */
#define GD32_RCU_AHB2EN_OFFSET         0x0034  /* AHB2 enable register offset */
#define GD32_RCU_AHB3EN_OFFSET         0x0038  /* AHB3 enable register offset */
#define GD32_RCU_AHB4EN_OFFSET         0x003c  /* AHB4 enable register offset */
#define GD32_RCU_APB1EN_OFFSET         0x0040  /* APB1 enable register offset */
#define GD32_RCU_APB2EN_OFFSET         0x0044  /* APB2 enable register offset */
#define GD32_RCU_APB3EN_OFFSET         0x0048  /* APB3 enable register offset */
#define GD32_RCU_APB4EN_OFFSET         0x004c  /* APB4 enable register offset */
#define GD32_RCU_AHB1SPEN_OFFSET       0x0050  /* AHB1 sleep mode enable register offset */
#define GD32_RCU_AHB2SPEN_OFFSET       0x0054  /* AHB2 sleep mode enable register offset */
#define GD32_RCU_AHB3SPEN_OFFSET       0x0058  /* AHB3 sleep mode enable register offset */
#define GD32_RCU_AHB4SPEN_OFFSET       0x005c  /* AHB4 sleep mode enable register offset */
#define GD32_RCU_APB1SPEN_OFFSET       0x0060  /* APB1 sleep mode enable register offset */
#define GD32_RCU_APB2SPEN_OFFSET       0x0064  /* APB2 sleep mode enable register offset */
#define GD32_RCU_APB3SPEN_OFFSET       0x0068  /* APB3 sleep mode enable register offset */
#define GD32_RCU_APB4SPEN_OFFSET       0x006c  /* APB4 sleep mode enable register offset */
#define GD32_RCU_BDCTL_OFFSET          0x0070  /* Backup domain control register offset */
#define GD32_RCU_RSTSCK_OFFSET         0x0074  /* Reset source / clock register offset */
#define GD32_RCU_PLLADDCTL_OFFSET      0x0080  /* PLL clock additional control register offset */
#define GD32_RCU_PLL1_OFFSET           0x0084  /* PLL1 register offset */
#define GD32_RCU_PLL2_OFFSET           0x0088  /* PLL2 register offset */
#define GD32_RCU_CFG1_OFFSET           0x008c  /* Clock configuration register 1 offset */
#define GD32_RCU_CFG2_OFFSET           0x0090  /* Clock configuration register 2 offset */
#define GD32_RCU_CFG3_OFFSET           0x0094  /* Clock configuration register 3 offset */
#define GD32_RCU_PLLALL_OFFSET         0x0098  /* PLL configuration register offset */
#define GD32_RCU_PLL0FRA_OFFSET        0x009c  /* PLL0 fraction configuration offset */
#define GD32_RCU_PLL1FRA_OFFSET        0x00a0  /* PLL1 fraction configuration offset */
#define GD32_RCU_PLL2FRA_OFFSET        0x00a4  /* PLL2 fraction configuration offset */
#define GD32_RCU_ADDCTL0_OFFSET        0x00c0  /* Additional clock control register 0 offset */
#define GD32_RCU_ADDCTL1_OFFSET        0x00c4  /* Additional clock control register 1 offset */
#define GD32_RCU_ADDINT_OFFSET         0x00cc  /* Additional clock interrupt register offset */
#define GD32_RCU_CFG4_OFFSET           0x00d0  /* Clock configuration register 4 offset */
#define GD32_RCU_USBCLKCTL_OFFSET      0x00d4  /* USB clock control register offset */
#define GD32_RCU_PLLUSBCFG_OFFSET      0x00d8  /* PLLUSB configuration register offset */
#define GD32_RCU_ADDAPB2RST_OFFSET     0x00e0  /* APB2 additional reset register offset */
#define GD32_RCU_ADDAPB2EN_OFFSET      0x00e4  /* APB2 additional enable register offset */
#define GD32_RCU_ADDAPB2SPEN_OFFSET    0x00e8  /* APB2 additional sleep mode enable register offset */
#define GD32_RCU_CFG5_OFFSET           0x00f0  /* Clock configuration register 5 offset */

/* Register Addresses *******************************************************/

#define GD32_RCU_CTL                   (GD32_RCU_BASE+GD32_RCU_CTL_OFFSET)          /* Control register */
#define GD32_RCU_PLL0                  (GD32_RCU_BASE+GD32_RCU_PLL0_OFFSET)         /* PLL0 register */
#define GD32_RCU_CFG0                  (GD32_RCU_BASE+GD32_RCU_CFG0_OFFSET)         /* Clock configuration register */
#define GD32_RCU_INT                   (GD32_RCU_BASE+GD32_RCU_INT_OFFSET)          /* Clock interrupt register */
#define GD32_RCU_AHB1RST               (GD32_RCU_BASE+GD32_RCU_AHB1RST_OFFSET)      /* AHB1 reset register */
#define GD32_RCU_AHB2RST               (GD32_RCU_BASE+GD32_RCU_AHB2RST_OFFSET)      /* AHB2 reset register */
#define GD32_RCU_AHB3RST               (GD32_RCU_BASE+GD32_RCU_AHB3RST_OFFSET)      /* AHB3 reset register */
#define GD32_RCU_AHB4RST               (GD32_RCU_BASE+GD32_RCU_AHB4RST_OFFSET)      /* AHB4 reset register */
#define GD32_RCU_APB1RST               (GD32_RCU_BASE+GD32_RCU_APB1RST_OFFSET)      /* APB1 reset register */
#define GD32_RCU_APB2RST               (GD32_RCU_BASE+GD32_RCU_APB2RST_OFFSET)      /* APB2 reset register */
#define GD32_RCU_APB3RST               (GD32_RCU_BASE+GD32_RCU_APB3RST_OFFSET)      /* APB3 reset register */
#define GD32_RCU_APB4RST               (GD32_RCU_BASE+GD32_RCU_APB4RST_OFFSET)      /* APB4 reset register */
#define GD32_RCU_AHB1EN                (GD32_RCU_BASE+GD32_RCU_AHB1EN_OFFSET)       /* AHB1 enable register */
#define GD32_RCU_AHB2EN                (GD32_RCU_BASE+GD32_RCU_AHB2EN_OFFSET)       /* AHB2 enable register */
#define GD32_RCU_AHB3EN                (GD32_RCU_BASE+GD32_RCU_AHB3EN_OFFSET)       /* AHB3 enable register */
#define GD32_RCU_AHB4EN                (GD32_RCU_BASE+GD32_RCU_AHB4EN_OFFSET)       /* AHB4 enable register */
#define GD32_RCU_APB1EN                (GD32_RCU_BASE+GD32_RCU_APB1EN_OFFSET)       /* APB1 enable register */
#define GD32_RCU_APB2EN                (GD32_RCU_BASE+GD32_RCU_APB2EN_OFFSET)       /* APB2 enable register */
#define GD32_RCU_APB3EN                (GD32_RCU_BASE+GD32_RCU_APB3EN_OFFSET)       /* APB3 enable register */
#define GD32_RCU_APB4EN                (GD32_RCU_BASE+GD32_RCU_APB4EN_OFFSET)       /* APB4 enable register */
#define GD32_RCU_AHB1SPEN              (GD32_RCU_BASE+GD32_RCU_AHB1SPEN_OFFSET)     /* AHB1 sleep mode enable register */
#define GD32_RCU_AHB2SPEN              (GD32_RCU_BASE+GD32_RCU_AHB2SPEN_OFFSET)     /* AHB2 sleep mode enable register */
#define GD32_RCU_AHB3SPEN              (GD32_RCU_BASE+GD32_RCU_AHB3SPEN_OFFSET)     /* AHB3 sleep mode enable register */
#define GD32_RCU_AHB4SPEN              (GD32_RCU_BASE+GD32_RCU_AHB4SPEN_OFFSET)     /* AHB4 sleep mode enable register */
#define GD32_RCU_APB1SPEN              (GD32_RCU_BASE+GD32_RCU_APB1SPEN_OFFSET)     /* APB1 sleep mode enable register */
#define GD32_RCU_APB2SPEN              (GD32_RCU_BASE+GD32_RCU_APB2SPEN_OFFSET)     /* APB2 sleep mode enable register */
#define GD32_RCU_APB3SPEN              (GD32_RCU_BASE+GD32_RCU_APB3SPEN_OFFSET)     /* APB3 sleep mode enable register */
#define GD32_RCU_APB4SPEN              (GD32_RCU_BASE+GD32_RCU_APB4SPEN_OFFSET)     /* APB4 sleep mode enable register */
#define GD32_RCU_BDCTL                 (GD32_RCU_BASE+GD32_RCU_BDCTL_OFFSET)        /* Backup domain control register */
#define GD32_RCU_RSTSCK                (GD32_RCU_BASE+GD32_RCU_RSTSCK_OFFSET)       /* Reset source / clock register */
#define GD32_RCU_PLLADDCTL             (GD32_RCU_BASE+GD32_RCU_PLLADDCTL_OFFSET)    /* PLL clock additional control register */
#define GD32_RCU_PLL1                  (GD32_RCU_BASE+GD32_RCU_PLL1_OFFSET)         /* PLL1 register */
#define GD32_RCU_PLL2                  (GD32_RCU_BASE+GD32_RCU_PLL2_OFFSET)         /* PLL2 register */
#define GD32_RCU_CFG1                  (GD32_RCU_BASE+GD32_RCU_CFG1_OFFSET)         /* Clock configuration register 1 */
#define GD32_RCU_CFG2                  (GD32_RCU_BASE+GD32_RCU_CFG2_OFFSET)         /* Clock configuration register 2 */
#define GD32_RCU_CFG3                  (GD32_RCU_BASE+GD32_RCU_CFG3_OFFSET)         /* Clock configuration register 3 */
#define GD32_RCU_PLLALL                (GD32_RCU_BASE+GD32_RCU_PLLALL_OFFSET)       /* PLL configuration register */
#define GD32_RCU_PLL0FRA               (GD32_RCU_BASE+GD32_RCU_PLL0FRA_OFFSET)      /* PLL0 fraction configuration */
#define GD32_RCU_PLL1FRA               (GD32_RCU_BASE+GD32_RCU_PLL1FRA_OFFSET)      /* PLL1 fraction configuration */
#define GD32_RCU_PLL2FRA               (GD32_RCU_BASE+GD32_RCU_PLL2FRA_OFFSET)      /* PLL2 fraction configuration */
#define GD32_RCU_ADDCTL0               (GD32_RCU_BASE+GD32_RCU_ADDCTL0_OFFSET)      /* Additional clock control register 0 */
#define GD32_RCU_ADDCTL1               (GD32_RCU_BASE+GD32_RCU_ADDCTL1_OFFSET)      /* Additional clock control register 1 */
#define GD32_RCU_ADDINT                (GD32_RCU_BASE+GD32_RCU_ADDINT_OFFSET)       /* Additional clock interrupt register */
#define GD32_RCU_CFG4                  (GD32_RCU_BASE+GD32_RCU_CFG4_OFFSET)         /* Clock configuration register 4 */
#define GD32_RCU_USBCLKCTL             (GD32_RCU_BASE+GD32_RCU_USBCLKCTL_OFFSET)    /* USB clock control register */
#define GD32_RCU_PLLUSBCFG             (GD32_RCU_BASE+GD32_RCU_PLLUSBCFG_OFFSET)    /* PLLUSB configuration register */
#define GD32_RCU_ADDAPB2RST            (GD32_RCU_BASE+GD32_RCU_ADDAPB2RST_OFFSET)   /* APB2 additional reset register */
#define GD32_RCU_ADDAPB2EN             (GD32_RCU_BASE+GD32_RCU_ADDAPB2EN_OFFSET)    /* APB2 additional enable register */
#define GD32_RCU_ADDAPB2SPEN           (GD32_RCU_BASE+GD32_RCU_ADDAPB2SPEN_OFFSET)  /* APB2 additional sleep mode enable register */
#define GD32_RCU_CFG5                  (GD32_RCU_BASE+GD32_RCU_CFG5_OFFSET)         /* Clock configuration register 5 */

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCU_CTL_IRC64MADJ_SHIFT        (0)         /* Bits 0-6: IRC64M oscillator clock trim adjust value */
#define RCU_CTL_IRC64MADJ_MASK         (0x7f << RCU_CTL_IRC64MADJ_SHIFT)
#define RCU_CTL_IRC64MCALIB_SHIFT      (7)         /* Bits 7-15: IRC64M oscillator calibration value register */
#define RCU_CTL_IRC64MCALIB_MASK       (0x1ff << RCU_CTL_IRC64MCALIB_SHIFT)
#define RCU_CTL_HXTALEN                (1 << 16)   /* Bit 16: HXTAL oscillator enable */
#define RCU_CTL_HXTALSTB               (1 << 17)   /* Bit 17: HXTAL oscillator stabilization flag */
#define RCU_CTL_HXTALBPS               (1 << 18)   /* Bit 18: HXTAL oscillator bypass mode enable */
#define RCU_CTL_CKMEN                  (1 << 19)   /* Bit 19: HXTAL oscillator clock monitor enable */
#define RCU_CTL_PLL0EN                 (1 << 24)   /* Bit 24: PLL0 enable */
#define RCU_CTL_PLL0STB                (1 << 25)   /* Bit 25: PLL0 stabilization flag */
#define RCU_CTL_PLL1EN                 (1 << 26)   /* Bit 26: PLL1 enable */
#define RCU_CTL_PLL1STB                (1 << 27)   /* Bit 27: PLL1 stabilization flag */
#define RCU_CTL_PLL2EN                 (1 << 28)   /* Bit 28: PLL2 enable */
#define RCU_CTL_PLL2STB                (1 << 29)   /* Bit 29: PLL2 stabilization flag */
#define RCU_CTL_IRC64MEN               (1 << 30)   /* Bit 30: IRC64M high speed oscillator enable */
#define RCU_CTL_IRC64MSTB              (1 << 31)   /* Bit 31: IRC64M high speed internal oscillator stabilization flag */

/* PLL0 register */

#define RCU_PLL0_PLL0PSC_SHIFT         (0)         /* Bits 0-5: PLL0 VCO source clock prescaler */
#define RCU_PLL0_PLL0PSC_MASK          (0x3f << RCU_PLL0_PLL0PSC_SHIFT)
#  define RCU_PLL0_PLL0PSC(n)          ((n) << RCU_PLL0_PLL0PSC_SHIFT)
#define RCU_PLL0_PLL0N_SHIFT           (6)         /* Bits 6-14: PLL0 VCO clock multi factor */
#define RCU_PLL0_PLL0N_MASK            (0x1ff << RCU_PLL0_PLL0N_SHIFT)
#  define RCU_PLL0_PLL0N(n)            ((n) << RCU_PLL0_PLL0N_SHIFT)
#define RCU_PLL0_PLL0P_SHIFT           (16)        /* Bits 16-22: PLL0P output frequency division factor */
#define RCU_PLL0_PLL0P_MASK            (0x7f << RCU_PLL0_PLL0P_SHIFT)
#  define RCU_PLL0_PLL0P(n)            ((n) << RCU_PLL0_PLL0P_SHIFT)
#define RCU_PLL0_PLL0R_SHIFT           (24)        /* Bits 24-30: PLL0R output frequency division factor */
#define RCU_PLL0_PLL0R_MASK            (0x7f << RCU_PLL0_PLL0R_SHIFT)
#  define RCU_PLL0_PLL0R(n)            ((n) << RCU_PLL0_PLL0R_SHIFT)
#define RCU_PLL0_PLLSTBSRC             (1 << 31)   /* Bit 31: PLLs stabilization signal sources */

/* Clock configuration register 0 */

#define RCU_CFG0_SCS_SHIFT             (0)         /* Bits 0-1: System clock switch */
#define RCU_CFG0_SCS_MASK              (3 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_IRC64M          (0 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_HXTAL           (1 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_LPIRC4M         (2 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_PLL0P           (3 << RCU_CFG0_SCS_SHIFT)
#define RCU_CFG0_SCSS_SHIFT            (2)         /* Bits 2-3: System clock switch status */
#define RCU_CFG0_SCSS_MASK             (3 << RCU_CFG0_SCSS_SHIFT)
#  define RCU_CFG0_SCSS_IRC64M         (0 << RCU_CFG0_SCSS_SHIFT)
#  define RCU_CFG0_SCSS_HXTAL          (1 << RCU_CFG0_SCSS_SHIFT)
#  define RCU_CFG0_SCSS_LPIRC4M        (2 << RCU_CFG0_SCSS_SHIFT)
#  define RCU_CFG0_SCSS_PLL0P          (3 << RCU_CFG0_SCSS_SHIFT)
#define RCU_CFG0_AHBPSC_SHIFT          (4)         /* Bits 4-7: AHB prescaler selection */
#define RCU_CFG0_AHBPSC_MASK           (0xf << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV1   (0 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV2   (8 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV4   (9 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV8   (10 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV16  (11 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV64  (12 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV128 (13 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV256 (14 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV512 (15 << RCU_CFG0_AHBPSC_SHIFT)
#define RCU_CFG0_APB1PSC_SHIFT         (10)        /* Bits 10-12: APB1 prescaler selection */
#define RCU_CFG0_APB1PSC_MASK          (7 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_CKAHB_DIV1  (0 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_CKAHB_DIV2  (4 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_CKAHB_DIV4  (5 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_CKAHB_DIV8  (6 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_CKAHB_DIV16 (7 << RCU_CFG0_APB1PSC_SHIFT)
#define RCU_CFG0_APB2PSC_SHIFT         (13)        /* Bits 13-15: APB2 prescaler selection */
#define RCU_CFG0_APB2PSC_MASK          (7 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_CKAHB_DIV1  (0 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_CKAHB_DIV2  (4 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_CKAHB_DIV4  (5 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_CKAHB_DIV8  (6 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_CKAHB_DIV16 (7 << RCU_CFG0_APB2PSC_SHIFT)
#define RCU_CFG0_RTCDIV_SHIFT          (16)        /* Bits 16-21: RTC clock divider factor */
#define RCU_CFG0_RTCDIV_MASK           (0x3f << RCU_CFG0_RTCDIV_SHIFT)
#  define RCU_CFG0_RTCDIV(n)           ((n) << RCU_CFG0_RTCDIV_SHIFT)
#define RCU_CFG0_APB4PSC_SHIFT         (24)        /* Bits 24-26: APB4 prescaler selection */
#define RCU_CFG0_APB4PSC_MASK          (7 << RCU_CFG0_APB4PSC_SHIFT)
#  define RCU_CFG0_APB4PSC_CKAHB_DIV1  (0 << RCU_CFG0_APB4PSC_SHIFT)
#  define RCU_CFG0_APB4PSC_CKAHB_DIV2  (4 << RCU_CFG0_APB4PSC_SHIFT)
#  define RCU_CFG0_APB4PSC_CKAHB_DIV4  (5 << RCU_CFG0_APB4PSC_SHIFT)
#  define RCU_CFG0_APB4PSC_CKAHB_DIV8  (6 << RCU_CFG0_APB4PSC_SHIFT)
#  define RCU_CFG0_APB4PSC_CKAHB_DIV16 (7 << RCU_CFG0_APB4PSC_SHIFT)
#define RCU_CFG0_APB3PSC_SHIFT         (27)        /* Bits 27-29: APB3 prescaler selection */
#define RCU_CFG0_APB3PSC_MASK          (7 << RCU_CFG0_APB3PSC_SHIFT)
#  define RCU_CFG0_APB3PSC_CKAHB_DIV1  (0 << RCU_CFG0_APB3PSC_SHIFT)
#  define RCU_CFG0_APB3PSC_CKAHB_DIV2  (4 << RCU_CFG0_APB3PSC_SHIFT)
#  define RCU_CFG0_APB3PSC_CKAHB_DIV4  (5 << RCU_CFG0_APB3PSC_SHIFT)
#  define RCU_CFG0_APB3PSC_CKAHB_DIV8 (6 << RCU_CFG0_APB3PSC_SHIFT)
#  define RCU_CFG0_APB3PSC_CKAHB_DIV16 (7 << RCU_CFG0_APB3PSC_SHIFT)
#define RCU_CFG0_I2C0SEL_SHIFT         (30)        /* Bits 30-31: I2C0 clock source selection */
#define RCU_CFG0_I2C0SEL_MASK          (3 << RCU_CFG0_I2C0SEL_SHIFT)

/* Clock interrupt register */

#define RCU_INT_IRC32KSTBIF            (1 << 0)    /* Bit 0: IRC32K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF             (1 << 1)    /* Bit 1: LXTAL stabilization interrupt flag */
#define RCU_INT_IRC64MSTBIF            (1 << 2)    /* Bit 2: IRC64M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF             (1 << 3)    /* Bit 3: HXTAL stabilization interrupt flag */
#define RCU_INT_PLL0STBIF              (1 << 4)    /* Bit 4: PLL0 stabilization interrupt flag */
#define RCU_INT_PLL1STBIF              (1 << 5)    /* Bit 5: PLL1 stabilization interrupt flag */
#define RCU_INT_PLL2STBIF              (1 << 6)    /* Bit 6: PLL2 stabilization interrupt flag */
#define RCU_INT_CKMIF                  (1 << 7)    /* Bit 7: HXTAL clock stuck interrupt flag */
#define RCU_INT_IRC32KSTBIE            (1 << 8)    /* Bit 8: IRC32K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE             (1 << 9)    /* Bit 9: LXTAL stabilization interrupt enable */
#define RCU_INT_IRC64MSTBIE            (1 << 10)   /* Bit 10: IRC64M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE             (1 << 11)   /* Bit 11: HXTAL stabilization interrupt enable */
#define RCU_INT_PLL0STBIE              (1 << 12)   /* Bit 12: PLL0 stabilization interrupt enable */
#define RCU_INT_PLL1STBIE              (1 << 13)   /* Bit 13: PLL1 stabilization interrupt enable */
#define RCU_INT_PLL2STBIE              (1 << 14)   /* Bit 14: PLL2 stabilization interrupt enable */
#define RCU_INT_IRC32KSTBIC            (1 << 16)   /* Bit 16: IRC32K stabilization interrupt clear */
#define RCU_INT_LXTALSTBIC             (1 << 17)   /* Bit 17: LXTAL stabilization interrupt clear */
#define RCU_INT_IRC64MSTBIC            (1 << 18)   /* Bit 18: IRC64M stabilization interrupt clear */
#define RCU_INT_HXTALSTBIC             (1 << 19)   /* Bit 19: HXTAL stabilization interrupt clear */
#define RCU_INT_PLL0STBIC              (1 << 20)   /* Bit 20: PLL0 stabilization interrupt clear */
#define RCU_INT_PLL1STBIC              (1 << 21)   /* Bit 21: PLL1 stabilization interrupt clear */
#define RCU_INT_PLL2STBIC              (1 << 22)   /* Bit 22: PLL2 stabilization interrupt clear */
#define RCU_INT_CKMIC                  (1 << 23)   /* Bit 23: HXTAL clock stuck interrupt clear */
#define RCU_INT_LPIRC4MSTBIF           (1 << 24)   /* Bit 24: LPIRC4M stabilization interrupt flag */
#define RCU_INT_LPIRC4MSTBIE           (1 << 25)   /* Bit 25: LPIRC4M stabilization interrupt enable */
#define RCU_INT_LPIRC4MSTBIC           (1 << 26)   /* Bit 26: LPIRC4M stabilization interrupt clear */
#define RCU_INT_LCKMIF                 (1 << 27)   /* Bit 27: LXTAL clock stuck interrupt flag */
#define RCU_INT_LCKMIC                 (1 << 28)   /* Bit 28: LXTAL clock stuck interrupt clear */

/* AHB1 reset register */

#define RCU_AHB1RST_ENET1RST           (1 << 0)    /* Bit 0: ENET1 reset */
#define RCU_AHB1RST_USBHS0RST          (1 << 14)   /* Bit 14: USBHS0 reset */
#define RCU_AHB1RST_DMA0RST            (1 << 21)   /* Bit 21: DMA0 reset */
#define RCU_AHB1RST_DMA1RST            (1 << 22)   /* Bit 22: DMA1 reset */
#define RCU_AHB1RST_DMAMUXRST          (1 << 23)   /* Bit 23: DMAMUX reset */
#define RCU_AHB1RST_ENET0RST           (1 << 25)   /* Bit 25: ENET0 reset */
#define RCU_AHB1RST_USBHS1RST          (1 << 29)   /* Bit 29: USBHS1 reset */

/* AHB2 reset register */

#define RCU_AHB2RST_DCIRST             (1 << 0)    /* Bit 0: DCI reset */
#define RCU_AHB2RST_FACRST             (1 << 1)    /* Bit 1: FAC reset */
#define RCU_AHB2RST_SDIO1RST           (1 << 2)    /* Bit 2: SDIO1 reset */
#define RCU_AHB2RST_CAURST             (1 << 3)    /* Bit 3: CAU reset */
#define RCU_AHB2RST_HAURST             (1 << 4)    /* Bit 4: HAU reset */
#define RCU_AHB2RST_TRNGRST            (1 << 6)    /* Bit 6: TRNG reset */
#define RCU_AHB2RST_TMURST             (1 << 7)    /* Bit 7: TMU reset */

/* AHB3 reset register */

#define RCU_AHB3RST_EXMCRST            (1 << 0)    /* Bit 0: EXMC reset */
#define RCU_AHB3RST_IPARST             (1 << 1)    /* Bit 1: IPA reset */
#define RCU_AHB3RST_SDIO0RST           (1 << 2)    /* Bit 2: SDIO0 reset */
#define RCU_AHB3RST_MDMARST            (1 << 3)    /* Bit 3: MDMA reset */
#define RCU_AHB3RST_OSPIMRST           (1 << 4)    /* Bit 4: OSPIM reset */
#define RCU_AHB3RST_OSPI0RST           (1 << 5)    /* Bit 5: OSPI0 reset */
#define RCU_AHB3RST_OSPI1RST           (1 << 6)    /* Bit 6: OSPI1 reset */
#define RCU_AHB3RST_RTDEC0RST          (1 << 8)    /* Bit 8: RTDEC0 reset */
#define RCU_AHB3RST_RTDEC1RST          (1 << 9)    /* Bit 9: RTDEC1 reset */

/* AHB4 reset register */

#define RCU_AHB4RST_PARST              (1 << 0)    /* Bit 0: GPIO port A reset */
#define RCU_AHB4RST_PBRST              (1 << 1)    /* Bit 1: GPIO port B reset */
#define RCU_AHB4RST_PCRST              (1 << 2)    /* Bit 2: GPIO port C reset */
#define RCU_AHB4RST_PDRST              (1 << 3)    /* Bit 3: GPIO port D reset */
#define RCU_AHB4RST_PERST              (1 << 4)    /* Bit 4: GPIO port E reset */
#define RCU_AHB4RST_PFRST              (1 << 5)    /* Bit 5: GPIO port F reset */
#define RCU_AHB4RST_PGRST              (1 << 6)    /* Bit 6: GPIO port G reset */
#define RCU_AHB4RST_PHRST              (1 << 7)    /* Bit 7: GPIO port H reset */
#define RCU_AHB4RST_PJRST              (1 << 8)    /* Bit 8: GPIO port J reset */
#define RCU_AHB4RST_PKRST              (1 << 9)    /* Bit 9: GPIO port K reset */
#define RCU_AHB4RST_CRCRST             (1 << 14)   /* Bit 14: CRC reset */
#define RCU_AHB4RST_HWSEMRST           (1 << 15)   /* Bit 15: HWSEM reset */

/* APB1 reset register (additional bits) */

#define RCU_APB1RST_TIMER1RST          (1 << 0)    /* Bit 0: TIMER1 reset */
#define RCU_APB1RST_TIMER2RST          (1 << 1)    /* Bit 1: TIMER2 reset */
#define RCU_APB1RST_TIMER3RST          (1 << 2)    /* Bit 2: TIMER3 reset */
#define RCU_APB1RST_TIMER4RST          (1 << 3)    /* Bit 3: TIMER4 reset */
#define RCU_APB1RST_TIMER5RST          (1 << 4)    /* Bit 4: TIMER5 reset */
#define RCU_APB1RST_TIMER6RST          (1 << 5)    /* Bit 5: TIMER6 reset */
#define RCU_APB1RST_TIMER22RST         (1 << 6)    /* Bit 6: TIMER22 reset */
#define RCU_APB1RST_TIMER23RST         (1 << 7)    /* Bit 7: TIMER23 reset */
#define RCU_APB1RST_TIMER30RST         (1 << 8)    /* Bit 8: TIMER30 reset */
#define RCU_APB1RST_TIMER31RST         (1 << 9)    /* Bit 9: TIMER31 reset */
#define RCU_APB1RST_TIMER50RST         (1 << 10)   /* Bit 10: TIMER50 reset */
#define RCU_APB1RST_TIMER51RST         (1 << 11)   /* Bit 11: TIMER51 reset */
#define RCU_APB1RST_RSPDIFRST          (1 << 13)   /* Bit 13: RSPDIF reset */
#define RCU_APB1RST_SPI1RST            (1 << 14)   /* Bit 14: SPI1 reset */
#define RCU_APB1RST_SPI2RST            (1 << 15)   /* Bit 15: SPI2 reset */
#define RCU_APB1RST_MDIORST            (1 << 16)   /* Bit 16: MDIO reset */
#define RCU_APB1RST_USART1RST          (1 << 17)   /* Bit 17: USART1 reset */
#define RCU_APB1RST_USART2RST          (1 << 18)   /* Bit 18: USART2 reset */
#define RCU_APB1RST_UART3RST           (1 << 19)   /* Bit 19: UART3 reset */
#define RCU_APB1RST_UART4RST           (1 << 20)   /* Bit 20: UART4 reset */
#define RCU_APB1RST_I2C0RST            (1 << 21)   /* Bit 21: I2C0 reset */
#define RCU_APB1RST_I2C1RST            (1 << 22)   /* Bit 22: I2C1 reset */
#define RCU_APB1RST_I2C2RST            (1 << 23)   /* Bit 23: I2C2 reset */
#define RCU_APB1RST_I2C3RST            (1 << 24)   /* Bit 24: I2C3 reset */
#define RCU_APB1RST_CTCRST             (1 << 27)   /* Bit 27: CTC reset */
#define RCU_APB1RST_DACHOLDRST         (1 << 28)   /* Bit 28: DAC hold clock reset */
#define RCU_APB1RST_DACRST             (1 << 29)   /* Bit 29: DAC reset */
#define RCU_APB1RST_UART6RST           (1 << 30)   /* Bit 30: UART6 reset */
#define RCU_APB1RST_UART7RST           (1 << 31)   /* Bit 31: UART7 reset */

/* APB2 reset register */

#define RCU_APB2RST_TIMER0RST          (1 << 0)    /* Bit 0: TIMER0 reset */
#define RCU_APB2RST_TIMER7RST          (1 << 1)    /* Bit 1: TIMER7 reset */
#define RCU_APB2RST_USART0RST          (1 << 4)    /* Bit 4: USART0 reset */
#define RCU_APB2RST_USART5RST          (1 << 5)    /* Bit 5: USART5 reset */
#define RCU_APB2RST_ADC0RST            (1 << 8)    /* Bit 8: ADC0 reset */
#define RCU_APB2RST_ADC1RST            (1 << 9)    /* Bit 9: ADC1 reset */
#define RCU_APB2RST_ADC2RST            (1 << 10)   /* Bit 10: ADC2 reset */
#define RCU_APB2RST_SPI0RST            (1 << 12)   /* Bit 12: SPI0 reset */
#define RCU_APB2RST_SPI3RST            (1 << 13)   /* Bit 13: SPI3 reset */
#define RCU_APB2RST_TIMER14RST         (1 << 16)   /* Bit 16: TIMER14 reset */
#define RCU_APB2RST_TIMER15RST         (1 << 17)   /* Bit 17: TIMER15 reset */
#define RCU_APB2RST_TIMER16RST         (1 << 18)   /* Bit 18: TIMER16 reset */
#define RCU_APB2RST_HPDFRST            (1 << 19)   /* Bit 19: HPDF reset */
#define RCU_APB2RST_SPI4RST            (1 << 20)   /* Bit 20: SPI4 reset */
#define RCU_APB2RST_SPI5RST            (1 << 21)   /* Bit 21: SPI5 reset */
#define RCU_APB2RST_SAI0RST            (1 << 22)   /* Bit 22: SAI0 reset */
#define RCU_APB2RST_SAI1RST            (1 << 23)   /* Bit 23: SAI1 reset */
#define RCU_APB2RST_SAI2RST            (1 << 24)   /* Bit 24: SAI2 reset */
#define RCU_APB2RST_TIMER40RST         (1 << 25)   /* Bit 25: TIMER40 reset */
#define RCU_APB2RST_TIMER41RST         (1 << 26)   /* Bit 26: TIMER41 reset */
#define RCU_APB2RST_TIMER42RST         (1 << 27)   /* Bit 27: TIMER42 reset */
#define RCU_APB2RST_TIMER43RST         (1 << 28)   /* Bit 28: TIMER43 reset */
#define RCU_APB2RST_TIMER44RST         (1 << 29)   /* Bit 29: TIMER44 reset */
#define RCU_APB2RST_EDOUTRST           (1 << 30)   /* Bit 30: EDOUT reset */
#define RCU_APB2RST_TRIGSELRST         (1 << 31)   /* Bit 31: TRIGSEL reset */

/* APB3 reset register */

#define RCU_APB3RST_TLIRST             (1 << 0)    /* Bit 0: TLI reset */
#define RCU_APB3RST_WWDGTRST           (1 << 1)    /* Bit 1: WWDGT reset */

/* APB4 reset register */

#define RCU_APB4RST_SYSCFGRST          (1 << 0)    /* Bit 0: SYSCFG reset */
#define RCU_APB4RST_CMPRST             (1 << 1)    /* Bit 1: CMP reset */
#define RCU_APB4RST_VREFRST            (1 << 2)    /* Bit 2: VREF reset */
#define RCU_APB4RST_LPDTSRST           (1 << 3)    /* Bit 3: LPDTS reset */
#define RCU_APB4RST_PMURST             (1 << 4)    /* Bit 4: PMU reset */
#define RCU_APB4RST_PMURST             (1 << 4)    /* Bit 4: PMU reset */

/* AHB1 enable register */

#define RCU_AHB1EN_ENET1EN             (1 << 0)    /* Bit 0: ENET1 clock enable */
#define RCU_AHB1EN_ENET1TXEN           (1 << 1)    /* Bit 1: ENET1 TX clock enable */
#define RCU_AHB1EN_ENET1RXEN           (1 << 2)    /* Bit 2: ENET1 RX clock enable */
#define RCU_AHB1EN_ENET1PTPEN          (1 << 3)    /* Bit 3: ENET1 PTP clock enable */
#define RCU_AHB1EN_USBHS0EN            (1 << 14)   /* Bit 14: USBHS0 clock enable */
#define RCU_AHB1EN_USBHS0ULPIEN        (1 << 15)   /* Bit 15: USBHS0 ULPI clock enable */
#define RCU_AHB1EN_DMA0EN              (1 << 21)   /* Bit 21: DMA0 clock enable */
#define RCU_AHB1EN_DMA1EN              (1 << 22)   /* Bit 22: DMA1 clock enable */
#define RCU_AHB1EN_DMAMUXEN            (1 << 23)   /* Bit 23: DMAMUX clock enable */
#define RCU_AHB1EN_ENET0EN             (1 << 25)   /* Bit 25: ENET0 clock enable */
#define RCU_AHB1EN_ENET0TXEN           (1 << 26)   /* Bit 26: ENET0 TX clock enable */
#define RCU_AHB1EN_ENET0RXEN           (1 << 27)   /* Bit 27: ENET0 RX clock enable */
#define RCU_AHB1EN_ENET0PTPEN          (1 << 28)   /* Bit 28: ENET0 PTP clock enable */
#define RCU_AHB1EN_USBHS1EN            (1 << 29)   /* Bit 29: USBHS1 clock enable */
#define RCU_AHB1EN_USBHS1ULPIEN        (1 << 30)   /* Bit 30: USBHS1 ULPI clock enable */

/* AHB2 enable register */

#define RCU_AHB2EN_DCIEN               (1 << 0)    /* Bit 0: DCI clock enable */
#define RCU_AHB2EN_FACEN               (1 << 1)    /* Bit 1: FAC clock enable */
#define RCU_AHB2EN_SDIO1EN             (1 << 2)    /* Bit 2: SDIO1 clock enable */
#define RCU_AHB2EN_CAUEN               (1 << 3)    /* Bit 3: CAU clock enable */
#define RCU_AHB2EN_HAUEN               (1 << 4)    /* Bit 4: HAU clock enable */
#define RCU_AHB2EN_TRNGEN              (1 << 6)    /* Bit 6: TRNG clock enable */
#define RCU_AHB2EN_TMUEN               (1 << 7)    /* Bit 7: TMU clock enable */
#define RCU_AHB2EN_RAMECCMU1EN         (1 << 8)    /* Bit 8: RAMECCMU1 clock enable */

/* AHB3 enable register */

#define RCU_AHB3EN_EXMCEN              (1 << 0)    /* Bit 0: EXMC clock enable */
#define RCU_AHB3EN_IPAEN               (1 << 1)    /* Bit 1: IPA clock enable */
#define RCU_AHB3EN_SDIO0EN             (1 << 2)    /* Bit 2: SDIO0 clock enable */
#define RCU_AHB3EN_MDMAEN              (1 << 3)    /* Bit 3: MDMA clock enable */
#define RCU_AHB3EN_OSPIMEN             (1 << 4)    /* Bit 4: OSPIM clock enable */
#define RCU_AHB3EN_OSPI0EN             (1 << 5)    /* Bit 5: OSPI0 clock enable */
#define RCU_AHB3EN_OSPI1EN             (1 << 6)    /* Bit 6: OSPI1 clock enable */
#define RCU_AHB3EN_RTDEC0EN            (1 << 8)    /* Bit 8: RTDEC0 clock enable */
#define RCU_AHB3EN_RTDEC1EN            (1 << 9)    /* Bit 9: RTDEC1 clock enable */
#define RCU_AHB3EN_RAMECCMU0EN         (1 << 10)   /* Bit 10: RAMECCMU0 clock enable */
#define RCU_AHB3EN_CPUEN               (1 << 15)   /* Bit 15: CPU clock enable */

/* AHB4 enable register */

#define RCU_AHB4EN_PAEN                (1 << 0)    /* Bit 0: GPIO port A clock enable */
#define RCU_AHB4EN_PBEN                (1 << 1)    /* Bit 1: GPIO port B clock enable */
#define RCU_AHB4EN_PCEN                (1 << 2)    /* Bit 2: GPIO port C clock enable */
#define RCU_AHB4EN_PDEN                (1 << 3)    /* Bit 3: GPIO port D clock enable */
#define RCU_AHB4EN_PEEN                (1 << 4)    /* Bit 4: GPIO port E clock enable */
#define RCU_AHB4EN_PFEN                (1 << 5)    /* Bit 5: GPIO port F clock enable */
#define RCU_AHB4EN_PGEN                (1 << 6)    /* Bit 6: GPIO port G clock enable */
#define RCU_AHB4EN_PHEN                (1 << 7)    /* Bit 7: GPIO port H clock enable */
#define RCU_AHB4EN_PJEN                (1 << 8)    /* Bit 8: GPIO port J clock enable */
#define RCU_AHB4EN_PKEN                (1 << 9)    /* Bit 9: GPIO port K clock enable */
#define RCU_AHB4EN_BKPSRAMEN           (1 << 13)   /* Bit 13: Backup SRAM enable */
#define RCU_AHB4EN_CRCEN               (1 << 14)   /* Bit 14: CRC clock enable */
#define RCU_AHB4EN_HWSEMEN             (1 << 15)   /* Bit 15: HWSEM clock enable */

/* APB1 enable register */

#define RCU_APB1EN_TIMER1EN            (1 << 0)    /* Bit 0: TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN            (1 << 1)    /* Bit 1: TIMER2 clock enable */
#define RCU_APB1EN_TIMER3EN            (1 << 2)    /* Bit 2: TIMER3 clock enable */
#define RCU_APB1EN_TIMER4EN            (1 << 3)    /* Bit 3: TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN            (1 << 4)    /* Bit 4: TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN            (1 << 5)    /* Bit 5: TIMER6 clock enable */
#define RCU_APB1EN_TIMER22EN           (1 << 6)    /* Bit 6: TIMER22 clock enable */
#define RCU_APB1EN_TIMER23EN           (1 << 7)    /* Bit 7: TIMER23 clock enable */
#define RCU_APB1EN_TIMER30EN           (1 << 8)    /* Bit 8: TIMER30 clock enable */
#define RCU_APB1EN_TIMER31EN           (1 << 9)    /* Bit 9: TIMER31 clock enable */
#define RCU_APB1EN_TIMER50EN           (1 << 10)   /* Bit 10: TIMER50 clock enable */
#define RCU_APB1EN_TIMER51EN           (1 << 11)   /* Bit 11: TIMER51 clock enable */
#define RCU_APB1EN_RSPDIFEN            (1 << 13)   /* Bit 13: RSPDIF clock enable */
#define RCU_APB1EN_SPI1EN              (1 << 14)   /* Bit 14: SPI1 clock enable */
#define RCU_APB1EN_SPI2EN              (1 << 15)   /* Bit 15: SPI2 clock enable */
#define RCU_APB1EN_MDIOEN              (1 << 16)   /* Bit 16: MDIO clock enable */
#define RCU_APB1EN_USART1EN            (1 << 17)   /* Bit 17: USART1 clock enable */
#define RCU_APB1EN_USART2EN            (1 << 18)   /* Bit 18: USART2 clock enable */
#define RCU_APB1EN_UART3EN             (1 << 19)   /* Bit 19: UART3 clock enable */
#define RCU_APB1EN_UART4EN             (1 << 20)   /* Bit 20: UART4 clock enable */
#define RCU_APB1EN_I2C0EN              (1 << 21)   /* Bit 21: I2C0 clock enable */
#define RCU_APB1EN_I2C1EN              (1 << 22)   /* Bit 22: I2C1 clock enable */
#define RCU_APB1EN_I2C2EN              (1 << 23)   /* Bit 23: I2C2 clock enable */
#define RCU_APB1EN_I2C3EN              (1 << 24)   /* Bit 24: I2C3 clock enable */
#define RCU_APB1EN_CTCEN               (1 << 27)   /* Bit 27: CTC clock enable */
#define RCU_APB1EN_DACHOLDEN           (1 << 28)   /* Bit 28: DAC hold clock enable */
#define RCU_APB1EN_DACEN               (1 << 29)   /* Bit 29: DAC clock enable */
#define RCU_APB1EN_UART6EN             (1 << 30)   /* Bit 30: UART6 clock enable */
#define RCU_APB1EN_UART7EN             (1 << 31)   /* Bit 31: UART7 clock enable */

/* APB2 enable register */

#define RCU_APB2EN_TIMER0EN            (1 << 0)    /* Bit 0: TIMER0 clock enable */
#define RCU_APB2EN_TIMER7EN            (1 << 1)    /* Bit 1: TIMER7 clock enable */
#define RCU_APB2EN_USART0EN            (1 << 4)    /* Bit 4: USART0 clock enable */
#define RCU_APB2EN_USART5EN            (1 << 5)    /* Bit 5: USART5 clock enable */
#define RCU_APB2EN_ADC0EN              (1 << 8)    /* Bit 8: ADC0 clock enable */
#define RCU_APB2EN_ADC1EN              (1 << 9)    /* Bit 9: ADC1 clock enable */
#define RCU_APB2EN_ADC2EN              (1 << 10)   /* Bit 10: ADC2 clock enable */
#define RCU_APB2EN_SPI0EN              (1 << 12)   /* Bit 12: SPI0 clock enable */
#define RCU_APB2EN_SPI3EN              (1 << 13)   /* Bit 13: SPI3 clock enable */
#define RCU_APB2EN_TIMER14EN           (1 << 16)   /* Bit 16: TIMER14 clock enable */
#define RCU_APB2EN_TIMER15EN           (1 << 17)   /* Bit 17: TIMER15 clock enable */
#define RCU_APB2EN_TIMER16EN           (1 << 18)   /* Bit 18: TIMER16 clock enable */
#define RCU_APB2EN_HPDFEN              (1 << 19)   /* Bit 19: HPDF clock enable */
#define RCU_APB2EN_SPI4EN              (1 << 20)   /* Bit 20: SPI4 clock enable */
#define RCU_APB2EN_SPI5EN              (1 << 21)   /* Bit 21: SPI5 clock enable */
#define RCU_APB2EN_SAI0EN              (1 << 22)   /* Bit 22: SAI0 clock enable */
#define RCU_APB2EN_SAI1EN              (1 << 23)   /* Bit 23: SAI1 clock enable */
#define RCU_APB2EN_SAI2EN              (1 << 24)   /* Bit 24: SAI2 clock enable */
#define RCU_APB2EN_TIMER40EN           (1 << 25)   /* Bit 25: TIMER40 clock enable */
#define RCU_APB2EN_TIMER41EN           (1 << 26)   /* Bit 26: TIMER41 clock enable */
#define RCU_APB2EN_TIMER42EN           (1 << 27)   /* Bit 27: TIMER42 clock enable */
#define RCU_APB2EN_TIMER43EN           (1 << 28)   /* Bit 28: TIMER43 clock enable */
#define RCU_APB2EN_TIMER44EN           (1 << 29)   /* Bit 29: TIMER44 clock enable */
#define RCU_APB2EN_EDOUTEN             (1 << 30)   /* Bit 30: EDOUT clock enable */
#define RCU_APB2EN_TRIGSELEN           (1 << 31)   /* Bit 31: TRIGSEL clock enable */

/* APB3 enable register */

#define RCU_APB3EN_TLIEN               (1 << 0)    /* Bit 0: TLI clock enable */
#define RCU_APB3EN_WWDGTEN             (1 << 1)    /* Bit 1: WWDGT clock enable */

/* APB4 enable register */

#define RCU_APB4EN_SYSCFGEN            (1 << 0)    /* Bit 0: SYSCFG clock enable */
#define RCU_APB4EN_CMPEN               (1 << 1)    /* Bit 1: CMP clock enable */
#define RCU_APB4EN_VREFEN              (1 << 2)    /* Bit 2: VREF clock enable */
#define RCU_APB4EN_LPDTSEN             (1 << 3)    /* Bit 3: LPDTS clock enable */
#define RCU_APB4EN_PMUEN               (1 << 4)    /* Bit 4: PMU clock enable */

/* AHB1 sleep mode enable register */

#define RCU_AHB1SPEN_ENET1SPEN         (1 << 0)    /* Bit 0: ENET1 clock enable when sleep mode */
#define RCU_AHB1SPEN_ENET1TXSPEN       (1 << 1)    /* Bit 1: ENET1 TX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENET1RXSPEN       (1 << 2)    /* Bit 2: ENET1 RX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENET1PTPSPEN      (1 << 3)    /* Bit 3: ENET1 PTP clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHS0SPEN        (1 << 14)   /* Bit 14: USBHS0 clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHS0ULPISPEN    (1 << 15)   /* Bit 15: USBHS0 ULPI clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM0SPEN         (1 << 16)   /* Bit 16: SRAM0 clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM1SPEN         (1 << 17)   /* Bit 17: SRAM1 clock enable when sleep mode */
#define RCU_AHB1SPEN_DMA0SPEN          (1 << 21)   /* Bit 21: DMA0 clock enable when sleep mode */
#define RCU_AHB1SPEN_DMA1SPEN          (1 << 22)   /* Bit 22: DMA1 clock enable when sleep mode */
#define RCU_AHB1SPEN_DMAMUXSPEN        (1 << 23)   /* Bit 23: DMAMUX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENET0SPEN         (1 << 25)   /* Bit 25: ENET0 clock enable when sleep mode */
#define RCU_AHB1SPEN_ENET0TXSPEN       (1 << 26)   /* Bit 26: ENET0 TX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENET0RXSPEN       (1 << 27)   /* Bit 27: ENET0 RX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENET0PTPSPEN      (1 << 28)   /* Bit 28: ENET0 PTP clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHS1SPEN        (1 << 29)   /* Bit 29: USBHS1 clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHS1ULPISPEN    (1 << 30)   /* Bit 30: USBHS1 ULPI clock enable when sleep mode */

/* AHB2 sleep mode enable register */

#define RCU_AHB2SPEN_DCISPEN           (1 << 0)    /* Bit 0: DCI clock enable when sleep mode */
#define RCU_AHB2SPEN_FACSPEN           (1 << 1)    /* Bit 1: FAC clock enable when sleep mode */
#define RCU_AHB2SPEN_SDIO1SPEN         (1 << 2)    /* Bit 2: SDIO1 clock enable when sleep mode */
#define RCU_AHB2SPEN_CAUSPEN           (1 << 3)    /* Bit 3: CAU clock enable when sleep mode */
#define RCU_AHB2SPEN_HAUSPEN           (1 << 4)    /* Bit 4: HAU clock enable when sleep mode */
#define RCU_AHB2SPEN_TRNGSPEN          (1 << 6)    /* Bit 6: TRNG clock enable when sleep mode */
#define RCU_AHB2SPEN_TMUSPEN           (1 << 7)    /* Bit 7: TMU clock enable when sleep mode */
#define RCU_AHB2SPEN_RAMECCMU1SPEN     (1 << 8)    /* Bit 8: RAMECCMU1 clock enable when sleep mode */

/* AHB3 sleep mode enable register */

#define RCU_AHB3SPEN_EXMCSPEN          (1 << 0)    /* Bit 0: EXMC clock enable when sleep mode */
#define RCU_AHB3SPEN_IPASPEN           (1 << 1)    /* Bit 1: IPA clock enable when sleep mode */
#define RCU_AHB3SPEN_SDIO0SPEN         (1 << 2)    /* Bit 2: SDIO0 clock enable when sleep mode */
#define RCU_AHB3SPEN_MDMASPEN          (1 << 3)    /* Bit 3: MDMA clock enable when sleep mode */
#define RCU_AHB3SPEN_OSPIMSPEN         (1 << 4)    /* Bit 4: OSPIM clock enable when sleep mode */
#define RCU_AHB3SPEN_OSPI0SPEN         (1 << 5)    /* Bit 5: OSPI0 clock enable when sleep mode */
#define RCU_AHB3SPEN_OSPI1SPEN         (1 << 6)    /* Bit 6: OSPI1 clock enable when sleep mode */
#define RCU_AHB3SPEN_RTDEC0SPEN        (1 << 8)    /* Bit 8: RTDEC0 clock enable when sleep mode */
#define RCU_AHB3SPEN_RTDEC1SPEN        (1 << 9)    /* Bit 9: RTDEC1 clock enable when sleep mode */
#define RCU_AHB3SPEN_RAMECCMU0SPEN     (1 << 10)   /* Bit 10: RAMECCMU0 clock enable when sleep mode */
#define RCU_AHB3SPEN_AXISRAMSPEN       (1 << 14)   /* Bit 14: AXISRAM clock enable when sleep mode */
#define RCU_AHB3SPEN_FMCSPEN           (1 << 15)   /* Bit 15: FMC clock enable when sleep mode */

/* AHB4 sleep mode enable register */

#define RCU_AHB4SPEN_PASPEN            (1 << 0)    /* Bit 0: GPIO port A clock enable when sleep mode */
#define RCU_AHB4SPEN_PBSPEN            (1 << 1)    /* Bit 1: GPIO port B clock enable when sleep mode */
#define RCU_AHB4SPEN_PCSPEN            (1 << 2)    /* Bit 2: GPIO port C clock enable when sleep mode */
#define RCU_AHB4SPEN_PDSPEN            (1 << 3)    /* Bit 3: GPIO port D clock enable when sleep mode */
#define RCU_AHB4SPEN_PESPEN            (1 << 4)    /* Bit 4: GPIO port E clock enable when sleep mode */
#define RCU_AHB4SPEN_PFSPEN            (1 << 5)    /* Bit 5: GPIO port F clock enable when sleep mode */
#define RCU_AHB4SPEN_PGSPEN            (1 << 6)    /* Bit 6: GPIO port G clock enable when sleep mode */
#define RCU_AHB4SPEN_PHSPEN            (1 << 7)    /* Bit 7: GPIO port H clock enable when sleep mode */
#define RCU_AHB4SPEN_PJSPEN            (1 << 8)    /* Bit 8: GPIO port J clock enable when sleep mode */
#define RCU_AHB4SPEN_PKSPEN            (1 << 9)    /* Bit 9: GPIO port K clock enable when sleep mode */
#define RCU_AHB4SPEN_BKPSRAMSPEN       (1 << 13)   /* Bit 13: Backup SRAM clock enable when sleep mode */
#define RCU_AHB4SPEN_CRCSPEN           (1 << 14)   /* Bit 14: CRC clock enable when sleep mode */

/* APB1 sleep mode enable register */

#define RCU_APB1SPEN_TIMER1SPEN        (1 << 0)    /* Bit 0: TIMER1 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER2SPEN        (1 << 1)    /* Bit 1: TIMER2 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER3SPEN        (1 << 2)    /* Bit 2: TIMER3 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER4SPEN        (1 << 3)    /* Bit 3: TIMER4 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER5SPEN        (1 << 4)    /* Bit 4: TIMER5 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER6SPEN        (1 << 5)    /* Bit 5: TIMER6 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER22SPEN       (1 << 6)    /* Bit 6: TIMER22 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER23SPEN       (1 << 7)    /* Bit 7: TIMER23 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER30SPEN       (1 << 8)    /* Bit 8: TIMER30 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER31SPEN       (1 << 9)    /* Bit 9: TIMER31 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER50SPEN       (1 << 10)   /* Bit 10: TIMER50 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER51SPEN       (1 << 11)   /* Bit 11: TIMER51 clock enable when sleep mode */
#define RCU_APB1SPEN_RSPDIFSPEN        (1 << 13)   /* Bit 13: RSPDIF clock enable when sleep mode */
#define RCU_APB1SPEN_SPI1SPEN          (1 << 14)   /* Bit 14: SPI1 clock enable when sleep mode */
#define RCU_APB1SPEN_SPI2SPEN          (1 << 15)   /* Bit 15: SPI2 clock enable when sleep mode */
#define RCU_APB1SPEN_MDIOSPEN          (1 << 16)   /* Bit 16: MDIO clock enable when sleep mode */
#define RCU_APB1SPEN_USART1SPEN        (1 << 17)   /* Bit 17: USART1 clock enable when sleep mode */
#define RCU_APB1SPEN_USART2SPEN        (1 << 18)   /* Bit 18: USART2 clock enable when sleep mode */
#define RCU_APB1SPEN_UART3SPEN         (1 << 19)   /* Bit 19: UART3 clock enable when sleep mode */
#define RCU_APB1SPEN_UART4SPEN         (1 << 20)   /* Bit 20: UART4 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C0SPEN          (1 << 21)   /* Bit 21: I2C0 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C1SPEN          (1 << 22)   /* Bit 22: I2C1 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C2SPEN          (1 << 23)   /* Bit 23: I2C2 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C3SPEN          (1 << 24)   /* Bit 24: I2C3 clock enable when sleep mode */
#define RCU_APB1SPEN_CTCSPEN           (1 << 27)   /* Bit 27: CTC clock enable when sleep mode */
#define RCU_APB1SPEN_DACHOLDSPEN       (1 << 28)   /* Bit 28: DAC hold clock enable when sleep mode */
#define RCU_APB1SPEN_DACSPEN           (1 << 29)   /* Bit 29: DAC clock enable when sleep mode */
#define RCU_APB1SPEN_UART6SPEN         (1 << 30)   /* Bit 30: UART6 clock enable when sleep mode */
#define RCU_APB1SPEN_UART7SPEN         (1 << 31)   /* Bit 31: UART7 clock enable when sleep mode */

/* APB2 sleep mode enable register */

#define RCU_APB2SPEN_TIMER0SPEN        (1 << 0)    /* Bit 0: TIMER0 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER7SPEN        (1 << 1)    /* Bit 1: TIMER7 clock enable when sleep mode */
#define RCU_APB2SPEN_USART0SPEN        (1 << 4)    /* Bit 4: USART0 clock enable when sleep mode */
#define RCU_APB2SPEN_USART5SPEN        (1 << 5)    /* Bit 5: USART5 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC0SPEN          (1 << 8)    /* Bit 8: ADC0 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC1SPEN          (1 << 9)    /* Bit 9: ADC1 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC2SPEN          (1 << 10)   /* Bit 10: ADC2 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI0SPEN          (1 << 12)   /* Bit 12: SPI0 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI3SPEN          (1 << 13)   /* Bit 13: SPI3 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER14SPEN       (1 << 16)   /* Bit 16: TIMER14 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER15SPEN       (1 << 17)   /* Bit 17: TIMER15 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER16SPEN       (1 << 18)   /* Bit 18: TIMER16 clock enable when sleep mode */
#define RCU_APB2SPEN_HPDFSPEN          (1 << 19)   /* Bit 19: HPDF clock enable when sleep mode */
#define RCU_APB2SPEN_SPI4SPEN          (1 << 20)   /* Bit 20: SPI4 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI5SPEN          (1 << 21)   /* Bit 21: SPI5 clock enable when sleep mode */
#define RCU_APB2SPEN_SAI0SPEN          (1 << 22)   /* Bit 22: SAI0 clock enable when sleep mode */
#define RCU_APB2SPEN_SAI1SPEN          (1 << 23)   /* Bit 23: SAI1 clock enable when sleep mode */
#define RCU_APB2SPEN_SAI2SPEN          (1 << 24)   /* Bit 24: SAI2 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER40SPEN       (1 << 25)   /* Bit 25: TIMER40 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER41SPEN       (1 << 26)   /* Bit 26: TIMER41 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER42SPEN       (1 << 27)   /* Bit 27: TIMER42 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER43SPEN       (1 << 28)   /* Bit 28: TIMER43 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER44SPEN       (1 << 29)   /* Bit 29: TIMER44 clock enable when sleep mode */
#define RCU_APB2SPEN_EDOUTSPEN         (1 << 30)   /* Bit 30: EDOUT clock enable when sleep mode */
#define RCU_APB2SPEN_TRIGSELSPEN       (1 << 31)   /* Bit 31: TRIGSEL clock enable when sleep mode */

/* APB3 sleep mode enable register */

#define RCU_APB3SPEN_TLISPEN           (1 << 0)    /* Bit 0: TLI clock enable when sleep mode */
#define RCU_APB3SPEN_WWDGTSPEN         (1 << 1)    /* Bit 1: WWDGT clock enable when sleep mode */

/* APB4 sleep mode enable register */

#define RCU_APB4SPEN_SYSCFGSPEN        (1 << 0)    /* Bit 0: SYSCFG clock enable when sleep mode */
#define RCU_APB4SPEN_CMPSPEN           (1 << 1)    /* Bit 1: CMP clock enable when sleep mode */
#define RCU_APB4SPEN_VREFSPEN          (1 << 2)    /* Bit 2: VREF clock enable when sleep mode */
#define RCU_APB4SPEN_LPDTSSPEN         (1 << 3)    /* Bit 3: LPDTS clock enable when sleep mode */
#define RCU_APB4SPEN_PMUSPEN           (1 << 4)    /* Bit 4: PMU clock enable when sleep mode */

/* Backup domain control register */

#define RCU_BDCTL_LXTALEN              (1 << 0)    /* Bit 0: LXTAL enable */
#define RCU_BDCTL_LXTALSTB             (1 << 1)    /* Bit 1: LXTAL stabilization flag */
#define RCU_BDCTL_LXTALBPS             (1 << 2)    /* Bit 2: LXTAL bypass mode enable */
#define RCU_BDCTL_LXTALDRI_SHIFT       (3)         /* Bits 3-4: LXTAL drive capability */
#define RCU_BDCTL_LXTALDRI_MASK        (3 << RCU_BDCTL_LXTALDRI_SHIFT)
#define RCU_BDCTL_LCKMEN               (1 << 5)    /* Bit 5: LXTAL clock monitor enable */
#define RCU_BDCTL_LCKMD                (1 << 6)    /* Bit 6: LXTAL clock failure detection flag */
#define RCU_BDCTL_RTCSRC_SHIFT         (8)         /* Bits 8-9: RTC clock source selection */
#define RCU_BDCTL_RTCSRC_MASK          (3 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_NONE        (0 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_LXTAL       (1 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_IRC32K      (2 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_HXTAL       (3 << RCU_BDCTL_RTCSRC_SHIFT)
#define RCU_BDCTL_RTCEN                (1 << 15)   /* Bit 15: RTC clock enable */
#define RCU_BDCTL_BKPRST               (1 << 16)   /* Bit 16: Backup domain reset */

/* Reset source / clock register */

#define RCU_RSTSCK_IRC32KEN            (1 << 0)    /* Bit 0: IRC32K enable */
#define RCU_RSTSCK_IRC32KSTB           (1 << 1)    /* Bit 1: IRC32K stabilization flag */
#define RCU_RSTSCK_RSTFC               (1 << 24)   /* Bit 24: reset flag clear */
#define RCU_RSTSCK_BORRSTF             (1 << 25)   /* Bit 25: BOR reset flag */
#define RCU_RSTSCK_EPRSTF              (1 << 26)   /* Bit 26: external pin reset flag */
#define RCU_RSTSCK_PORRSTF             (1 << 27)   /* Bit 27: power reset flag */
#define RCU_RSTSCK_SWRSTF              (1 << 28)   /* Bit 28: software reset flag */
#define RCU_RSTSCK_FWDGTRSTF           (1 << 29)   /* Bit 29: free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF           (1 << 30)   /* Bit 30: window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF              (1 << 31)   /* Bit 31: low-power reset flag */

/* PLL clock additional control register */

#define RCU_PLLADDCTL_PLL0Q_SHIFT      (0)         /* Bits 0-6: PLL0Q output frequency division factor */
#define RCU_PLLADDCTL_PLL0Q_MASK       (0x7f << RCU_PLLADDCTL_PLL0Q_SHIFT)
#  define RCU_PLLADDCTL_PLL0Q(n)       ((n) << RCU_PLLADDCTL_PLL0Q_SHIFT)
#define RCU_PLLADDCTL_PLL1Q_SHIFT      (8)         /* Bits 8-14: PLL1Q output frequency division factor */
#define RCU_PLLADDCTL_PLL1Q_MASK       (0x7f << RCU_PLLADDCTL_PLL1Q_SHIFT)
#  define RCU_PLLADDCTL_PLL1Q(n)       ((n) << RCU_PLLADDCTL_PLL1Q_SHIFT)
#define RCU_PLLADDCTL_PLL2Q_SHIFT      (16)        /* Bits 16-22: PLL2Q output frequency division factor */
#define RCU_PLLADDCTL_PLL2Q_MASK       (0x7f << RCU_PLLADDCTL_PLL2Q_SHIFT)
#  define RCU_PLLADDCTL_PLL2Q(n)       ((n) << RCU_PLLADDCTL_PLL2Q_SHIFT)
#define RCU_PLLADDCTL_PLL0QEN          (1 << 23)   /* Bit 23: PLL0Q divider output enable */
#define RCU_PLLADDCTL_PLL0REN          (1 << 24)   /* Bit 24: PLL0R divider output enable */
#define RCU_PLLADDCTL_PLL0PEN          (1 << 25)   /* Bit 25: PLL0P divider output enable */
#define RCU_PLLADDCTL_PLL1QEN          (1 << 26)   /* Bit 26: PLL1Q divider output enable */
#define RCU_PLLADDCTL_PLL1REN          (1 << 27)   /* Bit 27: PLL1R divider output enable */
#define RCU_PLLADDCTL_PLL1PEN          (1 << 28)   /* Bit 28: PLL1P divider output enable */
#define RCU_PLLADDCTL_PLL2QEN          (1 << 29)   /* Bit 29: PLL2Q divider output enable */
#define RCU_PLLADDCTL_PLL2REN          (1 << 30)   /* Bit 30: PLL2R divider output enable */
#define RCU_PLLADDCTL_PLL2PEN          (1 << 31)   /* Bit 31: PLL2P divider output enable */

/* PLL1 register */

#define RCU_PLL1_PLL1PSC_SHIFT         (0)         /* Bits 0-5: PLL1 VCO source clock prescaler */
#define RCU_PLL1_PLL1PSC_MASK          (0x3f << RCU_PLL1_PLL1PSC_SHIFT)
#  define RCU_PLL1_PLL1PSC(n)          ((n) << RCU_PLL1_PLL1PSC_SHIFT)
#define RCU_PLL1_PLL1N_SHIFT           (6)         /* Bits 6-14: PLL1 VCO clock multi factor */
#define RCU_PLL1_PLL1N_MASK            (0x1ff << RCU_PLL1_PLL1N_SHIFT)
#  define RCU_PLL1_PLL1N(n)            ((n) << RCU_PLL1_PLL1N_SHIFT)
#define RCU_PLL1_PLL1P_SHIFT           (16)        /* Bits 16-22: PLL1P output frequency division factor */
#define RCU_PLL1_PLL1P_MASK            (0x7f << RCU_PLL1_PLL1P_SHIFT)
#  define RCU_PLL1_PLL1P(n)            ((n) << RCU_PLL1_PLL1P_SHIFT)
#define RCU_PLL1_PLL1R_SHIFT           (24)        /* Bits 24-30: PLL1R output frequency division factor */
#define RCU_PLL1_PLL1R_MASK            (0x7f << RCU_PLL1_PLL1R_SHIFT)
#  define RCU_PLL1_PLL1R(n)            ((n) << RCU_PLL1_PLL1R_SHIFT)

/* PLL2 register */

#define RCU_PLL2_PLL2PSC_SHIFT         (0)         /* Bits 0-5: PLL2 VCO source clock prescaler */
#define RCU_PLL2_PLL2PSC_MASK          (0x3f << RCU_PLL2_PLL2PSC_SHIFT)
#  define RCU_PLL2_PLL2PSC(n)          ((n) << RCU_PLL2_PLL2PSC_SHIFT)
#define RCU_PLL2_PLL2N_SHIFT           (6)         /* Bits 6-14: PLL2 VCO clock multi factor */
#define RCU_PLL2_PLL2N_MASK            (0x1ff << RCU_PLL2_PLL2N_SHIFT)
#  define RCU_PLL2_PLL2N(n)            ((n) << RCU_PLL2_PLL2N_SHIFT)
#define RCU_PLL2_PLL2P_SHIFT           (16)        /* Bits 16-22: PLL2P output frequency division factor */
#define RCU_PLL2_PLL2P_MASK            (0x7f << RCU_PLL2_PLL2P_SHIFT)
#  define RCU_PLL2_PLL2P(n)            ((n) << RCU_PLL2_PLL2P_SHIFT)
#define RCU_PLL2_PLL2R_SHIFT           (24)        /* Bits 24-30: PLL2R output frequency division factor */
#define RCU_PLL2_PLL2R_MASK            (0x7f << RCU_PLL2_PLL2R_SHIFT)
#  define RCU_PLL2_PLL2R(n)            ((n) << RCU_PLL2_PLL2R_SHIFT)

/* Clock configuration register 1 */

#define RCU_CFG1_USART0SEL_SHIFT       (0)         /* Bits 0-1: USART0 clock source selection */
#define RCU_CFG1_USART0SEL_MASK        (3 << RCU_CFG1_USART0SEL_SHIFT)
#define RCU_CFG1_RSPDIFSEL_SHIFT       (4)         /* Bits 4-5: RSPDIF clock source selection */
#define RCU_CFG1_RSPDIFSEL_MASK        (3 << RCU_CFG1_RSPDIFSEL_SHIFT)
#define RCU_CFG1_CAN0SEL_SHIFT         (8)         /* Bits 8-9: CAN0 clock source selection */
#define RCU_CFG1_CAN0SEL_MASK          (3 << RCU_CFG1_CAN0SEL_SHIFT)
#define RCU_CFG1_CAN1SEL_SHIFT         (10)        /* Bits 10-11: CAN1 clock source selection */
#define RCU_CFG1_CAN1SEL_MASK          (3 << RCU_CFG1_CAN1SEL_SHIFT)
#define RCU_CFG1_CAN2SEL_SHIFT         (12)        /* Bits 12-13: CAN2 clock source selection */
#define RCU_CFG1_CAN2SEL_MASK          (3 << RCU_CFG1_CAN2SEL_SHIFT)
#define RCU_CFG1_PERSEL_SHIFT          (14)        /* Bits 14-15: CK_PER clock source selection */
#define RCU_CFG1_PERSEL_MASK           (3 << RCU_CFG1_PERSEL_SHIFT)
#define RCU_CFG1_PLL2RDIV_SHIFT        (16)        /* Bits 16-17: PLL2R clock divider factor */
#define RCU_CFG1_PLL2RDIV_MASK         (3 << RCU_CFG1_PLL2RDIV_SHIFT)
#define RCU_CFG1_USART1SEL_SHIFT       (18)        /* Bits 18-19: USART1 clock source selection */
#define RCU_CFG1_USART1SEL_MASK        (3 << RCU_CFG1_USART1SEL_SHIFT)
#define RCU_CFG1_USART2SEL_SHIFT       (20)        /* Bits 20-21: USART2 clock source selection */
#define RCU_CFG1_USART2SEL_MASK        (3 << RCU_CFG1_USART2SEL_SHIFT)
#define RCU_CFG1_USART5SEL_SHIFT       (22)        /* Bits 22-23: USART5 clock source selection */
#define RCU_CFG1_USART5SEL_MASK        (3 << RCU_CFG1_USART5SEL_SHIFT)
#define RCU_CFG1_TIMERSEL              (1 << 24)   /* Bit 24: TIMER clock selection */
#define RCU_CFG1_HPDFSEL               (1 << 31)   /* Bit 31: HPDF clock source selection */

/* Clock configuration register 2 */

#define RCU_CFG2_CKOUT0DIV_SHIFT       (0)         /* Bits 0-3: CK_OUT0 divider */
#define RCU_CFG2_CKOUT0DIV_MASK        (0xf << RCU_CFG2_CKOUT0DIV_SHIFT)
#define RCU_CFG2_CKOUT0SEL_SHIFT       (4)         /* Bits 4-6: CKOUT0 clock source selection */
#define RCU_CFG2_CKOUT0SEL_MASK        (7 << RCU_CFG2_CKOUT0SEL_SHIFT)
#define RCU_CFG2_CKOUT1DIV_SHIFT       (8)         /* Bits 8-11: CK_OUT1 divider */
#define RCU_CFG2_CKOUT1DIV_MASK        (0xf << RCU_CFG2_CKOUT1DIV_SHIFT)
#define RCU_CFG2_CKOUT1SEL_SHIFT       (12)        /* Bits 12-14: CKOUT1 clock source selection */
#define RCU_CFG2_CKOUT1SEL_MASK        (7 << RCU_CFG2_CKOUT1SEL_SHIFT)
#define RCU_CFG2_SAI0SEL_SHIFT         (16)        /* Bits 16-18: SAI0 clock source selection */
#define RCU_CFG2_SAI0SEL_MASK          (7 << RCU_CFG2_SAI0SEL_SHIFT)
#define RCU_CFG2_SAI1SEL_SHIFT         (20)        /* Bits 20-22: SAI1 clock source selection */
#define RCU_CFG2_SAI1SEL_MASK          (7 << RCU_CFG2_SAI1SEL_SHIFT)
#define RCU_CFG2_SAI2B0SEL_SHIFT       (24)        /* Bits 24-26: SAI2 Block 0 clock source selection */
#define RCU_CFG2_SAI2B0SEL_MASK        (7 << RCU_CFG2_SAI2B0SEL_SHIFT)
#define RCU_CFG2_SAI2B1SEL_SHIFT       (28)        /* Bits 28-30: SAI2 Block 1 clock source selection */
#define RCU_CFG2_SAI2B1SEL_MASK        (7 << RCU_CFG2_SAI2B1SEL_SHIFT)

/* Clock configuration register 3 */

#define RCU_CFG3_I2C1SEL_SHIFT         (0)         /* Bits 0-1: I2C1 clock source selection */
#define RCU_CFG3_I2C1SEL_MASK          (3 << RCU_CFG3_I2C1SEL_SHIFT)
#define RCU_CFG3_I2C2SEL_SHIFT         (2)         /* Bits 2-3: I2C2 clock source selection */
#define RCU_CFG3_I2C2SEL_MASK          (3 << RCU_CFG3_I2C2SEL_SHIFT)
#define RCU_CFG3_I2C3SEL_SHIFT         (4)         /* Bits 4-5: I2C3 clock source selection */
#define RCU_CFG3_I2C3SEL_MASK          (3 << RCU_CFG3_I2C3SEL_SHIFT)
#define RCU_CFG3_SDIO1SEL              (1 << 12)   /* Bit 12: SDIO1 function clock source selection */
#define RCU_CFG3_DSPWUSSEL             (1 << 24)   /* Bit 24: deep-sleep wakeup system clock source selection */
#define RCU_CFG3_ADC01SEL_SHIFT        (26)        /* Bits 26-27: ADC0 and ADC1 clock source selection */
#define RCU_CFG3_ADC01SEL_MASK         (3 << RCU_CFG3_ADC01SEL_SHIFT)
#define RCU_CFG3_ADC2SEL_SHIFT         (28)        /* Bits 28-29: ADC2 clock source selection */
#define RCU_CFG3_ADC2SEL_MASK          (3 << RCU_CFG3_ADC2SEL_SHIFT)

/* PLL configuration register */

#define RCU_PLLALL_PLL0RNG_SHIFT       (0)         /* Bits 0-1: PLL0 input clock range */
#define RCU_PLLALL_PLL0RNG_MASK        (3 << RCU_PLLALL_PLL0RNG_SHIFT)
#define RCU_PLLALL_PLL0RNG(n)          ((n) << RCU_PLLALL_PLL0RNG_SHIFT)
#  define RCU_PLLALL_PLL0RNG_1M_2M     RCU_PLLALL_PLL0RNG(0)
#  define RCU_PLLALL_PLL0RNG_2M_4M     RCU_PLLALL_PLL0RNG(1)
#  define RCU_PLLALL_PLL0RNG_4M_8M     RCU_PLLALL_PLL0RNG(2)
#  define RCU_PLLALL_PLL0RNG_8M_16M    RCU_PLLALL_PLL0RNG(3)
#define RCU_PLLALL_PLL0VCOSEL          (1 << 2)    /* Bit 2: PLL0 VCO selection */
#define RCU_PLLALL_PLL1RNG_SHIFT       (4)         /* Bits 4-5: PLL1 input clock range */
#define RCU_PLLALL_PLL1RNG_MASK        (3 << RCU_PLLALL_PLL1RNG_SHIFT)
#define RCU_PLLALL_PLL1VCOSEL          (1 << 6)    /* Bit 6: PLL1 VCO selection */
#define RCU_PLLALL_PLL2RNG_SHIFT       (8)         /* Bits 8-9: PLL2 input clock range */
#define RCU_PLLALL_PLL2RNG_MASK        (3 << RCU_PLLALL_PLL2RNG_SHIFT)
#define RCU_PLLALL_PLL2VCOSEL          (1 << 10)   /* Bit 10: PLL2 VCO selection */
#define RCU_PLLALL_PLLSEL_SHIFT        (16)        /* Bits 16-17: PLLs clock source selection */
#define RCU_PLLALL_PLLSEL_MASK         (3 << RCU_PLLALL_PLLSEL_SHIFT)
#define RCU_PLLALL_PLLSEL(n)           ((n) << RCU_PLLALL_PLLSEL_SHIFT)
#  define RCU_PLLALL_PLLSEL_IRC64MDIV  RCU_PLLALL_PLLSEL(0)
#  define RCU_PLLALL_PLLSEL_LPIRC4M    RCU_PLLALL_PLLSEL(1)
#  define RCU_PLLALL_PLLSEL_HXTAL      RCU_PLLALL_PLLSEL(2)

/* PLL0 fraction configuration register */

#define RCU_PLL0FRA_PLL0FRAN_SHIFT     (0)         /* Bits 0-12: PLL0 VCO fractional part */
#define RCU_PLL0FRA_PLL0FRAN_MASK      (0x1fff << RCU_PLL0FRA_PLL0FRAN_SHIFT)
#define RCU_PLL0FRA_PLL0FRAEN          (1 << 15)   /* Bit 15: PLL0 fractional latch enable */

/* PLL1 fraction configuration register */

#define RCU_PLL1FRA_PLL1FRAN_SHIFT     (0)         /* Bits 0-12: PLL1 VCO fractional part */
#define RCU_PLL1FRA_PLL1FRAN_MASK      (0x1fff << RCU_PLL1FRA_PLL1FRAN_SHIFT)
#define RCU_PLL1FRA_PLL1FRAEN          (1 << 15)   /* Bit 15: PLL1 fractional latch enable */

/* PLL2 fraction configuration register */

#define RCU_PLL2FRA_PLL2FRAN_SHIFT     (0)         /* Bits 0-12: PLL2 VCO fractional part */
#define RCU_PLL2FRA_PLL2FRAN_MASK      (0x1fff << RCU_PLL2FRA_PLL2FRAN_SHIFT)
#define RCU_PLL2FRA_PLL2FRAEN          (1 << 15)   /* Bit 15: PLL2 fractional latch enable */

/* Additional clock control register 0 */

#define RCU_ADDCTL0_CK48MSEL           (1 << 0)    /* Bit 0: 48MHz clock selection */
#define RCU_ADDCTL0_PLL48MSEL          (1 << 1)    /* Bit 1: PLL48M clock selection */
#define RCU_ADDCTL0_IRC48MEN           (1 << 16)   /* Bit 16: internal 48MHz RC oscillator enable */
#define RCU_ADDCTL0_IRC48MSTB          (1 << 17)   /* Bit 17: internal 48MHz RC oscillator stabilization flag */
#define RCU_ADDCTL0_IRC48MCALB_SHIFT   (24)        /* Bits 24-31: internal 48MHz RC oscillator calibration value */
#define RCU_ADDCTL0_IRC48MCALB_MASK    (0xff << RCU_ADDCTL0_IRC48MCALB_SHIFT)

/* Additional clock control register 1 */

#define RCU_ADDCTL1_LPIRC4MEN          (1 << 0)    /* Bit 0: LPIRC4M oscillator enable */
#define RCU_ADDCTL1_LPIRC4MSTB         (1 << 1)    /* Bit 1: LPIRC4M oscillator stabilization flag */
#define RCU_ADDCTL1_LPIRC4MTRIM_SHIFT  (2)         /* Bits 2-7: LPIRC4M clock trim adjust value */
#define RCU_ADDCTL1_LPIRC4MTRIM_MASK   (0x3f << RCU_ADDCTL1_LPIRC4MTRIM_SHIFT)
#define RCU_ADDCTL1_LPIRC4MCAL_SHIFT   (8)         /* Bits 8-15: LPIRC4M clock calibration value */
#define RCU_ADDCTL1_LPIRC4MCAL_MASK    (0xff << RCU_ADDCTL1_LPIRC4MCAL_SHIFT)
#define RCU_ADDCTL1_IRC64MDIV_SHIFT    (16)        /* Bits 16-17: IRC64M clock divider */
#define RCU_ADDCTL1_IRC64MDIV_MASK     (3 << RCU_ADDCTL1_IRC64MDIV_SHIFT)
#define RCU_ADDCTL1_IRC64MDIV(n)       ((n) << RCU_ADDCTL1_IRC64MDIV_SHIFT)
#  define RCU_ADDCTL1_IRC64M_DIV1      RCU_ADDCTL1_IRC64MDIV(0)
#  define RCU_ADDCTL1_IRC64M_DIV2      RCU_ADDCTL1_IRC64MDIV(1)
#  define RCU_ADDCTL1_IRC64M_DIV4      RCU_ADDCTL1_IRC64MDIV(2)
#  define RCU_ADDCTL1_IRC64M_DIV8      RCU_ADDCTL1_IRC64MDIV(3)
#define RCU_ADDCTL1_LPIRC4MDSPEN       (1 << 20)   /* Bit 20: LPIRC4M clock enable in deepsleep mode */
#define RCU_ADDCTL1_PLLUSBHS0EN        (1 << 28)   /* Bit 28: PLLUSBHS0 enable */
#define RCU_ADDCTL1_PLLUSBHS0STB       (1 << 29)   /* Bit 29: PLLUSBHS0 stabilization flag */
#define RCU_ADDCTL1_PLLUSBHS1EN        (1 << 30)   /* Bit 30: PLLUSBHS1 enable */
#define RCU_ADDCTL1_PLLUSBHS1STB       (1 << 31)   /* Bit 31: PLLUSBHS1 stabilization flag */

/* Additional clock interrupt register */

#define RCU_ADDINT_PLLUSBHS0STBIF      (1 << 4)    /* Bit 4: PLLUSBHS0 stabilization interrupt flag */
#define RCU_ADDINT_PLLUSBHS1STBIF      (1 << 5)    /* Bit 5: PLLUSBHS1 stabilization interrupt flag */
#define RCU_ADDINT_IRC48MSTBIF         (1 << 6)    /* Bit 6: IRC48M stabilization interrupt flag */
#define RCU_ADDINT_PLLUSBHS0STBIE      (1 << 12)   /* Bit 12: PLLUSBHS0 stabilization interrupt enable */
#define RCU_ADDINT_PLLUSBHS1STBIE      (1 << 13)   /* Bit 13: PLLUSBHS1 stabilization interrupt enable */
#define RCU_ADDINT_IRC48MSTBIE         (1 << 14)   /* Bit 14: IRC48M stabilization interrupt enable */
#define RCU_ADDINT_PLLUSBHS0STBIC      (1 << 20)   /* Bit 20: PLLUSBHS0 stabilization interrupt clear */
#define RCU_ADDINT_PLLUSBHS1STBIC      (1 << 21)   /* Bit 21: PLLUSBHS1 stabilization interrupt clear */
#define RCU_ADDINT_IRC48MSTBIC         (1 << 22)   /* Bit 22: IRC48M stabilization interrupt clear */

/* Clock configuration register 4 */

#define RCU_CFG4_SDIO0SEL              (1 << 0)    /* Bit 0: SDIO0 clock source selection */
#define RCU_CFG4_EXMCSEL_SHIFT         (8)         /* Bits 8-9: EXMC clock source selection */
#define RCU_CFG4_EXMCSEL_MASK          (3 << RCU_CFG4_EXMCSEL_SHIFT)

/* USB clock control register */

#define RCU_USBCLKCTL_USBHS0SEL        (1 << 1)    /* Bit 1: USBHS0 clock selection */
#define RCU_USBCLKCTL_PLLUSBHS0PRESEL  (1 << 3)    /* Bit 3: PLLUSBHS0 clock source selection */
#define RCU_USBCLKCTL_USBHS0SWEN       (1 << 4)    /* Bit 4: USBHS0 clock source selection enable */
#define RCU_USBCLKCTL_USBHS048MSEL_SHIFT (5)       /* Bits 5-6: USBHS0 48M clock source selection */
#define RCU_USBCLKCTL_USBHS048MSEL_MASK (3 << RCU_USBCLKCTL_USBHS048MSEL_SHIFT)
#define RCU_USBCLKCTL_USBHS1SEL        (1 << 9)    /* Bit 9: USBHS1 clock selection */
#define RCU_USBCLKCTL_PLLUSBHS1PRESEL  (1 << 11)   /* Bit 11: PLLUSBHS1 clock source selection */
#define RCU_USBCLKCTL_USBHS1SWEN       (1 << 12)   /* Bit 12: USBHS1 clock source selection enable */
#define RCU_USBCLKCTL_USBHS148MSEL_SHIFT (13)      /* Bits 13-14: USBHS1 48M clock source selection */
#define RCU_USBCLKCTL_USBHS148MSEL_MASK (3 << RCU_USBCLKCTL_USBHS148MSEL_SHIFT)
#define RCU_USBCLKCTL_USBHS0PSC_SHIFT  (16)        /* Bits 16-18: USBHS0 clock prescaler selection */
#define RCU_USBCLKCTL_USBHS0PSC_MASK   (7 << RCU_USBCLKCTL_USBHS0PSC_SHIFT)
#define RCU_USBCLKCTL_USBHS1PSC_SHIFT  (19)        /* Bits 19-21: USBHS1 clock prescaler selection */
#define RCU_USBCLKCTL_USBHS1PSC_MASK   (7 << RCU_USBCLKCTL_USBHS1PSC_SHIFT)

/* PLLUSB configuration register */

#define RCU_PLLUSBCFG_PLLUSBHS0PREDV_SHIFT (0)     /* Bits 0-3: PLLUSBHS0PREDV clock prescaler */
#define RCU_PLLUSBCFG_PLLUSBHS0PREDV_MASK (0xf << RCU_PLLUSBCFG_PLLUSBHS0PREDV_SHIFT)
#define RCU_PLLUSBCFG_USBHS0DV_SHIFT   (4)         /* Bits 4-6: USBHS0 clock divider */
#define RCU_PLLUSBCFG_USBHS0DV_MASK    (7 << RCU_PLLUSBCFG_USBHS0DV_SHIFT)
#define RCU_PLLUSBCFG_PLLUSBHS0MF_SHIFT (8)        /* Bits 8-14: PLLUSBHS0 clock multiplication factor */
#define RCU_PLLUSBCFG_PLLUSBHS0MF_MASK (0x7f << RCU_PLLUSBCFG_PLLUSBHS0MF_SHIFT)
#define RCU_PLLUSBCFG_PLLUSBHS1PREDV_SHIFT (16)    /* Bits 16-19: PLLUSBHS1PREDV clock prescaler */
#define RCU_PLLUSBCFG_PLLUSBHS1PREDV_MASK (0xf << RCU_PLLUSBCFG_PLLUSBHS1PREDV_SHIFT)
#define RCU_PLLUSBCFG_USBHS1DV_SHIFT   (20)        /* Bits 20-22: USBHS1 clock divider */
#define RCU_PLLUSBCFG_USBHS1DV_MASK    (7 << RCU_PLLUSBCFG_USBHS1DV_SHIFT)
#define RCU_PLLUSBCFG_PLLUSBHS1MF_SHIFT (24)       /* Bits 24-30: PLLUSBHS1 clock multiplication factor */
#define RCU_PLLUSBCFG_PLLUSBHS1MF_MASK (0x7f << RCU_PLLUSBCFG_PLLUSBHS1MF_SHIFT)

/* APB2 additional reset register */

#define RCU_ADDAPB2RST_CAN0RST         (1 << 0)    /* Bit 0: CAN0 reset */
#define RCU_ADDAPB2RST_CAN1RST         (1 << 1)    /* Bit 1: CAN1 reset */
#define RCU_ADDAPB2RST_CAN2RST         (1 << 2)    /* Bit 2: CAN2 reset */

/* APB2 additional enable register */

#define RCU_ADDAPB2EN_CAN0EN           (1 << 0)    /* Bit 0: CAN0 clock enable */
#define RCU_ADDAPB2EN_CAN1EN           (1 << 1)    /* Bit 1: CAN1 clock enable */
#define RCU_ADDAPB2EN_CAN2EN           (1 << 2)    /* Bit 2: CAN2 clock enable */

/* APB2 additional sleep mode enable register */

#define RCU_ADDAPB2SPEN_CAN0SPEN       (1 << 0)    /* Bit 0: CAN0 clock enable when sleep mode */
#define RCU_ADDAPB2SPEN_CAN1SPEN       (1 << 1)    /* Bit 1: CAN1 clock enable when sleep mode */
#define RCU_ADDAPB2SPEN_CAN2SPEN       (1 << 2)    /* Bit 2: CAN2 clock enable when sleep mode */

/* Clock configuration register 5 */

#define RCU_CFG5_SPI0SEL_SHIFT         (0)         /* Bits 0-2: SPI0 and I2S0 clock source selection */
#define RCU_CFG5_SPI0SEL_MASK          (7 << RCU_CFG5_SPI0SEL_SHIFT)
#define RCU_CFG5_SPI1SEL_SHIFT         (4)         /* Bits 4-6: SPI1 and I2S1 clock source selection */
#define RCU_CFG5_SPI1SEL_MASK          (7 << RCU_CFG5_SPI1SEL_SHIFT)
#define RCU_CFG5_SPI2SEL_SHIFT         (8)         /* Bits 8-10: SPI2 and I2S2 clock source selection */
#define RCU_CFG5_SPI2SEL_MASK          (7 << RCU_CFG5_SPI2SEL_SHIFT)
#define RCU_CFG5_SPI3SEL_SHIFT         (12)        /* Bits 12-14: SPI3 clock source selection */
#define RCU_CFG5_SPI3SEL_MASK          (7 << RCU_CFG5_SPI3SEL_SHIFT)
#define RCU_CFG5_SPI4SEL_SHIFT         (16)        /* Bits 16-18: SPI4 clock source selection */
#define RCU_CFG5_SPI4SEL_MASK          (7 << RCU_CFG5_SPI4SEL_SHIFT)
#define RCU_CFG5_SPI5SEL_SHIFT         (20)        /* Bits 20-22: SPI5 and I2S5 clock source selection */
#define RCU_CFG5_SPI5SEL_MASK          (7 << RCU_CFG5_SPI5SEL_SHIFT)

#endif /* __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_RCU_H */
