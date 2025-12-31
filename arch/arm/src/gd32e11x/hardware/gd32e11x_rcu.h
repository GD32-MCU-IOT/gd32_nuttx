/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_rcu.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_RCU_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_RCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_RCU_CTL_OFFSET            0x0000  /* Control register */
#define GD32_RCU_CFG0_OFFSET           0x0004  /* Clock configuration register 0 */
#define GD32_RCU_INT_OFFSET            0x0008  /* Clock interrupt register */
#define GD32_RCU_APB2RST_OFFSET        0x000c  /* APB2 reset register */
#define GD32_RCU_APB1RST_OFFSET        0x0010  /* APB1 reset register */
#define GD32_RCU_AHBEN_OFFSET          0x0014  /* AHB enable register */
#define GD32_RCU_APB2EN_OFFSET         0x0018  /* APB2 enable register */
#define GD32_RCU_APB1EN_OFFSET         0x001c  /* APB1 enable register */
#define GD32_RCU_BDCTL_OFFSET          0x0020  /* Backup domain control register */
#define GD32_RCU_RSTSCK_OFFSET         0x0024  /* Reset source / clock register */
#define GD32_RCU_AHBRST_OFFSET         0x0028  /* AHB reset register */
#define GD32_RCU_CFG1_OFFSET           0x002c  /* Clock configuration register 1 */
#define GD32_RCU_DSV_OFFSET            0x0034  /* Deep-sleep mode voltage register */

/* Register Addresses *******************************************************/

#define GD32_RCU_CTL                   (GD32_RCU_BASE+GD32_RCU_CTL_OFFSET)
#define GD32_RCU_CFG0                  (GD32_RCU_BASE+GD32_RCU_CFG0_OFFSET)
#define GD32_RCU_INT                   (GD32_RCU_BASE+GD32_RCU_INT_OFFSET)
#define GD32_RCU_APB2RST               (GD32_RCU_BASE+GD32_RCU_APB2RST_OFFSET)
#define GD32_RCU_APB1RST               (GD32_RCU_BASE+GD32_RCU_APB1RST_OFFSET)
#define GD32_RCU_AHBEN                 (GD32_RCU_BASE+GD32_RCU_AHBEN_OFFSET)
#define GD32_RCU_APB2EN                (GD32_RCU_BASE+GD32_RCU_APB2EN_OFFSET)
#define GD32_RCU_APB1EN                (GD32_RCU_BASE+GD32_RCU_APB1EN_OFFSET)
#define GD32_RCU_BDCTL                 (GD32_RCU_BASE+GD32_RCU_BDCTL_OFFSET)
#define GD32_RCU_RSTSCK                (GD32_RCU_BASE+GD32_RCU_RSTSCK_OFFSET)
#define GD32_RCU_AHBRST                (GD32_RCU_BASE+GD32_RCU_AHBRST_OFFSET)
#define GD32_RCU_CFG1                  (GD32_RCU_BASE+GD32_RCU_CFG1_OFFSET)
#define GD32_RCU_DSV                   (GD32_RCU_BASE+GD32_RCU_DSV_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Control register (CTL) */

#define RCU_CTL_IRC8MEN                (1 << 0)        /* Internal 8MHz RC oscillator enable */
#define RCU_CTL_IRC8MSTB               (1 << 1)        /* IRC8M stabilization flag */
#define RCU_CTL_IRC8MADJ_SHIFT         (3)             /* IRC8M adjust value shift */
#define RCU_CTL_IRC8MADJ_MASK          (0x1f << RCU_CTL_IRC8MADJ_SHIFT)
#define RCU_CTL_IRC8MCALIB_SHIFT       (8)             /* IRC8M calibration value shift */
#define RCU_CTL_IRC8MCALIB_MASK        (0xff << RCU_CTL_IRC8MCALIB_SHIFT)
#define RCU_CTL_HXTALEN                (1 << 16)       /* External high speed oscillator enable */
#define RCU_CTL_HXTALSTB               (1 << 17)       /* HXTAL stabilization flag */
#define RCU_CTL_HXTALBPS               (1 << 18)       /* HXTAL bypass mode enable */
#define RCU_CTL_CKMEN                  (1 << 19)       /* Clock monitor enable */
#define RCU_CTL_PLLEN                  (1 << 24)       /* PLL enable */
#define RCU_CTL_PLLSTB                 (1 << 25)       /* PLL stabilization flag */
#define RCU_CTL_PLL1EN                 (1 << 26)       /* PLL1 enable */
#define RCU_CTL_PLL1STB                (1 << 27)       /* PLL1 stabilization flag */
#define RCU_CTL_PLL2EN                 (1 << 28)       /* PLL2 enable */
#define RCU_CTL_PLL2STB                (1 << 29)       /* PLL2 stabilization flag */

/* Clock configuration register 0 (CFG0) */

#define RCU_CFG0_SCS_SHIFT             (0)             /* System clock switch */
#define RCU_CFG0_SCS_MASK              (0x3 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_IRC8M           (0x0 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_HXTAL           (0x1 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_PLL             (0x2 << RCU_CFG0_SCS_SHIFT)

#define RCU_CFG0_SCSS_SHIFT            (2)             /* System clock switch status */
#define RCU_CFG0_SCSS_MASK             (0x3 << RCU_CFG0_SCSS_SHIFT)

#define RCU_CFG0_AHBPSC_SHIFT          (4)             /* AHB prescaler */
#define RCU_CFG0_AHBPSC_MASK           (0xf << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV1         (0x0 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV2         (0x8 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV4         (0x9 << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV8         (0xa << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV16        (0xb << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV64        (0xc << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV128       (0xd << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV256       (0xe << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_DIV512       (0xf << RCU_CFG0_AHBPSC_SHIFT)

#define RCU_CFG0_APB1PSC_SHIFT         (8)             /* APB1 prescaler */
#define RCU_CFG0_APB1PSC_MASK          (0x7 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_DIV1        (0x0 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_DIV2        (0x4 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_DIV4        (0x5 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_DIV8        (0x6 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_DIV16       (0x7 << RCU_CFG0_APB1PSC_SHIFT)

#define RCU_CFG0_APB2PSC_SHIFT         (11)            /* APB2 prescaler */
#define RCU_CFG0_APB2PSC_MASK          (0x7 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_DIV1        (0x0 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_DIV2        (0x4 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_DIV4        (0x5 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_DIV8        (0x6 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_DIV16       (0x7 << RCU_CFG0_APB2PSC_SHIFT)

#define RCU_CFG0_ADCPSC_SHIFT          (14)            /* ADC prescaler */
#define RCU_CFG0_ADCPSC_MASK           (0x3 << RCU_CFG0_ADCPSC_SHIFT)

#define RCU_CFG0_PLLSEL                (1 << 16)       /* PLL clock source selection */
#define RCU_CFG0_PREDV0_LSB            (1 << 17)       /* PREDV0 LSB */

#define RCU_CFG0_PLLMF_SHIFT           (18)            /* PLL multiply factor */
#define RCU_CFG0_PLLMF_MASK            (0xf << RCU_CFG0_PLLMF_SHIFT)

#define RCU_CFG0_USBFSPSC_SHIFT        (22)            /* USBFS prescaler */
#define RCU_CFG0_USBFSPSC_MASK         (0x3 << RCU_CFG0_USBFSPSC_SHIFT)

#define RCU_CFG0_CKOUT0SEL_SHIFT       (24)            /* CKOUT0 clock source selection */
#define RCU_CFG0_CKOUT0SEL_MASK        (0xf << RCU_CFG0_CKOUT0SEL_SHIFT)

#define RCU_CFG0_PLLMF_MSB             (1 << 29)       /* PLL multiply factor MSB */
#define RCU_CFG0_PLLDV                 (1 << 31)       /* PLL divide by 2 for clock */

/* Clock interrupt register (INT) */

#define RCU_INT_IRC40KSTBIF            (1 << 0)        /* IRC40K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF             (1 << 1)        /* LXTAL stabilization interrupt flag */
#define RCU_INT_IRC8MSTBIF             (1 << 2)        /* IRC8M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF             (1 << 3)        /* HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF               (1 << 4)        /* PLL stabilization interrupt flag */
#define RCU_INT_PLL1STBIF              (1 << 5)        /* PLL1 stabilization interrupt flag */
#define RCU_INT_PLL2STBIF              (1 << 6)        /* PLL2 stabilization interrupt flag */
#define RCU_INT_CKMIF                  (1 << 7)        /* Clock monitor interrupt flag */
#define RCU_INT_IRC40KSTBIE            (1 << 8)        /* IRC40K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE             (1 << 9)        /* LXTAL stabilization interrupt enable */
#define RCU_INT_IRC8MSTBIE             (1 << 10)       /* IRC8M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE             (1 << 11)       /* HXTAL stabilization interrupt enable */
#define RCU_INT_PLLSTBIE               (1 << 12)       /* PLL stabilization interrupt enable */
#define RCU_INT_PLL1STBIE              (1 << 13)       /* PLL1 stabilization interrupt enable */
#define RCU_INT_PLL2STBIE              (1 << 14)       /* PLL2 stabilization interrupt enable */
#define RCU_INT_IRC40KSTBIC            (1 << 16)       /* IRC40K stabilization interrupt clear */
#define RCU_INT_LXTALSTBIC             (1 << 17)       /* LXTAL stabilization interrupt clear */
#define RCU_INT_IRC8MSTBIC             (1 << 18)       /* IRC8M stabilization interrupt clear */
#define RCU_INT_HXTALSTBIC             (1 << 19)       /* HXTAL stabilization interrupt clear */
#define RCU_INT_PLLSTBIC               (1 << 20)       /* PLL stabilization interrupt clear */
#define RCU_INT_PLL1STBIC              (1 << 21)       /* PLL1 stabilization interrupt clear */
#define RCU_INT_PLL2STBIC              (1 << 22)       /* PLL2 stabilization interrupt clear */
#define RCU_INT_CKMIC                  (1 << 23)       /* Clock monitor interrupt clear */

/* AHB enable register (AHBEN) */

#define RCU_AHBEN_DMA0EN               (1 << 0)        /* DMA0 clock enable */
#define RCU_AHBEN_DMA1EN               (1 << 1)        /* DMA1 clock enable */
#define RCU_AHBEN_SRAMSPEN             (1 << 2)        /* SRAM interface clock enable */
#define RCU_AHBEN_FMCSPEN              (1 << 4)        /* FMC clock enable */
#define RCU_AHBEN_CRCEN                (1 << 6)        /* CRC clock enable */
#define RCU_AHBEN_EXMCEN               (1 << 8)        /* EXMC clock enable */
#define RCU_AHBEN_USBFSEN              (1 << 12)       /* USBFS clock enable */

/* APB2 enable register (APB2EN) */

#define RCU_APB2EN_AFEN                (1 << 0)        /* Alternate function I/O clock enable */
#define RCU_APB2EN_PAEN                (1 << 2)        /* GPIO port A clock enable */
#define RCU_APB2EN_PBEN                (1 << 3)        /* GPIO port B clock enable */
#define RCU_APB2EN_PCEN                (1 << 4)        /* GPIO port C clock enable */
#define RCU_APB2EN_PDEN                (1 << 5)        /* GPIO port D clock enable */
#define RCU_APB2EN_PEEN                (1 << 6)        /* GPIO port E clock enable */
#define RCU_APB2EN_ADC0EN              (1 << 9)        /* ADC0 clock enable */
#define RCU_APB2EN_ADC1EN              (1 << 10)       /* ADC1 clock enable */
#define RCU_APB2EN_TIMER0EN            (1 << 11)       /* TIMER0 clock enable */
#define RCU_APB2EN_SPI0EN              (1 << 12)       /* SPI0 clock enable */
#define RCU_APB2EN_TIMER7EN            (1 << 13)       /* TIMER7 clock enable */
#define RCU_APB2EN_USART0EN            (1 << 14)       /* USART0 clock enable */
#define RCU_APB2EN_TIMER8EN            (1 << 19)       /* TIMER8 clock enable */
#define RCU_APB2EN_TIMER9EN            (1 << 20)       /* TIMER9 clock enable */
#define RCU_APB2EN_TIMER10EN           (1 << 21)       /* TIMER10 clock enable */

/* APB1 enable register (APB1EN) */

#define RCU_APB1EN_TIMER1EN            (1 << 0)        /* TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN            (1 << 1)        /* TIMER2 clock enable */
#define RCU_APB1EN_TIMER3EN            (1 << 2)        /* TIMER3 clock enable */
#define RCU_APB1EN_TIMER4EN            (1 << 3)        /* TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN            (1 << 4)        /* TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN            (1 << 5)        /* TIMER6 clock enable */
#define RCU_APB1EN_TIMER11EN           (1 << 6)        /* TIMER11 clock enable */
#define RCU_APB1EN_TIMER12EN           (1 << 7)        /* TIMER12 clock enable */
#define RCU_APB1EN_TIMER13EN           (1 << 8)        /* TIMER13 clock enable */
#define RCU_APB1EN_WWDGTEN             (1 << 11)       /* Window watchdog timer clock enable */
#define RCU_APB1EN_SPI1EN              (1 << 14)       /* SPI1 clock enable */
#define RCU_APB1EN_SPI2EN              (1 << 15)       /* SPI2 clock enable */
#define RCU_APB1EN_USART1EN            (1 << 17)       /* USART1 clock enable */
#define RCU_APB1EN_USART2EN            (1 << 18)       /* USART2 clock enable */
#define RCU_APB1EN_UART3EN             (1 << 19)       /* UART3 clock enable */
#define RCU_APB1EN_UART4EN             (1 << 20)       /* UART4 clock enable */
#define RCU_APB1EN_I2C0EN              (1 << 21)       /* I2C0 clock enable */
#define RCU_APB1EN_I2C1EN              (1 << 22)       /* I2C1 clock enable */
#define RCU_APB1EN_BKPIEN              (1 << 27)       /* Backup interface clock enable */
#define RCU_APB1EN_PMUEN               (1 << 28)       /* Power control clock enable */
#define RCU_APB1EN_DACEN               (1 << 29)       /* DAC clock enable */

/* APB2 reset register (APB2RST) */

#define RCU_APB2RST_AFRST              (1 << 0)        /* Alternate function I/O reset */
#define RCU_APB2RST_PARST              (1 << 2)        /* GPIO port A reset */
#define RCU_APB2RST_PBRST              (1 << 3)        /* GPIO port B reset */
#define RCU_APB2RST_PCRST              (1 << 4)        /* GPIO port C reset */
#define RCU_APB2RST_PDRST              (1 << 5)        /* GPIO port D reset */
#define RCU_APB2RST_PERST              (1 << 6)        /* GPIO port E reset */
#define RCU_APB2RST_ADC0RST            (1 << 9)        /* ADC0 reset */
#define RCU_APB2RST_ADC1RST            (1 << 10)       /* ADC1 reset */
#define RCU_APB2RST_TIMER0RST          (1 << 11)       /* TIMER0 reset */
#define RCU_APB2RST_SPI0RST            (1 << 12)       /* SPI0 reset */
#define RCU_APB2RST_TIMER7RST          (1 << 13)       /* TIMER7 reset */
#define RCU_APB2RST_USART0RST          (1 << 14)       /* USART0 reset */
#define RCU_APB2RST_TIMER8RST          (1 << 19)       /* TIMER8 reset */
#define RCU_APB2RST_TIMER9RST          (1 << 20)       /* TIMER9 reset */
#define RCU_APB2RST_TIMER10RST         (1 << 21)       /* TIMER10 reset */

/* APB1 reset register (APB1RST) */

#define RCU_APB1RST_TIMER1RST          (1 << 0)        /* TIMER1 reset */
#define RCU_APB1RST_TIMER2RST          (1 << 1)        /* TIMER2 reset */
#define RCU_APB1RST_TIMER3RST          (1 << 2)        /* TIMER3 reset */
#define RCU_APB1RST_TIMER4RST          (1 << 3)        /* TIMER4 reset */
#define RCU_APB1RST_TIMER5RST          (1 << 4)        /* TIMER5 reset */
#define RCU_APB1RST_TIMER6RST          (1 << 5)        /* TIMER6 reset */
#define RCU_APB1RST_TIMER11RST         (1 << 6)        /* TIMER11 reset */
#define RCU_APB1RST_TIMER12RST         (1 << 7)        /* TIMER12 reset */
#define RCU_APB1RST_TIMER13RST         (1 << 8)        /* TIMER13 reset */
#define RCU_APB1RST_WWDGTRST           (1 << 11)       /* Window watchdog timer reset */
#define RCU_APB1RST_SPI1RST            (1 << 14)       /* SPI1 reset */
#define RCU_APB1RST_SPI2RST            (1 << 15)       /* SPI2 reset */
#define RCU_APB1RST_USART1RST          (1 << 17)       /* USART1 reset */
#define RCU_APB1RST_USART2RST          (1 << 18)       /* USART2 reset */
#define RCU_APB1RST_UART3RST           (1 << 19)       /* UART3 reset */
#define RCU_APB1RST_UART4RST           (1 << 20)       /* UART4 reset */
#define RCU_APB1RST_I2C0RST            (1 << 21)       /* I2C0 reset */
#define RCU_APB1RST_I2C1RST            (1 << 22)       /* I2C1 reset */
#define RCU_APB1RST_BKPIRST            (1 << 27)       /* Backup interface reset */
#define RCU_APB1RST_PMURST             (1 << 28)       /* Power control reset */
#define RCU_APB1RST_DACRST             (1 << 29)       /* DAC reset */

/* AHB reset register (AHBRST) */

#define RCU_AHBRST_USBFSRST            (1 << 12)       /* USBFS reset */

/* Backup domain control register (BDCTL) */

#define RCU_BDCTL_LXTALEN              (1 << 0)        /* LXTAL enable */
#define RCU_BDCTL_LXTALSTB             (1 << 1)        /* LXTAL stabilization flag */
#define RCU_BDCTL_LXTALBPS             (1 << 2)        /* LXTAL bypass mode enable */
#define RCU_BDCTL_RTCSRC_SHIFT         (8)             /* RTC clock entry selection */
#define RCU_BDCTL_RTCSRC_MASK          (0x3 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_NONE        (0x0 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_LXTAL       (0x1 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_IRC40K      (0x2 << RCU_BDCTL_RTCSRC_SHIFT)
#  define RCU_BDCTL_RTCSRC_HXTAL       (0x3 << RCU_BDCTL_RTCSRC_SHIFT)
#define RCU_BDCTL_RTCEN                (1 << 15)       /* RTC clock enable */
#define RCU_BDCTL_BKPRST               (1 << 16)       /* Backup domain reset */

/* Reset source / clock register (RSTSCK) */

#define RCU_RSTSCK_IRC40KEN            (1 << 0)        /* IRC40K enable */
#define RCU_RSTSCK_IRC40KSTB           (1 << 1)        /* IRC40K stabilization flag */
#define RCU_RSTSCK_RSTFC               (1 << 24)       /* Reset flag clear */
#define RCU_RSTSCK_EPRSTF              (1 << 26)       /* External pin reset flag */
#define RCU_RSTSCK_PORRSTF             (1 << 27)       /* Power reset flag */
#define RCU_RSTSCK_SWRSTF              (1 << 28)       /* Software reset flag */
#define RCU_RSTSCK_FWDGTRSTF           (1 << 29)       /* Free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF           (1 << 30)       /* Window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF              (1 << 31)       /* Low-power reset flag */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_RCU_H */
