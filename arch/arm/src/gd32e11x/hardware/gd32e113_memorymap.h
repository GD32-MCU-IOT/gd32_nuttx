/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e113_memorymap.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E113_MEMORYMAP_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E113_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GD32E113 Address Blocks ************************************************/

#define GD32_CODE_BASE       0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define GD32_SRAM_BASE       0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define GD32_PERIPH_BASE     0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define GD32_EXMC_BASE       0x60000000     /* 0x60000000-0x9fffffff: EXMC region */
#define GD32_EXMC_REG_BASE   0xa0000000     /* 0xa0000000-0xbfffffff: EXMC register block */
#define GD32_CORTEX_BASE     0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define GD32_REGION_MASK     0xf0000000
#define GD32_IS_SRAM(a)      ((((uint32_t)(a)) & GD32_REGION_MASK) == GD32_SRAM_BASE)
#define GD32_IS_EXTSRAM(a)   ((((uint32_t)(a)) & GD32_REGION_MASK) == GD32_EXMC_BASE)

/* Code Base Addresses ******************************************************/

#define GD32_BOOT_BASE       0x00000000     /* 0x00000000-0x0001ffff: Aliased to boot device */
#define GD32_FLASH_BASE      0x08000000     /* 0x08000000-0x0801ffff: Main flash (128KB) */
                                            /* 0x08020000-0x082fffff: Reserved */
                                            /* 0x08300000-0x0fffffff: Reserved */
                                            /* 0x10000000-0x1ffebfff: Reserved */
#define GD32_BOOTLD_BASE     0x1fffc000     /* 0x1fffc000-0x1fffc00f: Boot loader */
#define GD32_OPBYTE_BASE     0x1ffff800     /* 0x1ffff800-0x1ffff80f: Option bytes */
                                            /* 0x1ffff810-0x1fffffff: Reserved */

/* System Memory Addresses **************************************************/

#define GD32_UNIQUE_ID       0x1ffff7e8     /* The 96-bit unique device ID */
#define GD32_DBG_BASE        0xe0042000     /* DBG base address */

/* SRAM Base Addresses ******************************************************/

/* GD32E113: 32KB SRAM at 0x20000000-0x20007fff */

/* Peripheral Base Addresses ************************************************/

#define GD32_APB1_BUS_BASE   0x40000000     /* APB1 base address */
#define GD32_APB2_BUS_BASE   0x40010000     /* APB2 base address */
#define GD32_AHB1_BUS_BASE   0x40018000     /* AHB1 base address */
#define GD32_AHB3_BUS_BASE   0x60000000     /* AHB3 base address */

/* APB1 Base Addresses ******************************************************/

#define GD32_TIMER_BASE      (GD32_APB1_BUS_BASE + 0x00000000U)  /* TIMER base address */
#define GD32_RTC_BASE        (GD32_APB1_BUS_BASE + 0x00002800U)  /* RTC base address */
#define GD32_WWDGT_BASE      (GD32_APB1_BUS_BASE + 0x00002c00U)  /* WWDGT base address */
#define GD32_FWDGT_BASE      (GD32_APB1_BUS_BASE + 0x00003000U)  /* FWDGT base address */
#define GD32_SPI_BASE        (GD32_APB1_BUS_BASE + 0x00003800U)  /* SPI base address */
#define GD32_USART_BASE      (GD32_APB1_BUS_BASE + 0x00004400U)  /* USART base address */
#define GD32_I2C_BASE        (GD32_APB1_BUS_BASE + 0x00005400U)  /* I2C base address */
#define GD32_USBD_BASE       (GD32_APB1_BUS_BASE + 0x00005c00U)  /* USBD base address */
#define GD32_CAN_BASE        (GD32_APB1_BUS_BASE + 0x00006400U)  /* CAN base address */
#define GD32_BKP_BASE        (GD32_APB1_BUS_BASE + 0x00006c00U)  /* BKP base address */
#define GD32_PMU_BASE        (GD32_APB1_BUS_BASE + 0x00007000U)  /* PMU base address */
#define GD32_DAC_BASE        (GD32_APB1_BUS_BASE + 0x00007400U)  /* DAC base address */
#define GD32_CTC_BASE        (GD32_APB1_BUS_BASE + 0x0000c800U)  /* CTC base address */

/* APB2 Base Addresses ******************************************************/

#define GD32_AFIO_BASE       (GD32_APB2_BUS_BASE + 0x00000000U)  /* AFIO base address */
#define GD32_EXTI_BASE       (GD32_APB2_BUS_BASE + 0x00000400U)  /* EXTI base address */
#define GD32_GPIO_BASE       (GD32_APB2_BUS_BASE + 0x00000800U)  /* GPIO base address */
#define GD32_ADC_BASE        (GD32_APB2_BUS_BASE + 0x00002400U)  /* ADC base address */

/* AHB1 Base Addresses ******************************************************/

#define GD32_DMA_BASE        (GD32_AHB1_BUS_BASE + 0x00008000U)  /* DMA base address */
#define GD32_RCU_BASE        (GD32_AHB1_BUS_BASE + 0x00009000U)  /* RCU base address */
#define GD32_FMC_BASE        (GD32_AHB1_BUS_BASE + 0x0000a000U)  /* FMC base address */
#define GD32_CRC_BASE        (GD32_AHB1_BUS_BASE + 0x0000b000U)  /* CRC base address */
#define GD32_USBFS_BASE      (GD32_AHB1_BUS_BASE + 0x0ffe8000U)  /* USBFS base address */

/* AHB3 Base Addresses ******************************************************/

#define GD32_EXMC_SWREG_BASE (GD32_EXMC_REG_BASE + 0x00000000U)  /* EXMC SWREG base address */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E113_MEMORYMAP_H */
