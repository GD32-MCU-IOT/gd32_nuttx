/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h759im_memorymap.h
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

#ifndef __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H759IM_MEMORYMAP_H
#define __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H759IM_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GD32H7XX Address Blocks **************************************************/

#define GD32_CODE_BASE       0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define GD32_SRAM_BASE       0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define GD32_PERIPH_BASE     0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */

#define GD32_EXMC_REG_BASE   0xa0000000     /* 0xa0000000-0xbfffffff: 512Mb EXMC register block */
#define GD32_EXMC_BASE       0x60000000     /* 0x60000000-0x7fffffff: EXMC base */
#define GD32_EXMC_BANK0      0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/PSRAM/SRAM */
#define GD32_EXMC_BANK1      0x70000000     /* 0x70000000-0x7fffffff: 256Mb NAND FLASH */
#define GD32_EXMC_BANK2      0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#define GD32_EXMC_BANK3      0x90000000     /* 0x90000000-0x9fffffff: 256Mb PC CARD*/

#define GD32_CORTEX_BASE     0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M7 block */

#define GD32_REGION_MASK     0xf0000000
#define GD32_IS_SRAM(a)      ((((uint32_t)(a)) & GD32_REGION_MASK) == GD32_SRAM_BASE)
#define GD32_IS_EXTSRAM(a)   ((((uint32_t)(a)) & GD32_REGION_MASK) == GD32_EXMC_BANK0)

/* Code Base Addresses ******************************************************/

#define GD32_BOOT_BASE       0x00000000     /* 0x00000000-0x007fffff: Aliased to the boot device */
#define GD32_FLASH_BASE      0x08000000     /* 0x08000000-0x081fffff: Main flash (2MB) */
                                            /* 0x08200000-0x0fffffff: Reserved */
#define GD32_ITCMSRAM_BASE   0x00000000     /* 0x00000000-0x0000ffff: 64Kb ITCM SRAM */
#define GD32_DTCMSRAM_BASE   0x20000000     /* 0x20000000-0x2001ffff: 128Kb DTCM SRAM */
#define GD32_AXISRAM_BASE    0x24000000     /* 0x24000000-0x2407ffff: 512Kb AXI SRAM */
#define GD32_SRAM0_BASE      0x30000000     /* 0x30000000-0x30003fff: 16Kb SRAM0 */
#define GD32_SRAM1_BASE      0x30004000     /* 0x30004000-0x30007fff: 16Kb SRAM1 */
#define GD32_BKPSRAM_BASE    0x38800000     /* 0x38800000-0x38800fff: 4Kb Backup SRAM */

#define GD32_UNIQUE_ID       0x1ff1e800     /* The 96-bit unique device ID */
#define GD32_FLASH_DENSITY   0x1ff1e880     /* The memory density information */

/* SRAM Base Addresses ******************************************************/

#define GD32_SRAMBB_BASE     0x22000000     /* 0x22000000-          : SRAM bit-band region */
#define GD32_PERIPHBB_BASE   0x42000000     /* 0x42000000-          : Peripheral bit-band region */

/* Peripheral Base Addresses ************************************************/

#define GD32_APB1_BUS_BASE   0x40000000     /* APB1 base address */
#define GD32_APB2_BUS_BASE   0x40010000     /* APB2 base address */
#define GD32_APB3_BUS_BASE   0x50000000     /* APB3 base address */
#define GD32_APB4_BUS_BASE   0x58000000     /* APB4 base address */
#define GD32_AHB1_BUS_BASE   0x40020000     /* AHB1 base address */
#define GD32_AHB2_BUS_BASE   0x48000000     /* AHB2 base address */
#define GD32_AHB3_BUS_BASE   0x51000000     /* AHB3 base address */
#define GD32_AHB4_BUS_BASE   0x58020000     /* AHB4 base address */

/* APB1 Base Addresses ******************************************************/

#define GD32_TIMER_BASE      (GD32_APB1_BUS_BASE + 0x00000000U)     /* TIMER base address */
#define GD32_SPI_BASE        (GD32_APB1_BUS_BASE + 0x00003800U)     /* SPI base address */
#define GD32_RSPDIF_BASE     (GD32_APB1_BUS_BASE + 0x00004000U)     /* RSPDIF base address */
#define GD32_USART_BASE      (GD32_APB1_BUS_BASE + 0x00004400U)     /* USART base address */
#define GD32_I2C_BASE        (GD32_APB1_BUS_BASE + 0x00005400U)     /* I2C base address */
#define GD32_DAC_BASE        (GD32_APB1_BUS_BASE + 0x00007400U)     /* DAC base address */
#define GD32_CTC_BASE        (GD32_APB1_BUS_BASE + 0x00008400U)     /* CTC base address */
#define GD32_MDIO_BASE       (GD32_APB1_BUS_BASE + 0x00009400U)     /* MDIO base address */

/* APB2 Base Addresses ******************************************************/

#define GD32_ADC_BASE        (GD32_APB2_BUS_BASE + 0x00002400U)     /* ADC base address */
#define GD32_SAI_BASE        (GD32_APB2_BUS_BASE + 0x00005800U)     /* SAI base address */
#define GD32_HPDF_BASE       (GD32_APB2_BUS_BASE + 0x00007000U)     /* HPDF base address */
#define GD32_TRIGSEL_BASE    (GD32_APB2_BUS_BASE + 0x00008400U)     /* TRIGSEL base address */
#define GD32_EDOUT_BASE      (GD32_APB2_BUS_BASE + 0x00008800U)     /* EDOUT base address */
#define GD32_CAN_BASE        (GD32_APB2_BUS_BASE + 0x0000A000U)     /* CAN base address */

/* APB3 Base Addresses ******************************************************/

#define GD32_TLI_BASE        (GD32_APB3_BUS_BASE + 0x00001000U)     /* TLI base address */
#define GD32_WWDGT_BASE      (GD32_APB3_BUS_BASE + 0x00003000U)     /* WWDGT base address */

/* APB4 Base Addresses ******************************************************/

#define GD32_EXTI_BASE       (GD32_APB4_BUS_BASE + 0x00000000U)     /* EXTI base address */
#define GD32_SYSCFG_BASE     (GD32_APB4_BUS_BASE + 0x00000400U)     /* SYSCFG base address */
#define GD32_CMP_BASE        (GD32_APB4_BUS_BASE + 0x00003800U)     /* CMP base address */
#define GD32_VREF_BASE       (GD32_APB4_BUS_BASE + 0x00003C00U)     /* VREF base address */
#define GD32_RTC_BASE        (GD32_APB4_BUS_BASE + 0x00004000U)     /* RTC base address */
#define GD32_FWDGT_BASE      (GD32_APB4_BUS_BASE + 0x00004800U)     /* FWDGT base address */
#define GD32_PMU_BASE        (GD32_APB4_BUS_BASE + 0x00005800U)     /* PMU base address */
#define GD32_LPDTS_BASE      (GD32_APB4_BUS_BASE + 0x00006800U)     /* LPDTS base address */

/* AHB1 Base Addresses ******************************************************/

#define GD32_DMA_BASE        (GD32_AHB1_BUS_BASE + 0x00000000U)     /* DMA base address */
#define GD32_DMAMUX_BASE     (GD32_AHB1_BUS_BASE + 0x00000800U)     /* DMAMUX base address */
#define GD32_EFUSE_BASE      (GD32_AHB1_BUS_BASE + 0x00002800U)     /* EFUSE base address */
#define GD32_ENET_BASE       (GD32_AHB1_BUS_BASE + 0x00008000U)     /* ENET base address */
#define GD32_USBHS_BASE      (GD32_AHB1_BUS_BASE + 0x00020000U)     /* USBHS base address */

/* AHB2 Base Addresses ******************************************************/

#define GD32_DCI_BASE        (GD32_AHB2_BUS_BASE + 0x00020000U)     /* DCI base address */
#define GD32_CAU_BASE        (GD32_AHB2_BUS_BASE + 0x00021000U)     /* CAU base address */
#define GD32_HAU_BASE        (GD32_AHB2_BUS_BASE + 0x00021400U)     /* HAU base address */
#define GD32_TRNG_BASE       (GD32_AHB2_BUS_BASE + 0x00021800U)     /* TRNG base address */
#define GD32_SDIO_BASE       (GD32_AHB2_BUS_BASE + 0x00022400U)     /* SDIO base address */
#define GD32_CPDM_BASE       (GD32_AHB2_BUS_BASE + 0x00022800U)     /* CPDM base address */
#define GD32_RAMECCMU_BASE   (GD32_AHB2_BUS_BASE + 0x00023000U)     /* RAMECCMU base address */
#define GD32_TMU_BASE        (GD32_AHB2_BUS_BASE + 0x00024400U)     /* TMU base address */
#define GD32_FAC_BASE        (GD32_AHB2_BUS_BASE + 0x00024800U)     /* FAC base address */

/* AHB3 Base Addresses ******************************************************/

#define GD32_AXIM_BASE       (GD32_AHB3_BUS_BASE + 0x00000000U)     /* AXIM base address */
#define GD32_MDMA_BASE       (GD32_AHB3_BUS_BASE + 0x01000000U)     /* MDMA base address */
#define GD32_IPA_BASE        (GD32_AHB3_BUS_BASE + 0x01001000U)     /* IPA base address */
#define GD32_FMC_BASE        (GD32_AHB3_BUS_BASE + 0x01002000U)     /* FMC base address */
#define GD32_FLEXRAMC_BASE   (GD32_AHB3_BUS_BASE + 0x01003000U)     /* FLEXRAMC base address */
#define GD32_EXMC_BASE_ADDR  (GD32_AHB3_BUS_BASE + 0x01004000U)     /* EXMC base address */
#define GD32_OSPI_BASE       (GD32_AHB3_BUS_BASE + 0x01005000U)     /* OSPI base address */
#define GD32_OSPM_BASE       (GD32_AHB3_BUS_BASE + 0x0100B400U)     /* OSPM base address */
#define GD32_RTDEC_BASE      (GD32_AHB3_BUS_BASE + 0x0100B800U)     /* RTDEC base address */

/* AHB4 Base Addresses ******************************************************/

#define GD32_GPIO_BASE       (GD32_AHB4_BUS_BASE + 0x00000000U)     /* GPIO base address */
#define GD32_RCU_BASE        (GD32_AHB4_BUS_BASE + 0x00004400U)     /* RCU base address */
#define GD32_CRC_BASE        (GD32_AHB4_BUS_BASE + 0x00004C00U)     /* CRC base address */
#define GD32_HWSEM_BASE      (GD32_AHB4_BUS_BASE + 0x00006400U)     /* HWSEM base address */

/* Option Byte and Debug Addresses ******************************************/

#define GD32_OB_BASE         0x1FFFF800     /* Option Byte base address */
#define GD32_DBG_BASE        0xE00E1000     /* Debug MCU base address */

#endif /* __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H759IM_MEMORYMAP_H */