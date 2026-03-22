/****************************************************************************
 * arch/arm/include/gd32h7xx/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_GD32H7_CHIP_H
#define __ARCH_ARM_INCLUDE_GD32H7_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_GD32H759IM)
#  define GD32_FLASH_SIZE          (3840*1024)   /* 3840KB */
#  define GD32_ITCM_SIZE           (64*1024)     /* 64KB ITCM */
#  define GD32_DTCM_SIZE           (128*1024)    /* 128KB DTCM */
#  define GD32_AXI_SRAM_SIZE       (512*1024)    /* 512KB AXI SRAM */
#  define GD32_SRAM1_SIZE          (16*1024)     /* 16KB SRAM1 */
#  define GD32_SRAM2_SIZE          (16*1024)     /* 16KB SRAM2 */
#  define GD32_BACKUP_SRAM_SIZE    (4*1024)      /* 4KB Backup SRAM */
#  define GD32_SRAM_SIZE           (GD32_DTCM_SIZE + GD32_AXI_SRAM_SIZE + \
                                    GD32_SRAM1_SIZE + GD32_SRAM2_SIZE)

#  define GD32_NGPIO_PORTS         10             /* Ports A-K, not include I */
#  define GD32_NATIMER             2              /* Advanced timers: TIMER0,7 */
#  define GD32_NGTIMER             4              /* General timers: TIMER1,4, 22-23 */
#  define GD32_NGTIMER_16          12             /* General timers (16-bit): TIMER2-3, 14-16, 30-31, 40-44 */
#  define GD32_NBTIMER             2              /* Basic timers: TIMER5-6, */
#  define GD32_NBTIMER_64          2              /* Basic timers (64-bit): TIMER50-51 */
#  define GD32_NUSART              8              /* USART0-2,5, UART3-4,6-7 */
#  define GD32_NI2C                4              /* I2C0-3 */
#  define GD32_NSPI                6              /* SPI0-5 */
#  define GD32_NADC                3              /* ADC0-2 */
#  define GD32_NDAC                1              /* DAC */
#  define GD32_NCAN                3              /* CAN0-2 */
#  define GD32_NSDIO               2              /* SDIO0-1 */
#  define GD32_NCPDM               2              /* Clock phase delay module */
#  define GD32_NSAI                3              /* SAI0-2 */
#  define GD32_NEXMC               1              /* External memory controller */
#  define GD32_NENET               2              /* ENET0-1 */
#  define GD32_NDMA                2              /* DMA0-1 */
#  define GD32_NMDMA               1              /* MDMA */
#  define GD32_NDMAMUX             1              /* DMAMUX */
#  define GD32_NUSBHS              2              /* USB HS0-1 OTG */
#  define GD32_NOSPI               2              /* OSPI0-1 */
#  define GD32_NTLI                1              /* TFT-LCD interface */
#  define GD32_NDCI                1              /* Digital camera interface */
#  define GD32_NIPA                1              /* Image processing accelerator */
#  define GD32_NHPDF               1              /* High-pass digital filter */
#  define GD32_NCAU                1              /* Cryptographic acceleration unit */
#  define GD32_NHAU                1              /* Hash acceleration unit */
#  define GD32_NTRNG               1              /* True random number generator */
#  define GD32_NCRC                1              /* CRC calculation unit */
#  define GD32_NRTC                1              /* Real-time clock */
#  define GD32_NWWDGT              1              /* Window watchdog */
#  define GD32_NFWDGT              1              /* Free watchdog */
#  define GD32_NHWSEM              1              /* Hardware semaphore */
#  define GD32_NEFUSE              1              /* Electronic fuse */
#  define GD32_NCMP                1              /* Comparator */
#  define GD32_NVREF               1              /* Voltage reference */
#  define GD32_NTMU                1              /* Trigonometric math unit */
#  define GD32_NFAC                1              /* Filter and convolver accelerator */
#  define GD32_NCTC                1              /* Clock trim controller */
#  define GD32_NRSPDIF             1              /* Receiver S/PDIF */
#  define GD32_NMDIO               1              /* Management data I/O */
#  define GD32_NLPDTS              1              /* Low power temperature sensor */
#  define GD32_NEDOUT              1              /* Event detector and output */
#  define GD32_NRAMECCMU           2              /* RAM ECC monitor unit */

/* GD32H757 - 2048KB Flash */

#elif  defined(CONFIG_ARCH_CHIP_GD32H757VMT6)
#  define GD32_FLASH_SIZE          (2048*1024)   /* 2048KB */
#  define GD32_ITCM_SIZE           (64*1024)     /* 64KB ITCM */
#  define GD32_DTCM_SIZE           (128*1024)    /* 128KB DTCM */
#  define GD32_AXI_SRAM_SIZE       (512*1024)    /* 512KB AXI SRAM */
#  define GD32_SRAM1_SIZE          (128*1024)    /* 128KB SRAM1 */
#  define GD32_SRAM2_SIZE          (128*1024)    /* 128KB SRAM2 */
#  define GD32_SRAM3_SIZE          (32*1024)     /* 32KB SRAM3 */
#  define GD32_SRAM4_SIZE          (64*1024)     /* 64KB SRAM4 */
#  define GD32_BACKUP_SRAM_SIZE    (4*1024)      /* 4KB Backup SRAM */
#  define GD32_SRAM_SIZE           (GD32_DTCM_SIZE + GD32_AXI_SRAM_SIZE + \
                                    GD32_SRAM1_SIZE + GD32_SRAM2_SIZE + \
                                    GD32_SRAM3_SIZE + GD32_SRAM4_SIZE)
#  define GD32_NGPIO_PORTS         10             /* Ports A-K */
#  define GD32_NATIMER             4              /* Advanced timers: TIMER0,7,40,41 */
#  define GD32_NGTIMER             8              /* General timers: TIMER1-6, 22-23 */
#  define GD32_NBTIMER             11             /* Basic timers: TIMER14-16, 30-31, 42-44, 50-54 */
#  define GD32_NUSART              8              /* USART0-2,5, UART3-4,6-7 */
#  define GD32_NI2C                6              /* I2C0-5 */
#  define GD32_NSPI                7              /* SPI0-6 */
#  define GD32_NADC                3              /* ADC0-2 */
#  define GD32_NDAC                1              /* DAC */
#  define GD32_NCAN                7              /* CAN0-6 */
#  define GD32_NSDIO               2              /* SDIO0-1 */
#  define GD32_NSAI                4              /* SAI0-3 */
#  define GD32_NEXMC               1              /* External memory controller */
#  define GD32_NENET               2              /* ENET0-1 */
#  define GD32_NDMA                2              /* DMA0-1 */
#  define GD32_NMDMA               1              /* MDMA */
#  define GD32_NDMAMUX             2              /* DMAMUX0-1 */
#  define GD32_NUSBHS              1              /* USB HS OTG */
#  define GD32_NOSPI               2              /* OSPI0-1 */
#  define GD32_NTLI                1              /* TFT-LCD interface */
#  define GD32_NDCI                1              /* Digital camera interface */
#  define GD32_NIPA                1              /* Image processing accelerator */
#  define GD32_NHPDF               6              /* High-pass digital filter */
#  define GD32_NCAU                1              /* Cryptographic acceleration unit */
#  define GD32_NHAU                1              /* Hash acceleration unit */
#  define GD32_NTRNG               1              /* True random number generator */
#  define GD32_NCRC                1              /* CRC calculation unit */
#  define GD32_NRTC                1              /* Real-time clock */
#  define GD32_NWWDGT              1              /* Window watchdog */
#  define GD32_NFWDGT              1              /* Free watchdog */
#  define GD32_NHWSEM              2              /* Hardware semaphore */
#  define GD32_NEFUSE              1              /* Electronic fuse */
#  define GD32_NCMP                3              /* Comparator */
#  define GD32_NVREF               1              /* Voltage reference */
#  define GD32_NTMU                1              /* Trigonometric math unit */
#  define GD32_NFAC                1              /* Filter and convolver accelerator */
#  define GD32_NCTC                1              /* Clock trim controller */
#  define GD32_NRSPDIF             1              /* Receiver S/PDIF */
#  define GD32_NMDIO               1              /* Management data I/O */
#  define GD32_NSWPMI              1              /* Single wire protocol master interface */
#  define GD32_NLPDTS              1              /* Low power temperature sensor */
#  define GD32_NEDOUT              2              /* Event detector and output */
#  define GD32_NRAMECCMU           1              /* RAM ECC monitor unit */
#  define GD32_NCPDM               1              /* Clock and power detection module */

/* GD32H737 series - 1024KB Flash */

#elif defined(CONFIG_ARCH_CHIP_GD32H737V)
#  define GD32_FLASH_SIZE          (1024*1024)   /* 1024KB */
#  define GD32_ITCM_SIZE           (64*1024)     /* 64KB ITCM */
#  define GD32_DTCM_SIZE           (128*1024)    /* 128KB DTCM */
#  define GD32_AXI_SRAM_SIZE       (512*1024)    /* 512KB AXI SRAM */
#  define GD32_SRAM1_SIZE          (128*1024)    /* 128KB SRAM1 */
#  define GD32_SRAM2_SIZE          (128*1024)    /* 128KB SRAM2 */
#  define GD32_SRAM3_SIZE          (32*1024)     /* 32KB SRAM3 */
#  define GD32_SRAM4_SIZE          (64*1024)     /* 64KB SRAM4 */
#  define GD32_BACKUP_SRAM_SIZE    (4*1024)      /* 4KB Backup SRAM */
#  define GD32_SRAM_SIZE           (GD32_DTCM_SIZE + GD32_AXI_SRAM_SIZE + \
                                    GD32_SRAM1_SIZE + GD32_SRAM2_SIZE + \
                                    GD32_SRAM3_SIZE + GD32_SRAM4_SIZE)
#  define GD32_NGPIO_PORTS         10             /* Ports A-K */
#  define GD32_NATIMER             4              /* Advanced timers: TIMER0,7,40,41 */
#  define GD32_NGTIMER             8              /* General timers: TIMER1-6, 22-23 */
#  define GD32_NBTIMER             8              /* Basic timers: TIMER14-16, 30-31, 42-44 */
#  define GD32_NUSART              8              /* USART0-2,5, UART3-4,6-7 */
#  define GD32_NI2C                6              /* I2C0-5 */
#  define GD32_NSPI                7              /* SPI0-6 */
#  define GD32_NADC                3              /* ADC0-2 */
#  define GD32_NDAC                1              /* DAC */
#  define GD32_NCAN                7              /* CAN0-6 */
#  define GD32_NSDIO               2              /* SDIO0-1 */
#  define GD32_NSAI                4              /* SAI0-3 */
#  define GD32_NEXMC               1              /* External memory controller */
#  define GD32_NENET               2              /* ENET0-1 */
#  define GD32_NDMA                2              /* DMA0-1 */
#  define GD32_NMDMA               1              /* MDMA */
#  define GD32_NDMAMUX             2              /* DMAMUX0-1 */
#  define GD32_NUSBHS              1              /* USB HS OTG */
#  define GD32_NOSPI               2              /* OSPI0-1 */
#  define GD32_NTLI                1              /* TFT-LCD interface */
#  define GD32_NDCI                1              /* Digital camera interface */
#  define GD32_NIPA                1              /* Image processing accelerator */
#  define GD32_NHPDF               6              /* High-pass digital filter */
#  define GD32_NCAU                1              /* Cryptographic acceleration unit */
#  define GD32_NHAU                1              /* Hash acceleration unit */
#  define GD32_NTRNG               1              /* True random number generator */
#  define GD32_NCRC                1              /* CRC calculation unit */
#  define GD32_NRTC                1              /* Real-time clock */
#  define GD32_NWWDGT              1              /* Window watchdog */
#  define GD32_NFWDGT              1              /* Free watchdog */
#  define GD32_NHWSEM              2              /* Hardware semaphore */
#  define GD32_NEFUSE              1              /* Electronic fuse */
#  define GD32_NCMP                3              /* Comparator */
#  define GD32_NVREF               1              /* Voltage reference */
#  define GD32_NTMU                1              /* Trigonometric math unit */
#  define GD32_NFAC                1              /* Filter and convolver accelerator */
#  define GD32_NCTC                1              /* Clock trim controller */
#  define GD32_NRSPDIF             1              /* Receiver S/PDIF */
#  define GD32_NMDIO               1              /* Management data I/O */
#  define GD32_NSWPMI              1              /* Single wire protocol master interface */
#  define GD32_NLPDTS              1              /* Low power temperature sensor */
#  define GD32_NEDOUT              2              /* Event detector and output */
#  define GD32_NRAMECCMU           1              /* RAM ECC monitor unit */
#  define GD32_NCPDM               1              /* Clock and power detection module */

#else
#  error "Unsupported GD32H7 chip"
#endif

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_GD32H7_CHIP_H */