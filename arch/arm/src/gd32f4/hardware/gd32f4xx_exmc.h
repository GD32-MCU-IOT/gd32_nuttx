/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_exmc.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_EXMC_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_EXMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_EXMC_SNCTL_OFFSET(n)  (8 * ((n) - 1))
#define GD32_EXMC_SNCTL0_OFFSET    0x0000 /* SRAM/NOR-Flash chip-select control registers 1 */
#define GD32_EXMC_SNCTL1_OFFSET    0x0008 /* SRAM/NOR-Flash chip-select control registers 2 */
#define GD32_EXMC_SNCTL2_OFFSET    0x0010 /* SRAM/NOR-Flash chip-select control registers 3 */
#define GD32_EXMC_SNCTL3_OFFSET    0x0018 /* SRAM/NOR-Flash chip-select control registers 4 */

#define GD32_EXMC_SNTCFG_OFFSET(n)  (8 * ((n) - 1) + 0x0004)
#define GD32_EXMC_SNTCFG0_OFFSET    0x0004 /* SRAM/NOR-Flash chip-select timing registers 1 */
#define GD32_EXMC_SNTCFG1_OFFSET    0x000c /* SRAM/NOR-Flash chip-select timing registers 2 */
#define GD32_EXMC_SNTCFG2_OFFSET    0x0014 /* SRAM/NOR-Flash chip-select timing registers 3 */
#define GD32_EXMC_SNTCFG3_OFFSET    0x001c /* SRAM/NOR-Flash chip-select timing registers 4 */

#define GD32_EXMC_SNWTCFG_OFFSET(n) (8 * ((n) - 1) + 0x0104)
#define GD32_EXMC_SNWTCFG0_OFFSET   0x0104 /* SRAM/NOR-Flash write timing registers 1 */
#define GD32_EXMC_SNWTCFG1_OFFSET   0x010c /* SRAM/NOR-Flash write timing registers 2 */
#define GD32_EXMC_SNWTCFG2_OFFSET   0x0114 /* SRAM/NOR-Flash write timing registers 3 */
#define GD32_EXMC_SNWTCFG3_OFFSET   0x011c /* SRAM/NOR-Flash write timing registers 4 */

#define GD32_EXMC_NPCTL_OFFSET(n)  (0x0020 * ((n) - 1) + 0x0040)
#define GD32_EXMC_NPCTL1_OFFSET    0x0060 /* NAND Flash/PC Card controller register 2 */
#define GD32_EXMC_NPCTL2_OFFSET    0x0080 /* NAND Flash/PC Card controller register 3 */
#define GD32_EXMC_NPCTL3_OFFSET    0x00a0 /* NAND Flash/PC Card controller register 4 */

#define GD32_EXMC_NPINTEN_OFFSET(n)   (0x0020 * ((n) - 1) + 0x0044)
#define GD32_EXMC_NPINTEN1_OFFSET     0x0064 /* NAND Flash/PC Card controller register 2 */
#define GD32_EXMC_NPINTEN2_OFFSET     0x0084 /* NAND Flash/PC Card controller register 3 */
#define GD32_EXMC_NPINTEN3_OFFSET     0x00a4 /* NAND Flash/PC Card controller register 4 */

#define GD32_EXMC_NPCTCFG_OFFSET(n) (0x0020 * ((n) - 1) + 0x0048)
#define GD32_EXMC_NPCTCFG1_OFFSET   0x0068 /* Common memory space timing register 2 */
#define GD32_EXMC_NPCTCFG2_OFFSET   0x0088 /* Common memory space timing register 3 */
#define GD32_EXMC_NPCTCFG3_OFFSET   0x00a8 /* Common memory space timing register 4 */

#define GD32_EXMC_NPATCFG_OFFSET(n) (0x0020 * ((n) - 1) + 0x004c)
#define GD32_EXMC_NPATCFG1_OFFSET   0x006c /* Attribute memory space timing register 2 */
#define GD32_EXMC_NPATCFG2_OFFSET   0x008c /* Attribute memory space timing register 3 */
#define GD32_EXMC_NPATCFG3_OFFSET   0x00ac /* Attribute memory space timing register 4 */

#define GD32_EXMC_PIOTCFG3_OFFSET    0x00b0 /* I/O space timing register 4 */

#define GD32_EXMC_NECC_OFFSET(n) (0x0020 * ((n) - 1) + 0x0054)
#define GD32_EXMC_NECC1_OFFSET   0x0074 /* ECC result register 2 */
#define GD32_EXMC_NECC2_OFFSET   0x0094 /* ECC result register 3 */

#define GD32_EXMC_SDCTL0_OFFSET   0x0140 /* SDRAM Control Register, Bank 1 */
#define GD32_EXMC_SDCTL1_OFFSET   0x0144 /* SDRAM Control Register, Bank 2 */

#define GD32_EXMC_SDTCFG0_OFFSET   0x0148 /* SDRAM Timing Register, Bank 1 */
#define GD32_EXMC_SDTCFG1_OFFSET   0x014c /* SDRAM Timing Register, Bank 2 */

#define GD32_EXMC_SDCMD_OFFSET   0x0150  /* SDRAM Config Memory register */
#define GD32_EXMC_SDARI_OFFSET   0x0154  /* SDRAM Refresh Timing Register maybe */
#define GD32_EXMC_SDSTAT_OFFSET  0x0158  /* SDRAM Status Register */

/* Register Addresses *******************************************************/

#define GD32_EXMC_SNCTL(n)         (GD32_EXMC_REG_BASE + GD32_EXMC_SNCTL_OFFSET(n))
#define GD32_EXMC_SNCTL0           (GD32_EXMC_REG_BASE + GD32_EXMC_SNCTL0_OFFSET)
#define GD32_EXMC_SNCTL1           (GD32_EXMC_REG_BASE + GD32_EXMC_SNCTL1_OFFSET)
#define GD32_EXMC_SNCTL2           (GD32_EXMC_REG_BASE + GD32_EXMC_SNCTL2_OFFSET)
#define GD32_EXMC_SNCTL3           (GD32_EXMC_REG_BASE + GD32_EXMC_SNCTL3_OFFSET)

#define GD32_EXMC_SNTCFG(n)         (GD32_EXMC_REG_BASE + GD32_EXMC_SNTCFG_OFFSET(n))
#define GD32_EXMC_SNTCFG0           (GD32_EXMC_REG_BASE + GD32_EXMC_SNTCFG0_OFFSET)
#define GD32_EXMC_SNTCFG1           (GD32_EXMC_REG_BASE + GD32_EXMC_SNTCFG1_OFFSET)
#define GD32_EXMC_SNTCFG2           (GD32_EXMC_REG_BASE + GD32_EXMC_SNTCFG2_OFFSET)
#define GD32_EXMC_SNTCFG3           (GD32_EXMC_REG_BASE + GD32_EXMC_SNTCFG3_OFFSET)

#define GD32_EXMC_SNWTCFG(n)        (GD32_EXMC_REG_BASE + GD32_EXMC_SNWTCFG_OFFSET(n))
#define GD32_EXMC_SNWTCFG0          (GD32_EXMC_REG_BASE + GD32_EXMC_SNWTCFG0_OFFSET)
#define GD32_EXMC_SNWTCFG1          (GD32_EXMC_REG_BASE + GD32_EXMC_SNWTCFG1_OFFSET)
#define GD32_EXMC_SNWTCFG2          (GD32_EXMC_REG_BASE + GD32_EXMC_SNWTCFG2_OFFSET)
#define GD32_EXMC_SNWTCFG3          (GD32_EXMC_REG_BASE + GD32_EXMC_SNWTCFG3_OFFSET)

#define GD32_EXMC_NPCTL(n)         (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTL_OFFSET(n))
#define GD32_EXMC_NPCTL1           (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTL1_OFFSET)
#define GD32_EXMC_NPCTL2           (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTL2_OFFSET)
#define GD32_EXMC_NPCTL3           (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTL3_OFFSET)

#define GD32_EXMC_NPINTEN(n)          (GD32_EXMC_REG_BASE + GD32_EXMC_NPINTEN_OFFSET(n))
#define GD32_EXMC_NPINTEN1            (GD32_EXMC_REG_BASE + GD32_EXMC_NPINTEN1_OFFSET)
#define GD32_EXMC_NPINTEN2            (GD32_EXMC_REG_BASE + GD32_EXMC_NPINTEN2_OFFSET)
#define GD32_EXMC_NPINTEN3            (GD32_EXMC_REG_BASE + GD32_EXMC_NPINTEN3_OFFSET)

#define GD32_EXMC_NPCTCFG(n)        (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTCFG_OFFSET(n))
#define GD32_EXMC_NPCTCFG1          (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTCFG1_OFFSET)
#define GD32_EXMC_NPCTCFG2          (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTCFG2_OFFSET)
#define GD32_EXMC_NPCTCFG3          (GD32_EXMC_REG_BASE + GD32_EXMC_NPCTCFG3_OFFSET)

#define GD32_EXMC_NPATCFG(n)        (GD32_EXMC_REG_BASE + GD32_EXMC_NPATCFG_OFFSET(n))
#define GD32_EXMC_NPATCFG1          (GD32_EXMC_REG_BASE + GD32_EXMC_NPATCFG1_OFFSET)
#define GD32_EXMC_NPATCFG2          (GD32_EXMC_REG_BASE + GD32_EXMC_NPATCFG2_OFFSET)
#define GD32_EXMC_NPATCFG3          (GD32_EXMC_REG_BASE + GD32_EXMC_NPATCFG3_OFFSET)

#define GD32_EXMC_PIOTCFG3           (GD32_EXMC_REG_BASE + GD32_EXMC_PIOTCFG3_OFFSET)

#define GD32_EXMC_NECC(n)        (GD32_EXMC_REG_BASE + GD32_EXMC_NECC_OFFSET(n))
#define GD32_EXMC_NECC1          (GD32_EXMC_REG_BASE + GD32_EXMC_NECC1_OFFSET)
#define GD32_EXMC_NECC2          (GD32_EXMC_REG_BASE + GD32_EXMC_NECC2_OFFSET)

#define GD32_EXMC_SDCTL0          (GD32_EXMC_REG_BASE + GD32_EXMC_SDCTL0_OFFSET)
#define GD32_EXMC_SDCTL1          (GD32_EXMC_REG_BASE + GD32_EXMC_SDCTL1_OFFSET)

#define GD32_EXMC_SDTCFG0          (GD32_EXMC_REG_BASE + GD32_EXMC_SDTCFG0_OFFSET)
#define GD32_EXMC_SDTCFG1         (GD32_EXMC_REG_BASE + GD32_EXMC_SDTCFG1_OFFSET)

#define GD32_EXMC_SDCMD          (GD32_EXMC_REG_BASE + GD32_EXMC_SDCMD_OFFSET)
#define GD32_EXMC_SDARI          (GD32_EXMC_REG_BASE + GD32_EXMC_SDARI_OFFSET)
#define GD32_EXMC_SDSTAT           (GD32_EXMC_REG_BASE + GD32_EXMC_SDSTAT_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define EXMC_SNCTL_NRBKEN           (1 << 0)  /* Memory bank enable bit */
#define EXMC_SNCTL_NRMUX            (1 << 1)  /* Address/data multiplexing enable bit */
#define EXMC_SNCTL_NRTP_SHIFT       (2)       /* Memory type */
#define EXMC_SNCTL_NRTP_MASK        (3 << EXMC_SNCTL_NRTP_SHIFT)
#  define EXMC_MEMORY_TYPE_SRAM           (0 << EXMC_SNCTL_NRTP_SHIFT)
#  define EXMC_MEMORY_TYPE_ROM            (0 << EXMC_SNCTL_NRTP_SHIFT)
#  define EXMC_MEMORY_TYPE_PSRAM          (1 << EXMC_SNCTL_NRTP_SHIFT)
#  define EXMC_MEMORY_TYPE_CRAM           (1 << EXMC_SNCTL_NRTP_SHIFT)
#  define EXMC_MEMORY_TYPE_NOR            (2 << EXMC_SNCTL_NRTP_SHIFT)
#define EXMC_SNCTL_NRW_SHIFT        (4)       /* Memory data bus width */
#define EXMC_SNCTL_NRW_MASK         (3 << EXMC_SNCTL_NRW_SHIFT)
#  define EXMC_SNCTL_NAND_DATABUS_WIDTH_8B          (0 << EXMC_SNCTL_NRW_SHIFT)
#  define EXMC_SNCTL_NAND_DATABUS_WIDTH_16B         (1 << EXMC_SNCTL_NRW_SHIFT)
#define EXMC_SNCTL_NREN             (1 << 6)  /* Flash access enable */
#define EXMC_SNCTL_SBRSTEN          (1 << 8)  /* Burst enable bit */
#define EXMC_SNCTL_NRWTPOL          (1 << 9)  /* Wait signal polarity bit */
#define EXMC_SNCTL_WRAPEN           (1 << 10) /* Wrapped burst mode support */
#define EXMC_SNCTL_NRWTCFG          (1 << 11) /* Wait timing configuration */
#define EXMC_SNCTL_WEN              (1 << 12) /* Write enable bit */
#define EXMC_SNCTL_NRWTEN           (1 << 13) /* Wait enable bit */
#define EXMC_SNCTL_EXMODEN          (1 << 14) /* Extended mode enable */
#define EXMC_SNCTL_ASYNCWTEN        (1 << 15) /* Wait signal during async */

/* TODO 16-18 */

#define EXMC_SNCTL_SYNCWR         (1 << 19)  /* Write burst enable */

#define FMC_BCR_RSTVALUE         0x000003d2

#define EXMC_SNTCFG_ASET_SHIFT     (0)        /* Address setup phase duration */
#define EXMC_SNTCFG_ASET_MASK      (15 << EXMC_SNTCFG_ASET_SHIFT)
#  define EXMC_SNTCFG_ASET(n)      ((n-1) << EXMC_SNTCFG_ASET_SHIFT)  /* (n)xHCLK n=1..16 */

#define EXMC_SNTCFG_AHLD_SHIFT     (4)        /* Address-hold phase duration */
#define EXMC_SNTCFG_AHLD_MASK      (15 << EXMC_SNTCFG_AHLD_SHIFT)
#  define EXMC_SNTCFG_AHLD(n)      ((n-1) << EXMC_SNTCFG_AHLD_SHIFT)  /* (n)xHCLK n=2..16*/

#define EXMC_SNTCFG_DSET_SHIFT     (8)        /* Data-phase duration */
#define EXMC_SNTCFG_DSET_MASK      (255 << EXMC_SNTCFG_DSET_SHIFT)
#  define EXMC_SNTCFG_DSET(n)      ((n-1) << EXMC_SNTCFG_DSET_SHIFT)  /* (n)xHCLK n=2..256 */

#define EXMC_SNTCFG_BUSLAT_SHIFT    (16)       /* Bus turnaround phase duration */
#define EXMC_SNTCFG_BUSLAT_MASK     (15 << EXMC_SNTCFG_BUSLAT_SHIFT)
#  define EXMC_SNTCFG_BUSLAT(n)     ((n-1) << EXMC_SNTCFG_BUSLAT_SHIFT) /* (n)xHCLK n=1..16 */

#define EXMC_SNTCFG_CKDIV_SHIFT     (20)       /* Clock divide ratio */
#define EXMC_SNTCFG_CKDIV_MASK      (15 << EXMC_SNTCFG_CKDIV_SHIFT)
#  define EXMC_SNTCFG_CKDIV(n)      ((n-1) << EXMC_SNTCFG_CKDIV_SHIFT)  /* (n)xHCLK n=2..16 */

#define EXMC_SNTCFG_DLAT_SHIFT     (24)      /* Data latency */
#define EXMC_SNTCFG_DLAT_MASK      (15 << EXMC_SNTCFG_DLAT_SHIFT)
#  define EXMC_SNTCFG_DLAT(n)      ((n-2) << EXMC_SNTCFG_DLAT_SHIFT)  /* (n)xHCLK n=2..17 */

#define EXMC_SNTCFG_ASYNCMOD_SHIFT     (28)      /* Access mode */
#define EXMC_SNTCFG_ASYNCMOD_MASK      (3 << EXMC_SNTCFG_ASYNCMOD_SHIFT)
#  define EXMC_SNTCFG_MODE_A        (0 << EXMC_SNTCFG_ASYNCMOD_SHIFT)
#  define EXMC_SNTCFG_MODE_B        (1 << EXMC_SNTCFG_ASYNCMOD_SHIFT)
#  define EXMC_SNTCFG_MODE_C        (2 << EXMC_SNTCFG_ASYNCMOD_SHIFT)
#  define EXMC_SNTCFG_MODE_D        (3 << EXMC_SNTCFG_ASYNCMOD_SHIFT)

#define EXMC_SNTCFG_RSTVALUE         0xffffffff

#define EXMC_SNWTCFG_WASET_SHIFT    (0)        /* Address setup phase duration */
#define EXMC_SNWTCFG_WASET_MASK     (15 << EXMC_SNWTCFG_WASET_SHIFT)
#  define EXMC_SNWTCFG_WASET(n)     ((n-1) << EXMC_SNWTCFG_WASET_SHIFT)  /* (n)xHCLK n=1..16 */

#define EXMC_SNWTCFG_WAHLD_SHIFT    (4)        /* Address-hold phase duration */
#define EXMC_SNWTCFG_WAHLD_MASK     (15 << EXMC_SNWTCFG_WAHLD_SHIFT)
#  define EXMC_SNWTCFG_WAHLD(n)     ((n-1) << EXMC_SNWTCFG_WAHLD_SHIFT)  /* (n)xHCLK n=2..16*/

#define EXMC_SNWTCFG_WDSET_SHIFT    (8)        /* Data-phase duration */
#define EXMC_SNWTCFG_WDSET_MASK     (255 << EXMC_SNWTCFG_WDSET_SHIFT)
#  define EXMC_SNWTCFG_WDSET(n)     ((n-1) << EXMC_SNWTCFG_WDSET_SHIFT)  /* (n)xHCLK n=2..256 */

/* #define FMC_BWTR_CLKDIV_SHIFT    (20)        Clock divide ratio
 * #define FMC_BWTR_CLKDIV_MASK     (15 << FMC_BWTR_CLKDIV_SHIFT)
 * #  define FMC_BWTR_CLKDIV(n)     ((n - 1) << FMC_BWTR_CLKDIV_SHIFT)
 *
 * #define FMC_BWTR_DATLAT_SHIFT    (24)       Data latency
 * #define FMC_BWTR_DATLAT_MASK     (15 << FMC_BWTR_DATLAT_SHIFT)
 * #  define FMC_BWTR_DATLAT(n)     ((n - 2) << FMC_BWTR_DATLAT_SHIFT)
 */

#define EXMC_SNWTCFG_WASYNCMOD_SHIFT    (28)      /* Access mode */
#define EXMC_SNWTCFG_WASYNCMOD_MASK     (3 << EXMC_SNWTCFG_WASYNCMOD_SHIFT)
#  define EXMC_SNWTCFG_MODE_A       (0 << EXMC_SNWTCFG_WASYNCMOD_SHIFT)
#  define EXMC_SNWTCFG_MODE_B       (1 << EXMC_SNWTCFG_WASYNCMOD_SHIFT) /* TODO */
#  define EXMC_SNWTCFG_MODE_C       (2 << EXMC_SNWTCFG_WASYNCMOD_SHIFT)
#  define EXMC_SNWTCFG_MODE_D       (3 << EXMC_SNWTCFG_WASYNCMOD_SHIFT)

#define EXMC_NPCTL_NDWTEN          (1 << 1)  /* Wait feature enable bit */
#define EXMC_NPCTL_NDBKEN          (1 << 2)  /* PC Card/NAND Flash memory bank enable bit */
#define EXMC_NPCTL_NDTP            (1 << 3)  /* Memory type */
#define EXMC_NPCTL_NDW_SHIFT       (4)       /* NAND Flash databus width */
#define EXMC_NPCTL_NDW_MASK        (3 <<  EXMC_NPCTL_NDW_SHIFT)
#  define EXMC_NPCTL_NAND_DATABUS_WIDTH_8B          (0 <<  EXMC_NPCTL_NDW_SHIFT)
#  define EXMC_NPCTL_NAND_DATABUS_WIDTH_16B         (1 <<  EXMC_NPCTL_NDW_SHIFT)
#define EXMC_NPCTL_ECCEN           (1 << 6)  /* ECC computation logic enable bit */
#define EXMC_NPCTL_CTR_SHIFT       (9)       /* CLE to RE delay */
#define EXMC_NPCTL_CTR_MASK        (15 << EXMC_NPCTL_CTR_SHIFT)
#  define EXMC_NPCTL_CTR(n)        ((n-1) << EXMC_NPCTL_CTR_SHIFT) /* (n)xHCLK n=1..16 */

#define EXMC_NPCTL_ATR_SHIFT        (13)      /* ALE to RE delay */
#define EXMC_NPCTL_ATR_MASK         (15 << EXMC_NPCTL_ATR_SHIFT)
#  define EXMC_NPCTL_ATR(n)         ((n-1) << EXMC_NPCTL_ATR_SHIFT)  /* (n)xHCLK n=1..16 */

#define EXMC_NPCTL_ECCSZ_SHIFT      (17)      /* ECC page size */
#define EXMC_NPCTL_ECCSZ_MASK       (7 << EXMC_NPCTL_ECCSZ_SHIFT)
#  define EXMC_ECC_SIZE_256BYTES       (0 << EXMC_NPCTL_ECCSZ_SHIFT) /* 256 bytes */
#  define EXMC_ECC_SIZE_512BYTES       (1 << EXMC_NPCTL_ECCSZ_SHIFT) /* 512 bytes */
#  define EXMC_ECC_SIZE_1024BYTES      (2 << EXMC_NPCTL_ECCSZ_SHIFT) /* 1024 bytes */
#  define EXMC_ECC_SIZE_2048BYTES      (3 << EXMC_NPCTL_ECCSZ_SHIFT) /* 2048 bytes */
#  define EXMC_ECC_SIZE_4096BYTES      (4 << EXMC_NPCTL_ECCSZ_SHIFT) /* 4096 bytes */
#  define EXMC_ECC_SIZE_8192BYTES      (5 << EXMC_NPCTL_ECCSZ_SHIFT) /* 8192 bytes */

#define EXMC_NPINTEN_INTRS          (1 << 0)  /* Interrupt Rising Edge status */
#define EXMC_NPINTEN_INTHS          (1 << 1)  /* Interrupt Level status */
#define EXMC_NPINTEN_INTFS          (1 << 2)  /* Interrupt Falling Edge status */
#define EXMC_NPINTEN_INTREN         (1 << 3)  /* Interrupt Rising Edge Enable */
#define EXMC_NPINTEN_INTHEN         (1 << 4)  /* Interrupt Level Enable bit */
#define EXMC_NPINTEN_INTFEN         (1 << 5)  /* Interrupt Falling Edge Enable */
#define EXMC_NPINTEN_FFEPT          (1 << 6)  /* FIFO empty */

#define EXMC_NPCTCFG_COMSET_SHIFT    (0)       /* Common memory setup time */
#define EXMC_NPCTCFG_COMSET_MASK     (255 << EXMC_NPCTCFG_COMSET_SHIFT)
#  define EXMC_NPCTCFG_COMSET(n)     ((n-1) << EXMC_NPCTCFG_COMSET_SHIFT) /* (n)xHCLK n=1..256 */

#define EXMC_NPCTCFG_COMWAIT_SHIFT   (8)       /* Common memory wait time */
#define EXMC_NPCTCFG_COMWAIT_MASK    (255 << EXMC_NPCTCFG_COMWAIT_SHIFT)
#  define EXMC_NPCTCFG_COMWAIT(n)    ((n-1) << EXMC_NPCTCFG_COMWAIT_SHIFT) /* (n)xHCLK n=2..256 */

#define EXMC_NPCTCFG_COMHLD_SHIFT   (16)      /* Common memoryhold time */
#define EXMC_NPCTCFG_COMHLD_MASK    (255 << EXMC_NPCTCFG_COMHLD_SHIFT)
#  define EXMC_NPCTCFG_COMHLD(n)    ((n) <<  EXMC_NPCTCFG_COMHLD_SHIFT) /* (n)xHCLK n=1..255 */

#define EXMC_NPCTCFG_COMHIZ_SHIFT    (24)       /* Common memory databus HiZ time */
#define EXMC_NPCTCFG_COMHIZ_MASK     (255 << EXMC_NPCTCFG_COMHIZ_SHIFT)
#  define EXMC_NPCTCFG_COMHIZ(n)     ((n) << EXMC_NPCTCFG_COMHIZ_SHIFT) /* (n)xHCLK n=0..255 */

#define EXMC_NPATCFG_ATTSET_SHIFT    (0)       /* Attribute memory setup time */
#define EXMC_NPATCFG_ATTSET_MASK     (255 << EXMC_NPATCFG_ATTSET_SHIFT)
#  define EXMC_NPATCFG_ATTSET(n)     ((n-1) << EXMC_NPATCFG_ATTSET_SHIFT) /* (n)xHCLK n=1..256 */

#define EXMC_NPATCFG_ATTWAIT_SHIFT   (8)       /* Attribute memory wait time */
#define EXMC_NPATCFG_ATTWAIT_MASK    (255 << EXMC_NPATCFG_ATTWAIT_SHIFT)
#  define EXMC_NPATCFG_ATTWAIT(n)    ((n-1) << EXMC_NPATCFG_ATTWAIT_SHIFT) /* (n)xHCLK n=2..256 */

#define EXMC_NPATCFG_ATTHLD_SHIFT   (16)      /* Attribute memory hold time */
#define EXMC_NPATCFG_ATTHLD_MASK    (255 << EXMC_NPATCFG_ATTHLD_SHIFT)
#  define EXMC_NPATCFG_ATTHLD(n)    ((n) <<  EXMC_NPATCFG_ATTHLD_SHIFT) /* (n)xHCLK n=1..255 */

#define EXMC_NPATCFG_ATTHIZ_SHIFT    (24)       /* Attribute memory databus HiZ time */
#define EXMC_NPATCFG_ATTHIZ_MASK     (255 << EXMC_NPATCFG_ATTHIZ_SHIFT)
#  define EXMC_NPATCFG_ATTHIZ(n)     ((n) << EXMC_NPATCFG_ATTHIZ_SHIFT) /* (n)xHCLK n=0..255 */

#define EXMC_PIOTCFG3_IOSET_SHIFT     (0)       /* IO memory setup time */
#define EXMC_PIOTCFG3_IOSET_MASK      (255 << EXMC_PIOTCFG3_IOSET_SHIFT)
#  define EXMC_PIOTCFG3_IOSET(n)      ((n-1) << EXMC_PIOTCFG3_IOSET_SHIFT) /* (n)xHCLK n=1..256 */

#define EXMC_PIOTCFG3_IOWAIT_SHIFT    (8)       /* IO memory wait time */
#define EXMC_PIOTCFG3_IOWAIT_MASK     (255 << EXMC_PIOTCFG3_IOWAIT_SHIFT)
#  define EXMC_PIOTCFG3_IOWAIT(n)     ((n-1) << EXMC_PIOTCFG3_IOWAIT_SHIFT) /* (n)xHCLK n=2..256 */

#define EXMC_PIOTCFG3_IOHLD_SHIFT    (16)      /* IO memory hold time */
#define EXMC_PIOTCFG3_IOHLD_MASK     (255 << EXMC_PIOTCFG3_IOHLD_SHIFT)
#  define EXMC_PIOTCFG3_IOHLD(n)     ((n) <<  EXMC_PIOTCFG3_IOHLD_SHIFT) /* (n)xHCLK n=1..255 */

#define EXMC_PIOTCFG3_IOHIZ_SHIFT     (24)      /* IO memory databus HiZ time */
#define EXMC_PIOTCFG3_IOHIZ_MASK      (255 << EXMC_PIOTCFG3_IOHIZ_SHIFT)
#  define EXMC_PIOTCFG3_IOHIZ(n)      ((n) << EXMC_PIOTCFG3_IOHIZ_SHIFT) /* (n)xHCLK n=0..255 */

#define EXMC_SDCTL_RESERVED        (0x1ffff << 15)  /* reserved bits */

#define EXMC_PIPELINE_DELAY_0_HCLK         (0 << 13) /* read pipe */
#define EXMC_PIPELINE_DELAY_1_HCLK         (1 << 13)
#define EXMC_PIPELINE_DELAY_2_HCLK         (2 << 13)
#define EXMC_SDCTL_BRSTRD           (1 << 12) /* read burst */
#define EXMC_SDCLK_DISABLE          (0 << 10) /* sdram clock */
#define EXMC_SDCLK_PERIODS_2_HCLK        (2 << 10)
#define EXMC_SDCLK_PERIODS_3_HCLK        (3 << 10)
#define EXMC_SDCTL_WPEN             (1 << 9)  /* write protect */
#define EXMC_CAS_LATENCY_1_SDCLK    (1 << 7)  /* cas latency */
#define EXMC_CAS_LATENCY_2_SDCLK   (2 << 7)
#define EXMC_CAS_LATENCY_3_SDCLK   (3 << 7)
#define EXMC_SDCTL_NBK_2        (0 << 6) /* number of internal banks */
#define EXMC_SDCTL_NBK_4        (1 << 6)
#define EXMC_SDRAM_DATABUS_WIDTH_8B         (0 << 4) /* memory width */
#define EXMC_SDRAM_DATABUS_WIDTH_16B        (1 << 4)
#define EXMC_SDRAM_DATABUS_WIDTH_32B        (2 << 4)
#define EXMC_SDRAM_ROW_ADDRESS_11         (0 << 2) /* number of rows */
#define EXMC_SDRAM_ROW_ADDRESS_12         (1 << 2)
#define EXMC_SDRAM_ROW_ADDRESS_13         (2 << 2)
#define EXMC_SDRAM_COL_ADDRESS_8          (0 << 0) /* number of columns */
#define EXMC_SDRAM_COL_ADDRESS_9          (1 << 0)
#define EXMC_SDRAM_COL_ADDRESS_10         (2 << 0)
#define EXMC_SDRAM_COL_ADDRESS_11         (3 << 0)

#define EXMC_SDTCFG_RESERVED        (15 << 28)  /* reserved bits */
#define EXMC_SDTCFG_LMRD(n)         (((n & 15) - 1) << 0)
#define EXMC_SDTCFG_XSRD(n)         (((n & 15) - 1) << 4)
#define EXMC_SDTCFG_RASD(n)         (((n & 15) - 1) << 8)
#define EXMC_SDTCFG_ARFD(n)         (((n & 15) - 1) << 12)
#define EXMC_SDTCFG_WRD(n)          (((n & 15) - 1) << 16)
#define EXMC_SDTCFG_RPD(n)          (((n & 15) - 1) << 20)
#define EXMC_SDTCFG_RCD(n)          (((n & 15) - 1) << 24)

/* Note: The FMC_SDCMR_MDR_x values can be found in the SDRAM datasheet.
 * They should be standard, but it's probably a good idea to review
 * the datasheet for your SDRAM device.
 */
#define EXMC_SDCMD_RESERVED                     (0x3ff << 22)  /* reserved bits */
#define EXMC_SDCMR_MDR_BURST_LENGTH_1              ((0 << 0) << 9)
#define EXMC_SDCMR_MDR_BURST_LENGTH_2              ((1 << 0) << 9)//TODO
#define EXMC_SDCMR_MDR_BURST_LENGTH_4              ((2 << 0) << 9)
#define EXMC_SDCMR_MDR_BURST_LENGTH_8              ((3 << 0) << 9)
#define EXMC_SDCMR_MDR_BURST_LENGTH_FULL           ((7 << 0) << 9)
#define EXMC_SDCMR_MDR_BURST_TYPE_SEQUENTIAL       ((0 << 3) << 9)//TODO
#define EXMC_SDCMR_MDR_BURST_TYPE_INTERLEAVE       ((1 << 3) << 9)
#define EXMC_SDCMR_MDR_CAS_LATENCY_1               ((1 << 4) << 9)//TODO
#define EXMC_SDCMR_MDR_CAS_LATENCY_2               ((2 << 4) << 9)
#define EXMC_SDCMR_MDR_CAS_LATENCY_3               ((3 << 4) << 9)
#define EXMC_SDCMR_MDR_MODE_NORMAL                 ((0 << 7) << 9)
#define EXMC_SDCMR_MDR_WBL_BURST                   ((0 << 9) << 9)
#define EXMC_SDCMR_MDR_WBL_SINGLE                  ((1 << 9) << 9)//TODO
#define EXMC_SDCMD_NARF(n)                         (((n & 15) - 1) << 5)
#define EXMC_SDCMD_DS0                         (1 << 4)
#define EXMC_SDCMD_DS1                         (1 << 3)
#define EXMC_SDRAM_NORMAL_OPERATION            (0 << 0)
#define EXMC_SDRAM_CLOCK_ENABLE                (1 << 0)
#define EXMC_SDRAM_PRECHARGE_ALL               (2 << 0)
#define EXMC_SDRAM_AUTO_REFRESH                (3 << 0)
#define EXMC_SDRAM_LOAD_MODE_REGISTER          (4 << 0)
#define EXMC_SDRAM_SELF_REFRESH                (5 << 0)
#define EXMC_SDRAM_POWERDOWN_ENTRY             (6 << 0)

#define EXMC_SDSDAT_REIF                     (1 << 0)
#define EXMC_SDSDAT_NRDY                     (1 << 5)
#define EXMC_SDRAM_DEVICE1_NORMAL            (0 << 1)
#define EXMC_SDRAM_DEVICE1_SELF_REFRESH      (1 << 1)
#define EXMC_SDRAM_DEVICE1_POWER_DOWN        (2 << 1)
#define EXMC_SDRAM_DEVICE2_NORMAL            (0 << 3)
#define EXMC_SDRAM_DEVICE2_SELF_REFRESH      (1 << 3)
#define EXMC_SDRAM_DEVICE2_POWER_DOWN        (2 << 3)

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_EXMC_H */
