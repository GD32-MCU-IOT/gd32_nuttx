/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_tli.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_TLI_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_TLI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/gd32f4xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GD32_TLI_NCLUT            256    /* Number of entries in the CLUTs */

/* LCDC Register Offsets ****************************************************/

#define GD32_TLI_SPSZ_OFFSET     0x0008  /* LTDC Sync Size Config Reg */
#define GD32_TLI_BPSZ_OFFSET     0x000c  /* LTDC Back Porch Config Reg */
#define GD32_TLI_ASZ_OFFSET      0x0010  /* LTDC Active Width Config Reg */
#define GD32_TLI_TSZ_OFFSET      0x0014  /* LTDC Total Width Config Reg */
#define GD32_TLI_CTL_OFFSET      0x0018  /* LTDC Global Control Register */
#define GD32_TLI_RL_OFFSET       0x0024  /* LTDC Shadow Reload Config Reg */
#define GD32_TLI_BGC_OFFSET      0x002c  /* LTDC Background Color Config */
#define GD32_TLI_INTEN_OFFSET    0x0034  /* LTDC Interrupt Enable Register */
#define GD32_TLI_INTF_OFFSET     0x0038  /* LTDC Interrupt Status Register */
#define GD32_TLI_INTC_OFFSET     0x003c  /* LTDC Interrupt Clear Register */
#define GD32_TLI_LM_OFFSET       0x0040  /* LTDC Line Int Position Config */
#define GD32_TLI_CPPOS_OFFSET    0x0044  /* LTDC Current Position Status */
#define GD32_TLI_STAT_OFFSET     0x0048  /* LTDC Current Display Status */
#define GD32_TLI_L0CTL_OFFSET    0x0084  /* LTDC Layer 1 Control Register */
#define GD32_TLI_L0HPOS_OFFSET   0x0088  /* LTDC L1 Window Horiz Pos Config */
#define GD32_TLI_L0VPOS_OFFSET   0x008c  /* LTDC L1 Window Vert Pos Config */
#define GD32_TLI_L0CKEY_OFFSET   0x0090  /* LTDC L1 Color Keying Config */
#define GD32_TLI_L0PPF_OFFSET    0x0094  /* LTDC L1 Pixel Format Config */
#define GD32_TLI_L0SA_OFFSET     0x0098  /* LTDC L1 Constant Alpha Config */
#define GD32_TLI_L0DC_OFFSET     0x009c  /* LTDC L1 Default Color Config */
#define GD32_TLI_L0BLEND_OFFSET  0x00a0  /* LTDC L1 Blending Factors Config */
#define GD32_TLI_L0FBADDR_OFFSET 0x00ac  /* LTDC L1 Color Frame Buffer Addr */
#define GD32_TLI_L0FLLEN_OFFSET  0x00b0  /* LTDC L1 Color Frame Buffer Len */
#define GD32_TLI_L0FTLN_OFFSET   0x00b4  /* LTDC L1 Color Frame Buffer Line */
#define GD32_TLI_L0LUT_OFFSET    0x00c4  /* LTDC Layer 1 CLUT Write Reg */
#define GD32_TLI_L1CTL_OFFSET    0x0104  /* LTDC Layer 2 Control Register */
#define GD32_TLI_L1HPOS_OFFSET   0x0108  /* LTDC L2 Window Horiz Pos Config */
#define GD32_TLI_L1VPOS_OFFSET   0x010c  /* LTDC L2 Window Vert Pos Config */
#define GD32_TLI_L1CKEY_OFFSET   0x0110  /* LTDC L2 Color Keying Config */
#define GD32_TLI_L1PPF_OFFSET    0x0114  /* LTDC L2 Pixel Format Config */
#define GD32_TLI_L1SA_OFFSET     0x0118  /* LTDC L2 Constant Alpha Config */
#define GD32_TLI_L1DC_OFFSET     0x011c  /* LTDC L2 Default Color Config */
#define GD32_TLI_L1BLEND_OFFSET  0x0120  /* LTDC L2 Blending Factors Config */
#define GD32_TLI_L1FBADDR_OFFSET 0x012c  /* LTDC L2 Color Frame Buffer Addr */
#define GD32_TLI_L1FLLEN_OFFSET  0x0130  /* LTDC L2 Color Frame Buffer Len */
#define GD32_TLI_L1FTLN_OFFSET   0x0134  /* LTDC L2 Color Frame Buffer Line */
#define GD32_TLI_L1LUT_OFFSET    0x0144  /* LTDC Layer 2 CLUT Write Reg */

/* LTDC Register Addresses **************************************************/

#define GD32_TLI_SPSZ              (GD32_TLI_BASE + GD32_TLI_SPSZ_OFFSET)
#define GD32_TLI_BPSZ              (GD32_TLI_BASE + GD32_TLI_BPSZ_OFFSET)
#define GD32_TLI_ASZ               (GD32_TLI_BASE + GD32_TLI_ASZ_OFFSET)
#define GD32_TLI_TSZ               (GD32_TLI_BASE + GD32_TLI_TSZ_OFFSET)
#define GD32_TLI_CTL               (GD32_TLI_BASE + GD32_TLI_CTL_OFFSET)
#define GD32_TLI_RL                (GD32_TLI_BASE + GD32_TLI_RL_OFFSET)
#define GD32_TLI_BGC               (GD32_TLI_BASE + GD32_TLI_BGC_OFFSET)
#define GD32_TLI_INTEN             (GD32_TLI_BASE + GD32_TLI_INTEN_OFFSET)
#define GD32_TLI_INTF              (GD32_TLI_BASE + GD32_TLI_INTF_OFFSET)
#define GD32_TLI_INTC              (GD32_TLI_BASE + GD32_TLI_INTC_OFFSET)
#define GD32_TLI_LM                (GD32_TLI_BASE + GD32_TLI_LM_OFFSET)
#define GD32_TLI_CPPOS             (GD32_TLI_BASE + GD32_TLI_CPPOS_OFFSET)
#define GD32_TLI_STAT              (GD32_TLI_BASE + GD32_TLI_STAT_OFFSET)

#define GD32_TLI_L0CTL             (GD32_TLI_BASE + GD32_TLI_L0CTL_OFFSET)
#define GD32_TLI_L0HPOS            (GD32_TLI_BASE + GD32_TLI_L0HPOS_OFFSET)
#define GD32_TLI_L0VPOS            (GD32_TLI_BASE + GD32_TLI_L0VPOS_OFFSET)
#define GD32_TLI_L0CKEY            (GD32_TLI_BASE + GD32_TLI_L0CKEY_OFFSET)
#define GD32_TLI_L0PPF             (GD32_TLI_BASE + GD32_TLI_L0PPF_OFFSET)
#define GD32_TLI_L0SA              (GD32_TLI_BASE + GD32_TLI_L0SA_OFFSET)
#define GD32_TLI_L0DC              (GD32_TLI_BASE + GD32_TLI_L0DC_OFFSET)
#define GD32_TLI_L0BLEND           (GD32_TLI_BASE + GD32_TLI_L0BLEND_OFFSET)
#define GD32_TLI_L0FBADDR          (GD32_TLI_BASE + GD32_TLI_L0FBADDR_OFFSET)
#define GD32_TLI_L0FLLEN           (GD32_TLI_BASE + GD32_TLI_L0FLLEN_OFFSET)
#define GD32_TLI_L0FTLN            (GD32_TLI_BASE + GD32_TLI_L0FTLN_OFFSET)
#define GD32_TLI_L0LUT             (GD32_TLI_BASE + GD32_TLI_L0LUT_OFFSET)

#define GD32_TLI_L1CTL             (GD32_TLI_BASE + GD32_TLI_L1CTL_OFFSET)
#define GD32_TLI_L1HPOS            (GD32_TLI_BASE + GD32_TLI_L1HPOS_OFFSET)
#define GD32_TLI_L1VPOS            (GD32_TLI_BASE + GD32_TLI_L1VPOS_OFFSET)
#define GD32_TLI_L1CKEY            (GD32_TLI_BASE + GD32_TLI_L1CKEY_OFFSET)
#define GD32_TLI_L1PPF             (GD32_TLI_BASE + GD32_TLI_L1PPF_OFFSET)
#define GD32_TLI_L1SA              (GD32_TLI_BASE + GD32_TLI_L1SA_OFFSET)
#define GD32_TLI_L1DC              (GD32_TLI_BASE + GD32_TLI_L1DC_OFFSET)
#define GD32_TLI_L1BLEND           (GD32_TLI_BASE + GD32_TLI_L1BLEND_OFFSET)
#define GD32_TLI_L1FBADDR          (GD32_TLI_BASE + GD32_TLI_L1FBADDR_OFFSET)
#define GD32_TLI_L1FLLEN           (GD32_TLI_BASE + GD32_TLI_L1FLLEN_OFFSET)
#define GD32_TLI_L1FTLN            (GD32_TLI_BASE + GD32_TLI_L1FTLN_OFFSET)
#define GD32_TLI_L1LUT             (GD32_TLI_BASE + GD32_TLI_L1LUT_OFFSET)

/* LTDC Register Bit Definitions ********************************************/

/* LTDC Synchronization Size Configuration Register */

#define TLI_SPSZ_VPSZ_SHIFT         (0)       /* Bits 0-10: Vertical Sync Height (scan lines) */
#define TLI_SPSZ_VPSZ_MASK          (0x7ff << TLI_SPSZ_VPSZ_SHIFT)
#  define TLI_SPSZ_VPSZ(n)          ((uint32_t)(n) << TLI_SPSZ_VPSZ_SHIFT)
#define TLI_SPSZ_HPSZ_SHIFT         (16)      /* Bits 16-27: Horizontal Sync Width (pixel clocks) */
#define TLI_SPSZ_HPSZ_MASK          (0xfff << TLI_SPSZ_HPSZ_SHIFT)
#  define TLI_SPSZ_HPSZ(n)          ((uint32_t)(n) << TLI_SPSZ_HPSZ_SHIFT)

/* LTDC Back Porch Configuration Register */

#define TLI_BPSZ_VBPSZ_SHIFT        (0)       /* Bits 0-10: Accumulated Vertical back porch (scan lines) */
#define TLI_BPSZ_VBPSZ_MASK         (0x7ff << TLI_BPSZ_VBPSZ_SHIFT)
#  define TLI_BPSZ_VBPSZ(n)         ((uint32_t)(n) << TLI_BPSZ_VBPSZ_SHIFT)
#define TLI_BPSZ_HBPSZ_SHIFT        (16)      /* Bits 16-27: Accumulated Horizontal back porch (pixel clocks) */
#define TLI_BPSZ_HBPSZ_MASK         (0xfff << TLI_BPSZ_HBPSZ_SHIFT)
#  define TLI_BPSZ_HBPSZ(n)         ((uint32_t)(n) << TLI_BPSZ_HBPSZ_SHIFT)

/* LTDC Active Width Configuration Register */

#define TLI_ASZ_VASZ_SHIFT         (0)       /* Bits 0-10: Accumulated Active Height (scan lines) */
#define TLI_ASZ_VASZ_MASK          (0x7ff << TLI_ASZ_VASZ_SHIFT)
#  define TLI_ASZ_VASZ(n)          ((uint32_t)(n) << TLI_ASZ_VASZ_SHIFT)
#define TLI_ASZ_HASZ_SHIFT         (16)      /* Bits 16-27: Accumulated Active Width (pixel clocks) */
#define TLI_ASZ_HASZ_MASK          (0xfff << TLI_ASZ_HASZ_SHIFT)
#  define TLI_ASZ_HASZ(n)          ((uint32_t)(n) << TLI_ASZ_HASZ_SHIFT)

/* LTDC Total Width Configuration Register */

#define TLI_TSZ_VTSZ_SHIFT      (0)       /* Bits 0-10: Total Height (scan lines) */
#define TLI_TSZ_VTSZ_MASK       (0x7ff << TLI_TSZ_VTSZ_SHIFT)
#  define TLI_TSZ_VTSZ(n)       ((uint32_t)(n) << TLI_TSZ_VTSZ_SHIFT)
#define TLI_TSZ_HTSZ_SHIFT      (16)      /* Bits 16-27: Total Width (pixel clocks) */
#define TLI_TSZ_HTSZ_MASK       (0xfff << TLI_TSZ_HTSZ_SHIFT)
#  define TLI_TSZ_HTSZ(n)       ((uint32_t)(n) << TLI_TSZ_HTSZ_SHIFT)

/* LTDC Global Control Register */

#define TLI_CTL_TLIEN            (1 << 0)  /* Bit 0: LCD-TFT Controller Enable */
#define TLI_CTL_BDB_SHIFT        (4)       /* Bits 4-6: Dither Blue Width */
#define TLI_CTL_BDB_MASK         (0x7 << TLI_CTL_BDB_SHIFT)
#  define TLI_CTL_BDB(n)         ((uint32_t)(n) << TLI_CTL_BDB_SHIFT)
#define TLI_CTL_GDB_SHIFT        (8)       /* Bits 8-10: Dither Green Width */
#define TLI_CTL_GDB_MASK         (0x7 << TLI_CTL_GDB_SHIFT)
#  define TLI_CTL_GDB(n)         ((uint32_t)(n) << TLI_CTL_GDB_SHIFT)
#define TLI_CTL_RDB_SHIFT        (12)      /* Bits 12-14: Dither Red Width */
#define TLI_CTL_RDB_MASK         (0x7 << TLI_CTL_RDB_SHIFT)
#  define TLI_CTL_RDB(n)         ((uint32_t)(n) << TLI_CTL_RDB_SHIFT)
#define TLI_CTL_DFEN             (1 << 16) /* Bit 16:  Dither Enable */
#define TLI_CTL_CLKPS            (1 << 28) /* Bit 28: Pixel Clock Polarity */
#define TLI_CTL_DEPS             (1 << 29) /* Bit 29: Data Enable Polarity */
#define TLI_CTL_VPPS             (1 << 30) /* Bit 30: Vertical Sync Polarity */
#define TLI_CTL_HPPS             (1 << 31) /* Bit 31: Horizontal Sync Polar */

/* LTDC Shadow Reload Configuration Register */

#define TLI_RL_RQR               (1 << 0)  /* Bit 0: Immediate Reload */
#define TLI_RL_FBR               (1 << 1)  /* Bit 1: Vertical Blanking Reload */

/* LTDC Background Color Configuration Register */

#define TLI_BGC_BVB_SHIFT        (0)       /* Bits 0-7: BG Color Blue Value */
#define TLI_BGC_BVB_MASK         (0xff << TLI_BGC_BVB_SHIFT)
#  define TLI_BGC_BVB(n)         ((uint32_t)(n) << TLI_BGC_BVB_SHIFT)
#define TLI_BGC_BVG_SHIFT        (8)       /* Bits 8-15: BG Color Green Val */
#define TLI_BGC_BVG_MASK         (0xff << TLI_BGC_BVG_SHIFT)
#  define TLI_BGC_BVG(n)         ((uint32_t)(n) << TLI_BGC_BVG_SHIFT)
#define TLI_BGC_BVR_SHIFT        (16)      /* Bits 16-23: BG Color Red Val */
#define TLI_BGC_BVR_MASK         (0xff << TLI_BGC_BVR_SHIFT)
#  define TLI_BGC_BVR(n)         ((uint32_t)(n) << TLI_BGC_BVR_SHIFT)

/* LTDC Interrupt Enable Register */

#define TLI_INTEN_LMIE           (1 << 0)  /* Bit 0: Line Interrupt Enable */
#define TLI_INTEN_FEIE           (1 << 1)  /* Bit 1: FIFO Underrun Int En */
#define TLI_INTEN_TEIE           (1 << 2)  /* Bit 2: Transfer Error Int En */
#define TLI_INTEN_LCRIE          (1 << 3)  /* Bit 3: Register Reload Int En */

/* LTDC Interrupt Status Register */

#define TLI_INTF_LMF             (1 << 0)  /* Bit 0: Line Interrupt Flag */
#define TLI_INTF_FEF             (1 << 1)  /* Bit 1: FIFO Underrun Int Flag */
#define TLI_INTF_TEF             (1 << 2)  /* Bit 2: Transfer Error Int Flag */
#define TLI_INTF_LCRF            (1 << 3)  /* Bit 3: Register Reload Int Flag */

/* LTDC Interrupt Clear Register */

#define TLI_INTC_LMC             (1 << 0)  /* Bit 0: Clear Line Int Flag */
#define TLI_INTC_FEC             (1 << 1)  /* Bit 1: Clear FIFO Underrun Int */
#define TLI_INTC_TEC             (1 << 2)  /* Bit 2: Clear Transfer Err Int */
#define TLI_INTC_LCRC            (1 << 3)  /* Bit 3: Clear Reg Reload Int */

/* LTDC Line Interrupt Position Configuration Register */

#define TLI_LM_LM_SHIFT      (0)       /* Bits 0-10: Line Interrupt Position */
#define TLI_LM_LM_MASK       (0x7ff << TLI_LM_LM_SHIFT)
#  define TLI_LM_LM(n)       ((uint32_t)(n) << TLI_LM_LM_SHIFT)

/* LTDC Current Position Status Register */

#define TLI_CPPOS_VPOS_SHIFT       (0)       /* Bits 0-15: Current Y Position */
#define TLI_CPPOS_VPOS_MASK        (0xffff << TLI_CPPOS_VPOS_SHIFT)
#  define TLI_CPPOS_VPOS(n)        ((uint32_t)(n) << TLI_CPPOS_VPOS_SHIFT)
#define TLI_CPPOS_HPOS_SHIFT       (16)      /* Bits 16-31: Current X Position */
#define TLI_CPPOS_HPOS_MASK        (0xffff << TLI_CPPOS_HPOS_SHIFT)
#  define TLI_CPPOS_HPOS(n)        ((uint32_t)(n) << TLI_CPPOS_HPOS_SHIFT)

/* LTDC Current Display Status Register */

#define TLI_STAT_VDE             (1 << 0)  /* Bit 0: Vert Data Enable Status */
#define TLI_STAT_HDE             (1 << 1)  /* Bit 1: Horiz Data Enable Status */
#define TLI_STAT_VS              (1 << 2)  /* Bit 2: Vertical Sync Status */
#define TLI_STAT_HS              (1 << 3)  /* Bit 3: Horizontal Sync Status */

/* LTDC Layer x Control Register */

#define TLI_LXCTL_LEN            (1 << 0)  /* Bit 0: Layer Enable */
#define TLI_LXCTL_CKEYEN         (1 << 1)  /* Bit 1: Color Keying Enable */
#define TLI_LXCTL_LUTEN          (1 << 4)  /* Bit 4: Color Look-Up Table En */

/* LTDC Layer x Window Horizontal Position Configuration Register */

#define TLI_LXHPOS_WLP_SHIFT  (0)       /* Bits 0-11: Window Horizontal Start Position */
#define TLI_LXHPOS_WLP_MASK   (0xFFF << TLI_LXHPOS_WLP_SHIFT)
#  define TLI_LXHPOS_WLP(n)   ((uint32_t)(n) << TLI_LXHPOS_WLP_SHIFT)
#define TLI_LXHPOS_WRP_SHIFT  (16)      /* Bits 16-27: Window Horizontal Stop Position */
#define TLI_LXHPOS_WRP_MASK   (0xFFF << TLI_LXHPOS_WRP_SHIFT)
#  define TLI_LXHPOS_WRP(n)   ((uint32_t)(n) << TLI_LXHPOS_WRP_SHIFT)

/* LTDC Layer x Window Vertical Position Configuration Register */

#define TLI_LXVPOS_WTP_SHIFT  (0)       /* Bits 0-10: Window Vertical Start Position */
#define TLI_LXVPOS_WTP_MASK   (0x7ff << TLI_LXVPOS_WTP_SHIFT)
#  define TLI_LXVPOS_WTP(n)   ((uint32_t)(n) << TLI_LXVPOS_WTP_SHIFT)
#define TLI_LXVPOS_WBP_SHIFT  (16)      /* Bits 16-26: Window Vertical Stop Position */
#define TLI_LXVPOS_WBP_MASK   (0x7ff << TLI_LXVPOS_WBP_SHIFT)
#  define TLI_LXVPOS_WBP(n)   ((uint32_t)(n) << TLI_LXVPOS_WBP_SHIFT)

/* LTDC Layer x Color Keying Configuration Register */

#define TLI_LXCKEY_CKEYB_SHIFT    (0)       /* Bits 0-7: Color Key Blue Value */
#define TLI_LXCKEY_CKEYB_MASK     (0xff << TLI_LXCKEY_CKEYB_SHIFT)
#  define TLI_LXCKEY_CKEYB(n)     ((uint32_t)(n) << TLI_LXCKEY_CKEYB_SHIFT)
#define TLI_LXCKEY_CKEYG_SHIFT   (8)       /* Bits 8-15: Color Key Green Value */
#define TLI_LXCKEY_CKEYG_MASK    (0xff << TLI_LXCKEY_CKEYG_SHIFT)
#  define TLI_LXCKEY_CKEYG(n)    ((uint32_t)(n) << TLI_LXCKEY_CKEYG_SHIFT)
#define TLI_LXCKEY_CKEYR_SHIFT     (16)       /* Bits 16-23: Color Key Red Value */
#define TLI_LXCKEY_CKEYR_MASK      (0xff << TLI_LXCKEY_CKEYR_SHIFT)
#  define TLI_LXCKEY_CKEYR(n)      ((uint32_t)(n) << TLI_LXCKEY_CKEYR_SHIFT)

/* LTDC Layer x Pixel Format Configuration Register */

#define TLI_LXPPF_PPF_SHIFT        (0)       /* Bits 0-2: Pixel Format */
#define TLI_LXPPF_PPF_MASK         (0x7 << TLI_LXPPF_PPF_SHIFT)
#  define TLI_LXPPF_PPF(n)         ((uint32_t)(n) << TLI_LXPPF_PPF_SHIFT)

#define TLI_PF_ARGB8888            0  //to  do
#define TLI_PF_RGB888              1
#define TLI_PF_RGB565              2
#define TLI_PF_ARGB1555            3
#define TLI_PF_ARGB4444            4
#define TLI_PF_L8                5         /* 8-bit Luminance (CLUT lookup) */
#define TLI__PF_AL44             6         /* 4-bit Alpha, 4-bit Luminance */
#define TLI_PF_AL88              7         /* 8-bit Alpha, 8-bit Luminance */

/* LTDC Layer x Constant Alpha Configuration Register */

#define TLI_LXSA_SA_SHIFT    (0)       /* Bits 0-7: Constant Alpha */
#define TLI_LXSA_SA_MASK     (0x7 << TLI_LXSA_SA_SHIFT)
#  define TLI_LXSA_SA(n)     ((uint32_t)(n) << TLI_LXSA_SA_SHIFT)

/* LTDC Layer x Default Color Configuration Register */

#define TLI_LXDC_DCB_SHIFT    (0)       /* Bits 0-7: Default Color Blue Value */
#define TLI_LXDC_DCB_MASK     (0xff << TLI_LXDC_DCB_SHIFT)
#  define TLI_LXDC_DCB(n)     ((uint32_t)(n) << TLI_LXDC_DCB_SHIFT)
#define TLI_LXDC_DCG_SHIFT   (8)       /* Bits 8-15: Default Color Green Value */
#define TLI_LXDC_DCG_MASK    (0xff << TLI_LXDC_DCG_SHIFT)
#  define TLI_LXDC_DCG(n)    ((uint32_t)(n) << TLI_LXDC_DCG_SHIFT)
#define TLI_LXDC_DCR_SHIFT     (16)       /* Bits 16-23: Default Color Red Value */
#define TLI_LXDC_DCR_MASK      (0xff << TLI_LXDC_DCR_SHIFT)
#  define TLI_LXDC_DCR(n)      ((uint32_t)(n) << TLI_LXDC_DCR_SHIFT)
#define TLI_LXDC_DCA_SHIFT   (24)       /* Bits 24-31: Default Color Alpha Value */
#define TLI_LXDC_DCA_MASK    (0xff << TLI_LXDC_DCA_SHIFT)
#  define TLI_LXDC_DCA(n)    ((uint32_t)(n) << TLI_LXDC_DCA_SHIFT)

/* LTDC Layer x Blending Factors Configuration Register */

#define TLI_LXBLEND_ACF2_SHIFT       (0)       /* Bits 0-2: Blending Factor 2 */
#define TLI_LXBLEND_ACF2_MASK        (0x7 << TLI_LXBLEND_ACF2_SHIFT)
#  define TLI_LXBLEND_ACF2(n)        ((uint32_t)(n) << TLI_LXBLEND_ACF2_SHIFT)
#define TLI_LXBLEND_ACF1_SHIFT       (8)       /* Bits 8-10: Blending Factor 1 */
#define TLI_LXBLEND_ACF1_MASK        (0x7 << TLI_LXBLEND_ACF1_SHIFT)
#  define TLI_LXBLEND_ACF1(n)        ((uint32_t)(n) << TLI_LXBLEND_ACF1_SHIFT)

#define TLI_BF1_CONST_ALPHA        0x04      /* Constant Alpha */   //to do
#define TLI_BF1_PIXEL_ALPHA        0x06      /* Pixel Alpha x Constant Alpha */
#define TLI_BF2_CONST_ALPHA        0x05      /* Constant Alpha */
#define TLI_BF2_PIXEL_ALPHA        0x07      /* Pixel Alpha x Constant Alpha */

/* LTDC Layer x Color Frame Buffer Length Configuration Register */

#define TLI_LXFLLEN_FLL_SHIFT    (0)       /* Bits 0-12: Color Frame Buffer Line Length */
#define TLI_LXFLLEN_FLL_MASK     (0x1fff << TLI_LXFLLEN_FLL_SHIFT)
#  define TLI_LXFLLEN_FLL(n)     ((uint32_t)(n) << TLI_LXFLLEN_FLL_SHIFT)
#define TLI_LXFLLEN_STDOFF_SHIFT     (16)       /* Bits 16-28: Color Frame Buffer Pitch */
#define TLI_LXFLLEN_STDOFF_MASK      (0x1fff << TLI_LXFLLEN_STDOFF_SHIFT)
#  define TLI_LXFLLEN_STDOFF(n)      ((uint32_t)(n) << TLI_LXFLLEN_STDOFF_SHIFT)

/* LTDC Layer x Color Frame Buffer Line Number Register */

#define TLI_LXFTLN_FTLN_SHIFT      (0)       /* Bits 0-10: Color Frame Buffer Line Number */
#define TLI_LXFTLN_FTLN_MASK       (0x7ff << TLI_LXFTLN_FTLN_SHIFT)
#  define TLI_LXFTLN_FTLN(n)       ((uint32_t)(n) << TLI_LXFTLN_FTLN_SHIFT)

/* LTDC Layer x CLUT Write Register */

#define TLI_LXLUT_TB_SHIFT    (0)       /* Bits 0-7: Default Color Blue Value */
#define TLI_LXLUT_TB_MASK     (0xff << TLI_LXLUT_TB_SHIFT)
#  define TLI_LXLUT_TB(n)     ((uint32_t)(n) << TLI_LXLUT_TB_SHIFT)
#define TLI_LXLUT_TG_SHIFT   (8)       /* Bits 8-15: Default Color Green Value */
#define TLI_LXLUT_TG_MASK    (0xff << TLI_LXLUT_TG_SHIFT)
#  define TLI_LXLUT_TG(n)    ((uint32_t)(n) << TLI_LXLUT_TG_SHIFT)
#define TLI_LXLUT_TR_SHIFT     (16)       /* Bits 16-23: Default Color Red Value */
#define TLI_LXLUT_TR_MASK      (0xff << TLI_LXLUT_TR_SHIFT)
#  define TLI_LXLUT_TR(n)      ((uint32_t)(n) << TLI_LXLUT_TR_SHIFT)
#define TLI_LXLUT_TADD_SHIFT (24)       /* Bits 24-31: CLUT Address */
#define TLI_LXLUT_TADD_MASK  (0xff << TLI_LXLUT_TADD_SHIFT)
#  define TLI_LXLUT_TADD(n)  ((uint32_t)(n) << TLI_LXLUT_TADD_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_TLI_H */
