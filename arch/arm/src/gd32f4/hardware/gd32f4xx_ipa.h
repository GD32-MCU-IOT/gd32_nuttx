/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_ipa.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_IPA_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_IPA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/gd32f4xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GD32_IPA_NCLUT           256    /* Number of entries in the CLUT */

/* IPA Register Offsets *****************************************************/

#define GD32_IPA_CTL_OFFSET      0x0000  /* IPA Control Register */
#define GD32_IPA_INTF_OFFSET     0x0004  /* IPA Interrupt Status Register */
#define GD32_IPA_INTC_OFFSET     0x0008  /* IPA Interrupt Flag Clear Reg */
#define GD32_IPA_FMADDR_OFFSET   0x000c  /* IPA Foreground Memory Addr Reg */
#define GD32_IPA_FLOFF_OFFSET    0x0010  /* IPA Foreground Offset Register */
#define GD32_IPA_BMADDR_OFFSET   0x0014  /* IPA Background Memory Addr Reg */
#define GD32_IPA_BLOFF_OFFSET    0x0018  /* IPA Background Offset Register */
#define GD32_IPA_FPCTL_OFFSET    0x001c  /* IPA Foreground PFC Control Reg */
#define GD32_IPA_FPV_OFFSET      0x0020  /* IPA Foreground Color Register */
#define GD32_IPA_BPCTL_OFFSET    0x0024  /* IPA Background PFC Control Reg */
#define GD32_IPA_BPV_OFFSET      0x0028  /* IPA Background Color Register */
#define GD32_IPA_FLMADDR_OFFSET  0x002c  /* IPA FG CLUT Memory Addr Reg */
#define GD32_IPA_BLMADDR_OFFSET  0x0030  /* IPA BG CLUT Memory Addr Reg */
#define GD32_IPA_DPCTL_OFFSET    0x0034  /* IPA Output PFC Control Register */
#define GD32_IPA_DPV_OFFSET      0x0038  /* IPA Output Color Register */
#define GD32_IPA_DMADDR_OFFSET   0x003c  /* IPA Output Memory Address Reg */
#define GD32_IPA_DLOFF_OFFSET    0x0040  /* IPA Output Offset Register */
#define GD32_IPA_IMS_OFFSET      0x0044  /* IPA Number Of Line Register */
#define GD32_IPA_LM_OFFSET       0x0048  /* IPA Line Watermark Register */
#define GD32_IPA_ITCTL_OFFSET    0x004c  /* IPA AHB Master Time Config Reg */

/* IPA Register Addresses ***************************************************/

#define GD32_IPA_CTL              (GD32_IPA_BASE + GD32_IPA_CTL_OFFSET)
#define GD32_IPA_INTF             (GD32_IPA_BASE + GD32_IPA_INTF_OFFSET)
#define GD32_IPA_INTC             (GD32_IPA_BASE + GD32_IPA_INTC_OFFSET)
#define GD32_IPA_FMADDR           (GD32_IPA_BASE + GD32_IPA_FMADDR_OFFSET)
#define GD32_IPA_FLOFF            (GD32_IPA_BASE + GD32_IPA_FLOFF_OFFSET)
#define GD32_IPA_BMADDR           (GD32_IPA_BASE + GD32_IPA_BMADDR_OFFSET)
#define GD32_IPA_BLOFF            (GD32_IPA_BASE + GD32_IPA_BLOFF_OFFSET)
#define GD32_IPA_FPCTL            (GD32_IPA_BASE + GD32_IPA_FPCTL_OFFSET)
#define GD32_IPA_FPV              (GD32_IPA_BASE + GD32_IPA_FPV_OFFSET)
#define GD32_IPA_BPCTL            (GD32_IPA_BASE + GD32_IPA_BPCTL_OFFSET)
#define GD32_IPA_BPV              (GD32_IPA_BASE + GD32_IPA_BPV_OFFSET)
#define GD32_IPA_FLMADDR          (GD32_IPA_BASE + GD32_IPA_FLMADDR_OFFSET)
#define GD32_IPA_BLMADDR          (GD32_IPA_BASE + GD32_IPA_BLMADDR_OFFSET)
#define GD32_IPA_DPCTL            (GD32_IPA_BASE + GD32_IPA_DPCTL_OFFSET)
#define GD32_IPA_DPV              (GD32_IPA_BASE + GD32_IPA_DPV_OFFSET)
#define GD32_IPA_DMADDR           (GD32_IPA_BASE + GD32_IPA_DMADDR_OFFSET)
#define GD32_IPA_DLOFF            (GD32_IPA_BASE + GD32_IPA_DLOFF_OFFSET)
#define GD32_IPA_IMS              (GD32_IPA_BASE + GD32_IPA_IMS_OFFSET)
#define GD32_IPA_LM               (GD32_IPA_BASE + GD32_IPA_LM_OFFSET)
#define GD32_IPA_ITCTL            (GD32_IPA_BASE + GD32_IPA_ITCTL_OFFSET)

/* IPA Register Bit Definitions *********************************************/

/* IPA Control Register */

#define IPA_CTL_TEN              (1 << 0)  /* Start Bit */
#define IPA_CTL_THU              (1 << 1)  /* Suspend Bit */
#define IPA_CTL_TST              (1 << 2)  /* Abort Bit */
#define IPA_CTL_TAEIE            (1 << 8)  /* Transfer Error Int Enable Bit */
#define IPA_CTL_FTFIE            (1 << 9)  /* Transfer Complete Int Enable */
#define IPA_CTL_TLMIE            (1 << 10) /* Transfer Watermark Int Enable */
#define IPA_CTL_LACIE            (1 << 11) /* CLUT Access Error Int Enable */
#define IPA_CTL_LLFIE            (1 << 12) /* CLUT Transfer Complete Int En */
#define IPA_CTL_WCFIE            (1 << 13) /* Config Error Int Enable Bit */
#define IPA_CTL_PFCM_SHIFT       (16)      /* Bits 16-17 IPA mode Bits */
#define IPA_CTL_PFCM_MASK        (3 << IPA_CTL_PFCM_SHIFT)
#define IPA_CTL_PFCM(n)          ((uint32_t)(n) << IPA_CTL_PFCM_SHIFT)

/* IPA Interrupt Status Register */

#define IPA_INTF_TAEIF           (1 << 0)  /* Transfer error interrupt flag */
#define IPA_INTF_FTFIF           (1 << 1)  /* Transfer Complete Int flag */
#define IPA_INTF_TLMIF           (1 << 2)  /* Transfer Watermark Int flag */
#define IPA_INTF_LACIF           (1 << 3)  /* CLUT Access Error Int flag */
#define IPA_INTF_LLFIF           (1 << 4)  /* CLUT Transfer Complete Int flag */
#define IPA_INTF_WCFIF           (1 << 5)  /* Configuration Error Int flag */

/* IPA Interrupt Flag Clear Register */

#define IPA_INTC_TAEIFC          (1 << 0)  /* Clear Transfer Int Flag */
#define IPA_INTC_FTFIFC          (1 << 1)  /* Clear Transfer Complete Int */
#define IPA_INTC_TLMIFC          (1 << 2)  /* Clear Transfer Watermark Int */
#define IPA_INTC_LACIFC          (1 << 3)  /* Clear CLUT Access Error Int */
#define IPA_INTC_LLFIFC          (1 << 4)  /* Clear CLUT Transfer Complete */
#define IPA_INTC_WCFIFC          (1 << 5)  /* Clear Config Error Int Flag */

/* IPA Foreground Memory Access Register */

/* IPA Background Memory Access Register */

/* IPA Foreground/Background Offset Register */

/* TODO:
 * #define IPA_XGOR_SHIFT              (0)       Bits 0-13 Line Offset
 * #define IPA_XGOR_MASK               (0x3fff << IPA_XGOR_SHIFT)
 * #define IPA_XGOR(n)                 ((uint32_t)(n) << IPA_XGOR_SHIFT)
 */

/* IPA Foreground/Background PFC Control Register */

#define IPA_FPCTL_FPF_SHIFT      (0)      /* Bits 0-3 Color Mode */
#define IPA_FPCTL_FPF_MASK       (0xf << IPA_FPCTL_FPF_SHIFT)
#define IPA_FPCTL_FPF(n)         ((uint32_t)(n) << IPA_FPCTL_FPF_SHIFT)
#define IPA_FPCTL_FLPF           (1 << 4)  /* CLUT Color Mode */
#define IPA_FPCTL_FLLEN          (1 << 5)  /* Start */
#define IPA_FPCTL_FCNP_SHIFT     (8)       /* Bits 8-15 CLUT Size */
#define IPA_FPCTL_FCNP_MASK      (0xff << IPA_FPCTL_FCNP_SHIFT)
#define IPA_FPCTL_FCNP(n)        ((uint32_t)(n) << IPA_FPCTL_FCNP_SHIFT)
#define IPA_FPCTL_FAVCA_SHIFT    (16)      /* Bits 16-17 Alpha Mode */
#define IPA_FPCTL_FAVCA_MASK       (3 << IPA_FPCTL_FAVCA_SHIFT)
#define IPA_FPCTL_FAVCA(n)         ((uint32_t)(n) << IPA_FPCTL_FAVCA_SHIFT)
#define IPA_FPCTL_FPDAV_SHIFT   (24)     /* Bits 24-31 Alpha Value */
#define IPA_FPCTL_FPDAV_MASK    (0xff << IPA_FPCTL_FPDAV_SHIFT)
#define IPA_FPCTL_FPDAV(n)      ((uint32_t)(n) << IPA_FPCTL_FPDAV_SHIFT)

/* IPA PFC alpha mode */

#define GD32_IPA_PFCCR_AM_NONE   0
#define GD32_IPA_PFCCR_AM_CONST  1
#define GD32_IPA_PFCCR_AM_PIXEL  2

/* IPA Foreground/Background Color Register */

#define IPA_FPV_FPDBV_SHIFT     (0)      /* Bits 0-7 Blue Value */
#define IPA_FPV_FPDBV_MASK      (0xff << IPA_FPV_FPDBV_SHIFT)
#define IPA_FPV_FPDBV(n)        ((uint32_t)(n) << IPA_FPV_FPDBV_SHIFT)
#define IPA_FPV_FPDGV_SHIFT    (8)      /* Bits 8-15 Green Value */
#define IPA_FPV_FPDGV_MASK     (0xff << IPA_FPV_FPDGV_SHIFT)
#define IPA_FPV_FPDGV(n)       ((uint32_t)(n) << IPA_FPV_FPDGV_SHIFT)
#define IPA_FPV_FPDRV_SHIFT      (16)     /* Bits 16-23 Red Value */
#define IPA_FPV_FPDRV_MASK       (0xff << IPA_FPV_FPDRV_SHIFT)
#define IPA_FPV_FPDRV(n)         ((uint32_t)(n) << IPA_FPV_FPDRV_SHIFT)

/* IPA Foreground CLUT Memory Address Register */
#define IPA_FLMADDR_FLMADDR_SHIFT     (0)      /* Bits 0-31 Memory Address */
#define IPA_FLMADDR_FLMADDR_MASK      (0xffffffff << IPA_FLMADDR_FLMADDR_SHIFT)
#define IPA_FLMADDR_FLMADDR(n)        ((uint32_t)(n) << IPA_FLMADDR_FLMADDR_SHIFT)

/* IPA Background CLUT Memory Address Register */
#define IPA_BLMADDR_BLMADDR_SHIFT     (0)      /* Bits 0-31 Memory Address */
#define IPA_BLMADDR_BLMADDR_MASK      (0xffffffff << IPA_BLMADDR_BLMADDR_SHIFT)
#define IPA_BLMADDR_BLMADDR(n)        ((uint32_t)(n) << IPA_BLMADDR_BLMADDR_SHIFT)

/* IPA Output PFC Control Register */

/* TODO:
 * #define IPA_DPCTL_CM_SHIFT     (0)  Bits 0-2 Color Mode
 * #define IPA_DPCTL_CM_MASK      (7 << IPA_DPCTL_CM_SHIFT)
 * #define IPA_DPCTL_CM(n)        ((uint32_t)(n) << IPA_DPCTL_CM_SHIFT)
 */

#define IPA_DPCTL_DPF_SHIFT       (0)  /* Bits 0-2 Color Mode */
#define IPA_DPCTL_DPF_MASK        (7 << IPA_DPCTL_DPF_SHIFT)
#define IPA_DPCTL_DPF(n)          ((uint32_t)(n) << IPA_DPCTL_DPF_SHIFT)

/* IPA PFC Pixel Format */

#define IPA_PF_ARGB8888           0
#define IPA_PF_RGB888             1
#define IPA_PF_RGB565             2
#define IPA_PF_ARGB1555           3
#define IPA_PF_ARGB4444           4
#define IPA_PF_L8                 5
#define IPA_PF_AL44               6
#define IPA_PF_AL88               7
#define IPA_PF_L4                 8
#define IPA_PF_A8                 9
#define IPA_PF_A4                 10

/* IPA Output Color Register */

#define IPA_DPV_DPDBV_SHIFT         (0)  /* Bits 0-7 Blue Value */
#define IPA_DPV_DPDBV_MASK          (0xff << IPA_DPV_DPDBV_SHIFT)
#define IPA_DPV_DPDBV(n)            ((uint32_t)(n) << IPA_DPV_DPDBV_SHIFT)
#define IPA_DPV_DPDGV_SHIFT         (8)  /* Bits 8-15 Green Value */
#define IPA_DPV_DPDGV_MASK          (0xff << IPA_DPV_DPDGV_SHIFT)
#define IPA_DPV_DPDGV(n)            ((uint32_t)(n) << IPA_DPV_DPDGV_SHIFT)
#define IPA_DPV_DPDRV_SHIFT         (16)  /* Bits 16-23 Red Value */
#define IPA_DPV_DPDRV_MASK          (0xff << IPA_DPV_DPDRV_SHIFT)
#define IPA_DPV_DPDRV(n)            ((uint32_t)(n) << IPA_DPV_DPDRV_SHIFT)
#define IPA_DPV_DPDAV_SHIFT         (24)  /* Bits 24-31 Alpha Value */
#define IPA_DPV_DPDAV_MASK          (0xff << IPA_DPV_DPDAV_SHIFT)
#define IPA_DPV_DPDAV(n)            ((uint32_t)(n) << IPA_DPV_DPDAV_SHIFT)

/* TODO */

/* IPA Output Memory Address Register */
#define IPA_DMADDR_DMADDR_SHIFT     (0)      /* Bits 0-31 Memory Address */
#define IPA_DMADDR_DMADDR_MASK      (0xffffffffu << IPA_DMADDR_DMADDR_SHIFT)
#define IPA_DMADDR_DMADDR(n)        ((uint32_t)(n) << IPA_DMADDR_DMADDR_SHIFT)

/* IPA Output Offset Register */

#define IPA_DLOFF_DLOFF_SHIFT          (0)      /* Bits 0-13 Line Offset */
#define IPA_DLOFF_DLOFF_MASK           (0x3fff << IPA_DLOFF_DLOFF_SHIFT)
#define IPA_DLOFF_DLOFF(n)             ((uint32_t)(n) << IPA_DLOFF_DLOFF_SHIFT)

/* IPA Number Of Line Register */

#define IPA_IMS_HEIGHT_SHIFT          (0)      /* Bits 0-15 Number Of Lines */
#define IPA_IMS_HEIGHT_MASK           (0xffff << IPA_IMS_HEIGHT_SHIFT)
#define IPA_IMS_HEIGHT(n)             ((uint32_t)(n) << IPA_IMS_HEIGHT_SHIFT)
#define IPA_IMS_WIDTH_SHIFT          (16)     /* Bits 16-29 Pixel per Lines */
#define IPA_IMS_WIDTH_MASK           (0x3fff << IPA_IMS_WIDTH_SHIFT)
#define IPA_IMS_WIDTH(n)             ((uint32_t)(n) << IPA_IMS_WIDTH_SHIFT)

/* IPA Line Watermark Register */

#define IPA_LM_LM_SHIFT          (0)      /* Bits 0-15 Line Watermark */
#define IPA_LM_LM_MASK           (0xffff << IPA_LM_LM_SHIFT)
#define IPA_LM_LM(n)             ((uint32_t)(n) << IPA_LM_LM_SHIFT)

/* IPA AHB Master Timer Configuration Register */

#define IPA_ITCTL_ITEN              (1 << 0) /* Enable */
#define IPA_ITCTL_NCCI_SHIFT        (8)      /* Bits 8-15 Dead Time */
#define IPA_ITCTL_NCCI_MASK         (0xff << IPA_ITCTL_NCCI_SHIFT)
#define IPA_ITCTL_NCCI(n)           ((uint32_t)(n) << IPA_ITCTL_NCCI_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_IPA_H */
