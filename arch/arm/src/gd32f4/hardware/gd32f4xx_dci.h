/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_dci.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_DCI_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_DCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/gd32f4xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DCI Register Offsets *****************************************************/

#define GD32_DCI_CTL_OFFSET       0x0000  /* DCI control register */
#define GD32_DCI_STAT0_OFFSET     0x0004  /* DCI status register 0 */
#define GD32_DCI_STAT1_OFFSET     0x0008  /* DCI status register 1 */
#define GD32_DCI_INTEN_OFFSET     0x000c  /* DCI interrupt enable register */
#define GD32_DCI_INTF_OFFSET      0x0010  /* DCI interrupt flag register */
#define GD32_DCI_INTC_OFFSET      0x0014  /* DCI interrupt clear register */
#define GD32_DCI_SC_OFFSET        0x0018  /* DCI synchronization codes register */
#define GD32_DCI_SCUMSK_OFFSET    0x001c  /* DCI synchronization codes unmask register */
#define GD32_DCI_CWSPOS_OFFSET    0x0020  /* DCI cropping window start position register */
#define GD32_DCI_CWSZ_OFFSET      0x0024  /* DCI cropping window size register */
#define GD32_DCI_DATA_OFFSET      0x0028  /* DCI data register */

/* DCI Register Addresses ***************************************************/

#define GD32_DCI_CTL              (GD32_DCI_BASE + GD32_DCI_CTL_OFFSET)
#define GD32_DCI_STAT0            (GD32_DCI_BASE + GD32_DCI_STAT0_OFFSET)
#define GD32_DCI_STAT1            (GD32_DCI_BASE + GD32_DCI_STAT1_OFFSET)
#define GD32_DCI_INTEN            (GD32_DCI_BASE + GD32_DCI_INTEN_OFFSET)
#define GD32_DCI_INTF             (GD32_DCI_BASE + GD32_DCI_INTF_OFFSET)
#define GD32_DCI_INTC             (GD32_DCI_BASE + GD32_DCI_INTC_OFFSET)
#define GD32_DCI_SC               (GD32_DCI_BASE + GD32_DCI_SC_OFFSET)
#define GD32_DCI_SCUMSK           (GD32_DCI_BASE + GD32_DCI_SCUMSK_OFFSET)
#define GD32_DCI_CWSPOS           (GD32_DCI_BASE + GD32_DCI_CWSPOS_OFFSET)
#define GD32_DCI_CWSZ             (GD32_DCI_BASE + GD32_DCI_CWSZ_OFFSET)
#define GD32_DCI_DATA             (GD32_DCI_BASE + GD32_DCI_DATA_OFFSET)

/* DCI Register Bit Definitions *********************************************/

/* DCI Control Register (DCI_CTL) */

#define DCI_CTL_CAP               (1 << 0)   /* Bit 0:  Capture enable */
#define DCI_CTL_SNAP              (1 << 1)   /* Bit 1:  Snapshot mode */
#define DCI_CTL_WDEN              (1 << 2)   /* Bit 2:  Window enable */
#define DCI_CTL_JM                (1 << 3)   /* Bit 3:  JPEG mode */
#define DCI_CTL_ESM               (1 << 4)   /* Bit 4:  Embedded synchronous mode */
#define DCI_CTL_CKS               (1 << 5)   /* Bit 5:  Clock polarity selection */
#define DCI_CTL_HPS               (1 << 6)   /* Bit 6:  Horizontal polarity selection */
#define DCI_CTL_VPS               (1 << 7)   /* Bit 7:  Vertical polarity selection */

#define DCI_CTL_FR_SHIFT          (8)        /* Bits 8-9: Frame rate */
#define DCI_CTL_FR_MASK           (3 << DCI_CTL_FR_SHIFT)
#define DCI_CTL_FR(n)             ((uint32_t)(n) << DCI_CTL_FR_SHIFT)
#define DCI_CTL_FR_ALL            DCI_CTL_FR(0)  /* Capture all frames */
#define DCI_CTL_FR_1_2            DCI_CTL_FR(1)  /* Capture one in 2 frames */
#define DCI_CTL_FR_1_4            DCI_CTL_FR(2)  /* Capture one in 4 frames */

#define DCI_CTL_DCIF_SHIFT        (10)       /* Bits 10-11: DCI interface format */
#define DCI_CTL_DCIF_MASK         (3 << DCI_CTL_DCIF_SHIFT)
#define DCI_CTL_DCIF(n)           ((uint32_t)(n) << DCI_CTL_DCIF_SHIFT)
#define DCI_CTL_DCIF_8BITS        DCI_CTL_DCIF(0)  /* 8-bit data on every pixel clock */
#define DCI_CTL_DCIF_10BITS       DCI_CTL_DCIF(1)  /* 10-bit data on every pixel clock */
#define DCI_CTL_DCIF_12BITS       DCI_CTL_DCIF(2)  /* 12-bit data on every pixel clock */
#define DCI_CTL_DCIF_14BITS       DCI_CTL_DCIF(3)  /* 14-bit data on every pixel clock */

#define DCI_CTL_DCIEN             (1 << 14)  /* Bit 14: DCI enable */

/* DCI Status Register 0 (DCI_STAT0) */

#define DCI_STAT0_HS              (1 << 0)   /* Bit 0:  HS line status */
#define DCI_STAT0_VS              (1 << 1)   /* Bit 1:  VS line status */
#define DCI_STAT0_FV              (1 << 2)   /* Bit 2:  FIFO valid */

/* DCI Status Register 1 (DCI_STAT1) */

#define DCI_STAT1_EFF             (1 << 0)   /* Bit 0:  End of frame flag */
#define DCI_STAT1_OVRF            (1 << 1)   /* Bit 1:  FIFO overrun flag */
#define DCI_STAT1_ESEF            (1 << 2)   /* Bit 2:  Embedded synchronous error flag */
#define DCI_STAT1_VSF             (1 << 3)   /* Bit 3:  Vsync flag */
#define DCI_STAT1_ELF             (1 << 4)   /* Bit 4:  End of line flag */

/* DCI Interrupt Enable Register (DCI_INTEN) */

#define DCI_INTEN_EFIE            (1 << 0)   /* Bit 0:  End of frame interrupt enable */
#define DCI_INTEN_OVRIE           (1 << 1)   /* Bit 1:  FIFO overrun interrupt enable */
#define DCI_INTEN_ESEIE           (1 << 2)   /* Bit 2:  Embedded synchronous error interrupt enable */
#define DCI_INTEN_VSIE            (1 << 3)   /* Bit 3:  Vsync interrupt enable */
#define DCI_INTEN_ELIE            (1 << 4)   /* Bit 4:  End of line interrupt enable */

/* DCI Interrupt Flag Register (DCI_INTF) */

#define DCI_INTF_EFIF             (1 << 0)   /* Bit 0:  End of frame int flag */
#define DCI_INTF_OVRIF            (1 << 1)   /* Bit 1:  FIFO overrun int flag */
#define DCI_INTF_ESEIF            (1 << 2)   /* Bit 2:  Embedded sync err int flag */
#define DCI_INTF_VSIF             (1 << 3)   /* Bit 3:  Vsync int flag */
#define DCI_INTF_ELIF             (1 << 4)   /* Bit 4:  End of line int flag */

/* DCI Interrupt Clear Register (DCI_INTC) */

#define DCI_INTC_EFFC             (1 << 0)   /* Bit 0:  Clear end of frame flag */
#define DCI_INTC_OVRFC            (1 << 1)   /* Bit 1:  Clear FIFO overrun flag */
#define DCI_INTC_ESEFC            (1 << 2)   /* Bit 2:  Clear embedded synchronous error flag */
#define DCI_INTC_VSFC             (1 << 3)   /* Bit 3:  Vsync flag clear */
#define DCI_INTC_ELFC             (1 << 4)   /* Bit 4:  End of line flag clear */

#define DCI_INTC_ALLFC            (DCI_INTC_EFFC | DCI_INTC_OVRFC | \
                                   DCI_INTC_ESEFC | DCI_INTC_VSFC | \
                                   DCI_INTC_ELFC)

/* DCI Synchronization Codes Register (DCI_SC) */

#define DCI_SC_FS_SHIFT           (0)        /* Bits 0-7:   Frame start code */
#define DCI_SC_FS_MASK            (0xff << DCI_SC_FS_SHIFT)
#define DCI_SC_FS(n)              ((uint32_t)(n) << DCI_SC_FS_SHIFT)
#define DCI_SC_LS_SHIFT           (8)        /* Bits 8-15:  Line start code */
#define DCI_SC_LS_MASK            (0xff << DCI_SC_LS_SHIFT)
#define DCI_SC_LS(n)              ((uint32_t)(n) << DCI_SC_LS_SHIFT)
#define DCI_SC_LE_SHIFT           (16)       /* Bits 16-23: Line end code */
#define DCI_SC_LE_MASK            (0xff << DCI_SC_LE_SHIFT)
#define DCI_SC_LE(n)              ((uint32_t)(n) << DCI_SC_LE_SHIFT)
#define DCI_SC_FE_SHIFT           (24)       /* Bits 24-31: Frame end code */
#define DCI_SC_FE_MASK            (0xff << DCI_SC_FE_SHIFT)
#define DCI_SC_FE(n)              ((uint32_t)(n) << DCI_SC_FE_SHIFT)

/* DCI Synchronization Codes Unmask Register (DCI_SCUMSK) */

#define DCI_SCUMSK_FSM_SHIFT      (0)        /* Bits 0-7:   Frame start code unmask */
#define DCI_SCUMSK_FSM_MASK       (0xff << DCI_SCUMSK_FSM_SHIFT)
#define DCI_SCUMSK_FSM(n)         ((uint32_t)(n) << DCI_SCUMSK_FSM_SHIFT)
#define DCI_SCUMSK_LSM_SHIFT      (8)        /* Bits 8-15:  Line start code unmask */
#define DCI_SCUMSK_LSM_MASK       (0xff << DCI_SCUMSK_LSM_SHIFT)
#define DCI_SCUMSK_LSM(n)         ((uint32_t)(n) << DCI_SCUMSK_LSM_SHIFT)
#define DCI_SCUMSK_LEM_SHIFT      (16)       /* Bits 16-23: Line end code unmask */
#define DCI_SCUMSK_LEM_MASK       (0xff << DCI_SCUMSK_LEM_SHIFT)
#define DCI_SCUMSK_LEM(n)         ((uint32_t)(n) << DCI_SCUMSK_LEM_SHIFT)
#define DCI_SCUMSK_FEM_SHIFT      (24)       /* Bits 24-31: Frame end code unmask */
#define DCI_SCUMSK_FEM_MASK       (0xff << DCI_SCUMSK_FEM_SHIFT)
#define DCI_SCUMSK_FEM(n)         ((uint32_t)(n) << DCI_SCUMSK_FEM_SHIFT)

/* DCI Cropping Window Start Position Register (DCI_CWSPOS) */

#define DCI_CWSPOS_WHSP_SHIFT    (0)        /* Bits 0-13:  Window horizontal start position */
#define DCI_CWSPOS_WHSP_MASK     (0x3fff << DCI_CWSPOS_WHSP_SHIFT)
#define DCI_CWSPOS_WHSP(n)       ((uint32_t)(n) << DCI_CWSPOS_WHSP_SHIFT)
#define DCI_CWSPOS_WVSP_SHIFT    (16)       /* Bits 16-28: Window vertical start position */
#define DCI_CWSPOS_WVSP_MASK     (0x1fff << DCI_CWSPOS_WVSP_SHIFT)
#define DCI_CWSPOS_WVSP(n)       ((uint32_t)(n) << DCI_CWSPOS_WVSP_SHIFT)

/* DCI Cropping Window Size Register (DCI_CWSZ) */

#define DCI_CWSZ_WHSZ_SHIFT      (0)        /* Bits 0-13:  Window horizontal size */
#define DCI_CWSZ_WHSZ_MASK       (0x3fff << DCI_CWSZ_WHSZ_SHIFT)
#define DCI_CWSZ_WHSZ(n)         ((uint32_t)(n) << DCI_CWSZ_WHSZ_SHIFT)
#define DCI_CWSZ_WVSZ_SHIFT      (16)       /* Bits 16-29: Window vertical size */
#define DCI_CWSZ_WVSZ_MASK       (0x3fff << DCI_CWSZ_WVSZ_SHIFT)
#define DCI_CWSZ_WVSZ(n)         ((uint32_t)(n) << DCI_CWSZ_WVSZ_SHIFT)

/* DCI Data Register (DCI_DATA) */

#define DCI_DATA_MASK             (0xffffffff)  /* Bits 0-31: Data */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_DCI_H */
