/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_usbfs.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_USBFS_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_USBFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware capabilities */

#define GD32_USBFS_NENDPOINTS           4   /* EP0-EP3 */
#define GD32_USBFS_NHOST_CHANNELS       8   /* Host channels 0-7 */

/* General endpoint type / PID definitions */

#define USBFS_EPTYPE_CTRL               (0) /* Control */
#define USBFS_EPTYPE_ISOC               (1) /* Isochronous */
#define USBFS_EPTYPE_BULK               (2) /* Bulk */
#define USBFS_EPTYPE_INTR               (3) /* Interrupt */
#define USBFS_PID_DATA0                 (0)
#define USBFS_PID_DATA2                 (1)
#define USBFS_PID_DATA1                 (2)
#define USBFS_PID_MDATA                 (3) /* Non-control */
#define USBFS_PID_SETUP                 (3) /* Control */

/****************************************************************************
 * Register Offsets
 ****************************************************************************/

/* USBFS_GLOBAL registers (base + 0x000) */

#define GD32_USBFS_GOTGCS_OFFSET       0x0000 /* OTG control and status */
#define GD32_USBFS_GOTGINTF_OFFSET     0x0004 /* OTG interrupt flag */
#define GD32_USBFS_GAHBCS_OFFSET       0x0008 /* AHB control and status */
#define GD32_USBFS_GUSBCS_OFFSET       0x000c /* USB control and status */
#define GD32_USBFS_GRSTCTL_OFFSET      0x0010 /* Global reset control */
#define GD32_USBFS_GINTF_OFFSET        0x0014 /* Global interrupt flag */
#define GD32_USBFS_GINTEN_OFFSET       0x0018 /* Global interrupt enable */
#define GD32_USBFS_GRSTATR_OFFSET      0x001c /* Receive status debug read */
#define GD32_USBFS_GRSTATP_OFFSET      0x0020 /* Receive status pop */
#define GD32_USBFS_GRFLEN_OFFSET       0x0024 /* Receive FIFO length */
#define GD32_USBFS_HNPTFLEN_OFFSET     0x0028 /* Host non-periodic TxFIFO length (host mode) */
#define GD32_USBFS_DIEP0TFLEN_OFFSET   0x0028 /* Device IN EP0 TxFIFO length (device mode) */
#define GD32_USBFS_HNPTFQSTAT_OFFSET   0x002c /* Host non-periodic TxFIFO/queue status */
#define GD32_USBFS_GCCFG_OFFSET        0x0038 /* Global core configuration */
#define GD32_USBFS_CID_OFFSET          0x003c /* Core ID */
#define GD32_USBFS_HPTFLEN_OFFSET      0x0100 /* Host periodic TxFIFO length */
#define GD32_USBFS_DIEP1TFLEN_OFFSET   0x0104 /* Device IN EP1 TxFIFO length */
#define GD32_USBFS_DIEP2TFLEN_OFFSET   0x0108 /* Device IN EP2 TxFIFO length */
#define GD32_USBFS_DIEP3TFLEN_OFFSET   0x010c /* Device IN EP3 TxFIFO length */

/* Parametric: device IN endpoint n TxFIFO length (n=1..3) */

#define GD32_USBFS_DIEPnTFLEN_OFFSET(n) (0x0100 + ((n) << 2))

/* USBFS_HOST registers (base + 0x400) */

#define GD32_USBFS_HCTL_OFFSET         0x0400 /* Host configuration */
#define GD32_USBFS_HFT_OFFSET          0x0404 /* Host frame interval */
#define GD32_USBFS_HFINFR_OFFSET       0x0408 /* Host frame information */
#define GD32_USBFS_HPTFQSTAT_OFFSET    0x0410 /* Host periodic TxFIFO/queue status */
#define GD32_USBFS_HACHINT_OFFSET      0x0414 /* Host all channels interrupt */
#define GD32_USBFS_HACHINTEN_OFFSET    0x0418 /* Host all channels interrupt enable */
#define GD32_USBFS_HPCS_OFFSET         0x0440 /* Host port control and status */

/* Host channel registers (per-channel, n=0..7, stride 0x20) */

#define GD32_USBFS_HCH_OFFSET(n)       (0x0500 + ((n) << 5))
#define GD32_USBFS_HCHCTL_CHOFFSET     0x0000 /* Channel characteristics */
#define GD32_USBFS_HCHINTF_CHOFFSET    0x0008 /* Channel interrupt flag */
#define GD32_USBFS_HCHINTEN_CHOFFSET   0x000c /* Channel interrupt enable */
#define GD32_USBFS_HCHLEN_CHOFFSET     0x0010 /* Channel transfer length */
#define GD32_USBFS_HCHCTL_OFFSET(n)    (0x0500 + ((n) << 5))
#define GD32_USBFS_HCHINTF_OFFSET(n)   (0x0508 + ((n) << 5))
#define GD32_USBFS_HCHINTEN_OFFSET(n)  (0x050c + ((n) << 5))
#define GD32_USBFS_HCHLEN_OFFSET(n)    (0x0510 + ((n) << 5))

/* USBFS_DEVICE registers (base + 0x800) */

#define GD32_USBFS_DCFG_OFFSET         0x0800 /* Device configuration */
#define GD32_USBFS_DCTL_OFFSET         0x0804 /* Device control */
#define GD32_USBFS_DSTAT_OFFSET        0x0808 /* Device status */
#define GD32_USBFS_DIEPINTEN_OFFSET    0x0810 /* Device IN endpoint common interrupt enable */
#define GD32_USBFS_DOEPINTEN_OFFSET    0x0814 /* Device OUT endpoint common interrupt enable */
#define GD32_USBFS_DAEPINT_OFFSET      0x0818 /* Device all endpoints interrupt */
#define GD32_USBFS_DAEPINTEN_OFFSET    0x081c /* Device all endpoints interrupt enable */
#define GD32_USBFS_DVBUSDT_OFFSET      0x0828 /* Device VBUS discharge time */
#define GD32_USBFS_DVBUSPT_OFFSET      0x082c /* Device VBUS pulsing time */
#define GD32_USBFS_DIEPFEINTEN_OFFSET  0x0834 /* Device IN endpoint FIFO empty interrupt enable */

/* Device IN endpoint registers (per-endpoint, stride 0x20) */

#define GD32_USBFS_DIEP_OFFSET(n)       (0x0900 + ((n) << 5))
#define GD32_USBFS_DIEPCTL_EPOFFSET     0x0000 /* IN endpoint control */
#define GD32_USBFS_DIEPINTF_EPOFFSET    0x0008 /* IN endpoint interrupt flag */
#define GD32_USBFS_DIEPLEN_EPOFFSET     0x0010 /* IN endpoint transfer length */
#define GD32_USBFS_DIEPTFSTAT_EPOFFSET  0x0018 /* IN endpoint TxFIFO status */
#define GD32_USBFS_DIEPCTL_OFFSET(n)    (0x0900 + ((n) << 5))
#define GD32_USBFS_DIEPINTF_OFFSET(n)   (0x0908 + ((n) << 5))
#define GD32_USBFS_DIEPLEN_OFFSET(n)    (0x0910 + ((n) << 5))
#define GD32_USBFS_DIEPTFSTAT_OFFSET(n) (0x0918 + ((n) << 5))

/* Device OUT endpoint registers (per-endpoint, stride 0x20) */

#define GD32_USBFS_DOEP_OFFSET(n)      (0x0b00 + ((n) << 5))
#define GD32_USBFS_DOEPCTL_EPOFFSET    0x0000 /* OUT endpoint control */
#define GD32_USBFS_DOEPINTF_EPOFFSET   0x0008 /* OUT endpoint interrupt flag */
#define GD32_USBFS_DOEPLEN_EPOFFSET    0x0010 /* OUT endpoint transfer length */
#define GD32_USBFS_DOEPCTL_OFFSET(n)   (0x0b00 + ((n) << 5))
#define GD32_USBFS_DOEPINTF_OFFSET(n)  (0x0b08 + ((n) << 5))
#define GD32_USBFS_DOEPLEN_OFFSET(n)   (0x0b10 + ((n) << 5))

/* USBFS_PWRCLK register (base + 0xE00) */

#define GD32_USBFS_PWRCLKCTL_OFFSET    0x0e00 /* Power and clock control */

/* Data FIFO access */

#define GD32_USBFS_DFIFO_DEP_OFFSET(n) (0x1000 + ((n) << 12))
#define GD32_USBFS_DFIFO_HCH_OFFSET(n) (0x1000 + ((n) << 12))

/****************************************************************************
 * Register Addresses
 ****************************************************************************/

/* USBFS_GLOBAL */

#define GD32_USBFS_GOTGCS              (GD32_USBFS_BASE + GD32_USBFS_GOTGCS_OFFSET)
#define GD32_USBFS_GOTGINTF            (GD32_USBFS_BASE + GD32_USBFS_GOTGINTF_OFFSET)
#define GD32_USBFS_GAHBCS              (GD32_USBFS_BASE + GD32_USBFS_GAHBCS_OFFSET)
#define GD32_USBFS_GUSBCS              (GD32_USBFS_BASE + GD32_USBFS_GUSBCS_OFFSET)
#define GD32_USBFS_GRSTCTL             (GD32_USBFS_BASE + GD32_USBFS_GRSTCTL_OFFSET)
#define GD32_USBFS_GINTF               (GD32_USBFS_BASE + GD32_USBFS_GINTF_OFFSET)
#define GD32_USBFS_GINTEN              (GD32_USBFS_BASE + GD32_USBFS_GINTEN_OFFSET)
#define GD32_USBFS_GRSTATR             (GD32_USBFS_BASE + GD32_USBFS_GRSTATR_OFFSET)
#define GD32_USBFS_GRSTATP             (GD32_USBFS_BASE + GD32_USBFS_GRSTATP_OFFSET)
#define GD32_USBFS_GRFLEN              (GD32_USBFS_BASE + GD32_USBFS_GRFLEN_OFFSET)
#define GD32_USBFS_HNPTFLEN            (GD32_USBFS_BASE + GD32_USBFS_HNPTFLEN_OFFSET)
#define GD32_USBFS_DIEP0TFLEN          (GD32_USBFS_BASE + GD32_USBFS_DIEP0TFLEN_OFFSET)
#define GD32_USBFS_HNPTFQSTAT          (GD32_USBFS_BASE + GD32_USBFS_HNPTFQSTAT_OFFSET)
#define GD32_USBFS_GCCFG               (GD32_USBFS_BASE + GD32_USBFS_GCCFG_OFFSET)
#define GD32_USBFS_CID                 (GD32_USBFS_BASE + GD32_USBFS_CID_OFFSET)
#define GD32_USBFS_HPTFLEN             (GD32_USBFS_BASE + GD32_USBFS_HPTFLEN_OFFSET)
#define GD32_USBFS_DIEPnTFLEN(n)       (GD32_USBFS_BASE + GD32_USBFS_DIEPnTFLEN_OFFSET(n))
#  define GD32_USBFS_DIEP1TFLEN        GD32_USBFS_DIEPnTFLEN(1)
#  define GD32_USBFS_DIEP2TFLEN        GD32_USBFS_DIEPnTFLEN(2)
#  define GD32_USBFS_DIEP3TFLEN        GD32_USBFS_DIEPnTFLEN(3)

/* USBFS_HOST */

#define GD32_USBFS_HCTL                (GD32_USBFS_BASE + GD32_USBFS_HCTL_OFFSET)
#define GD32_USBFS_HFT                 (GD32_USBFS_BASE + GD32_USBFS_HFT_OFFSET)
#define GD32_USBFS_HFINFR              (GD32_USBFS_BASE + GD32_USBFS_HFINFR_OFFSET)
#define GD32_USBFS_HPTFQSTAT           (GD32_USBFS_BASE + GD32_USBFS_HPTFQSTAT_OFFSET)
#define GD32_USBFS_HACHINT             (GD32_USBFS_BASE + GD32_USBFS_HACHINT_OFFSET)
#define GD32_USBFS_HACHINTEN           (GD32_USBFS_BASE + GD32_USBFS_HACHINTEN_OFFSET)
#define GD32_USBFS_HPCS                (GD32_USBFS_BASE + GD32_USBFS_HPCS_OFFSET)
#define GD32_USBFS_HCH(n)              (GD32_USBFS_BASE + GD32_USBFS_HCH_OFFSET(n))
#define GD32_USBFS_HCHCTL(n)           (GD32_USBFS_BASE + GD32_USBFS_HCHCTL_OFFSET(n))
#define GD32_USBFS_HCHINTF(n)          (GD32_USBFS_BASE + GD32_USBFS_HCHINTF_OFFSET(n))
#define GD32_USBFS_HCHINTEN(n)         (GD32_USBFS_BASE + GD32_USBFS_HCHINTEN_OFFSET(n))
#define GD32_USBFS_HCHLEN(n)           (GD32_USBFS_BASE + GD32_USBFS_HCHLEN_OFFSET(n))

/* USBFS_DEVICE */

#define GD32_USBFS_DCFG                (GD32_USBFS_BASE + GD32_USBFS_DCFG_OFFSET)
#define GD32_USBFS_DCTL                (GD32_USBFS_BASE + GD32_USBFS_DCTL_OFFSET)
#define GD32_USBFS_DSTAT               (GD32_USBFS_BASE + GD32_USBFS_DSTAT_OFFSET)
#define GD32_USBFS_DIEPINTEN           (GD32_USBFS_BASE + GD32_USBFS_DIEPINTEN_OFFSET)
#define GD32_USBFS_DOEPINTEN           (GD32_USBFS_BASE + GD32_USBFS_DOEPINTEN_OFFSET)
#define GD32_USBFS_DAEPINT             (GD32_USBFS_BASE + GD32_USBFS_DAEPINT_OFFSET)
#define GD32_USBFS_DAEPINTEN           (GD32_USBFS_BASE + GD32_USBFS_DAEPINTEN_OFFSET)
#define GD32_USBFS_DVBUSDT             (GD32_USBFS_BASE + GD32_USBFS_DVBUSDT_OFFSET)
#define GD32_USBFS_DVBUSPT             (GD32_USBFS_BASE + GD32_USBFS_DVBUSPT_OFFSET)
#define GD32_USBFS_DIEPFEINTEN         (GD32_USBFS_BASE + GD32_USBFS_DIEPFEINTEN_OFFSET)
#define GD32_USBFS_DIEP(n)             (GD32_USBFS_BASE + GD32_USBFS_DIEP_OFFSET(n))
#define GD32_USBFS_DIEPCTL(n)          (GD32_USBFS_BASE + GD32_USBFS_DIEPCTL_OFFSET(n))
#define GD32_USBFS_DIEPINTF(n)         (GD32_USBFS_BASE + GD32_USBFS_DIEPINTF_OFFSET(n))
#define GD32_USBFS_DIEPLEN(n)          (GD32_USBFS_BASE + GD32_USBFS_DIEPLEN_OFFSET(n))
#define GD32_USBFS_DIEPTFSTAT(n)       (GD32_USBFS_BASE + GD32_USBFS_DIEPTFSTAT_OFFSET(n))
#define GD32_USBFS_DOEP(n)             (GD32_USBFS_BASE + GD32_USBFS_DOEP_OFFSET(n))
#define GD32_USBFS_DOEPCTL(n)          (GD32_USBFS_BASE + GD32_USBFS_DOEPCTL_OFFSET(n))
#define GD32_USBFS_DOEPINTF(n)         (GD32_USBFS_BASE + GD32_USBFS_DOEPINTF_OFFSET(n))
#define GD32_USBFS_DOEPLEN(n)          (GD32_USBFS_BASE + GD32_USBFS_DOEPLEN_OFFSET(n))

/* USBFS_PWRCLK */

#define GD32_USBFS_PWRCLKCTL           (GD32_USBFS_BASE + GD32_USBFS_PWRCLKCTL_OFFSET)

/* Data FIFO */

#define GD32_USBFS_DFIFO_DEP(n)        (GD32_USBFS_BASE + GD32_USBFS_DFIFO_DEP_OFFSET(n))
#define GD32_USBFS_DFIFO_HCH(n)        (GD32_USBFS_BASE + GD32_USBFS_DFIFO_HCH_OFFSET(n))

/****************************************************************************
 * Register Bitfield Definitions
 ****************************************************************************/

/* GOTGCS - OTG control and status register */

#define USBFS_GOTGCS_SRPS               (1 << 0)  /* Bit 0:  SRP success (RO) */
#define USBFS_GOTGCS_SRPREQ             (1 << 1)  /* Bit 1:  SRP request */
#define USBFS_GOTGCS_HNPS               (1 << 8)  /* Bit 8:  HNP success (RO) */
#define USBFS_GOTGCS_HNPREQ             (1 << 9)  /* Bit 9:  HNP request */
#define USBFS_GOTGCS_HHNPEN             (1 << 10) /* Bit 10: Host HNP enable */
#define USBFS_GOTGCS_DHNPEN             (1 << 11) /* Bit 11: Device HNP enabled */
#define USBFS_GOTGCS_IDPS               (1 << 16) /* Bit 16: ID pin status (RO) */
#define USBFS_GOTGCS_DI                 (1 << 17) /* Bit 17: Debounce interval (RO) */
#define USBFS_GOTGCS_ASV                (1 << 18) /* Bit 18: A-session valid (RO) */
#define USBFS_GOTGCS_BSV                (1 << 19) /* Bit 19: B-session valid (RO) */

/* GOTGINTF - OTG interrupt flag register */

#define USBFS_GOTGINTF_SESEND           (1 << 2)  /* Bit 2:  Session end */
#define USBFS_GOTGINTF_SRPEND           (1 << 8)  /* Bit 8:  SRP success status change */
#define USBFS_GOTGINTF_HNPEND           (1 << 9)  /* Bit 9:  HNP end */
#define USBFS_GOTGINTF_HNPDET           (1 << 17) /* Bit 17: HNP request detected */
#define USBFS_GOTGINTF_ADTO             (1 << 18) /* Bit 18: A-device timeout */
#define USBFS_GOTGINTF_DF               (1 << 19) /* Bit 19: Debounce finish */

/* GAHBCS - AHB control and status register */

#define USBFS_GAHBCS_GINTEN             (1 << 0)  /* Bit 0:  Global interrupt enable */
#define USBFS_GAHBCS_TXFTH              (1 << 7)  /* Bit 7:  Tx FIFO threshold */
#define USBFS_GAHBCS_PTXFTH             (1 << 8)  /* Bit 8:  Periodic Tx FIFO threshold */

/* GUSBCS - USB control and status register */

#define USBFS_GUSBCS_TOC_SHIFT          (0)
#define USBFS_GUSBCS_TOC_MASK           (7 << USBFS_GUSBCS_TOC_SHIFT)
#define USBFS_GUSBCS_SRPCEN             (1 << 8)  /* Bit 8:  SRP capability enable */
#define USBFS_GUSBCS_HNPCEN             (1 << 9)  /* Bit 9:  HNP capability enable */
#define USBFS_GUSBCS_UTT_SHIFT          (10)
#define USBFS_GUSBCS_UTT_MASK           (15 << USBFS_GUSBCS_UTT_SHIFT)
#define USBFS_GUSBCS_UTT(n)             ((n) << USBFS_GUSBCS_UTT_SHIFT)
#define USBFS_GUSBCS_FHM                (1 << 29) /* Bit 29: Force host mode */
#define USBFS_GUSBCS_FDM                (1 << 30) /* Bit 30: Force device mode */

/* GRSTCTL - Global reset control register */

#define USBFS_GRSTCTL_CSRST             (1 << 0)  /* Bit 0:  Core soft reset */
#define USBFS_GRSTCTL_HCSRST            (1 << 1)  /* Bit 1:  HCLK soft reset */
#define USBFS_GRSTCTL_HFCRST            (1 << 2)  /* Bit 2:  Host frame counter reset */
#define USBFS_GRSTCTL_RXFF              (1 << 4)  /* Bit 4:  RxFIFO flush */
#define USBFS_GRSTCTL_TXFF              (1 << 5)  /* Bit 5:  TxFIFO flush */
#define USBFS_GRSTCTL_TXFNUM_SHIFT      (6)
#define USBFS_GRSTCTL_TXFNUM_MASK       (31 << USBFS_GRSTCTL_TXFNUM_SHIFT)
#define USBFS_GRSTCTL_TXFNUM_HNONPER    (0 << USBFS_GRSTCTL_TXFNUM_SHIFT)
#define USBFS_GRSTCTL_TXFNUM_HPER       (1 << USBFS_GRSTCTL_TXFNUM_SHIFT)
#define USBFS_GRSTCTL_TXFNUM_HALL       (16 << USBFS_GRSTCTL_TXFNUM_SHIFT)
#define USBFS_GRSTCTL_TXFNUM_D(n)       ((n) << USBFS_GRSTCTL_TXFNUM_SHIFT)
#define USBFS_GRSTCTL_TXFNUM_DALL       (16 << USBFS_GRSTCTL_TXFNUM_SHIFT)

/* GINTF - Global interrupt flag register
 * GINTEN - Global interrupt enable register
 * (same bit positions; GINTF has status, GINTEN has enables)
 */

#define USBFS_GINTF_COPM                (1 << 0)  /* Bit 0:  Current operation mode (RO) */
#define USBFS_GINTF_DEVMODE             (0)
#define USBFS_GINTF_HOSTMODE            (USBFS_GINTF_COPM)
#define USBFS_GINTF_MFIF                (1 << 1)  /* Bit 1:  Mode fault interrupt flag */
#define USBFS_GINTF_OTGIF               (1 << 2)  /* Bit 2:  OTG interrupt flag */
#define USBFS_GINTF_SOF                 (1 << 3)  /* Bit 3:  Start of frame */
#define USBFS_GINTF_RXFNEIF             (1 << 4)  /* Bit 4:  RxFIFO non-empty */
#define USBFS_GINTF_NPTXFEIF            (1 << 5)  /* Bit 5:  Non-periodic TxFIFO empty */
#define USBFS_GINTF_GNPINAK             (1 << 6)  /* Bit 6:  Global non-periodic IN NAK effective */
#define USBFS_GINTF_GONAK               (1 << 7)  /* Bit 7:  Global OUT NAK effective */
#define USBFS_GINTF_RES89               (3 << 8)  /* Reserved bits 8-9 mask */
#define USBFS_GINTF_ESP                 (1 << 10) /* Bit 10: Early suspend */
#define USBFS_GINTF_SP                  (1 << 11) /* Bit 11: USB suspend */
#define USBFS_GINTF_RST                 (1 << 12) /* Bit 12: USB reset */
#define USBFS_GINTF_ENUMF               (1 << 13) /* Bit 13: Enumeration finished */
#define USBFS_GINTF_ISOOPDIF            (1 << 14) /* Bit 14: Isochronous OUT dropped */
#define USBFS_GINTF_EOPFIF              (1 << 15) /* Bit 15: End of periodic frame */
#define USBFS_GINTF_RES1617             (3 << 16) /* Reserved bits 16-17 mask */
#define USBFS_GINTF_IEPIF               (1 << 18) /* Bit 18: IN endpoint interrupt */
#define USBFS_GINTF_OEPIF               (1 << 19) /* Bit 19: OUT endpoint interrupt */
#define USBFS_GINTF_ISOINCIF            (1 << 20) /* Bit 20: Isochronous IN not complete */
#define USBFS_GINTF_PXNCIF_ISOONCIF     (1 << 21) /* Bit 21: Periodic not complete / ISO OUT not complete */
#define USBFS_GINTF_RES22               (1 << 22) /* Reserved bit 22 mask */
#define USBFS_GINTF_HPIF                (1 << 24) /* Bit 24: Host port interrupt */
#define USBFS_GINTF_HCIF                (1 << 25) /* Bit 25: Host channels interrupt */
#define USBFS_GINTF_PTXFEIF             (1 << 26) /* Bit 26: Periodic TxFIFO empty */
#define USBFS_GINTF_IDPSC               (1 << 28) /* Bit 28: ID pin status change */
#define USBFS_GINTF_DISCIF              (1 << 29) /* Bit 29: Disconnect detected */
#define USBFS_GINTF_SESIF               (1 << 30) /* Bit 30: Session interrupt */
#define USBFS_GINTF_WKUPIF              (1 << 31) /* Bit 31: Wakeup interrupt */

/* GINTEN bit names (enable counterparts of GINTF; same bit positions) */

#define USBFS_GINTEN_MFIE               USBFS_GINTF_MFIF
#define USBFS_GINTEN_OTGIE              USBFS_GINTF_OTGIF
#define USBFS_GINTEN_SOFIE              USBFS_GINTF_SOF
#define USBFS_GINTEN_RXFNEIE            USBFS_GINTF_RXFNEIF
#define USBFS_GINTEN_NPTXFEIE           USBFS_GINTF_NPTXFEIF
#define USBFS_GINTEN_GNPINAKIE          USBFS_GINTF_GNPINAK
#define USBFS_GINTEN_GONAKIE            USBFS_GINTF_GONAK
#define USBFS_GINTEN_ESPIE              USBFS_GINTF_ESP
#define USBFS_GINTEN_SPIE               USBFS_GINTF_SP
#define USBFS_GINTEN_RSTIE              USBFS_GINTF_RST
#define USBFS_GINTEN_ENUMFIE            USBFS_GINTF_ENUMF
#define USBFS_GINTEN_ISOOPDIE           USBFS_GINTF_ISOOPDIF
#define USBFS_GINTEN_EOPFIE             USBFS_GINTF_EOPFIF
#define USBFS_GINTEN_IEPIE              USBFS_GINTF_IEPIF
#define USBFS_GINTEN_OEPIE              USBFS_GINTF_OEPIF
#define USBFS_GINTEN_ISOINCIE           USBFS_GINTF_ISOINCIF
#define USBFS_GINTEN_PXNCIE_ISOONCIE    USBFS_GINTF_PXNCIF_ISOONCIF
#define USBFS_GINTEN_HPIE               USBFS_GINTF_HPIF
#define USBFS_GINTEN_HCIE               USBFS_GINTF_HCIF
#define USBFS_GINTEN_PTXFEIE            USBFS_GINTF_PTXFEIF
#define USBFS_GINTEN_IDPSCIE            USBFS_GINTF_IDPSC
#define USBFS_GINTEN_DISCIE             USBFS_GINTF_DISCIF
#define USBFS_GINTEN_SESIE              USBFS_GINTF_SESIF
#define USBFS_GINTEN_WKUPIE             USBFS_GINTF_WKUPIF

/* GRSTATR / GRSTATP - Receive status registers (host mode) */
#define USBFS_GRXSTAH_CNUM_SHIFT        (0)
#define USBFS_GRXSTAH_CNUM_MASK         (15 << USBFS_GRXSTAH_CNUM_SHIFT)
#define USBFS_GRXSTAH_BCOUNT_SHIFT      (4)
#define USBFS_GRXSTAH_BCOUNT_MASK       (0x7ff << USBFS_GRXSTAH_BCOUNT_SHIFT)
#define USBFS_GRXSTAH_DPID_SHIFT        (15)
#define USBFS_GRXSTAH_DPID_MASK         (3 << USBFS_GRXSTAH_DPID_SHIFT)
#define USBFS_GRXSTAH_DPID_DATA0        (0 << USBFS_GRXSTAH_DPID_SHIFT)
#define USBFS_GRXSTAH_DPID_DATA2        (1 << USBFS_GRXSTAH_DPID_SHIFT)
#define USBFS_GRXSTAH_DPID_DATA1        (2 << USBFS_GRXSTAH_DPID_SHIFT)
#define USBFS_GRXSTAH_DPID_MDATA        (3 << USBFS_GRXSTAH_DPID_SHIFT)
#define USBFS_GRXSTAH_RPCKST_SHIFT      (17)
#define USBFS_GRXSTAH_RPCKST_MASK       (15 << USBFS_GRXSTAH_RPCKST_SHIFT)
#define USBFS_GRXSTAH_RPCKST_INRECVD    (2 << USBFS_GRXSTAH_RPCKST_SHIFT)
#define USBFS_GRXSTAH_RPCKST_INDONE     (3 << USBFS_GRXSTAH_RPCKST_SHIFT)
#define USBFS_GRXSTAH_RPCKST_DTOGERR    (5 << USBFS_GRXSTAH_RPCKST_SHIFT)
#define USBFS_GRXSTAH_RPCKST_HALTED     (7 << USBFS_GRXSTAH_RPCKST_SHIFT)

/* GRSTATR / GRSTATP - Receive status registers (device mode) */

#define USBFS_GRXSTAD_EPNUM_SHIFT       (0)
#define USBFS_GRXSTAD_EPNUM_MASK        (15 << USBFS_GRXSTAD_EPNUM_SHIFT)
#define USBFS_GRXSTAD_BCOUNT_SHIFT      (4)
#define USBFS_GRXSTAD_BCOUNT_MASK       (0x7ff << USBFS_GRXSTAD_BCOUNT_SHIFT)
#define USBFS_GRXSTAD_DPID_SHIFT        (15)
#define USBFS_GRXSTAD_DPID_MASK         (3 << USBFS_GRXSTAD_DPID_SHIFT)
#define USBFS_GRXSTAD_DPID_DATA0        (0 << USBFS_GRXSTAD_DPID_SHIFT)
#define USBFS_GRXSTAD_DPID_DATA2        (1 << USBFS_GRXSTAD_DPID_SHIFT)
#define USBFS_GRXSTAD_DPID_DATA1        (2 << USBFS_GRXSTAD_DPID_SHIFT)
#define USBFS_GRXSTAD_DPID_MDATA        (3 << USBFS_GRXSTAD_DPID_SHIFT)
#define USBFS_GRXSTAD_RPCKST_SHIFT      (17)
#define USBFS_GRXSTAD_RPCKST_MASK       (15 << USBFS_GRXSTAD_RPCKST_SHIFT)
#define USBFS_GRXSTAD_RPCKST_OUTNAK     (1 << USBFS_GRXSTAD_RPCKST_SHIFT)
#define USBFS_GRXSTAD_RPCKST_OUTRECVD   (2 << USBFS_GRXSTAD_RPCKST_SHIFT)
#define USBFS_GRXSTAD_RPCKST_OUTDONE    (3 << USBFS_GRXSTAD_RPCKST_SHIFT)
#define USBFS_GRXSTAD_RPCKST_SETUPDONE  (4 << USBFS_GRXSTAD_RPCKST_SHIFT)
#define USBFS_GRXSTAD_RPCKST_SETUPRECVD (6 << USBFS_GRXSTAD_RPCKST_SHIFT)

/* GRFLEN - Receive FIFO length */

#define USBFS_GRFLEN_RXFD_MASK          (0xffff)

/* HNPTFLEN - Host non-periodic TxFIFO length (host mode)
 * DIEP0TFLEN - Device IN EP0 TxFIFO length (device mode) [same offset]
 */

#define USBFS_HNPTFLEN_HNPTXRSAR_SHIFT    (0)
#define USBFS_HNPTFLEN_HNPTXRSAR_MASK     (0xffff << USBFS_HNPTFLEN_HNPTXRSAR_SHIFT)
#define USBFS_HNPTFLEN_HNPTXFD_SHIFT      (16)
#define USBFS_HNPTFLEN_HNPTXFD_MASK       (0xffff << USBFS_HNPTFLEN_HNPTXFD_SHIFT)
#define USBFS_HNPTFLEN_HNPTXFD_MIN        (16 << USBFS_HNPTFLEN_HNPTXFD_SHIFT)
#define USBFS_HNPTFLEN_HNPTXFD_MAX        (256 << USBFS_HNPTFLEN_HNPTXFD_SHIFT)
#define USBFS_DIEP0TFLEN_IEP0TXRSAR_SHIFT (0)
#define USBFS_DIEP0TFLEN_IEP0TXRSAR_MASK  (0xffff << USBFS_DIEP0TFLEN_IEP0TXRSAR_SHIFT)
#define USBFS_DIEP0TFLEN_IEP0TXFD_SHIFT   (16)
#define USBFS_DIEP0TFLEN_IEP0TXFD_MASK    (0xffff << USBFS_DIEP0TFLEN_IEP0TXFD_SHIFT)
#define USBFS_DIEP0TFLEN_IEP0TXFD_MIN     (16 << USBFS_DIEP0TFLEN_IEP0TXFD_SHIFT)
#define USBFS_DIEP0TFLEN_IEP0TXFD_MAX     (256 << USBFS_DIEP0TFLEN_IEP0TXFD_SHIFT)

/* HNPTFQSTAT - Host non-periodic TxFIFO/queue status */

#define USBFS_HNPTFQSTAT_NPTXFS_SHIFT    (0)
#define USBFS_HNPTFQSTAT_NPTXFS_MASK     (0xffff << USBFS_HNPTFQSTAT_NPTXFS_SHIFT)
#define USBFS_HNPTFQSTAT_NPTXFS_FULL     (0 << USBFS_HNPTFQSTAT_NPTXFS_SHIFT)
#define USBFS_HNPTFQSTAT_NPTXRQS_SHIFT   (16)
#define USBFS_HNPTFQSTAT_NPTXRQS_MASK    (0xff << USBFS_HNPTFQSTAT_NPTXRQS_SHIFT)
#define USBFS_HNPTFQSTAT_NPTXRQS_FULL    (0 << USBFS_HNPTFQSTAT_NPTXRQS_SHIFT)
#define USBFS_HNPTFQSTAT_NPTXRQTOP_SHIFT (24)
#define USBFS_HNPTFQSTAT_NPTXRQTOP_MASK  (0x7f << USBFS_HNPTFQSTAT_NPTXRQTOP_SHIFT)
#define USBFS_HNPTFQSTAT_TERMINATE       (1 << 24)
#define USBFS_HNPTFQSTAT_TYPE_SHIFT      (25)
#define USBFS_HNPTFQSTAT_TYPE_MASK       (3 << USBFS_HNPTFQSTAT_TYPE_SHIFT)
#define USBFS_HNPTFQSTAT_TYPE_INOUT      (0 << USBFS_HNPTFQSTAT_TYPE_SHIFT)
#define USBFS_HNPTFQSTAT_TYPE_ZLP        (1 << USBFS_HNPTFQSTAT_TYPE_SHIFT)
#define USBFS_HNPTFQSTAT_TYPE_HALT       (3 << USBFS_HNPTFQSTAT_TYPE_SHIFT)
#define USBFS_HNPTFQSTAT_CNUM_SHIFT      (27)
#define USBFS_HNPTFQSTAT_CNUM_MASK       (15 << USBFS_HNPTFQSTAT_CNUM_SHIFT)
#define USBFS_HNPTFQSTAT_EPNUM_SHIFT     (27)
#define USBFS_HNPTFQSTAT_EPNUM_MASK      (15 << USBFS_HNPTFQSTAT_EPNUM_SHIFT)

/* GCCFG - Global core configuration register
 *
 * NOTE: GD32E11x uses traditional VBUS comparator model (no BCD detection).
 */

#define USBFS_GCCFG_PWRON                (1 << 16) /* Bit 16: Power on */
#define USBFS_GCCFG_VBUSACEN             (1 << 18) /* Bit 18: VBUS A-device comparator enable */
#define USBFS_GCCFG_VBUSBCEN             (1 << 19) /* Bit 19: VBUS B-device comparator enable */
#define USBFS_GCCFG_SOFOEN               (1 << 20) /* Bit 20: SOF output enable */
#define USBFS_GCCFG_VBUSIG               (1 << 21) /* Bit 21: VBUS ignored */

/* HPTFLEN - Host periodic TxFIFO length */

#define USBFS_HPTFLEN_HPTXFSAR_SHIFT     (0)
#define USBFS_HPTFLEN_HPTXFSAR_MASK      (0xffff << USBFS_HPTFLEN_HPTXFSAR_SHIFT)
#define USBFS_HPTFLEN_HPTXFD_SHIFT       (16)
#define USBFS_HPTFLEN_HPTXFD_MASK        (0xffff << USBFS_HPTFLEN_HPTXFD_SHIFT)

/* DIEPnTFLEN (n=1..3) - Device IN endpoint n TxFIFO length */

#define USBFS_DIEPnTFLEN_IEPTXRSAR_SHIFT (0)
#define USBFS_DIEPnTFLEN_IEPTXRSAR_MASK  (0xffff << USBFS_DIEPnTFLEN_IEPTXRSAR_SHIFT)
#define USBFS_DIEPnTFLEN_IEPTXFD_SHIFT   (16)
#define USBFS_DIEPnTFLEN_IEPTXFD_MASK    (0xffff << USBFS_DIEPnTFLEN_IEPTXFD_SHIFT)

/* HCTL - Host configuration register */

#define USBFS_HCTL_CLKSEL_SHIFT          (0)
#define USBFS_HCTL_CLKSEL_MASK           (3 << USBFS_HCTL_CLKSEL_SHIFT)
#define USBFS_HCTL_CLKSEL_FS48MHz        (1 << USBFS_HCTL_CLKSEL_SHIFT)
#define USBFS_HCTL_CLKSEL_LS48MHz        (1 << USBFS_HCTL_CLKSEL_SHIFT)
#define USBFS_HCTL_CLKSEL_LS6MHz         (2 << USBFS_HCTL_CLKSEL_SHIFT)

/* HFT - Host frame interval register */
#define USBFS_HFT_FRI_MASK               (0xffff)

/* HFINFR - Host frame number/frame time remaining */
#define USBFS_HFINFR_FRNUM_SHIFT         (0)
#define USBFS_HFINFR_FRNUM_MASK          (0xffff << USBFS_HFINFR_FRNUM_SHIFT)
#define USBFS_HFINFR_FRT_SHIFT           (16)
#define USBFS_HFINFR_FRT_MASK            (0xffff << USBFS_HFINFR_FRT_SHIFT)

/* HPTFQSTAT - Host periodic TxFIFO/queue status */

#define USBFS_HPTFQSTAT_PTXFS_SHIFT      (0)
#define USBFS_HPTFQSTAT_PTXFS_MASK       (0xffff << USBFS_HPTFQSTAT_PTXFS_SHIFT)
#define USBFS_HPTFQSTAT_PTXFS_FULL       (0 << USBFS_HPTFQSTAT_PTXFS_SHIFT)
#define USBFS_HPTFQSTAT_PTXREQS_SHIFT    (16)
#define USBFS_HPTFQSTAT_PTXREQS_MASK     (0xff << USBFS_HPTFQSTAT_PTXREQS_SHIFT)
#define USBFS_HPTFQSTAT_PTXREQS_FULL     (0 << USBFS_HPTFQSTAT_PTXREQS_SHIFT)
#define USBFS_HPTFQSTAT_PTXREQT_SHIFT    (24)
#define USBFS_HPTFQSTAT_PTXREQT_MASK     (0xff << USBFS_HPTFQSTAT_PTXREQT_SHIFT)
#define USBFS_HPTFQSTAT_TERMINATE        (1 << 24)
#define USBFS_HPTFQSTAT_TYPE_SHIFT       (25)
#define USBFS_HPTFQSTAT_TYPE_MASK        (3 << USBFS_HPTFQSTAT_TYPE_SHIFT)
#define USBFS_HPTFQSTAT_TYPE_INOUT       (0 << USBFS_HPTFQSTAT_TYPE_SHIFT)
#define USBFS_HPTFQSTAT_TYPE_ZLP         (1 << USBFS_HPTFQSTAT_TYPE_SHIFT)
#define USBFS_HPTFQSTAT_TYPE_HALT        (3 << USBFS_HPTFQSTAT_TYPE_SHIFT)
#define USBFS_HPTFQSTAT_EPNUM_SHIFT      (27)
#define USBFS_HPTFQSTAT_EPNUM_MASK       (15 << USBFS_HPTFQSTAT_EPNUM_SHIFT)
#define USBFS_HPTFQSTAT_CNUM_SHIFT       (27)
#define USBFS_HPTFQSTAT_CNUM_MASK        (15 << USBFS_HPTFQSTAT_CNUM_SHIFT)
#define USBFS_HPTFQSTAT_ODD              (1 << 31)

/* HACHINT / HACHINTEN - Host all channels interrupt */

#define USBFS_HACHINT(n)                (1 << (n))

/* HPCS - Host port control and status register */

#define USBFS_HPCS_PCST                 (1 << 0)  /* Bit 0:  Port connect status */
#define USBFS_HPCS_PCD                  (1 << 1)  /* Bit 1:  Port connect detected */
#define USBFS_HPCS_PE                   (1 << 2)  /* Bit 2:  Port enable */
#define USBFS_HPCS_PEDC                 (1 << 3)  /* Bit 3:  Port enable/disable change */
                                                  /* Bits 4-5: Reserved (DWC2 overcurrent, not in GD32E11x) */
#define USBFS_HPCS_PREM                 (1 << 6)  /* Bit 6:  Port resume */
#define USBFS_HPCS_PSP                  (1 << 7)  /* Bit 7:  Port suspend */
#define USBFS_HPCS_PRST                 (1 << 8)  /* Bit 8:  Port reset */
#define USBFS_HPCS_PLST_SHIFT           (10)
#define USBFS_HPCS_PLST_MASK            (3 << USBFS_HPCS_PLST_SHIFT)
#define USBFS_HPCS_PLST_DP              (1 << 10)
#define USBFS_HPCS_PLST_DM              (1 << 11)
#define USBFS_HPCS_PP                   (1 << 12) /* Bit 12: Port power */
#define USBFS_HPCS_PS_SHIFT             (17)
#define USBFS_HPCS_PS_MASK              (3 << USBFS_HPCS_PS_SHIFT)
#define USBFS_HPCS_PS_FS                (1 << USBFS_HPCS_PS_SHIFT)
#define USBFS_HPCS_PS_LS                (2 << USBFS_HPCS_PS_SHIFT)

/* HCHnCTL - Host channel-n characteristics register */

#define USBFS_HCHCTL_MPL_SHIFT          (0)
#define USBFS_HCHCTL_MPL_MASK           (0x7ff << USBFS_HCHCTL_MPL_SHIFT)
#define USBFS_HCHCTL_EPNUM_SHIFT        (11)
#define USBFS_HCHCTL_EPNUM_MASK         (15 << USBFS_HCHCTL_EPNUM_SHIFT)
#define USBFS_HCHCTL_EPDIR              (1 << 15) /* Bit 15: Endpoint direction */
#define USBFS_HCHCTL_EPDIR_OUT          (0)
#define USBFS_HCHCTL_EPDIR_IN           USBFS_HCHCTL_EPDIR
#define USBFS_HCHCTL_LSD                (1 << 17) /* Bit 17: Low-speed device */
#define USBFS_HCHCTL_EPTYPE_SHIFT       (18)
#define USBFS_HCHCTL_EPTYPE_MASK        (3 << USBFS_HCHCTL_EPTYPE_SHIFT)
#define USBFS_HCHCTL_EPTYPE_CTRL        (0 << USBFS_HCHCTL_EPTYPE_SHIFT)
#define USBFS_HCHCTL_EPTYPE_ISOC        (1 << USBFS_HCHCTL_EPTYPE_SHIFT)
#define USBFS_HCHCTL_EPTYPE_BULK        (2 << USBFS_HCHCTL_EPTYPE_SHIFT)
#define USBFS_HCHCTL_EPTYPE_INTR        (3 << USBFS_HCHCTL_EPTYPE_SHIFT)
#define USBFS_HCHCTL_DAR_SHIFT          (22)
#define USBFS_HCHCTL_DAR_MASK           (0x7f << USBFS_HCHCTL_DAR_SHIFT)
#define USBFS_HCHCTL_ODDFRM             (1 << 29) /* Bit 29: Odd frame */
#define USBFS_HCHCTL_CDIS               (1 << 30) /* Bit 30: Channel disable */
#define USBFS_HCHCTL_CEN                (1 << 31) /* Bit 31: Channel enable */

/* HCHnINTF - Host channel-n interrupt flag register
 * HCHnINTEN - Host channel-n interrupt enable register
 */

#define USBFS_HCHINTF_TF                (1 << 0)  /* Bit 0:  Transfer finished */
#define USBFS_HCHINTF_CH                (1 << 1)  /* Bit 1:  Channel halted */
#define USBFS_HCHINTF_STALL             (1 << 3)  /* Bit 3:  STALL response */
#define USBFS_HCHINTF_NAK               (1 << 4)  /* Bit 4:  NAK response */
#define USBFS_HCHINTF_ACK               (1 << 5)  /* Bit 5:  ACK response */
#define USBFS_HCHINTF_USBER             (1 << 7)  /* Bit 7:  USB bus error */
#define USBFS_HCHINTF_BBER              (1 << 8)  /* Bit 8:  Babble error */
#define USBFS_HCHINTF_REQOVR            (1 << 9)  /* Bit 9:  Request queue overrun */
#define USBFS_HCHINTF_DTER              (1 << 10) /* Bit 10: Data toggle error */

/* HCHnLEN - Host channel-n transfer length register */

#define USBFS_HCHLEN_TLEN_SHIFT         (0)
#define USBFS_HCHLEN_TLEN_MASK          (0x7ffff << USBFS_HCHLEN_TLEN_SHIFT)
#define USBFS_HCHLEN_PCNT_SHIFT         (19)
#define USBFS_HCHLEN_PCNT_MASK          (0x3ff << USBFS_HCHLEN_PCNT_SHIFT)
#define USBFS_HCHLEN_DPID_SHIFT         (29)
#define USBFS_HCHLEN_DPID_MASK          (3 << USBFS_HCHLEN_DPID_SHIFT)
#define USBFS_HCHLEN_DPID_DATA0         (0 << USBFS_HCHLEN_DPID_SHIFT)
#define USBFS_HCHLEN_DPID_DATA2         (1 << USBFS_HCHLEN_DPID_SHIFT)
#define USBFS_HCHLEN_DPID_DATA1         (2 << USBFS_HCHLEN_DPID_SHIFT)
#define USBFS_HCHLEN_DPID_MDATA         (3 << USBFS_HCHLEN_DPID_SHIFT)
#define USBFS_HCHLEN_PID_SETUP          (3 << USBFS_HCHLEN_DPID_SHIFT)

/* DCFG - Device configuration register */
#define USBFS_DCFG_DS_SHIFT             (0)
#define USBFS_DCFG_DS_MASK              (3 << USBFS_DCFG_DS_SHIFT)
#define USBFS_DCFG_DS_FS                (3 << USBFS_DCFG_DS_SHIFT)
#define USBFS_DCFG_NZLSOH               (1 << 2)
#define USBFS_DCFG_DAR_SHIFT            (4)
#define USBFS_DCFG_DAR_MASK             (0x7f << USBFS_DCFG_DAR_SHIFT)
#define USBFS_DCFG_EOPFT_SHIFT          (11)
#define USBFS_DCFG_EOPFT_MASK           (3 << USBFS_DCFG_EOPFT_SHIFT)
#define USBFS_DCFG_EOPFT_80PCT          (0 << USBFS_DCFG_EOPFT_SHIFT)
#define USBFS_DCFG_EOPFT_85PCT          (1 << USBFS_DCFG_EOPFT_SHIFT)
#define USBFS_DCFG_EOPFT_90PCT          (2 << USBFS_DCFG_EOPFT_SHIFT)
#define USBFS_DCFG_EOPFT_95PCT          (3 << USBFS_DCFG_EOPFT_SHIFT)

/* DCTL - Device control register */

#define USBFS_DCTL_RWKUP                (1 << 0)  /* Bit 0:  Remote wakeup signaling */
#define USBFS_DCTL_SD                   (1 << 1)  /* Bit 1:  Soft disconnect */
#define USBFS_DCTL_GINS                 (1 << 2)  /* Bit 2:  Global IN NAK status */
#define USBFS_DCTL_GONS                 (1 << 3)  /* Bit 3:  Global OUT NAK status */
#define USBFS_DCTL_SGINAK               (1 << 7)  /* Bit 7:  Set global IN NAK */
#define USBFS_DCTL_CGINAK               (1 << 8)  /* Bit 8:  Clear global IN NAK */
#define USBFS_DCTL_SGONAK               (1 << 9)  /* Bit 9:  Set global OUT NAK */
#define USBFS_DCTL_CGONAK               (1 << 10) /* Bit 10: Clear global OUT NAK */
#define USBFS_DCTL_POIF                 (1 << 11) /* Bit 11: Power-on initialization finished */

/* DSTAT - Device status register */

#define USBFS_DSTAT_SPST                (1 << 0)  /* Bit 0:  Suspend status */
#define USBFS_DSTAT_ES_SHIFT            (1)
#define USBFS_DSTAT_ES_MASK             (3 << USBFS_DSTAT_ES_SHIFT)
#define USBFS_DSTAT_ES_FS               (3 << USBFS_DSTAT_ES_SHIFT)
#define USBFS_DSTAT_FNRSOF_SHIFT        (8)
#define USBFS_DSTAT_FNRSOF_MASK         (0x3fff << USBFS_DSTAT_FNRSOF_SHIFT)
#define USBFS_DSTAT_FNRSOF0             (1 << 8)
#define USBFS_DSTAT_FNRSOF_EVEN         0
#define USBFS_DSTAT_FNRSOF_ODD          USBFS_DSTAT_FNRSOF0

/* DIEPINTEN - Device IN endpoint common interrupt enable
 * DOEPINTEN - Device OUT endpoint common interrupt enable
 */

#define USBFS_DIEPINTEN_TFEN            (1 << 0)  /* Bit 0:  Transfer finished enable */
#define USBFS_DIEPINTEN_EPDISEN         (1 << 1)  /* Bit 1:  Endpoint disabled enable */
#define USBFS_DIEPINTEN_CITOEN          (1 << 3)  /* Bit 3:  Timeout condition enable */
#define USBFS_DIEPINTEN_EPTXFUDEN       (1 << 4)  /* Bit 4:  IN token received with TxFIFO empty */
#define USBFS_DIEPINTEN_IEPNEEN         (1 << 6)  /* Bit 6:  IN endpoint NAK effective */
#define USBFS_DOEPINTEN_TFEN            (1 << 0)  /* Bit 0:  Transfer finished enable */
#define USBFS_DOEPINTEN_EPDISEN         (1 << 1)  /* Bit 1:  Endpoint disabled enable */
#define USBFS_DOEPINTEN_STPFEN          (1 << 3)  /* Bit 3:  SETUP phase done enable */
#define USBFS_DOEPINTEN_EPRXFOVREN      (1 << 4)  /* Bit 4:  Endpoint Rx FIFO overflow enable */
#define USBFS_DOEPINTEN_BTBSTPEN        (1 << 6)  /* Bit 6:  Back-to-back SETUP packets received */

/* DAEPINT - Device all endpoints interrupt
 * DAEPINTEN - Device all endpoints interrupt enable
 */
#define USBFS_DAEPINT_IEP_SHIFT         (0)
#define USBFS_DAEPINT_IEP_MASK          (0xffff << USBFS_DAEPINT_IEP_SHIFT)
#define USBFS_DAEPINT_IEP(n)            (1 << (n))
#define USBFS_DAEPINT_OEP_SHIFT         (16)
#define USBFS_DAEPINT_OEP_MASK          (0xffff << USBFS_DAEPINT_OEP_SHIFT)
#define USBFS_DAEPINT_OEP(n)            (1 << ((n) + 16))

/* DVBUSDT - Device VBUS discharge time */
#define USBFS_DVBUSDT_MASK              (0xffff)

/* DVBUSPT - Device VBUS pulsing time */
#define USBFS_DVBUSPT_MASK              (0xfff)

/* DIEPFEINTEN - Device IN endpoint FIFO empty interrupt enable */
#define USBFS_DIEPFEINTEN(n)            (1 << (n))

/* DIEPnCTL - Device control IN endpoint n control register (n=0) */
#define USBFS_DIEP0CTL_MPL_SHIFT        (0)
#define USBFS_DIEP0CTL_MPL_MASK         (3 << USBFS_DIEP0CTL_MPL_SHIFT)
#define USBFS_DIEP0CTL_MPL_64           (0 << USBFS_DIEP0CTL_MPL_SHIFT)
#define USBFS_DIEP0CTL_MPL_32           (1 << USBFS_DIEP0CTL_MPL_SHIFT)
#define USBFS_DIEP0CTL_MPL_16           (2 << USBFS_DIEP0CTL_MPL_SHIFT)
#define USBFS_DIEP0CTL_MPL_8            (3 << USBFS_DIEP0CTL_MPL_SHIFT)
#define USBFS_DIEP0CTL_EPACT            (1 << 15) /* Bit 15: Endpoint active (RO) */
#define USBFS_DIEP0CTL_NAKS             (1 << 17) /* Bit 17: NAK status */
#define USBFS_DIEP0CTL_EPTYPE_SHIFT     (18)
#define USBFS_DIEP0CTL_EPTYPE_MASK      (3 << USBFS_DIEP0CTL_EPTYPE_SHIFT)
#define USBFS_DIEP0CTL_EPTYPE_CTRL      (0 << USBFS_DIEP0CTL_EPTYPE_SHIFT)
#define USBFS_DIEP0CTL_STALL            (1 << 21) /* Bit 21: STALL handshake */
#define USBFS_DIEP0CTL_TXFNUM_SHIFT     (22)
#define USBFS_DIEP0CTL_TXFNUM_MASK      (15 << USBFS_DIEP0CTL_TXFNUM_SHIFT)
#define USBFS_DIEP0CTL_CNAK             (1 << 26) /* Bit 26: Clear NAK */
#define USBFS_DIEP0CTL_SNAK             (1 << 27) /* Bit 27: Set NAK */
#define USBFS_DIEP0CTL_EPD              (1 << 30) /* Bit 30: Endpoint disable */
#define USBFS_DIEP0CTL_EPEN             (1 << 31) /* Bit 31: Endpoint enable */

/* DIEPnCTL - Device control IN endpoint n control register (n>=1) */

#define USBFS_DIEPCTL_MPL_SHIFT         (0)
#define USBFS_DIEPCTL_MPL_MASK          (0x7ff << USBFS_DIEPCTL_MPL_SHIFT)
#define USBFS_DIEPCTL_EPACT             (1 << 15) /* Bit 15: Endpoint active */
#define USBFS_DIEPCTL_EOFRM_DPID        (1 << 16) /* Bit 16: Endpoint data PID */
#define USBFS_DIEPCTL_DPID_EVEN         (0)
#define USBFS_DIEPCTL_DPID_ODD          USBFS_DIEPCTL_EOFRM_DPID
#define USBFS_DIEPCTL_DPID_DATA0        (0)
#define USBFS_DIEPCTL_DPID_DATA1        USBFS_DIEPCTL_EOFRM_DPID
#define USBFS_DIEPCTL_NAKS              (1 << 17) /* Bit 17: NAK status */
#define USBFS_DIEPCTL_EPTYPE_SHIFT      (18)
#define USBFS_DIEPCTL_EPTYPE_MASK       (3 << USBFS_DIEPCTL_EPTYPE_SHIFT)
#define USBFS_DIEPCTL_EPTYPE_CTRL       (0 << USBFS_DIEPCTL_EPTYPE_SHIFT)
#define USBFS_DIEPCTL_EPTYPE_ISOC       (1 << USBFS_DIEPCTL_EPTYPE_SHIFT)
#define USBFS_DIEPCTL_EPTYPE_BULK       (2 << USBFS_DIEPCTL_EPTYPE_SHIFT)
#define USBFS_DIEPCTL_EPTYPE_INTR       (3 << USBFS_DIEPCTL_EPTYPE_SHIFT)
#define USBFS_DIEPCTL_STALL             (1 << 21) /* Bit 21: STALL handshake */
#define USBFS_DIEPCTL_TXFNUM_SHIFT      (22)
#define USBFS_DIEPCTL_TXFNUM_MASK       (15 << USBFS_DIEPCTL_TXFNUM_SHIFT)
#define USBFS_DIEPCTL_CNAK              (1 << 26) /* Bit 26: Clear NAK */
#define USBFS_DIEPCTL_SNAK              (1 << 27) /* Bit 27: Set NAK */
#define USBFS_DIEPCTL_SD0PID            (1 << 28) /* Bit 28: Set DATA0 PID */
#define USBFS_DIEPCTL_SEVFRM            (1 << 28) /* Bit 28: Set even frame */
#define USBFS_DIEPCTL_SODDFRM           (1 << 29) /* Bit 29: Set odd frame */
#define USBFS_DIEPCTL_SD1PID            (1 << 29) /* Bit 29: Set DATA1 PID */
#define USBFS_DIEPCTL_EPD               (1 << 30) /* Bit 30: Endpoint disable */
#define USBFS_DIEPCTL_EPEN              (1 << 31) /* Bit 31: Endpoint enable */

/* DIEPnINTF - Device IN endpoint n interrupt flag register
 * DOEPnINTF - Device OUT endpoint n interrupt flag register
 */

#define USBFS_DIEPINTF_TF               (1 << 0)  /* Bit 0:  Transfer finished */
#define USBFS_DIEPINTF_EPDIS            (1 << 1)  /* Bit 1:  Endpoint disabled */
#define USBFS_DIEPINTF_CITO             (1 << 3)  /* Bit 3:  Control IN timeout */
#define USBFS_DIEPINTF_EPTXFUD          (1 << 4)  /* Bit 4:  IN token received with TxFIFO empty */
#define USBFS_DIEPINTF_IEPNE            (1 << 6)  /* Bit 6:  IN endpoint NAK effective */
#define USBFS_DIEPINTF_TXFE             (1 << 7)  /* Bit 7:  TxFIFO empty */
#define USBFS_DOEPINTF_TF               (1 << 0)  /* Bit 0:  Transfer finished */
#define USBFS_DOEPINTF_EPDIS            (1 << 1)  /* Bit 1:  Endpoint disabled */
#define USBFS_DOEPINTF_STPF             (1 << 3)  /* Bit 3:  SETUP phase done */
#define USBFS_DOEPINTF_EPRXFOVR         (1 << 4)  /* Bit 4:  Endpoint Rx FIFO overflow */
#define USBFS_DOEPINTF_BTBSTP           (1 << 6)  /* Bit 6:  Back-to-back SETUP packets */

/* DIEPnLEN - Device IN endpoint n transfer length */

#define USBFS_DIEPLEN_TLEN_SHIFT        (0)
#define USBFS_DIEPLEN_TLEN_MASK         (0x7ffff << USBFS_DIEPLEN_TLEN_SHIFT)
#define USBFS_DIEPLEN_PCNT_SHIFT        (19)
#define USBFS_DIEPLEN_PCNT_MASK         (0x3ff << USBFS_DIEPLEN_PCNT_SHIFT)
#define USBFS_DIEPLEN_MCPF_SHIFT        (29)
#define USBFS_DIEPLEN_MCPF_MASK         (3 << USBFS_DIEPLEN_MCPF_SHIFT)

/* DIEP0LEN - Device IN endpoint 0 transfer length (EP0 has smaller TLEN) */
#define USBFS_DIEP0LEN_TLEN_MASK        (0x7f)
#define USBFS_DIEP0LEN_PCNT_SHIFT       (19)
#define USBFS_DIEP0LEN_PCNT_MASK        (3 << USBFS_DIEP0LEN_PCNT_SHIFT)

/* DOEPnLEN - Device OUT endpoint n transfer length */
#define USBFS_DOEPLEN_TLEN_SHIFT        (0)
#define USBFS_DOEPLEN_TLEN_MASK         (0x7ffff << USBFS_DOEPLEN_TLEN_SHIFT)
#define USBFS_DOEPLEN_PCNT_SHIFT        (19)
#define USBFS_DOEPLEN_PCNT_MASK         (0x3ff << USBFS_DOEPLEN_PCNT_SHIFT)
#define USBFS_DOEPLEN_STPCNT_SHIFT      (29)
#define USBFS_DOEPLEN_STPCNT_MASK       (3 << USBFS_DOEPLEN_STPCNT_SHIFT)
#define USBFS_DOEPLEN_RXDPID_SHIFT      (29)
#define USBFS_DOEPLEN_RXDPID_MASK       (3 << USBFS_DOEPLEN_RXDPID_SHIFT)

/* DOEP0LEN - Device OUT endpoint 0 transfer length */
#define USBFS_DOEP0LEN_TLEN_MASK        (0x7f)
#define USBFS_DOEP0LEN_PCNT_SHIFT       (19)
#define USBFS_DOEP0LEN_PCNT_MASK        (1 << USBFS_DOEP0LEN_PCNT_SHIFT)
#define USBFS_DOEP0LEN_STPCNT_SHIFT     (29)
#define USBFS_DOEP0LEN_STPCNT_MASK      (3 << USBFS_DOEP0LEN_STPCNT_SHIFT)

/* DOEPnCTL - Device OUT endpoint n control register */
#define USBFS_DOEPCTL_MPL_SHIFT         (0)
#define USBFS_DOEPCTL_MPL_MASK          (0x7ff << USBFS_DOEPCTL_MPL_SHIFT)
#define USBFS_DOEPCTL_EPACT             (1 << 15) /* Bit 15: Endpoint active */
#define USBFS_DOEPCTL_EOFRM_DPID        (1 << 16) /* Bit 16: Endpoint data PID */
#define USBFS_DOEPCTL_NAKS              (1 << 17) /* Bit 17: NAK status */
#define USBFS_DOEPCTL_EPTYPE_SHIFT      (18)
#define USBFS_DOEPCTL_EPTYPE_MASK       (3 << USBFS_DOEPCTL_EPTYPE_SHIFT)
#define USBFS_DOEPCTL_EPTYPE_CTRL       (0 << USBFS_DOEPCTL_EPTYPE_SHIFT)
#define USBFS_DOEPCTL_EPTYPE_ISOC       (1 << USBFS_DOEPCTL_EPTYPE_SHIFT)
#define USBFS_DOEPCTL_EPTYPE_BULK       (2 << USBFS_DOEPCTL_EPTYPE_SHIFT)
#define USBFS_DOEPCTL_EPTYPE_INTR       (3 << USBFS_DOEPCTL_EPTYPE_SHIFT)
#define USBFS_DOEPCTL_SNOOP             (1 << 20) /* Bit 20: Snoop mode */
#define USBFS_DOEPCTL_STALL             (1 << 21) /* Bit 21: STALL handshake */
#define USBFS_DOEPCTL_CNAK              (1 << 26) /* Bit 26: Clear NAK */
#define USBFS_DOEPCTL_SNAK              (1 << 27) /* Bit 27: Set NAK */
#define USBFS_DOEPCTL_SD0PID            (1 << 28) /* Bit 28: Set DATA0 PID */
#define USBFS_DOEPCTL_SEVFRM            (1 << 28) /* Bit 28: Set even frame */
#define USBFS_DOEPCTL_SD1PID            (1 << 29) /* Bit 29: Set DATA1 PID */
#define USBFS_DOEPCTL_SODDFRM           (1 << 29) /* Bit 29: Set odd frame */
#define USBFS_DOEPCTL_EPD               (1 << 30) /* Bit 30: Endpoint disable */
#define USBFS_DOEPCTL_EPEN              (1 << 31) /* Bit 31: Endpoint enable */

/* DOEP0CTL - Device OUT endpoint 0 control register */

#define USBFS_DOEP0CTL_MPL_SHIFT        (0)
#define USBFS_DOEP0CTL_MPL_MASK         (3 << USBFS_DOEP0CTL_MPL_SHIFT)
#define USBFS_DOEP0CTL_MPL_64           (0 << USBFS_DOEP0CTL_MPL_SHIFT)
#define USBFS_DOEP0CTL_MPL_32           (1 << USBFS_DOEP0CTL_MPL_SHIFT)
#define USBFS_DOEP0CTL_MPL_16           (2 << USBFS_DOEP0CTL_MPL_SHIFT)
#define USBFS_DOEP0CTL_MPL_8            (3 << USBFS_DOEP0CTL_MPL_SHIFT)
#define USBFS_DOEP0CTL_EPACT            (1 << 15) /* Bit 15: Endpoint active (RO) */
#define USBFS_DOEP0CTL_NAKS             (1 << 17) /* Bit 17: NAK status */
#define USBFS_DOEP0CTL_EPTYPE_MASK      (3 << 18)
#define USBFS_DOEP0CTL_SNOOP            (1 << 20) /* Bit 20: Snoop mode */
#define USBFS_DOEP0CTL_STALL            (1 << 21) /* Bit 21: STALL handshake */
#define USBFS_DOEP0CTL_CNAK             (1 << 26) /* Bit 26: Clear NAK */
#define USBFS_DOEP0CTL_SNAK             (1 << 27) /* Bit 27: Set NAK */
#define USBFS_DOEP0CTL_EPD              (1 << 30) /* Bit 30: Endpoint disable */
#define USBFS_DOEP0CTL_EPEN             (1 << 31) /* Bit 31: Endpoint enable */

/* DIEPnTFSTAT - Device IN endpoint n TxFIFO status */

#define USBFS_DIEPTFSTAT_IEPTFS_MASK    (0xffff)

/* PWRCLKCTL - Power and clock control register */

#define USBFS_PWRCLKCTL_SUCLK           (1 << 0)  /* Bit 0:  Stop USB clock */
#define USBFS_PWRCLKCTL_SHCLK           (1 << 1)  /* Bit 1:  Stop HCLK */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_USBFS_H */