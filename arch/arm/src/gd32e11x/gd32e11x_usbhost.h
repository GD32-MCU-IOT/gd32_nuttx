/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_usbhost.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_USBHOST_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_USBHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_trace.h>
#include <stdint.h>

#include "chip.h"

#if defined(CONFIG_GD32E11X_USBFS) && defined(CONFIG_USBHOST)

#include "hardware/gd32e11x_usbfs.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE
enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

  USBFS_TRACE1_DEVDISCONN,           /* USBFS ERROR: Host Port Device disconnected */
  USBFS_TRACE1_IRQATTACH,            /* USBFS ERROR: Failed to attach IRQ */
  USBFS_TRACE1_TRNSFRFAILED,         /* USBFS ERROR: Host Port Transfer Failed */
  USBFS_TRACE1_SENDSETUP,            /* USBFS ERROR: sendsetup() failed with: */
  USBFS_TRACE1_SENDDATA,             /* USBFS ERROR: senddata() failed with: */
  USBFS_TRACE1_RECVDATA,             /* USBFS ERROR: recvdata() failed with: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  USBFS_VTRACE1_CONNECTED,           /* USBFS Host Port connected */
  USBFS_VTRACE1_DISCONNECTED,        /* USBFS Host Port disconnected */
  USBFS_VTRACE1_GINT,                /* USBFS Handling Interrupt. Entry Point */
  USBFS_VTRACE1_GINT_SOF,            /* USBFS Handle the start of frame interrupt */
  USBFS_VTRACE1_GINT_RXFLVL,         /* USBFS Handle the RxFIFO non-empty interrupt */
  USBFS_VTRACE1_GINT_NPTXFE,         /* USBFS Handle the non-periodic TxFIFO empty interrupt */
  USBFS_VTRACE1_GINT_PTXFE,          /* USBFS Handle the periodic TxFIFO empty interrupt */
  USBFS_VTRACE1_GINT_HC,             /* USBFS Handle the host channels interrupt */
  USBFS_VTRACE1_GINT_HPRT,           /* USBFS Handle the host port interrupt */
  USBFS_VTRACE1_GINT_HPRT_PCDET,     /* USBFS  HPCS: Port Connect Detect */
  USBFS_VTRACE1_GINT_HPRT_PENCHNG,   /* USBFS  HPCS: Port Enable Changed */
  USBFS_VTRACE1_GINT_HPRT_LSDEV,     /* USBFS  HPCS: Low Speed Device Connected */
  USBFS_VTRACE1_GINT_HPRT_FSDEV,     /* USBFS  HPCS: Full Speed Device Connected */
  USBFS_VTRACE1_GINT_HPRT_LSFSSW,    /* USBFS  HPCS: Host Switch: LS -> FS */
  USBFS_VTRACE1_GINT_HPRT_FSLSSW,    /* USBFS  HPCS: Host Switch: FS -> LS */
  USBFS_VTRACE1_GINT_DISC,           /* USBFS Handle the disconnect detected interrupt */
  USBFS_VTRACE1_GINT_IPXFR,          /* USBFS Handle the incomplete periodic transfer */

#endif

  __TRACE1_NSTRINGS,                 /* Separates the format 1 from the format 2 strings */

  USBFS_TRACE2_CLIP,                 /* USBFS CLIP: chidx:  buflen: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  USBFS_VTRACE2_CHANWAKEUP_IN,       /* USBFS IN Channel wake up with result */
  USBFS_VTRACE2_CHANWAKEUP_OUT,      /* USBFS OUT Channel wake up with result */
  USBFS_VTRACE2_CTRLIN,              /* USBFS CTRLIN */
  USBFS_VTRACE2_CTRLOUT,             /* USBFS CTRLOUT */
  USBFS_VTRACE2_INTRIN,              /* USBFS INTRIN */
  USBFS_VTRACE2_INTROUT,             /* USBFS INTROUT */
  USBFS_VTRACE2_BULKIN,              /* USBFS BULKIN */
  USBFS_VTRACE2_BULKOUT,             /* USBFS BULKOUT */
  USBFS_VTRACE2_ISOCIN,              /* USBFS ISOCIN */
  USBFS_VTRACE2_ISOCOUT,             /* USBFS ISOCOUT */
  USBFS_VTRACE2_STARTTRANSFER,       /* USBFS EP buflen */
  USBFS_VTRACE2_CHANCONF_CTRL_IN,
  USBFS_VTRACE2_CHANCONF_CTRL_OUT,
  USBFS_VTRACE2_CHANCONF_INTR_IN,
  USBFS_VTRACE2_CHANCONF_INTR_OUT,
  USBFS_VTRACE2_CHANCONF_BULK_IN,
  USBFS_VTRACE2_CHANCONF_BULK_OUT,
  USBFS_VTRACE2_CHANCONF_ISOC_IN,
  USBFS_VTRACE2_CHANCONF_ISOC_OUT,
  USBFS_VTRACE2_CHANHALT,            /* Channel halted. chidx: , reason:  */

#endif

  __TRACE2_NSTRINGS                  /* Total number of enumeration values */
};

#  define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#  define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#  define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS)

#  define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#  define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#  define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)

#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* GD32E11x USBFS Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST        - Enable general USB host support
 *  CONFIG_GD32E11X_USBFS  - Enable the GD32E11x USBFS block
 *
 * Options:
 *
 *  CONFIG_GD32E11X_USBFS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_GD32E11X_USBFS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_GD32E11X_USBFS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in
 *    32-bit words.  Default 96 (384 bytes)
 *  CONFIG_GD32E11X_USBFS_SOFINTR - Enable SOF interrupts.  Why would you
 *    ever want to do that?
 *
 *  CONFIG_GD32E11X_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG.
 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gd32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided be each platform that implements the GD32E11x USBFS host
 *   interface
 *
 *   "On-chip 5 V VBUS generation is not supported. For this reason, a charge
 *    pump or, if 5 V are available on the application board, a basic power
 *    switch, must be added externally to drive the 5 V VBUS line. The
 *    external charge pump can be driven by any GPIO output. When the
 *    application decides to power on VBUS using the chosen GPIO, it must
 *    also set the port power bit in the host port control and status
 *    register (PPWR bit in USBFS_HPCS).
 *
 *   "The application uses this field to control power to this port, and the
 *    core clears this bit on an over current condition."
 *
 * Input Parameters:
 *   iface - For future growth to handle multiple USB host interface.
 *     Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_usbhost_vbusdrive(int iface, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_GD32E11X_USBFS && CONFIG_USBHOST */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_USBHOST_H */
