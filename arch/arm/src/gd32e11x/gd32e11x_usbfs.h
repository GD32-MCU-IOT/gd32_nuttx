/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_usbfs.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_USBFS_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_USBFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/gd32e11x_usbfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of device-mode endpoints (EP0-EP3) */

#define GD32_NENDPOINTS GD32_USBFS_NENDPOINTS

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * NOTE: The device-mode initialization entry point is the NuttX standard
 * arm_usbinitialize() defined in gd32e11x_usbfsdev.c, not a chip-specific
 * function.
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_usbfshost_initialize
 *
 * Description:
 *   Initialize the USB FS host driver.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one USB host controller,
 *     this identifies which controller is being initialized.  Normally,
 *     this is just zero.
 *
 * Returned Value:
 *   An instance of the USB host connection.
 *
 ****************************************************************************/

#if defined(CONFIG_USBHOST) && defined(CONFIG_GD32E11X_USBFS)
struct usbhost_connection_s;
FAR struct usbhost_connection_s *gd32_usbfshost_initialize(int controller);
#endif

/****************************************************************************
 * Name: gd32_usbsuspend
 *
 * Description:
 *   Board logic must provide the gd32_usbsuspend logic if the USBFS device
 *   driver is used.  This function is called whenever the USB enters or
 *   leaves suspend mode.
 *
 *   When 'resume' is false, this function call provides an opportunity for
 *   the board logic to prepare for a new power state.
 *
 *   When 'resume' is true, normal USB operation is about to be resumed and
 *   the board should prepare for this.
 *
 ****************************************************************************/

struct usbdev_s;
#if defined(CONFIG_USBDEV) && defined(CONFIG_GD32E11X_USBFS)
void gd32_usbsuspend(FAR struct usbdev_s *dev, bool resume);
#endif

/****************************************************************************
 * Name: gd32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the USB host interface.
 *
 * Input Parameters:
 *   iface - For future support of multiple USB host interfaces.  Should be
 *     zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBHOST) && defined(CONFIG_GD32E11X_USBFS)
void gd32_usbhost_vbusdrive(int iface, bool enable);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_USBFS_H */
