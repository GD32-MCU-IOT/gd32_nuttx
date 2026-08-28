/****************************************************************************
 * include/nuttx/usb/usbdev_custom_hid.h
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

#ifndef __INCLUDE_NUTTX_USB_USBDEV_CUSTOM_HID_H
#define __INCLUDE_NUTTX_USB_USBDEV_CUSTOM_HID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_CUSTOM_HID_EP0MAXPACKET
#  define CONFIG_CUSTOM_HID_EP0MAXPACKET  64
#endif

#ifndef CONFIG_CUSTOM_HID_EPIN
#  define CONFIG_CUSTOM_HID_EPIN          1
#endif

#ifndef CONFIG_CUSTOM_HID_EPOUT
#  define CONFIG_CUSTOM_HID_EPOUT         1
#endif

#ifndef CONFIG_CUSTOM_HID_EPSIZE
#  define CONFIG_CUSTOM_HID_EPSIZE        8
#endif

#ifndef CONFIG_CUSTOM_HID_EPINTERVAL
#  define CONFIG_CUSTOM_HID_EPINTERVAL    32
#endif

#ifndef CONFIG_CUSTOM_HID_NRDREQS
#  define CONFIG_CUSTOM_HID_NRDREQS       4
#endif

#ifndef CONFIG_CUSTOM_HID_NWRREQS
#  define CONFIG_CUSTOM_HID_NWRREQS       4
#endif

#ifndef CONFIG_CUSTOM_HID_VENDORID
#  define CONFIG_CUSTOM_HID_VENDORID      0x28e9
#endif

#ifndef CONFIG_CUSTOM_HID_PRODUCTID
#  define CONFIG_CUSTOM_HID_PRODUCTID     0x028a
#endif

#ifndef CONFIG_CUSTOM_HID_VENDORSTR
#  define CONFIG_CUSTOM_HID_VENDORSTR     "GigaDevice"
#endif

#ifndef CONFIG_CUSTOM_HID_PRODUCTSTR
#  define CONFIG_CUSTOM_HID_PRODUCTSTR    "GD32-CustomHID"
#endif

#ifndef CONFIG_CUSTOM_HID_SERIALSTR
#  define CONFIG_CUSTOM_HID_SERIALSTR     "0"
#endif

/* Report IDs (matching the GD32 standard-library custom_hid example) *******/

#define CUSTOM_HID_REPORTID_LED1          0x11
#define CUSTOM_HID_REPORTID_LED2          0x12
#define CUSTOM_HID_REPORTID_LED3          0x13
#define CUSTOM_HID_REPORTID_LED4          0x14
#define CUSTOM_HID_REPORTID_BUTTON1       0x15  /* Wakeup key */
#define CUSTOM_HID_REPORTID_BUTTON2       0x16  /* Tamper key */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: custom_hid_initialize
 *
 * Description:
 *   Register the custom HID USB device class driver and its /dev/hidN
 *   character device.
 *
 * Input Parameters:
 *   minor  - Device minor number; the character device is registered as
 *            /dev/hid<minor>.
 *   handle - An optional opaque reference to the class driver object that
 *            may subsequently be used with custom_hid_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int custom_hid_initialize(int minor, FAR void **handle);

/****************************************************************************
 * Name: custom_hid_uninitialize
 *
 * Description:
 *   Un-register the custom HID USB device class driver.
 *
 * Input Parameters:
 *   handle - The handle returned by custom_hid_initialize().
 *
 ****************************************************************************/

void custom_hid_uninitialize(FAR void *handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USBDEV_CUSTOM_HID_H */
