/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_usb.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "gd32e11x.h"
#include "gd32e11x_usbfs.h"
#include "gd32e113v_eval.h"

#ifdef CONFIG_GD32E11X_USBFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#  define HAVE_USB 1
#else
#  warning \
  "CONFIG_GD32E11X_USBFS is enabled but neither " \
  "CONFIG_USBDEV nor CONFIG_USBHOST"
#  undef HAVE_USB
#endif

#ifndef CONFIG_GD32E113VB_USBHOST_PRIO
#  define CONFIG_GD32E113VB_USBHOST_PRIO 100
#endif

#ifndef CONFIG_GD32E113VB_USBHOST_STACKSIZE
#  define CONFIG_GD32E113VB_USBHOST_STACKSIZE 1024
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  uinfo("Running\n");

  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_usbinitialize
 *
 * Description:
 *   Called from gd32e11x_start in early initialization to setup USB-related
 *   GPIO pins for the GD32E113VB-EVAL board.
 *
 *   The GD32E11x USBFS has an internal soft pull-up; no external pull-up
 *   GPIO is needed.  Configure VBUS sense and PWRON GPIOs when used.
 *
 ****************************************************************************/

void gd32_usbinitialize(void)
{
#ifdef GPIO_USBFS_VBUS

  /* Configure VBUS sense pin (input) */

  gd32_gpio_config(GPIO_USBFS_VBUS);
#endif

#ifdef GPIO_USBFS_PWRON

  /* Configure VBUS power switch output pin, default off */

  gd32_gpio_config(GPIO_USBFS_PWRON);
  gd32_gpio_write(GPIO_USBFS_PWRON, true); /* Drive high = power off */
#endif
}

/****************************************************************************
 * Name: gd32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor
 *   for device connection/disconnection events.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int gd32_usbhost_initialize(void)
{
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  uinfo("Register class drivers\n");

#ifdef CONFIG_USBHOST_HUB

  /* Initialize USB hub class support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      uerr("ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC

  /* Register the USB mass storage class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM

  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD

  /* Initialize the HID keyboard class */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("Failed to register the HID keyboard class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE

  /* Initialize the HID mouse class */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      uerr("Failed to register the HID mouse class\n");
    }
#endif

  /* Then get an instance of the USB host interface */

  uinfo("Initialize USB host\n");
  g_usbconn = gd32_usbfshost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      uinfo("Start usbhost_waiter\n");
      ret = kthread_create("usbhost", CONFIG_GD32E113VB_USBHOST_PRIO,
                           CONFIG_GD32E113VB_USBHOST_STACKSIZE,
                           usbhost_waiter, NULL);
      return ret < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: gd32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the USB OTG FS host interface.
 *
 * Input Parameters:
 *   iface  - For future support of multiple USB host interface.
 *            Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
void gd32_usbhost_vbusdrive(int iface, bool enable)
{
  DEBUGASSERT(iface == 0);

#ifdef GPIO_USBFS_PWRON
  if (enable)
    {
      /* Enable the Power Switch: active-low, drive low to enable VBUS */

      gd32_gpio_write(GPIO_USBFS_PWRON, false);
    }
  else
    {
      /* Disable the Power Switch: drive high to disable VBUS */

      gd32_gpio_write(GPIO_USBFS_PWRON, true);
    }
#endif
}
#endif

/****************************************************************************
 * Name: gd32_usbsuspend
 *
 * Description:
 *   Board logic must provide the gd32_usbsuspend logic if the USBDEV
 *   driver is used. This function is called whenever the USB enters or
 *   leaves suspend mode. This is an opportunity for the board logic to
 *   shutdown clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void gd32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
#endif

#endif /* CONFIG_GD32E11X_USBFS */