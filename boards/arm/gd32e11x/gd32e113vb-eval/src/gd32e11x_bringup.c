/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <sys/stat.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>

#include "gd32e11x.h"

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include "gd32e113v_eval.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via boardctl()
 *
 ****************************************************************************/

int gd32_bringup(void)
{
  int ret = OK;
  static bool initialized = false;

  /* Prevent multiple initialization */

  if (initialized)
    {
      return OK;
    }

  initialized = true;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  /* Create the mount point directory if it doesn't exist */

  ret = mkdir(GD32_PROCFS_MOUNTPOINT, 0755);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_ERR, "ERROR: Failed to create %s: %d\n",
             GD32_PROCFS_MOUNTPOINT, errno);
    }

  ret = nx_mount(NULL, GD32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             GD32_PROCFS_MOUNTPOINT, ret);
    }
  else
    {
      syslog(LOG_INFO, "INFO: procfs mounted at %s\n",
             GD32_PROCFS_MOUNTPOINT);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_GD25
  /* Initialize and mount the GD25 SPI flash */

  syslog(LOG_INFO, "INFO: Initializing GD25 SPI Flash...\n");
  ret = gd32_gd25_automount(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gd32_gd25_automount() failed: %d\n", ret);
    }
#else
  syslog(LOG_WARNING, "WARNING: HAVE_GD25 not defined!\n");
#endif

#ifdef CONFIG_SPI
  /* Initialize SPI-based devices */

#ifdef CONFIG_GD32E11X_SPI1
  struct spi_dev_s *spi1 = gd32_spibus_initialize(1);
  if (spi1)
    {
      spi_register(spi1, 1);
      syslog(LOG_INFO, "SPI1 registered as /dev/spi1\n");
    }
#endif

#ifdef CONFIG_GD32E11X_SPI2
  struct spi_dev_s *spi2 = gd32_spibus_initialize(2);
  if (spi2)
    {
      spi_register(spi2, 2);
      syslog(LOG_INFO, "SPI2 registered as /dev/spi2\n");
    }
#endif
#endif

  UNUSED(ret);
  return OK;
}
