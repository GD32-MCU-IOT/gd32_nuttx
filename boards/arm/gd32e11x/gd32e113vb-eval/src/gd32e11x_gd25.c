/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_gd25.c
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
#include <errno.h>
#include <debug.h>
#include <sys/stat.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "gd32e11x.h"
#include "gd32e11x_spi.h"
#include "gd32e113v_eval.h"

#ifdef HAVE_GD25

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gd25_automount
 *
 * Description:
 *   Initialize, configure, and mount the GD25 SPI FLASH.  The FLASH will
 *   be mounted at /dev/gd25.
 *
 ****************************************************************************/

int gd32_gd25_automount(int minor)
{
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  if (!initialized)
    {
      /* Get the SPI port driver */

      spi = gd32_spibus_initialize(SPI_FLASH_CSNUM);
      if (!spi)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
                 SPI_FLASH_CSNUM);
          return -ENODEV;
        }

      /* Bind the SPI interface to the GD25 driver */

      mtd = gd25_initialize(spi, 0);
      if (!mtd)
        {
          syslog(LOG_ERR, "ERROR: Failed to bind SPI to GD25 driver\n");
          return -ENODEV;
        }

#ifdef CONFIG_GD32E113VB_EVAL_GD25_LITTLEFS
      /* Register MTD driver */

      ret = register_mtddriver("/dev/spiflash", mtd, 0755, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register MTD: %d\n", ret);
          return ret;
        }

      /* Mount LittleFS */

      ret = nx_mount("/dev/spiflash", "/mnt/gd25", "littlefs", 0, NULL);
      if (ret < 0)
        {
          /* Try formatting */

          ret = nx_mount("/dev/spiflash", "/mnt/gd25", "littlefs", 0,
                         "forceformat");
          if (ret < 0)
            {
              ferr("ERROR: Failed to mount FS volume: %d\n", ret);
              return ret;
            }
        }

      syslog(LOG_INFO, "INFO: LittleFS mounted at /mnt/gd25\n");
#endif

      initialized = true;
    }

  return OK;
}

#endif
