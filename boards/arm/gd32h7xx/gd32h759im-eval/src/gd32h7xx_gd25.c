/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/src/gd32h7xx_gd25.c
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

#ifdef CONFIG_FS_NXFFS
#  include <nuttx/fs/nxffs.h>
#endif

#include "gd32h7xx.h"
#include "gd32h7xx_spi.h"
#include "gd32h759im_eval.h"

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

#ifdef CONFIG_GD32H759IM_EVAL_GD25_LITTLEFS
      /* Register MTD driver */

      ret = register_mtddriver("/dev/spiflash", mtd, 0755, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register MTD: %d\n", ret);
          return ret;
        }

      /* Create mount point if needed */

      ret = mkdir("/mnt/gd25", 0755);
      if (ret < 0 && errno != EEXIST)
        {
          syslog(LOG_WARNING, "WARNING: mkdir /mnt/gd25 failed: %d\n",
                 errno);
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

#elif defined(CONFIG_GD32H759IM_EVAL_GD25_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Create mount point if needed */

      ret = mkdir("/mnt/gd25", 0755);
      if (ret < 0 && errno != EEXIST)
        {
          syslog(LOG_WARNING, "WARNING: mkdir /mnt/gd25 failed: %d\n",
                 errno);
        }

      /* Mount NXFFS */

      ret = nx_mount(NULL, "/mnt/gd25", "nxffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount NXFFS: %d\n", ret);
          return ret;
        }

      syslog(LOG_INFO, "INFO: NXFFS mounted at /mnt/gd25\n");

#elif defined(CONFIG_GD32H759IM_EVAL_GD25_FTL)
      /* Register MTD as a block device via FTL */

      ret = ftl_initialize(minor, mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: FTL initialization failed: %d\n", ret);
          return ret;
        }

      syslog(LOG_INFO, "INFO: GD25 registered as /dev/mtdblock%d\n", minor);

#else
      /* Just register the MTD driver */

      ret = register_mtddriver("/dev/spiflash", mtd, 0755, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register MTD: %d\n", ret);
          return ret;
        }

      syslog(LOG_INFO, "INFO: GD25 registered as /dev/spiflash\n");
#endif

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_GD25 */
