/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_progmem.c
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
#include <stdint.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#ifdef CONFIG_GD32E113VB_EVAL_PROGMEM_NXFFS
#  include <nuttx/fs/nxffs.h>
#endif

#include "arm_internal.h"
#include "gd32e11x.h"
#include "gd32e11x_progmem.h"
#include "gd32e113v_eval.h"

#ifdef HAVE_PROGMEM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROGMEM_MOUNTPOINT "/mnt/progmem"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_progmem_flashend
 *
 * Description:
 *   Return the first flash address beyond the current firmware image.
 *
 ****************************************************************************/

static uintptr_t gd32_progmem_flashend(void)
{
  uintptr_t flashend;

  flashend = (((uintptr_t)_eronly + 3) & ~(uintptr_t)3) +
             ((uintptr_t)_edata - (uintptr_t)_sdata);
#ifdef CONFIG_ARCH_RAMFUNCS
  uintptr_t ramend = (uintptr_t)_framfuncs +
                     ((uintptr_t)_eramfuncs - (uintptr_t)_sramfuncs);

  if (ramend > flashend)
    {
      flashend = ramend;
    }
#endif

  return flashend;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_progmem_automount
 *
 * Description:
 *   Register the on-chip flash MTD device and mount a file system on it.
 *
 * Input Parameters:
 *   minor - The device minor number.  Only 0 is supported.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gd32_progmem_automount(int minor)
{
  static bool registered;
  static bool initialized;
  static struct mtd_dev_s *mtd;
  uintptr_t flashend;
  char devname[32];
  int ret;

  if (initialized)
    {
      return OK;
    }

  if (minor != 0)
    {
      return -EINVAL;
    }

  flashend = gd32_progmem_flashend();
  if (flashend > GD32_PROGMEM_STARTADDR)
    {
      syslog(LOG_ERR,
             "ERROR: Image ends at %08lx, PROGMEM starts at %08lx\n",
             (unsigned long)flashend,
             (unsigned long)GD32_PROGMEM_STARTADDR);
      return -ENOSPC;
    }

  snprintf(devname, sizeof(devname), "/dev/progmem%d", minor);
  if (!registered)
    {
      mtd = progmem_initialize();
      if (mtd == NULL)
        {
          return -ENODEV;
        }

      ret = register_mtddriver(devname, mtd, 0600, NULL);
      if (ret < 0)
        {
          return ret;
        }

      registered = true;
    }

#ifdef CONFIG_GD32E113VB_EVAL_PROGMEM_LITTLEFS
  ret = nx_mount(devname, PROGMEM_MOUNTPOINT, "littlefs", 0, NULL);
#ifdef CONFIG_GD32E113VB_EVAL_PROGMEM_AUTOFMT
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WARNING: Formatting PROGMEM LittleFS\n");
      ret = nx_mount(devname, PROGMEM_MOUNTPOINT, "littlefs", 0,
                     "forceformat");
    }
#endif

  if (ret < 0)
    {
      return ret;
    }

  syslog(LOG_INFO, "INFO: LittleFS mounted at %s\n", PROGMEM_MOUNTPOINT);
#elif defined(CONFIG_GD32E113VB_EVAL_PROGMEM_NXFFS)
  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      return ret;
    }

  ret = nx_mount(NULL, PROGMEM_MOUNTPOINT, "nxffs", 0, NULL);
  if (ret < 0)
    {
      return ret;
    }
#endif

  initialized = true;
  return OK;
}

#endif /* HAVE_PROGMEM */
