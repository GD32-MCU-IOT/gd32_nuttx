/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_dac.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/dac.h>

#include "gd32e11x_dac.h"

#if defined(CONFIG_DAC) && defined(CONFIG_GD32E11X_DAC)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dac_setup
 *
 * Description:
 *   Initialize and register the DAC device(s).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gd32_dac_setup(void)
{
  static bool initialized = false;
  struct dac_dev_s *dac;
  int ret;

  if (initialized)
    {
      return OK;
    }

#ifdef CONFIG_GD32E11X_DAC0
  /* Initialize DAC OUT0 (PA4) */

  dac = gd32_dacinitialize(0);
  if (dac == NULL)
    {
      aerr("ERROR: Failed to get DAC0 interface\n");
      return -ENODEV;
    }

  ret = dac_register("/dev/dac0", dac);
  if (ret < 0)
    {
      aerr("ERROR: dac_register(/dev/dac0) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_GD32E11X_DAC1
  /* Initialize DAC OUT1 (PA5) */

  dac = gd32_dacinitialize(1);
  if (dac == NULL)
    {
      aerr("ERROR: Failed to get DAC1 interface\n");
      return -ENODEV;
    }

  ret = dac_register("/dev/dac1", dac);
  if (ret < 0)
    {
      aerr("ERROR: dac_register(/dev/dac1) failed: %d\n", ret);
      return ret;
    }
#endif

  initialized = true;
  UNUSED(ret);
  return OK;
}

#endif /* CONFIG_DAC && CONFIG_GD32E11X_DAC */
