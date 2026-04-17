/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/src/gd32h7xx_i2c.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "gd32h7xx_i2c.h"
#include "gd32h759im_eval.h"

#ifdef CONFIG_GD32H7_I2C

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C device handles */

#ifdef CONFIG_GD32H7_I2C0
static struct i2c_master_s *g_i2c0_dev;
#endif

#ifdef CONFIG_GD32H7_I2C1
static struct i2c_master_s *g_i2c1_dev;
#endif

#ifdef CONFIG_GD32H7_I2C2
static struct i2c_master_s *g_i2c2_dev;
#endif

#ifdef CONFIG_GD32H7_I2C3
static struct i2c_master_s *g_i2c3_dev;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2c_initialize
 *
 * Description:
 *   Initialize I2C buses on the eval board and optionally register
 *   /dev/i2cN character devices.
 *
 *   Call from gd32_bringup() once hardware is ready.
 *
 ****************************************************************************/

void gd32_i2c_initialize(void)
{
  int ret;

#ifdef CONFIG_GD32H7_I2C0
  i2cinfo("Initializing I2C0\n");

  g_i2c0_dev = gd32_i2cbus_initialize(0);
  if (g_i2c0_dev == NULL)
    {
      i2cerr("Failed to initialize I2C0\n");
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(g_i2c0_dev, 0);
      if (ret < 0)
        {
          i2cerr("Failed to register /dev/i2c0: %d\n", ret);
        }
#else
      (void)ret;
#endif
    }
#endif /* CONFIG_GD32H7_I2C0 */

#ifdef CONFIG_GD32H7_I2C1
  i2cinfo("Initializing I2C1\n");

  g_i2c1_dev = gd32_i2cbus_initialize(1);
  if (g_i2c1_dev == NULL)
    {
      i2cerr("Failed to initialize I2C1\n");
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(g_i2c1_dev, 1);
      if (ret < 0)
        {
          i2cerr("Failed to register /dev/i2c1: %d\n", ret);
        }
#else
      (void)ret;
#endif
    }
#endif /* CONFIG_GD32H7_I2C1 */

#ifdef CONFIG_GD32H7_I2C2
  i2cinfo("Initializing I2C2\n");

  g_i2c2_dev = gd32_i2cbus_initialize(2);
  if (g_i2c2_dev == NULL)
    {
      i2cerr("Failed to initialize I2C2\n");
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(g_i2c2_dev, 2);
      if (ret < 0)
        {
          i2cerr("Failed to register /dev/i2c2: %d\n", ret);
        }
#else
      (void)ret;
#endif
    }
#endif /* CONFIG_GD32H7_I2C2 */

#ifdef CONFIG_GD32H7_I2C3
  i2cinfo("Initializing I2C3\n");

  g_i2c3_dev = gd32_i2cbus_initialize(3);
  if (g_i2c3_dev == NULL)
    {
      i2cerr("Failed to initialize I2C3\n");
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(g_i2c3_dev, 3);
      if (ret < 0)
        {
          i2cerr("Failed to register /dev/i2c3: %d\n", ret);
        }
#else
      (void)ret;
#endif
    }
#endif /* CONFIG_GD32H7_I2C3 */
}

#endif /* CONFIG_GD32H7_I2C */
