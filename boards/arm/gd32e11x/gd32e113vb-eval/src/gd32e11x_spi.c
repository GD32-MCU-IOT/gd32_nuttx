/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_spi.c
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

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32e11x.h"
#include "gd32e113v_eval.h"

#if defined(CONFIG_SPI)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************/

void gd32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI has already been enabled in gd32_rcu.c.
   * Configurations of SPI pins are performed in gd32_spi.c.
   * Here, we only initialize chip select pins unique to the board
   * architecture.
   */

#ifdef CONFIG_GD32E11X_SPI0
  /* Configure SPI0 chip select pins if needed */

#endif

#ifdef CONFIG_GD32E11X_SPI1
  /* Configure SPI1 chip select pins if needed */

#endif

#ifdef CONFIG_GD32E11X_SPI2
  /* Configure SPI2 chip select pins if needed */

#endif
}

/****************************************************************************
 * Name:  gd32_spi0/1/2select and gd32_spi0/1/2status
 *
 * Description:
 *   The external functions, gd32_spi0/1/2select and gd32_spi0/1/2status
 *   must be provided by board-specific logic. They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   gd32_spibus_initialize()) are provided by common GD32 logic. To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in gd32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide gd32_spi0/1/2select() and gd32_spi0/1/2status() functions
 *      in your board-specific logic. These functions will perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   3. Add a calls to gd32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by gd32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_SPI0
void gd32_spi0select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t gd32_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_GD32E11X_SPI1
void gd32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t gd32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_GD32E11X_SPI2
void gd32_spi2select(struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t gd32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_SPI */
