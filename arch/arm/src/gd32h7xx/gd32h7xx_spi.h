/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_spi.h
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

#ifndef __ARCH_ARM_SRC_GD32H7XX_GD32H7XX_SPI_H
#define __ARCH_ARM_SRC_GD32H7XX_GD32H7XX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include "chip.h"
#include "hardware/gd32h7xx_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   bus - SPI bus number (0, 1, 2, 3, 4, or 5)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *gd32_spibus_initialize(int bus);

/****************************************************************************
 * Name: gd32_spiN_select, gd32_spiN_status, and gd32_spiN_cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods including
 *   gd32_spibus_initialize() are provided by common GD32H7xx logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in gd32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide gd32_spiNselect() and gd32_spiNstatus() functions in your
 *      board-specific logic. These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is
 *      configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, then
 *      provide gd32_spiNcmddata() functions in your board-specific logic.
 *      These functions will perform cmd/data selection operations using
 *      GPIOs in the way your board is configured.
 *   4. Add a call to gd32_spibus_initialize() in your low level application
 *      initialization logic
 *   5. The handle returned by gd32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI0
void gd32_spi0select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t gd32_spi0status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int gd32_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_GD32H7_SPI1
void gd32_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t gd32_spi1status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int gd32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_GD32H7_SPI2
void gd32_spi2select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t gd32_spi2status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int gd32_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_GD32H7_SPI3
void gd32_spi3select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t gd32_spi3status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int gd32_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_GD32H7_SPI4
void gd32_spi4select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t gd32_spi4status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int gd32_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_GD32H7_SPI5
void gd32_spi5select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t gd32_spi5status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int gd32_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: gd32_spi_register
 *
 * Description:
 *   Register one SPI driver for a SPI port
 *
 * Input Parameters:
 *   bus - SPI bus number (0, 1, 2, 3, 4, or 5)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
int gd32_spi_register(int bus);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32H7XX_GD32H7XX_SPI_H */
