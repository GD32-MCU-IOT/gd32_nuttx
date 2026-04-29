/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/src/gd32h7xx_spi.c
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
#include "gd32h7xx.h"
#include "gd32h759im_eval.h"

#if defined(CONFIG_GD32H7_SPI0) || defined(CONFIG_GD32H7_SPI1) || \
    defined(CONFIG_GD32H7_SPI2) || defined(CONFIG_GD32H7_SPI3) || \
    defined(CONFIG_GD32H7_SPI4) || defined(CONFIG_GD32H7_SPI5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI0 chip select pin - define as needed for your board
 * Example: this could be connected to an external SPI Flash on PA4
 */

#ifdef CONFIG_GD32H7_SPI0
#  ifndef GPIO_SPI0_CS
#    define GPIO_SPI0_CS  (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PP | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_60MHZ | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_4)
#  endif
#endif

/* SPI1 chip select pin - PB12, push-pull output, default high */

#ifdef CONFIG_GD32H7_SPI1
#  ifndef GPIO_SPI1_CS
#    define GPIO_SPI1_CS  (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PP | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_60MHZ | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_12)
#  endif
#endif

/* SPI2 chip select pin - PA15, push-pull output, default high */

#ifdef CONFIG_GD32H7_SPI2
#  ifndef GPIO_SPI2_CS
#    define GPIO_SPI2_CS  (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PP | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_60MHZ | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_15)
#  endif
#endif

/* SPI3 chip select pin - PE4, push-pull output, default high
 * Used for GD25 SPI Flash on the eval board.
 */

#ifdef CONFIG_GD32H7_SPI3
#  ifndef GPIO_SPI3_CS
#    define GPIO_SPI3_CS  (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PP | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_60MHZ | \
                           GPIO_CFG_PORT_E | GPIO_CFG_PIN_4)
#  endif
#endif

/* SPI4 chip select pin - PF6, push-pull output, default high */

#ifdef CONFIG_GD32H7_SPI4
#  ifndef GPIO_SPI4_CS
#    define GPIO_SPI4_CS  (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PP | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_60MHZ | \
                           GPIO_CFG_PORT_F | GPIO_CFG_PIN_6)
#  endif
#endif

/* SPI5 chip select pin - PG8, push-pull output, default high */

#ifdef CONFIG_GD32H7_SPI5
#  ifndef GPIO_SPI5_CS
#    define GPIO_SPI5_CS  (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PP | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_60MHZ | \
                           GPIO_CFG_PORT_G | GPIO_CFG_PIN_8)
#  endif
#endif

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
#ifdef CONFIG_GD32H7_SPI0
  gd32_gpio_config(GPIO_SPI0_CS);
  gd32_gpio_write(GPIO_SPI0_CS, 1);
#endif

#ifdef CONFIG_GD32H7_SPI1
  gd32_gpio_config(GPIO_SPI1_CS);
  gd32_gpio_write(GPIO_SPI1_CS, 1);
#endif

#ifdef CONFIG_GD32H7_SPI2
  gd32_gpio_config(GPIO_SPI2_CS);
  gd32_gpio_write(GPIO_SPI2_CS, 1);
#endif

#ifdef CONFIG_GD32H7_SPI3
  gd32_gpio_config(GPIO_SPI3_CS);
  gd32_gpio_write(GPIO_SPI3_CS, 1);
#endif

#ifdef CONFIG_GD32H7_SPI4
  gd32_gpio_config(GPIO_SPI4_CS);
  gd32_gpio_write(GPIO_SPI4_CS, 1);
#endif

#ifdef CONFIG_GD32H7_SPI5
  gd32_gpio_config(GPIO_SPI5_CS);
  gd32_gpio_write(GPIO_SPI5_CS, 1);
#endif
}

/****************************************************************************
 * Name: gd32_spi0select and gd32_spi0status
 *
 * Description:
 *   The external functions, gd32_spi0select and gd32_spi0status
 *   must be provided by board-specific logic. They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   gd32_spibus_initialize()) are provided by common GD32 logic.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI0
void gd32_spi0select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  /* SPI0 chip select is active low */

  gd32_gpio_write(GPIO_SPI0_CS, !selected);
}

uint8_t gd32_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_GD32H7_SPI0 */

/****************************************************************************
 * Name: gd32_spi1select and gd32_spi1status
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI1
void gd32_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI1_CS, !selected);
}

uint8_t gd32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_GD32H7_SPI1 */

/****************************************************************************
 * Name: gd32_spi2select and gd32_spi2status
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI2
void gd32_spi2select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI2_CS, !selected);
}

uint8_t gd32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_GD32H7_SPI2 */

/****************************************************************************
 * Name: gd32_spi3select and gd32_spi3status
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI3
void gd32_spi3select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  /* PE4 as CS for GD25 Flash - active low */

  gd32_gpio_write(GPIO_SPI3_CS, !selected);
}

uint8_t gd32_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_GD32H7_SPI3 */

/****************************************************************************
 * Name: gd32_spi4select and gd32_spi4status
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI4
void gd32_spi4select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI4_CS, !selected);
}

uint8_t gd32_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_GD32H7_SPI4 */

/****************************************************************************
 * Name: gd32_spi5select and gd32_spi5status
 ****************************************************************************/

#ifdef CONFIG_GD32H7_SPI5
void gd32_spi5select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  gd32_gpio_write(GPIO_SPI5_CS, !selected);
}

uint8_t gd32_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int gd32_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_GD32H7_SPI5 */

#endif /* CONFIG_GD32H7_SPI0 || ... || CONFIG_GD32H7_SPI5 */
