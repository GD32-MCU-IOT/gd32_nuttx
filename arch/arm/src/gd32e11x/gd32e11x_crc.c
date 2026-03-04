/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_crc.c
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
#include <debug.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/gd32e11x_rcu.h"
#include "hardware/gd32e11x_crc.h"
#include "gd32e11x_crc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_CRC_INFO
#  define crcinfo _info
#else
#  define crcinfo(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_crc_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_crc_clock_enable
 *
 * Description:
 *   Enable the CRC peripheral clock.
 *
 ****************************************************************************/

static void gd32_crc_clock_enable(void)
{
  uint32_t regval;

  /* Enable CRC clock in AHB enable register */

  regval  = getreg32(GD32_RCU_AHBEN);
  regval |= RCU_AHBEN_CRCEN;
  putreg32(regval, GD32_RCU_AHBEN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_crc_init
 *
 * Description:
 *   Initialize the CRC hardware.
 *
 ****************************************************************************/

void gd32_crc_init(void)
{
  if (g_crc_initialized)
    {
      return;
    }

  /* Enable CRC clock */

  gd32_crc_clock_enable();

  /* Reset CRC data register to initial value */

  gd32_crc_reset();

  g_crc_initialized = true;

  crcinfo("CRC initialized\n");
}

/****************************************************************************
 * Name: gd32_crc_reset
 *
 * Description:
 *   Reset the CRC data register to its initial value (0xFFFFFFFF).
 *
 ****************************************************************************/

void gd32_crc_reset(void)
{
  /* Set the RST bit in CRC_CTL register */

  putreg32(CRC_CTL_RST, GD32_CRC_CTL);

  /* Wait for reset to complete (RST bit is cleared automatically) */

  while ((getreg32(GD32_CRC_CTL) & CRC_CTL_RST) != 0)
    {
      /* Wait */
    }

  crcinfo("CRC reset complete, DATA=0x%08lx\n",
          (unsigned long)getreg32(GD32_CRC_DATA));
}

/****************************************************************************
 * Name: gd32_crc_calculate
 *
 * Description:
 *   Calculate CRC-32 for a single 32-bit word.
 *
 ****************************************************************************/

uint32_t gd32_crc_calculate(uint32_t data)
{
  /* Write data to CRC_DATA register to start calculation */

  putreg32(data, GD32_CRC_DATA);

  /* Read and return the CRC result */

  return getreg32(GD32_CRC_DATA);
}

/****************************************************************************
 * Name: gd32_crc_calculate_block
 *
 * Description:
 *   Calculate CRC-32 for a block of 32-bit words.
 *
 ****************************************************************************/

uint32_t gd32_crc_calculate_block(const uint32_t *data, uint32_t length)
{
  uint32_t i;

  DEBUGASSERT(data != NULL || length == 0);

  /* Process each 32-bit word */

  for (i = 0; i < length; i++)
    {
      putreg32(data[i], GD32_CRC_DATA);
    }

  /* Return the final CRC result */

  return getreg32(GD32_CRC_DATA);
}

/****************************************************************************
 * Name: gd32_crc_get_value
 *
 * Description:
 *   Get the current CRC value without adding new data.
 *
 ****************************************************************************/

uint32_t gd32_crc_get_value(void)
{
  return getreg32(GD32_CRC_DATA);
}

/****************************************************************************
 * Name: gd32_crc_fdata_write
 *
 * Description:
 *   Write to the free data register (CRC_FDATA).
 *
 ****************************************************************************/

void gd32_crc_fdata_write(uint8_t data)
{
  putreg32((uint32_t)data, GD32_CRC_FDATA);
}

/****************************************************************************
 * Name: gd32_crc_fdata_read
 *
 * Description:
 *   Read from the free data register (CRC_FDATA).
 *
 ****************************************************************************/

uint8_t gd32_crc_fdata_read(void)
{
  return (uint8_t)(getreg32(GD32_CRC_FDATA) & CRC_FDATA_MASK);
}
