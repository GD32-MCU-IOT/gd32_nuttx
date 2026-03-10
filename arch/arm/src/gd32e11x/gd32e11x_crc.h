/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_crc.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_CRC_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/gd32e11x_crc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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
 * Name: gd32_crc_init
 *
 * Description:
 *   Initialize the CRC hardware. This function enables the CRC clock and
 *   resets the CRC data register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_crc_init(void);

/****************************************************************************
 * Name: gd32_crc_reset
 *
 * Description:
 *   Reset the CRC data register to its initial value (0xFFFFFFFF).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_crc_reset(void);

/****************************************************************************
 * Name: gd32_crc_calculate
 *
 * Description:
 *   Calculate CRC-32 for a single 32-bit word. The calculation is based
 *   on the current value in the CRC_DATA register.
 *
 * Input Parameters:
 *   data - The 32-bit data word to process
 *
 * Returned Value:
 *   The CRC-32 result
 *
 ****************************************************************************/

uint32_t gd32_crc_calculate(uint32_t data);

/****************************************************************************
 * Name: gd32_crc_calculate_block
 *
 * Description:
 *   Calculate CRC-32 for a block of 32-bit words. The calculation
 *   continues from the current CRC value (use gd32_crc_reset() first
 *   if a fresh calculation is needed).
 *
 * Input Parameters:
 *   data   - Pointer to the array of 32-bit words
 *   length - Number of 32-bit words in the array
 *
 * Returned Value:
 *   The CRC-32 result
 *
 ****************************************************************************/

uint32_t gd32_crc_calculate_block(const uint32_t *data, uint32_t length);

/****************************************************************************
 * Name: gd32_crc_get_value
 *
 * Description:
 *   Get the current CRC value without adding new data.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current CRC-32 value
 *
 ****************************************************************************/

uint32_t gd32_crc_get_value(void);

/****************************************************************************
 * Name: gd32_crc_fdata_write
 *
 * Description:
 *   Write to the free data register (CRC_FDATA). This register is
 *   independent of CRC calculation and can be used for any purpose.
 *
 * Input Parameters:
 *   data - The 8-bit data to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_crc_fdata_write(uint8_t data);

/****************************************************************************
 * Name: gd32_crc_fdata_read
 *
 * Description:
 *   Read from the free data register (CRC_FDATA).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The 8-bit data from the free data register
 *
 ****************************************************************************/

uint8_t gd32_crc_fdata_read(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_CRC_H */
