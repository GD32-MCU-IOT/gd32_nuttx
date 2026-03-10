/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_crc.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_CRC_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRC Calculation Unit
 *
 * The CRC (Cyclic Redundancy Check) calculation unit is used for error
 * detection in data transmission and storage. It uses a fixed polynomial
 * 0x4C11DB7 which is the same as Ethernet CRC-32.
 *
 * Features:
 *   - 32-bit data input/output register
 *   - CRC calculation takes 4 AHB clock cycles for 32-bit data
 *   - Independent 8-bit data register (FDATA) for general purpose use
 *   - Fixed polynomial: 0x04c11db7 (Ethernet CRC-32)
 */

/* Register Offsets *********************************************************/

#define GD32_CRC_DATA_OFFSET        0x0000  /* CRC data register */
#define GD32_CRC_FDATA_OFFSET       0x0004  /* CRC free data register */
#define GD32_CRC_CTL_OFFSET         0x0008  /* CRC control register */

/* Register Addresses *******************************************************/

#define GD32_CRC_DATA               (GD32_CRC_BASE + GD32_CRC_DATA_OFFSET)
#define GD32_CRC_FDATA              (GD32_CRC_BASE + GD32_CRC_FDATA_OFFSET)
#define GD32_CRC_CTL                (GD32_CRC_BASE + GD32_CRC_CTL_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* CRC data register (CRC_DATA)
 *
 * Address offset: 0x00
 * Reset value: 0xFFFF FFFF
 * Access: 32-bit only (word access)
 *
 * This register is used to write new data for CRC calculation.
 * Write: Input new data for CRC calculation
 * Read: Get the result of the previous CRC calculation
 * Note: The data just written cannot be read back; reading returns the
 *       previous CRC result.
 */

#define CRC_DATA_SHIFT              (0)       /* Bits 0-31: CRC calculation result */
#define CRC_DATA_MASK               (0xffffffff << CRC_DATA_SHIFT)

/* CRC free data register (CRC_FDATA)
 *
 * Address offset: 0x04
 * Reset value: 0x0000 0000
 * Access: 32-bit only (word access)
 *
 * Independent 8-bit data register that is not affected by CRC calculation.
 * Can be used for any purpose by other peripherals.
 * Not affected by CRC_CTL reset operation.
 */

#define CRC_FDATA_SHIFT             (0)       /* Bits 0-7: Free data register */
#define CRC_FDATA_MASK              (0xff << CRC_FDATA_SHIFT)

/* CRC control register (CRC_CTL)
 *
 * Address offset: 0x08
 * Reset value: 0x0000 0000
 * Access: 32-bit only (word access)
 */

#define CRC_CTL_RST                 (1 << 0)  /* Bit 0: Reset CRC_DATA register
                                               * Setting this bit resets CRC_DATA to 0xFFFFFFFF.
                                               * This bit is automatically cleared by hardware.
                                               * This bit has no effect on CRC_FDATA register.
                                               */

/* CRC Polynomial
 *
 * The GD32E11x CRC unit uses a fixed 32-bit polynomial:
 * 0x04c11db7 (Ethernet CRC-32)
 */

#define CRC_POLYNOMIAL              0x04c11db7

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_CRC_H */
