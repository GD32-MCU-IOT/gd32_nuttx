/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_progmem.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_PROGMEM_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_PROGMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/progmem.h>

#include "chip.h"
#include "hardware/gd32e11x_memorymap.h"
#include "hardware/gd32e11x_fmc.h"

/****************************************************************************
 * Refer to the GD32E11x User Manual, chapter "Flash memory controller", to
 * know about how the main flash memory is programmed.  The region handed
 * over to the MTD layer is taken from the end of the main flash memory and
 * must not overlap the firmware image.
 *
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GD32_PROGMEM_ERASEDVAL     (0xffu)

/* All error flags of the FMC_STAT register */

#define FMC_STAT_PERR              (FMC_STAT_ENDF | FMC_STAT_WPERR | \
                                    FMC_STAT_PGAERR | FMC_STAT_PGERR)

/* The GD32E11x FMC programs words, so any transfer has to be aligned on a
 * four byte boundary.
 */

#define GD32_PROGMEM_BLOCKSIZE     (4)
#define GD32_PROGMEM_BLOCKMASK     (GD32_PROGMEM_BLOCKSIZE - 1)

#define GD32_PROGMEM_SIZE          (CONFIG_GD32E11X_PROGMEM_SIZE * 1024)
#define GD32_PROGMEM_NPAGES        (GD32_PROGMEM_SIZE / GD32_FLASH_PAGESIZE)
#define GD32_PROGMEM_STARTADDR     (GD32_FLASH_BASE + GD32_FLASH_SIZE - \
                                    GD32_PROGMEM_SIZE)
#define GD32_PROGMEM_ENDADDR       (GD32_FLASH_BASE + GD32_FLASH_SIZE)

/* Index of the first main flash page belonging to the progmem region */

#define GD32_PROGMEM_STARTPAGE     (GD32_FLASH_NPAGES - GD32_PROGMEM_NPAGES)

#if (GD32_PROGMEM_SIZE % GD32_FLASH_PAGESIZE) != 0
#  error "CONFIG_GD32E11X_PROGMEM_SIZE must be a multiple of the page size"
#endif

#if GD32_PROGMEM_SIZE <= 0 || GD32_PROGMEM_SIZE >= GD32_FLASH_SIZE
#  error "CONFIG_GD32E11X_PROGMEM_SIZE exceeds the available flash memory"
#endif

#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_PROGMEM_H */
