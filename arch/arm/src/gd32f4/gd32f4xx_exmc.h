/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_exmc.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_EXMC_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_EXMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/gd32f4xx_exmc.h"

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
 * Name: gd32_exmc_sdram_wait
 *
 * Description:
 *   Wait for the SDRAM controller to be ready.
 *
 ****************************************************************************/

void gd32_exmc_sdram_wait(void);

/****************************************************************************
 * Name: gd32_exmc_enable
 *
 * Description:
 *   Enable clocking to the EXMC.
 *
 ****************************************************************************/

void gd32_exmc_enable(void);

/****************************************************************************
 * Name: gd32_exmc_disable
 *
 * Description:
 *   Disable clocking to the EXMC.
 *
 ****************************************************************************/

void gd32_exmc_disable(void);

/****************************************************************************
 * Name: gd32_exmc_sdram_write_protect
 *
 * Description:
 *   Enable/Disable writes to an SDRAM.
 *
 ****************************************************************************/

void gd32_exmc_sdram_write_protect(int bank, bool state);

/****************************************************************************
 * Name: gd32_exmc_sdram_set_refresh_rate
 *
 * Description:
 *   Set the SDRAM refresh rate.
 *
 ****************************************************************************/

void gd32_exmc_sdram_set_refresh_rate(int count);

/****************************************************************************
 * Name: gd32_exmc_sdram_set_timing
 *
 * Description:
 *   Set the SDRAM timing parameters.
 *
 ****************************************************************************/

void gd32_exmc_sdram_set_timing(int bank, uint32_t timing);

/****************************************************************************
 * Name: gd32_exmc_sdram_set_control
 *
 * Description:
 *   Set the SDRAM control parameters.
 *
 ****************************************************************************/

void gd32_exmc_sdram_set_control(int bank, uint32_t ctrl);

/****************************************************************************
 * Name: gd32_exmc_sdram_command
 *
 * Description:
 *   Send a command to the SDRAM.
 *
 ****************************************************************************/

void gd32_exmc_sdram_command(uint32_t cmd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_EXMC_H */
