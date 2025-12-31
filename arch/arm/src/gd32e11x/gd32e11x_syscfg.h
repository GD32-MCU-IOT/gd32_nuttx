/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_syscfg.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_SYSCFG_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include "hardware/gd32e11x_syscfg.h"

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
 * Name: gd32_syscfg_clock_enable
 *
 * Description:
 *   This function enable the AFIO clock.
 *
 ****************************************************************************/

void gd32_syscfg_clock_enable(void);

/****************************************************************************
 * Name: gd32_syscfg_clock_disable
 *
 * Description:
 *   This function disable the AFIO clock.
 *
 ****************************************************************************/

void gd32_syscfg_clock_disable(void);

/****************************************************************************
 * Name: gd32_syscfg_exti_line_config
 *
 * Description:
 *   This function configure the GPIO pin as EXTI Line.
 *
 * Input Parameters:
 *   exti_port - EXTI port source (GPIO_PORT_x)
 *   exti_pin  - EXTI pin source (GPIO_PIN_x)
 *
 ****************************************************************************/

void gd32_syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin);

/****************************************************************************
 * Name: gd32_gpio_remap
 *
 * Description:
 *   Configure GPIO remap for alternate functions.
 *
 ****************************************************************************/

void gd32_gpio_remap(void);

/****************************************************************************
 * Name: gd32_gpio_compensation_config
 *
 * Description:
 *   Enable or disable the I/O compensation cell.
 *
 * Input Parameters:
 *   compensation - GPIO_COMPENSATION_ENABLE or GPIO_COMPENSATION_DISABLE
 *
 ****************************************************************************/

void gd32_gpio_compensation_config(uint32_t compensation);

/****************************************************************************
 * Name: gd32_gpio_compensation_flag_get
 *
 * Description:
 *   Check if I/O compensation cell is ready.
 *
 * Returned Value:
 *   true if ready, false otherwise
 *
 ****************************************************************************/

bool gd32_gpio_compensation_flag_get(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_SYSCFG_H */
