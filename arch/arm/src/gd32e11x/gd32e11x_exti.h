/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_exti.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_EXTI_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "hardware/gd32e11x_exti.h"

/* External interrupt and event */

#define EXTI_INTERRUPT                   0  /* EXTI interrupt mode */
#define EXTI_EVENT                       1  /* EXTI event mode */

/* Interrupt trigger mode */

#define EXTI_TRIG_RISING                 0  /* EXTI rising edge trigger */
#define EXTI_TRIG_FALLING                1  /* EXTI falling edge trigger */
#define EXTI_TRIG_BOTH                   2  /* EXTI rising and falling edge trigger */
#define EXTI_TRIG_NONE                   3  /* None EXTI edge trigger */

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

int gd32_exti_gpioirq_init(uint32_t cfgset, uint8_t exti_mode,
						   uint8_t trig_type, uint8_t *irqnum);

int gd32_exti_gpio_irq_attach(uint8_t irqpin, xcpt_t irqhandler,
							  void *arg);

void gd32_exti_init(uint32_t linex, uint8_t exti_mode, uint8_t trig_type);

int gd32_gpio_exti_linex_get(uint32_t cfgset, uint32_t *linex);
int gd32_gpio_exti_irqnum_get(uint32_t cfgset, uint8_t *irqnum);

void gd32_exti_interrupt_enable(uint32_t linex);
void gd32_exti_interrupt_disable(uint32_t linex);

void gd32_exti_event_enable(uint32_t linex);
void gd32_exti_event_disable(uint32_t linex);

void gd32_exti_software_interrupt_enable(uint32_t linex);
void gd32_exti_software_interrupt_disable(uint32_t linex);

bool gd32_exti_interrupt_flag_get(uint32_t linex);
void gd32_exti_interrupt_flag_clear(uint32_t linex);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_EXTI_H */
