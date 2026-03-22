/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_GD32H7_GD32H7XX_SYSCFG_H
#define __ARCH_ARM_SRC_GD32H7_GD32H7XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#include "hardware/gd32h7xx_syscfg.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_syscfg_deinit
 *
 * Description:
 *   Reset the SYSCFG peripheral registers to default state.
 *
 ****************************************************************************/

void gd32_syscfg_deinit(void);

/****************************************************************************
 * Name: gd32_syscfg_i2c_fast_mode_plus_enable
 *
 * Description:
 *   Enable I2C fast mode plus for specified I2C peripheral or pins.
 *
 ****************************************************************************/

void gd32_syscfg_i2c_fast_mode_plus_enable(uint32_t i2c_fmp);

/****************************************************************************
 * Name: gd32_syscfg_i2c_fast_mode_plus_disable
 *
 * Description:
 *   Disable I2C fast mode plus for specified I2C peripheral or pins.
 *
 ****************************************************************************/

void gd32_syscfg_i2c_fast_mode_plus_disable(uint32_t i2c_fmp);

/****************************************************************************
 * Name: gd32_syscfg_analog_switch_enable
 *
 * Description:
 *   Open analog switch (Pxy and Pxy_C are separated pads).
 *
 ****************************************************************************/

void gd32_syscfg_analog_switch_enable(uint32_t gpio_answ);

/****************************************************************************
 * Name: gd32_syscfg_analog_switch_disable
 *
 * Description:
 *   Close analog switch (Pxy and Pxy_C are connected through the analog
 *   switch).
 *
 ****************************************************************************/

void gd32_syscfg_analog_switch_disable(uint32_t gpio_answ);

/****************************************************************************
 * Name: gd32_syscfg_enet_phy_interface_config
 *
 * Description:
 *   Configure the PHY interface mode for the ethernet MAC (ENET0 or ENET1).
 *
 ****************************************************************************/

void gd32_syscfg_enet_phy_interface_config(uint32_t ethernet,
                                           uint32_t phy_interface);

/****************************************************************************
 * Name: gd32_syscfg_exti_line_config
 *
 * Description:
 *   Configure the GPIO pin as EXTI Line source.
 *
 ****************************************************************************/

void gd32_syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin);

/****************************************************************************
 * Name: gd32_syscfg_lockup_enable
 *
 * Description:
 *   Enable module lockup functions (can be disabled by system reset).
 *
 ****************************************************************************/

void gd32_syscfg_lockup_enable(uint32_t lockup);

/****************************************************************************
 * Name: gd32_syscfg_io_compensation_config
 *
 * Description:
 *   Configure the I/O compensation cell.
 *
 ****************************************************************************/

void gd32_syscfg_io_compensation_config(uint32_t mode);

/****************************************************************************
 * Name: gd32_syscfg_io_low_voltage_speed_optimization_enable
 *
 * Description:
 *   Enable I/O speed optimization for high-speed at low-voltage.
 *
 ****************************************************************************/

void gd32_syscfg_io_low_voltage_speed_optimization_enable(void);

/****************************************************************************
 * Name: gd32_syscfg_io_low_voltage_speed_optimization_disable
 *
 * Description:
 *   Disable I/O speed optimization for high-speed at low-voltage.
 *
 ****************************************************************************/

void gd32_syscfg_io_low_voltage_speed_optimization_disable(void);

/****************************************************************************
 * Name: gd32_syscfg_pnmos_compensation_code_set
 *
 * Description:
 *   Set P/N MOS compensation value.
 *
 ****************************************************************************/

void gd32_syscfg_pnmos_compensation_code_set(uint32_t mos, uint32_t code);

/****************************************************************************
 * Name: gd32_syscfg_secure_sram_size_set
 *
 * Description:
 *   Set secure SRAM size.
 *
 ****************************************************************************/

void gd32_syscfg_secure_sram_size_set(uint32_t size);

/****************************************************************************
 * Name: gd32_syscfg_secure_sram_size_get
 *
 * Description:
 *   Get secure SRAM size.
 *
 ****************************************************************************/

uint32_t gd32_syscfg_secure_sram_size_get(void);

/****************************************************************************
 * Name: gd32_syscfg_bootmode_get
 *
 * Description:
 *   Get BOOT mode.
 *
 ****************************************************************************/

uint32_t gd32_syscfg_bootmode_get(void);

/****************************************************************************
 * Name: gd32_syscfg_tcm_wait_state_enable
 *
 * Description:
 *   Enable TCM wait state.
 *
 ****************************************************************************/

void gd32_syscfg_tcm_wait_state_enable(void);

/****************************************************************************
 * Name: gd32_syscfg_tcm_wait_state_disable
 *
 * Description:
 *   Disable TCM wait state.
 *
 ****************************************************************************/

void gd32_syscfg_tcm_wait_state_disable(void);

/****************************************************************************
 * Name: gd32_syscfg_fpu_interrupt_enable
 *
 * Description:
 *   Enable FPU interrupt.
 *
 ****************************************************************************/

void gd32_syscfg_fpu_interrupt_enable(uint32_t fpu_int);

/****************************************************************************
 * Name: gd32_syscfg_fpu_interrupt_disable
 *
 * Description:
 *   Disable FPU interrupt.
 *
 ****************************************************************************/

void gd32_syscfg_fpu_interrupt_disable(uint32_t fpu_int);

/****************************************************************************
 * Name: gd32_syscfg_compensation_flag_get
 *
 * Description:
 *   Get compensation cell flags.
 *
 ****************************************************************************/

bool gd32_syscfg_compensation_flag_get(uint32_t cps_flag);

/****************************************************************************
 * Name: gd32_syscfg_cpu_cache_status_get
 *
 * Description:
 *   Get the ICACHE or DCACHE detection and error information.
 *
 ****************************************************************************/

uint32_t gd32_syscfg_cpu_cache_status_get(uint32_t cache, uint32_t status);

/****************************************************************************
 * Name: gd32_syscfg_brownout_reset_threshold_level_get
 *
 * Description:
 *   Get brownout reset threshold level.
 *
 ****************************************************************************/

uint32_t gd32_syscfg_brownout_reset_threshold_level_get(void);

/****************************************************************************
 * Name: gd32_syscfg_clock_enable
 *
 * Description:
 *   Enable SYSCFG clock.
 *
 ****************************************************************************/

void gd32_syscfg_clock_enable(void);

/****************************************************************************
 * Name: gd32_syscfg_clock_disable
 *
 * Description:
 *   Disable SYSCFG clock.
 *
 ****************************************************************************/

void gd32_syscfg_clock_disable(void);

#endif /* __ARCH_ARM_SRC_GD32H7_GD32H7XX_SYSCFG_H */
