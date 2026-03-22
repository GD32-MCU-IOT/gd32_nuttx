/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_syscfg.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"

#include "chip.h"
#include "gd32h7xx_syscfg.h"
#include "gd32h7xx.h"

#ifdef CONFIG_GD32H7_SYSCFG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_syscfg_deinit
 *
 * Description:
 *   Reset the SYSCFG peripheral registers.
 *
 ****************************************************************************/

void gd32_syscfg_deinit(void)
{
  rcu_periph_reset_enable(RCU_SYSCFGRST);
  rcu_periph_reset_disable(RCU_SYSCFGRST);
}

/****************************************************************************
 * Name: gd32_syscfg_i2c_fast_mode_plus_enable
 *
 * Description:
 *   Enable I2Cx(x=0,1,2,3) fast mode plus or I2C fast mode plus
 *   PBx(x=6,7,8,9).
 *
 ****************************************************************************/

void gd32_syscfg_i2c_fast_mode_plus_enable(uint32_t i2c_fmp)
{
  uint32_t regval = getreg32(GD32_SYSCFG_PMCFG);
  regval |= i2c_fmp;
  putreg32(regval, GD32_SYSCFG_PMCFG);
}

/****************************************************************************
 * Name: gd32_syscfg_i2c_fast_mode_plus_disable
 *
 * Description:
 *   Disable I2Cx(x=0,1,2,3) fast mode plus or I2C fast mode plus
 *   PBx(x=6,7,8,9).
 *
 ****************************************************************************/

void gd32_syscfg_i2c_fast_mode_plus_disable(uint32_t i2c_fmp)
{
  uint32_t regval = getreg32(GD32_SYSCFG_PMCFG);
  regval &= ~i2c_fmp;
  putreg32(regval, GD32_SYSCFG_PMCFG);
}

/****************************************************************************
 * Name: gd32_syscfg_analog_switch_enable
 *
 * Description:
 *   Open analog switch (Pxy and Pxy_C are separated pads).
 *
 ****************************************************************************/

void gd32_syscfg_analog_switch_enable(uint32_t gpio_answ)
{
  uint32_t regval = getreg32(GD32_SYSCFG_PMCFG);
  regval |= gpio_answ;
  putreg32(regval, GD32_SYSCFG_PMCFG);
}

/****************************************************************************
 * Name: gd32_syscfg_analog_switch_disable
 *
 * Description:
 *   Close analog switch (Pxy and Pxy_C are separated pads).
 *
 ****************************************************************************/

void gd32_syscfg_analog_switch_disable(uint32_t gpio_answ)
{
  uint32_t regval = getreg32(GD32_SYSCFG_PMCFG);
  regval &= ~gpio_answ;
  putreg32(regval, GD32_SYSCFG_PMCFG);
}

/****************************************************************************
 * Name: gd32_syscfg_enet_phy_interface_config
 *
 * Description:
 *   Configure the PHY interface mode for ENET0/ENET1.
 *
 ****************************************************************************/

void gd32_syscfg_enet_phy_interface_config(uint32_t ethernet,
                                           uint32_t phy_interface)
{
  uint32_t regval = getreg32(GD32_SYSCFG_PMCFG);
  if (ethernet == ENET0)
    {
      regval &= ~SYSCFG_PMCFG_ENET0_PHY_SEL;
      regval |= ENET0_MEDIA_INTERFACE(phy_interface);
    }
  else
    {
      regval &= ~SYSCFG_PMCFG_ENET1_PHY_SEL;
      regval |= ENET1_MEDIA_INTERFACE(phy_interface);
    }

  putreg32(regval, GD32_SYSCFG_PMCFG);
}

/****************************************************************************
 * Name: gd32_syscfg_exti_line_config
 *
 * Description:
 *   Configure a GPIO pin as an EXTI source line.
 *
 ****************************************************************************/

void gd32_syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin)
{
  uint32_t clear_mask =
             ~((uint32_t)EXTI_SS_MASK << (EXTI_SS_MSTEP(exti_pin)));
  uint32_t set_mask   = ((uint32_t)exti_port) << (EXTI_SS_MSTEP(exti_pin));
  volatile uint32_t regaddr;

  switch (exti_pin / SYSCFG_EXTI_SS_JSTEP)
    {
    /* EXTI source line(0..3) */

    case SYSCFG_EXTISS0:
      regaddr = GD32_SYSCFG_EXTISS0;
      break;

    /* EXTI source line(4..7) */

    case SYSCFG_EXTISS1:
      regaddr = GD32_SYSCFG_EXTISS1;
      break;

    /* EXTI source line(8..11) */

    case SYSCFG_EXTISS2:
      regaddr = GD32_SYSCFG_EXTISS2;
      break;

    /* EXTI source line(12..15) */

    case SYSCFG_EXTISS3:
      regaddr = GD32_SYSCFG_EXTISS3;
      break;
    default:
      DEBUGASSERT(false);
      break;
    }

  regval = getreg32(regaddr);
  regval &= ~(SYSCFG_EXTI_SS_MASK << (SYSCFG_EXTI_SS_MSTEP(exti_pin)));
  regval |= ((exti_port) << (SYSCFG_EXTI_SS_MSTEP(exti_pin)));
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_syscfg_lockup_enable
 ****************************************************************************/

void gd32_syscfg_lockup_enable(uint32_t lockup)
{
  uint32_t regval = getreg32(GD32_SYSCFG_LKCTL);
  regval |= lockup;
  putreg32(regval, GD32_SYSCFG_LKCTL);
}

/****************************************************************************
 * Name: gd32_syscfg_io_compensation_config
 *
 * Description:
 *   Configure the I/O compensation cell.
 *
 ****************************************************************************/

void gd32_syscfg_io_compensation_config(uint32_t mode)
{
  uint32_t regval = getreg32(GD32_SYSCFG_CPSCTL);
  regval &= ~SYSCFG_CPSCTL_CPS_EN;
  regval |= mode;
  putreg32(regval, GD32_SYSCFG_CPSCTL);
}

void gd32_syscfg_io_low_voltage_speed_optimization_enable(void)
{
  uint32_t regval = getreg32(GD32_SYSCFG_CPSCTL);
  regval |= SYSCFG_CPSCTL_IOSPDOP;
  putreg32(regval, GD32_SYSCFG_CPSCTL);
}

void gd32_syscfg_io_low_voltage_speed_optimization_disable(void)
{
  uint32_t regval = getreg32(GD32_SYSCFG_CPSCTL);
  regval &= ~SYSCFG_CPSCTL_IOSPDOP;
  putreg32(regval, GD32_SYSCFG_CPSCTL);
}

/****************************************************************************
 * Name: gd32_syscfg_pnmos_compensation_code_set
 ****************************************************************************/

void gd32_syscfg_pnmos_compensation_code_set(uint32_t mos, uint32_t code)
{
  uint32_t value = getreg32(GD32_SYSCFG_CPSCCCFG);
  if (mos == NMOS_COMPENSATION)
    {
      value &= ~0x0f;
      value |= (code & 0x0f);
    }
  else
    {
      value &= ~(0x0f << 8);
      value |= ((code & 0x0f) << 8);
    }

  putreg32(value, GD32_SYSCFG_CPSCCCFG);
}

/****************************************************************************
 * Name: gd32_syscfg_secure_sram_size_set / get
 ****************************************************************************/

void gd32_syscfg_secure_sram_size_set(uint32_t size)
{
  uint32_t regval = getreg32(GD32_SYSCFG_SRAMCFG0);
  regval &= ~SYSCFG_SRAMCFG0_SECURE_SRAM_SIZE;
  regval |= size;
  putreg32(regval, GD32_SYSCFG_SRAMCFG0);
}

uint32_t gd32_syscfg_secure_sram_size_get(void)
{
  return (getreg32(GD32_SYSCFG_SRAMCFG0) & SYSCFG_SRAMCFG0_SECURE_SRAM_SIZE);
}

/****************************************************************************
 * Name: gd32_syscfg_bootmode_get
 ****************************************************************************/

uint32_t gd32_syscfg_bootmode_get(void)
{
  return ((getreg32(GD32_SYSCFG_USERCFG) & SYSCFG_USERCFG_BOOT_MODE) >> 4);
}

/****************************************************************************
 * Name: gd32_syscfg_tcm_wait_state_enable / disable
 ****************************************************************************/

void gd32_syscfg_tcm_wait_state_enable(void)
{
  uint32_t regval = getreg32(GD32_SYSCFG_SRAMCFG1);
  regval |= SYSCFG_SRAMCFG1_TCM_WAITSTATE;
  putreg32(regval, GD32_SYSCFG_SRAMCFG1);
}

void gd32_syscfg_tcm_wait_state_disable(void)
{
  uint32_t regval = getreg32(GD32_SYSCFG_SRAMCFG1);
  regval &= ~SYSCFG_SRAMCFG1_TCM_WAITSTATE;
  putreg32(regval, GD32_SYSCFG_SRAMCFG1);
}

/****************************************************************************
 * Name: gd32_syscfg_fpu_interrupt_enable / disable
 ****************************************************************************/

void gd32_syscfg_fpu_interrupt_enable(uint32_t fpu_int)
{
  uint32_t regval = getreg32(GD32_SYSCFG_FPUINTEN);
  regval |= fpu_int;
  putreg32(regval, GD32_SYSCFG_FPUINTEN);
}

void gd32_syscfg_fpu_interrupt_disable(uint32_t fpu_int)
{
  uint32_t regval = getreg32(GD32_SYSCFG_FPUINTEN);
  regval &= ~fpu_int;
  putreg32(regval, GD32_SYSCFG_FPUINTEN);
}

/****************************************************************************
 * Name: gd32_syscfg_compensation_flag_get
 ****************************************************************************/

bool gd32_syscfg_compensation_flag_get(uint32_t cps_flag)
{
  if (getreg32(GD32_SYSCFG_CPSCTL) & cps_flag)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: gd32_syscfg_cpu_cache_status_get
 ****************************************************************************/

uint32_t gd32_syscfg_cpu_cache_status_get(uint32_t cache, uint32_t status)
{
  uint32_t value = 0;
  switch (cache)
    {
    case ICACHE_STATUS:

      /* Access ICACHE related status registers (implementation dependent) */

      value = getreg32(GD32_SYSCFG_ICACHESTS + status);
      break;
    case DCACHE_STATUS:
      value = getreg32(GD32_SYSCFG_DCACHESTS + status);
      break;
    default:
      break;
    }

  return value;
}

/****************************************************************************
 * Name: gd32_syscfg_brownout_reset_threshold_level_get
 ****************************************************************************/

uint32_t gd32_syscfg_brownout_reset_threshold_level_get(void)
{
  return (getreg32(GD32_SYSCFG_USERCFG) & SYSCFG_USERCFG_BOR_TH);
}

/****************************************************************************
 * Name: gd32_syscfg_clock_enable / disable
 ****************************************************************************/

void gd32_syscfg_clock_enable(void)
{
  uint32_t regval = getreg32(GD32_RCU_APB4EN);
  if ((regval & RCU_APB4EN_SYSCFGEN) == 0)
    {
      modifyreg32(GD32_RCU_APB4EN, 0, RCU_APB4EN_SYSCFGEN);
    }
}

void gd32_syscfg_clock_disable(void)
{
  modifyreg32(GD32_RCU_APB4EN, RCU_APB4EN_SYSCFGEN, 0);
}

#endif /* CONFIG_GD32H7_SYSCFG */
