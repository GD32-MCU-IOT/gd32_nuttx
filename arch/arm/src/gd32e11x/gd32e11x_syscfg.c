/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_syscfg.c
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

#include <assert.h>
#include <stdint.h>

#include "arm_internal.h"

#include "chip.h"
#include "hardware/gd32e11x_rcu.h"
#include "hardware/gd32e11x_syscfg.h"

#include "gd32e11x_syscfg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_syscfg_clock_enable
 *
 * Description:
 *   This function enable the AFIO clock.
 *
 ****************************************************************************/

void gd32_syscfg_clock_enable(void)
{
  modifyreg32(GD32_RCU_APB2EN, 0, RCU_APB2EN_AFEN);
}

/****************************************************************************
 * Name: gd32_syscfg_clock_disable
 *
 * Description:
 *   This function disable the AFIO clock.
 *
 ****************************************************************************/

void gd32_syscfg_clock_disable(void)
{
  modifyreg32(GD32_RCU_APB2EN, RCU_APB2EN_AFEN, 0);
}

/****************************************************************************
 * Function:  gd32_gpio_remap
 *
 * Description:
 *   Configure GPIO remap for alternate functions.
 *
 ****************************************************************************/

void gd32_gpio_remap(void)
{
  uint32_t pcf0_val = 0;
  uint32_t pcf1_val = 0;

  /* Remap according to the configuration within .config file */

#ifdef CONFIG_GD32_SPI0_REMAP
  pcf0_val |= AFIO_PCF0_SPI0_REMAP;
#endif
#ifdef CONFIG_GD32_I2C0_REMAP
  pcf0_val |= AFIO_PCF0_I2C0_REMAP;
#endif
#ifdef CONFIG_GD32_USART0_REMAP
  pcf0_val |= AFIO_PCF0_USART0_REMAP;
#endif
#ifdef CONFIG_GD32_USART1_REMAP
  pcf0_val |= AFIO_PCF0_USART1_REMAP;
#endif
#ifdef CONFIG_GD32_USART2_PARTIAL_REMAP
  pcf0_val |= PCF0_USART2_REMAP(1);
#endif
#ifdef CONFIG_GD32_USART2_FULL_REMAP
  pcf0_val |= PCF0_USART2_REMAP(3);
#endif
#ifdef CONFIG_GD32_TIMER0_PARTIAL_REMAP
  pcf0_val |= PCF0_TIMER0_REMAP(1);
#endif
#ifdef CONFIG_GD32_TIMER0_FULL_REMAP
  pcf0_val |= PCF0_TIMER0_REMAP(3);
#endif
#ifdef CONFIG_GD32_TIMER1_PARTIAL_REMAP0
  pcf0_val |= PCF0_TIMER1_REMAP(1);
#endif
#ifdef CONFIG_GD32_TIMER1_PARTIAL_REMAP1
  pcf0_val |= PCF0_TIMER1_REMAP(2);
#endif
#ifdef CONFIG_GD32_TIMER1_FULL_REMAP
  pcf0_val |= PCF0_TIMER1_REMAP(3);
#endif
#ifdef CONFIG_GD32_TIMER2_PARTIAL_REMAP
  pcf0_val |= PCF0_TIMER2_REMAP(2);
#endif
#ifdef CONFIG_GD32_TIMER2_FULL_REMAP
  pcf0_val |= PCF0_TIMER2_REMAP(3);
#endif
#ifdef CONFIG_GD32_TIMER3_REMAP
  pcf0_val |= AFIO_PCF0_TIMER3_REMAP;
#endif
#ifdef CONFIG_GD32_PD01_REMAP
  pcf0_val |= AFIO_PCF0_PD01_REMAP;
#endif
#ifdef CONFIG_GD32_SPI2_REMAP
  pcf0_val |= AFIO_PCF0_SPI2_REMAP;
#endif
#ifdef CONFIG_GD32_TIMER1ITI1_REMAP
  pcf0_val |= AFIO_PCF0_TIMER1ITI1_REMAP;
#endif
#ifdef CONFIG_GD32_SWJ_NONJTRST_ENABLE
  pcf0_val |= PCF0_SWJ_CFG(1);
#endif
#ifdef CONFIG_GD32_SWJ_SWDP_ENABLE
  pcf0_val |= PCF0_SWJ_CFG(2);
#endif
#ifdef CONFIG_GD32_SWJ_DISABLE
  pcf0_val |= PCF0_SWJ_CFG(4);
#endif

#ifdef CONFIG_GD32_TIMER8_REMAP
  pcf1_val |= AFIO_PCF1_TIMER8_REMAP;
#endif
#ifdef CONFIG_GD32_EXMC_NADV_REMAP
  pcf1_val |= AFIO_PCF1_EXMC_NADV;
#endif
#ifdef CONFIG_GD32_CTC_REMAP
  pcf1_val |= PCF1_CTC_REMAP(1);
#endif

  gd32_syscfg_clock_enable();

  if (pcf0_val != 0)
    {
      putreg32(pcf0_val, GD32_AFIO_PCF0);
    }

  if (pcf1_val != 0)
    {
      putreg32(pcf1_val, GD32_AFIO_PCF1);
    }
}

/****************************************************************************
 * Name: gd32_syscfg_exti_line_config
 *
 * Description:
 *   This function configure the GPIO pin as EXTI Line.
 *
 ****************************************************************************/

void gd32_syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin)
{
  uintptr_t regaddr;
  uint32_t shift;
  uint32_t mask;

  DEBUGASSERT(exti_pin < 16);

  if (exti_pin < 4)
    {
      /* Select EXTI0/EXTI1/EXTI2/EXTI3 */

      regaddr = GD32_AFIO_EXTISS0;
    }
  else if (exti_pin < 8)
    {
      /* Select EXTI4/EXTI5/EXTI6/EXTI7 */

      regaddr = GD32_AFIO_EXTISS1;
    }
  else if (exti_pin < 12)
    {
      /* Select EXTI8/EXTI9/EXTI10/EXTI11 */

      regaddr = GD32_AFIO_EXTISS2;
    }
  else
    {
      /* Select EXTI12/EXTI13/EXTI14/EXTI15 */

      regaddr = GD32_AFIO_EXTISS3;
    }

  shift = AFIO_EXTISS_SHIFT(exti_pin);
  mask  = AFIO_EXTISS_MASK(exti_pin);

  modifyreg32(regaddr, mask, ((uint32_t)exti_port << shift));
}

/****************************************************************************
 * Function:  gd32_gpio_compensation_config
 *
 * Description:
 *   Enable or disable the I/O compensation cell.
 *   The I/O compensation cell can be used to optimize I/O speed when the
 *   supply voltage is low.
 *
 * Input Parameters:
 *   compensation - GPIO_COMPENSATION_ENABLE or GPIO_COMPENSATION_DISABLE
 *
 ****************************************************************************/

void gd32_gpio_compensation_config(uint32_t compensation)
{
  if (compensation == GPIO_COMPENSATION_ENABLE)
    {
      modifyreg32(GD32_AFIO_CPSCTL, 0, AFIO_CPSCTL_CPS_EN);
    }
  else
    {
      modifyreg32(GD32_AFIO_CPSCTL, AFIO_CPSCTL_CPS_EN, 0);
    }
}

/****************************************************************************
 * Function:  gd32_gpio_compensation_flag_get
 *
 * Description:
 *   Check if I/O compensation cell is ready.
 *
 * Returned Value:
 *   true if the I/O compensation cell is ready, false otherwise
 *
 ****************************************************************************/

bool gd32_gpio_compensation_flag_get(void)
{
  return (getreg32(GD32_AFIO_CPSCTL) & AFIO_CPSCTL_CPS_RDY) != 0;
}
