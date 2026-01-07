/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_gpio.c
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
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32e11x_syscfg.h"
#include "gd32e11x_gpio.h"
#include "gd32e11x.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_configgpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO block */

const uint32_t g_gpio_base[GD32_NGPIO_PORTS] =
{
#if GD32_NGPIO_PORTS > 0
  GD32_GPIOA,
#endif
#if GD32_NGPIO_PORTS > 1
  GD32_GPIOB,
#endif
#if GD32_NGPIO_PORTS > 2
  GD32_GPIOC,
#endif
#if GD32_NGPIO_PORTS > 3
  GD32_GPIOD,
#endif
#if GD32_NGPIO_PORTS > 4
  GD32_GPIOE,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  gd32_gpio_clock_enable
 *
 * Description:
 *   Enable GPIO port clock
 *
 ****************************************************************************/

static void gd32_gpio_clock_enable(uint32_t port_base)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which GPIO port to configure */

  switch (port_base)
    {
    default:
      return;
#if GD32_NGPIO_PORTS > 0
    case GD32_GPIOA:
      rcu_en = RCU_APB2EN_PAEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 1
    case GD32_GPIOB:
      rcu_en = RCU_APB2EN_PBEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 2
    case GD32_GPIOC:
      rcu_en = RCU_APB2EN_PCEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 3
    case GD32_GPIOD:
      rcu_en = RCU_APB2EN_PDEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 4
    case GD32_GPIOE:
      rcu_en = RCU_APB2EN_PEEN;
      break;
#endif
    }

  regaddr = GD32_RCU_APB2EN;

  /* Check clock if already enable. */

  if (rcu_en != (rcu_en & getreg32(regaddr)))
    {
      /* Enable APB2 clock for GPIO */

      modifyreg32(regaddr, 0, rcu_en);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Return value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 *
 ****************************************************************************/

int gd32_gpio_config(uint32_t cfgset)
{
  uintptr_t port_base;
  uintptr_t ctl;
  uint32_t regval;
  uint32_t modectl;
  unsigned int port;
  unsigned int pin;
  unsigned int pos;
  irqstate_t flags;
  bool input;

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;
  if (pin > 15)
    {
      return -EINVAL;
    }

  /* Get the port base address */

  port_base = g_gpio_base[port];

  /* Enable GPIO port clock */

  gd32_gpio_clock_enable(port_base);

  if (pin < 8)
    {
      ctl = GD32_GPIO_CTL0(port_base);
      pos = pin;
    }
  else
    {
      ctl = GD32_GPIO_CTL1(port_base);
      pos = pin - 8;
    }

  input = ((cfgset & GPIO_CFG_INPUT) != 0);

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = spin_lock_irqsave(&g_configgpio_lock);

  if (input)
    {
      /* Input mode: MODE bits = 00 */

      modectl =  0;
    }
  else
    {
      /* Output mode: MODE bits = 01, 10, or 11 */

      modectl = (cfgset & GPIO_CFG_MODE_MASK) >> GPIO_CFG_MODE_SHIFT;

      /* Handle extended speed bit if present */

      if ((cfgset & GPIO_CFG_OSPEED_A) != 0)
        {
          putreg32(GPIO_SPD_SET(pin), GD32_GPIO_SPD(port_base));
        }
    }

  /* Input or Output configuration */

  modectl |= ((cfgset & GPIO_CFG_CTL_MASK) >> GPIO_CFG_CTL_SHIFT) << 2;

  regval  = getreg32(ctl);
  regval &= ~(GPIO_CTL0_MDCTL_MASK(pos));
  regval |= (modectl << GPIO_CTL0_MDCTL_SHIFT(pos));
  putreg32(regval, ctl);

  /* Handle output initial value and input pull-up/down via OCTL. */

  if (!input)
    {
      uint32_t cfgctl = cfgset & GPIO_CFG_CTL_MASK;

      /* For output modes, check if it's a GPIO output
       * or alternate function
       */

      if (cfgctl != GPIO_CFG_CTL_OUTPP && cfgctl != GPIO_CFG_CTL_OUTOD)
        {
          /* Its an alternate function pin... we can return early */

          spin_unlock_irqrestore(&g_configgpio_lock, flags);
          return OK;
        }
    }
  else
    {
      if ((cfgset & GPIO_CFG_CTL_MASK) != GPIO_CFG_CTL_IPUD)
        {
          /* Neither... we can return early */

          spin_unlock_irqrestore(&g_configgpio_lock, flags);
          return OK;
        }
    }

  /* Use BOP/BC to set/clear the OCTL bit for initial
   * output value or pull-up.
   */

  if ((cfgset & GPIO_CFG_OUTPUT_SET) != 0)
    {
      putreg32(GPIO_BOP_SET(pin), GD32_GPIO_BOP(port_base));
    }
  else
    {
      putreg32(GPIO_BC_SET(pin), GD32_GPIO_BC(port_base));
    }

  spin_unlock_irqrestore(&g_configgpio_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: gd32_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin,
 *   set it into default HiZ state.
 *
 * Returned Value:
 *  OK on success
 *  A negated errno value on invalid port or mode
 *
 ****************************************************************************/

int gd32_gpio_unconfig(uint32_t cfgset)
{
  /* Return pin to default input floating. */

  cfgset &= GPIO_CFG_PORT_MASK | GPIO_CFG_PIN_MASK;
  cfgset |= (GPIO_CFG_INPUT | GPIO_CFG_CTL_INFLOAT);
  return gd32_gpio_config(cfgset);
}

/****************************************************************************
 * Name: gd32_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void gd32_gpio_write(uint32_t cfgset, bool value)
{
  unsigned int port;
  unsigned int pin;
  uintptr_t port_base;

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;

  if (port < GD32_NGPIO_PORTS)
    {
      /* Get the port base address */

      port_base = g_gpio_base[port];

      /* Get the pin number  */

      pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

      if (value)
        {
          putreg32(GPIO_BOP_SET(pin), GD32_GPIO_BOP(port_base));
        }
      else
        {
          putreg32(GPIO_BC_SET(pin), GD32_GPIO_BC(port_base));
        }
    }
}

/****************************************************************************
 * Name: gd32_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool gd32_gpio_read(uint32_t cfgset)
{
  unsigned int port;
  unsigned int pin;
  uintptr_t port_base;

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;

  if (port < GD32_NGPIO_PORTS)
    {
      /* Get the port base address */

      port_base = g_gpio_base[port];

      /* Get the pin number */

      pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

      /* Return the input state of the pin */

      return (getreg32(GD32_GPIO_ISTAT(port_base)) & (1 << pin)) != 0;
    }

  return 0;
}

