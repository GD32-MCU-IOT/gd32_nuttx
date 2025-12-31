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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "arm_internal.h"

#include "hardware/gd32e11x_gpio.h"
#include "hardware/gd32e11x_rcu.h"

#include "gd32e11x_gpio.h"

static inline uintptr_t gd32_gpiobase(unsigned int port)
{
	switch (port)
		{
			default:
			case 0:
				return GD32_GPIOA;
			case 1:
				return GD32_GPIOB;
			case 2:
				return GD32_GPIOC;
			case 3:
				return GD32_GPIOD;
			case 4:
				return GD32_GPIOE;
		}
}

static inline void gd32_gpio_clock_enable(unsigned int port)
{
	uint32_t bit;

	switch (port)
		{
			default:
			case 0:
				bit = RCU_APB2EN_PAEN;
				break;
			case 1:
				bit = RCU_APB2EN_PBEN;
				break;
			case 2:
				bit = RCU_APB2EN_PCEN;
				break;
			case 3:
				bit = RCU_APB2EN_PDEN;
				break;
			case 4:
				bit = RCU_APB2EN_PEEN;
				break;
		}

	modifyreg32(GD32_RCU_APB2EN, 0, bit);
}

int gd32_configgpio(uint32_t cfgset)
{
	uintptr_t base;
	uintptr_t ctl;
	uint32_t regval;
	uint32_t shift;
	uint32_t modecnf;
	unsigned int port;
	unsigned int pin;
	unsigned int pos;
	bool input;

	port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	if (port > 4)
		{
			return -EINVAL;
		}

	pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	if (pin > 15)
		{
			return -EINVAL;
		}

	/* Enable AFIO and GPIO clocks (AFIO for remap/EXTI/alternate usage). */

	modifyreg32(GD32_RCU_APB2EN, 0, RCU_APB2EN_AFEN);
	gd32_gpio_clock_enable(port);

	base = gd32_gpiobase(port);

	if (pin < 8)
		{
			ctl = GD32_GPIO_CTL0(base);
			pos = pin;
		}
	else
		{
			ctl = GD32_GPIO_CTL1(base);
			pos = pin - 8;
		}

	shift = (uint32_t)pos << 2; /* 4 bits per pin */
	input = ((cfgset & GPIO_INPUT) != 0);

	/* Decode mode/cnf into the 4-bit CTL field: [3:2]=CNF, [1:0]=MODE */

	if (input)
		{
			modecnf = 0; /* MODE=00 */
		}
	else
		{
			modecnf = (cfgset & GPIO_MODE_MASK) >> GPIO_MODE_SHIFT;
		}

	modecnf |= ((cfgset & GPIO_CNF_MASK) >> GPIO_CNF_SHIFT) << 2;

	regval  = getreg32(ctl);
	regval &= ~(0x0fu << shift);
	regval |= (modecnf & 0x0fu) << shift;
	putreg32(regval, ctl);

	/* Handle output initial value and input pull-up/down via OCTL. */

	if (!input)
		{
			unsigned int cnf = (cfgset & GPIO_CNF_MASK);

			if (cnf != GPIO_CNF_OUTPP && cnf != GPIO_CNF_OUTOD)
				{
					/* Alternate function output: nothing more to do here. */

					return OK;
				}
		}
	else
		{
			if ((cfgset & GPIO_CNF_MASK) != GPIO_CNF_INPULLUD)
				{
					return OK;
				}
		}

	/* Use BOP to set/reset the OCTL bit for initial output value or pull-up. */

	if ((cfgset & GPIO_OUTPUT_SET) != 0)
		{
			putreg32(GPIO_BOP_SET(pin), GD32_GPIO_BOP(base));
		}
	else
		{
			putreg32(GPIO_BOP_CLEAR(pin), GD32_GPIO_BOP(base));
		}

	return OK;
}

int gd32_unconfiggpio(uint32_t cfgset)
{
  /* Return pin to default input floating. */

  cfgset &= ~(GPIO_CNF_MASK | GPIO_MODE_MASK | GPIO_INPUT | GPIO_EXTI);
  cfgset |= (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT);
  return gd32_configgpio(cfgset);
}

void gd32_gpiowrite(uint32_t cfgset, bool value)
{
  unsigned int port;
  unsigned int pin;
  uintptr_t base;

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  base = gd32_gpiobase(port);

  if (value)
    {
      putreg32(GPIO_BOP_SET(pin), GD32_GPIO_BOP(base));
    }
  else
    {
      putreg32(GPIO_BOP_CLEAR(pin), GD32_GPIO_BOP(base));
    }
}
