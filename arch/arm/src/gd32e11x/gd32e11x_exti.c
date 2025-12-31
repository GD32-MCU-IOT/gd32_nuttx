/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_exti.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "arm_internal.h"

#include "gd32e11x_exti.h"
#include "gd32e11x_gpio.h"
#include "gd32e11x_syscfg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_gpio_irq_s
{
	xcpt_t irqhandler;
	void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_exti_gpio_irqs[16] =
{
	GD32_IRQ_EXTI0, GD32_IRQ_EXTI1, GD32_IRQ_EXTI2, GD32_IRQ_EXTI3,
	GD32_IRQ_EXTI4, GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI5_9,
	GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI10_15, GD32_IRQ_EXTI10_15,
	GD32_IRQ_EXTI10_15, GD32_IRQ_EXTI10_15, GD32_IRQ_EXTI10_15,
	GD32_IRQ_EXTI10_15
};

/* Interrupt handlers attached to each EXTI (GPIO lines 0..15 only) */

static struct gd32_gpio_irq_s g_gpio_irq_s[16];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gd32_exti0_irqhandler(int irq, void *context, void *arg)
{
	int ret = OK;

	gd32_exti_interrupt_flag_clear(EXTI_0);

	if (g_gpio_irq_s[0].irqhandler != NULL)
		{
			ret = g_gpio_irq_s[0].irqhandler(irq, context, g_gpio_irq_s[0].arg);
		}

	return ret;
}

static int gd32_exti1_irqhandler(int irq, void *context, void *arg)
{
	int ret = OK;

	gd32_exti_interrupt_flag_clear(EXTI_1);

	if (g_gpio_irq_s[1].irqhandler != NULL)
		{
			ret = g_gpio_irq_s[1].irqhandler(irq, context, g_gpio_irq_s[1].arg);
		}

	return ret;
}

static int gd32_exti2_irqhandler(int irq, void *context, void *arg)
{
	int ret = OK;

	gd32_exti_interrupt_flag_clear(EXTI_2);

	if (g_gpio_irq_s[2].irqhandler != NULL)
		{
			ret = g_gpio_irq_s[2].irqhandler(irq, context, g_gpio_irq_s[2].arg);
		}

	return ret;
}

static int gd32_exti3_irqhandler(int irq, void *context, void *arg)
{
	int ret = OK;

	gd32_exti_interrupt_flag_clear(EXTI_3);

	if (g_gpio_irq_s[3].irqhandler != NULL)
		{
			ret = g_gpio_irq_s[3].irqhandler(irq, context, g_gpio_irq_s[3].arg);
		}

	return ret;
}

static int gd32_exti4_irqhandler(int irq, void *context, void *arg)
{
	int ret = OK;

	gd32_exti_interrupt_flag_clear(EXTI_4);

	if (g_gpio_irq_s[4].irqhandler != NULL)
		{
			ret = g_gpio_irq_s[4].irqhandler(irq, context, g_gpio_irq_s[4].arg);
		}

	return ret;
}

static int gd32_exti5_9_irqhandler(int irq, void *context, void *arg)
{
	int ret = OK;
	uint32_t pd_flag;
	uint8_t irq_pin;

	pd_flag = getreg32(GD32_EXTI_PD);

	for (irq_pin = 5; irq_pin <= 9; irq_pin++)
		{
			uint32_t pd_bit = (1u << irq_pin);

			if ((pd_flag & pd_bit) != 0)
				{
					gd32_exti_interrupt_flag_clear(pd_bit);

					if (g_gpio_irq_s[irq_pin].irqhandler != NULL)
						{
							ret = g_gpio_irq_s[irq_pin].irqhandler(irq, context,
																										 g_gpio_irq_s[irq_pin].arg);
						}
				}
		}

	return ret;
}

static int gd32_exti10_15_irqhandler(int irq, void *context, void *arg)
{
	int ret = OK;
	uint32_t pd_flag;
	uint8_t irq_pin;

	pd_flag = getreg32(GD32_EXTI_PD);

	for (irq_pin = 10; irq_pin <= 15; irq_pin++)
		{
			uint32_t pd_bit = (1u << irq_pin);

			if ((pd_flag & pd_bit) != 0)
				{
					gd32_exti_interrupt_flag_clear(pd_bit);

					if (g_gpio_irq_s[irq_pin].irqhandler != NULL)
						{
							ret = g_gpio_irq_s[irq_pin].irqhandler(irq, context,
																										 g_gpio_irq_s[irq_pin].arg);
						}
				}
		}

	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gd32_exti_gpioirq_init(uint32_t cfgset, uint8_t exti_mode,
													 uint8_t trig_type, uint8_t *irqnum)
{
	uint32_t exti_linex;
	uint8_t port;
	uint8_t pin;

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

	if (irqnum != NULL)
		{
			*irqnum = pin;
		}

	exti_linex = (1u << pin);

	gd32_syscfg_clock_enable();

	/* Connect EXTI line to selected GPIO port */

	gd32_syscfg_exti_line_config(port, pin);

	/* Configure the interrupt/event */

	gd32_exti_init(exti_linex, exti_mode, trig_type);
	gd32_exti_interrupt_flag_clear(exti_linex);

	return OK;
}

int gd32_exti_gpio_irq_attach(uint8_t irqpin, xcpt_t irqhandler, void *arg)
{
	if (irqpin > 15)
		{
			return -EINVAL;
		}

	switch (g_exti_gpio_irqs[irqpin])
		{
			case GD32_IRQ_EXTI0:
				irq_attach(GD32_IRQ_EXTI0, gd32_exti0_irqhandler, NULL);
				break;

			case GD32_IRQ_EXTI1:
				irq_attach(GD32_IRQ_EXTI1, gd32_exti1_irqhandler, NULL);
				break;

			case GD32_IRQ_EXTI2:
				irq_attach(GD32_IRQ_EXTI2, gd32_exti2_irqhandler, NULL);
				break;

			case GD32_IRQ_EXTI3:
				irq_attach(GD32_IRQ_EXTI3, gd32_exti3_irqhandler, NULL);
				break;

			case GD32_IRQ_EXTI4:
				irq_attach(GD32_IRQ_EXTI4, gd32_exti4_irqhandler, NULL);
				break;

			case GD32_IRQ_EXTI5_9:
				irq_attach(GD32_IRQ_EXTI5_9, gd32_exti5_9_irqhandler, NULL);
				break;

			case GD32_IRQ_EXTI10_15:
				irq_attach(GD32_IRQ_EXTI10_15, gd32_exti10_15_irqhandler, NULL);
				break;

			default:
				break;
		}

	g_gpio_irq_s[irqpin].irqhandler = irqhandler;
	g_gpio_irq_s[irqpin].arg        = arg;

	return OK;
}

void gd32_exti_init(uint32_t linex, uint8_t exti_mode, uint8_t trig_type)
{
	modifyreg32(GD32_EXTI_INTEN, linex, 0);
	modifyreg32(GD32_EXTI_EVEN, linex, 0);
	modifyreg32(GD32_EXTI_RTEN, linex, 0);
	modifyreg32(GD32_EXTI_FTEN, linex, 0);

	switch (exti_mode)
		{
			case EXTI_INTERRUPT:
				modifyreg32(GD32_EXTI_INTEN, 0, linex);
				break;

			case EXTI_EVENT:
				modifyreg32(GD32_EXTI_EVEN, 0, linex);
				break;

			default:
				break;
		}

	switch (trig_type)
		{
			case EXTI_TRIG_RISING:
				modifyreg32(GD32_EXTI_RTEN, 0, linex);
				break;

			case EXTI_TRIG_FALLING:
				modifyreg32(GD32_EXTI_FTEN, 0, linex);
				break;

			case EXTI_TRIG_BOTH:
				modifyreg32(GD32_EXTI_RTEN, 0, linex);
				modifyreg32(GD32_EXTI_FTEN, 0, linex);
				break;

			case EXTI_TRIG_NONE:
			default:
				break;
		}
}

int gd32_gpio_exti_linex_get(uint32_t cfgset, uint32_t *linex)
{
	uint8_t port;
	uint8_t pin;

	if (linex == NULL)
		{
			return -EINVAL;
		}

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

	*linex = (1u << pin);
	return OK;
}

int gd32_gpio_exti_irqnum_get(uint32_t cfgset, uint8_t *irqnum)
{
	uint8_t port;
	uint8_t pin;

	if (irqnum == NULL)
		{
			return -EINVAL;
		}

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

	*irqnum = g_exti_gpio_irqs[pin];
	return OK;
}

void gd32_exti_interrupt_enable(uint32_t linex)
{
	modifyreg32(GD32_EXTI_INTEN, 0, linex);
}

void gd32_exti_interrupt_disable(uint32_t linex)
{
	modifyreg32(GD32_EXTI_INTEN, linex, 0);
}

void gd32_exti_event_enable(uint32_t linex)
{
	modifyreg32(GD32_EXTI_EVEN, 0, linex);
}

void gd32_exti_event_disable(uint32_t linex)
{
	modifyreg32(GD32_EXTI_EVEN, linex, 0);
}

void gd32_exti_software_interrupt_enable(uint32_t linex)
{
	modifyreg32(GD32_EXTI_SWIEV, 0, linex);
}

void gd32_exti_software_interrupt_disable(uint32_t linex)
{
	modifyreg32(GD32_EXTI_SWIEV, linex, 0);
}

bool gd32_exti_interrupt_flag_get(uint32_t linex)
{
	uint32_t pd;
	uint32_t inten;

	pd    = (getreg32(GD32_EXTI_PD) & linex);
	inten = (getreg32(GD32_EXTI_INTEN) & linex);

	return ((pd & inten) != 0);
}

void gd32_exti_interrupt_flag_clear(uint32_t linex)
{
	putreg32(linex, GD32_EXTI_PD);
}
