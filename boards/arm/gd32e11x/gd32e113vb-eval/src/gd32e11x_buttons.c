/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_buttons.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "gd32e11x.h"
#include "gd32e113v_eval.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each button.  This array is indexed by
 * the BUTTON_* definitions in board.h
 */

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BTN_WAKEUP,
  GPIO_BTN_TAMPER,
  GPIO_BTN_USER
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  /* Configure the GPIO pins as inputs */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      gd32_gpio_config(g_buttons[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* A LOW value means that the key is pressed. */

      bool released = gd32_gpio_read(g_buttons[i]);

      /* Accumulate the set of depressed keys */

      if (!released)
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns a
 *   32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is pressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if ((unsigned)id < NUM_BUTTONS)
    {
      irqstate_t flags;

      /* Disable interrupts until we are done.  This guarantees that the
       * following operations are atomic.
       */

      flags = enter_critical_section();

      /* Are we attaching or detaching? */

      if (irqhandler != NULL)
        {
          uint8_t gpio_irq;
          uint8_t gpio_irqnum;

          ret = gd32_gpio_exti_irqnum_get(g_buttons[id], &gpio_irqnum);

          if (ret < 0)
            {
              leave_critical_section(flags);
              return ret;
            }

          ret = gd32_exti_gpioirq_init(g_buttons[id], EXTI_INTERRUPT,
                                       EXTI_TRIG_FALLING, &gpio_irq);

          if (ret < 0)
            {
              leave_critical_section(flags);
              return ret;
            }

          /* Attach and enable the interrupt */

          gd32_exti_gpio_irq_attach(gpio_irq, irqhandler, arg);
          up_enable_irq(gpio_irqnum);
        }
      else
        {
          /* Detach this button's callback only.  Pins that share the
           * same NVIC vector (10-15) are unaffected: the vector is only
           * disabled once every pin sharing it has no callback left.
           */

          gd32_exti_gpio_irq_detach(g_buttons[id]);
        }

      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
