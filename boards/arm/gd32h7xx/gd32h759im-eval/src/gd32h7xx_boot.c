/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/src/gd32h7xx_boot.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "gd32h7xx.h"
#include "gd32h759im_eval.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_boardinitialize
 *
 * Description:
 *   All GD32H7 architectures must provide the following interface to work
 *   with the board-specific logic:
 *
 *   void gd32_boardinitialize(void) - This function is called during
 *     system startup to perform board-specific initialization.  This is
 *     the place to do things like configure GPIO output pins to control
 *     LEDs, configure the SPI chip select GPIO pin, etc.
 *
 *     NOTE: Be very careful about what you do in this function.  Much of
 *     the system has not yet been initialized.
 *
 ****************************************************************************/

void gd32_boardinitialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#ifdef CONFIG_ARCH_BUTTONS
  /* Configure on-board buttons if button support has been selected. */

  board_button_initialize();
#endif

#ifdef CONFIG_GD32H7_SPI
  /* Configure SPI chip select GPIO pins */

  gd32_spidev_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization here if so configured */

  gd32_bringup();
}
#endif