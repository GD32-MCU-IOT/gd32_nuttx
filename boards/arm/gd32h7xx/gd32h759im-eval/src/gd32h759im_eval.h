/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/src/gd32h759im_eval.h
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

#ifndef __BOARDS_ARM_GD32H7_GD32H759IM_EVAL_SRC_GD32H759IM_EVAL_H
#define __BOARDS_ARM_GD32H7_GD32H759IM_EVAL_SRC_GD32H759IM_EVAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define GD32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define GD32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

#ifdef CONFIG_FS_NXFFS
#  ifndef CONFIG_GD32F4_NXFFS_MOUNTPT
#    define CONFIG_GD32F4_NXFFS_MOUNTPT "/mnt/gd32nxffs"
#  endif
#endif

/* GD32H759IM GPIO Pin Definitions ******************************************/

/* LED
 *
 * The GD32H759IM-EVAL board has three LEDs, LED1, LED2, and LED3, that can be
 * controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LED1       (GPIO_CFG_PORT_F | GPIO_CFG_OUTPUT_RESET | \
                         GPIO_CFG_SPEED_85MHZ | GPIO_PIN10_OUTPUT)
#define GPIO_LED2       (GPIO_CFG_PORT_A | GPIO_CFG_OUTPUT_RESET | \
                         GPIO_CFG_SPEED_85MHZ | GPIO_PIN6_OUTPUT)

#define LED1            GPIO_LED1
#define LED2            GPIO_LED2
#define LED3            GPIO_LED3

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The GD32H759IM Eval board has User, Tamper, and Wakeup key, the are
 * connected to GPIO PF8, PC13 and PA0.
 * A low value will be sensed when the button is depressed.
 *
 * Note:
 *   That the EXTI is included in the definition to enable
 *       an interrupt on this IO.
 */

#define GPIO_BTN_USER      (GPIO_CFG_PORT_F | GPIO_CFG_EXTI | GPIO_PIN8_INPUT)
#define GPIO_BTN_TAMPER    (GPIO_CFG_PORT_C | GPIO_CFG_EXTI | GPIO_PIN13_INPUT)
#define GPIO_BTN_WAKEUP    (GPIO_CFG_PORT_A | GPIO_CFG_EXTI | GPIO_PIN0_INPUT)


/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#define GPIO_OUT1         (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_60MHZ | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_1)
#define GPIO_INT1         (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_2)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int gd32_bringup(void);

/****************************************************************************
 * Name: gd32_usart_setup
 *
 * Description:
 *   Configure USART.
 *
 ****************************************************************************/

#ifdef HAVE_USART_DEVICE
void gd32_usart_setup(void);
#endif

/****************************************************************************
 * Name: gd32_led_initialize
 *
 * Description:
 *   This function is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, LED initialization() is called from
 *   board-specific initialization and should, therefore, have the name
 *   <arch>_led_initialize().  This function is, however, named
 *   board_autoled_initialize() to conform with the LED interface defined in
 *   include/nuttx/board.h.  If CONFIG_ARCH_LEDS is defined, this function
 *   will be called automatically during board initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void);
#endif

/****************************************************************************
 * Name: gd32_button_initialize
 *
 * Description:
 *   This function is called very early in initialization to perform board-
 *   specific initialization of button-related resources.  This includes
 *   such things as, for example, configure GPIO pins to sense button
 *   status.
 *
 *   NOTE: In most architectures, button initialization() is called from
 *   board-specific initialization and should, therefore, have the name
 *   <arch>_button_initialize().  This function is, however, named
 *   board_button_initialize() to conform with the button interface defined
 *   in include/nuttx/board.h.  If CONFIG_ARCH_BUTTONS is defined, this
 *   function will be called automatically during board initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void board_button_initialize(void);
#endif

#endif /* __BOARDS_ARM_GD32H7_GD32H759IM_EVAL_SRC_GD32H759IM_EVAL_H */