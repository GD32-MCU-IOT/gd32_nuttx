/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e113v_eval.h
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

#ifndef __BOARDS_ARM_GD32E11X_GD32E113VB_EVAL_SRC_GD32E113V_EVAL_H
#define __BOARDS_ARM_GD32E11X_GD32E113VB_EVAL_SRC_GD32E113V_EVAL_H

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

/* GD32E113V GPIO Pin Definitions *******************************************/

/* LEDs
 *
 * The GD32E113VB-EVAL board has four LEDs, LED1-LED4, that can be
 * controlled by software.
 * The following definitions assume the default configuration.
 *
 * LED1 - PC0, LED2 - PC2, LED3 - PE0, LED4 - PE1
 */

#define GPIO_LED1       (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_SPEED_50MHZ | \
                         GPIO_CFG_OUTPUT_CLEAR | GPIO_CFG_PORT_C | GPIO_CFG_PIN_0)
#define GPIO_LED2       (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_SPEED_50MHZ | \
                         GPIO_CFG_OUTPUT_CLEAR | GPIO_CFG_PORT_C | GPIO_CFG_PIN_2)
#define GPIO_LED3       (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_SPEED_50MHZ | \
                         GPIO_CFG_OUTPUT_CLEAR | GPIO_CFG_PORT_E | GPIO_CFG_PIN_0)
#define GPIO_LED4       (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_SPEED_50MHZ | \
                         GPIO_CFG_OUTPUT_CLEAR | GPIO_CFG_PORT_E | GPIO_CFG_PIN_1)

#define LED1            GPIO_LED1
#define LED2            GPIO_LED2
#define LED3            GPIO_LED3
#define LED4            GPIO_LED4

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The GD32E113V Eval board has Wakeup, Tamper, and User key, they are
 * connected to GPIO PA0, PC13 and PB14.
 * A low value will be sensed when the button is depressed.
 *
 * Note:
 *   That the EXTI is included in the definition to enable
 *       an interrupt on this IO.
 */

#define GPIO_BTN_WAKEUP    (GPIO_CFG_INPUT | GPIO_CFG_CTL_INFLOAT | GPIO_CFG_EXTI | \
                            GPIO_CFG_PORT_A | GPIO_CFG_PIN_0)
#define GPIO_BTN_TAMPER    (GPIO_CFG_INPUT | GPIO_CFG_CTL_INFLOAT | GPIO_CFG_EXTI | \
                            GPIO_CFG_PORT_C | GPIO_CFG_PIN_13)
#define GPIO_BTN_USER      (GPIO_CFG_INPUT | GPIO_CFG_CTL_INFLOAT | GPIO_CFG_EXTI | \
                            GPIO_CFG_PORT_B | GPIO_CFG_PIN_14)

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_CFG_INPUT | GPIO_CFG_CTL_INFLOAT | GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#define GPIO_OUT1         (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_MODE_OSPEED_50MHZ | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_B | GPIO_CFG_PIN_1)
#define GPIO_INT1         (GPIO_CFG_INPUT | GPIO_CFG_CTL_INFLOAT | GPIO_CFG_EXTI | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_2)

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
 * Name: gd32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI
void gd32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: gd32_usbinitialize
 *
 * Description:
 *   Called from gd32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_USBFS
void gd32_usbinitialize(void);
#endif

#endif /* __BOARDS_ARM_GD32E11X_GD32E113VB_EVAL_SRC_GD32E113V_EVAL_H */

/* GD25 SPI FLASH */
#if defined(CONFIG_MTD_GD25) && defined(CONFIG_GD32E11X_SPI0)
#  define HAVE_GD25  1
#  define SPI_FLASH_CSNUM 0
#endif

#if defined(HAVE_GD25) && defined(CONFIG_GD32E113VB_EVAL_GD25_BLOCKMOUNT)
#  if defined(CONFIG_GD32E113VB_EVAL_GD25_LITTLEFS)
#    define GD25_MOUNT_FSTYPE "littlefs"
#  endif
#endif

/* Function prototypes */
#ifdef HAVE_GD25
int gd32_gd25_automount(int minor);
#endif
