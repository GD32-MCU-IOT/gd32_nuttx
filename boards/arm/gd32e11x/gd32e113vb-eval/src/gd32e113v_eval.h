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
#define GPIO_OUT1         (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_SPEED_50MHZ | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_B | GPIO_CFG_PIN_1)
#define GPIO_INT1         (GPIO_CFG_INPUT | GPIO_CFG_CTL_INFLOAT | GPIO_CFG_EXTI | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_2)

/* ADC
 *
 * GD32E11x ADC channel-to-pin mapping:
 *   ADC0/1 CH0  - PA0
 *   ADC0/1 CH1  - PA1
 *   ADC0/1 CH4  - PA4
 *   ADC0/1 CH8  - PB0
 *   ADC0/1 CH10 - PC0
 *   ADC0/1 CH11 - PC1
 *
 * Note: PC0 is shared with LED1. If ADC is enabled on CH10,
 *       LED1 cannot be used simultaneously.
 */

#define GPIO_ADC0_IN0     (GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_0)
#define GPIO_ADC0_IN1     (GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_1)
#define GPIO_ADC0_IN4     (GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_4)
#define GPIO_ADC0_IN13    (GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN | \
                           GPIO_CFG_PORT_C | GPIO_CFG_PIN_3)

#define GPIO_ADC1_IN8     (GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#define GPIO_ADC1_IN10    (GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN | \
                           GPIO_CFG_PORT_C | GPIO_CFG_PIN_0)
#define GPIO_ADC1_IN11    (GPIO_CFG_INPUT | GPIO_CFG_CTL_AIN | \
                           GPIO_CFG_PORT_C | GPIO_CFG_PIN_1)

/* AT24 Serial EEPROM */

#if defined(CONFIG_MTD_AT24XX) && \
    (defined(CONFIG_GD32E11X_I2C0) || defined(CONFIG_GD32E11X_I2C1)) && \
    defined(CONFIG_GD32E113VB_EVAL_AT24_TEST)
#  define HAVE_AT24       1
#  if defined(CONFIG_GD32E11X_I2C0)
#    define AT24_BUS      0
#  else
#    define AT24_BUS      1
#  endif
#  define AT24_MINOR      0
#endif

#ifdef CONFIG_GD32E11X_TIMER0_PWM
#  define GD32E113VBEVAL_PWMTIMER   0
#else
#  define GD32E113VBEVAL_PWMTIMER   2
#endif

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

/****************************************************************************
 * Name: gd32_gpio_test
 *
 * Description:
 *   Run GPIO tests including output, input, and interrupt tests.
 *   Called automatically during system bringup when enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_GPIO_TEST
int gd32_gpio_test(void);
#endif

/****************************************************************************
 * Name: gd32_usart_test
 *
 * Description:
 *   Run USART tests including write, echo, stress, and format tests.
 *   Called automatically during system bringup when enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_USART_TEST
int gd32_usart_test(void);
#endif

/****************************************************************************
 * Name: gd32_gpio_initialize
 *
 * Description:
 *   Initialize and register GPIO character devices.
 *   This function should be called during board initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int gd32_gpio_initialize(void);
#endif

#ifdef HAVE_AT24
int gd32_at24_wr_test(int minor);
#  ifdef CONFIG_GD32E11X_I2C_DMA
int gd32_at24_multimsg_dma_test(int minor);
#  endif
#endif

/****************************************************************************
 * Name: gd32_watchdog_initialize
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *   This function initializes the watchdog timer and starts a kernel thread
 *   to automatically feed the watchdog.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E113VB_EVAL_WDG_THREAD
int gd32_watchdog_initialize(void);
#endif

/****************************************************************************
 * Name: gd32_wdt_start
 *
 * Description:
 *   Start the watchdog timer on the specified device.
 *
 * Input Parameters:
 *   devpath - Path to watchdog device (e.g., "/dev/watchdog0")
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG)
int gd32_wdt_start(const char *devpath);
#endif

/****************************************************************************
 * Name: gd32_wdt_stop
 *
 * Description:
 *   Stop the watchdog timer on the specified device.
 *
 * Input Parameters:
 *   devpath - Path to watchdog device (e.g., "/dev/watchdog0")
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG)
int gd32_wdt_stop(const char *devpath);
#endif

/****************************************************************************
 * Name: gd32_wdt_keepalive
 *
 * Description:
 *   Feed (keepalive) the watchdog timer on the specified device.
 *
 * Input Parameters:
 *   devpath - Path to watchdog device (e.g., "/dev/watchdog0")
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG)
int gd32_wdt_keepalive(const char *devpath);
#endif

/****************************************************************************
 * Name: gd32_wdt_settimeout
 *
 * Description:
 *   Set the watchdog timeout value on the specified device.
 *
 * Input Parameters:
 *   devpath - Path to watchdog device (e.g., "/dev/watchdog0")
 *   timeout - Timeout value in milliseconds
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG)
int gd32_wdt_settimeout(const char *devpath, uint32_t timeout);
#endif

/****************************************************************************
 * Name: gd32_wdt_setwindow
 *
 * Description:
 *   Set the watchdog window value (WWDGT only).
 *   This sets the upper limit for feeding the watchdog. Feeding outside
 *   the window will cause a reset.
 *
 * Input Parameters:
 *   devpath - Path to watchdog device (should be "/dev/watchdog1" for WWDGT)
 *   window  - Window value (64-127 for WWDGT)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG)
int gd32_wdt_setwindow(const char *devpath, uint32_t window);
#endif

/****************************************************************************
 * Name: gd32_wwdgt_test
 *
 * Description:
 *   Test the window watchdog (WWDGT) with configurable timeout and window.
 *   Demonstrates the WWDGT window mechanism by feeding the watchdog
 *   multiple times within the safe window.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_GD32E11X_WWDG)
int gd32_wwdgt_test(void);
#endif

/****************************************************************************
 * Name: gd32_crc_test
 *
 * Description:
 *   Test the CRC hardware by computing CRC-32 for known test vectors.
 *   The GD32E11x CRC unit uses the Ethernet CRC-32 polynomial (0x4C11DB7).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_GD32E11X_CRC)
int gd32_crc_test(void);
#endif

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

/****************************************************************************
 * Name: gd32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#if defined(CONFIG_ADC) && \
    (defined(CONFIG_GD32E11X_ADC0) || defined(CONFIG_GD32E11X_ADC1))
int gd32_adc_setup(void);
#endif

/****************************************************************************
 * Name: gd32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int gd32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: gd32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver and register /dev/timerX.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device (e.g., /dev/timer0)
 *   timer   - The timer number (0-13 for GD32E11X)
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int gd32_timer_driver_setup(const char *devpath, int timer);
#endif

/* Timer selection for timer test.
 *
 * Default: TIMER5 (basic timer, no GPIO pins needed, no conflicts).
 * TIMER5 is ideal for interval timing tests.  For tests requiring
 * channel I/O (input capture / output compare), use a general timer.
 *
 *   Available timers (avoid TIMER2 if PWM is also enabled):
 *     TIMER5 - Basic timer, no channels (default for timer test)
 *     TIMER6 - Basic timer, no channels
 *     TIMER1 - General 4-ch (PA0 conflicts with WAKEUP button)
 *     TIMER3 - General 4-ch (PB6/PB7/PB8/PB9)
 *     TIMER4 - General 4-ch (PA0 conflicts)
 */

#ifndef GD32E113VBEVAL_TIMER
#  define GD32E113VBEVAL_TIMER      5
#endif

#ifndef GD32E113VBEVAL_TIMER_DEVPATH
#  define GD32E113VBEVAL_TIMER_DEVPATH "/dev/timer0"
#endif

#endif /* __BOARDS_ARM_GD32E11X_GD32E113VB_EVAL_SRC_GD32E113V_EVAL_H */