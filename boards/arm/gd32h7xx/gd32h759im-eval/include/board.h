/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/include/board.h
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

#ifndef __BOARDS_ARM_GD32H7_GD32H759IM_EVAL_INCLUDE_BOARD_H
#define __BOARDS_ARM_GD32H7_GD32H759IM_EVAL_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#define GD32_BOARD_SYSCLK_PLL_HXTAL

/* Do not include GD32H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The GD32H759IM-EVAL board features a single 25MHz crystal.
 *
 * This is the default configuration:
 *   System clock source           : PLL (HXTAL)
 *   SYSCLK(Hz)                    : 600000000    Determined by PLL config
 *   HCLK(Hz)                      : 600000000    (GD32_SYSCLK_FREQUENCY)
 *   AHB Prescaler                 : 2            (GD32_RCU_CFG0_AHB_PSC)
 *   APB4 Prescaler                : 4            (GD32_RCU_CFG0_APB4_PSC)
 *   APB3 Prescaler                : 4            (GD32_RCU_CFG0_APB3_PSC)
 *   APB2 Prescaler                : 2            (GD32_RCU_CFG0_APB2_PSC)
 *   APB1 Prescaler                : 4            (GD32_RCU_CFG0_APB1_PSC)
 *   HXTAL value(Hz)               : 25000000     (GD32_BOARD_XTAL)
 *   PLL0PSC                       : 5            (GD32_PLL_PLL0PSC)
 *   PLL0N                         : (120-1)      (GD32_RCU_PLL_PLL0N)
 *   PLL0P                         : (1-1)        (GD32_PLL_PLL0P)
 *   PLL0Q                         : (2-1)        (GD32_PLL_PLL0Q)
 *   PLL0R                         : (2-1)        (GD32_PLL_PLL0R)
 */

/* IRC64M - 64 MHz RC factory-trimmed
 * IRC32K - 32 KHz RC
 * HXTAL  - On-board crystal frequency is 25MHz
 * LXTAL  - 32.768 kHz
 */

#ifndef CONFIG_GD32H759I_BOARD_HXTAL_VALUE
#  define GD32_BOARD_HXTAL       25000000ul
#else
#  define GD32_BOARD_HXTAL       CONFIG_GD32H759I_BOARD_HXTAL_VALUE
#endif

#define GD32_IRC64M_VALUE      64000000ul
#define GD32_IRC32K_VALUE      32000u
#define GD32_HXTAL_VALUE       GD32_BOARD_HXTAL
#define GD32_LXTAL_VALUE       32768u

#if defined(CONFIG_GD32H759I_600MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL0_VCO = (GD32_HXTAL_VALUE / PLL0PSC) * (PLL0N+1)
 *          = (25,000,000 / 5) * 120
 *          = 600,000,000
 * SYSCLK  = PLL0_VCO / PLL0P
 *         = 600,000,000 / 1 = 600,000,000
 */

#define GD32_PLL_PLL0PSC            RCU_PLL0_PLL0PSC(5)
#define GD32_PLL_PLL0N              RCU_PLL0_PLL0N(120)
#define GD32_PLL_PLL0P              RCU_PLL0_PLL0P(0)
#define GD32_PLL_PLL0Q              RCU_PLLADDCTL_PLL0Q(1)
#define GD32_PLL_PLL0R              RCU_PLL0_PLL0R(1)

#define GD32_SYSCLK_FREQUENCY       600000000ul

#elif defined(CONFIG_GD32H759I_480MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL0_VCO = (GD32_HXTAL_VALUE / PLL0PSC) * (PLL0N+1)
 *          = (25,000,000 / 5) * 96
 *          = 480,000,000
 * SYSCLK  = PLL0_VCO / PLL0P
 *         = 480,000,000 / 1 = 480,000,000
 */

#define GD32_PLL_PLL0PSC            RCU_PLL0_PLL0PSC(5)
#define GD32_PLL_PLL0N              RCU_PLL0_PLL0N(96)
#define GD32_PLL_PLL0P              RCU_PLL0_PLL0P(0)
#define GD32_PLL_PLL0Q              RCU_PLLADDCTL_PLL0Q(1)
#define GD32_PLL_PLL0R              RCU_PLL0_PLL0R(1)

#define GD32_SYSCLK_FREQUENCY       480000000ul

#elif defined(CONFIG_GD32H759I_400MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL0_VCO = (GD32_HXTAL_VALUE / PLL0PSC) * (PLL0N+1)
 *          = (25,000,000 / 5) * 80
 *          = 400,000,000
 * SYSCLK  = PLL0_VCO / PLL0P
 *         = 400,000,000 / 1 = 400,000,000
 */

#define GD32_PLL_PLL0PSC            RCU_PLL0_PLL0PSC(5)
#define GD32_PLL_PLL0N              RCU_PLL0_PLL0N(80)
#define GD32_PLL_PLL0P              RCU_PLL0_PLL0P(0)
#define GD32_PLL_PLL0Q              RCU_PLLADDCTL_PLL0Q(1)
#define GD32_PLL_PLL0R              RCU_PLL0_PLL0R(1)

#define GD32_SYSCLK_FREQUENCY       400000000ul

#endif

/* AHB clock (HCLK) is SYSCLK */

#define GD32_RCU_CFG0_AHB_PSC      RCU_CFG0_AHBPSC_CKSYS_DIV2  /* HCLK  = SYSCLK / 2 */
#define GD32_HCLK_FREQUENCY        (GD32_SYSCLK_FREQUENCY/2)

/* APB4 clock (PCLK4) is HCLK/2 */

#define GD32_RCU_CFG0_APB4_PSC     RCU_CFG0_APB4PSC_CKAHB_DIV2 /* PCLK4 = HCLK / 2 */
#define GD32_PCLK4_FREQUENCY       (GD32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/2 */

#define GD32_RCU_CFG0_APB3_PSC     RCU_CFG0_APB3PSC_CKAHB_DIV2 /* PCLK3 = HCLK / 2 */
#define GD32_PCLK3_FREQUENCY       (GD32_HCLK_FREQUENCY / 2)
/* APB2 clock (PCLK2) is HCLK/2 */

#define GD32_RCU_CFG0_APB2_PSC     RCU_CFG0_APB2PSC_CKAHB_DIV1 /* PCLK2 = HCLK / 1 */
#define GD32_PCLK2_FREQUENCY       (GD32_HCLK_FREQUENCY/1)

/* APB1 clock (PCLK1) is HCLK/2 */

#define GD32_RCU_CFG0_APB1_PSC     RCU_CFG0_APB1PSC_CKAHB_DIV2 /* PCLK1 = HCLK / 2 */
#define GD32_PCLK1_FREQUENCY       (GD32_HCLK_FREQUENCY / 2)

/* LED definitions **********************************************************/

/* The GD32H759IM_EVAL board has board has two LEDs. The LED1, and
 * LED2 are controlled by GPIO. LED1 is connected to PF10, LED2 is connected
 * to PA6
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values */

typedef enum
{
    BOARD_LED1 = 0,
    BOARD_LED2 = 1,
    BOARD_LEDS
} led_typedef_enum;

/* LED bits */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/gd32f4xx_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        LED1  LED2
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF    */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  ON     OFF    */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON     */
#define LED_STACKCREATED   3 /* Idle stack created       ON     ON     */
#define LED_INIRQ          4 /* In an interrupt          OFF    FLASH  */
#define LED_SIGNAL         5 /* In a signal handler      FLASH  OFF    */
#define LED_ASSERTION      6 /* An assertion failed      ON     FLASH  */
#define LED_PANIC          7 /* The system has crashed   FLASH  ON     */
#define LED_IDLE           8 /* MCU is is sleep mode     FLASH  FLASH  */

/* Button definitions *******************************************************/

/* The GD32H759I Eval supports three user buttons:  Wakeup, Tamper and
 * User key, they are connected to GPIO PA0, PC13, PF8.
 * A low value will be sensed when the button is depressed.
 */

typedef enum
{
    BUTTON_WAKEUP = 0,
    BUTTON_TAMPER = 1,
    BUTTON_USER = 2,
    NUM_BUTTONS
} key_typedef_enum;

#define BUTTON_WAKEUP_BIT    (1 << BUTTON_WAKEUP)
#define BUTTON_TAMPER_BIT    (1 << BUTTON_TAMPER)
#define BUTTON_USER_BIT      (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART0 (Console) */

#define GPIO_USART0_RX (GPIO_USART0_RX_1 | GPIO_CFG_SPEED_85MHZ)
#define GPIO_USART0_TX (GPIO_USART0_TX_1 | GPIO_CFG_SPEED_85MHZ)

/* USART2 (DMA bring-up port)
 * TX: PB10
 * RX: PB11
 */

#define GPIO_USART2_TX (GPIO_USART2_TX_1 | GPIO_CFG_SPEED_85MHZ)
#define GPIO_USART2_RX (GPIO_USART2_RX_1 | GPIO_CFG_SPEED_85MHZ)

/* I2C0: PB6/SCL, PB7/SDA */

#define GPIO_I2C0_SCL  (GPIO_I2C0_SCL_1 | GPIO_CFG_SPEED_85MHZ)
#define GPIO_I2C0_SDA  (GPIO_I2C0_SDA_1 | GPIO_CFG_SPEED_85MHZ)

/* I2C1: PH4/SCL, PB11/SDA */

#define GPIO_I2C1_SCL  (GPIO_I2C1_SCL_3 | GPIO_CFG_SPEED_85MHZ)
#define GPIO_I2C1_SDA  (GPIO_I2C1_SDA_1 | GPIO_CFG_SPEED_85MHZ)

/* I2C2: PA8/SCL, PC9/SDA */

#define GPIO_I2C2_SCL  (GPIO_I2C2_SCL_1 | GPIO_CFG_SPEED_85MHZ)
#define GPIO_I2C2_SDA  (GPIO_I2C2_SDA_1 | GPIO_CFG_SPEED_85MHZ)

/* I2C3: PD12/SCL, PD13/SDA */

#define GPIO_I2C3_SCL  (GPIO_I2C3_SCL_3 | GPIO_CFG_SPEED_85MHZ)
#define GPIO_I2C3_SDA  (GPIO_I2C3_SDA_3 | GPIO_CFG_SPEED_85MHZ)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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

void gd32_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_GD32H7_GD32H759IM_EVAL_INCLUDE_BOARD_H */