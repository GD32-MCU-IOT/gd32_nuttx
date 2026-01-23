/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/include/board.h
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

#ifndef __BOARDS_ARM_GD32E113VB_EVAL_INCLUDE_BOARD_H
#define __BOARDS_ARM_GD32E113VB_EVAL_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#if (CONFIG_GD32E113_BOARD_USE_HXTAL)
#define GD32_BOARD_SYSCLK_PLL_HXTAL
#elif (CONFIG_GD32E113_BOARD_USE_IRC8M)
#define GD32_BOARD_SYSCLK_PLL_IRC8M
#endif

/* Do not include GD32E11X header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The GD32E113VB-EVAL board features a single 8MHz crystal.
 *
 * This is the default configuration:
 *   System clock source           : PLL (HXTAL)
 *   SYSCLK(Hz)                    : 120000000    Determined by PLL config
 *   HCLK(Hz)                      : 120000000    (GD32_SYSCLK_FREQUENCY)
 *   AHB Prescaler                 : 1            (GD32_RCU_CFG0_AHB_PSC)
 *   APB2 Prescaler                : 2            (GD32_RCU_CFG0_APB2_PSC)
 *   APB1 Prescaler                : 4            (GD32_RCU_CFG0_APB1_PSC)
 *   HXTAL value(Hz)               : 8000000      (GD32_BOARD_XTAL)
 *   PLLM                          : 2            (GD32_PLL_PLLM)
 *   PLLN                          : 30           (RCU_PLL_PLLN)
 */

/* IRC8M - 8 MHz RC factory-trimmed
 * IRC40K - 40 KHz RC
 * HXTAL  - On-board crystal frequency is 8MHz
 * LXTAL  - 32.768 kHz
 */

#ifndef CONFIG_GD32E113_BOARD_HXTAL_VALUE
#  define GD32_BOARD_HXTAL       8000000ul
#else
#  define GD32_BOARD_HXTAL       CONFIG_GD32E113_BOARD_HXTAL_VALUE
#endif

#define GD32_IRC8M_VALUE       8000000ul
#define GD32_IRC40K_VALUE      40000u
#define GD32_HXTAL_VALUE       GD32_BOARD_HXTAL
#define GD32_LXTAL_VALUE       32768u

#if defined(CONFIG_GD32E113_120MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL_VCO = (GD32_HXTAL_VALUE / PREDV0) * PLLMF
 *         = 4 MHz * 30
 *         = 120 MHz
 * PREDV0 = (GD32_HXTAL_VALUE / 2) * GD32_RCU_PLL1_MULF / GD32_RCU_PREDV0_DIV
 *        = (8 Hz / 2) * 10 / 10
 *        = 4 MHz
 * SYSCLK  = PLL_VCO
 *         = 120 MHz
 */

#define GD32_RCU_CFG0_PLLMF    RCU_CFG0_PLLMF_MUL30
#if defined(CONFIG_GD32E113_BOARD_PREDIV0_SELECT_PLL1)
#define GD32_RCU_PLL1_MULF     RCU_CFG1_PLL1MF_MUL10
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV10
#define GD32_RCU_PREDV1_DIV    RCU_CFG1_PREDV1_DIV2
#else
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV2
#endif

#define GD32_FMC_WAIT_STATE   FMC_WS_WSCNT_7

#define GD32_SYSCLK_FREQUENCY  120000000ul

#elif defined(CONFIG_GD32E113_108MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL_VCO = (GD32_HXTAL_VALUE / PREDV0) * PLLMF
 *         = 4 MHz * 27
 *         = 108 MHz
 * PREDV0 = (GD32_HXTAL_VALUE / 2) * GD32_RCU_PLL1_MULF / GD32_RCU_PREDV0_DIV
 *        = (8 Hz / 2) * 10 / 10
 *        = 4 MHz
 * SYSCLK  = PLL_VCO
 *         = 108 MHz
 */

 #if defined(CONFIG_GD32E113_BOARD_PREDIV0_SELECT_PLL1)
#define GD32_RCU_CFG0_PLLMF    RCU_CFG0_PLLMF_MUL27
#define GD32_RCU_PLL1_MULF     RCU_CFG1_PLL1MF_MUL10
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV10
#define GD32_RCU_PREDV1_DIV    RCU_CFG1_PREDV1_DIV2
#else
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV2
#endif

#define GD32_FMC_WAIT_STATE   FMC_WS_WSCNT_7

#define GD32_SYSCLK_FREQUENCY  108000000ul

#elif defined(CONFIG_GD32E113_96MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL_VCO = (GD32_HXTAL_VALUE / PREDV0) * PLLMF
 *         = 4 MHz * 24
 *         = 96 MHz
 * PREDV0 = (GD32_HXTAL_VALUE / 2) * GD32_RCU_PLL1_MULF / GD32_RCU_PREDV0_DIV
 *        = (8 Hz / 2) * 10 / 10
 *        = 4 MHz
 * SYSCLK  = PLL_VCO
 *         = 96 MHz
 */

 #if defined(CONFIG_GD32E113_BOARD_PREDIV0_SELECT_PLL1)
#define GD32_RCU_CFG0_PLLMF    RCU_CFG0_PLLMF_MUL24
#define GD32_RCU_PLL1_MULF     RCU_CFG1_PLL1MF_MUL10
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV10
#define GD32_RCU_PREDV1_DIV    RCU_CFG1_PREDV1_DIV2
#else
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV2
#endif

#define GD32_FMC_WAIT_STATE   FMC_WS_WSCNT_7

#define GD32_SYSCLK_FREQUENCY  96000000ul

#elif defined(CONFIG_GD32E113_72MHZ)

/* Main PLL Configuration.
 *
 * PLL source is HXTAL
 * PLL_VCO = (GD32_HXTAL_VALUE / PREDV0) * PLLMF
 *         = 4 MHz * 18
 *         = 72 MHz
 * PREDV0 = (GD32_HXTAL_VALUE / 2) * GD32_RCU_PLL1_MULF / GD32_RCU_PREDV0_DIV
 *        = (8 Hz / 2) * 10 / 10
 *        = 4 MHz
 * SYSCLK  = PLL_VCO
 *         = 72 MHz
 */

#if defined(CONFIG_GD32E113_BOARD_PREDIV0_SELECT_PLL1)
#define GD32_RCU_CFG0_PLLMF    RCU_CFG0_PLLMF_MUL18
#define GD32_RCU_PLL1_MULF     RCU_CFG1_PLL1MF_MUL10
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV10
#define GD32_RCU_PREDV1_DIV    RCU_CFG1_PREDV1_DIV2
#else
#define GD32_RCU_PREDV0_DIV    RCU_CFG1_PREDV0_DIV2
#endif

#define GD32_FMC_WAIT_STATE   FMC_WS_WSCNT_5

#define GD32_SYSCLK_FREQUENCY  72000000ul

#endif

/* AHB clock (HCLK) is SYSCLK */

#define GD32_RCU_CFG0_AHB_PSC      RCU_CFG0_AHBPSC_DIV1  /* HCLK  = SYSCLK / 1 */
#define GD32_HCLK_FREQUENCY        GD32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK/1 */

#define GD32_RCU_CFG0_APB2_PSC     RCU_CFG0_APB2PSC_DIV1 /* PCLK2 = HCLK / 1 */
#define GD32_PCLK2_FREQUENCY       (GD32_HCLK_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 */

#define GD32_RCU_CFG0_APB1_PSC     RCU_CFG0_APB1PSC_DIV2 /* PCLK1 = HCLK / 2 */
#define GD32_PCLK1_FREQUENCY       (GD32_HCLK_FREQUENCY / 2)

/* Timers driven from APB1 will be twice PCLK1 */

#define GD32_APB1_TIMER1_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER2_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER3_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER4_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER5_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER6_CLKIN   (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER11_CLKIN  (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER12_CLKIN  (2*GD32_PCLK1_FREQUENCY)
#define GD32_APB1_TIMER13_CLKIN  (2*GD32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define GD32_APB2_TIMER0_CLKIN   (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER7_CLKIN   (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER8_CLKIN   (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER9_CLKIN   (2*GD32_PCLK2_FREQUENCY)
#define GD32_APB2_TIMER10_CLKIN  (2*GD32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIMER0,7-10 are on APB2, others on APB1
 */

#define BOARD_TIMER0_FREQUENCY    GD32_HCLK_FREQUENCY
#define BOARD_TIMER1_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER2_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER3_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER4_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER5_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER6_FREQUENCY    (GD32_HCLK_FREQUENCY/2)
#define BOARD_TIMER7_FREQUENCY    GD32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The GD32E113VB_EVAL board has four LEDs. The LED1, LED2, LED3 and LED4
 * are controlled by GPIO.
 * LED1 is connected to PC0, LED2 is connected to PC2,
 * LED3 is connected to PE0, LED4 is connected to PE1
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
    BOARD_LED3 = 2,
    BOARD_LED4 = 3,
    BOARD_LEDS
} led_typedef_enum;

/* LED bits */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/gd32e11x_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                     LED1  LED2  LED3  LED4
 */

#define LED_STARTED        0 /* NuttX has been started   OFF   OFF   OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  ON    OFF   OFF   OFF  */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF   ON    OFF   OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF   OFF   ON    OFF  */
#define LED_INIRQ          4 /* In an interrupt          ON    ON    OFF   OFF  */
#define LED_SIGNAL         5 /* In a signal handler      ON    OFF   ON    OFF  */
#define LED_ASSERTION      6 /* An assertion failed      OFF   ON    ON    OFF  */
#define LED_PANIC          7 /* The system has crashed   FLASH ON    ON    ON   */
#define LED_IDLE           8 /* MCU is is sleep mode     OFF   FLASH OFF   OFF  */

/* Button definitions *******************************************************/

/* The GD32E113V Eval supports three user buttons: Wakeup, Tamper and
 * User key, they are connected to GPIO PA0, PC13, PB14.
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

#if defined(CONFIG_GD32E113VB_EVAL_CONSOLE_BOARD)

/* USART0:
 *
 * These configurations assume that you are using a standard RS-232
 * shield with the serial interface with RX on PA10 and TX on PA9:
 *
 *   -------- ---------------
 *           GD32E113VB-EVAL
 *   -- ----- --------- -----
 *   RX    USART0_RX PA10
 *   TX    USART0_TX PA9
 *   -- ----- --------- -----
 */

#  define GPIO_USART0_RX (GPIO_USART0_RX_1 | GPIO_CFG_SPEED_50MHZ)
#  define GPIO_USART0_TX (GPIO_USART0_TX_1 | GPIO_CFG_SPEED_50MHZ)

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART0_IFLOWCONTROL)
#    define GPIO_USART0_RTS (GPIO_USART0_RTS | GPIO_CFG_SPEED_50MHZ)
#  endif
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART0_OFLOWCONTROL)
#    define GPIO_USART0_CTS (GPIO_USART0_CTS | GPIO_CFG_SPEED_50MHZ)
#  endif
#endif

#if CONFIG_GD32E11X_USART0_TXDMA
#  define DMA_CHANNEL_USART0_TX    DMA_REQ_USART0_TX_1
#endif
#if CONFIG_GD32E11X_USART0_RXDMA
#  define DMA_CHANNEL_USART0_RX    DMA_REQ_USART0_RX_1
#endif

#if defined(CONFIG_GD32E11X_USART_RXDMA) || defined(CONFIG_GD32E11X_USART_TXDMA)
#  define USART_DMA_INTEN          (DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE | DMA_CHXCTL_FTFIE)
#endif

/* USART1:
 * Use USART1 as alternative console
 */

#if defined(CONFIG_GD32E113VB_EVAL_CONSOLE_VIRTUAL)
#  define GPIO_USART1_RX (GPIO_USART1_RX_1 | GPIO_CFG_SPEED_50MHZ)
#  define GPIO_USART1_TX (GPIO_USART1_TX_1 | GPIO_CFG_SPEED_50MHZ)
#endif

/* I2C pin definitions
 * I2C0: Using remapped pins PB6(SCL)/PB7(SDA) to avoid conflict with USART0 on PA9/PA10
 * Note: GPIO_I2C0_SCL_1/GPIO_I2C0_SDA_1 are defined in hardware/gd32e113v_pinmap.h
 * Note: I2C1 pins (PB10/PB11) are also pre-defined in hardware/gd32e113v_pinmap.h
 */

#define GPIO_I2C0_SCL    GPIO_I2C0_SCL_1
#define GPIO_I2C0_SDA    GPIO_I2C0_SDA_1

/* SPI flash
 *
 *  PB13  SPI1_SCK
 *  PB14  SPI1_MISO
 *  PB15  SPI1_MOSI
 *
 *  PD11  SPI1_CS
 *
 */

#define GPIO_SPI1_CSPIN (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_SPEED_50MHZ | \
                         GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_D | GPIO_CFG_PIN_11)

#define GPIO_SPI1_MISO_PIN  ((GPIO_SPI1_MISO_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_50MHZ)
#define GPIO_SPI1_MOSI_PIN  ((GPIO_SPI1_MOSI_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_50MHZ)
#define GPIO_SPI1_SCK_PIN   ((GPIO_SPI1_SCK_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_50MHZ)

#ifdef CONFIG_GD32E11X_SPI0
#  define GPIO_SPI0_CSPIN     (GPIO_CFG_OUTPUT | GPIO_CFG_CTL_OUTPP | GPIO_CFG_SPEED_50MHZ | \
                               GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_A | GPIO_CFG_PIN_4)
#  define GPIO_SPI0_MISO_PIN  ((GPIO_SPI0_MISO_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_50MHZ)
#  define GPIO_SPI0_MOSI_PIN  ((GPIO_SPI0_MOSI_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_50MHZ)
#  define GPIO_SPI0_SCK_PIN   ((GPIO_SPI0_SCK_1 & ~GPIO_CFG_SPEED_MASK) | GPIO_CFG_SPEED_50MHZ)
#endif

#ifdef CONFIG_GD32E11X_SPI0_DMA
#  define DMA_CHANNEL_SPI0_TX      DMA_REQ_SPI0_TX_1
#  define DMA_CHANNEL_SPI0_RX      DMA_REQ_SPI0_RX_1
#endif

#ifdef CONFIG_GD32E11X_SPI_DMA
#  define SPI_DMA_INTEN            (DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE | DMA_CHXCTL_FTFIE)
#endif

#endif /* __BOARDS_ARM_GD32E113VB_EVAL_INCLUDE_BOARD_H */

