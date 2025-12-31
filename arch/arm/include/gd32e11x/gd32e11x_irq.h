/****************************************************************************
 * arch/arm/include/gd32e11x/gd32e11x_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_GD32_GD32E11X_IRQ_H
#define __ARCH_ARM_INCLUDE_GD32_GD32E11X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in
 * the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15). These common definitions can
 * be found in nuttx/arch/arm/include/gd32/irq.h
 *
 * External interrupts (vectors >= 16)
 */

/* External interrupts (vectors >= 16)
 *
 * The following list follows the CMSIS IRQn order used by GD32E10x.
 * No per-device (sub-model) differentiation is applied here.
 */

#define GD32_IRQ_WWDGT                      (GD32_IRQ_EXINT+0)  /* 0:  Window watchdog timer interrupt */
#define GD32_IRQ_LVD                        (GD32_IRQ_EXINT+1)  /* 1:  LVD through EXTI line detect interrupt */
#define GD32_IRQ_TAMPER                     (GD32_IRQ_EXINT+2)  /* 2:  Tamper through EXTI line detect interrupt */
#define GD32_IRQ_RTC                        (GD32_IRQ_EXINT+3)  /* 3:  RTC through EXTI line interrupt */
#define GD32_IRQ_FMC                        (GD32_IRQ_EXINT+4)  /* 4:  FMC interrupt */
#define GD32_IRQ_RCU_CTC                    (GD32_IRQ_EXINT+5)  /* 5:  RCU and CTC interrupt */
#define GD32_IRQ_EXTI0                      (GD32_IRQ_EXINT+6)  /* 6:  EXTI line 0 interrupt */
#define GD32_IRQ_EXTI1                      (GD32_IRQ_EXINT+7)  /* 7:  EXTI line 1 interrupt */
#define GD32_IRQ_EXTI2                      (GD32_IRQ_EXINT+8)  /* 8:  EXTI line 2 interrupt */
#define GD32_IRQ_EXTI3                      (GD32_IRQ_EXINT+9)  /* 9:  EXTI line 3 interrupt */
#define GD32_IRQ_EXTI4                      (GD32_IRQ_EXINT+10) /* 10: EXTI line 4 interrupt */
#define GD32_IRQ_DMA0_CHANNEL0              (GD32_IRQ_EXINT+11) /* 11: DMA0 channel0 interrupt */
#define GD32_IRQ_DMA0_CHANNEL1              (GD32_IRQ_EXINT+12) /* 12: DMA0 channel1 interrupt */
#define GD32_IRQ_DMA0_CHANNEL2              (GD32_IRQ_EXINT+13) /* 13: DMA0 channel2 interrupt */
#define GD32_IRQ_DMA0_CHANNEL3              (GD32_IRQ_EXINT+14) /* 14: DMA0 channel3 interrupt */
#define GD32_IRQ_DMA0_CHANNEL4              (GD32_IRQ_EXINT+15) /* 15: DMA0 channel4 interrupt */
#define GD32_IRQ_DMA0_CHANNEL5              (GD32_IRQ_EXINT+16) /* 16: DMA0 channel5 interrupt */
#define GD32_IRQ_DMA0_CHANNEL6              (GD32_IRQ_EXINT+17) /* 17: DMA0 channel6 interrupt */
#define GD32_IRQ_ADC0_1                     (GD32_IRQ_EXINT+18) /* 18: ADC0 and ADC1 interrupt */
#define GD32_IRQ_ADC                        GD32_IRQ_ADC0_1
#define GD32_IRQ_CAN0_TX                    (GD32_IRQ_EXINT+19) /* 19: CAN0 TX interrupt */
#define GD32_IRQ_CAN0_RX0                   (GD32_IRQ_EXINT+20) /* 20: CAN0 RX0 interrupt */
#define GD32_IRQ_CAN0_RX1                   (GD32_IRQ_EXINT+21) /* 21: CAN0 RX1 interrupt */
#define GD32_IRQ_CAN0_EWMC                  (GD32_IRQ_EXINT+22) /* 22: CAN0 EWMC interrupt */
#define GD32_IRQ_EXTI5_9                    (GD32_IRQ_EXINT+23) /* 23: EXTI[9:5] interrupts */
#define GD32_IRQ_TIMER0_BRK_TIMER8          (GD32_IRQ_EXINT+24) /* 24: TIMER0 break and TIMER8 interrupts */
#define GD32_IRQ_TIMER0_UP_TIMER9           (GD32_IRQ_EXINT+25) /* 25: TIMER0 update and TIMER9 interrupts */
#define GD32_IRQ_TIMER0_TRG_CMT_TIMER10     (GD32_IRQ_EXINT+26) /* 26: TIMER0 trigger/commutation and TIMER10 interrupts */
#define GD32_IRQ_TIMER0_CHANNEL             (GD32_IRQ_EXINT+27) /* 27: TIMER0 channel capture compare interrupt */
#define GD32_IRQ_TIMER1                     (GD32_IRQ_EXINT+28) /* 28: TIMER1 interrupt */
#define GD32_IRQ_TIMER2                     (GD32_IRQ_EXINT+29) /* 29: TIMER2 interrupt */
#define GD32_IRQ_TIMER3                     (GD32_IRQ_EXINT+30) /* 30: TIMER3 interrupt */
#define GD32_IRQ_I2C0_EV                    (GD32_IRQ_EXINT+31) /* 31: I2C0 event interrupt */
#define GD32_IRQ_I2C0_ER                    (GD32_IRQ_EXINT+32) /* 32: I2C0 error interrupt */
#define GD32_IRQ_I2C1_EV                    (GD32_IRQ_EXINT+33) /* 33: I2C1 event interrupt */
#define GD32_IRQ_I2C1_ER                    (GD32_IRQ_EXINT+34) /* 34: I2C1 error interrupt */
#define GD32_IRQ_SPI0                       (GD32_IRQ_EXINT+35) /* 35: SPI0 interrupt */
#define GD32_IRQ_SPI1                       (GD32_IRQ_EXINT+36) /* 36: SPI1 interrupt */
#define GD32_IRQ_USART0                     (GD32_IRQ_EXINT+37) /* 37: USART0 interrupt */
#define GD32_IRQ_USART1                     (GD32_IRQ_EXINT+38) /* 38: USART1 interrupt */
#define GD32_IRQ_USART2                     (GD32_IRQ_EXINT+39) /* 39: USART2 interrupt */
#define GD32_IRQ_EXTI10_15                  (GD32_IRQ_EXINT+40) /* 40: EXTI[15:10] interrupts */
#define GD32_IRQ_RTC_ALARM                  (GD32_IRQ_EXINT+41) /* 41: RTC alarm interrupt */
#define GD32_IRQ_USBFS_WKUP                 (GD32_IRQ_EXINT+42) /* 42: USBFS wakeup interrupt */
#define GD32_IRQ_TIMER7_BRK_TIMER11         (GD32_IRQ_EXINT+43) /* 43: TIMER7 break and TIMER11 interrupts */
#define GD32_IRQ_TIMER7_UP_TIMER12          (GD32_IRQ_EXINT+44) /* 44: TIMER7 update and TIMER12 interrupts */
#define GD32_IRQ_TIMER7_TRG_CMT_TIMER13     (GD32_IRQ_EXINT+45) /* 45: TIMER7 trigger/commutation and TIMER13 interrupts */
#define GD32_IRQ_TIMER7_CHANNEL             (GD32_IRQ_EXINT+46) /* 46: TIMER7 channel capture compare interrupt */

#define GD32_IRQ_EXMC                       (GD32_IRQ_EXINT+48) /* 48: EXMC global interrupt */
#define GD32_IRQ_TIMER4                     (GD32_IRQ_EXINT+50) /* 50: TIMER4 global interrupt */
#define GD32_IRQ_SPI2                       (GD32_IRQ_EXINT+51) /* 51: SPI2 global interrupt */
#define GD32_IRQ_UART3                      (GD32_IRQ_EXINT+52) /* 52: UART3 global interrupt */
#define GD32_IRQ_UART4                      (GD32_IRQ_EXINT+53) /* 53: UART4 global interrupt */
#define GD32_IRQ_TIMER5                     (GD32_IRQ_EXINT+54) /* 54: TIMER5 global interrupt */
#define GD32_IRQ_TIMER6                     (GD32_IRQ_EXINT+55) /* 55: TIMER6 global interrupt */
#define GD32_IRQ_DMA1_CHANNEL0              (GD32_IRQ_EXINT+56) /* 56: DMA1 channel0 global interrupt */
#define GD32_IRQ_DMA1_CHANNEL1              (GD32_IRQ_EXINT+57) /* 57: DMA1 channel1 global interrupt */
#define GD32_IRQ_DMA1_CHANNEL2              (GD32_IRQ_EXINT+58) /* 58: DMA1 channel2 global interrupt */
#define GD32_IRQ_DMA1_CHANNEL3              (GD32_IRQ_EXINT+59) /* 59: DMA1 channel3 global interrupt */
#define GD32_IRQ_DMA1_CHANNEL4              (GD32_IRQ_EXINT+60) /* 60: DMA1 channel4 global interrupt */

#define GD32_IRQ_CAN1_TX                    (GD32_IRQ_EXINT+63) /* 63: CAN1 TX interrupt */
#define GD32_IRQ_CAN1_RX0                   (GD32_IRQ_EXINT+64) /* 64: CAN1 RX0 interrupt */
#define GD32_IRQ_CAN1_RX1                   (GD32_IRQ_EXINT+65) /* 65: CAN1 RX1 interrupt */
#define GD32_IRQ_CAN1_EWMC                  (GD32_IRQ_EXINT+66) /* 66: CAN1 EWMC interrupt */
#define GD32_IRQ_USBFS                      (GD32_IRQ_EXINT+67) /* 67: USBFS global interrupt */

#define GD32_IRQ_NEXTINT                    (68)
#define NR_IRQS                             (GD32_IRQ_EXINT + GD32_IRQ_NEXTINT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_GD32_GD32E11X_IRQ_H */
