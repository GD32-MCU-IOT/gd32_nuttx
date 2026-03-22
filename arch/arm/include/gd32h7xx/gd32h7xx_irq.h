/****************************************************************************
 * arch/arm/include/gd32h7xx/gd32h7xx_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_GD32H7_GD32H7XX_IRQ_H
#define __ARCH_ARM_INCLUDE_GD32H7_GD32H7XX_IRQ_H

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.
 * Processor Exceptions (vectors 0-15). These common definitions can
 * be found in nuttx/arch/arm/include/gd32/irq.h
 * External interrupts (vectors >= 16).  These definitions are
 * chip-specific
 */

#define GD32_IRQ_WWDGT                 (GD32_IRQ_EXINT + 0)   /* Vector 16: Window watchdog timer interrupt */
#define GD32_IRQ_VAVD_LVD_VOVD         (GD32_IRQ_EXINT + 1)   /* Vector 17: AVD/LVD/OVD through EXTI line detect interrupt */
#define GD32_IRQ_TAMPER_STAMP_LXTAL    (GD32_IRQ_EXINT + 2)   /* Vector 18: RTC tamper and timestamp/LXTAL clock stuck interrupt */
#define GD32_IRQ_RTC_WKUP              (GD32_IRQ_EXINT + 3)   /* Vector 19: RTC Wakeup interrupt */
#define GD32_IRQ_FMC                   (GD32_IRQ_EXINT + 4)   /* Vector 20: FMC interrupt */
#define GD32_IRQ_RCU                   (GD32_IRQ_EXINT + 5)   /* Vector 21: RCU interrupt */
#define GD32_IRQ_EXTI0                 (GD32_IRQ_EXINT + 6)   /* Vector 22: EXTI line 0 */
#define GD32_IRQ_EXTI1                 (GD32_IRQ_EXINT + 7)   /* Vector 23: EXTI line 1 */
#define GD32_IRQ_EXTI2                 (GD32_IRQ_EXINT + 8)   /* Vector 24: EXTI line 2 */
#define GD32_IRQ_EXTI3                 (GD32_IRQ_EXINT + 9)   /* Vector 25: EXTI line 3 */
#define GD32_IRQ_EXTI4                 (GD32_IRQ_EXINT + 10)  /* Vector 26: EXTI line 4 */
#define GD32_IRQ_DMA0_CH0              (GD32_IRQ_EXINT + 11)  /* Vector 27: DMA0 channel 0 interrupt */
#define GD32_IRQ_DMA0_CH1              (GD32_IRQ_EXINT + 12)  /* Vector 28: DMA0 channel 1 interrupt */
#define GD32_IRQ_DMA0_CH2              (GD32_IRQ_EXINT + 13)  /* Vector 29: DMA0 channel 2 interrupt */
#define GD32_IRQ_DMA0_CH3              (GD32_IRQ_EXINT + 14)  /* Vector 30: DMA0 channel 3 interrupt */
#define GD32_IRQ_DMA0_CH4              (GD32_IRQ_EXINT + 15)  /* Vector 31: DMA0 channel 4 interrupt */
#define GD32_IRQ_DMA0_CH5              (GD32_IRQ_EXINT + 16)  /* Vector 32: DMA0 channel 5 interrupt */
#define GD32_IRQ_DMA0_CH6              (GD32_IRQ_EXINT + 17)  /* Vector 33: DMA0 channel 6 interrupt */
#define GD32_IRQ_ADC0_1                (GD32_IRQ_EXINT + 18)  /* Vector 34: ADC0 and ADC1 */
#define GD32_IRQ_EXTI5_9               (GD32_IRQ_EXINT + 23)  /* Vector 39: EXTI line 5 to 9 */
#define GD32_IRQ_TIMER0_BRK            (GD32_IRQ_EXINT + 24)  /* Vector 40: TIMER0 break interrupt */
#define GD32_IRQ_TIMER0_UP             (GD32_IRQ_EXINT + 25)  /* Vector 41: TIMER0 update interrupt */
#define GD32_IRQ_TIMER0_TRG_CMT        (GD32_IRQ_EXINT + 26)  /* Vector 42: TIMER0 trigger/commutation interrupt */
#define GD32_IRQ_TIMER0_CHANNEL        (GD32_IRQ_EXINT + 27)  /* Vector 43: TIMER0 capture/compare interrupt */
#define GD32_IRQ_TIMER1                (GD32_IRQ_EXINT + 28)  /* Vector 44: TIMER1 interrupt */
#define GD32_IRQ_TIMER2                (GD32_IRQ_EXINT + 29)  /* Vector 45: TIMER2 interrupt */
#define GD32_IRQ_TIMER3                (GD32_IRQ_EXINT + 30)  /* Vector 46: TIMER3 interrupt */
#define GD32_IRQ_I2C0_EV               (GD32_IRQ_EXINT + 31)  /* Vector 47: I2C0 event interrupt */
#define GD32_IRQ_I2C0_ER               (GD32_IRQ_EXINT + 32)  /* Vector 48: I2C0 error interrupt */
#define GD32_IRQ_I2C1_EV               (GD32_IRQ_EXINT + 33)  /* Vector 49: I2C1 event interrupt */
#define GD32_IRQ_I2C1_ER               (GD32_IRQ_EXINT + 34)  /* Vector 50: I2C1 error interrupt */
#define GD32_IRQ_SPI0                  (GD32_IRQ_EXINT + 35)  /* Vector 51: SPI0 interrupt */
#define GD32_IRQ_SPI1                  (GD32_IRQ_EXINT + 36)  /* Vector 52: SPI1 interrupt */
#define GD32_IRQ_USART0                (GD32_IRQ_EXINT + 37)  /* Vector 53: USART0 global and wakeup interrupt */
#define GD32_IRQ_USART1                (GD32_IRQ_EXINT + 38)  /* Vector 54: USART1 global and wakeup interrupt */
#define GD32_IRQ_USART2                (GD32_IRQ_EXINT + 39)  /* Vector 55: USART2 global and wakeup interrupt */
#define GD32_IRQ_EXTI10_15             (GD32_IRQ_EXINT + 40)  /* Vector 56: EXTI line 10 to 15 */
#define GD32_IRQ_RTC_ALARM             (GD32_IRQ_EXINT + 41)  /* Vector 57: RTC Alarm interrupt */
#define GD32_IRQ_TIMER7_BRK            (GD32_IRQ_EXINT + 43)  /* Vector 59: TIMER7 Break global interrupt */
#define GD32_IRQ_TIMER7_UP             (GD32_IRQ_EXINT + 44)  /* Vector 60: TIMER7 Update global interrupt */
#define GD32_IRQ_TIMER7_TRG_CMT        (GD32_IRQ_EXINT + 45)  /* Vector 61: TIMER7 Trigger and Commutation global interrupt */
#define GD32_IRQ_TIMER7_CHANNEL        (GD32_IRQ_EXINT + 46)  /* Vector 62: TIMER7 Capture Compare interrupt */
#define GD32_IRQ_DMA0_CH7              (GD32_IRQ_EXINT + 47)  /* Vector 63: DMA0 channel 7 interrupt */
#define GD32_IRQ_EXMC                  (GD32_IRQ_EXINT + 48)  /* Vector 64: EXMC interrupt */
#define GD32_IRQ_SDIO0                 (GD32_IRQ_EXINT + 49)  /* Vector 65: SDMMC0 interrupt */
#define GD32_IRQ_TIMER4                (GD32_IRQ_EXINT + 50)  /* Vector 66: TIMER4 interrupt */
#define GD32_IRQ_SPI2                  (GD32_IRQ_EXINT + 51)  /* Vector 67: SPI2 interrupt */
#define GD32_IRQ_UART3                 (GD32_IRQ_EXINT + 52)  /* Vector 68: UART3 interrupt */
#define GD32_IRQ_UART4                 (GD32_IRQ_EXINT + 53)  /* Vector 69: UART4 interrupt */
#define GD32_IRQ_TIMER5_DAC_UDR        (GD32_IRQ_EXINT + 54)  /* Vector 70: TIMER5 global interrupt and DAC1/DAC0 underrun */
#define GD32_IRQ_TIMER6                (GD32_IRQ_EXINT + 55)  /* Vector 71: TIMER6 interrupt */
#define GD32_IRQ_DMA1_CH0              (GD32_IRQ_EXINT + 56)  /* Vector 72: DMA1 channel0 interrupt */
#define GD32_IRQ_DMA1_CH1              (GD32_IRQ_EXINT + 57)  /* Vector 73: DMA1 channel1 interrupt */
#define GD32_IRQ_DMA1_CH2              (GD32_IRQ_EXINT + 58)  /* Vector 74: DMA1 channel2 interrupt */
#define GD32_IRQ_DMA1_CH3              (GD32_IRQ_EXINT + 59)  /* Vector 75: DMA1 channel3 interrupt */
#define GD32_IRQ_DMA1_CH4              (GD32_IRQ_EXINT + 60)  /* Vector 76: DMA1 channel4 interrupt */
#define GD32_IRQ_ENET0                 (GD32_IRQ_EXINT + 61)  /* Vector 77: ENET 0 interrupt */
#define GD32_IRQ_ENET0_WKUP            (GD32_IRQ_EXINT + 62)  /* Vector 78: ENET0 wakeup through EXTI line interrupt */
#define GD32_IRQ_CAN0_TX               (GD32_IRQ_EXINT + 63)  /* Vector 79: CAN0 TX interrupt */
#define GD32_IRQ_CAN0_RX0              (GD32_IRQ_EXINT + 64)  /* Vector 80: CAN0 RX0 interrupt */
#define GD32_IRQ_CAN0_RX1              (GD32_IRQ_EXINT + 65)  /* Vector 81: CAN0 RX1 interrupt */
#define GD32_IRQ_CAN0_EWMC             (GD32_IRQ_EXINT + 66)  /* Vector 82: CAN0 EWMC interrupt */
#define GD32_IRQ_USBHS_OTG             (GD32_IRQ_EXINT + 67)  /* Vector 83: USBHS OTG interrupt */
#define GD32_IRQ_DMA1_CH5              (GD32_IRQ_EXINT + 68)  /* Vector 84: DMA1 channel5 interrupt */
#define GD32_IRQ_DMA1_CH6              (GD32_IRQ_EXINT + 69)  /* Vector 85: DMA1 channel6 interrupt */
#define GD32_IRQ_DMA1_CH7              (GD32_IRQ_EXINT + 70)  /* Vector 86: DMA1 channel7 interrupt */
#define GD32_IRQ_USART5                (GD32_IRQ_EXINT + 71)  /* Vector 87: USART5 global and wakeup interrupt */
#define GD32_IRQ_I2C2_EV               (GD32_IRQ_EXINT + 72)  /* Vector 88: I2C2 event interrupt */
#define GD32_IRQ_I2C2_ER               (GD32_IRQ_EXINT + 73)  /* Vector 89: I2C2 error interrupt */
#define GD32_IRQ_USBHS_EP1_OUT         (GD32_IRQ_EXINT + 74)  /* Vector 90: USBHS endpoint 1 out interrupt */
#define GD32_IRQ_USBHS_EP1_IN          (GD32_IRQ_EXINT + 75)  /* Vector 91: USBHS endpoint 1 in interrupt */
#define GD32_IRQ_USBHS_WKUP            (GD32_IRQ_EXINT + 76)  /* Vector 92: USBHS wakeup through EXTI line interrupt */
#define GD32_IRQ_USBHS                 (GD32_IRQ_EXINT + 77)  /* Vector 93: USBHS interrupt */
#define GD32_IRQ_DCI                   (GD32_IRQ_EXINT + 78)  /* Vector 94: DCI interrupt */
#define GD32_IRQ_CAU                   (GD32_IRQ_EXINT + 79)  /* Vector 95: CAU interrupt */
#define GD32_IRQ_HAU_TRNG              (GD32_IRQ_EXINT + 80)  /* Vector 96: HAU and TRNG interrupt */
#define GD32_IRQ_FPU                   (GD32_IRQ_EXINT + 81)  /* Vector 97: FPU interrupt */
#define GD32_IRQ_UART6                 (GD32_IRQ_EXINT + 82)  /* Vector 98: UART6 global and wakeup interrupt */
#define GD32_IRQ_UART7                 (GD32_IRQ_EXINT + 83)  /* Vector 99: UART7 global and wakeup interrupt */
#define GD32_IRQ_SPI3                  (GD32_IRQ_EXINT + 84)  /* Vector 100: SPI3 interrupt */
#define GD32_IRQ_SPI4                  (GD32_IRQ_EXINT + 85)  /* Vector 101: SPI4 interrupt */
#define GD32_IRQ_SPI5                  (GD32_IRQ_EXINT + 86)  /* Vector 102: SPI5 interrupt */
#define GD32_IRQ_SAI0                  (GD32_IRQ_EXINT + 87)  /* Vector 103: SAI0 interrupt */
#define GD32_IRQ_TLI                   (GD32_IRQ_EXINT + 88)  /* Vector 104: TLI interrupt */
#define GD32_IRQ_TLI_ER                (GD32_IRQ_EXINT + 89)  /* Vector 105: TLI error interrupt */
#define GD32_IRQ_IPA                   (GD32_IRQ_EXINT + 90)  /* Vector 106: IPA interrupt */
#define GD32_IRQ_SAI1                  (GD32_IRQ_EXINT + 91)  /* Vector 107: SAI1 interrupt */
#define GD32_IRQ_OSPI0                 (GD32_IRQ_EXINT + 92)  /* Vector 108: OSPI0 interrupt */
#define GD32_IRQ_OSPI1                 (GD32_IRQ_EXINT + 93)  /* Vector 109: OSPI1 interrupt */
#define GD32_IRQ_DMAMUX0_OVR           (GD32_IRQ_EXINT + 94)  /* Vector 110: DMAMUX0 overrun interrupt */
#define GD32_IRQ_HPDF_INT0             (GD32_IRQ_EXINT + 95)  /* Vector 111: HPDF filter0 interrupt */
#define GD32_IRQ_HPDF_INT1             (GD32_IRQ_EXINT + 96)  /* Vector 112: HPDF filter1 interrupt */
#define GD32_IRQ_HPDF_INT2             (GD32_IRQ_EXINT + 97)  /* Vector 113: HPDF filter2 interrupt */
#define GD32_IRQ_HPDF_INT3             (GD32_IRQ_EXINT + 98)  /* Vector 114: HPDF filter3 interrupt */
#define GD32_IRQ_SAI2                  (GD32_IRQ_EXINT + 99)  /* Vector 115: SAI2 interrupt */
#define GD32_IRQ_SWPMI                 (GD32_IRQ_EXINT + 100) /* Vector 116: SWPMI interrupt */
#define GD32_IRQ_TIMER14               (GD32_IRQ_EXINT + 101) /* Vector 117: TIMER14 interrupt */
#define GD32_IRQ_TIMER15               (GD32_IRQ_EXINT + 102) /* Vector 118: TIMER15 interrupt */
#define GD32_IRQ_TIMER16               (GD32_IRQ_EXINT + 103) /* Vector 119: TIMER16 interrupt */
#define GD32_IRQ_MDIO                  (GD32_IRQ_EXINT + 104) /* Vector 120: MDIO interrupt */
#define GD32_IRQ_MDMA                  (GD32_IRQ_EXINT + 105) /* Vector 121: MDMA interrupt */
#define GD32_IRQ_SDIO1                 (GD32_IRQ_EXINT + 106) /* Vector 122: SDIO1 interrupt */
#define GD32_IRQ_HWSEM0                (GD32_IRQ_EXINT + 107) /* Vector 123: HWSEM0 interrupt */
#define GD32_IRQ_ADC2                  (GD32_IRQ_EXINT + 108) /* Vector 124: ADC2 interrupt */
#define GD32_IRQ_CMP0_CMP1             (GD32_IRQ_EXINT + 109) /* Vector 125: CMP0 and CMP1 interrupt */
#define GD32_IRQ_CMP2                  (GD32_IRQ_EXINT + 110) /* Vector 126: CMP2 interrupt */
#define GD32_IRQ_CTC                   (GD32_IRQ_EXINT + 111) /* Vector 127: CTC interrupt */
#define GD32_IRQ_RAMECCMU              (GD32_IRQ_EXINT + 112) /* Vector 128: RAMECCMU interrupt */
#define GD32_IRQ_CAN0_BUSOFF           (GD32_IRQ_EXINT + 113) /* Vector 129: CAN0 busoff interrupt */
#define GD32_IRQ_CAN0_ERROR            (GD32_IRQ_EXINT + 114) /* Vector 130: CAN0 error interrupt */
#define GD32_IRQ_CAN1_TX               (GD32_IRQ_EXINT + 115) /* Vector 131: CAN1 TX interrupt */
#define GD32_IRQ_CAN1_RX0              (GD32_IRQ_EXINT + 116) /* Vector 132: CAN1 RX0 interrupt */
#define GD32_IRQ_CAN1_RX1              (GD32_IRQ_EXINT + 117) /* Vector 133: CAN1 RX1 interrupt */
#define GD32_IRQ_CAN1_EWMC             (GD32_IRQ_EXINT + 118) /* Vector 134: CAN1 EWMC interrupt */
#define GD32_IRQ_CAN1_BUSOFF           (GD32_IRQ_EXINT + 119) /* Vector 135: CAN1 busoff interrupt */
#define GD32_IRQ_CAN1_ERROR            (GD32_IRQ_EXINT + 120) /* Vector 136: CAN1 error interrupt */
#define GD32_IRQ_CAN2_TX               (GD32_IRQ_EXINT + 121) /* Vector 137: CAN2 TX interrupt */
#define GD32_IRQ_CAN2_RX0              (GD32_IRQ_EXINT + 122) /* Vector 138: CAN2 RX0 interrupt */
#define GD32_IRQ_CAN2_RX1              (GD32_IRQ_EXINT + 123) /* Vector 139: CAN2 RX1 interrupt */
#define GD32_IRQ_CAN2_EWMC             (GD32_IRQ_EXINT + 124) /* Vector 140: CAN2 EWMC interrupt */
#define GD32_IRQ_CAN2_BUSOFF           (GD32_IRQ_EXINT + 125) /* Vector 141: CAN2 busoff interrupt */
#define GD32_IRQ_CAN2_ERROR            (GD32_IRQ_EXINT + 126) /* Vector 142: CAN2 error interrupt */
#define GD32_IRQ_EFUSE                 (GD32_IRQ_EXINT + 127) /* Vector 143: EFUSE interrupt */
#define GD32_IRQ_I2C3_EV               (GD32_IRQ_EXINT + 128) /* Vector 144: I2C3 event interrupt */
#define GD32_IRQ_I2C3_ER               (GD32_IRQ_EXINT + 129) /* Vector 145: I2C3 error interrupt */
#define GD32_IRQ_TIMER22               (GD32_IRQ_EXINT + 130) /* Vector 146: TIMER22 interrupt */
#define GD32_IRQ_TIMER23               (GD32_IRQ_EXINT + 131) /* Vector 147: TIMER23 interrupt */
#define GD32_IRQ_TIMER30               (GD32_IRQ_EXINT + 132) /* Vector 148: TIMER30 interrupt */
#define GD32_IRQ_TIMER31               (GD32_IRQ_EXINT + 133) /* Vector 149: TIMER31 interrupt */
#define GD32_IRQ_TIMER40               (GD32_IRQ_EXINT + 134) /* Vector 150: TIMER40 interrupt */
#define GD32_IRQ_TIMER41               (GD32_IRQ_EXINT + 135) /* Vector 151: TIMER41 interrupt */
#define GD32_IRQ_TIMER42               (GD32_IRQ_EXINT + 136) /* Vector 152: TIMER42 interrupt */
#define GD32_IRQ_TIMER43               (GD32_IRQ_EXINT + 137) /* Vector 153: TIMER43 interrupt */
#define GD32_IRQ_TIMER44               (GD32_IRQ_EXINT + 138) /* Vector 154: TIMER44 interrupt */
#define GD32_IRQ_TIMER50               (GD32_IRQ_EXINT + 139) /* Vector 155: TIMER50 interrupt */
#define GD32_IRQ_TIMER51               (GD32_IRQ_EXINT + 140) /* Vector 156: TIMER51 interrupt */
#define GD32_IRQ_USART8                (GD32_IRQ_EXINT + 141) /* Vector 157: USART8 global and wakeup interrupt */
#define GD32_IRQ_USART9                (GD32_IRQ_EXINT + 142) /* Vector 158: USART9 global and wakeup interrupt */
#define GD32_IRQ_I2C4_EV               (GD32_IRQ_EXINT + 143) /* Vector 159: I2C4 event interrupt */
#define GD32_IRQ_I2C4_ER               (GD32_IRQ_EXINT + 144) /* Vector 160: I2C4 error interrupt */
#define GD32_IRQ_SPI6                  (GD32_IRQ_EXINT + 145) /* Vector 161: SPI6 interrupt */
#define GD32_IRQ_WIFI                  (GD32_IRQ_EXINT + 146) /* Vector 162: WiFi interrupt */
#define GD32_IRQ_EDOUT0                (GD32_IRQ_EXINT + 147) /* Vector 163: EDOUT0 interrupt */
#define GD32_IRQ_EDOUT1                (GD32_IRQ_EXINT + 148) /* Vector 164: EDOUT1 interrupt */
#define GD32_IRQ_TMU                   (GD32_IRQ_EXINT + 149) /* Vector 165: TMU interrupt */
#define GD32_IRQ_TIMER41_BRK           (GD32_IRQ_EXINT + 150) /* Vector 166: TIMER41 break interrupt */
#define GD32_IRQ_TIMER41_UP            (GD32_IRQ_EXINT + 151) /* Vector 167: TIMER41 update interrupt */
#define GD32_IRQ_TIMER41_TRG_CMT       (GD32_IRQ_EXINT + 152) /* Vector 168: TIMER41 trigger and commutation interrupt */
#define GD32_IRQ_TIMER41_CHANNEL       (GD32_IRQ_EXINT + 153) /* Vector 169: TIMER41 capture compare interrupt */
#define GD32_IRQ_SAI3                  (GD32_IRQ_EXINT + 154) /* Vector 170: SAI3 interrupt */
#define GD32_IRQ_HWSEM1                (GD32_IRQ_EXINT + 155) /* Vector 171: HWSEM1 interrupt */
#define GD32_IRQ_DMAMUX1_OVR           (GD32_IRQ_EXINT + 156) /* Vector 172: DMAMUX1 overrun interrupt */
#define GD32_IRQ_HPDF_INT4             (GD32_IRQ_EXINT + 157) /* Vector 173: HPDF filter4 interrupt */
#define GD32_IRQ_HPDF_INT5             (GD32_IRQ_EXINT + 158) /* Vector 174: HPDF filter5 interrupt */
#define GD32_IRQ_ENET1                 (GD32_IRQ_EXINT + 159) /* Vector 175: ENET1 interrupt */
#define GD32_IRQ_ENET1_WKUP            (GD32_IRQ_EXINT + 160) /* Vector 176: ENET1 wakeup through EXTI line interrupt */
#define GD32_IRQ_I2C5_EV               (GD32_IRQ_EXINT + 161) /* Vector 177: I2C5 event interrupt */
#define GD32_IRQ_I2C5_ER               (GD32_IRQ_EXINT + 162) /* Vector 178: I2C5 error interrupt */
#define GD32_IRQ_CAN3_TX               (GD32_IRQ_EXINT + 163) /* Vector 179: CAN3 TX interrupt */
#define GD32_IRQ_CAN3_RX0              (GD32_IRQ_EXINT + 164) /* Vector 180: CAN3 RX0 interrupt */
#define GD32_IRQ_CAN3_RX1              (GD32_IRQ_EXINT + 165) /* Vector 181: CAN3 RX1 interrupt */
#define GD32_IRQ_CAN3_EWMC             (GD32_IRQ_EXINT + 166) /* Vector 182: CAN3 EWMC interrupt */
#define GD32_IRQ_CAN3_BUSOFF           (GD32_IRQ_EXINT + 167) /* Vector 183: CAN3 busoff interrupt */
#define GD32_IRQ_CAN3_ERROR            (GD32_IRQ_EXINT + 168) /* Vector 184: CAN3 error interrupt */
#define GD32_IRQ_RSPDIF                (GD32_IRQ_EXINT + 169) /* Vector 185: RSPDIF interrupt */
#define GD32_IRQ_CAN4_TX               (GD32_IRQ_EXINT + 170) /* Vector 186: CAN4 TX interrupt */
#define GD32_IRQ_CAN4_RX0              (GD32_IRQ_EXINT + 171) /* Vector 187: CAN4 RX0 interrupt */
#define GD32_IRQ_CAN4_RX1              (GD32_IRQ_EXINT + 172) /* Vector 188: CAN4 RX1 interrupt */
#define GD32_IRQ_CAN4_EWMC             (GD32_IRQ_EXINT + 173) /* Vector 189: CAN4 EWMC interrupt */
#define GD32_IRQ_CAN4_BUSOFF           (GD32_IRQ_EXINT + 174) /* Vector 190: CAN4 busoff interrupt */
#define GD32_IRQ_CAN4_ERROR            (GD32_IRQ_EXINT + 175) /* Vector 191: CAN4 error interrupt */
#define GD32_IRQ_TIMER52               (GD32_IRQ_EXINT + 176) /* Vector 192: TIMER52 interrupt */
#define GD32_IRQ_TIMER53               (GD32_IRQ_EXINT + 177) /* Vector 193: TIMER53 interrupt */
#define GD32_IRQ_TIMER54               (GD32_IRQ_EXINT + 178) /* Vector 194: TIMER54 interrupt */
#define GD32_IRQ_LPDTS                 (GD32_IRQ_EXINT + 179) /* Vector 195: LPDTS interrupt */
#define GD32_IRQ_VREF                  (GD32_IRQ_EXINT + 180) /* Vector 196: VREF ready interrupt */
#define GD32_IRQ_RTC_TEMPER            (GD32_IRQ_EXINT + 181) /* Vector 197: RTC temperature monitor interrupt */
#define GD32_IRQ_PMU_WKUP              (GD32_IRQ_EXINT + 182) /* Vector 198: PMU wakeup event interrupt */
#define GD32_IRQ_PMU_LVDR              (GD32_IRQ_EXINT + 183) /* Vector 199: PMU voltage detection interrupt */
#define GD32_IRQ_CAN5_TX               (GD32_IRQ_EXINT + 184) /* Vector 200: CAN5 TX interrupt */
#define GD32_IRQ_CAN5_RX0              (GD32_IRQ_EXINT + 185) /* Vector 201: CAN5 RX0 interrupt */
#define GD32_IRQ_CAN5_RX1              (GD32_IRQ_EXINT + 186) /* Vector 202: CAN5 RX1 interrupt */
#define GD32_IRQ_CAN5_EWMC             (GD32_IRQ_EXINT + 187) /* Vector 203: CAN5 EWMC interrupt */
#define GD32_IRQ_CAN5_BUSOFF           (GD32_IRQ_EXINT + 188) /* Vector 204: CAN5 busoff interrupt */
#define GD32_IRQ_CAN5_ERROR            (GD32_IRQ_EXINT + 189) /* Vector 205: CAN5 error interrupt */
#define GD32_IRQ_CAN6_TX               (GD32_IRQ_EXINT + 190) /* Vector 206: CAN6 TX interrupt */
#define GD32_IRQ_CAN6_RX0              (GD32_IRQ_EXINT + 191) /* Vector 207: CAN6 RX0 interrupt */
#define GD32_IRQ_CAN6_RX1              (GD32_IRQ_EXINT + 192) /* Vector 208: CAN6 RX1 interrupt */
#define GD32_IRQ_CAN6_EWMC             (GD32_IRQ_EXINT + 193) /* Vector 209: CAN6 EWMC interrupt */
#define GD32_IRQ_CAN6_BUSOFF           (GD32_IRQ_EXINT + 194) /* Vector 210: CAN6 busoff interrupt */
#define GD32_IRQ_CAN6_ERROR            (GD32_IRQ_EXINT + 195) /* Vector 211: CAN6 error interrupt */
#define GD32_IRQ_TIMER31_BRK           (GD32_IRQ_EXINT + 196) /* Vector 212: TIMER31 break interrupt */
#define GD32_IRQ_TIMER31_UP            (GD32_IRQ_EXINT + 197) /* Vector 213: TIMER31 update interrupt */
#define GD32_IRQ_TIMER31_TRG_CMT       (GD32_IRQ_EXINT + 198) /* Vector 214: TIMER31 trigger and commutation interrupt */
#define GD32_IRQ_TIMER31_CHANNEL       (GD32_IRQ_EXINT + 199) /* Vector 215: TIMER31 capture compare interrupt */
#define GD32_IRQ_TIMER31_DEC           (GD32_IRQ_EXINT + 200) /* Vector 216: TIMER31 DEC interrupt */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Total number of IRQ numbers */

#define GD32_IRQ_NEXTINT       201
#define NR_VECTORS             (GD32_IRQ_EXINT + GD32_IRQ_NEXTINT)
#define NR_IRQS                NR_VECTORS

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

#endif /* __ARCH_ARM_INCLUDE_GD32H7_GD32H7XX_IRQ_H */
