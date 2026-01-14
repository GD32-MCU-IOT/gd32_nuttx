/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_serial.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_SERIAL_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "chip.h"

#if defined(CONFIG_GD32E11X_GD32E11X)
#  include "hardware/gd32e11x_usart.h"
#else
#  error "Unsupported GD32 USART"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Make sure that we have not enabled more U[S]ARTs than are supported by
 * the device.
 */

#if GD32_NUSART < 5
#  undef CONFIG_GD32E11X_UART4
#endif
#if GD32_NUSART < 4
#  undef CONFIG_GD32E11X_UART3
#endif
#if GD32_NUSART < 3
#  undef CONFIG_GD32E11X_USART2
#endif
#if GD32_NUSART < 2
#  undef CONFIG_GD32E11X_USART1
#endif
#if GD32_NUSART < 1
#  undef CONFIG_GD32E11X_USART0
#endif

/* Sanity checks */

#if !defined(CONFIG_GD32E11X_USART0)
#  undef CONFIG_GD32E11X_USART0_SERIALDRIVER
#  undef CONFIG_GD32E11X_USART0_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32E11X_USART1)
#  undef CONFIG_GD32E11X_USART1_SERIALDRIVER
#  undef CONFIG_GD32E11X_USART1_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32E11X_USART2)
#  undef CONFIG_GD32E11X_USART2_SERIALDRIVER
#  undef CONFIG_GD32E11X_USART2_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32E11X_UART3)
#  undef CONFIG_GD32E11X_UART3_SERIALDRIVER
#  undef CONFIG_GD32E11X_UART3_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32E11X_UART4)
#  undef CONFIG_GD32E11X_UART4_SERIALDRIVER
#  undef CONFIG_GD32E11X_UART4_1WIREDRIVER
#endif

/* Check 1-Wire and U(S)ART conflicts */

#if defined(CONFIG_GD32E11X_USART0_1WIREDRIVER) && defined(CONFIG_GD32E11X_USART0_SERIALDRIVER)
#  error "Both CONFIG_GD32E11X_USART0_1WIREDRIVER and CONFIG_GD32E11X_USART0_SERIALDRIVER defined"
#  undef CONFIG_GD32E11X_USART0_1WIREDRIVER
#endif
#if defined(CONFIG_GD32E11X_USART1_1WIREDRIVER) && defined(CONFIG_GD32E11X_USART1_SERIALDRIVER)
#  error "Both CONFIG_GD32E11X_USART1_1WIREDRIVER and CONFIG_GD32E11X_USART1_SERIALDRIVER defined"
#  undef CONFIG_GD32E11X_USART1_1WIREDRIVER
#endif
#if defined(CONFIG_GD32E11X_USART2_1WIREDRIVER) && defined(CONFIG_GD32E11X_USART2_SERIALDRIVER)
#  error "Both CONFIG_GD32E11X_USART2_1WIREDRIVER and CONFIG_GD32E11X_USART2_SERIALDRIVER defined"
#  undef CONFIG_GD32E11X_USART2_1WIREDRIVER
#endif
#if defined(CONFIG_GD32E11X_UART3_1WIREDRIVER) && defined(CONFIG_GD32E11X_UART3_SERIALDRIVER)
#  error "Both CONFIG_GD32E11X_UART3_1WIREDRIVER and CONFIG_GD32E11X_UART3_SERIALDRIVER defined"
#  undef CONFIG_GD32E11X_UART3_1WIREDRIVER
#endif
#if defined(CONFIG_GD32E11X_UART4_1WIREDRIVER) && defined(CONFIG_GD32E11X_UART4_SERIALDRIVER)
#  error "Both CONFIG_GD32E11X_UART4_1WIREDRIVER and CONFIG_GD32E11X_UART4_SERIALDRIVER defined"
#  undef CONFIG_GD32E11X_UART4_1WIREDRIVER
#endif

/* Is the serial driver enabled? */

#if defined(CONFIG_GD32E11X_USART0_SERIALDRIVER) || defined(CONFIG_GD32E11X_USART1_SERIALDRIVER) || \
    defined(CONFIG_GD32E11X_USART2_SERIALDRIVER) || defined(CONFIG_GD32E11X_UART3_SERIALDRIVER)  || \
    defined(CONFIG_GD32E11X_UART4_SERIALDRIVER)
#  define HAVE_SERIALDRIVER 1
#endif

/* Is the 1-Wire driver? */

#if defined(CONFIG_GD32E11X_USART0_1WIREDRIVER) || defined(CONFIG_GD32E11X_USART1_1WIREDRIVER) || \
    defined(CONFIG_GD32E11X_USART2_1WIREDRIVER) || defined(CONFIG_GD32E11X_UART3_1WIREDRIVER) || \
    defined(CONFIG_GD32E11X_UART4_1WIREDRIVER)
#  define HAVE_1WIREDRIVER 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART0_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define CONSOLE_UART 0
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART1_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define CONSOLE_UART 1
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART2_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define CONSOLE_UART 2
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_UART3_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define CONSOLE_UART 3
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_UART4_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define CONSOLE_UART 4
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONSOLE_UART
#  undef HAVE_CONSOLE
#endif

/* DMA support is only provided if CONFIG_ARCH_DMA is in the
 * NuttX configuration
 */

#if !defined(HAVE_SERIALDRIVER) || !defined(CONFIG_ARCH_DMA)
#  undef CONFIG_GD32E11X_USART0_RXDMA
#  undef CONFIG_GD32E11X_USART0_TXDMA
#  undef CONFIG_GD32E11X_USART1_RXDMA
#  undef CONFIG_GD32E11X_USART1_TXDMA
#  undef CONFIG_GD32E11X_USART2_RXDMA
#  undef CONFIG_GD32E11X_USART2_TXDMA
#  undef CONFIG_GD32E11X_UART3_RXDMA
#  undef CONFIG_GD32E11X_UART3_TXDMA
#  undef CONFIG_GD32E11X_UART4_RXDMA
#  undef CONFIG_GD32E11X_UART4_TXDMA
#endif

/* Disable the DMA configuration on all unused USARTs */

#ifndef CONFIG_GD32E11X_USART0_SERIALDRIVER
#  undef CONFIG_GD32E11X_USART0_RXDMA
#  undef CONFIG_GD32E11X_USART0_TXDMA
#endif

#ifndef CONFIG_GD32E11X_USART1_SERIALDRIVER
#  undef CONFIG_GD32E11X_USART1_RXDMA
#  undef CONFIG_GD32E11X_USART1_TXDMA
#endif

#ifndef CONFIG_GD32E11X_USART2_SERIALDRIVER
#  undef CONFIG_GD32E11X_USART2_RXDMA
#  undef CONFIG_GD32E11X_USART2_TXDMA
#endif

#ifndef CONFIG_GD32E11X_UART3_SERIALDRIVER
#  undef CONFIG_GD32E11X_UART3_RXDMA
#  undef CONFIG_GD32E11X_UART3_TXDMA
#endif

#ifndef CONFIG_GD32E11X_UART4_SERIALDRIVER
#  undef CONFIG_GD32E11X_UART4_RXDMA
#  undef CONFIG_GD32E11X_UART4_TXDMA
#endif

/* Is RX DMA available on any (enabled) USART? */

#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_GD32E11X_USART0_RXDMA) || defined(CONFIG_GD32E11X_USART1_RXDMA) || \
    defined(CONFIG_GD32E11X_USART2_RXDMA) || defined(CONFIG_GD32E11X_UART3_RXDMA)  || \
    defined(CONFIG_GD32E11X_UART4_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
#endif

/* Is TX DMA available on any (enabled) USART? */

#undef SERIAL_HAVE_TXDMA
#if defined(CONFIG_GD32E11X_USART0_TXDMA) || defined(CONFIG_GD32E11X_USART1_TXDMA) || \
    defined(CONFIG_GD32E11X_USART2_TXDMA) || defined(CONFIG_GD32E11X_UART3_TXDMA)  || \
    defined(CONFIG_GD32E11X_UART4_TXDMA)
#  define SERIAL_HAVE_TXDMA 1
#endif

/* Is RX DMA used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_RXDMA
#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART0_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART1_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART2_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_UART3_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_UART4_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#endif

/* Is TX DMA used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_TXDMA
#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART0_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART1_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_USART2_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_UART3_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_GD32E11X_UART4_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#endif

/* No DMA */

#undef SERIAL_NOT_HAVE_DMA
#if defined(CONFIG_GD32E11X_USART0) && !defined(CONFIG_GD32E11X_USART0_RXDMA) && \
    !defined(CONFIG_GD32E11X_USART0_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32E11X_USART1) && !defined(CONFIG_GD32E11X_USART1_RXDMA) && \
    !defined(CONFIG_GD32E11X_USART1_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32E11X_USART2) && !defined(CONFIG_GD32E11X_USART2_RXDMA) && \
    !defined(CONFIG_GD32E11X_USART2_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32E11X_UART3) && !defined(CONFIG_GD32E11X_UART3_RXDMA) && \
    !defined(CONFIG_GD32E11X_UART3_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32E11X_UART4) && !defined(CONFIG_GD32E11X_UART4_RXDMA) && \
    !defined(CONFIG_GD32E11X_UART4_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#endif

/* RX DMA */

#undef SERIAL_HAVE_RX_DMA
#if defined(CONFIG_GD32E11X_USART0_RXDMA) && !defined(CONFIG_GD32E11X_USART0_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32E11X_USART1_RXDMA) && !defined(CONFIG_GD32E11X_USART1_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32E11X_USART2_RXDMA) && !defined(CONFIG_GD32E11X_USART2_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32E11X_UART3_RXDMA) && !defined(CONFIG_GD32E11X_UART3_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32E11X_UART4_RXDMA) && !defined(CONFIG_GD32E11X_UART4_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#endif

/* TX DMA */

#undef SERIAL_HAVE_TX_DMA
#if !defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32E11X_USART1_RXDMA) && defined(CONFIG_GD32E11X_USART1_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32E11X_USART2_RXDMA) && defined(CONFIG_GD32E11X_USART2_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32E11X_UART3_RXDMA) && defined(CONFIG_GD32E11X_UART3_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32E11X_UART4_RXDMA) && defined(CONFIG_GD32E11X_UART4_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#endif

/* RX and TX DMA */

#undef SERIAL_HAVE_RXTX_DMA
#if defined(CONFIG_GD32E11X_USART0_RXDMA) && defined(CONFIG_GD32E11X_USART0_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32E11X_USART1_RXDMA) && defined(CONFIG_GD32E11X_USART1_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32E11X_USART2_RXDMA) && defined(CONFIG_GD32E11X_USART2_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32E11X_UART3_RXDMA) && defined(CONFIG_GD32E11X_UART3_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32E11X_UART4_RXDMA) && defined(CONFIG_GD32E11X_UART4_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#endif

/* Is RS-485 used? */

#if defined(CONFIG_USART0_RS485) || defined(CONFIG_USART1_RS485) || \
    defined(CONFIG_USART2_RS485) || defined(CONFIG_UART3_RS485)  || \
    defined(CONFIG_UART4_RS485)
#  define HAVE_RS485 1
#endif

#ifdef HAVE_RS485
#  define USART_CTL0_USED_INTS    USART_CTL0_INT_MASK
#else
#  define USART_CTL0_USED_INTS    (USART_CTL0_RBNEIE | USART_CTL0_TBEIE | USART_CTL0_PERRIE)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_serial_get_uart
 *
 * Description:
 *   Get serial driver structure for GD32 USART
 *
 ****************************************************************************/

uart_dev_t *gd32_serial_get_uart(int uart_num);

/****************************************************************************
 * Name: gd32_serial_dma_poll
 *
 * Description:
 *   Must be called periodically if any GD32 UART is configured for DMA.
 *   The DMA callback is triggered for each fifo size/2 bytes, but this can
 *   result in some bytes being transferred but not collected if the incoming
 *   data is not a whole multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void gd32_serial_dma_poll(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_SERIAL_H */
