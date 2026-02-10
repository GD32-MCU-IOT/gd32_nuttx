/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_dbg.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DBG_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DBG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_DBG_ID_OFFSET          0x0000  /* DBG ID code register */
#define GD32_DBG_CTL_OFFSET         0x0004  /* DBG control register */

/* Register Addresses *******************************************************/

#define GD32_DBG_ID                 (GD32_DBG_BASE + GD32_DBG_ID_OFFSET)
#define GD32_DBG_CTL                (GD32_DBG_BASE + GD32_DBG_CTL_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* DBG ID code register (DBG_ID) */

#define DBG_ID_MASK                 (0xffffffff)

/* DBG control register (DBG_CTL) */

#define DBG_CTL_SLP_HOLD            (1 << 0)  /* Bit 0: Sleep mode hold */
#define DBG_CTL_DSLP_HOLD           (1 << 1)  /* Bit 1: Deep-sleep mode hold */
#define DBG_CTL_STB_HOLD            (1 << 2)  /* Bit 2: Standby mode hold */
#define DBG_CTL_TRACE_IOEN          (1 << 5)  /* Bit 5: Trace pin enable */

#define DBG_CTL_TRACE_MODE_SHIFT    (6)       /* Bits 6-7: Trace mode */
#define DBG_CTL_TRACE_MODE_MASK     (0x3 << DBG_CTL_TRACE_MODE_SHIFT)
#  define DBG_CTL_TRACE_ASYNC       (0 << DBG_CTL_TRACE_MODE_SHIFT)
#  define DBG_CTL_TRACE_SYNC_1BIT   (1 << DBG_CTL_TRACE_MODE_SHIFT)
#  define DBG_CTL_TRACE_SYNC_2BIT   (2 << DBG_CTL_TRACE_MODE_SHIFT)
#  define DBG_CTL_TRACE_SYNC_4BIT   (3 << DBG_CTL_TRACE_MODE_SHIFT)

#define DBG_CTL_FWDGT_HOLD          (1 << 8)  /* Bit 8: FWDGT hold in debug */
#define DBG_CTL_WWDGT_HOLD          (1 << 9)  /* Bit 9: WWDGT hold in debug */
#define DBG_CTL_TIMER0_HOLD         (1 << 10) /* Bit 10: TIMER0 hold */
#define DBG_CTL_TIMER1_HOLD         (1 << 11) /* Bit 11: TIMER1 hold */
#define DBG_CTL_TIMER2_HOLD         (1 << 12) /* Bit 12: TIMER2 hold */
#define DBG_CTL_TIMER3_HOLD         (1 << 13) /* Bit 13: TIMER3 hold */
#define DBG_CTL_CAN0_HOLD           (1 << 14) /* Bit 14: CAN0 hold */
#define DBG_CTL_I2C0_HOLD           (1 << 15) /* Bit 15: I2C0 hold */
#define DBG_CTL_I2C1_HOLD           (1 << 16) /* Bit 16: I2C1 hold */
#define DBG_CTL_TIMER4_HOLD         (1 << 18) /* Bit 18: TIMER4 hold */
#define DBG_CTL_TIMER5_HOLD         (1 << 19) /* Bit 19: TIMER5 hold */
#define DBG_CTL_TIMER6_HOLD         (1 << 20) /* Bit 20: TIMER6 hold */
#define DBG_CTL_CAN1_HOLD           (1 << 21) /* Bit 21: CAN1 hold */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_DBG_H */
