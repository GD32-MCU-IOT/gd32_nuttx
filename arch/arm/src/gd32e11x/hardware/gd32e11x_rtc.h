/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_rtc.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_RTC_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_RTC_INTEN_OFFSET      0x0000  /* RTC interrupt enable register */
#define GD32_RTC_CTL_OFFSET        0x0004  /* RTC control register */
#define GD32_RTC_PSCH_OFFSET       0x0008  /* RTC prescaler high register */
#define GD32_RTC_PSCL_OFFSET       0x000c  /* RTC prescaler low register */
#define GD32_RTC_DIVH_OFFSET       0x0010  /* RTC divider high register */
#define GD32_RTC_DIVL_OFFSET       0x0014  /* RTC divider low register */
#define GD32_RTC_CNTH_OFFSET       0x0018  /* RTC counter high register */
#define GD32_RTC_CNTL_OFFSET       0x001c  /* RTC counter low register */
#define GD32_RTC_ALRMH_OFFSET      0x0020  /* RTC alarm high register */
#define GD32_RTC_ALRML_OFFSET      0x0024  /* RTC alarm low register */

/* Register Addresses *******************************************************/

#define GD32_RTC_INTEN             (GD32_RTC_BASE+GD32_RTC_INTEN_OFFSET)
#define GD32_RTC_CTL               (GD32_RTC_BASE+GD32_RTC_CTL_OFFSET)
#define GD32_RTC_PSCH              (GD32_RTC_BASE+GD32_RTC_PSCH_OFFSET)
#define GD32_RTC_PSCL              (GD32_RTC_BASE+GD32_RTC_PSCL_OFFSET)
#define GD32_RTC_DIVH              (GD32_RTC_BASE+GD32_RTC_DIVH_OFFSET)
#define GD32_RTC_DIVL              (GD32_RTC_BASE+GD32_RTC_DIVL_OFFSET)
#define GD32_RTC_CNTH              (GD32_RTC_BASE+GD32_RTC_CNTH_OFFSET)
#define GD32_RTC_CNTL              (GD32_RTC_BASE+GD32_RTC_CNTL_OFFSET)
#define GD32_RTC_ALRMH             (GD32_RTC_BASE+GD32_RTC_ALRMH_OFFSET)
#define GD32_RTC_ALRML             (GD32_RTC_BASE+GD32_RTC_ALRML_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* RTC interrupt enable register */

#define RTC_INTEN_SCIE             (1 << 0)   /* Bit 0: Second interrupt enable */
#define RTC_INTEN_ALRMIE           (1 << 1)   /* Bit 1: Alarm interrupt enable */
#define RTC_INTEN_OVIE             (1 << 2)   /* Bit 2: Overflow interrupt enable */

/* RTC control register */

#define RTC_CTL_SCIF               (1 << 0)   /* Bit 0: Second interrupt flag */
#define RTC_CTL_ALRMIF             (1 << 1)   /* Bit 1: Alarm interrupt flag */
#define RTC_CTL_OVIF               (1 << 2)   /* Bit 2: Overflow interrupt flag */
#define RTC_CTL_RSYNF              (1 << 3)   /* Bit 3: Registers synchronized flag */
#define RTC_CTL_CMF                (1 << 4)   /* Bit 4: Configuration mode flag */
#define RTC_CTL_LWOFF              (1 << 5)   /* Bit 5: Last write operation finished flag */

/* RTC prescaler high register */

#define RTC_PSCH_PSC_SHIFT         (0)        /* Bits 0-3: Prescaler high value */
#define RTC_PSCH_PSC_MASK          (0x0f << RTC_PSCH_PSC_SHIFT)

/* RTC prescaler low register */

#define RTC_PSCL_PSC_SHIFT         (0)        /* Bits 0-15: Prescaler low value */
#define RTC_PSCL_PSC_MASK          (0xffff << RTC_PSCL_PSC_SHIFT)

/* RTC divider high register */

#define RTC_DIVH_DIV_SHIFT         (0)        /* Bits 0-3: Divider high value */
#define RTC_DIVH_DIV_MASK          (0x0f << RTC_DIVH_DIV_SHIFT)

/* RTC divider low register */

#define RTC_DIVL_DIV_SHIFT         (0)        /* Bits 0-15: Divider low value */
#define RTC_DIVL_DIV_MASK          (0xffff << RTC_DIVL_DIV_SHIFT)

/* RTC counter high register */

#define RTC_CNTH_CNT_SHIFT         (0)        /* Bits 0-15: Counter high value */
#define RTC_CNTH_CNT_MASK          (0xffff << RTC_CNTH_CNT_SHIFT)

/* RTC counter low register */

#define RTC_CNTL_CNT_SHIFT         (0)        /* Bits 0-15: Counter low value */
#define RTC_CNTL_CNT_MASK          (0xffff << RTC_CNTL_CNT_SHIFT)

/* RTC alarm high register */

#define RTC_ALRMH_ALRM_SHIFT       (0)        /* Bits 0-15: Alarm high value */
#define RTC_ALRMH_ALRM_MASK        (0xffff << RTC_ALRMH_ALRM_SHIFT)

/* RTC alarm low register */

#define RTC_ALRML_ALRM_SHIFT       (0)        /* Bits 0-15: Alarm low value */
#define RTC_ALRML_ALRM_MASK        (0xffff << RTC_ALRML_ALRM_SHIFT)

/* RTC interrupt definitions */

#define RTC_INT_SECOND             RTC_INTEN_SCIE
#define RTC_INT_ALARM              RTC_INTEN_ALRMIE
#define RTC_INT_OVERFLOW           RTC_INTEN_OVIE

/* RTC interrupt flag definitions */

#define RTC_INT_FLAG_SECOND        RTC_CTL_SCIF
#define RTC_INT_FLAG_ALARM         RTC_CTL_ALRMIF
#define RTC_INT_FLAG_OVERFLOW      RTC_CTL_OVIF

/* RTC flag definitions */
#define RTC_FLAG_SECOND            RTC_CTL_SCIF
#define RTC_FLAG_ALARM             RTC_CTL_ALRMIF
#define RTC_FLAG_OVERFLOW          RTC_CTL_OVIF
#define RTC_FLAG_RSYN              RTC_CTL_RSYNF
#define RTC_FLAG_LWOF              RTC_CTL_LWOFF

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_RTC_H */
