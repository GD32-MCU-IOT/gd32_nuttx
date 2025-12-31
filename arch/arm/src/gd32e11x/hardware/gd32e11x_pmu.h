/****************************************************************************
 * arch/arm/src/gd32e11x/hardware/gd32e11x_pmu.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_PMU_H
#define __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_PMU_CTL_OFFSET        0x0000            /* Power control register offset */
#define GD32_PMU_CS_OFFSET         0x0004            /* Power control and status register offset */

/* Register Addresses *******************************************************/

#define GD32_PMU_CTL               (GD32_PMU_BASE+GD32_PMU_CTL_OFFSET)       /* Power control register */
#define GD32_PMU_CS                (GD32_PMU_BASE+GD32_PMU_CS_OFFSET)        /* Power control and status register */

/* Register Bitfield Definitions ********************************************/

/* Power control register */

#define PMU_CTL_LDOLP              (1 << 0)          /* Bit 0: LDO low power mode */
#define PMU_CTL_STBMOD             (1 << 1)          /* Bit 1: Standby mode */
#define PMU_CTL_WURST              (1 << 2)          /* Bit 2: Wakeup flag reset */
#define PMU_CTL_STBRST             (1 << 3)          /* Bit 3: Standby flag reset */
#define PMU_CTL_LVDEN              (1 << 4)          /* Bit 4: Low voltage detector enable */

#define PMU_CTL_LVDT_SHIFT         (5)               /* Bits 5-7: Low voltage detector threshold */
#define PMU_CTL_LVDT_MASK          (7 << PMU_CTL_LVDT_SHIFT)
#  define PMU_CTL_LVDT(n)          ((n) << PMU_CTL_LVDT_SHIFT)
#  define PMU_LVDT_0               PMU_CTL_LVDT(0)   /* Voltage threshold is 2.1V */
#  define PMU_LVDT_1               PMU_CTL_LVDT(1)   /* Voltage threshold is 2.3V */
#  define PMU_LVDT_2               PMU_CTL_LVDT(2)   /* Voltage threshold is 2.4V */
#  define PMU_LVDT_3               PMU_CTL_LVDT(3)   /* Voltage threshold is 2.6V */
#  define PMU_LVDT_4               PMU_CTL_LVDT(4)   /* Voltage threshold is 2.7V */
#  define PMU_LVDT_5               PMU_CTL_LVDT(5)   /* Voltage threshold is 2.9V */
#  define PMU_LVDT_6               PMU_CTL_LVDT(6)   /* Voltage threshold is 3.0V */
#  define PMU_LVDT_7               PMU_CTL_LVDT(7)   /* Voltage threshold is 3.1V */

#define PMU_CTL_BKPWEN             (1 << 8)          /* Bit 8: Backup domain write enable */

#define PMU_CTL_LDOVS_SHIFT        (14)              /* Bits 14-15: LDO output voltage select */
#define PMU_CTL_LDOVS_MASK         (3 << PMU_CTL_LDOVS_SHIFT)
#  define PMU_CTL_LDOVS(n)         ((n) << PMU_CTL_LDOVS_SHIFT)
#  define PMU_LDOVS_NORMAL         PMU_CTL_LDOVS(1)  /* LDO output voltage select normal mode */
#  define PMU_LDOVS_LOW            PMU_CTL_LDOVS(3)  /* LDO output voltage select low mode */

/* Power control and status register */

#define PMU_CS_WUF                 (1 << 0)          /* Bit 0: Wakeup flag */
#define PMU_CS_STBF                (1 << 1)          /* Bit 1: Standby flag */
#define PMU_CS_LVDF                (1 << 2)          /* Bit 2: Low voltage detector status flag */
#define PMU_CS_WUPEN               (1 << 8)          /* Bit 8: Wakeup pin enable */

/* PMU ldo definitions */

#define PMU_LDO_NORMAL             (0)               /* LDO normal work when PMU enter deepsleep mode */
#define PMU_LDO_LOWPOWER           PMU_CTL_LDOLP     /* LDO work at low power status when PMU enter deepsleep mode */

/* PMU flag definitions */

#define PMU_FLAG_WAKEUP            PMU_CS_WUF        /* Wakeup flag status */
#define PMU_FLAG_STANDBY           PMU_CS_STBF       /* Standby flag status */
#define PMU_FLAG_LVD               PMU_CS_LVDF       /* LVD flag status */

/* PMU flag reset definitions */

#define PMU_FLAG_RESET_WAKEUP      (0x00)            /* Wakeup flag reset */
#define PMU_FLAG_RESET_STANDBY     (0x01)            /* Standby flag reset */

/* PMU command constants definitions */

#define WFI_CMD                    (0x00)            /* Use WFI command */
#define WFE_CMD                    (0x01)            /* Use WFE command */

#endif /* __ARCH_ARM_SRC_GD32E11X_HARDWARE_GD32E11X_PMU_H */
