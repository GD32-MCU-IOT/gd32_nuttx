/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_dac.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_DAC_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/gd32e11x_dac.h"

#include <nuttx/analog/dac.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic DAC outputs.  If CONFIG_GD32E11X_TIMERn is defined
 * then CONFIG_GD32E11X_TIMERn_DAC must also be defined to indicate that
 * timer "n" is intended to be used for that purpose.
 */

#ifndef CONFIG_GD32E11X_TIMER0
#  undef CONFIG_GD32E11X_TIMER0_DAC
#endif
#ifndef CONFIG_GD32E11X_TIMER1
#  undef CONFIG_GD32E11X_TIMER1_DAC
#endif
#ifndef CONFIG_GD32E11X_TIMER2
#  undef CONFIG_GD32E11X_TIMER2_DAC
#endif
#ifndef CONFIG_GD32E11X_TIMER3
#  undef CONFIG_GD32E11X_TIMER3_DAC
#endif
#ifndef CONFIG_GD32E11X_TIMER4
#  undef CONFIG_GD32E11X_TIMER4_DAC
#endif
#ifndef CONFIG_GD32E11X_TIMER5
#  undef CONFIG_GD32E11X_TIMER5_DAC
#endif
#ifndef CONFIG_GD32E11X_TIMER6
#  undef CONFIG_GD32E11X_TIMER6_DAC
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gd32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   intf - The DAC interface number (0 for OUT0, 1 for OUT1).
 *
 * Returned Value:
 *   Valid DAC device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct dac_dev_s;
struct dac_dev_s *gd32_dacinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_DAC_H */
