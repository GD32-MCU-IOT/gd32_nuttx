/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_wdg.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_WDG_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_WDG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fwdgt_initialize
 *
 * Description:
 *   Initialize the FWDGT (Free Watchdog Timer) and register the FWDGT
 *   device at the specified device path.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog device.  This should be of
 *     the form /dev/watchdog0
 *   lsifreq - The calibrated frequency of the IRC40K oscillator
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_FWDGT
void gd32_fwdgt_initialize(const char *devpath, uint32_t lsifreq);
#endif

/****************************************************************************
 * Name: gd32_wwdg_initialize
 *
 * Description:
 *   Initialize the WWDG (Window Watchdog) and register the WWDG device at
 *   the specified device path.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog device.  This should be of
 *     the form /dev/watchdog1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_WWDG
void gd32_wwdg_initialize(const char *devpath);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_WDG_H */
