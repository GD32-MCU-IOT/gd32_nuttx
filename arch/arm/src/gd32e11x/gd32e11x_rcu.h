/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_rcu.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_RCU_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_RCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Default clock assumptions for minimal bring-up.
 * Boards can override these by defining the macros before including this
 * header.
 */

#ifndef GD32_IRC8M_FREQUENCY
#  define GD32_IRC8M_FREQUENCY   8000000u
#endif

#ifndef GD32_SYSCLK_FREQUENCY
#  define GD32_SYSCLK_FREQUENCY  GD32_IRC8M_FREQUENCY
#endif

#ifndef GD32_HCLK_FREQUENCY
#  define GD32_HCLK_FREQUENCY    GD32_SYSCLK_FREQUENCY
#endif

#ifndef GD32_PCLK1_FREQUENCY
#  define GD32_PCLK1_FREQUENCY   GD32_HCLK_FREQUENCY
#endif

#ifndef GD32_PCLK2_FREQUENCY
#  define GD32_PCLK2_FREQUENCY   GD32_HCLK_FREQUENCY
#endif

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void gd32_clockconfig(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_RCU_H */
