/****************************************************************************
 * arch/arm/src/gd32h7xx/hardware/gd32h7xx_pinmap.h
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

#ifndef __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_PINMAP_H
#define __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* GD32 H7 Family ***********************************************************/

#if defined(CONFIG_ARCH_CHIP_GD32H759IM)
#  include "hardware/gd32h759_pinmap.h"

#else
#  error "No pinmap file for this GD32 chip"
#endif

#endif /* __ARCH_ARM_SRC_GD32H7_HARDWARE_GD32H7XX_PINMAP_H */
