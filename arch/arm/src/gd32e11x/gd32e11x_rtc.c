/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_rtc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* This file is only a thin shell that includes the correct RTC
 * implementation for the selected GD32 family.  The correct file cannot be
 * selected by the make system because it needs the intelligence that only
 * exists in chip.h that can associate an STM32 part number with an STM32
 * family.
 */

/* The GD32 E11x has a simple battery-backed counter for its RTC and has a
 * separate block for the BKP registers.
 */

#include "gd32e11x_rtcounter.c"

/****************************************************************************
 * Public Functions
 ****************************************************************************/
