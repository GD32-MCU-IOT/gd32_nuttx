/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_dci.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_DCI_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_DCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/video/imgdata.h>

#include "chip.h"
#include "hardware/gd32f4xx_dci.h"

#ifdef CONFIG_GD32F4_DCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DCI capture mode */

#define DCI_CAPTURE_MODE_CONTINUOUS  0   /* Continuous capture mode */
#define DCI_CAPTURE_MODE_SNAPSHOT    1   /* Snapshot capture mode */

/* DCI clock polarity */

#define DCI_CK_POLARITY_FALLING     0   /* Capture at falling edge */
#define DCI_CK_POLARITY_RISING      1   /* Capture at rising edge */

/* DCI horizontal sync polarity */

#define DCI_HSYNC_POLARITY_LOW      0   /* Low level during blanking period */
#define DCI_HSYNC_POLARITY_HIGH     1   /* High level during blanking period */

/* DCI vertical sync polarity */

#define DCI_VSYNC_POLARITY_LOW      0   /* Low level during blanking period */
#define DCI_VSYNC_POLARITY_HIGH     1   /* High level during blanking period */

/* DCI interface format */

#define DCI_INTERFACE_FORMAT_8BITS  0   /* 8-bit data on every pixel clock */
#define DCI_INTERFACE_FORMAT_10BITS 1   /* 10-bit data on every pixel clock */
#define DCI_INTERFACE_FORMAT_12BITS 2   /* 12-bit data on every pixel clock */
#define DCI_INTERFACE_FORMAT_14BITS 3   /* 14-bit data on every pixel clock */

/* DCI frame rate */

#define DCI_FRAME_RATE_ALL          0   /* Capture all frames */
#define DCI_FRAME_RATE_1_2          1   /* Capture one in 2 frames */
#define DCI_FRAME_RATE_1_4          2   /* Capture one in 4 frames */

/****************************************************************************
 * Public Function Prototypes
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
 * Name: gd32_dci_initialize
 *
 * Description:
 *   Initialize the DCI driver.
 *
 * Returned Value:
 *   Upon successful, a reference to imgdata_s is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct imgdata_s *gd32_dci_initialize(void);

/****************************************************************************
 * Name: gd32_dci_uninitialize
 *
 * Description:
 *   Uninitialize the DCI driver.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_dci_uninitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_GD32F4_DCI */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_DCI_H */
