/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_tli.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_TLI_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_TLI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/video/fb.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_tlireset
 *
 * Description:
 *   Reset TLI via APB2RSTR
 *
 ****************************************************************************/

void gd32_tlireset(void);

/****************************************************************************
 * Name: gd32_tliinitialize
 *
 * Description:
 *   Initialize the TLI controller
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

int gd32_tliinitialize(void);

/****************************************************************************
 * Name: gd32_tliuninitialize
 *
 * Description:
 *   Uninitialize the TLI controller
 *
 ****************************************************************************/

void gd32_tliuninitialize(void);

/****************************************************************************
 * Name: gd32_tligetvplane
 *
 * Description:
 *   Get video plane reference used by framebuffer interface
 *
 * Parameter:
 *   vplane - Video plane
 *
 * Returned Value:
 *   Video plane reference
 *
 ****************************************************************************/

struct fb_vtable_s *gd32_tligetvplane(int vplane);

/****************************************************************************
 * Name: gd32_backlight
 *
 * Description:
 *   If CONFIG_GD32F4_LCD_BACKLIGHT is defined, then the board-specific logic
 *   must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_LCD_BACKLIGHT
void gd32_backlight(bool blon);
#endif
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_TLI_H */
