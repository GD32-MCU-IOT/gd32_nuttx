/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_ipa.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_IPA_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_IPA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/video/fb.h>

#ifdef CONFIG_FB_OVERLAY

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes DMA2D overlay information */

struct gd32_ipa_overlay_s
{
  uint8_t    fmt;                 /* DMA2D pixel format */
  uint8_t    transp_mode;         /* DMA2D transparency mode */
  fb_coord_t xres;                /* X-resolution overlay */
  fb_coord_t yres;                /* Y-resolution overlay */
  struct fb_overlayinfo_s *oinfo; /* Framebuffer overlay information */
};

/* DMA2D is controlled by the following interface */

struct ipa_layer_s
{
  /* Name: setclut
   *
   * Description:
   *   Set the cmap table for both foreground and background layer.
   *   Up to 256 colors supported.
   *
   * Parameter:
   *   cmap  - Reference to the cmap table
   *
   * Returned Value:
   *   On success - OK
   *   On error   - -EINVAL
   */

#ifdef CONFIG_GD32F4_FB_CMAP
  int (*setclut)(const struct fb_cmap_s * cmap);
#endif

  /* Name: fillcolor
   *
   * Description:
   *   Fill a specific memory region with a color.
   *   The caller must ensure that the memory region (area) is within the
   *   entire overlay.
   *
   * Parameter:
   *   oinfo  - Reference to overlay information
   *   area   - Reference to the area to fill
   *   argb   - argb8888 color
   *
   * Returned Value:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*fillcolor)(struct gd32_ipa_overlay_s *oinfo,
                   const struct fb_area_s *area, uint32_t argb);

  /* Name: blit
   *
   * Description:
   *   Copies memory from a source overlay (defined by sarea) to destination
   *   overlay position (defined by destxpos and destypos) without
   *   pixelformat conversion. The caller must ensure that the memory region
   *   (area) is within the entire overlay.
   *
   * Parameter:
   *   doverlay - Reference destination overlay
   *   destxpos - x-Offset destination overlay
   *   destypos - y-Offset destination overlay
   *   soverlay - Reference source overlay
   *   sarea    - Reference source area
   *
   * Returned Value:
   *   On success - OK
   *   On error   - -EINVAL
   */

  int (*blit)(struct gd32_ipa_overlay_s *doverlay,
              uint32_t destxpos, uint32_t destypos,
              struct gd32_ipa_overlay_s *soverlay,
              const struct fb_area_s *sarea);

  /* Name: blend
   *
   * Description:
   *   Blends two source memory areas to a destination memory area with
   *   pixelformat conversion if necessary. The caller must ensure that the
   *   memory region (area) is within the entire overlays.
   *
   * Parameter:
   *   doverlay - Destination overlay
   *   destxpos - x-Offset destination overlay
   *   destypos - y-Offset destination overlay
   *   foverlay - Foreground overlay
   *   forexpos - x-Offset foreground overlay
   *   foreypos - y-Offset foreground overlay
   *   boverlay - Background overlay
   *   barea    - x-Offset, y-Offset, x-resolution and y-resolution of
   *              background overlay
   *
   * Returned Value:
   *   On success - OK
   *   On error   - -EINVAL or -ECANCELED
   */

  int (*blend)(struct gd32_ipa_overlay_s *doverlay,
               uint32_t destxpos, uint32_t destypos,
               struct gd32_ipa_overlay_s *foverlay,
               uint32_t forexpos, uint32_t foreypos,
               struct gd32_ipa_overlay_s *boverlay,
               const struct fb_area_s *barea);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_ipadev
 *
 * Description:
 *   Get a reference to the IPA controller.
 *
 * Returned Value:
 *   On success - A valid IPA controller reference
 *   On error   - NULL and errno is set to
 *                -EINVAL if one of the parameter is invalid
 *
 ****************************************************************************/

struct ipa_layer_s *gd32_ipadev(void);

/****************************************************************************
 * Name: gd32_ipainitialize
 *
 * Description:
 *   Initialize the IPA controller
 *
 * Returned Value:
 *   OK - On success
 *   An error if initializing failed.
 *
 ****************************************************************************/

int gd32_ipainitialize(void);

/****************************************************************************
 * Name: gd32_ipauninitialize
 *
 * Description:
 *   Uninitialize the IPA controller
 *
 ****************************************************************************/

void gd32_ipauninitialize(void);

#endif /* CONFIG_FB_OVERLAY */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_IPA_H */
