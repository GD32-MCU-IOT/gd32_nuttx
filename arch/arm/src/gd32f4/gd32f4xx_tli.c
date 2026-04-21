/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_tli.c
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

/* References:
 *   GD32F4xx User Manual
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "chip.h"
#include "gd32f4xx.h"
#include "gd32f4xx_tli.h"
#include "gd32f4xx_ipa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definition ******************************************************/

#ifndef BOARD_TLI_WIDTH
#error BOARD_TLI_WIDTH must be defined in the board.h header file
#endif

#ifndef BOARD_TLI_HEIGHT
#error BOARD_TLI_HEIGHT must be defined in the board.h header file
#endif

#define GD32_TLI_HEIGHT BOARD_TLI_HEIGHT
#define GD32_TLI_WIDTH BOARD_TLI_WIDTH

/* Configure TLI register */

/* TLI_LxWHPCR register */

#define GD32_TLI_LXHPOS_WLP (BOARD_TLI_HSYNC + BOARD_TLI_HBP - 1)
#define GD32_TLI_LXHPOS_WRP (BOARD_TLI_HSYNC + BOARD_TLI_HBP + \
                             GD32_TLI_WIDTH - 1)

/* TLI_LxWVPCR register */

#define GD32_TLI_LXVPOS_WTP (BOARD_TLI_VSYNC + BOARD_TLI_VBP - 1)
#define GD32_TLI_LXVPOS_WRP (BOARD_TLI_VSYNC + BOARD_TLI_VBP + \
                             GD32_TLI_HEIGHT - 1)

/* TLI_SSCR register */

#define GD32_TLI_SPSZ_VPSZ TLI_SPSZ_VPSZ(BOARD_TLI_VSYNC - 1)
#define GD32_TLI_SPSZ_HPSZ TLI_SPSZ_HPSZ(BOARD_TLI_HSYNC - 1)

/* TLI_BPCR register */

#define GD32_TLI_BPSZ_VBPSZ TLI_BPSZ_VBPSZ(GD32_TLI_LXVPOS_WTP)
#define GD32_TLI_BPSZ_HBPSZ TLI_BPSZ_HBPSZ(GD32_TLI_LXHPOS_WLP)

/* TLI_AWCR register */

#define GD32_TLI_ASZ_VASZ TLI_ASZ_VASZ(GD32_TLI_LXVPOS_WRP)
#define GD32_TLI_ASZ_HASZ TLI_ASZ_HASZ(GD32_TLI_LXHPOS_WRP)

/* TLI_TWCR register */

#define GD32_TLI_TSZ_VTSZ TLI_TSZ_VTSZ(BOARD_TLI_VSYNC + \
                                       BOARD_TLI_VBP +   \
                                       GD32_TLI_HEIGHT + BOARD_TLI_VFP - 1)
#define GD32_TLI_TSZ_HTSZ TLI_TSZ_HTSZ(BOARD_TLI_HSYNC + \
                                       BOARD_TLI_HBP +   \
                                       GD32_TLI_WIDTH + BOARD_TLI_HFP - 1)

/* Global GCR register */

#define BOARD_TLI_GCR_BCR 0xFF /* LI_GCR_BCR */

#define BOARD_TLI_GCR_BCG 0xFF /* LI_GCR_BCG */

#define BOARD_TLI_GCR_BCB 0xFF /* LI_GCR_BCB */

#define GD32_TLI_GCR_BCR BOARD_TLI_GCR_BCR
#define GD32_TLI_GCR_BCG BOARD_TLI_GCR_BCG
#define GD32_TLI_GCR_BCB BOARD_TLI_GCR_BCB

/* Synchronisation and Polarity */

#define GD32_TLI_CTL_CLKPS BOARD_TLI_GCR_PCPOL
#define GD32_TLI_CTL_DEPS BOARD_TLI_GCR_DEPOL
#define GD32_TLI_CTL_VPPS BOARD_TLI_GCR_VSPOL
#define GD32_TLI_CTL_HPPS BOARD_TLI_GCR_HSPOL

/* Dither */

#define GD32_TLI_CTL_DFEN BOARD_TLI_CTL_DFEN            /* TODO */
#define GD32_TLI_CTL_BDB TLI_CTL_BDB(BOARD_TLI_CTL_BDB) /* TODO */
#define GD32_TLI_CTL_GDB TLI_CTL_GDB(BOARD_TLI_CTL_GDB) /* TODO */
#define GD32_TLI_CTL_RDB TLI_CTL_BDB(BOARD_TLI_CTL_RDB) /* TODO */

/* LIPCR register */

#define GD32_TLI_LM_LM TLI_LM_LM(GD32_TLI_TSZ_HTSZ)

/* Configuration ************************************************************/

#ifndef CONFIG_GD32F4_TLI_DEFBACKLIGHT
#define CONFIG_GD32F4_TLI_DEFBACKLIGHT 0xf0
#endif
#define GD32_TLI_BACKLIGHT_OFF 0x00

/* Color/video formats */

/* Layer 1 format */

#if defined(CONFIG_GD32F4_TLI_L1_L8)
#define GD32_TLI_L1_BPP 8
#define GD32_TLI_L1_COLOR_FMT FB_FMT_RGB8
#define GD32_TLI_L1PPF_PF TLI_LXPPF_PPF(TLI_PF_L8)
#define GD32_TLI_L1_IPA_PF IPA_PF_L8
#define GD32_TLI_L1CMAP
#elif defined(CONFIG_GD32F4_TLI_L1_RGB565)
#define GD32_TLI_L1_BPP 16
#define GD32_TLI_L1_COLOR_FMT FB_FMT_RGB16_565
#define GD32_TLI_L1PPF_PF TLI_LXPPF_PPF(TLI_PF_RGB565)
#define GD32_TLI_L1_IPA_PF IPA_PF_RGB565
#elif defined(CONFIG_GD32F4_TLI_L1_RGB888)
#define GD32_TLI_L1_BPP 24
#define GD32_TLI_L1_COLOR_FMT FB_FMT_RGB24
#define GD32_TLI_L1PPF_PF TLI_LXPPF_PPF(TLI_PF_RGB888)
#define GD32_TLI_L1_IPA_PF IPA_PF_RGB888
#elif defined(CONFIG_GD32F4_TLI_L1_ARGB8888)
#define GD32_TLI_L1_BPP 32
#define GD32_TLI_L1_COLOR_FMT FB_FMT_RGB32
#define GD32_TLI_L1PPF_PF TLI_LXPPF_PPF(TLI_PF_ARGB8888)
#define GD32_TLI_L1_IPA_PF IPA_PF_ARGB8888
#else
#error "TLI pixel format not supported"
#endif

/* Layer 2 format */

#ifdef CONFIG_GD32F4_TLI_L2
#if defined(CONFIG_GD32F4_TLI_L2_L8)
#define GD32_TLI_L2_BPP 8
#define GD32_TLI_L2_COLOR_FMT FB_FMT_RGB8
#define GD32_TLI_L2PPF_PF TLI_LXPPF_PPF(TLI_PF_L8)
#define GD32_TLI_L2_IPA_PF IPA_PF_L8
#define GD32_LTLI_L2CMAP
#elif defined(CONFIG_GD32F4_TLI_L2_RGB565)
#define GD32_TLI_L2_BPP 16
#define GD32_TLI_L2_COLOR_FMT FB_FMT_RGB16_565
#define GD32_TLI_L2PPF_PF TLI_LXPPF_PPF(TLI_PF_RGB565)
#define GD32_TLI_L2_IPA_PF IPA_PF_RGB565
#elif defined(CONFIG_GD32F4_TLI_L2_RGB888)
#define GD32_TLI_L2_BPP 24
#define GD32_TLI_L2_COLOR_FMT FB_FMT_RGB24
#define GD32_TLI_L2PPF_PF TLI_LXPPF_PPF(TLI_PF_RGB888)
#define GD32_TLI_L2_IPA_PF IPA_PF_RGB888
#elif defined(CONFIG_GD32F4_TLI_L2_ARGB8888)
#define GD32_TLI_L2_BPP 32
#define GD32_TLI_L2_COLOR_FMT FB_FMT_RGB32
#define GD32_TLI_L2PPF_PF TLI_LXPPF_PPF(TLI_PF_ARGB8888)
#define GD32_TLI_L2_IPA_PF IPA_PF_ARGB8888
#else
#error "TLI pixel format not supported"
#endif
#endif /* CONFIG_GD32F4_TLI_L2 */

/* Framebuffer sizes in bytes */

#if GD32_TLI_L1_BPP == 8
#define GD32_TLI_L1_STRIDE (GD32_TLI_WIDTH)
#elif GD32_TLI_L1_BPP == 16
#define GD32_TLI_L1_STRIDE ((GD32_TLI_WIDTH * 16 + 7) / 8)
#elif GD32_TLI_L1_BPP == 24
#define GD32_TLI_L1_STRIDE ((GD32_TLI_WIDTH * 24 + 7) / 8)
#elif GD32_TLI_L1_BPP == 32
#define GD32_TLI_L1_STRIDE ((GD32_TLI_WIDTH * 32 + 7) / 8)
#else
#error Undefined or unrecognized base resolution
#endif

/* TLI only supports 8 bit per pixel overall */

#define GD32_TLI_LX_BYPP(n) ((n) / 8)

#define GD32_TLI_L1_FBSIZE (GD32_TLI_L1_STRIDE * GD32_TLI_HEIGHT)

#ifdef CONFIG_GD32F4_TLI_L2
#ifndef CONFIG_GD32_TLI_L2_WIDTH
#define CONFIG_GD32_TLI_L2_WIDTH GD32_TLI_WIDTH
#endif

#if CONFIG_GD32_TLI_L2_WIDTH > GD32_TLI_WIDTH
#error Width of Layer 2 exceeds the width of the display
#endif

#ifndef CONFIG_GD32_TLI_L2_HEIGHT
#define CONFIG_GD32_TLI_L2_HEIGHT GD32_TLI_HEIGHT
#endif

#if CONFIG_GD32_TLI_L2_HEIGHT > GD32_TLI_HEIGHT
#error Height of Layer 2 exceeds the height of the display
#endif

#if GD32_TLI_L2_BPP == 8
#define GD32_TLI_L2_STRIDE (CONFIG_GD32_TLI_L2_WIDTH)
#elif GD32_TLI_L2_BPP == 16
#define GD32_TLI_L2_STRIDE ((CONFIG_GD32_TLI_L2_WIDTH * 16 + 7) / 8)
#elif GD32_TLI_L2_BPP == 24
#define GD32_TLI_L2_STRIDE ((CONFIG_GD32_TLI_L2_WIDTH * 24 + 7) / 8)
#elif GD32_TLI_L2_BPP == 32
#define GD32_TLI_L2_STRIDE ((CONFIG_GD32_TLI_L2_WIDTH * 32 + 7) / 8)
#else
#error Undefined or unrecognized base resolution
#endif

#define GD32_TLI_L2_FBSIZE (GD32_TLI_L2_STRIDE * \
                            CONFIG_GD32_TLI_L2_HEIGHT)

#else
#define GD32_TLI_L2_FBSIZE (0)
#endif

/* Total memory used for framebuffers */

#define GD32_TLI_TOTAL_FBSIZE (GD32_TLI_L1_FBSIZE + \
                               GD32_TLI_L2_FBSIZE)

/* Debug option */

#ifdef CONFIG_GD32_TLI_REGDEBUG
#define regerr lcderr
#define reginfo lcdinfo
#else
#define regerr(x...)
#define reginfo(x...)
#endif

/* Preallocated TLI framebuffers */

/* Position the framebuffer memory in the center of the memory set aside.  We
 * will use any skirts before or after the framebuffer memory as a guard
 * against wild framebuffer writes.
 */

#define GD32_TLI_BUFFER_SIZE CONFIG_GD32F4_TLI_FB_SIZE
#define GD32_TLI_BUFFER_FREE (GD32_TLI_BUFFER_SIZE - \
                              GD32_TLI_TOTAL_FBSIZE)
#define GD32_TLI_BUFFER_START (CONFIG_GD32F4_TLI_FB_BASE + \
                               GD32_TLI_BUFFER_FREE / 2)

#if GD32_TLI_BUFFER_FREE < 0
#error "GD32_TLI_BUFFER_SIZE not large enough for frame buffers"
#endif

/* Layer frame buffer */

#define GD32_TLI_BUFFER_L1 GD32_TLI_BUFFER_START
#define GD32_TLI_ENDBUF_L1 (GD32_TLI_BUFFER_L1 + \
                            GD32_TLI_L1_FBSIZE)

#ifdef CONFIG_GD32F4_TLI_L2
#define GD32_TLI_BUFFER_L2 GD32_TLI_ENDBUF_L1
#define GD32_TLI_ENDBUF_L2 (GD32_TLI_BUFFER_L2 + \
                            GD32_TLI_L2_FBSIZE)
#else
#define GD32_TLI_ENDBUF_L2 GD32_TLI_ENDBUF_L1
#endif

/* TLI layer */

#ifdef CONFIG_GD32F4_TLI_L2
#define TLI_NLAYERS 2
#else
#define TLI_NLAYERS 1
#endif

/* IPA layer */

#ifdef CONFIG_GD32F4_IPA
#define IPA_NLAYERS CONFIG_GD32F4_IPA_NLAYERS
#if IPA_NLAYERS < 1
#error "IPA must at least support 1 overlay"
#endif

#define GD32_IPA_WIDTH CONFIG_GD32F4_IPA_LAYER_PPLINE

#if defined(CONFIG_GD32F4_IPA_L08)
#define GD32_IPA_STRIDE (GD32_IPA_WIDTH)
#define GD32_IPA_BPP 8
#define GD32_IPA_COLOR_FMT IPA_PF_L8
#elif defined(CONFIG_GD32F4_IPA_RGB565)
#define GD32_IPA_STRIDE ((GD32_IPA_WIDTH * 16 + 7) / 8)
#define GD32_IPA_BPP 16
#define GD32_IPA_COLOR_FMT IPA_PF_RGB565
#elif defined(CONFIG_GD32F4_IPA_RGB888)
#define GD32_IPA_STRIDE ((GD32_IPA_WIDTH * 24 + 7) / 8)
#define GD32_IPA_BPP 24
#define GD32_IPA_COLOR_FMT IPA_PF_RGB888
#elif defined(CONFIG_GD32F4_IPA_ARGB8888)
#define GD32_IPA_STRIDE ((GD32_IPA_WIDTH * 32 + 7) / 8)
#define GD32_IPA_BPP 32
#define GD32_IPA_COLOR_FMT IPA_PF_ARGB8888
#else
#error "IPA pixel format not supported"
#endif

#ifdef CONFIG_GD32F4_IPA_LAYER_SHARED
#define GD32_IPA_FBSIZE CONFIG_GD32F4_IPA_FB_SIZE
#define GD32_IPA_LAYER_SIZE 0
#else
#define GD32_IPA_FBSIZE CONFIG_GD32F4_IPA_FB_SIZE / IPA_NLAYERS
#define GD32_IPA_LAYER_SIZE GD32_IPA_FBSIZE
#if GD32_IPA_FBSIZE * IPA_NLAYERS > CONFIG_GD32F4_IPA_FB_SIZE
#error "IPA framebuffer size to small for configured number of overlays"
#endif
#endif /* CONFIG_GD32F4_IPA_LAYER_SHARED */

#define GD32_IPA_HEIGHT GD32_IPA_FBSIZE / GD32_IPA_STRIDE

#define GD32_IPA_BUFFER_START CONFIG_GD32F4_IPA_FB_BASE
#else
#define IPA_NLAYERS 0
#endif /* CONFIG_GD32F4_IPA */

#define TLI_NOVERLAYS TLI_NLAYERS + IPA_NLAYERS

/* Dithering */

#ifndef CONFIG_GD32F4_TLI_DITHER_RED
#define GD32_TLI_DITHER_RED 0
#else
#define GD32_TLI_DITHER_RED CONFIG_GD32F4_TLI_DITHER_RED
#endif
#ifndef CONFIG_GD32F4_TLI_DITHER_GREEN
#define GD32_TLI_DITHER_GREEN 0
#else
#define GD32_TLI_DITHER_GREEN CONFIG_GD32F4_TLI_DITHER_GREEN
#endif
#ifndef CONFIG_GD32F4_TLI_DITHER_BLUE
#define GD32_TLI_DITHER_BLUE 0
#else
#define GD32_TLI_DITHER_BLUE CONFIG_GD32F4_TLI_DITHER_BLUE
#endif

/* Background color */

#ifndef CONFIG_GD32F4_TLI_BACKCOLOR
#define GD32_TLI_BACKCOLOR 0
#else
#define GD32_TLI_BACKCOLOR CONFIG_GD32F4_TLI_BACKCOLOR
#endif

/* Layer default color */

#ifdef CONFIG_GD32F4_TLI_L1_COLOR
#define GD32_TLI_L1_COLOR CONFIG_GD32F4_TLI_L1_COLOR
#else
#define GD32_TLI_L1_COLOR 0x000000
#endif

#ifdef CONFIG_GD32F4_TLI_L2
#ifdef CONFIG_GD32F4_TLI_L2_COLOR
#define GD32_TLI_L2_COLOR CONFIG_GD32F4_TLI_L2_COLOR
#else
#define GD32_TLI_L2_COLOR 0x000000
#endif
#endif

/* Internal operation flags */

#define TLI_LAYER_SETAREA (1 << 0)        /* Change visible area */
#define TLI_LAYER_SETALPHAVALUE (1 << 1)  /* Change constant alpha value */
#define TLI_LAYER_SETBLENDMODE (1 << 2)   /* Change blendmode */
#define TLI_LAYER_SETCOLORKEY (1 << 3)    /* Change color key */
#define TLI_LAYER_ENABLECOLORKEY (1 << 4) /* Enable colorkey */
#define TLI_LAYER_SETCOLOR (1 << 5)       /* Change default color */
#define TLI_LAYER_SETENABLE (1 << 6)      /* Change enabled state */
#define TLI_LAYER_ENABLE (1 << 7)         /* Enable the layer */

/* Layer initializing state */

#define TLI_LAYER_INIT TLI_LAYER_SETAREA |           \
                           TLI_LAYER_SETALPHAVALUE | \
                           TLI_LAYER_SETBLENDMODE |  \
                           TLI_LAYER_SETCOLORKEY |   \
                           TLI_LAYER_SETCOLOR |      \
                           TLI_LAYER_SETENABLE |     \
                           TLI_LAYER_ENABLE

/* Blendfactor reset values for flip operation */

#define GD32_TLI_BF1_RESET 6
#define GD32_TLI_BF2_RESET 7

/* Check pixel format support by IPA driver */

#ifdef CONFIG_GD32F4_IPA
#if defined(CONFIG_GD32F4_TLI_L1_L8) || \
    defined(CONFIG_GD32F4_TLI_L2_L8)
#if !defined(CONFIG_GD32F4_IPA_L8)
#error "IPA must support FB_FMT_RGB8 pixel format"
#endif
#endif
#if defined(CONFIG_GD32F4_TLI_L1_RGB565) || \
    defined(CONFIG_GD32F4_TLI_L2_RGB565)
#if !defined(CONFIG_GD32F4_IPA_RGB565)
#error "IPA must support FB_FMT_RGB16_565 pixel format"
#endif
#endif
#if defined(CONFIG_GD32F4_TLI_L1_RGB888) || \
    defined(CONFIG_GD32F4_TLI_L2_RGB888)
#if !defined(CONFIG_GD32F4_IPA_RGB888)
#error "IPA must support FB_FMT_RGB24 pixel format"
#endif
#endif
#if defined(CONFIG_GD32F4_TLI_L1_ARGB8888) || \
    defined(CONFIG_GD32F4_TLI_L2_ARGB8888)
#if !defined(CONFIG_GD32F4_IPA_ARGB8888)
#error "IPA must support FB_FMT_RGB32 pixel format"
#endif
#endif
#endif

/* Calculate the size of the layers clut table */

#ifdef CONFIG_GD32F4_FB_CMAP
#if defined(CONFIG_GD32F4_IPA) && !defined(CONFIG_GD32F4_IPA_L8)
#error "IPA must also support L8 CLUT pixel format if supported by TLI"
#endif
#ifdef GD32_TLI_L1CMAP
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
#define GD32_LAYER_CLUT_SIZE GD32_TLI_NCLUT * sizeof(uint32_t)
#else
#define GD32_LAYER_CLUT_SIZE GD32_TLI_NCLUT * 3 * sizeof(uint8_t)
#endif
#endif
#ifdef GD32_LTLI_L2CMAP
#undef GD32_LAYER_CLUT_SIZE
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
#define GD32_LAYER_CLUT_SIZE GD32_TLI_NCLUT * sizeof(uint32_t) * 2
#else
#define GD32_LAYER_CLUT_SIZE GD32_TLI_NCLUT * 3 * sizeof(uint8_t) * 2
#endif
#endif
#endif

#ifndef CONFIG_GD32F4_FB_CMAP
#if defined(GD32_TLI_L1CMAP) || defined(GD32_LTLI_L2CMAP)
#undef GD32_TLI_L1CMAP
#undef GD32_LTLI_L2CMAP
#error "Enable cmap to support the configured layer format!"
#endif
#endif

/* Layer clut rgb value positioning */

#define TLI_L1CLUT_REDOFFSET 0
#define TLI_L1CLUT_GREENOFFSET 256
#define TLI_L1CLUT_BLUEOFFSET 512
#define TLI_L2CLUT_REDOFFSET 768
#define TLI_L2CLUT_GREENOFFSET 1024
#define TLI_L2CLUT_BLUEOFFSET 1280

/* Layer argb clut register position */

#define TLI_CLUT_ADD(n) ((uint32_t)(n) << 24)
#define TLI_CLUT_ALPHA(n) TLI_CLUT_ADD(n)
#define TLI_CLUT_RED(n) ((uint32_t)(n) << 16)
#define TLI_CLUT_GREEN(n) ((uint32_t)(n) << 8)
#define TLI_CLUT_BLUE(n) ((uint32_t)(n) << 0)
#define TLI_CLUT_RGB888_MASK 0xffffff

/* Layer argb cmap conversion */

#define TLI_CMAP_ALPHA(n) ((uint32_t)(n) >> 24)
#define TLI_CMAP_RED(n) ((uint32_t)(n) >> 16)
#define TLI_CMAP_GREEN(n) ((uint32_t)(n) >> 8)
#define TLI_CMAP_BLUE(n) ((uint32_t)(n) >> 0)

/* Hardware acceleration support */

/* Acceleration support for TLI overlays */

#ifdef CONFIG_GD32F4_TLI_L1_CHROMAKEYEN
#define GD32_TLI_L1_CHROMAEN true
#define GD32_TLI_L1_CHROMAKEY CONFIG_GD32F4_TLI_L1_CHROMAKEY
#define TLI_TLI_ACCL_L1 FB_ACCL_TRANSP | FB_ACCL_CHROMA
#else
#define GD32_TLI_L1_CHROMAEN false
#define GD32_TLI_L1_CHROMAKEY 0
#define TLI_TLI_ACCL_L1 FB_ACCL_TRANSP
#endif

#ifdef CONFIG_GD32F4_TLI_L2_CHROMAKEYEN
#define GD32_TLI_L2_CHROMAEN true
#define GD32_TLI_L2_CHROMAKEY CONFIG_GD32F4_TLI_L2_CHROMAKEY
#define TLI_TLI_ACCL_L2 FB_ACCL_TRANSP | FB_ACCL_CHROMA
#else
#define GD32_TLI_L2_CHROMAEN false
#define GD32_TLI_L2_CHROMAKEY 0
#define TLI_TLI_ACCL_L2 FB_ACCL_TRANSP
#endif

#ifdef CONFIG_GD32F4_IPA
#ifdef CONFIG_FB_OVERLAY_BLIT
#ifdef CONFIG_GD32F4_FB_CMAP
#define TLI_BLIT_ACCL FB_ACCL_BLIT
#else
#define TLI_BLIT_ACCL FB_ACCL_BLIT | FB_ACCL_BLEND
#endif /* CONFIG_GD32F4_FB_CMAP */
#else
#define TLI_BLIT_ACCL 0
#endif /* CONFIG_FB_OVERLAY_BLIT */

#ifdef CONFIG_GD32F4_FB_CMAP
#define TLI_IPA_ACCL TLI_BLIT_ACCL
#else
#define TLI_IPA_ACCL FB_ACCL_COLOR | TLI_BLIT_ACCL
#endif /* CONFIG_GD32F4_FB_CMAP */
#else
#define TLI_IPA_ACCL 0
#endif /* CONFIG_GD32F4_IPA */

#define TLI_L1_ACCL TLI_TLI_ACCL_L1 | TLI_IPA_ACCL
#ifdef CONFIG_GD32F4_TLI_L2
#define TLI_L2_ACCL TLI_TLI_ACCL_L2 | TLI_IPA_ACCL
#endif

/* Acceleration support for IPA overlays */

#ifdef CONFIG_GD32F4_FB_CMAP
#ifdef CONFIG_FB_OVERLAY_BLIT
#define IPA_ACCL FB_ACCL_BLIT | FB_ACCL_AREA
#else
#define IPA_ACCL FB_ACCL_AREA
#endif
#else
#ifdef CONFIG_FB_OVERLAY_BLIT
#define IPA_ACCL FB_ACCL_AREA |       \
                     FB_ACCL_TRANSP | \
                     FB_ACCL_COLOR |  \
                     FB_ACCL_BLIT |   \
                     FB_ACCL_BLEND
#else
#define IPA_ACCL FB_ACCL_AREA |       \
                     FB_ACCL_TRANSP | \
                     FB_ACCL_COLOR
#endif
#endif

/* Color normalization */

#if defined(CONFIG_GD32F4_TLI_L1_RGB565)
#define RGB888_R(x) (((((x) >> 11) & 0x1f) * 527 + 23) >> 6)
#define RGB888_G(x) (((((x) >> 5) & 0x3f) * 259 + 33) >> 6)
#define RGB888_B(x) ((((x)&0x1f) * 527 + 23) >> 6)
#define ARGB8888(x) ((RGB888_R(x) << 16) | \
                     (RGB888_G(x) << 8) |  \
                     RGB888_B(x))
#else
#define ARGB8888(x) (x)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration names each layer supported by the hardware */

enum gd32_layer_e
{
  TLI_LAYER_L1 = 0, /* LCD Layer 1 */
  TLI_LAYER_L2,     /* LCD Layer 2 */
};

/* TLI General layer information */

struct gd32_tli_s
{
  int layerno; /* layer number */

#ifdef CONFIG_FB_OVERLAY
  struct fb_overlayinfo_s oinfo; /* Overlay info */
#endif

#ifdef CONFIG_GD32F4_IPA
  struct gd32_ipa_overlay_s ipainfo; /* Overlay info for IPA */
#endif

  mutex_t *lock; /* Layer exclusive access */
};

/* This structure provides the overall state of the TLI layer */

struct gd32_tlidev_s
{
  /* Framebuffer interface */

  struct fb_vtable_s vtable;

  /* Framebuffer video information */

  struct fb_videoinfo_s vinfo;

  /* Framebuffer plane information */

  struct fb_planeinfo_s pinfo;

  /* Cmap information */

#ifdef CONFIG_GD32F4_FB_CMAP
  struct fb_cmap_s cmap;
#endif

  /* Layer information */

  struct gd32_tli_s layer[TLI_NOVERLAYS];

#ifdef CONFIG_GD32F4_IPA
  /* Interface to the ipa controller */

  struct ipa_layer_s *ipa;
#endif
};

/* Interrupt handling */

struct gd32_interrupt_s
{
  int irq;    /* irq number */
  int error;  /* Interrupt error */
  sem_t *sem; /* Semaphore for waiting for irq */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Overall TLI helper */

static void gd32_tli_enable(bool enable);
static void gd32_tli_gpioconfig(void);
static void gd32_tli_periphconfig(void);
static void gd32_tli_bgcolor(uint32_t rgb);
static void gd32_tli_dither(bool enable, uint8_t red,
                            uint8_t green, uint8_t blue);
static int gd32_tliirq(int irq, void *context, void *arg);
static int gd32_tli_waitforirq(void);
static int gd32_tli_reload(uint8_t value, bool waitvblank);

/* Helper for layer register configuration */

static void gd32_tli_lpixelformat(struct gd32_tli_s *layer);
static void gd32_tli_lframebuffer(struct gd32_tli_s *layer);
static void gd32_tli_lenable(struct gd32_tli_s *layer, bool enable);
static void gd32_tli_ldefaultcolor(struct gd32_tli_s *layer,
                                   uint32_t rgb);
static void gd32_tli_ltransp(struct gd32_tli_s *layer,
                             uint8_t transp,
                             uint32_t mode);
static void gd32_tli_lchromakey(struct gd32_tli_s *layer,
                                uint32_t chromakey);
static void gd32_tli_lchromakeyenable(struct gd32_tli_s *layer,
                                      bool enable);
static void gd32_tli_linit(uint8_t lid);

#ifdef CONFIG_GD32F4_IPA
static void gd32_tli_ipalinit(void);

#ifdef CONFIG_FB_OVERLAY_BLIT
static bool gd32_tli_lvalidate(const struct gd32_tli_s *layer,
                               const struct fb_area_s *area);
#endif
#endif

#ifdef CONFIG_GD32F4_FB_CMAP
static void gd32_tli_lputclut(struct gd32_tli_s *layer,
                              const struct fb_cmap_s *cmap);
static void gd32_tli_lgetclut(struct gd32_tli_s *layer,
                              struct fb_cmap_s *cmap);
static void gd32_tli_lclutenable(struct gd32_tli_s *layer,
                                 bool enable);
#endif

static void gd32_tli_lclear(uint8_t overlayno);

/* Framebuffer interface */

static int gd32_getvideoinfo(struct fb_vtable_s *vtable,
                             struct fb_videoinfo_s *vinfo);
static int gd32_getplaneinfo(struct fb_vtable_s *vtable,
                             int planeno,
                             struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_GD32F4_FB_CMAP
static int gd32_getcmap(struct fb_vtable_s *vtable,
                        struct fb_cmap_s *cmap);
static int gd32_putcmap(struct fb_vtable_s *vtable,
                        const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware signals vertical
 * synchronisation
 */

#ifdef CONFIG_FB_SYNC
static int gd32_waitforvsync(struct fb_vtable_s *vtable);
#endif

/* The following is provided only if the video hardware supports overlays */

#ifdef CONFIG_FB_OVERLAY
static int gd32_getoverlayinfo(struct fb_vtable_s *vtable,
                               int overlayno,
                               struct fb_overlayinfo_s *oinfo);
static int gd32_settransp(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo);
static int gd32_setchromakey(struct fb_vtable_s *vtable,
                             const struct fb_overlayinfo_s *oinfo);
static int gd32_setcolor(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo);
static int gd32_setblank(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo);
static int gd32_setarea(struct fb_vtable_s *vtable,
                        const struct fb_overlayinfo_s *oinfo);

/* The following is provided only if the video hardware supports blit and
 * blend operation
 */

#ifdef CONFIG_FB_OVERLAY_BLIT
static int gd32_blit(struct fb_vtable_s *vtable,
                     const struct fb_overlayblit_s *blit);
static int gd32_blend(struct fb_vtable_s *vtable,
                      const struct fb_overlayblend_s *blend);
#endif /* CONFIG_FB_OVERLAY_BLIT */
#endif /* CONFIG_FB_OVERLAY */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PIO pin configurations */

static const uint32_t g_tlipins[] =
{
  GPIO_TLI_R0, GPIO_TLI_R1, GPIO_TLI_R2, GPIO_TLI_R3,
  GPIO_TLI_R4, GPIO_TLI_R5, GPIO_TLI_R6, GPIO_TLI_R7,
  GPIO_TLI_G0, GPIO_TLI_G1, GPIO_TLI_G2, GPIO_TLI_G3,
  GPIO_TLI_G4, GPIO_TLI_G5, GPIO_TLI_G6, GPIO_TLI_G7,
  GPIO_TLI_B0, GPIO_TLI_B1, GPIO_TLI_B2, GPIO_TLI_B3,
  GPIO_TLI_B4, GPIO_TLI_B5, GPIO_TLI_B6, GPIO_TLI_B7,

  GPIO_TLI_VSYNC, GPIO_TLI_HSYNC,
  GPIO_TLI_CLK, GPIO_TLI_DE
};

#define GD32_TLI_NPINCONFIGS (sizeof(g_tlipins) / sizeof(uint32_t))

#ifdef CONFIG_GD32F4_FB_CMAP
/* The layers clut table entries */

static uint8_t g_redclut[GD32_TLI_NCLUT];
static uint8_t g_greenclut[GD32_TLI_NCLUT];
static uint8_t g_blueclut[GD32_TLI_NCLUT];
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
static uint8_t g_transpclut[GD32_TLI_NCLUT];
#endif
#endif /* CONFIG_GD32F4_FB_CMAP */

/* The TLI mutex that enforces mutually exclusive access */

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/* The semaphore for interrupt handling */

static sem_t g_semirq = SEM_INITIALIZER(0);

/* This structure provides irq handling */

static struct gd32_interrupt_s g_interrupt =
{
  .irq = GD32_IRQ_TLI,
  .error = OK,
  .sem = &g_semirq
};

/* This structure provides the internal interface */

static struct gd32_tlidev_s g_vtable =
{
  .vtable =
  {
    .getvideoinfo = gd32_getvideoinfo,
    .getplaneinfo = gd32_getplaneinfo
#ifdef CONFIG_FB_SYNC
    ,
    .waitforvsync = gd32_waitforvsync
#endif

#ifdef CONFIG_GD32F4_FB_CMAP
    ,
    .getcmap = gd32_getcmap,
    .putcmap = gd32_putcmap
#endif

#ifdef CONFIG_FB_OVERLAY
    ,
    .getoverlayinfo = gd32_getoverlayinfo,
    .settransp = gd32_settransp,
    .setchromakey = gd32_setchromakey,
    .setcolor = gd32_setcolor,
    .setblank = gd32_setblank,
    .setarea = gd32_setarea
#ifdef CONFIG_FB_OVERLAY_BLIT
    ,
    .blit = gd32_blit,
    .blend = gd32_blend
#endif
#endif /* CONFIG_FB_OVERLAY */
  },
#ifdef CONFIG_GD32F4_TLI_L2
  .pinfo =
  {
    .fbmem = (uint8_t *)GD32_TLI_BUFFER_L2,
    .fblen = GD32_TLI_L2_FBSIZE,
    .stride = GD32_TLI_L2_STRIDE,
    .display = 0,
    .bpp = GD32_TLI_L2_BPP
  },
  .vinfo =
  {
    .fmt = GD32_TLI_L2_COLOR_FMT,
    .xres = GD32_TLI_WIDTH,
    .yres = GD32_TLI_HEIGHT,
    .nplanes = 1,
#ifdef CONFIG_FB_OVERLAY
    .noverlays = TLI_NOVERLAYS
#endif
  }
#else
  .pinfo =
  {
    .fbmem = (uint8_t *)GD32_TLI_BUFFER_L1,
    .fblen = GD32_TLI_L1_FBSIZE,
    .stride = GD32_TLI_L1_STRIDE,
    .display = 0,
    .bpp = GD32_TLI_L1_BPP
  },
  .vinfo =
  {
    .fmt = GD32_TLI_L1_COLOR_FMT,
    .xres = GD32_TLI_WIDTH,
    .yres = GD32_TLI_HEIGHT,
    .nplanes = 1,
#ifdef CONFIG_FB_OVERLAY
    .noverlays = TLI_NOVERLAYS
#endif
  }
#endif /* CONFIG_GD32F4_TLI_L2 */
  ,
#ifdef CONFIG_GD32F4_FB_CMAP
  .cmap =
  {
    .first = 0,
    .len = GD32_TLI_NCLUT,
    .red = g_redclut,
    .green = g_greenclut,
    .blue = g_blueclut,
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
    .transp = g_transpclut
#endif
  },
#endif
  .layer[TLI_LAYER_L1] =
  {
    .layerno = TLI_LAYER_L1,
#ifdef CONFIG_FB_OVERLAY
    .oinfo =
    {
      .fbmem = (uint8_t *)GD32_TLI_BUFFER_L1,
      .fblen = GD32_TLI_L1_FBSIZE,
      .stride = GD32_TLI_L1_STRIDE,
      .overlay = TLI_LAYER_L1,
      .bpp = GD32_TLI_L1_BPP,
      .blank = 0,
      .chromakey = 0,
      .color = 0,
      .transp =
      {
        .transp = 0xff,
        .transp_mode = FB_PIXEL_ALPHA
      },
      .sarea =
      {
        .x = 0,
        .y = 0,
        .w = GD32_TLI_WIDTH,
        .h = GD32_TLI_HEIGHT
      },
      .accl = TLI_L1_ACCL
    },
#endif

#ifdef CONFIG_GD32F4_IPA
    .ipainfo =
    {
      .fmt = GD32_TLI_L1_IPA_PF,
      .transp_mode = GD32_IPA_PFCCR_AM_NONE,
      .xres = GD32_TLI_WIDTH,
      .yres = GD32_TLI_HEIGHT,
      .oinfo = &g_vtable.layer[TLI_LAYER_L1].oinfo
    },
#endif
    .lock = &g_lock
  }
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  .layer[TLI_LAYER_L2] =
  {
    .layerno = TLI_LAYER_L2,
#ifdef CONFIG_FB_OVERLAY
    .oinfo =
    {
      .overlay = TLI_LAYER_L2,
      .fbmem = (uint8_t *)GD32_TLI_BUFFER_L2,
      .fblen = GD32_TLI_L2_FBSIZE,
      .stride = GD32_TLI_L2_STRIDE,
      .bpp = GD32_TLI_L2_BPP,
      .blank = 0,
      .chromakey = 0,
      .color = 0,
      .transp =
      {
        .transp = 0xff,
        .transp_mode = FB_CONST_ALPHA
      },
      .sarea =
      {
        .x = 0,
        .y = 0,
        .w = GD32_TLI_WIDTH,
        .h = GD32_TLI_HEIGHT
      },
      .accl = TLI_L2_ACCL
    },
#endif

#ifdef CONFIG_GD32F4_IPA
    .ipainfo =
    {
      .fmt = GD32_TLI_L2_IPA_PF,
      .transp_mode = GD32_IPA_PFCCR_AM_NONE,
      .xres = GD32_TLI_WIDTH,
      .yres = GD32_TLI_HEIGHT,
      .oinfo = &g_vtable.layer[TLI_LAYER_L2].oinfo
    },
#endif
    .lock = &g_lock
  }
#endif
};

/* Configuration lookup tables */

/* TLI width */

static const uint32_t gd32_width_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_WIDTH
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_WIDTH
#endif
};

/* TLI height */

static const uint32_t gd32_height_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_HEIGHT
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_HEIGHT
#endif
};

/* TLI stride */

static const uint32_t gd32_stride_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L1_STRIDE
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L2_STRIDE
#endif
};

/* TLI bpp */

static const uint32_t gd32_bpp_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L1_BPP
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L2_BPP
#endif
};

/* TLI framebuffer len */

static const uint32_t gd32_fbsize_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L1_FBSIZE
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L2_FBSIZE
#endif
};

/* TLI framebuffer */

static const uint32_t gd32_buffer_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_BUFFER_L1
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_BUFFER_L2
#endif
};

/* TLI default color lookup table */

static const uint32_t gd32_defaultcolor_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L1_COLOR
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L2_COLOR
#endif
};

/* TLI default chromakey */

static const uint32_t gd32_chromakey_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L1_CHROMAKEY
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L2_CHROMAKEY
#endif
};

/* TLI chromakey enabled state */

static const bool gd32_chromaen_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L1_CHROMAEN
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L2_CHROMAEN
#endif
};

/* TLI pixel format lookup table */

static const uint32_t gd32_pf_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L1PPF_PF
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L2PPF_PF
#endif
};

/* Register lookup tables */

/* TLI_LxCR */

static const uintptr_t gd32_ctl_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0CTL
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1CTL
#endif
};

/* TLI_LxWHPCR */

static const uintptr_t gd32_hpos_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0HPOS
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1HPOS
#endif
};

/* TLI_LxWVPCR */

static const uintptr_t gd32_vpos_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0VPOS
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1VPOS
#endif
};

/* TLI_LxPFCR */

static const uintptr_t gd32_ppf_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0PPF
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1PPF
#endif
};

/* TLI_LxDCCR */

static const uintptr_t gd32_dc_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0DC
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1DC
#endif
};

/* TLI_LxCKCR */

static const uintptr_t gd32_ckey_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0CKEY
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1CKEY
#endif
};

/* TLI_LxCACR */

static const uintptr_t gd32_sa_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0SA
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1SA
#endif
};

/* TLI_LxBFCR */

static const uintptr_t gd32_blend_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0BLEND
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1BLEND
#endif
};

/* TLI_LxCFBAR */

static const uintptr_t gd32_fbaddr_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0FBADDR
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1FBADDR
#endif
};

/* TLI_LxCFBLR */

static const uintptr_t gd32_fllen_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0FLLEN
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1FLLEN
#endif
};

/* TLI_LxCFBLNR */

static const uintptr_t gd32_ftln_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0FTLN
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1FTLN
#endif
};

/* TLI_LxCLUTWR */

#ifdef CONFIG_GD32F4_FB_CMAP
static const uintptr_t gd32_lut_layer_t[TLI_NLAYERS] =
{
  GD32_TLI_L0LUT
#ifdef CONFIG_GD32F4_TLI_L2
  ,
  GD32_TLI_L1LUT
#endif
};
#endif /* CONFIG_GD32F4_FB_CMAP */

/* The initialized state of the driver */

static bool g_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_tli_gpioconfig
 *
 * Description:
 *   Configure GPIO pins for use with the TLI
 *
 ****************************************************************************/

static void gd32_tli_gpioconfig(void)
{
  int i;

  lcdinfo("Configuring pins\n");

  /* Configure each pin */

  for (i = 0; i < GD32_TLI_NPINCONFIGS; i++)
    {
      reginfo("set gpio%d = %08x\n", i, g_tlipins[i]);
      gd32_gpio_config(g_tlipins[i]);
    }

  gd32_gpio_config(GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE |
                   GPIO_CFG_PP | GPIO_CFG_SPEED_50MHZ |
                   GPIO_CFG_PORT_B | GPIO_CFG_PIN_15);
  gd32_gpio_write(GPIO_CFG_PORT_B | GPIO_PIN15_OUTPUT, 1);
}

/****************************************************************************
 * Name: gd32_tli_periphconfig
 *
 * Description:
 *   Configures the synchronous timings
 *   Configures the synchronous signals and clock polarity
 *
 ****************************************************************************/

static void gd32_tli_periphconfig(void)
{
  uint32_t regval;

  /* Configure GPIO's */

  gd32_tli_gpioconfig();

  /* Configure APB2 TLI clock external */

  /* Configure the SAI PLL external to provide the LCD_CLK */

  /* reginfo("configured RCC_PLLSAI=%08x\\n",
   *         getreg32(GD32_RCC_PLLSAICFGR));
   */

  /* Configure dedicated clock external */

  /* Configure RCU_TLI */

  regval  = getreg32(GD32_RCU_APB2EN);

  regval |= RCU_APB2EN_TLIEN;
  reginfo("set RCU_TLI=%08x\n", regval);
  putreg32(regval, GD32_RCU_APB2EN);
  reginfo("configured RCU_TLI=%08x\n", getreg32(GD32_RCU_APB2EN));

  /* Configure the main PLL, and set PSC, PLL_N, PLL_P, PLL_Q */

  regval = (GD32_RCU_PLLSAI_PLLSAIN |
            GD32_RCU_PLLSAI_PLLSAIP |
            GD32_RCU_PLLSAI_PLLSAIR);
  putreg32(regval, GD32_RCU_PLLSAI);

  /* Configure the main PLLSAI DIV 8 */

  regval = RCU_CFG1_PLLSAIRDIV_DIV_6;
  putreg32(regval, GD32_RCU_CFG1);

  /* Enable the main PLL */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLLSAIEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until the PLL is ready */

  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_PLLSAISTB) == 0)
    {
    }

  /* Configure TLI_SSCR */

  regval = (GD32_TLI_SPSZ_VPSZ | GD32_TLI_SPSZ_HPSZ);
  reginfo("set TLI_SPSZ=%08x\n", regval);
  putreg32(regval, GD32_TLI_SPSZ);
  reginfo("configured TLI_SPSZ=%08x\n", getreg32(GD32_TLI_SPSZ));

  /* Configure TLI_BPCR */

  regval = (GD32_TLI_BPSZ_VBPSZ | GD32_TLI_BPSZ_HBPSZ);
  reginfo("set TLI_BPSZ=%08x\n", regval);
  putreg32(regval, GD32_TLI_BPSZ);
  reginfo("configured TLI_BPSZ=%08x\n", getreg32(GD32_TLI_BPSZ));

  /* Configure TLI_AWCR */

  regval = (GD32_TLI_ASZ_VASZ | GD32_TLI_ASZ_HASZ);
  reginfo("set TLI_ASZ=%08x\n", regval);
  putreg32(regval, GD32_TLI_ASZ);
  reginfo("configured TLI_ASZ=%08x\n", getreg32(GD32_TLI_ASZ));

  /* Configure TLI_TWCR */

  regval = (GD32_TLI_TSZ_VTSZ | GD32_TLI_TSZ_HTSZ);
  reginfo("set TLI_TSZ=%08x\n", regval);
  putreg32(regval, GD32_TLI_TSZ);
  reginfo("configured TLI_TSZ=%08x\n", getreg32(GD32_TLI_TSZ));

  /* Configure TLI_GCR */

  regval = (GD32_TLI_GCR_BCB |
            (GD32_TLI_GCR_BCR << 8U) |
            (GD32_TLI_GCR_BCG << 16U));
  reginfo("set TLI_BGC=%08x\n", regval);
  putreg32(regval, GD32_TLI_BGC);
  reginfo("configured TLI_BGC=%08x\n", getreg32(GD32_TLI_BGC));

  /* Configure TLI_CTL */

  regval = getreg32(GD32_TLI_CTL);
  regval &= ~(TLI_CTL_CLKPS | TLI_CTL_DEPS | TLI_CTL_VPPS |
              TLI_CTL_HPPS);
  regval |= (GD32_TLI_CTL_CLKPS | GD32_TLI_CTL_DEPS |
             GD32_TLI_CTL_VPPS | GD32_TLI_CTL_HPPS);

  reginfo("set TLI_CTL=%08x\n", regval);
  putreg32(regval, GD32_TLI_CTL);
  reginfo("configured TLI_CTL=%08x\n", getreg32(GD32_TLI_CTL));
}

/****************************************************************************
 * Name: gd32_tli_ldefaultcolor
 *
 * Description:
 *   Configures layer default color.
 *
 * Input Parameters:
 *   layer - Reference to the layer control structure
 *   rgb - RGB888 background color
 *
 ****************************************************************************/

static void gd32_tli_ldefaultcolor(struct gd32_tli_s *layer,
                                   uint32_t rgb)
{
  DEBUGASSERT(layer->layerno < TLI_NLAYERS);
  reginfo("set TLI_L%dRL=%08x\n", layer->layerno + 1, rgb);

  putreg32(rgb, gd32_dc_layer_t[layer->layerno]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);

  reginfo("configured TLI_L%dRL=%08x\n", layer->layerno + 1,
          getreg32(GD32_TLI_BGC));
}

/****************************************************************************
 * Name: gd32_tli_bgcolor
 *
 * Description:
 *   Configures background color of the LCD controller.
 *
 * Input Parameters:
 *   rgb - RGB888 background color
 *
 ****************************************************************************/

static void gd32_tli_bgcolor(uint32_t rgb)
{
  reginfo("set TLI_BGC=%08x\n", rgb);
  putreg32(rgb, GD32_TLI_BGC);
  reginfo("configured TLI_BGC=%08x\n", getreg32(GD32_TLI_BGC));
}

/****************************************************************************
 * Name: gd32_tli_dither
 *
 * Description:
 *   Configures dither settings of the LCD controller.
 *
 * Input Parameters:
 *   enable - Enable dithering
 *   red    - Red dither width
 *   green  - Green dither width
 *   blue   - Blue dither width
 *
 ****************************************************************************/

static void gd32_tli_dither(bool enable, uint8_t red,
                            uint8_t green, uint8_t blue)
{
  uint32_t regval;

  regval = getreg32(GD32_TLI_CTL);

  if (enable == true)
    {
      regval |= TLI_CTL_DFEN;
    }
  else
    {
      regval &= ~TLI_CTL_DFEN;
    }

  regval &= ~(TLI_CTL_BDB_MASK | TLI_CTL_GDB_MASK | TLI_CTL_RDB_MASK);
  regval |= (TLI_CTL_RDB(red) | TLI_CTL_GDB(green) | TLI_CTL_BDB(blue));

  reginfo("set TLI_CTL=%08x\n", regval);
  putreg32(regval, GD32_TLI_CTL);
  reginfo("configured TLI_CTL=%08x\n", getreg32(GD32_TLI_CTL));
}

/****************************************************************************
 * Name: gd32_tli_linepos
 *
 * Description:
 *   Configures line position register
 *
 ****************************************************************************/

static void gd32_tli_linepos(void)
{
  /* Configure TLI_LIPCR */

  reginfo("set TLI_LM_LM=%08x\n", GD32_TLI_LM_LM);
  putreg32(GD32_TLI_LM_LM, GD32_TLI_LM_LM);
  reginfo("configured TLI_LM_LM=%08x\n", getreg32(GD32_TLI_LM_LM));
}

/****************************************************************************
 * Name: gd32_tli_irqctrl
 *
 * Description:
 *   Control  interrupts generated by the tli controller
 *
 * Input Parameters:
 *   setirqs  - set interrupt mask
 *   clrirqs  - clear interrupt mask
 *
 ****************************************************************************/

static void gd32_tli_irqctrl(uint32_t setirqs, uint32_t clrirqs)
{
  uint32_t regval;

  regval = getreg32(GD32_TLI_INTEN);
  regval &= ~clrirqs;
  regval |= setirqs;
  reginfo("set TLI_INTEN=%08x\n", regval);
  putreg32(regval, GD32_TLI_INTEN);
  reginfo("configured TLI_INTEN=%08x\n", getreg32(GD32_TLI_INTEN));
}

/****************************************************************************
 * Name: gd32_tliirq
 *
 * Description:
 *   TLI interrupt handler
 *
 ****************************************************************************/

static int gd32_tliirq(int irq, void *context, void *arg)
{
  int ret;
  struct gd32_interrupt_s *priv = &g_interrupt;
  uint32_t regval = getreg32(GD32_TLI_INTF);

  reginfo("irq = %d, regval = %08x\n", irq, regval);

  if (regval & TLI_INTF_LCRF)
    {
      /* Register reload interrupt */

      /* Clear the interrupt status register */

      reginfo("Register reloaded\n");
      putreg32(TLI_INTC_LCRC, GD32_TLI_INTC);
      priv->error = OK;
    }
  else if (regval & TLI_INTEN_LMIE)
    {
      /* Line interrupt */

      /* Clear the interrupt status register */

      reginfo("Line interrupt\n");
      putreg32(TLI_INTC_LMC, GD32_TLI_INTC);
      priv->error = OK;
    }
  else if (regval & TLI_INTEN_TEIE)
    {
      /* Transfer error interrupt */

      /* Clear the interrupt status register */

      reginfo("Error transfer\n");
      putreg32(TLI_INTC_TEC, GD32_TLI_INTC);
      priv->error = -ECANCELED;
    }
  else if (regval & TLI_INTEN_FEIE)
    {
      /* Fifo underrun error interrupt */

      /* Clear the interrupt status register */

      reginfo("Error fifo underrun\n");
      putreg32(TLI_INTC_FEC, GD32_TLI_INTC);
      priv->error = -ECANCELED;
    }
  else
    {
      DEBUGASSERT("Unknown interrupt");
    }

  /* Unlock the semaphore if locked */

  ret = nxsem_post(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_post() failed\n");
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_tli_waitforirq
 *
 * Description:
 *   Helper waits until the tli irq occurs. In the current design That means
 *   that a register reload was been completed.
 *   Note! The caller must use this function within a critical section.
 *
 * Returned Value:
 *   OK - On success otherwise ERROR
 *
 ****************************************************************************/

static int gd32_tli_waitforirq(void)
{
  int ret = OK;
  struct gd32_interrupt_s *priv = &g_interrupt;

  ret = nxsem_wait(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_wait() failed\n");
    }

  ret = priv->error;

  return ret;
}

/****************************************************************************
 * Name: gd32_tli_reload
 *
 * Description:
 *   Reload the layer shadow register and make layer changes visible.
 *   Note! The caller must ensure that a previous register reloading has been
 *   completed.
 *
 * Input Parameters:
 *   value      - Reload flag (e.g. upon vertical blank or immediately)
 *   waitvblank - Wait until register reload is finished
 *
 ****************************************************************************/

static int gd32_tli_reload(uint8_t value, bool waitvblank)
{
  int ret = OK;

  /* Reloads the shadow register.
   * Note! This will not trigger an register reload interrupt if
   * immediately reload is set.
   */

  reginfo("set TLI_RL=%08x\n", value);
  putreg32(value, GD32_TLI_RL);
  reginfo("configured TLI_RL=%08x\n", getreg32(GD32_TLI_RL));

  if (value == TLI_RL_FBR && waitvblank)
    {
      /* Wait upon vertical blanking period */

      ret = gd32_tli_waitforirq();
    }
  else
    {
      /* Wait until register reload hase been done */

      while (getreg32(GD32_TLI_RL) & value)
        ;
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_tli_irqconfig
 *
 * Description:
 *   Configure interrupts
 *
 ****************************************************************************/

static void gd32_tli_irqconfig(void)
{
  /* Attach TLI interrupt vector */

  irq_attach(g_interrupt.irq, gd32_tliirq, NULL);

  /* Enable the IRQ at the NVIC */

  up_enable_irq(g_interrupt.irq);

  /* Enable interrupts expect line interrupt */

  gd32_tli_irqctrl(TLI_INTEN_LCRIE |
                       TLI_INTEN_TEIE |
                       TLI_INTEN_FEIE,
                   TLI_INTEN_LMIE);

  /* Configure line interrupt */

  gd32_tli_linepos();
}

/****************************************************************************
 * Name: gd32_tli_globalconfig
 *
 * Description:
 *   Configure background color
 *   Configure dithering
 *
 ****************************************************************************/

static void gd32_tli_globalconfig(void)
{
  /* Configure dither */

  gd32_tli_dither(
#ifdef CONFIG_GD32F4_TLI_DITHER
      true,
#else
      false,
#endif
      GD32_TLI_DITHER_RED,
      GD32_TLI_DITHER_GREEN,
      GD32_TLI_DITHER_BLUE);

  /* Configure background color */

  gd32_tli_bgcolor(GD32_TLI_BACKCOLOR);
}

/****************************************************************************
 * Name: gd32_tli_enable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 * Input Parameters:
 *   enable - Enable or disable
 *
 ****************************************************************************/

static void gd32_tli_enable(bool enable)
{
  uint32_t regval;

  regval = getreg32(GD32_TLI_CTL);
  reginfo("get TLI_CTL=%08x\n", regval);

  if (enable == true)
    {
      regval |= TLI_CTL_TLIEN;
    }
  else
    {
      regval &= ~TLI_CTL_TLIEN;
    }

  reginfo("set TLI_CTL=%08x\n", regval);
  putreg32(regval, GD32_TLI_CTL);
  reginfo("configured TLI_CTL=%08x\n", getreg32(GD32_TLI_CTL));
}

/****************************************************************************
 * Name: gd32_tli_lpixelformat
 *
 * Description:
 *   Set the layer pixel format.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   Reference to the layer control structure
 *
 ****************************************************************************/

static void gd32_tli_lpixelformat(struct gd32_tli_s *layer)
{
  uint8_t overlay = layer->layerno;
  DEBUGASSERT(layer->layerno < TLI_NLAYERS);

  /* Configure PFCR register */

  reginfo("set TLI_L%dPPF=%08x\n", overlay + 1,
          gd32_pf_layer_t[overlay]);
  putreg32(gd32_pf_layer_t[overlay], gd32_ppf_layer_t[overlay]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_lframebuffer
 *
 * Description:
 *   Configure layer framebuffer of the entire window.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   Reference to the layer control structure
 *
 ****************************************************************************/

static void gd32_tli_lframebuffer(struct gd32_tli_s *layer)
{
  uint32_t cfblr;
  uint32_t rxpos;
  uint32_t rypos;
  uint32_t whpcr;
  uint32_t wvpcr;
  uint8_t layerno = layer->layerno;

  DEBUGASSERT(layer->layerno < TLI_NLAYERS);
  reginfo("xpos = %d, ypos = %d, xres = %d, yres = %d\n", 0, 0,
          gd32_width_layer_t[layerno], gd32_height_layer_t[layerno]);

  /* Calculate register position */

  rxpos = GD32_TLI_LXHPOS_WLP + 1;
  rypos = GD32_TLI_LXVPOS_WTP + 1;

  /* Accumulate horizontal position */

  whpcr = TLI_LXHPOS_WLP(rxpos);
  whpcr |= TLI_LXHPOS_WRP(rxpos + gd32_width_layer_t[layerno] - 1);

  /* Accumulate vertical position */

  wvpcr = TLI_LXVPOS_WTP(rypos);
  wvpcr |= TLI_LXVPOS_WBP(rypos + gd32_height_layer_t[layerno] - 1);

  /* Configure LxWHPCR / LxWVPCR register */

  reginfo("set TLI_L%dHPOS=%08x\n", layerno + 1, whpcr);
  putreg32(whpcr, gd32_hpos_layer_t[layerno]);
  reginfo("set TLI_L%dVPOS=%08x\n", layerno + 1, wvpcr);
  putreg32(wvpcr, gd32_vpos_layer_t[layerno]);

  /* Configure LxCFBAR register */

  reginfo("set TLI_L%dCFBAR=%08x\n", layerno + 1,
          gd32_buffer_layer_t[layerno]);
  putreg32(gd32_buffer_layer_t[layerno], gd32_fbaddr_layer_t[layerno]);

  /* Configure LxCFBLR register */

  /* Calculate line length */

  cfblr = TLI_LXFLLEN_STDOFF(gd32_stride_layer_t[layerno]) |
          TLI_LXFLLEN_FLL(gd32_width_layer_t[layerno] *
                              GD32_TLI_LX_BYPP(gd32_bpp_layer_t[layerno]) +
                          3);

  reginfo("set TLI_L%dCFBLR=%08x\n", layerno + 1, cfblr);
  putreg32(cfblr, gd32_fllen_layer_t[layerno]);

  /* Configure LxCFBLNR register */

  reginfo("set TLI_L%dCFBLNR=%08x\n", layerno + 1,
          gd32_height_layer_t[layerno]);
  putreg32(gd32_height_layer_t[layerno], gd32_ftln_layer_t[layerno]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_lenable
 *
 * Description:
 *   Enable or disable layer.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable layer
 *
 ****************************************************************************/

static void gd32_tli_lenable(struct gd32_tli_s *layer, bool enable)
{
  uint32_t regval;
  DEBUGASSERT(layer->layerno < TLI_NLAYERS);

  regval = getreg32(gd32_ctl_layer_t[layer->layerno]);

  if (enable == true)
    {
      regval |= TLI_LXCTL_LEN;
    }
  else
    {
      regval &= ~TLI_LXCTL_LEN;
    }

  /* Enable/Disable layer */

  reginfo("set TLI_L%dCTL=%08x\n", layer->layerno + 1, regval);
  putreg32(regval, gd32_ctl_layer_t[layer->layerno]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_ltransp
 *
 * Description:
 *   Change layer transparency.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   transp - Transparency
 *   mode   - Transparency mode
 *
 ****************************************************************************/

static void gd32_tli_ltransp(struct gd32_tli_s *layer,
                             uint8_t transp,
                             uint32_t mode)
{
  uint32_t bf1;
  uint32_t bf2;

  DEBUGASSERT(layer->layerno < TLI_NLAYERS);

#ifdef CONFIG_FB_OVERLAY
  if (mode == FB_CONST_ALPHA)
    {
      bf1 = TLI_BF1_CONST_ALPHA;
      bf2 = TLI_BF2_CONST_ALPHA;
    }
  else
    {
      bf1 = TLI_BF1_PIXEL_ALPHA;
      bf2 = TLI_BF2_PIXEL_ALPHA;
    }
#else
  bf1 = TLI_BF1_CONST_ALPHA;
  bf2 = TLI_BF2_CONST_ALPHA;
#endif

  reginfo("set TLI_L%dBLEND=%08x\n", layer->layerno + 1,
          (TLI_LXBLEND_ACF1(bf1) | TLI_LXBLEND_ACF2(bf2)));

  /* Set blendmode */

  putreg32((TLI_LXBLEND_ACF1(bf1) | TLI_LXBLEND_ACF2(bf2)),
           gd32_blend_layer_t[layer->layerno]);

  /* Set alpha */

  reginfo("set TLI_L%dCACR=%02x\n", layer->layerno + 1, transp);
  putreg32(transp, gd32_sa_layer_t[layer->layerno]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_lchromakey
 *
 * Description:
 *   Change layer chromakey.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   chroma - chromakey
 *
 ****************************************************************************/

static void gd32_tli_lchromakey(struct gd32_tli_s *layer,
                                uint32_t chroma)
{
  uint32_t rgb;
  DEBUGASSERT(layer->layerno < TLI_NLAYERS);

  reginfo("%08x\n", getreg32(gd32_ctl_layer_t[layer->layerno]));

  /* Set chromakey */

#ifdef CONFIG_GD32F4_FB_CMAP
  uint8_t r = g_vtable.cmap.red[chroma];
  uint8_t g = g_vtable.cmap.green[chroma];
  uint8_t b = g_vtable.cmap.blue[chroma];
  rgb = ((r << 16) | (g << 8) | b);
#else
  rgb = ARGB8888(chroma);
#endif

  reginfo("set TLI_L%dCKCR=%08x\n", layer->layerno + 1, rgb);
  putreg32(rgb, gd32_ckey_layer_t[layer->layerno]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_lchromakeyenable
 *
 * Description:
 *   Enable or disable layer chromakey support.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable chromakey
 *
 ****************************************************************************/

static void gd32_tli_lchromakeyenable(struct gd32_tli_s *layer,
                                      bool enable)
{
  uint32_t regval;
  DEBUGASSERT(layer->layerno < TLI_NLAYERS);

  regval = getreg32(gd32_ctl_layer_t[layer->layerno]);

  /* Enable/Disable colorkey */

  if (enable == true)
    {
      regval |= TLI_LXCTL_CKEYEN;
    }
  else
    {
      regval &= ~TLI_LXCTL_CKEYEN;
    }

  reginfo("set TLI_L%dCTL=%08x\n", layer->layerno + 1, regval);
  putreg32(regval, gd32_ctl_layer_t[layer->layerno]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_lclutenable
 *
 * Description:
 *   Disable or enable the layer clut support
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_FB_CMAP
static void gd32_tli_lclutenable(struct gd32_tli_s *layer,
                                 bool enable)
{
  uint32_t regval;

  regval = getreg32(gd32_ctl_layer_t[layer->oinfo.overlay]);
  reginfo("get TLI_L%dCR=%08x\n", layer->oinfo.overlay + 1, regval);

  /* Disable the clut support during update the color table */

  if (enable == true)
    {
      regval |= TLI_LXCTL_LUTEN;
    }
  else
    {
      regval &= ~TLI_LXCTL_LUTEN;
    }

  reginfo("set TLIL%dCR=%08x\n", layer->oinfo.overlay, regval);
  putreg32(regval, gd32_ctl_layer_t[layer->oinfo.overlay]);

  /* Reload shadow register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_lputclut
 *
 * Description:
 *   Update the clut layer register during blank period.
 *   Note! The clut register is no shadow register.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   cmap   - Color map
 *
 ****************************************************************************/

static void gd32_tli_lputclut(struct gd32_tli_s *layer,
                              const struct fb_cmap_s *cmap)
{
  int n;
  irqstate_t flags;

  /* Disable clut during register update */

  gd32_tli_lclutenable(layer, false);

  /* Update the clut registers. Ensure operation is atomic or in interrupt
   * protected context.
   */

  flags = enter_critical_section();

  for (n = cmap->first; n < cmap->len && n < GD32_TLI_NCLUT; n++)
    {
      uint32_t regval;

      regval = (uint32_t)TLI_CLUT_ADD(n) |
               (uint32_t)TLI_CLUT_RED(cmap->red[n]) |
               (uint32_t)TLI_CLUT_GREEN(cmap->green[n]) |
               (uint32_t)TLI_CLUT_BLUE(cmap->blue[n]);

      reginfo("set TLI_L%dCLUTWR = %08x, first = %d, len = %d\n",
              layer->oinfo.overlay + 1, regval, cmap->first, cmap->len);
      putreg32(regval, gd32_lut_layer_t[layer->oinfo.overlay]);
    }

  leave_critical_section(flags);

  /* Enable clut after register update */

  gd32_tli_lclutenable(layer, true);

  /* Reload shadow control register */

  gd32_tli_reload(TLI_RL_RQR, false);
}

/****************************************************************************
 * Name: gd32_tli_lgetclut
 *
 * Description:
 *   Copy the layers color lookup table.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   cmap   - Color map
 *
 ****************************************************************************/

static void gd32_tli_lgetclut(struct gd32_tli_s *layer,
                              struct fb_cmap_s *cmap)
{
  int n;
  struct fb_cmap_s *priv_cmap = &g_vtable.cmap;

  /* Copy from internal cmap */

  for (n = cmap->first; n < cmap->len && n < GD32_TLI_NCLUT; n++)
    {
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
      cmap->transp[n] = priv_cmap->transp[n];
#endif
      cmap->red[n] = priv_cmap->red[n];
      cmap->green[n] = priv_cmap->green[n];
      cmap->blue[n] = priv_cmap->blue[n];

      reginfo("color = %d, transp=%02x, red=%02x, green=%02x, blue=%02x\n",
              n,
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
              cmap->transp[n],
#endif
              cmap->red[n],
              cmap->green[n],
              cmap->blue[n]);
    }
}
#endif /* CONFIG_GD32F4_FB_CMAP */

/****************************************************************************
 * Name: gd32_tli_lclear
 *
 * Description:
 *   Clear the whole layer
 *
 * Input Parameters:
 *   overlayno - Number overlay
 *
 ****************************************************************************/

static void gd32_tli_lclear(uint8_t overlayno)
{
  memset((uint8_t *)gd32_buffer_layer_t[overlayno], 0,
         gd32_fbsize_layer_t[overlayno]);
}

/****************************************************************************
 * Name: gd32_tli_lvalidate
 *
 * Description:
 *   Validates if the given area is within the overlay framebuffer memory
 *   region
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   area   - Reference to the overlay area
 *
 ****************************************************************************/

#if defined(CONFIG_GD32F4_IPA) && defined(CONFIG_FB_OVERLAY_BLIT)
static bool gd32_tli_lvalidate(const struct gd32_tli_s *layer,
                               const struct fb_area_s *area)
{
  uint32_t offset;

  offset = (area->y + area->h - 1) * layer->oinfo.stride +
           (area->x + area->w) * layer->oinfo.bpp / 8;

  return (offset <= layer->oinfo.fblen && area->w > 0 && area->h > 0);
}
#endif /* defined(CONFIG_GD32F4_IPA) && defined(CONFIG_FB_OVERLAY_BLIT) */

/****************************************************************************
 * Name: gd32_tli_linit
 *
 * Description:
 *   Initialize layer to their default states.
 *
 *   Initialize:
 *   - layer framebuffer
 *   - layer pixelformat
 *   - layer defaultcolor
 *   - layer chromakey
 *   - layer transparency
 *   - layer clut
 *
 * Input Parameters:
 *   layer          - Reference to the layer control structure
 *
 ****************************************************************************/

static void gd32_tli_linit(uint8_t overlay)
{
  DEBUGASSERT(overlay < TLI_NLAYERS);

  struct gd32_tlidev_s *dev = &g_vtable;
  struct gd32_tli_s *layer = &dev->layer[overlay];

  /* Disable layer */

  gd32_tli_lenable(layer, false);

  /* Clear the layer framebuffer */

  gd32_tli_lclear(overlay);

  /* Set layers framebuffer */

  gd32_tli_lframebuffer(layer);

  /* Set layers pixel input format */

  gd32_tli_lpixelformat(layer);

  /* Configure layer default color */

  gd32_tli_ldefaultcolor(layer, gd32_defaultcolor_layer_t[overlay]);

  /* Layers default transparency */

  gd32_tli_ltransp(layer, 0xff, 1);

  /* Layers chromakey */

  gd32_tli_lchromakey(layer, gd32_chromakey_layer_t[overlay]);

  /* Enable chromakey */

  gd32_tli_lchromakeyenable(layer, gd32_chromaen_layer_t[overlay]);

#ifdef CONFIG_GD32F4_FB_CMAP
  /* Disable clut by default */

  if (dev->vinfo.fmt == FB_FMT_RGB8)
    {
      /* Initialize TLI clut register */

      gd32_tli_lputclut(layer, &g_vtable.cmap);

      /* Configure the clut register */

      gd32_tli_lclutenable(layer, true);
    }
#endif

  /* Finally enable the layer */

  gd32_tli_lenable(layer, true);
}

/****************************************************************************
 * Name: gd32_tli_ipalinit
 *
 * Description:
 *   Initialize ipa layer to their default states.
 *
 *   Initialize:
 *   - layer framebuffer
 *   - layer pixelformat
 *   - layer size
 *   - layer color
 *   - layer chromakey
 *   - layer transparency
 *   - layer clut
 *
 * Input Parameters:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_IPA
static void gd32_tli_ipalinit(void)
{
  int n;
  struct gd32_tlidev_s *dev = &g_vtable;

  for (n = 0; n < TLI_NLAYERS; n++)
    {
      uint32_t overlay = n + TLI_NLAYERS;
      struct gd32_tli_s *layer = &dev->layer[overlay];
      uint8_t *fbmem = (uint8_t *)GD32_IPA_BUFFER_START;

      layer->layerno = overlay;
      layer->oinfo.fbmem = fbmem + GD32_IPA_LAYER_SIZE * n;
      layer->oinfo.fblen = GD32_IPA_FBSIZE;
      layer->oinfo.stride = GD32_IPA_STRIDE;
      layer->oinfo.overlay = overlay;
      layer->oinfo.bpp = GD32_IPA_BPP;
      layer->oinfo.blank = 0;
      layer->oinfo.chromakey = 0;
      layer->oinfo.color = 0;
      layer->oinfo.transp.transp = 0xff;
      layer->oinfo.transp.transp_mode = 0;
      layer->oinfo.sarea.x = 0;
      layer->oinfo.sarea.y = 0;
      layer->oinfo.sarea.w = GD32_IPA_WIDTH;
      layer->oinfo.sarea.h = GD32_IPA_HEIGHT;
      layer->oinfo.accl = IPA_ACCL;
      layer->lock = &g_lock;
      layer->ipainfo.fmt = GD32_IPA_COLOR_FMT;
      layer->ipainfo.transp_mode = GD32_IPA_PFCCR_AM_NONE;
      layer->ipainfo.xres = layer->oinfo.sarea.w;
      layer->ipainfo.yres = layer->oinfo.sarea.h;
      layer->ipainfo.oinfo = &layer->oinfo;
    }
}
#endif /* CONFIG_GD32F4_IPA */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_getvideoinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_VIDEOINFO
 *   Get the videoinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   vinfo  - the videoinfo object
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int gd32_getvideoinfo(struct fb_vtable_s *vtable,
                             struct fb_videoinfo_s *vinfo)
{
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  lcdinfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  DEBUGASSERT(vtable != NULL && priv == &g_vtable && vinfo != NULL);

  memcpy(vinfo, &priv->vinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name: gd32_getplaneinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_PLANEINFO
 *   Get the planeinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   pinfo  - the planeinfo object
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int gd32_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                             struct fb_planeinfo_s *pinfo)
{
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable);
  lcdinfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);

  if (planeno == 0)
    {
      memcpy(pinfo, &priv->pinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_getcmap
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_CMAP
 *   Get a range of CLUT values for the LCD
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   cmap   - the color table
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_FB_CMAP
static int gd32_getcmap(struct fb_vtable_s *vtable,
                        struct fb_cmap_s *cmap)
{
  int ret;
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && cmap != NULL);
  lcdinfo("vtable=%p cmap=%p\n", vtable, cmap);

  if (priv->vinfo.fmt != FB_FMT_RGB8)
    {
      lcderr("ERROR: CLUT is not supported for the pixel format: %d\n",
             priv->vinfo.fmt);
      ret = -EINVAL;
    }
  else if (cmap->first >= GD32_TLI_NCLUT)
    {
      lcderr("ERROR: only %d color table entries supported\n",
             GD32_TLI_NCLUT);
      ret = -EINVAL;
    }
  else
    {
      /* Currently, there is no api to set color map for each overlay
       * separately. TLI layers can have different color maps. Get the cmap
       * from the main overlay.
       */

      struct gd32_tli_s *layer;
#ifdef CONFIG_GD32F4_TLI_L2
      layer = &priv->layer[TLI_LAYER_L2];
#else
      layer = &priv->layer[TLI_LAYER_L1];
#endif
      nxmutex_lock(layer->lock);
      gd32_tli_lgetclut(layer, cmap);
      nxmutex_unlock(layer->lock);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_putcmap
 *
 * Description:
 *   Entrypoint ioctl FBIOPUT_CMAP
 *   Set a range of the CLUT values for the LCD
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   cmap   - the color table
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int gd32_putcmap(struct fb_vtable_s *vtable,
                        const struct fb_cmap_s *cmap)
{
  int ret;
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && cmap != NULL);
  lcdinfo("vtable=%p cmap=%p\n", vtable, cmap);

  if (priv->vinfo.fmt != FB_FMT_RGB8)
    {
      lcderr("ERROR: CLUT is not supported for the pixel format: %d\n",
             priv->vinfo.fmt);
      ret = -EINVAL;
    }
  else if (cmap->first >= GD32_TLI_NCLUT)
    {
      lcderr("ERROR: only %d color table entries supported\n",
             GD32_TLI_NCLUT);
      ret = -EINVAL;
    }
  else
    {
      /* Currently, there is no api to set color map for each overlay
       * separately. TLI layers can have different color maps, but is shared
       * for now.
       */

      int n;
      struct fb_cmap_s *priv_cmap = &g_vtable.cmap;

      /* First copy to internal cmap */

      for (n = cmap->first; n < cmap->len && n < GD32_TLI_NCLUT; n++)
        {
          priv_cmap->red[n] = cmap->red[n];
          priv_cmap->green[n] = cmap->green[n];
          priv_cmap->blue[n] = cmap->blue[n];
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
          /* Not supported by TLI */

          priv_cmap->transp[n] = cmap->transp[n];
#endif
        }

      priv_cmap->first = cmap->first;
      priv_cmap->len = cmap->len;

      /* Update the layer clut register */

      nxmutex_lock(&g_lock);

      for (n = 0; n < TLI_NLAYERS; n++)
        {
          struct gd32_tli_s *layer = &priv->layer[n];
          gd32_tli_lputclut(layer, priv_cmap);
        }

#ifdef CONFIG_GD32F4_IPA
      /* Update ipa cmap */

      priv->ipa->setclut(cmap);
#endif
      nxmutex_unlock(&g_lock);

      ret = OK;
    }

  return ret;
}
#endif /* CONFIG_GD32F4_FB_CMAP */

/****************************************************************************
 * Name: gd32_ioctl_waitforvsync
 * Description:
 *   Entrypoint ioctl FBIO_WAITFORSYNC
 ****************************************************************************/

#ifdef CONFIG_FB_SYNC
static int gd32_waitforvsync(struct fb_vtable_s *vtable)
{
  int ret;

  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable);

  /* Wait upon vertical synchronization. */

  ret = gd32_tli_reload(TLI_RL_FBR, true);

  return ret;
}
#endif /* CONFIG_FB_SYNC */

/****************************************************************************
 * Name: gd32_getoverlayinfo
 * Description:
 *   Entrypoint ioctl FBIOGET_OVERLAYINFO
 ****************************************************************************/

#ifdef CONFIG_FB_OVERLAY
static int gd32_getoverlayinfo(struct fb_vtable_s *vtable,
                               int overlayno,
                               struct fb_overlayinfo_s *oinfo)
{
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  lcdinfo("vtable=%p overlay=%d oinfo=%p\n", vtable, overlayno, oinfo);
  DEBUGASSERT(vtable != NULL && priv == &g_vtable);

  if (overlayno < TLI_NOVERLAYS)
    {
      struct gd32_tli_s *layer = &priv->layer[overlayno];
      memcpy(oinfo, &layer->oinfo, sizeof(struct fb_overlayinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_settransp
 * Description:
 *   Entrypoint ioctl FBIOSET_TRANSP
 ****************************************************************************/

static int gd32_settransp(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo)
{
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable);
  lcdinfo("vtable=%p, overlay=%d, transp=%02x, transp_mode=%02x\n", vtable,
          oinfo->overlay, oinfo->transp.transp, oinfo->transp.transp_mode);

  if (oinfo->transp.transp_mode > 1)
    {
      lcderr("ERROR: Returning ENOSYS, transparency mode not supported\n");
      return -ENOSYS;
    }

  if (oinfo->overlay < TLI_NOVERLAYS)
    {
      struct gd32_tli_s *layer = &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      layer->oinfo.transp.transp = oinfo->transp.transp;
      layer->oinfo.transp.transp_mode = oinfo->transp.transp_mode;

#ifdef CONFIG_GD32F4_IPA
      if (layer->oinfo.transp.transp_mode == 0)
        {
          layer->ipainfo.transp_mode = GD32_IPA_PFCCR_AM_CONST;
        }
      else if (layer->oinfo.transp.transp_mode == 1)
        {
          layer->ipainfo.transp_mode = GD32_IPA_PFCCR_AM_PIXEL;
        }

      if (oinfo->overlay < TLI_NLAYERS)
#endif
        {
          /* Set TLI blendmode and alpha value */

          gd32_tli_ltransp(layer, layer->oinfo.transp.transp,
                           layer->oinfo.transp.transp_mode);
        }

      nxmutex_unlock(layer->lock);
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_setchromakey
 * Description:
 *   Entrypoint ioctl FBIOSET_CHROMAKEY
 ****************************************************************************/

static int gd32_setchromakey(struct fb_vtable_s *vtable,
                             const struct fb_overlayinfo_s *oinfo)
{
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, chromakey=%08" PRIx32 "\n", vtable,
          oinfo->overlay, oinfo->chromakey);

  if (oinfo->overlay < TLI_NLAYERS)
    {
      int ret;
      struct gd32_tli_s *layer = &priv->layer[oinfo->overlay];

#ifndef CONFIG_GD32F4_TLI_L1_CHROMAKEY
      if (oinfo->overlay == TLI_LAYER_L1)
        {
          return -ENOSYS;
        }
#endif

#ifndef CONFIG_GD32F4_TLI_L2_CHROMAKEY
      if (oinfo->overlay == TLI_LAYER_L2)
        {
          return -ENOSYS;
        }
#endif

      nxmutex_lock(layer->lock);
#ifdef CONFIG_GD32F4_FB_CMAP
      if (oinfo->chromakey >= g_vtable.cmap.len)
        {
          lcderr("ERROR: Clut index %" PRId32 " is out of range\n",
                 oinfo->chromakey);
          ret = -EINVAL;
        }
      else
#endif
        {
          layer->oinfo.chromakey = oinfo->chromakey;

          /* Set chromakey */

          gd32_tli_lchromakey(layer, layer->oinfo.chromakey);
          ret = OK;
        }

      nxmutex_unlock(layer->lock);
      return ret;
    }
#ifdef CONFIG_GD32F4_IPA
  else if (oinfo->overlay < TLI_NOVERLAYS)
    {
      /* Chromakey not supported by IPA */

      return -ENOSYS;
    }
#endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_setcolor
 * Description:
 *   Entrypoint ioctl FBIOSET_COLOR
 ****************************************************************************/

static int gd32_setcolor(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, color=%08" PRIx32 "\n",
          vtable, oinfo->overlay, oinfo->color);

  if (oinfo->overlay < TLI_NOVERLAYS)
    {
#ifdef CONFIG_GD32F4_IPA

      /* Set color within the active overlay is not supported by TLI. So use
       * IPA controller instead when configured.
       */

      int ret;
      struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)
          vtable;
      struct gd32_tli_s *layer = &priv->layer[oinfo->overlay];
      struct fb_overlayinfo_s *poverlay = layer->ipainfo.oinfo;

      DEBUGASSERT(&layer->oinfo == poverlay);

      nxmutex_lock(layer->lock);
      poverlay->color = oinfo->color;
      ret = priv->ipa->fillcolor(&layer->ipainfo, &poverlay->sarea,
                                 poverlay->color);
      nxmutex_unlock(layer->lock);

      return ret;
#else
      /* Coloring not supported by TLI */

      return -ENOSYS;
#endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_setblank
 * Description:
 *   Entrypoint ioctl FBIOSET_BLANK
 ****************************************************************************/

static int gd32_setblank(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo)
{
  struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, blank=%02x\n",
          vtable, oinfo->overlay, oinfo->blank);

  if (oinfo->overlay < TLI_NLAYERS)
    {
      struct gd32_tli_s *layer = &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      layer->oinfo.blank = oinfo->blank;

      /* Enable or disable layer */

      gd32_tli_lenable(layer, (layer->oinfo.blank == 0));
      nxmutex_unlock(layer->lock);

      return OK;
    }
#ifdef CONFIG_GD32F4_IPA
  else if (oinfo->overlay < TLI_NOVERLAYS)
    {
      /* IPA overlays are non visible */

      return OK;
    }
#endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_setarea
 * Description:
 *   Entrypoint ioctl FBIOSET_AREA
 ****************************************************************************/

static int gd32_setarea(struct fb_vtable_s *vtable,
                        const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, x=%d, y=%d, w=%d, h=%d\n", vtable,
          oinfo->overlay, oinfo->sarea.x, oinfo->sarea.y, oinfo->sarea.w,
          oinfo->sarea.h);

  if (oinfo->overlay < TLI_NLAYERS)
    {
      /* TLI area is defined by the overlay size (display resolution) only */

      return -ENOSYS;
    }

#ifdef CONFIG_GD32F4_IPA
  if (oinfo->overlay < TLI_NOVERLAYS)
    {
      struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)
          vtable;
      struct gd32_tli_s *layer = &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      memcpy(&layer->oinfo.sarea, &oinfo->sarea, sizeof(struct fb_area_s));
      nxmutex_unlock(layer->lock);

      return OK;
    }
#endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_blit
 * Description:
 *   Entrypoint ioctl FBIOSET_BLIT
 ****************************************************************************/

#ifdef CONFIG_FB_OVERLAY_BLIT
static int gd32_blit(struct fb_vtable_s *vtable,
                     const struct fb_overlayblit_s *blit)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && blit != NULL);
  lcdinfo("vtable = %p, blit = %p\n", vtable, blit);

  if (blit->dest.overlay < TLI_NOVERLAYS &&
      blit->src.overlay < TLI_NOVERLAYS)
    {
#ifdef CONFIG_GD32F4_IPA
      int ret;
      struct fb_area_s sarea;
      const struct fb_area_s *darea = &blit->dest.area;
      struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)
          vtable;
      struct gd32_tli_s *dlayer = &priv->layer[blit->dest.overlay];
      struct gd32_tli_s *slayer = &priv->layer[blit->src.overlay];

      DEBUGASSERT(&dlayer->oinfo == dlayer->ipainfo.oinfo &&
                  &slayer->oinfo == slayer->ipainfo.oinfo);

      /* IPA doesn't support image scale, so set to the smallest area */

      memcpy(&sarea, &blit->src.area, sizeof(struct fb_area_s));

      /* Check if area is within the entire overlay */

      if (!gd32_tli_lvalidate(dlayer, darea) ||
          !gd32_tli_lvalidate(slayer, &sarea))
        {
          return -EINVAL;
        }

      sarea.w = MIN(darea->w, sarea.w);
      sarea.h = MIN(darea->h, sarea.h);

      nxmutex_lock(dlayer->lock);
      ret = priv->ipa->blit(&dlayer->ipainfo, darea->x, darea->y,
                            &slayer->ipainfo, &sarea);
      nxmutex_unlock(dlayer->lock);

      return ret;
#else
      /* TLI doesn't support blit transfer */

      return -ENOSYS;
#endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: gd32_blend
 * Description:
 *   Entrypoint ioctl FBIOSET_BLEND
 ****************************************************************************/

static int gd32_blend(struct fb_vtable_s *vtable,
                      const struct fb_overlayblend_s *blend)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && blend != NULL);
  lcdinfo("vtable = %p, blend = %p\n", vtable, blend);

  if (blend->dest.overlay < TLI_NOVERLAYS &&
      blend->foreground.overlay < TLI_NOVERLAYS &&
      blend->background.overlay < TLI_NOVERLAYS)
    {
#ifdef CONFIG_GD32F4_IPA
      int ret;
      struct fb_area_s barea;
      const struct fb_area_s *darea = &blend->dest.area;
      const struct fb_area_s *farea = &blend->foreground.area;
      struct gd32_tlidev_s *priv = (struct gd32_tlidev_s *)
          vtable;
      struct gd32_tli_s *dlayer = &priv->layer[blend->dest.overlay];
      struct gd32_tli_s *flayer =
          &priv->layer[blend->foreground.overlay];
      struct gd32_tli_s *blayer =
          &priv->layer[blend->background.overlay];

      DEBUGASSERT(&dlayer->oinfo == dlayer->ipainfo.oinfo &&
                  &flayer->oinfo == flayer->ipainfo.oinfo &&
                  &blayer->oinfo == blayer->ipainfo.oinfo);

      /* IPA doesn't support image scale, so set to the smallest area */

      memcpy(&barea, &blend->background.area, sizeof(struct fb_area_s));

      /* Check if area is within the entire overlay */

      if (!gd32_tli_lvalidate(dlayer, darea) ||
          !gd32_tli_lvalidate(flayer, farea) ||
          !gd32_tli_lvalidate(blayer, &barea))
        {
          lcderr("ERROR: Returning EINVAL\n");
          return -EINVAL;
        }

      barea.w = MIN(darea->w, barea.w);
      barea.h = MIN(darea->h, barea.h);
      barea.w = MIN(farea->w, barea.w);
      barea.h = MIN(farea->h, barea.h);

      nxmutex_lock(dlayer->lock);
      ret = priv->ipa->blend(&dlayer->ipainfo, darea->x, darea->y,
                             &flayer->ipainfo, farea->x, farea->y,
                             &blayer->ipainfo, &barea);
      nxmutex_unlock(dlayer->lock);

      return ret;
#else
      /* TLI doesn't support blend transfer */

      return -ENOSYS;
#endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#endif /* CONFIG_FB_OVERLAY_BLIT */
#endif /* CONFIG_FB_OVERLAY */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_tlireset
 *
 * Description:
 *   Reset TLI via APB2RSTR
 *
 ****************************************************************************/

void gd32_tlireset(void)
{
  uint32_t regval = getreg32(GD32_RCU_APB2RST);
  putreg32(regval | RCU_APB2RST_TLIRST, GD32_RCU_APB2RST);
  putreg32(regval & ~RCU_APB2RST_TLIRST, GD32_RCU_APB2RST);
}

/****************************************************************************
 * Name: gd32_tliinitialize
 *
 * Description:
 *   Initialize the tli controller
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

int gd32_tliinitialize(void)
{
  int ret = OK;

  lcdinfo("Initialize TLI driver\n");

  if (g_initialized == true)
    {
      return ret;
    }

  /* Disable the LCD */

  gd32_tli_enable(false);

  lcdinfo("Configuring the LCD controller\n");

  /* Configure LCD periphery */

  lcdinfo("Configure lcd periphery\n");
  gd32_tli_periphconfig();

  /* Configure interrupts */

  lcdinfo("Configure interrupts\n");
  gd32_tli_irqconfig();

  /* Configure global tli register */

  lcdinfo("Configure global register\n");
  gd32_tli_globalconfig();

#ifdef CONFIG_GD32F4_IPA
  /* Initialize the ipa controller */

  ret = gd32_ipainitialize();

  if (ret != OK)
    {
      return ret;
    }

  /* Bind the ipa interface */

  g_vtable.ipa = gd32_ipadev();
  DEBUGASSERT(g_vtable.ipa != NULL);
#endif

#ifdef CONFIG_GD32F4_FB_CMAP
  /* Cleanup clut */

  memset(&g_redclut, 0, GD32_TLI_NCLUT);
  memset(&g_blueclut, 0, GD32_TLI_NCLUT);
  memset(&g_greenclut, 0, GD32_TLI_NCLUT);
#ifdef CONFIG_GD32F4_FB_TRANSPARENCY
  memset(&g_transpclut, 0, GD32_TLI_NCLUT);
#endif
#endif /* CONFIG_GD32F4_FB_CMAP */

  /* Initialize tli layer */

  lcdinfo("Initialize tli layer\n");
  gd32_tli_linit(TLI_LAYER_L1);
#ifdef CONFIG_GD32F4_TLI_L2
  gd32_tli_linit(TLI_LAYER_L2);
#endif

#ifdef CONFIG_GD32F4_IPA
  gd32_tli_ipalinit();
#endif
  /* Enable the backlight */

#ifdef CONFIG_GD32F4_LCD_BACKLIGHT
  gd32_backlight(true);
#endif

  /* Reload shadow register */

  lcdinfo("Reload shadow register\n");
  gd32_tli_reload(TLI_RL_RQR, false);

  /* Turn the LCD on */

  lcdinfo("Enabling the display\n");
  gd32_tli_enable(true);

  /* Set initialized state */

  g_initialized = true;
  return ret;
}

/****************************************************************************
 * Name: gd32_tligetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Reference to the framebuffer object (NULL on failure)
 *
 ****************************************************************************/

struct fb_vtable_s *gd32_tligetvplane(int vplane)
{
  lcdinfo("vplane: %d\n", vplane);

  if (vplane == 0)
    {
      return &g_vtable.vtable;
    }

  return NULL;
}

/****************************************************************************
 * Name: gd32_tliuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer driver.  Bad things will happen if you
 *   call this without first calling fb_initialize()!
 *
 ****************************************************************************/

void gd32_tliuninitialize(void)
{
  /* Disable all tli interrupts */

  gd32_tli_irqctrl(0, TLI_INTEN_LCRIE | TLI_INTEN_TEIE |
                          TLI_INTEN_FEIE | TLI_INTEN_LMIE);

  up_disable_irq(g_interrupt.irq);
  irq_detach(g_interrupt.irq);

  /* Disable the LCD controller */

  gd32_tli_enable(false);

  /* Set initialized state */

  g_initialized = false;
}

/****************************************************************************
 * Name: gd32_backlight
 *
 * Description:
 *   Provide this interface to turn the backlight on and off.
 *
 * Input Parameters:
 *   blon - Enable or disable the lcd backlight
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_LCD_BACKLIGHT
void gd32_backlight(bool blon)
{
  /* Set default backlight level CONFIG_GD32F4_TLI_DEFBACKLIGHT */

  lcderr("ERROR: Not supported\n");
}
#endif
