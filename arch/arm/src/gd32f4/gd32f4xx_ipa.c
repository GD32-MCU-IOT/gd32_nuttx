/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_ipa.c
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/fb.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "gd32f4xx.h"
#include "hardware/gd32f4xx_tli.h"
#include "hardware/gd32f4xx_ipa.h"
#include "gd32f4xx_ipa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA2D supported operation layer (output, foreground, background) */

#define IPA_NLAYERS                       3

/* DMA2D blender control */

#define GD32_IPA_CTL_PFCM_BLIT            IPA_CTL_PFCM(0)
#define GD32_IPA_CTL_PFCM_BLITPFC         IPA_CTL_PFCM(1)
#define GD32_IPA_CTL_PFCM_BLEND           IPA_CTL_PFCM(2)
#define GD32_IPA_CTL_PFCM_COLOR           IPA_CTL_PFCM(3)
#define GD32_IPA_CTL_PFCM_CLEAR           GD32_IPA_CTL_PFCM_BLITPFC | \
                                            GD32_IPA_CTL_PFCM_BLEND   | \
                                            GD32_IPA_CTL_PFCM_COLOR

/* Only 8 bit per pixel overall supported */

#define IPA_PF_BYPP(n)                    ((n) / 8)

/* CC clut size */

#define IPA_NCLUT_SIZE                     GD32_IPA_NCLUT - 1

/* Layer argb cmap conversion */

#define IPA_CLUT_ALPHA(n)                 ((uint32_t)(n) << 24)
#define IPA_NCLUT_RED(n)                   ((uint32_t)(n) << 16)
#define IPA_NCLUT_GREEN(n)                 ((uint32_t)(n) << 8)
#define IPA_NCLUT_BLUE(n)                  ((uint32_t)(n) << 0)

#define IPA_CMAP_ALPHA(n)                 ((uint32_t)(n) >> 24)
#define IPA_CMAP_RED(n)                   ((uint32_t)(n) >> 16)
#define IPA_CMAP_GREEN(n)                 ((uint32_t)(n) >> 8)
#define IPA_CMAP_BLUE(n)                  ((uint32_t)(n) >> 0)

/* Debug option */

#ifdef CONFIG_GD32F4_IPA_REGDEBUG
#  define regerr                            lcderr
#  define reginfo                           lcdinfo
#else
#  define regerr(x...)
#  define reginfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA2D General layer information */

struct gd32_ipa_s
{
  struct ipa_layer_s ipa;  /* Public dma2d interface */

#ifdef CONFIG_GD32F4_FB_CMAP
  uint32_t *clut;              /* Color lookup table */
#endif
  mutex_t  *lock;              /* Ensure mutually exclusive access */
};

/* Interrupt handling */

struct gd32_interrupt_s
{
  int    irq;       /* irq number */
  int  error;       /* Interrupt error */
  sem_t *sem;       /* Semaphore for waiting for irq */
};

/* This enumeration foreground and background layer supported by the dma2d
 * controller
 */

enum gd32_layer_e
{
  IPA_LAYER_LFORE = 0,       /* Foreground Layer */
  IPA_LAYER_LBACK,           /* Background Layer */
  IPA_LAYER_LOUT,            /* Output Layer */
};

/* DMA2D memory address register */

static const uintptr_t gd32_mar_layer_t[IPA_NLAYERS] =
{
  GD32_IPA_FMADDR,
  GD32_IPA_BMADDR,
  GD32_IPA_DMADDR
};

/* DMA2D offset register */

static const uintptr_t gd32_or_layer_t[IPA_NLAYERS] =
{
  GD32_IPA_FLOFF,
  GD32_IPA_BLOFF,
  GD32_IPA_DLOFF
};

/* DMA2D pfc control register */

static const uintptr_t gd32_pfccr_layer_t[IPA_NLAYERS] =
{
  GD32_IPA_FPCTL,
  GD32_IPA_BPCTL,
  GD32_IPA_DPCTL
};

/* DMA2D color register */

static const uintptr_t gd32_color_layer_t[IPA_NLAYERS] =
{
  GD32_IPA_FPV,
  GD32_IPA_BPV,
  GD32_IPA_DPV
};

/* DMA2D clut memory address register */

#ifdef CONFIG_GD32F4_FB_CMAP
static const uintptr_t gd32_cmar_layer_t[IPA_NLAYERS - 1] =
{
  GD32_IPA_FLMADDR,
  GD32_IPA_BLMADDR
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Private functions */

static void gd32_ipa_control(uint32_t setbits, uint32_t clrbits);
static int gd32_ipairq(int irq, void *context, void *arg);
static int gd32_ipa_waitforirq(void);
static int gd32_ipa_start(void);
#ifdef CONFIG_GD32F4_FB_CMAP
static int gd32_ipa_loadclut(uintptr_t reg);
#endif
static uint32_t gd32_ipa_memaddress(
                                   struct gd32_ipa_overlay_s *oinfo,
                                   uint32_t xpos, uint32_t ypos);
static uint32_t gd32_ipa_lineoffset(
                                   struct gd32_ipa_overlay_s *oinfo,
                                   const struct fb_area_s *area);
static void gd32_ipa_lfifo(struct gd32_ipa_overlay_s *oinfo,
                              int lid,
                              uint32_t xpos, uint32_t ypos,
                              const struct fb_area_s *area);
static void gd32_ipa_lcolor(int lid, uint32_t argb);
static void gd32_ipa_llnr(const struct fb_area_s *area);
static int gd32_ipa_loutpfc(uint8_t fmt);
static void gd32_ipa_lpfc(int lid, uint32_t blendmode, uint8_t alpha,
                             uint8_t fmt);

/* Public Functions */

#ifdef CONFIG_GD32F4_FB_CMAP
static int gd32_ipa_setclut(const struct fb_cmap_s *cmap);
#endif
static int gd32_ipa_fillcolor(struct gd32_ipa_overlay_s *oinfo,
                                 const struct fb_area_s *area,
                                 uint32_t argb);
static int gd32_ipa_blit(struct gd32_ipa_overlay_s *doverlay,
                            uint32_t destxpos, uint32_t destypos,
                            struct gd32_ipa_overlay_s *soverlay,
                            const struct fb_area_s *sarea);
static int gd32_ipa_blend(struct gd32_ipa_overlay_s *doverlay,
                             uint32_t destxpos, uint32_t destypos,
                             struct gd32_ipa_overlay_s *foverlay,
                             uint32_t forexpos, uint32_t foreypos,
                             struct gd32_ipa_overlay_s *boverlay,
                             const struct fb_area_s *barea);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The initialized state of the driver */

static bool g_initialized;

/* Allocate clut */

#ifdef CONFIG_GD32F4_FB_CMAP
static uint32_t g_clut[GD32_IPA_NCLUT *
#  ifdef CONFIG_GD32F4_FB_TRANSPARENCY
                      4
#  else
                      3
#  endif
                      / 4];
#endif /* CONFIG_GD32F4_FB_CMAP */

/* The DMA2D mutex that enforces mutually exclusive access */

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/* Semaphore for interrupt handling */

static sem_t g_semirq = SEM_INITIALIZER(0);

/* This structure provides irq handling */

static struct gd32_interrupt_s g_interrupt =
{
  .irq     = GD32_IRQ_IPA,
  .error   = OK,
  .sem     = &g_semirq
};

static struct gd32_ipa_s g_ipadev =
{
  .ipa =
  {
#ifdef CONFIG_GD32F4_FB_CMAP
    .setclut   = gd32_ipa_setclut,
#endif
    .fillcolor = gd32_ipa_fillcolor,
    .blit      = gd32_ipa_blit,
    .blend     = gd32_ipa_blend
  },
#ifdef CONFIG_GD32F4_FB_CMAP
  .clut = g_clut,
#endif
  .lock = &g_lock
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_ipa_control
 *
 * Description:
 *   Change the IPA control register
 *
 * Input Parameters:
 *   setbits - The bits to set
 *   clrbits - The bits to clear
 *
 ****************************************************************************/

static void gd32_ipa_control(uint32_t setbits, uint32_t clrbits)
{
  uint32_t   cr;

  lcdinfo("setbits=%08" PRIx32 ", clrbits=%08" PRIx32 "\n",
          setbits, clrbits);

  cr = getreg32(GD32_IPA_CTL);
  cr &= ~clrbits;
  cr |= setbits;

  lcdinfo("cr=%08" PRIx32 "\n", cr);
  putreg32(cr, GD32_IPA_CTL);
}

/****************************************************************************
 * Name: gd32_ipairq
 *
 * Description:
 *   IPA interrupt handler
 *
 ****************************************************************************/

static int gd32_ipairq(int irq, void *context, void *arg)
{
  int ret;
  uint32_t regval = getreg32(GD32_IPA_INTF);
  struct gd32_interrupt_s *priv = &g_interrupt;

  reginfo("irq = %d, regval = %08x\n", irq, regval);

  if (regval & IPA_INTF_FTFIF)
    {
      /* Transfer complete interrupt */

      /* Clear the interrupt status register */

      reginfo("DMA transfer complete\n");
      putreg32(IPA_INTC_FTFIFC, GD32_IPA_INTC);
      priv->error = OK;
    }
#ifdef CONFIG_GD32F4_IPA_L8
  else if (regval & IPA_INTF_LLFIF)
    {
      /* CLUT transfer complete interrupt */

      /* Clear the interrupt status register */

      reginfo("CLUT transfer complete\n");
      putreg32(IPA_INTC_LLFIFC, GD32_IPA_INTC);
      priv->error = OK;
    }
#endif
  else if (regval & IPA_INTF_TLMIF)
    {
      /* Watermark transfer complete interrupt */

      /* Clear the interrupt status register */

      reginfo("Watermark transfer complete\n");
      putreg32(IPA_INTC_TLMIFC, GD32_IPA_INTC);
      priv->error = OK;
    }
  else if (regval & IPA_INTF_TAEIF)
    {
      /* Transfer error interrupt */

      /* Clear the interrupt status register */

      reginfo("ERROR: transfer\n");
      putreg32(IPA_INTC_TAEIFC, GD32_IPA_INTC);
      priv->error = -ECANCELED;
    }
  else if (regval & IPA_INTF_LACIF)
    {
      /* CLUT access error interrupt */

      /* Clear the interrupt status register */

      reginfo("ERROR: clut access\n");
      putreg32(IPA_INTC_LACIFC, GD32_IPA_INTC);
      priv->error = -ECANCELED;
    }
  else if (regval & IPA_INTF_WCFIF)
    {
      /* Configuration error interrupt */

      /* Clear the interrupt status register */

      reginfo("ERROR: configuration\n");
      putreg32(IPA_INTC_WCFIFC, GD32_IPA_INTC);
      priv->error = -ECANCELED;
    }
  else
    {
      /* Unknown irq, should not occur */

      DEBUGASSERT("Unknown interrupt error\n");
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
 * Name: gd32_ipa_waitforirq
 *
 * Description:
 *   Helper waits until the IPA irq occurs. That means that an ongoing clut
 *   loading or dma transfer was completed.
 *   Note! The caller must use this function within a critical section.
 *
 * Returned Value:
 *   On success OK otherwise ERROR
 *
 ****************************************************************************/

static int gd32_ipa_waitforirq(void)
{
  int ret;
  struct gd32_interrupt_s *priv = &g_interrupt;

  ret = nxsem_wait(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_wait() failed\n");
      return ret;
    }

  ret = priv->error;

  return ret;
}

/****************************************************************************
 * Name: gd32_ipa_loadclut
 *
 * Description:
 *   Starts clut loading but doesn't wait until loading is complete!
 *
 * Input Parameters:
 *   pfcreg - PFC control Register
 *
 * Returned Value:
 *   On success - OK
 *   On error - -EINVAL
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_IPA_L8
static int gd32_ipa_loadclut(uintptr_t pfcreg)
{
  int      ret;
  uint32_t regval;

  /* Start clut loading */

  regval  = getreg32(pfcreg);
  regval |= IPA_FPCTL_FLLEN;
  reginfo("set regval=%08x\n", regval);
  putreg32(regval, pfcreg);
  reginfo("configured regval=%08x\n", getreg32(pfcreg));

  /* Wait until clut is finished */

  ret = gd32_ipa_waitforirq();

  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_ipa_start
 *
 * Description:
 *   Starts the dma transfer and waits until completed.
 *
 * Input Parameters:
 *   reg       - Register to set the start
 *   startflag - The related flag to start the dma transfer
 *   irqflag   - The interrupt enable flag in the DMA2D_CR register
 *
 ****************************************************************************/

static int gd32_ipa_start(void)
{
  int ret;

  /* Start dma transfer */

  gd32_ipa_control(IPA_CTL_TEN, 0);

  /* wait until transfer is complete */

  ret = gd32_ipa_waitforirq();

  return ret;
}

/****************************************************************************
 * Name: gd32_ipa_memaddress
 *
 * Description:
 *   Helper to calculate the layer memory address
 *
 * Input Parameters:
 *   oinfo - Reference to overlay information
 *   xpos  - x-Offset
 *   ypos  - y-Offset
 *
 * Returned Value:
 *   memory address
 *
 ****************************************************************************/

static uint32_t gd32_ipa_memaddress(
                                   struct gd32_ipa_overlay_s *oinfo,
                                   uint32_t xpos, uint32_t ypos)
{
  uint32_t offset;
  struct fb_overlayinfo_s *poverlay = oinfo->oinfo;

  offset = xpos * IPA_PF_BYPP(poverlay->bpp) + poverlay->stride * ypos;

  lcdinfo("%" PRIx32 ", offset=%" PRId32 "\n",
          ((uint32_t) poverlay->fbmem) + offset, offset);
  return ((uint32_t) poverlay->fbmem) + offset;
}

/****************************************************************************
 * Name: gd32_ipa_lineoffset
 *
 * Description:
 *   Helper to calculate the layer line offset
 *
 * Description:
 *   Helper to calculate the layer line offset
 *
 * Input Parameters:
 *   oinfo - Reference to overlay information
 *
 * Returned Value:
 *   line offset
 *
 ****************************************************************************/

static uint32_t gd32_ipa_lineoffset(
                                   struct gd32_ipa_overlay_s *oinfo,
                                   const struct fb_area_s *area)
{
  uint32_t loffset;

  /* offset at the end of each line in the context to the area layer */

  loffset = oinfo->xres - area->w;

  lcdinfo("%" PRId32 "\n", loffset);
  return loffset;
}

/****************************************************************************
 * Name: gd32_ipa_lfifo
 *
 * Description:
 *   Set the fifo for the foreground, background and output layer
 *   Configures the memory address register
 *   Configures the line offset register
 *
 * Input Parameters:
 *   layer - Reference to the common layer state structure
 *
 ****************************************************************************/

static void gd32_ipa_lfifo(struct gd32_ipa_overlay_s *oinfo,
                              int lid, uint32_t xpos, uint32_t ypos,
                              const struct fb_area_s *area)
{
  lcdinfo("oinfo=%p, lid=%d, xpos=%" PRId32 ", ypos=%" PRId32 ", area=%p\n",
           oinfo, lid, xpos, ypos, area);

  putreg32(gd32_ipa_memaddress(oinfo, xpos, ypos),
           gd32_mar_layer_t[lid]);
  putreg32(gd32_ipa_lineoffset(oinfo, area), gd32_or_layer_t[lid]);
}

/****************************************************************************
 * Name: gd32_ipa_lcolor
 *
 * Description:
 *   Set the color for the layer
 *
 * Input Parameters:
 *   lid  - Layer type (output, foreground, background)
 *   argb - argb8888 color
 *
 ****************************************************************************/

static void gd32_ipa_lcolor(int lid, uint32_t argb)
{
  lcdinfo("lid=%d, argb=%08" PRIx32 "\n", lid, argb);
  putreg32(argb, gd32_color_layer_t[lid]);
}

/****************************************************************************
 * Name: gd32_ipa_llnr
 *
 * Description:
 *   Set the number of line register
 *
 * Input Parameters:
 *   area - Reference to area information
 *
 ****************************************************************************/

static void gd32_ipa_llnr(const struct fb_area_s *area)
{
  uint32_t nlrreg;

  lcdinfo("pixel per line: %d, number of lines: %d\n", area->w, area->h);

  nlrreg = getreg32(GD32_IPA_IMS);
  nlrreg = (IPA_IMS_WIDTH(area->w) | IPA_IMS_HEIGHT(area->h));
  putreg32(nlrreg, GD32_IPA_IMS);
}

/****************************************************************************
 * Name: gd32_ipa_loutpfc
 *
 * Description:
 *   Set the output PFC control register
 *
 * Input Parameters:
 *   fmt - DMA2D pixel format
 *
 ****************************************************************************/

static int gd32_ipa_loutpfc(uint8_t fmt)
{
  lcdinfo("pixel format: %d\n", fmt);

  /* Set the mapped pixel format of the destination layer */

  putreg32(IPA_DPCTL_DPF(fmt), GD32_IPA_DPCTL);

  return OK;
}

/****************************************************************************
 * Name: gd32_ipa_lpfc
 *
 * Description:
 *   Configure foreground and background layer PFC control register
 *
 * Input Parameters:
 *   lid       - Layer id (output, foreground, background)
 *   blendmode - Layer blendmode (dma2d register values)
 *   alpha     - Transparency
 *
 ****************************************************************************/

static void gd32_ipa_lpfc(int lid, uint32_t blendmode, uint8_t alpha,
                             uint8_t fmt)
{
  uint32_t   pfccrreg;

  lcdinfo("lid=%d, blendmode=%08" PRIx32 ", alpha=%02x, fmt=%d\n",
          lid, blendmode, alpha, fmt);

  /* Set color format */

  pfccrreg = IPA_FPCTL_FPF(fmt);

#ifdef CONFIG_GD32F4_FB_CMAP
  if (fmt == IPA_PF_L8)
    {
      struct gd32_ipa_s *layer = &g_ipadev;

      /* Load CLUT automatically */

      pfccrreg |= IPA_FPCTL_FLLEN;

      /* Set the CLUT color mode */

#  ifndef CONFIG_GD32F4_FB_TRANSPARENCY
      pfccrreg |= IPA_FPCTL_FLPF;
#  endif

      /* Set CLUT size */

      pfccrreg |= IPA_FPCTL_FCNP(IPA_NCLUT_SIZE);

      /* Set the CLUT memory address */

      putreg32((uint32_t) layer->clut, gd32_cmar_layer_t[lid]);

      /* Start async clut loading */

      gd32_ipa_loadclut(gd32_pfccr_layer_t[lid]);
    }
#endif /* CONFIG_GD32F4_FB_CMAP */

  /* Set alpha blend mode */

  pfccrreg |= IPA_FPCTL_FAVCA(blendmode);

  if (blendmode == GD32_IPA_PFCCR_AM_CONST ||
        blendmode == GD32_IPA_PFCCR_AM_PIXEL)
    {
      /* Set alpha value */

      pfccrreg |= IPA_FPCTL_FPDAV(alpha);
    }

  putreg32(pfccrreg, gd32_pfccr_layer_t[lid]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_ipa_setclut
 *
 * Description:
 *   Configure layer clut (color lookup table).
 *
 * Input Parameters:
 *   cmap   - Color lookup table with up the 256 entries
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifdef CONFIG_GD32F4_FB_CMAP
static int gd32_ipa_setclut(const struct fb_cmap_s *cmap)
{
  int n;
  struct gd32_ipa_s *priv = &g_ipadev;

  lcdinfo("cmap=%p\n", cmap);

  nxmutex_lock(priv->lock);

  for (n = cmap->first; n < cmap->len - 1 && n < GD32_IPA_NCLUT; n++)
    {
      /* Update the layer clut entry, will be automatically loaded before
       * blit operation becomes active
       */

#  ifndef CONFIG_GD32F4_FB_TRANSPARENCY
      uint8_t *clut   = (uint8_t *)g_ipadev.clut;
      uint16_t offset = 3 * n;

      clut[offset]     = cmap->blue[n];
      clut[offset + 1] = cmap->green[n];
      clut[offset + 2] = cmap->red[n];

      reginfo("n=%d, red=%02x, green=%02x, blue=%02x\n", n, clut[offset],
              clut[offset + 1], clut[offset + 2]);
#  else
      uint32_t *clut  = g_ipadev.clut;

      clut[n] = (uint32_t)IPA_CLUT_ALPHA(cmap->transp[n]) |
                (uint32_t)IPA_NCLUT_RED(cmap->red[n]) |
                (uint32_t)IPA_NCLUT_GREEN(cmap->green[n]) |
                (uint32_t)IPA_NCLUT_BLUE(cmap->blue[n]);

      reginfo("n=%d, alpha=%02x, red=%02x, green=%02x, blue=%02x\n", n,
                IPA_NCLUT_ALPHA(cmap->transp[n]),
                IPA_NCLUT_RED(cmap->red[n]),
                IPA_NCLUT_GREEN(cmap->green[n]),
                IPA_NCLUT_BLUE(cmap->blue[n]));
#  endif
    }

  nxmutex_unlock(priv->lock);
  return OK;
}
#endif /* CONFIG_GD32F4_FB_CMAP */

/****************************************************************************
 * Name: gd32_ipa_fillcolor
 *
 * Description:
 *   Fill the selected area of the whole overlay with a specific color.
 *   The caller must ensure that the area is within the entire overlay.
 *
 * Input Parameters:
 *   oinfo - Overlay to fill
 *   area  - Reference to the valid area structure select the area
 *   argb  - Color to fill the selected area. Color must be argb8888
 *           formatted.
 *
 * Returned Value:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the
 *                selected area outside the visible area of the layer.
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int gd32_ipa_fillcolor(struct gd32_ipa_overlay_s *oinfo,
                                 const struct fb_area_s *area,
                                 uint32_t argb)
{
  int ret;
  struct gd32_ipa_s *priv = &g_ipadev;
  DEBUGASSERT(oinfo != NULL && oinfo->oinfo != NULL && area != NULL);

  lcdinfo("oinfo=%p, argb=%08" PRIx32 "\n", oinfo, argb);

#ifdef CONFIG_GD32F4_FB_CMAP
  if (oinfo->fmt == IPA_PF_L8)
    {
      /* CLUT output not supported */

      lcderr("ERROR: Returning ENOSYS, "
             "output to layer with CLUT format not supported.\n");
      return -ENOSYS;
    }
#endif

  nxmutex_lock(priv->lock);

  /* Set output pfc */

  gd32_ipa_loutpfc(oinfo->fmt);

  /* Set output fifo */

  gd32_ipa_lfifo(oinfo, IPA_LAYER_LOUT, area->x, area->y, area);

  /* Set the output color register */

  gd32_ipa_lcolor(IPA_LAYER_LOUT, argb);

  /* Set number of lines and pixel per line */

  gd32_ipa_llnr(area);

  /* Set register to memory transfer */

  gd32_ipa_control(GD32_IPA_CTL_PFCM_COLOR, GD32_IPA_CTL_PFCM_CLEAR);

  /* Start DMA2D and wait until completed */

  ret = gd32_ipa_start();

  if (ret != OK)
    {
      ret = -ECANCELED;
      lcderr("ERROR: Returning ECANCELED\n");
    }

  nxmutex_unlock(priv->lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_ipa_blit
 *
 * Description:
 *   Copy memory from a source overlay (defined by sarea) to destination
 *   overlay position (defined by destxpos and destypos).
 *
 * Input Parameters:
 *   doverlay - Valid reference to the destination overlay
 *   destxpos - Valid selected x position of the destination overlay
 *   destypos - Valid selected y position of the destination overlay
 *   soverlay - Valid reference to the source overlay
 *   sarea    - Valid reference to the selected area of the source overlay
 *
 * Returned Value:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the
 *                selected source area outside the visible area of the
 *                destination layer.
 *                (The visible area usually represents the display size)
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int gd32_ipa_blit(struct gd32_ipa_overlay_s *doverlay,
                            uint32_t destxpos, uint32_t destypos,
                            struct gd32_ipa_overlay_s *soverlay,
                            const struct fb_area_s *sarea)
{
  int      ret;
  uint32_t mode;
  struct gd32_ipa_s *priv = &g_ipadev;

  lcdinfo("doverlay=%p, destxpos=%" PRId32 ", destypos=%" PRId32
          ", soverlay=%p, sarea=%p\n",
          doverlay, destxpos, destypos, soverlay, sarea);

  nxmutex_lock(priv->lock);

  /* Set output pfc */

  gd32_ipa_loutpfc(doverlay->fmt);

  /* Set foreground pfc */

  gd32_ipa_lpfc(IPA_LAYER_LFORE, GD32_IPA_PFCCR_AM_NONE, 0,
                   soverlay->fmt);

  /* Set foreground fifo */

  gd32_ipa_lfifo(soverlay, IPA_LAYER_LFORE, sarea->x, sarea->y, sarea);

  /* Set output fifo */

  gd32_ipa_lfifo(doverlay, IPA_LAYER_LOUT, destxpos, destypos, sarea);

  /* Set number of lines and pixel per line */

  gd32_ipa_llnr(sarea);

  /* Set dma2d mode for blit operation */

  if (doverlay->fmt == soverlay->fmt)
    {
      /* Blit without pfc */

      mode = GD32_IPA_CTL_PFCM_BLIT;
    }
  else
    {
      /* Blit with pfc */

      mode = GD32_IPA_CTL_PFCM_BLITPFC;
    }

  gd32_ipa_control(mode, GD32_IPA_CTL_PFCM_CLEAR);

  /* Start DMA2D and wait until completed */

  ret = gd32_ipa_start();

  if (ret != OK)
    {
      ret = -ECANCELED;
      lcderr("ERROR: Returning ECANCELED\n");
    }

  nxmutex_unlock(priv->lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_ipa_blend
 *
 * Description:
 *   Blends the selected area from a background layer with selected position
 *   of the foreground layer. Copies the result to the selected position of
 *   the destination layer. Note! The content of the foreground and
 *   background layer keeps unchanged as long destination layer is unequal to
 *   the foreground and background layer.
 *
 * Input Parameters:
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
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the
 *                selected source area outside the visible area of the
 *                destination layer.
 *                (The visible area usually represents the display size)
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int gd32_ipa_blend(struct gd32_ipa_overlay_s *doverlay,
                             uint32_t destxpos, uint32_t destypos,
                             struct gd32_ipa_overlay_s *foverlay,
                             uint32_t forexpos, uint32_t foreypos,
                             struct gd32_ipa_overlay_s *boverlay,
                             const struct fb_area_s *barea)
{
  int ret;
  struct gd32_ipa_s *priv = &g_ipadev;

  lcdinfo("doverlay=%p, destxpos=%" PRId32 ", destypos=%" PRId32 ", "
          "foverlay=%p, forexpos=%" PRId32 ", foreypos=%" PRId32 ", "
          "boverlay=%p, barea=%p, barea.x=%d, barea.y=%d, barea.w=%d, "
          "barea.h=%d\n", doverlay, destxpos, destypos, foverlay, forexpos,
          foreypos, boverlay, barea, barea->x, barea->y, barea->w, barea->h);

#ifdef CONFIG_GD32F4_FB_CMAP
  if (doverlay->fmt == IPA_PF_L8)
    {
      /* CLUT output not supported */

      lcderr("ERROR: Returning ENOSYS, "
             "output to layer with CLUT format not supported.\n");
      return -ENOSYS;
    }
#endif

  nxmutex_lock(priv->lock);

  /* Set output pfc */

  gd32_ipa_loutpfc(doverlay->fmt);

  /* Set background pfc */

  gd32_ipa_lpfc(IPA_LAYER_LBACK, boverlay->transp_mode,
                   boverlay->oinfo->transp.transp, boverlay->fmt);

  /* Set foreground pfc */

  gd32_ipa_lpfc(IPA_LAYER_LFORE, foverlay->transp_mode,
                   foverlay->oinfo->transp.transp, foverlay->fmt);

  /* Set background fifo */

  gd32_ipa_lfifo(boverlay, IPA_LAYER_LBACK, barea->x, barea->y, barea);

  /* Set foreground fifo */

  gd32_ipa_lfifo(foverlay, IPA_LAYER_LFORE, forexpos, foreypos, barea);

  /* Set output fifo */

  gd32_ipa_lfifo(doverlay, IPA_LAYER_LOUT, destxpos, destypos, barea);

  /* Set number of lines and pixel per line */

  gd32_ipa_llnr(barea);

  /* Set watermark */

  /* Enable DMA2D blender */

  gd32_ipa_control(GD32_IPA_CTL_PFCM_BLEND, GD32_IPA_CTL_PFCM_CLEAR);

  /* Start DMA2D and wait until completed */

  ret = gd32_ipa_start();

  if (ret != OK)
    {
      ret = -ECANCELED;
      lcderr("ERROR: Returning ECANCELED\n");
    }

  nxmutex_unlock(priv->lock);
  return ret;
}

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

int gd32_ipainitialize(void)
{
  lcdinfo("Initialize IPA driver\n");

  if (g_initialized == false)
    {
      /* Abort current IPA data transfer */

      gd32_ipauninitialize();

      /* Enable IPA is done in rcc_enableahb1, see
       * arch/arm/src/gd32f4/gd32f4xx_rcc.c
       */

#ifdef CONFIG_GD32F4_FB_CMAP
      /* Enable dma2d transfer and clut loading interrupts only */

      gd32_ipa_control(IPA_CTL_FTFIE | IPA_CTL_LLFIE, IPA_CTL_TAEIE |
                          IPA_CTL_TLMIE | IPA_CTL_LACIE | IPA_CTL_WCFIE);
#else
      /* Enable dma transfer interrupt only */

      gd32_ipa_control(IPA_CTL_FTFIE, IPA_CTL_TAEIE | IPA_CTL_TLMIE |
                          IPA_CTL_LACIE | IPA_CTL_LLFIE | IPA_CTL_WCFIE);
#endif

      gd32_ipa_control(IPA_CTL_FTFIE | IPA_CTL_LLFIE | IPA_CTL_TAEIE |
                          IPA_CTL_LACIE | IPA_CTL_LLFIE | IPA_CTL_WCFIE,
                          0);

      /* Attach DMA2D interrupt vector */

      irq_attach(g_interrupt.irq, gd32_ipairq, NULL);

      /* Enable the IRQ at the NVIC */

      up_enable_irq(g_interrupt.irq);

      g_initialized = true;
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_ipauninitialize
 *
 * Description:
 *   Uninitialize the IPA controller
 *
 ****************************************************************************/

void gd32_ipauninitialize(void)
{
  /* Disable IPA interrupts */

  up_disable_irq(g_interrupt.irq);
  irq_detach(g_interrupt.irq);

  /* Abort current IPA transfer */

  gd32_ipa_control(IPA_CTL_TST, 0);

  /* Set initialized state */

  g_initialized = false;
}

/****************************************************************************
 * Name: gd32_ipadev
 *
 * Description:
 *   Get a reference to the IPA controller.
 *
 * Returned Value:
 *   On success - A valid IPA layer reference
 *   On error   - NULL
 *
 ****************************************************************************/

struct ipa_layer_s *gd32_ipadev(void)
{
  return &g_ipadev.ipa;
}
