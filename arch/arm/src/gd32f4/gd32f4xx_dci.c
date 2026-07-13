/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_dci.c
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

/* References:
 *   GD32F4xx User Manual
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/video/imgdata.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "gd32f4xx.h"
#include "gd32f4xx_dci.h"
#include "gd32f4xx_dma.h"
#include "hardware/gd32f4xx_dci.h"
#include "hardware/gd32f4xx_rcu.h"

#ifdef CONFIG_GD32F4_DCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DCI DMA configuration.
 * DCI uses DMA1 Channel 1, sub-peripheral 1 (DMA_REQ_DCI_1)
 * or DMA1 Channel 7, sub-peripheral 1 (DMA_REQ_DCI_2).
 */

#ifndef CONFIG_GD32F4_DCI_DMA_CHANNEL
#  define GD32_DCI_DMA_REQ      DMA_REQ_DCI_1
#else
#  if CONFIG_GD32F4_DCI_DMA_CHANNEL == 1
#    define GD32_DCI_DMA_REQ    DMA_REQ_DCI_1
#  else
#    define GD32_DCI_DMA_REQ    DMA_REQ_DCI_2
#  endif
#endif

/* DCI DMA transfer data width: 32-bit word from DCI_DATA register */

#define GD32_DCI_DMA_PWIDTH     DMA_PERIPH_WIDTH_32BIT
#define GD32_DCI_DMA_MWIDTH     DMA_MEMORY_WIDTH_32BIT

/* DCI DMA transfer direction: peripheral to memory */

#define GD32_DCI_DMA_DIR        DMA_PERIPH_TO_MEMORY

/* DCI DMA priority */

#ifdef CONFIG_GD32F4_DCI_DMA_HIGH_PRIORITY
#  define GD32_DCI_DMA_PRIO     DMA_PRIORITY_ULTRA_HIGH
#else
#  define GD32_DCI_DMA_PRIO     DMA_PRIORITY_HIGH
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_VIDEO
#  define dcierr                  _err
#  define dciwarn                 _warn
#  define dciinfo                 _info
#else
#  define dcierr(x...)
#  define dciwarn(x...)
#  define dciinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DCI device private data structure */

struct gd32_dci_s
{
  struct imgdata_s      data;         /* Must be the first member */
  DMA_HANDLE            dma;          /* DMA handle */
  imgdata_capture_t     capture_cb;   /* Frame capture callback */
  FAR void             *capture_arg;  /* Capture callback argument */
  FAR uint8_t          *fbuffer;      /* Frame buffer address */
  uint32_t              buf_size;     /* Frame buffer size */
  uint16_t              width;        /* Image width */
  uint16_t              height;       /* Image height */
  uint32_t              pixelformat;  /* Pixel format */
  bool                  capturing;    /* Capture in progress */
  mutex_t               lock;         /* Mutex for exclusive access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* imgdata_ops_s interface */

static int gd32_dci_data_init(FAR struct imgdata_s *data);
static int gd32_dci_data_uninit(FAR struct imgdata_s *data);
static int gd32_dci_data_set_buf(FAR struct imgdata_s *data,
                                 uint8_t nr_datafmts,
                                 FAR imgdata_format_t *datafmts,
                                 FAR uint8_t *addr, uint32_t size);
static int
gd32_dci_data_validate_frame_setting(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval);
static int gd32_dci_data_start_capture(FAR struct imgdata_s *data,
                                       uint8_t nr_datafmts,
                                       FAR imgdata_format_t *datafmts,
                                       FAR imgdata_interval_t *interval,
                                       FAR imgdata_capture_t callback,
                                       FAR void *arg);
static int gd32_dci_data_stop_capture(FAR struct imgdata_s *data);

/* Internal functions */

static void gd32_dci_reset(void);
static void gd32_dci_clock_enable(void);
static void gd32_dci_clock_disable(void);
static void gd32_dci_hw_init(FAR struct gd32_dci_s *priv);
static void gd32_dci_hw_deinit(FAR struct gd32_dci_s *priv);
static void gd32_dci_config(FAR struct gd32_dci_s *priv);
static void gd32_dci_dma_config(FAR struct gd32_dci_s *priv);
static void gd32_dci_dma_callback(DMA_HANDLE handle, uint16_t status,
                                  void *arg);
static int  gd32_dci_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* imgdata_ops_s interface implementation */

static const struct imgdata_ops_s g_dci_data_ops =
{
  .init                   = gd32_dci_data_init,
  .uninit                 = gd32_dci_data_uninit,
  .set_buf                = gd32_dci_data_set_buf,
  .validate_frame_setting = gd32_dci_data_validate_frame_setting,
  .start_capture          = gd32_dci_data_start_capture,
  .stop_capture           = gd32_dci_data_stop_capture,
};

/* DCI device instance */

static struct gd32_dci_s g_dci_priv =
{
  .data.ops    = &g_dci_data_ops,
  .lock        = NXMUTEX_INITIALIZER,
  .capturing   = false,
  .capture_cb  = NULL,
  .capture_arg = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dci_putreg
 *
 * Description:
 *   Write a value to a DCI register.
 *
 ****************************************************************************/

static inline void gd32_dci_putreg(uint32_t offset, uint32_t value)
{
  putreg32(value, GD32_DCI_BASE + offset);
}

/****************************************************************************
 * Name: gd32_dci_getreg
 *
 * Description:
 *   Read a value from a DCI register.
 *
 ****************************************************************************/

static inline uint32_t gd32_dci_getreg(uint32_t offset)
{
  return getreg32(GD32_DCI_BASE + offset);
}

/****************************************************************************
 * Name: gd32_dci_modifyreg
 *
 * Description:
 *   Modify a DCI register (read-modify-write).
 *
 ****************************************************************************/

static inline void gd32_dci_modifyreg(uint32_t offset, uint32_t clearbits,
                                      uint32_t setbits)
{
  modifyreg32(GD32_DCI_BASE + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: gd32_dci_clock_enable
 *
 * Description:
 *   Enable the DCI peripheral clock via AHB2.
 *
 ****************************************************************************/

static void gd32_dci_clock_enable(void)
{
  modifyreg32(GD32_RCU_AHB2EN, 0, RCU_AHB2EN_DCIEN);
}

/****************************************************************************
 * Name: gd32_dci_clock_disable
 *
 * Description:
 *   Disable the DCI peripheral clock.
 *
 ****************************************************************************/

static void gd32_dci_clock_disable(void)
{
  modifyreg32(GD32_RCU_AHB2EN, RCU_AHB2EN_DCIEN, 0);
}

/****************************************************************************
 * Name: gd32_dci_reset
 *
 * Description:
 *   Reset the DCI peripheral via AHB2 reset register.
 *
 ****************************************************************************/

static void gd32_dci_reset(void)
{
  /* Assert the DCI reset */

  modifyreg32(GD32_RCU_AHB2RST, 0, RCU_AHB2RST_DCIRST);

  /* Release the DCI reset */

  modifyreg32(GD32_RCU_AHB2RST, RCU_AHB2RST_DCIRST, 0);
}

/****************************************************************************
 * Name: gd32_dci_gpio_init
 *
 * Description:
 *   Configure GPIO pins for DCI operation. The actual pin selections are
 *   defined in the board header file (board.h).
 *
 *   Required pins (defined in board.h):
 *     BOARD_DCI_D0     - DCI data line 0
 *     BOARD_DCI_D1     - DCI data line 1
 *     BOARD_DCI_D2     - DCI data line 2
 *     BOARD_DCI_D3     - DCI data line 3
 *     BOARD_DCI_D4     - DCI data line 4
 *     BOARD_DCI_D5     - DCI data line 5
 *     BOARD_DCI_D6     - DCI data line 6
 *     BOARD_DCI_D7     - DCI data line 7
 *     BOARD_DCI_HSYNC  - DCI horizontal sync
 *     BOARD_DCI_VSYNC  - DCI vertical sync
 *     BOARD_DCI_PIXCLK - DCI pixel clock
 *
 ****************************************************************************/

static void gd32_dci_gpio_init(void)
{
  /* Configure data lines D0-D7 (8-bit interface) */

  gd32_gpio_config(BOARD_DCI_D0);
  gd32_gpio_config(BOARD_DCI_D1);
  gd32_gpio_config(BOARD_DCI_D2);
  gd32_gpio_config(BOARD_DCI_D3);
  gd32_gpio_config(BOARD_DCI_D4);
  gd32_gpio_config(BOARD_DCI_D5);
  gd32_gpio_config(BOARD_DCI_D6);
  gd32_gpio_config(BOARD_DCI_D7);

  /* Configure control lines */

  gd32_gpio_config(BOARD_DCI_HSYNC);
  gd32_gpio_config(BOARD_DCI_VSYNC);
  gd32_gpio_config(BOARD_DCI_PIXCLK);
}

/****************************************************************************
 * Name: gd32_dci_gpio_deinit
 *
 * Description:
 *   Unconfigure GPIO pins for DCI.
 *
 ****************************************************************************/

static void gd32_dci_gpio_deinit(void)
{
  /* Reconfigure pins as inputs (default state).
   * This is handled by resetting the GPIO configuration.
   */
}

/****************************************************************************
 * Name: gd32_dci_hw_init
 *
 * Description:
 *   Initialize the DCI hardware.
 *
 ****************************************************************************/

static void gd32_dci_hw_init(FAR struct gd32_dci_s *priv)
{
  /* Enable the DCI clock */

  gd32_dci_clock_enable();

  /* Reset the DCI peripheral */

  gd32_dci_reset();

  /* Configure the DCI GPIO pins */

  gd32_dci_gpio_init();
}

/****************************************************************************
 * Name: gd32_dci_hw_deinit
 *
 * Description:
 *   Deinitialize the DCI hardware.
 *
 ****************************************************************************/

static void gd32_dci_hw_deinit(FAR struct gd32_dci_s *priv)
{
  /* Disable DCI interrupts */

  gd32_dci_putreg(GD32_DCI_INTEN_OFFSET, 0);

  /* Disable DCI */

  gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET,
                     DCI_CTL_DCIEN | DCI_CTL_CAP, 0);

  /* Release DMA channel */

  if (priv->dma != NULL)
    {
      gd32_dma_stop(priv->dma);
      gd32_dma_channel_free(priv->dma);
      priv->dma = NULL;
    }

  /* Unconfigure GPIO pins */

  gd32_dci_gpio_deinit();

  /* Reset the DCI peripheral */

  gd32_dci_reset();

  /* Disable DCI clock */

  gd32_dci_clock_disable();
}

/****************************************************************************
 * Name: gd32_dci_config
 *
 * Description:
 *   Configure the DCI peripheral according to the capture settings.
 *
 ****************************************************************************/

static void gd32_dci_config(FAR struct gd32_dci_s *priv)
{
  uint32_t regval;

  /* Disable capture function and DCI before configuration */

  gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET,
                     DCI_CTL_CAP | DCI_CTL_DCIEN, 0);

  /* Build the control register value.
   * Default configuration:
   *   - Continuous capture mode
   *   - Pixel clock rising edge (configurable via board.h)
   *   - HSYNC active low
   *   - VSYNC active low
   *   - Capture all frames
   *   - 8-bit interface format
   */

  regval = 0;

  /* Capture mode: continuous */

  regval &= ~DCI_CTL_SNAP;

  /* Clock polarity: configurable */

#ifdef BOARD_DCI_CK_POLARITY_RISING
  regval |= DCI_CTL_CKS;
#endif

  /* HSYNC polarity: configurable */

#ifdef BOARD_DCI_HSYNC_POLARITY_HIGH
  regval |= DCI_CTL_HPS;
#endif

  /* VSYNC polarity: configurable */

#ifdef BOARD_DCI_VSYNC_POLARITY_HIGH
  regval |= DCI_CTL_VPS;
#endif

  /* Frame rate: capture all frames */

  regval |= DCI_CTL_FR_ALL;

  /* Interface format: 8-bit */

  regval |= DCI_CTL_DCIF_8BITS;

  /* Write control register */

  gd32_dci_putreg(GD32_DCI_CTL_OFFSET, regval);
}

/****************************************************************************
 * Name: gd32_dci_dma_config
 *
 * Description:
 *   Configure and start DMA for DCI data transfer.
 *
 ****************************************************************************/

static void gd32_dci_dma_config(FAR struct gd32_dci_s *priv)
{
  dma_single_data_parameter_struct dma_cfg;

  DEBUGASSERT(priv->dma != NULL);
  DEBUGASSERT(priv->fbuffer != NULL);

  /* Stop any ongoing DMA transfer */

  gd32_dma_stop(priv->dma);

  /* Configure DMA channel for DCI:
   *   - Peripheral address: DCI_DATA register
   *   - Memory address: frame buffer
   *   - Direction: peripheral to memory
   *   - Peripheral data width: 32-bit (DCI_DATA is 32-bit)
   *   - Memory data width: 32-bit
   *   - Peripheral address: no increment (always read from DCI_DATA)
   *   - Memory address: increment
   *   - Circular mode: enabled for continuous capture
   *   - Priority: high
   */

  memset(&dma_cfg, 0, sizeof(dma_cfg));
  dma_cfg.periph_addr         = GD32_DCI_DATA;
  dma_cfg.memory0_addr        = (uint32_t)(uintptr_t)priv->fbuffer;
  dma_cfg.number              = priv->buf_size / 4;  /* Number of 32-bit words */
  dma_cfg.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
  dma_cfg.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
  dma_cfg.periph_memory_width = DMA_WIDTH_32BITS_SELECT;
  dma_cfg.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
  dma_cfg.direction           = DMA_PERIPH_TO_MEMORY;
  dma_cfg.priority            = DMA_PRIO_ULTRA_HIGHSELECT;

  gd32_dma_setup(priv->dma, &dma_cfg, 0);  /* Single data mode */

  /* Start DMA with transfer complete and error interrupts */

  gd32_dma_start(priv->dma, gd32_dci_dma_callback, priv,
                 DMA_CHXCTL_FTFIE | DMA_CHXCTL_TAEIE);
}

/****************************************************************************
 * Name: gd32_dci_dma_callback
 *
 * Description:
 *   DMA transfer complete callback.
 *
 ****************************************************************************/

static void gd32_dci_dma_callback(DMA_HANDLE handle, uint16_t status,
                                  void *arg)
{
  FAR struct gd32_dci_s *priv = (FAR struct gd32_dci_s *)arg;
  UNUSED(priv);

  /* Filter out FEEIF (Bit 0) which is commonly set at circular/burst
   * boundaries and is harmless. Filter out SDEIF (Bit 2) which can happen
   * momentarily without breaking the capture.
   */

  if (status & DMA_INTF_TAEIF)
    {
      dcierr("DCI DMA error, status: 0x%04x\n", status);
    }
}

/****************************************************************************
 * Name: gd32_dci_interrupt
 *
 * Description:
 *   DCI interrupt handler.
 *
 ****************************************************************************/

static int gd32_dci_interrupt(int irq, void *context, void *arg)
{
  FAR struct gd32_dci_s *priv = (FAR struct gd32_dci_s *)arg;
  uint32_t intf;
  struct timeval ts;
  struct timespec tspec;

  /* Read interrupt flags */

  intf = gd32_dci_getreg(GD32_DCI_INTF_OFFSET);

  /* End of frame interrupt */

  if (intf & DCI_INTF_EFIF)
    {
      /* Clear the end of frame interrupt flag */

      gd32_dci_putreg(GD32_DCI_INTC_OFFSET, DCI_INTC_EFFC);

      /* Get current timestamp */

      clock_systime_timespec(&tspec);
      ts.tv_sec  = tspec.tv_sec;
      ts.tv_usec = tspec.tv_nsec / 1000;

      /* Notify the upper layer that a frame is ready */

      if (priv->capturing && priv->capture_cb != NULL)
        {
          priv->capture_cb(0, priv->buf_size, &ts, priv->capture_arg);
        }
    }

  /* FIFO overrun interrupt */

  if (intf & DCI_INTF_OVRIF)
    {
      /* Clear the overrun interrupt flag */

      gd32_dci_putreg(GD32_DCI_INTC_OFFSET, DCI_INTC_OVRFC);

      dciwarn("DCI FIFO overrun\n");

      if (priv->dma != NULL && priv->capturing && priv->fbuffer != NULL)
        {
          /* Disable capture to flush DCI FIFO */

          gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, DCI_CTL_CAP, 0);

          /* Restart DMA from buffer start */

          gd32_dci_dma_config(priv);

          /* Re-enable capture; DCI will wait for next VSYNC */

          gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, 0, DCI_CTL_CAP);
        }
    }

  /* Embedded synchronous error interrupt */

  if (intf & DCI_INTF_ESEIF)
    {
      /* Clear the embedded synchronous error flag */

      gd32_dci_putreg(GD32_DCI_INTC_OFFSET, DCI_INTC_ESEFC);

      dcierr("DCI embedded sync error\n");
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_dci_data_init
 *
 * Description:
 *   Initialize the DCI data interface (imgdata_ops_s.init).
 *
 ****************************************************************************/

static int gd32_dci_data_init(FAR struct imgdata_s *data)
{
  FAR struct gd32_dci_s *priv = (FAR struct gd32_dci_s *)data;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize DCI hardware */

  gd32_dci_hw_init(priv);

  /* Allocate a DMA channel for DCI */

  priv->dma = gd32_dma_channel_alloc(GD32_DCI_DMA_REQ);
  if (priv->dma == NULL)
    {
      dcierr("Failed to allocate DMA channel for DCI\n");
      gd32_dci_hw_deinit(priv);
      nxmutex_unlock(&priv->lock);
      return -EBUSY;
    }

  /* Attach the DCI interrupt handler */

  ret = irq_attach(GD32_IRQ_DCI, gd32_dci_interrupt, priv);
  if (ret < 0)
    {
      dcierr("Failed to attach DCI IRQ\n");
      gd32_dma_channel_free(priv->dma);
      priv->dma = NULL;
      gd32_dci_hw_deinit(priv);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* Clear all interrupt flags */

  gd32_dci_putreg(GD32_DCI_INTC_OFFSET, DCI_INTC_ALLFC);

  /* Enable the DCI interrupt at the NVIC */

  up_enable_irq(GD32_IRQ_DCI);

  priv->capturing = false;
  priv->capture_cb = NULL;
  priv->capture_arg = NULL;

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: gd32_dci_data_uninit
 *
 * Description:
 *   Uninitialize the DCI data interface (imgdata_ops_s.uninit).
 *
 ****************************************************************************/

static int gd32_dci_data_uninit(FAR struct imgdata_s *data)
{
  FAR struct gd32_dci_s *priv = (FAR struct gd32_dci_s *)data;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Stop capture if in progress */

  if (priv->capturing)
    {
      /* Disable DCI capture and DCI */

      gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET,
                         DCI_CTL_CAP | DCI_CTL_DCIEN, 0);
      priv->capturing = false;
    }

  /* Disable the DCI interrupt at the NVIC */

  up_disable_irq(GD32_IRQ_DCI);

  /* Detach the DCI interrupt handler */

  irq_detach(GD32_IRQ_DCI);

  /* Deinitialize the hardware */

  gd32_dci_hw_deinit(priv);

  priv->capture_cb = NULL;
  priv->capture_arg = NULL;
  priv->fbuffer = NULL;
  priv->buf_size = 0;

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: gd32_dci_data_set_buf
 *
 * Description:
 *   Set the frame buffer for DCI capture (imgdata_ops_s.set_buf).
 *
 ****************************************************************************/

static int gd32_dci_data_set_buf(FAR struct imgdata_s *data,
                                 uint8_t nr_datafmts,
                                 FAR imgdata_format_t *datafmts,
                                 FAR uint8_t *addr, uint32_t size)
{
  FAR struct gd32_dci_s *priv = (FAR struct gd32_dci_s *)data;
  int ret;

  DEBUGASSERT(priv != NULL);

  if (addr == NULL || size == 0)
    {
      return -EINVAL;
    }

  /* DMA requires 4-byte alignment for 32-bit transfers */

  if (((uintptr_t)addr & 0x3) != 0)
    {
      dcierr("Buffer address must be 4-byte aligned\n");
      return -EINVAL;
    }

  if ((size & 0x3) != 0)
    {
      dcierr("Buffer size must be multiple of 4 bytes\n");
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->fbuffer = addr;
  priv->buf_size = size;

  /* If already capturing, pause capture, rearm DMA, then re-enable.
   * Stopping CAP prevents new pixels from filling the FIFO while
   * the DMA channel is being reprogrammed.
   */

  if (priv->capturing)
    {
      gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, DCI_CTL_CAP, 0);
      gd32_dci_dma_config(priv);
      gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, 0, DCI_CTL_CAP);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: gd32_dci_data_validate_frame_setting
 *
 * Description:
 *   Validate frame settings (imgdata_ops_s.validate_frame_setting).
 *
 ****************************************************************************/

static int
gd32_dci_data_validate_frame_setting(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval)
{
  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  /* Validate the pixel format. DCI supports raw data pass-through,
   * so the actual pixel format depends on the connected sensor.
   * We support the common formats:
   *   - UYVY / YUYV (16-bit per pixel)
   *   - RGB565 (16-bit per pixel)
   *   - JPEG (variable)
   *   - YUV420P
   *   - NV12
   */

  switch (datafmts[IMGDATA_FMT_MAIN].pixelformat)
    {
      case IMGDATA_PIX_FMT_UYVY:
      case IMGDATA_PIX_FMT_YUYV:
      case IMGDATA_PIX_FMT_RGB565:
      case IMGDATA_PIX_FMT_JPEG:
      case IMGDATA_PIX_FMT_YUV420P:
      case IMGDATA_PIX_FMT_NV12:
        break;

      default:
        return -EINVAL;
    }

  /* Validate resolution. DCI supports up to 14-bit data width,
   * resolution is limited by the sensor and available memory.
   */

  if (datafmts[IMGDATA_FMT_MAIN].width == 0 ||
      datafmts[IMGDATA_FMT_MAIN].height == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_dci_data_start_capture
 *
 * Description:
 *   Start frame capture (imgdata_ops_s.start_capture).
 *
 ****************************************************************************/

static int gd32_dci_data_start_capture(FAR struct imgdata_s *data,
                                       uint8_t nr_datafmts,
                                       FAR imgdata_format_t *datafmts,
                                       FAR imgdata_interval_t *interval,
                                       FAR imgdata_capture_t callback,
                                       FAR void *arg)
{
  FAR struct gd32_dci_s *priv = (FAR struct gd32_dci_s *)data;
  int ret;

  DEBUGASSERT(priv != NULL && callback != NULL);

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if already capturing */

  if (priv->capturing)
    {
      nxmutex_unlock(&priv->lock);
      return -EBUSY;
    }

  /* Store format parameters */

  priv->width = datafmts[IMGDATA_FMT_MAIN].width;
  priv->height = datafmts[IMGDATA_FMT_MAIN].height;
  priv->pixelformat = datafmts[IMGDATA_FMT_MAIN].pixelformat;

  /* Store callback */

  priv->capture_cb = callback;
  priv->capture_arg = arg;

  /* Configure DCI registers */

  gd32_dci_config(priv);

  /* Enable JPEG mode if needed */

  if (priv->pixelformat == IMGDATA_PIX_FMT_JPEG)
    {
      gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, 0, DCI_CTL_JM);
    }
  else
    {
      gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, DCI_CTL_JM, 0);
    }

  /* Configure DMA if buffer is already set */

  if (priv->fbuffer != NULL)
    {
      gd32_dci_dma_config(priv);
    }

  /* Clear all interrupt flags */

  gd32_dci_putreg(GD32_DCI_INTC_OFFSET, DCI_INTC_ALLFC);

  /* Enable end of frame and FIFO overrun interrupts */

  gd32_dci_putreg(GD32_DCI_INTEN_OFFSET,
                  DCI_INTEN_EFIE | DCI_INTEN_OVRIE | DCI_INTEN_ESEIE);

  /* Enable DCI */

  gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, 0, DCI_CTL_DCIEN);

  /* Enable capture */

  gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, 0, DCI_CTL_CAP);

  priv->capturing = true;
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: gd32_dci_data_stop_capture
 *
 * Description:
 *   Stop frame capture (imgdata_ops_s.stop_capture).
 *
 ****************************************************************************/

static int gd32_dci_data_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct gd32_dci_s *priv = (FAR struct gd32_dci_s *)data;
  int ret;

  DEBUGASSERT(priv != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->capturing)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Disable capture */

  gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, DCI_CTL_CAP, 0);

  /* Disable DCI */

  gd32_dci_modifyreg(GD32_DCI_CTL_OFFSET, DCI_CTL_DCIEN, 0);

  /* Disable DCI interrupts */

  gd32_dci_putreg(GD32_DCI_INTEN_OFFSET, 0);

  /* Stop DMA */

  if (priv->dma != NULL)
    {
      gd32_dma_stop(priv->dma);
    }

  /* Clear all interrupt flags */

  gd32_dci_putreg(GD32_DCI_INTC_OFFSET, DCI_INTC_ALLFC);

  priv->capturing = false;
  priv->capture_cb = NULL;
  priv->capture_arg = NULL;

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dci_initialize
 *
 * Description:
 *   Initialize the DCI driver and return a reference to the imgdata_s
 *   interface that can be used with the V4L2 capture framework.
 *
 * Returned Value:
 *   Upon successful, a reference to imgdata_s is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct imgdata_s *gd32_dci_initialize(void)
{
  FAR struct gd32_dci_s *priv = &g_dci_priv;

  return &priv->data;
}

/****************************************************************************
 * Name: gd32_dci_uninitialize
 *
 * Description:
 *   Uninitialize the DCI driver.
 *
 ****************************************************************************/

void gd32_dci_uninitialize(void)
{
  FAR struct gd32_dci_s *priv = &g_dci_priv;

  if (priv->capturing)
    {
      gd32_dci_data_stop_capture(&priv->data);
    }

  gd32_dci_data_uninit(&priv->data);
}

#endif /* CONFIG_GD32F4_DCI */
