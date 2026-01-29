/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_dma.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "sched/sched.h"
#include "chip.h"
#include "gd32e11x_dma.h"
#include "gd32e11x.h"
#include "gd32e11x_rcu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_GD32E11X_DMA0) || defined(CONFIG_GD32E11X_DMA1)

#define DMA0_NCHANNELS      (7)
#define DMA1_NCHANNELS      (5)
#define DMA_NCHANNELS       (DMA0_NCHANNELS + DMA1_NCHANNELS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct gd32_dma_channel_s
{
  uint8_t        chan_num;         /* DMA channel number (0-6 for DMA0, 0-4 for DMA1) */
  uint8_t        irq;              /* DMA channel IRQ number */
  sem_t          chsem;            /* Used to wait for DMA channel to become available */
  uint32_t       dmabase;          /* DMA base address */
  uint32_t       transfer_count;   /* Transfer count (saved from setup) */
  dma_callback_t callback;         /* Callback invoked when the DMA completes */
  void           *arg;             /* Argument passed to callback function */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t gd32_dma_interrupt_flag_get(uint32_t dma_periph,
                                           uint8_t channelx);
static void gd32_dma_interrupt_flag_clear(uint32_t dma_periph,
                                          uint8_t channelx,
                                          uint8_t flag);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA */

static struct gd32_dma_channel_s g_dmachan[DMA_NCHANNELS] =
{
#ifdef CONFIG_GD32E11X_DMA0
  /* DMA0 Channel 0 */

  {
    .chan_num  = 0,
    .irq       = GD32_IRQ_DMA0_CHANNEL0,
    .dmabase   = GD32_DMA0,
  },

  /* DMA0 Channel 1 */

  {
    .chan_num  = 1,
    .irq       = GD32_IRQ_DMA0_CHANNEL1,
    .dmabase   = GD32_DMA0,
  },

  /* DMA0 Channel 2 */

  {
    .chan_num  = 2,
    .irq       = GD32_IRQ_DMA0_CHANNEL2,
    .dmabase   = GD32_DMA0,
  },

  /* DMA0 Channel 3 */

  {
    .chan_num  = 3,
    .irq       = GD32_IRQ_DMA0_CHANNEL3,
    .dmabase   = GD32_DMA0,
  },

  /* DMA0 Channel 4 */

  {
    .chan_num  = 4,
    .irq       = GD32_IRQ_DMA0_CHANNEL4,
    .dmabase   = GD32_DMA0,
  },

  /* DMA0 Channel 5 */

  {
    .chan_num  = 5,
    .irq       = GD32_IRQ_DMA0_CHANNEL5,
    .dmabase   = GD32_DMA0,
  },

  /* DMA0 Channel 6 */

  {
    .chan_num  = 6,
    .irq       = GD32_IRQ_DMA0_CHANNEL6,
    .dmabase   = GD32_DMA0,
  },
#endif

#ifdef CONFIG_GD32E11X_DMA1
  /* DMA1 Channel 0 */

  {
    .chan_num  = 0,
    .irq       = GD32_IRQ_DMA1_CHANNEL0,
    .dmabase   = GD32_DMA1,
  },

  /* DMA1 Channel 1 */

  {
    .chan_num  = 1,
    .irq       = GD32_IRQ_DMA1_CHANNEL1,
    .dmabase   = GD32_DMA1,
  },

  /* DMA1 Channel 2 */

  {
    .chan_num  = 2,
    .irq       = GD32_IRQ_DMA1_CHANNEL2,
    .dmabase   = GD32_DMA1,
  },

  /* DMA1 Channel 3 */

  {
    .chan_num  = 3,
    .irq       = GD32_IRQ_DMA1_CHANNEL3,
    .dmabase   = GD32_DMA1,
  },

  /* DMA1 Channel 4 */

  {
    .chan_num  = 4,
    .irq       = GD32_IRQ_DMA1_CHANNEL4,
    .dmabase   = GD32_DMA1,
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dma_clock_enable
 *
 * Description:
 *   Enable clock for the DMA controller
 *
 ****************************************************************************/

static void gd32_dma_clock_enable(uint32_t dmabase)
{
#ifdef CONFIG_GD32E11X_DMA0
  if (dmabase == GD32_DMA0)
    {
      gd32_rcu_periph_clock_enable(RCU_AHBEN_DMA0EN);
    }
#endif

#ifdef CONFIG_GD32E11X_DMA1
  if (dmabase == GD32_DMA1)
    {
      gd32_rcu_periph_clock_enable(RCU_AHBEN_DMA1EN);
    }
#endif
}

/****************************************************************************
 * Name: gd32_channel_enable
 *
 * Description:
 *   Enable DMA channel
 *
 ****************************************************************************/

static void gd32_channel_enable(uint32_t dma_periph, uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);
  regval = getreg32(regaddr);
  regval |= DMA_CHXCTL_CHEN;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_disable
 *
 * Description:
 *   Disable DMA channel
 *
 ****************************************************************************/

static void gd32_channel_disable(uint32_t dma_periph, uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);
  regval = getreg32(regaddr);
  regval &= ~DMA_CHXCTL_CHEN;
  putreg32(regval, regaddr);

  /* Wait until channel is disabled */

  while ((getreg32(regaddr) & DMA_CHXCTL_CHEN) != 0)
    {
    }
}

/****************************************************************************
 * Name: gd32_channel_interrupt_enable
 *
 * Description:
 *   Enable DMA channel interrupt
 *
 ****************************************************************************/

static void gd32_channel_interrupt_enable(uint32_t dma_periph,
                                          uint8_t channelx,
                                          uint32_t interrupt)
{
  uint32_t regaddr;
  uint32_t regval;

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);
  regval = getreg32(regaddr);
  regval |= interrupt;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_interrupt_disable
 *
 * Description:
 *   Disable DMA channel interrupt
 *
 ****************************************************************************/

static void gd32_channel_interrupt_disable(uint32_t dma_periph,
                                           uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);
  regval = getreg32(regaddr);
  regval &= ~DMA_INT_MASK;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_dma_interrupt_flag_get
 *
 * Description:
 *   Get DMA interrupt flag
 *
 ****************************************************************************/

static uint8_t gd32_dma_interrupt_flag_get(uint32_t dma_periph,
                                           uint8_t channelx)
{
  uint32_t regval;
  uint8_t flag = 0;
  uint8_t shift;

  /* Calculate bit shift for this channel (4 bits per channel) */

  shift = channelx * 4;

  regval = getreg32(GD32_DMA_INTF(dma_periph));
  flag = (regval >> shift) & DMA_INTF_MASK;

  return flag;
}

/****************************************************************************
 * Name: gd32_dma_interrupt_flag_clear
 *
 * Description:
 *   Clear DMA interrupt flag
 *
 ****************************************************************************/

static void gd32_dma_interrupt_flag_clear(uint32_t dma_periph,
                                          uint8_t channelx,
                                          uint8_t flag)
{
  uint8_t shift;
  uint32_t regval;

  /* Calculate bit shift for this channel (4 bits per channel) */

  shift = channelx * 4;

  regval = (uint32_t)flag << shift;
  putreg32(regval, GD32_DMA_INTC(dma_periph));
}

/****************************************************************************
 * Name: gd32_dma_interrupt
 *
 * Description:
 *   DMA interrupt handler
 *
 ****************************************************************************/

static int gd32_dma_interrupt(int irq, void *context, void *arg)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)arg;
  uint8_t chan_isr;
  uint16_t status = 0;

  DEBUGASSERT(dmachan != NULL);

  /* Get interrupt status for this channel */

  chan_isr = gd32_dma_interrupt_flag_get(dmachan->dmabase,
                                         dmachan->chan_num);

  /* Clear all pending interrupts */

  gd32_dma_interrupt_flag_clear(dmachan->dmabase, dmachan->chan_num,
                                chan_isr);

  /* Build status word */

  if (chan_isr & DMA_INTF_ERRIF)
    {
      status |= DMA_STATUS_ERROR;
    }

  if (chan_isr & (DMA_INTF_HTFIF | DMA_INTF_FTFIF))
    {
      status |= DMA_STATUS_SUCCESS;
    }

  /* Invoke the callback */

  if (dmachan->callback)
    {
      dmachan->callback((DMA_HANDLE)dmachan, status, dmachan->arg);
    }
  else
    {
      dmainfo("DMA CH%d NO CALLBACK!\n", dmachan->chan_num);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  int i;

#ifdef CONFIG_GD32E11X_DMA0
  modifyreg32(GD32_RCU_AHBEN, 0, RCU_AHBEN_DMA0EN);
#endif

#ifdef CONFIG_GD32E11X_DMA1
  modifyreg32(GD32_RCU_AHBEN, 0, RCU_AHBEN_DMA1EN);
#endif

  /* Initialize each DMA channel */

  for (i = 0; i < DMA_NCHANNELS; i++)
    {
      nxsem_init(&g_dmachan[i].chsem, 0, 1);

      /* Attach DMA interrupt vectors */

      irq_attach(g_dmachan[i].irq, gd32_dma_interrupt, &g_dmachan[i]);

      /* Enable the IRQ at the NVIC */

      up_enable_irq(g_dmachan[i].irq);
    }
}

/****************************************************************************
 * Name: gd32_dma_channel_alloc
 *
 * Description:
 *   Allocate a DMA channel. This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'periph_req'
 *   argument.
 *
 *   If the DMA channel is not available, then gd32_dma_channel_alloc()
 *   will wait until the holder of the channel relinquishes the channel
 *   by calling gd32_dma_channel_free().
 *
 * Input Parameters:
 *   periph_req - Identifies the DMA channel request
 *
 * Returned Value:
 *   If periph_req is valid, this function ALWAYS returns a non-NULL
 *   void* DMA channel handle.
 *
 ****************************************************************************/

DMA_HANDLE gd32_dma_channel_alloc(uint8_t periph_req)
{
  struct gd32_dma_channel_s *dmachan = NULL;
  int chan_idx = -1;

  /* Find the channel index based on peripheral request
   * periph_req format: bits [7:4] = DMA controller (0=DMA0, 1=DMA1)
   *                    bits [3:0] = Channel number
   */

  uint8_t dma_id = (periph_req >> 4) & 0x0f;
  uint8_t ch_num = periph_req & 0x0f;

#ifdef CONFIG_GD32E11X_DMA0
  if (dma_id == 0 && ch_num < DMA0_NCHANNELS)
    {
      chan_idx = ch_num;
    }
#endif

#ifdef CONFIG_GD32E11X_DMA1
  if (dma_id == 1 && ch_num < DMA1_NCHANNELS)
    {
#ifdef CONFIG_GD32E11X_DMA0
      chan_idx = DMA0_NCHANNELS + ch_num;
#else
      chan_idx = ch_num;
#endif
    }
#endif

  DEBUGASSERT(chan_idx >= 0 && chan_idx < DMA_NCHANNELS);

  dmachan = &g_dmachan[chan_idx];

  /* Get exclusive access to the DMA channel */

  nxsem_wait_uninterruptible(&dmachan->chsem);

  /* Enable clock for this DMA controller */

  gd32_dma_clock_enable(dmachan->dmabase);

  return (DMA_HANDLE)dmachan;
}

/****************************************************************************
 * Name: gd32_dma_channel_free
 *
 * Description:
 *   Release a DMA channel. If another thread is waiting for this DMA
 *   channel in a call to gd32_dma_channel_alloc, then this function will
 *   re-assign the DMA channel to that thread and wake it up.
 *
 *   NOTE: The 'handle' used in this argument must NEVER be used again
 *   until gd32_dma_channel_alloc() is called again to re-gain access to
 *   the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void gd32_dma_channel_free(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;

  DEBUGASSERT(dmachan != NULL);

  /* Disable the channel and interrupts */

  gd32_channel_disable(dmachan->dmabase, dmachan->chan_num);
  gd32_channel_interrupt_disable(dmachan->dmabase, dmachan->chan_num);

  /* Release the channel */

  nxsem_post(&dmachan->chsem);
}

/****************************************************************************
 * Name: gd32_dma_setup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void gd32_dma_setup(DMA_HANDLE handle, void *arg, uint8_t data_mode)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;
  dma_parameter_struct *dma_init = (dma_parameter_struct *)arg;
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(dmachan != NULL && dma_init != NULL);

  /* Disable the channel before configuration */

  gd32_channel_disable(dmachan->dmabase, dmachan->chan_num);

  /* Configure peripheral address */

  regaddr = GD32_DMA_CHPADDR(dmachan->dmabase, dmachan->chan_num);
  putreg32(dma_init->periph_addr, regaddr);

  /* Configure memory address */

  regaddr = GD32_DMA_CHMADDR(dmachan->dmabase, dmachan->chan_num);
  putreg32(dma_init->memory_addr, regaddr);

  /* Configure transfer number and save it for later use
   * Note: Some DMA hardware may clear CNT when channel is disabled,
   * so we save it here and re-write in gd32_dma_start if needed.
   */

  dmachan->transfer_count = dma_init->number & DMA_CHXCNT_CNT_MASK;
  regaddr = GD32_DMA_CHCNT(dmachan->dmabase, dmachan->chan_num);
  putreg32(dmachan->transfer_count, regaddr);

  /* Configure channel control register */

  regaddr = GD32_DMA_CHCTL(dmachan->dmabase, dmachan->chan_num);
  regval = getreg32(regaddr);

  /* Clear configuration bits */

  regval &= ~(DMA_CHXCTL_PWIDTH_MASK | DMA_CHXCTL_MWIDTH_MASK |
              DMA_CHXCTL_PRIO_MASK | DMA_CHXCTL_M2M |
              DMA_CHXCTL_MNAGA | DMA_CHXCTL_PNAGA |
              DMA_CHXCTL_CMEN | DMA_CHXCTL_DIR);

  /* Set transfer direction */

  regval |= dma_init->direction;

  /* Set peripheral width */

  regval |= dma_init->periph_width;

  /* Set memory width */

  regval |= dma_init->memory_width;

  /* Set priority */

  regval |= dma_init->priority;

  /* Set peripheral increment mode
   * Note: DMA_PERIPH_INCREASE_ENABLE = 0, DMA_PERIPH_INCREASE_DISABLE = 1
   * Only set PNAGA bit when ENABLE (0), which means increment
   */

  if (dma_init->periph_inc == DMA_PERIPH_INCREASE_ENABLE)
    {
      regval |= DMA_CHXCTL_PNAGA;
    }

  /* Set memory increment mode
   * Note: DMA_MEMORY_INCREASE_ENABLE = 0, DMA_MEMORY_INCREASE_DISABLE = 1
   * Only set MNAGA bit when ENABLE (0), which means increment
   */

  if (dma_init->memory_inc == DMA_MEMORY_INCREASE_ENABLE)
    {
      regval |= DMA_CHXCTL_MNAGA;
    }

  putreg32(regval, regaddr);

  dmainfo("DMA CH%d configured\n", dmachan->chan_num);
}

/****************************************************************************
 * Name: gd32_dma_start
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *   - No DMA in progress
 *
 ****************************************************************************/

void gd32_dma_start(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    uint32_t interrupt)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;
  uint32_t regaddr;
  uint32_t cnt;

  DEBUGASSERT(dmachan != NULL);

  /* Read current counter value and restore if it was cleared
   * Note: On some DMA hardware, CNT may be cleared when channel is disabled.
   * We detect this and restore the value saved during gd32_dma_setup.
   */

  regaddr = GD32_DMA_CHCNT(dmachan->dmabase, dmachan->chan_num);
  cnt = getreg32(regaddr) & DMA_CHXCNT_CNT_MASK;

  if (cnt == 0 && dmachan->transfer_count != 0)
    {
      cnt = dmachan->transfer_count;
    }

  /* Save the callback info */

  dmachan->callback = callback;
  dmachan->arg = arg;

  /* Clear any pending interrupts */

  gd32_dma_interrupt_flag_clear(dmachan->dmabase, dmachan->chan_num,
                                DMA_INTF_MASK);

  /* Enable interrupts */

  if (interrupt != 0)
    {
      gd32_channel_interrupt_enable(dmachan->dmabase, dmachan->chan_num,
                                    interrupt);
    }

  putreg32(cnt, regaddr);

  /* Enable the DMA channel */

  gd32_channel_enable(dmachan->dmabase, dmachan->chan_num);
}

/****************************************************************************
 * Name: gd32_dma_stop
 *
 * Description:
 *   Cancel the DMA. After gd32_dma_stop() is called, the DMA channel is
 *   reset and gd32_dma_setup() must be called before gd32_dma_start()
 *   can be called again.
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

void gd32_dma_stop(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;

  DEBUGASSERT(dmachan != NULL);

  /* Disable the channel */

  gd32_channel_disable(dmachan->dmabase, dmachan->chan_num);

  /* Disable interrupts */

  gd32_channel_interrupt_disable(dmachan->dmabase, dmachan->chan_num);
}

/****************************************************************************
 * Name: gd32_dma_tansnum_get
 *
 * Description:
 *   Get the number of remaining data to be transferred by the DMA
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

size_t gd32_dma_tansnum_get(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;
  uint32_t regaddr;

  DEBUGASSERT(dmachan != NULL);

  regaddr = GD32_DMA_CHCNT(dmachan->dmabase, dmachan->chan_num);
  return (size_t)(getreg32(regaddr) & DMA_CHXCNT_CNT_MASK);
}

/****************************************************************************
 * Name: gd32_dma_sample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_sample(DMA_HANDLE handle, struct gd32_dmaregs_s *regs)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;

  DEBUGASSERT(dmachan != NULL && regs != NULL);

  regs->intf = getreg32(GD32_DMA_INTF(dmachan->dmabase));
  regs->chctl = getreg32(GD32_DMA_CHCTL(dmachan->dmabase,
                                        dmachan->chan_num));
  regs->chcnt = getreg32(GD32_DMA_CHCNT(dmachan->dmabase,
                                        dmachan->chan_num));
  regs->chpaddr = getreg32(GD32_DMA_CHPADDR(dmachan->dmabase,
                                            dmachan->chan_num));
  regs->chmaddr = getreg32(GD32_DMA_CHMADDR(dmachan->dmabase,
                                            dmachan->chan_num));
}
#endif

/****************************************************************************
 * Name: gd32_dma_dump
 *
 * Description:
 *   Dump DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_dump(DMA_HANDLE handle, const struct gd32_dmaregs_s *regs,
                   const char *msg)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;
  DEBUGASSERT(dmachan != NULL && regs != NULL);
}
#endif

#endif /* CONFIG_GD32E11X_DMA0 || CONFIG_GD32E11X_DMA1 */
