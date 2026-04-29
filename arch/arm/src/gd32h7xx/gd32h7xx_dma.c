/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_dma.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "chip.h"
#include "gd32h7xx_dma.h"
#include "gd32h7xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_GD32H7_DMA

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_dma_channel_s
{
  bool           in_use;        /* Channel in use flag */
  bool           irq_attached;  /* IRQ has been attached */
  uint8_t        ctrl;          /* Controller: 0=DMA0, 1=DMA1 */
  uint8_t        chan;          /* Channel number: 0-7 */
  uint8_t        muxch;         /* DMAMUX channel: 0-15 */
  uint8_t        dmamux_reqid;  /* DMAMUX request ID */
  uint32_t       dma_base;      /* DMA base address */
  dma_callback_t callback;      /* Callback function */
  void          *arg;           /* Callback argument */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define GD32_DMA_NCHANNELS      8
#define GD32_DMA_TOTAL_CHANNELS 16

/* This array describes the state of each DMA channel */

static struct gd32_dma_channel_s g_dma_channels[GD32_DMA_TOTAL_CHANNELS];

/* INTF bit shift lookup table.
 *
 * H7 DMA INTF0/INTF1 register layout (6 bits per channel):
 *   Channel 0: bits [5:0],   shift = 0
 *   Channel 1: bits [11:6],  shift = 6
 *   Channel 2: bits [21:16], shift = 16
 *   Channel 3: bits [27:22], shift = 22
 */

static const uint8_t g_intf_shift[4] =
{
  0, 6, 16, 22
};

/* DMA IRQ lookup tables.
 * Note: DMA IRQs may not be contiguous on GD32H7xx.
 */

static const uint8_t g_dma0_irqs[GD32_DMA_NCHANNELS] =
{
  GD32_IRQ_DMA0_CH0, GD32_IRQ_DMA0_CH1, GD32_IRQ_DMA0_CH2,
  GD32_IRQ_DMA0_CH3, GD32_IRQ_DMA0_CH4, GD32_IRQ_DMA0_CH5,
  GD32_IRQ_DMA0_CH6, GD32_IRQ_DMA0_CH7
};

static const uint8_t g_dma1_irqs[GD32_DMA_NCHANNELS] =
{
  GD32_IRQ_DMA1_CH0, GD32_IRQ_DMA1_CH1, GD32_IRQ_DMA1_CH2,
  GD32_IRQ_DMA1_CH3, GD32_IRQ_DMA1_CH4, GD32_IRQ_DMA1_CH5,
  GD32_IRQ_DMA1_CH6, GD32_IRQ_DMA1_CH7
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gd32_dma_get_irq(struct gd32_dma_channel_s *dmach)
{
  if (dmach->ctrl == 0)
    {
      return g_dma0_irqs[dmach->chan];
    }
  else
    {
      return g_dma1_irqs[dmach->chan];
    }
}

static void gd32_channel_enable(uint32_t dmabase,
                                uint8_t channelx)
{
  modifyreg32(GD32_DMA_CHCTL(dmabase, channelx),
              0, DMA_CHXCTL_CHEN);
}

static void gd32_channel_disable(uint32_t dmabase,
                                 uint8_t channelx)
{
  modifyreg32(GD32_DMA_CHCTL(dmabase, channelx),
              DMA_CHXCTL_CHEN, 0);
}

static void gd32_channel_interrupt_enable(uint32_t dmabase,
                                          uint8_t channelx,
                                          uint32_t interrupt)
{
  modifyreg32(GD32_DMA_CHCTL(dmabase, channelx),
              DMA_INT_MASK, interrupt & DMA_INT_MASK);
}

static void gd32_dma_intflag_clear(uint32_t dmabase,
                                   uint8_t channelx,
                                   uint8_t flag)
{
  uint8_t shift;

  if (flag == 0)
    {
      return;
    }

  if (channelx < 4)
    {
      shift = g_intf_shift[channelx];
      putreg32((uint32_t)flag << shift,
               dmabase + GD32_DMA_INTC0_OFFSET);
    }
  else
    {
      shift = g_intf_shift[channelx - 4];
      putreg32((uint32_t)flag << shift,
               dmabase + GD32_DMA_INTC1_OFFSET);
    }
}

static uint8_t gd32_dma_intflag_get(uint32_t dmabase,
                                    uint8_t channelx)
{
  uint32_t regval;
  uint8_t shift;

  if (channelx < 4)
    {
      shift = g_intf_shift[channelx];
      regval = getreg32(dmabase + GD32_DMA_INTF0_OFFSET);
    }
  else
    {
      shift = g_intf_shift[channelx - 4];
      regval = getreg32(dmabase + GD32_DMA_INTF1_OFFSET);
    }

  return (regval >> shift) & DMA_INTF_MASK;
}

static void gd32_channel_int_disable(uint32_t dmabase,
                                     uint8_t channelx)
{
  modifyreg32(GD32_DMA_CHCTL(dmabase, channelx),
              DMA_INT_MASK, 0);
  gd32_dma_intflag_clear(dmabase, channelx,
                         DMA_INTF_FEEIF | DMA_INTF_SDEIF |
                         DMA_INTF_TAEIF | DMA_INTF_HTFIF |
                         DMA_INTF_FTFIF);
}

static int gd32_dma_interrupt(int irq, void *context,
                              void *arg)
{
  struct gd32_dma_channel_s *dmach = arg;
  uint8_t status;

  status = gd32_dma_intflag_get(dmach->dma_base,
                                dmach->chan);

  if (status != 0)
    {
      gd32_dma_intflag_clear(dmach->dma_base,
                             dmach->chan, status);
      if (dmach->callback != NULL)
        {
          dmach->callback((DMA_HANDLE)dmach, status,
                          dmach->arg);
        }
    }

  return OK;
}

static void gd32_dma_single_setup(
  struct gd32_dma_channel_s *dmach,
  const struct gd32_dma_config_s *config)
{
  uint32_t regval;
  uint32_t chctl;
  uint32_t chcnt;
  uint32_t chpaddr;
  uint32_t chm0addr;
  uint32_t chfctl;
  uint32_t muxcfg;

  chctl = GD32_DMA_CHCTL(dmach->dma_base,
                         dmach->chan);
  chcnt = GD32_DMA_CHCNT(dmach->dma_base,
                         dmach->chan);
  chpaddr = GD32_DMA_CHPADDR(dmach->dma_base,
                              dmach->chan);
  chm0addr = GD32_DMA_CHM0ADDR(dmach->dma_base,
                                dmach->chan);
  chfctl = GD32_DMA_CHFCTL(dmach->dma_base,
                            dmach->chan);
  muxcfg = GD32_DMAMUX_RM_CHCFG(GD32_DMAMUX0_BASE,
                                  dmach->muxch);

  gd32_channel_disable(dmach->dma_base, dmach->chan);
  gd32_dma_intflag_clear(dmach->dma_base, dmach->chan,
                         DMA_INTF_FEEIF | DMA_INTF_SDEIF |
                         DMA_INTF_TAEIF | DMA_INTF_HTFIF |
                         DMA_INTF_FTFIF);

  /* Fully reset CHCTL like official dma_deinit() */

  putreg32(0, chctl);

  /* Reset CHFCTL to default value (0x21) */

  putreg32(0x00000021, chfctl);

  putreg32(config->periph_addr, chpaddr);
  putreg32(config->memory_addr, chm0addr);
  putreg32(config->number, chcnt);

  /* Build CHCTL: direction, width, priority are
   * already register bit values.
   */

  regval = config->direction;
  regval |= config->periph_width;
  regval |= config->memory_width;
  regval |= config->priority;

  /* Set peripheral increment mode */

  if (config->periph_inc)
    {
      regval |= DMA_CHXCTL_PNAGA;
    }

  /* Set memory increment mode */

  if (config->memory_inc)
    {
      regval |= DMA_CHXCTL_MNAGA;
    }

  /* Set circular mode */

  if (config->circular)
    {
      regval |= DMA_CHXCTL_CMEN;
    }

  putreg32(regval, chctl);
  putreg32(dmach->dmamux_reqid & DMAMUX_RM_CHXCFG_MUXID_MASK,
           muxcfg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  int i;

  /* Enable DMA0, DMA1 and DMAMUX clocks */

  modifyreg32(GD32_RCU_AHB1EN, 0,
              RCU_AHB1EN_DMA0EN | RCU_AHB1EN_DMA1EN | RCU_AHB1EN_DMAMUXEN);

  /* Initialize each DMA channel */

  for (i = 0; i < GD32_DMA_TOTAL_CHANNELS; i++)
    {
      g_dma_channels[i].in_use = false;
      g_dma_channels[i].ctrl = i / GD32_DMA_NCHANNELS;
      g_dma_channels[i].chan = i % GD32_DMA_NCHANNELS;
      g_dma_channels[i].muxch = i;
      g_dma_channels[i].dma_base =
        (i < GD32_DMA_NCHANNELS) ? GD32_DMA0_BASE : GD32_DMA1_BASE;
      g_dma_channels[i].irq_attached = false;
      g_dma_channels[i].callback = NULL;
      g_dma_channels[i].arg = NULL;

      /* Disable channel and interrupts */

      gd32_channel_int_disable(g_dma_channels[i].dma_base,
                               g_dma_channels[i].chan);
      gd32_channel_disable(g_dma_channels[i].dma_base,
                           g_dma_channels[i].chan);
    }
}

DMA_HANDLE gd32_dma_channel_alloc(uint8_t dmamux_reqid)
{
  struct gd32_dma_channel_s *dmach;
  irqstate_t flags;
  int i;

  flags = enter_critical_section();

  /* Search for a free channel */

  for (i = 0; i < GD32_DMA_TOTAL_CHANNELS; i++)
    {
      dmach = &g_dma_channels[i];
      if (!dmach->in_use)
        {
          dmach->in_use = true;
          dmach->irq_attached = false;
          dmach->dmamux_reqid = dmamux_reqid;
          dmach->callback = NULL;
          dmach->arg = NULL;

          leave_critical_section(flags);
          return (DMA_HANDLE)dmach;
        }
    }

  leave_critical_section(flags);
  return NULL;  /* No free channel */
}

void gd32_dma_channel_free(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmach =
    (struct gd32_dma_channel_s *)handle;
  irqstate_t flags;
  int irq;

  /* Stop the channel */

  gd32_dma_stop(handle);

  /* Detach and disable interrupt */

  irq = gd32_dma_get_irq(dmach);
  up_disable_irq(irq);
  irq_detach(irq);

  /* Clear DMAMUX configuration */

  putreg32(0, GD32_DMAMUX_RM_CHCFG(GD32_DMAMUX0_BASE,
                                     dmach->muxch));

  flags = enter_critical_section();
  dmach->in_use = false;
  dmach->irq_attached = false;
  dmach->callback = NULL;
  dmach->arg = NULL;
  leave_critical_section(flags);
}

void gd32_dma_setup(DMA_HANDLE handle,
                    const struct gd32_dma_config_s *config)
{
  struct gd32_dma_channel_s *dmach =
    (struct gd32_dma_channel_s *)handle;

  gd32_dma_single_setup(dmach, config);
}

void gd32_dma_start(DMA_HANDLE handle,
                    dma_callback_t callback,
                    void *arg, uint32_t interrupt)
{
  struct gd32_dma_channel_s *dmach =
    (struct gd32_dma_channel_s *)handle;
  int irq;

  dmach->callback = callback;
  dmach->arg = arg;

  /* Clear any pending interrupts before enabling */

  gd32_dma_intflag_clear(dmach->dma_base,
                         dmach->chan,
                         DMA_INTF_FEEIF |
                         DMA_INTF_SDEIF |
                         DMA_INTF_TAEIF |
                         DMA_INTF_HTFIF |
                         DMA_INTF_FTFIF);

  /* Attach and enable interrupt */

  irq = gd32_dma_get_irq(dmach);
  if (!dmach->irq_attached)
    {
      irq_attach(irq, gd32_dma_interrupt, dmach);
      up_enable_irq(irq);
      dmach->irq_attached = true;
    }

  /* Enable DMA interrupts if requested */

  if (interrupt != 0)
    {
      gd32_channel_interrupt_enable(dmach->dma_base,
                                    dmach->chan,
                                    interrupt &
                                    DMA_INT_MASK);
    }

  gd32_channel_enable(dmach->dma_base, dmach->chan);
}

void gd32_dma_stop(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmach =
    (struct gd32_dma_channel_s *)handle;

  gd32_channel_int_disable(dmach->dma_base,
                           dmach->chan);
  gd32_channel_disable(dmach->dma_base,
                       dmach->chan);
}

size_t gd32_dma_transnum_get(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmach =
    (struct gd32_dma_channel_s *)handle;

  return (size_t)getreg32(
    GD32_DMA_CHCNT(dmach->dma_base, dmach->chan));
}

size_t gd32_dma_residual(DMA_HANDLE handle)
{
  return gd32_dma_transnum_get(handle);
}

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_sample(DMA_HANDLE handle,
                     struct gd32_dmaregs_s *regs)
{
  struct gd32_dma_channel_s *dmach =
    (struct gd32_dma_channel_s *)handle;
  irqstate_t flags;
  uint32_t base;
  uint8_t ch;

  base = dmach->dma_base;
  ch = dmach->chan;

  flags = enter_critical_section();
  regs->intf0 = getreg32(base + GD32_DMA_INTF0_OFFSET);
  regs->intf1 = getreg32(base + GD32_DMA_INTF1_OFFSET);
  regs->chctl = getreg32(GD32_DMA_CHCTL(base, ch));
  regs->chcnt = getreg32(GD32_DMA_CHCNT(base, ch));
  regs->chpaddr = getreg32(GD32_DMA_CHPADDR(base, ch));
  regs->chm0addr = getreg32(GD32_DMA_CHM0ADDR(base, ch));
  regs->chm1addr = getreg32(GD32_DMA_CHM1ADDR(base, ch));
  regs->chfctl = getreg32(GD32_DMA_CHFCTL(base, ch));
  leave_critical_section(flags);
}

void gd32_dma_dump(DMA_HANDLE handle,
                   const struct gd32_dmaregs_s *regs,
                   const char *msg)
{
  UNUSED(handle);
  dmainfo("DMA Registers: %s\n", msg);
  dmainfo("   INTF0: %08" PRIx32 "\n", regs->intf0);
  dmainfo("   INTF1: %08" PRIx32 "\n", regs->intf1);
  dmainfo("   CHCTL: %08" PRIx32 "\n", regs->chctl);
  dmainfo("   CHCNT: %08" PRIx32 "\n", regs->chcnt);
  dmainfo(" CHPADDR: %08" PRIx32 "\n", regs->chpaddr);
  dmainfo("CHM0ADDR: %08" PRIx32 "\n", regs->chm0addr);
  dmainfo("CHM1ADDR: %08" PRIx32 "\n", regs->chm1addr);
  dmainfo("  CHFCTL: %08" PRIx32 "\n", regs->chfctl);
}
#endif

#endif /* CONFIG_GD32H7_DMA */
