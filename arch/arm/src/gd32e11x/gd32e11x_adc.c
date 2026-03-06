/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_adc.c
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

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "gd32e11x.h"
#include "gd32e11x_adc.h"

#ifdef ADC_HAVE_DMA
#  include "gd32e11x_dma.h"
#endif

/* GD32E11x ADC lower-half driver.
 *
 * The GD32E11x ADC has the following features:
 *   - 2 independent ADC units (ADC0, ADC1)
 *   - No common ADC registers (each ADC is standalone)
 *   - 18 channels per ADC (0..17)
 *   - Software or external trigger for regular/inserted conversions
 *   - Scan mode, continuous mode, discontinuous mode
 *   - DMA support on ADC0 only (DMA0 Channel 0)
 *   - Analog watchdog
 *   - Temperature sensor and Vrefint on ADC0 ch16/ch17
 *   - Calibration via RSTCLB + CLB bits
 *   - Configurable resolution (12/10/8/6-bit) via OVSAMPCTL
 *   - Oversampling with configurable ratio and shift
 */

#ifdef CONFIG_GD32E11X_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default sample time */

#define ADC_SMPR_DEFAULT             ADC_SMPR_55p5
#define ADC_SMPR1_DEFAULT            ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP10_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP11_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP12_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP13_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP14_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP15_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP16_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP17_SHIFT))
#define ADC_SMPR2_DEFAULT            ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP0_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP1_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP2_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP3_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP4_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP5_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP6_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP7_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP8_SHIFT) | \
                                      (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP9_SHIFT))

/* Interrupt status/enable register offsets for generalized access */

#define GD32_ADC_ISR_OFFSET          GD32_ADC_SR_OFFSET
#define GD32_ADC_IER_OFFSET          GD32_ADC_CR1_OFFSET

/* ISR flags */

#define ADC_ISR_AWD                  ADC_SR_AWD
#define ADC_ISR_EOC                  ADC_SR_EOC
#define ADC_ISR_JEOC                 ADC_SR_JEOC
#define ADC_ISR_OVR                  0            /* No overrun flag on basic ADC */
#define ADC_ISR_ALLINTS              (ADC_ISR_AWD | ADC_ISR_EOC | ADC_ISR_JEOC)

/* IER bits (in CR1) */

#define ADC_IER_AWD                  ADC_CR1_AWDIE
#define ADC_IER_EOC                  ADC_CR1_EOCIE
#define ADC_IER_JEOC                 ADC_CR1_JEOCIE
#define ADC_IER_OVR                  0
#define ADC_IER_ALLINTS              (ADC_IER_AWD | ADC_IER_EOC | ADC_IER_JEOC)

/* ADC ANIOC trigger configuration */

#define ANIOC_TRIGGER_REGULAR        (1 << 0)
#define ANIOC_TRIGGER_INJECTED       (1 << 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one ADC block */

struct gd32_dev_s
{
  const struct adc_callback_s *cb;
  uint8_t irq;               /* Interrupt generated by this ADC block */
  uint8_t rnchannels;        /* Number of regular channels */
  uint8_t cr_channels;       /* Number of configured regular channels */
  uint8_t intf;              /* ADC interface number (0 or 1) */
  uint8_t initialized;       /* ADC interface initialization counter */
  uint8_t current;           /* Current ADC channel being converted */
  uint8_t anioc_trg;         /* ANIOC_TRIGGER configuration */
#ifdef ADC_HAVE_DMA
  bool    hasdma;            /* True: This channel supports DMA */
  uint8_t dmachan;           /* DMA channel needed by this ADC */
#endif
#ifdef ADC_HAVE_SCAN
  bool    scan;              /* True: Scan mode */
#endif
  xcpt_t   isr;              /* Interrupt handler for this ADC block */
  uint32_t base;             /* Base address of registers unique to this ADC */
  uint32_t extsel;           /* External event configuration for regular group */
#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;            /* Allocated DMA channel */
  uint16_t *r_dmabuffer;     /* DMA transfer buffer */
#endif

  /* List of selected ADC channels to sample */

  uint8_t  r_chanlist[CONFIG_GD32E11X_ADC_MAX_SAMPLES];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

static uint32_t adc_getreg(struct gd32_dev_s *priv, int offset);
static void     adc_putreg(struct gd32_dev_s *priv, int offset,
                            uint32_t value);
static void     adc_modifyreg(struct gd32_dev_s *priv, int offset,
                              uint32_t clrbits, uint32_t setbits);

/* ADC helpers */

static void     adc_rcu_reset(struct gd32_dev_s *priv, bool reset);
static void     adc_enable(struct gd32_dev_s *priv, bool enable);
static void     adc_calibrate(struct gd32_dev_s *priv);
static void     adc_watchdog_cfg(struct gd32_dev_s *priv);
static void     adc_sampletime_cfg(struct gd32_dev_s *priv);
static void     adc_mode_cfg(struct gd32_dev_s *priv);
static uint32_t adc_sqrbits(struct gd32_dev_s *priv, int first,
                            int last, int offset);
static int      adc_set_ch(struct adc_dev_s *dev, uint8_t ch);
static void     adc_reg_startconv(struct gd32_dev_s *priv, bool enable);
static void     adc_configure(struct adc_dev_s *dev);
static void     adc_dumpregs(struct gd32_dev_s *priv);

/* ADC Interrupt Handler */

static int      adc_interrupt(struct adc_dev_s *dev);
static int      adc01_interrupt(int irq, void *context, void *arg);

/* DMA support */

#ifdef ADC_HAVE_DMA
static void     adc_dmaconvcallback(DMA_HANDLE handle, uint16_t status,
                                    void *arg);
static void     adc_dma_cfg(struct gd32_dev_s *priv);
static void     adc_dma_start(struct adc_dev_s *dev);
#endif

/* ADC Driver Methods */

static int      adc_bind(struct adc_dev_s *dev,
                         const struct adc_callback_s *callback);
static void     adc_reset(struct adc_dev_s *dev);
static int      adc_setup(struct adc_dev_s *dev);
static void     adc_shutdown(struct adc_dev_s *dev);
static void     adc_rxint(struct adc_dev_s *dev, bool enable);
static int      adc_ioctl(struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = adc_bind,
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

/* ADC0 state */

#ifdef CONFIG_GD32E11X_ADC0

#ifdef ADC0_HAVE_DMA
static uint16_t g_adc0_dmabuffer[CONFIG_GD32E11X_ADC_MAX_SAMPLES];
#endif

static struct gd32_dev_s g_adcpriv0 =
{
  .irq         = GD32_IRQ_ADC0_1,
  .isr         = adc01_interrupt,
  .intf        = 0,
  .initialized = 0,
  .anioc_trg   = ANIOC_TRIGGER_REGULAR,
  .base        = GD32_ADC0_BASE,
  .extsel      = ADC0_EXTSEL_VALUE,
#ifdef ADC0_HAVE_DMA
  .dmachan     = ADC0_DMA_CHAN,
  .hasdma      = true,
  .r_dmabuffer = g_adc0_dmabuffer,
#endif
#ifdef ADC_HAVE_SCAN
  .scan        = CONFIG_GD32E11X_ADC0_SCAN,
#endif
};

static struct adc_dev_s g_adcdev0 =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adcpriv0,
};
#endif /* CONFIG_GD32E11X_ADC0 */

/* ADC1 state */

#ifdef CONFIG_GD32E11X_ADC1

static struct gd32_dev_s g_adcpriv1 =
{
  .irq         = GD32_IRQ_ADC0_1,
  .isr         = adc01_interrupt,
  .intf        = 1,
  .initialized = 0,
  .anioc_trg   = ANIOC_TRIGGER_REGULAR,
  .base        = GD32_ADC1_BASE,
  .extsel      = ADC1_EXTSEL_VALUE,
#ifdef ADC_HAVE_SCAN
  .scan        = CONFIG_GD32E11X_ADC1_SCAN,
#endif
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adcpriv1,
};
#endif /* CONFIG_GD32E11X_ADC1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_getreg
 *
 * Description:
 *   Read the value of an ADC register.
 *
 ****************************************************************************/

static uint32_t adc_getreg(struct gd32_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: adc_putreg
 *
 * Description:
 *   Write a value to an ADC register.
 *
 ****************************************************************************/

static void adc_putreg(struct gd32_dev_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: adc_modifyreg
 *
 * Description:
 *   Modify the value of an ADC register (not atomic).
 *
 ****************************************************************************/

static void adc_modifyreg(struct gd32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  adc_putreg(priv, offset,
             (adc_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: adc_rcu_reset
 *
 * Description:
 *   Reset the ADC peripheral via the RCU APB2RST register.
 *   GD32E11x has individual reset bits for ADC0 and ADC1.
 *
 ****************************************************************************/

static void adc_rcu_reset(struct gd32_dev_s *priv, bool reset)
{
  uint32_t rstbit;
  uint32_t enbit;

  switch (priv->intf)
    {
#ifdef CONFIG_GD32E11X_ADC0
      case 0:
        rstbit = RCU_APB2RST_ADC0RST;
        enbit  = RCU_APB2EN_ADC0EN;
        break;
#endif
#ifdef CONFIG_GD32E11X_ADC1
      case 1:
        rstbit = RCU_APB2RST_ADC1RST;
        enbit  = RCU_APB2EN_ADC1EN;
        break;
#endif
      default:
        return;
    }

  /* Enable ADC clock on APB2 */

  modifyreg32(GD32_RCU_APB2EN, 0, enbit);

  /* Configure ADC clock prescaler.
   * Per GD32E11x firmware library examples, ADC clock = CK_APB2 / N.
   * ADC max clock is 20MHz. Typical: APB2=60MHz, DIV6 => 10MHz.
   *
   * ADCPSC is spread across:
   *   CFG0 bits[15:14] = ADCPSC[1:0]
   *   CFG0 bit 28      = ADCPSC[2]
   *   CFG1 bit 29      = ADCPSC[3] (selects AHB-based prescaler)
   *
   * For CK_APB2/6: ADCPSC = 0b0010 => CFG0[15:14]=10, CFG0[28]=0, CFG1[29]=0
   */

  modifyreg32(GD32_RCU_CFG0,
              RCU_CFG0_ADCPSC_MASK | RCU_CFG0_ADCPSC_2,
              GD32_BOARD_ADCPSC_CONFIG);
  modifyreg32(GD32_RCU_CFG1, RCU_CFG1_ADCPSC_MSB, 0);

  if (reset)
    {
      /* Enable ADC reset state */

      modifyreg32(GD32_RCU_APB2RST, 0, rstbit);
    }
  else
    {
      /* Release ADC from reset state */

      modifyreg32(GD32_RCU_APB2RST, rstbit, 0);
    }
}

/****************************************************************************
 * Name: adc_enable
 *
 * Description:
 *   Enables or disables the ADC peripheral.
 *   For GD32E11x basic ADC, setting ADCON = 1 wakes up the ADC.
 *   Setting ADCON again while already on starts a conversion.
 *
 ****************************************************************************/

static void adc_enable(struct gd32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_ADCON);
    }
  else
    {
      adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, ADC_CTL1_ADCON, 0);
    }
}

/****************************************************************************
 * Name: adc_calibrate
 *
 * Description:
 *   Perform ADC calibration. Must be done after ADC is powered on.
 *     1. Power on ADC (ADCON = 1)
 *     2. Wait at least 2 ADC clock cycles
 *     3. Reset calibration (RSTCLB = 1), wait until cleared
 *     4. Start calibration (CLB = 1), wait until cleared
 *     5. Power off ADC
 *
 ****************************************************************************/

static void adc_calibrate(struct gd32_dev_s *priv)
{
  /* Power on the ADC */

  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_ADCON);

  /* Wait for the ADC power on at least 2 ADCCLK cycles */

  up_udelay(10);

  /* Reset calibration registers */

  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_RSTCLB);

  /* Wait for the calibration register reset to complete */

  while ((adc_getreg(priv, GD32_ADC_CTL1_OFFSET) & ADC_CTL1_RSTCLB) != 0);

  /* Start ADC auto-calibration procedure */

  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_CLB);

  /* Wait for the calibration procedure to complete */

  while ((adc_getreg(priv, GD32_ADC_CTL1_OFFSET) & ADC_CTL1_CLB) != 0);

  /* Power off the ADC */

  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, ADC_CTL1_ADCON, 0);
}

/****************************************************************************
 * Name: adc_watchdog_cfg
 *
 * Description:
 *   Configure the analog watchdog. Sets high threshold to max and low
 *   threshold to 0, and enables AWD on the first channel in the list.
 *
 ****************************************************************************/

static void adc_watchdog_cfg(struct gd32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Initialize the watchdog high threshold register */

  adc_putreg(priv, GD32_ADC_WDHT_OFFSET, 0x00000fff);

  /* Initialize the watchdog low threshold register */

  adc_putreg(priv, GD32_ADC_WDLT_OFFSET, 0x00000000);

  /* Configure watchdog on a single channel (per firmware library pattern):
   *   - Clear RWDEN, IWDEN, WDSC, WDCHSEL first
   *   - Set the target channel number
   *   - Enable RWDEN (regular watchdog) and WDSC (single channel mode)
   */

  clrbits = ADC_CTL0_RWDEN | ADC_CTL0_IWDEN |
            ADC_CTL0_WDSC | ADC_CTL0_WDCHSEL_MASK;
  setbits = ADC_CTL0_RWDEN | ADC_CTL0_WDSC |
            (priv->r_chanlist[0] << ADC_CTL0_WDCHSEL_SHIFT);

  /* Modify CTL0 configuration */

  adc_modifyreg(priv, GD32_ADC_CTL0_OFFSET, clrbits, setbits);
}

/****************************************************************************
 * Name: adc_sampletime_cfg
 *
 * Description:
 *   Configure default sample time for all channels.
 *
 ****************************************************************************/

static void adc_sampletime_cfg(struct gd32_dev_s *priv)
{
  adc_putreg(priv, GD32_ADC_SAMPT0_OFFSET, ADC_SMPR1_DEFAULT);
  adc_putreg(priv, GD32_ADC_SAMPT1_OFFSET, ADC_SMPR2_DEFAULT);
}

/****************************************************************************
 * Name: adc_mode_cfg
 *
 * Description:
 *   Configure ADC operating mode: independent mode, scan mode, alignment.
 *   Disable continuous mode and external trigger (set to SW trigger).
 *
 ****************************************************************************/

static void adc_mode_cfg(struct gd32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* SYNCM bits are only valid on ADC0 (firmware library hardcodes ADC0).
   * On ADC1, these bits are reserved and should not be modified.
   */

  if (priv->intf == 0)
    {
      /* Set independent mode */

      clrbits |= ADC_CTL0_SYNCM_MASK;
      setbits |= ADC_CTL0_SYNCM_FREE;
    }

#ifdef ADC_HAVE_SCAN
  if (priv->scan)
    {
      setbits |= ADC_CTL0_SM;
    }
#endif

  /* Set CTL0 configuration */

  adc_modifyreg(priv, GD32_ADC_CTL0_OFFSET, clrbits, setbits);

  /* Disable continuous mode and set align to right */

  clrbits = ADC_CTL1_CTN | ADC_CTL1_DAL;
  setbits = 0;

  /* Configure software trigger: set ETSRC=NONE (0x7) with ETERC=1.
   * Per the firmware library, software-triggered conversions require
   * ETERC enabled and ETSRC set to NONE (software trigger mode),
   * then use SWRCST bit to actually start conversion.
   */

  clrbits |= ADC_CTL1_ETSRC_MASK | ADC_CTL1_ETERC;
  setbits |= ADC_CTL1_ETSRC_NONE | ADC_CTL1_ETERC;

  /* Set CTL1 configuration */

  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, clrbits, setbits);
}

/****************************************************************************
 * Name: adc_reg_startconv
 *
 * Description:
 *   Start (or stop) the ADC regular conversion process.
 *   For GD32E11x, use SWRCST bit to trigger software conversion
 *   (requires ETERC=1 and ETSRC=NONE configured in adc_mode_cfg).
 *
 ****************************************************************************/

static void adc_reg_startconv(struct gd32_dev_s *priv, bool enable)
{
  ainfo("reg enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start regular channel conversion via SWRCST bit */

      adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_SWRCST);
    }
  else
    {
      /* Stop the conversion and power down the ADC */

      adc_enable(priv, false);
    }
}

/****************************************************************************
 * Name: adc_sqrbits
 *
 * Description:
 *   Calculate the SQR register bits for a range of channels.
 *
 ****************************************************************************/

static uint32_t adc_sqrbits(struct gd32_dev_s *priv, int first,
                            int last, int offset)
{
  uint32_t bits = 0;
  int i;

  for (i = first - 1;
       i < priv->rnchannels && i < last;
       i++, offset += ADC_SQ_OFFSET)
    {
      bits |= (uint32_t)priv->r_chanlist[i] << offset;
    }

  return bits;
}

/****************************************************************************
 * Name: adc_set_ch
 *
 * Description:
 *   Sets the ADC channel. ch=0 means all configured channels.
 *   ch > 0 means a specific channel (ch-1).
 *
 ****************************************************************************/

static int adc_set_ch(struct adc_dev_s *dev, uint8_t ch)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;
  uint32_t bits;
  int i;

  if (ch == 0)
    {
      priv->current    = 0;
      priv->rnchannels = priv->cr_channels;
    }
  else
    {
      for (i = 0; i < priv->cr_channels && priv->r_chanlist[i] != ch - 1;
           i++);

      if (i >= priv->cr_channels)
        {
          return -ENODEV;
        }

      priv->current    = i;
      priv->rnchannels = 1;
    }

  /* RSQ2 = SQR3: 1st..6th conversion */

  bits = adc_sqrbits(priv, ADC_SQR3_FIRST, ADC_SQR3_LAST,
                     ADC_SQR3_SQ_OFFSET);
  adc_modifyreg(priv, GD32_ADC_RSQ2_OFFSET, ~ADC_RSQ2_RESERVED, bits);

  /* RSQ1 = SQR2: 7th..12th conversion */

  bits = adc_sqrbits(priv, ADC_SQR2_FIRST, ADC_SQR2_LAST,
                     ADC_SQR2_SQ_OFFSET);
  adc_modifyreg(priv, GD32_ADC_RSQ1_OFFSET, ~ADC_RSQ1_RESERVED, bits);

  /* RSQ0 = SQR1: 13th..16th conversion + sequence length */

  bits = ((uint32_t)priv->rnchannels - 1) << ADC_RSQ0_RL_SHIFT;
  bits |= adc_sqrbits(priv, ADC_SQR1_FIRST, ADC_SQR1_LAST,
                      ADC_SQR1_SQ_OFFSET);
  adc_modifyreg(priv, GD32_ADC_RSQ0_OFFSET, ~ADC_RSQ0_RESERVED, bits);

  return OK;
}

#ifdef ADC_HAVE_DMA
/****************************************************************************
 * Name: adc_dma_cfg
 *
 * Description:
 *   Configure the ADC for DMA operation. Enable the DMA bit in CTL1.
 *
 ****************************************************************************/

static void adc_dma_cfg(struct gd32_dev_s *priv)
{
  /* Enable DMA */

  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_DMA);
}

/****************************************************************************
 * Name: adc_dma_start
 *
 * Description:
 *   Start DMA transfer for ADC conversions.
 *
 ****************************************************************************/

static void adc_dma_start(struct adc_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;
  dma_parameter_struct dma_init;

  /* Stop and free DMA if it was started before */

  if (priv->dma != NULL)
    {
      gd32_dma_stop(priv->dma);
      gd32_dma_channel_free(priv->dma);
    }

  priv->dma = gd32_dma_channel_alloc(priv->dmachan);

  /* Configure DMA: Peripheral to memory, 16-bit, memory increment,
   * circular mode enabled for continuous reading of channels.
   */

  dma_init.periph_addr  = priv->base + GD32_ADC_RDATA_OFFSET;
  dma_init.periph_width = DMA_PWIDTH_16BITS_SELECT;
  dma_init.memory_addr  = (uint32_t)priv->r_dmabuffer;
  dma_init.memory_width = DMA_MWIDTH_16BITS_SELECT;
  dma_init.number       = priv->rnchannels;
  dma_init.priority     = DMA_PRIO_HIGH_SELECT;
  dma_init.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
  dma_init.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
  dma_init.direction    = 0; /* Peripheral to memory (DIR=0) */

  gd32_dma_setup(priv->dma, &dma_init, DMA_MODE_CIRCULAR);
  gd32_dma_start(priv->dma, adc_dmaconvcallback, dev,
                 DMA_CHXCTL_FTFIE | DMA_CHXCTL_ERRIE);
}

/****************************************************************************
 * Name: adc_dmaconvcallback
 *
 * Description:
 *   Callback for DMA. Called from the DMA transfer complete interrupt
 *   after all channels have been converted and transferred with DMA.
 *
 ****************************************************************************/

static void adc_dmaconvcallback(DMA_HANDLE handle, uint16_t status,
                                void *arg)
{
  struct adc_dev_s  *dev  = (struct adc_dev_s *)arg;
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;
  int i;

  UNUSED(handle);

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < priv->rnchannels; i++)
        {
          priv->cb->au_receive(dev, priv->r_chanlist[priv->current],
                               priv->r_dmabuffer[i]);
          priv->current++;
          if (priv->current >= priv->rnchannels)
            {
              /* Restart the conversion sequence from the beginning */

              priv->current = 0;
            }
        }
    }

  /* Restart DMA for the next conversion series.
   * For basic ADC, clear and re-set the DMA bit to restart transfer.
   */

  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, ADC_CTL1_DMA, 0);
  adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_DMA);
}
#endif /* ADC_HAVE_DMA */

/****************************************************************************
 * Name: adc_configure
 *
 * Description:
 *   Configure the ADC: calibrate, configure watchdog, sample times,
 *   mode, channel sequence, optional DMA, and enable ADC.
 *
 ****************************************************************************/

static void adc_configure(struct adc_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;

  /* Turn off the ADC before configuration */

  adc_enable(priv, false);

  /* Calibrate ADC */

  adc_calibrate(priv);

  /* Initialize the ADC watchdog */

  adc_watchdog_cfg(priv);

  /* Initialize the ADC sample time */

  adc_sampletime_cfg(priv);

  /* Set ADC working mode */

  adc_mode_cfg(priv);

  /* Configuration of the channel conversions */

  if (priv->cr_channels > 0)
    {
      adc_set_ch(dev, 0);
    }

#ifdef ADC_HAVE_DMA
  /* Configure ADC DMA if enabled */

  if (priv->hasdma)
    {
      /* Configure ADC DMA */

      adc_dma_cfg(priv);

      /* Start ADC DMA */

      adc_dma_start(dev);
    }
#endif

  /* Enable ADC */

  adc_enable(priv, true);

  /* Set external trigger source if configured (for timer trigger).
   * For basic ADC, ETSRC selects the event and ETERC enables it.
   */

  if (priv->extsel != ADC_CTL1_ETSRC_NONE)
    {
      adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET,
                    ADC_CTL1_ETSRC_MASK,
                    priv->extsel | ADC_CTL1_ETERC);
    }

  /* Dump regs */

  adc_dumpregs(priv);
}

/****************************************************************************
 * Name: adc_dumpregs
 *
 * Description:
 *   Dump ADC registers for debugging.
 *
 ****************************************************************************/

static void adc_dumpregs(struct gd32_dev_s *priv)
{
  UNUSED(priv);

  ainfo("STAT: 0x%08" PRIx32 " CTL0: 0x%08" PRIx32
        " CTL1: 0x%08" PRIx32 "\n",
        adc_getreg(priv, GD32_ADC_STAT_OFFSET),
        adc_getreg(priv, GD32_ADC_CTL0_OFFSET),
        adc_getreg(priv, GD32_ADC_CTL1_OFFSET));

  ainfo("RSQ0: 0x%08" PRIx32 " RSQ1: 0x%08" PRIx32
        " RSQ2: 0x%08" PRIx32 "\n",
        adc_getreg(priv, GD32_ADC_RSQ0_OFFSET),
        adc_getreg(priv, GD32_ADC_RSQ1_OFFSET),
        adc_getreg(priv, GD32_ADC_RSQ2_OFFSET));

  ainfo("SAMPT0: 0x%08" PRIx32 " SAMPT1: 0x%08" PRIx32 "\n",
        adc_getreg(priv, GD32_ADC_SAMPT0_OFFSET),
        adc_getreg(priv, GD32_ADC_SAMPT1_OFFSET));
}

/****************************************************************************
 * Name: adc_ioc_enable_tvref_register
 *
 * Description:
 *   Enable/disable the temperature sensor and VREFINT channel.
 *   Only available on ADC0 (Channel 16 = Temp sensor, 17 = Vrefint).
 *
 ****************************************************************************/

static void adc_ioc_enable_tvref_register(struct gd32_dev_s *priv,
                                          bool enable)
{
  /* TSVREN bit is only available in ADC0 CTL1 register */

  if (priv->intf == 0)
    {
      if (enable)
        {
          adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, 0, ADC_CTL1_TSVREN);
        }
      else
        {
          adc_modifyreg(priv, GD32_ADC_CTL1_OFFSET, ADC_CTL1_TSVREN, 0);
        }
    }

  ainfo("ADC_CTL1 value: 0x%08" PRIx32 "\n",
        adc_getreg(priv, GD32_ADC_CTL1_OFFSET));
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device. Called early to initialize the hardware.
 *   This is called before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(struct adc_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;
  irqstate_t flags;

  ainfo("intf: %d\n", priv->intf);
  flags = enter_critical_section();

  /* Do nothing if ADC instance is currently in use */

  if (priv->initialized > 0)
    {
      goto out;
    }

  /* Enable ADC reset state */

  adc_rcu_reset(priv, true);

  /* Release ADC from reset state */

  adc_rcu_reset(priv, false);

out:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened. This setup includes configuring and attaching ADC
 *   interrupts. Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(struct adc_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;
  int ret = OK;

  /* Do nothing when the ADC device is already set up */

  if (priv->initialized > 0)
    {
      priv->initialized += 1;
      return OK;
    }

  /* Make sure that the ADC device is in the powered up, reset state */

  adc_reset(dev);

  /* Configure ADC device */

  adc_configure(dev);

  /* Attach the ADC interrupt */

  ret = irq_attach(priv->irq, priv->isr, NULL);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  /* Enable the ADC interrupt */

  ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
  up_enable_irq(priv->irq);

  /* Start regular conversion */

#ifndef CONFIG_GD32E11X_ADC_NO_STARTUP_CONV
  adc_reg_startconv(priv, true);
#endif

  /* The ADC device is ready */

  priv->initialized += 1;

  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC. This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;

  /* Decrement count only when ADC device is in use */

  if (priv->initialized > 0)
    {
      priv->initialized -= 1;
    }

  /* Shutdown the ADC device only when not in use */

  if (priv->initialized > 0)
    {
      return;
    }

  /* Disable ADC */

  adc_enable(priv, false);

  /* Disable ADC interrupts and detach the ADC interrupt handler */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

#ifdef ADC_HAVE_DMA
  /* Stop and free DMA if applicable */

  if (priv->hasdma && priv->dma != NULL)
    {
      gd32_dma_stop(priv->dma);
      gd32_dma_channel_free(priv->dma);
      priv->dma = NULL;
    }
#endif

  /* Reset the ADC module */

  adc_rcu_reset(priv, true);
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 ****************************************************************************/

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;

  ainfo("intf: %d enable: %d\n", priv->intf, enable ? 1 : 0);

  if (enable)
    {
      /* Enable the end-of-conversion and analog watchdog interrupts.
       * If DMA is enabled, don't enable EOC interrupt (DMA handles it).
       */

      uint32_t regval = ADC_IER_AWD;
#ifdef ADC_HAVE_DMA
      if (!priv->hasdma)
#endif
        {
          regval |= ADC_IER_EOC;
        }

      adc_modifyreg(priv, GD32_ADC_CTL0_OFFSET, 0, regval);
    }
  else
    {
      /* Disable all ADC interrupts */

      adc_modifyreg(priv, GD32_ADC_CTL0_OFFSET, ADC_IER_ALLINTS, 0);
    }
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start regular conversion if regular channels configured */

          if (priv->cr_channels > 0)
            {
              adc_reg_startconv(priv, true);
            }

          break;
        }

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->rnchannels;
          break;
        }

      case IO_ENABLE_TEMPER_VOLT_CH:
        {
          adc_ioc_enable_tvref_register(priv, *(bool *)arg);
          break;
        }

      case IO_STOP_ADC:
        {
          adc_enable(priv, false);
          break;
        }

      case IO_START_ADC:
        {
          adc_enable(priv, true);
          break;
        }

      case IO_START_CONV:
        {
          uint8_t ch = ((uint8_t)arg);

          ret = adc_set_ch(dev, ch);
          if (ret < 0)
            {
              return ret;
            }

#ifdef CONFIG_ADC
          if (ch)
            {
              /* Clear fifo if upper-half driver enabled */

              dev->ad_recv.af_head = 0;
              dev->ad_recv.af_tail = 0;
            }
#endif

          adc_reg_startconv(priv, true);
          break;
        }

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   Common ADC interrupt handler.
 *
 ****************************************************************************/

static int adc_interrupt(struct adc_dev_s *dev)
{
  struct gd32_dev_s *priv = (struct gd32_dev_s *)dev->ad_priv;
  uint32_t regval;
  uint32_t pending;
  int32_t  data;

  regval  = adc_getreg(priv, GD32_ADC_STAT_OFFSET);
  pending = regval & ADC_ISR_ALLINTS;
  if (pending == 0)
    {
      return OK;
    }

  /* Identifies the interruption AWD or EOC */

  if ((regval & ADC_STAT_WDE) != 0)
    {
      awarn("WARNING: Analog Watchdog, Value converted out of range!\n");
    }

  /* EOC: End of conversion */

  if ((regval & ADC_STAT_EOC) != 0)
    {
      /* Read the converted value and clear EOC bit
       * (It is cleared by reading the ADC_RDATA register)
       */

      data = adc_getreg(priv, GD32_ADC_RDATA_OFFSET) &
             ADC_RDATA_RDATA_MASK;

      /* Verify that the upper-half driver has bound its callback
       * functions
       */

      if (priv->cb != NULL)
        {
          /* Give the ADC data to the ADC driver.  The ADC receive()
           * method accepts 3 parameters:
           *
           * 1) The first is the ADC device instance for this ADC block.
           * 2) The second is the channel number for the data, and
           * 3) The third is the converted data for the channel.
           */

          DEBUGASSERT(priv->cb->au_receive != NULL);
          priv->cb->au_receive(dev, priv->r_chanlist[priv->current],
                               data);
        }

      /* Set the channel number of the next channel that will complete
       * conversion.
       */

      priv->current++;

      if (priv->current >= priv->rnchannels)
        {
          /* Restart the conversion sequence from the beginning */

          priv->current = 0;
        }
    }

  /* Clear pending interrupts by writing 0 to the status bits */

  adc_modifyreg(priv, GD32_ADC_STAT_OFFSET, pending, 0);

  return OK;
}

/****************************************************************************
 * Name: adc01_interrupt
 *
 * Description:
 *   ADC0/ADC1 shared interrupt handler.
 *   GD32E11x uses a single IRQ for both ADC0 and ADC1.
 *
 ****************************************************************************/

static int adc01_interrupt(int irq, void *context, void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

#ifdef CONFIG_GD32E11X_ADC0
  adc_interrupt(&g_adcdev0);
#endif

#ifdef CONFIG_GD32E11X_ADC1
  adc_interrupt(&g_adcdev1);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   intf      - Could be {0,1} for ADC0, ADC1
 *   chanlist  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *gd32_adc_initialize(int intf, const uint8_t *chanlist,
                                      int nchannels)
{
  struct adc_dev_s  *dev;
  struct gd32_dev_s *priv;
  uint8_t cr_channels = 0;

  switch (intf)
    {
#ifdef CONFIG_GD32E11X_ADC0
      case 0:
        {
          ainfo("ADC0 selected\n");
          dev = &g_adcdev0;
          break;
        }
#endif

#ifdef CONFIG_GD32E11X_ADC1
      case 1:
        {
          ainfo("ADC1 selected\n");
          dev = &g_adcdev1;
          break;
        }
#endif

      default:
        {
          aerr("ERROR: No ADC interface defined\n");
          return NULL;
        }
    }

  /* Configure the selected ADC */

  priv = (struct gd32_dev_s *)dev->ad_priv;

  cr_channels = nchannels;

  /* Configure regular channels */

  DEBUGASSERT(cr_channels <= CONFIG_GD32E11X_ADC_MAX_SAMPLES);
  if (cr_channels > CONFIG_GD32E11X_ADC_MAX_SAMPLES)
    {
      cr_channels = CONFIG_GD32E11X_ADC_MAX_SAMPLES;
    }

  priv->cr_channels = cr_channels;
  memcpy(priv->r_chanlist, chanlist, cr_channels);

  ainfo("intf: %d cr_channels: %d\n", intf, priv->cr_channels);

  return dev;
}

#endif /* CONFIG_GD32E11X_ADC */
