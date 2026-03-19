/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_dac.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/analog/dac.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32e11x.h"
#include "gd32e11x_dac.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* GD32E11X has a single DAC module (DAC0) with 2 output channels:
 *   OUT0 (PA4) - controlled by CONFIG_GD32E11X_DAC0
 *   OUT1 (PA5) - controlled by CONFIG_GD32E11X_DAC1
 *
 * Both channels share the same DAC_CTL0 control register.
 * OUT0 uses bits [0:12], OUT1 uses bits [16:28].
 */

#if !defined(CONFIG_GD32E11X_DAC0) && !defined(CONFIG_GD32E11X_DAC1)
#  warning "No DAC channel selected"
#endif

/* Timer trigger configuration.
 *
 * GD32E11X DAC trigger sources:
 *   TSEL  Source
 *   ----  ----------------------
 *   000   TIMER5 TRGO
 *   001   TIMER2 TRGO
 *   010   TIMER6 TRGO
 *   011   TIMER4 TRGO
 *   100   TIMER1 TRGO
 *   101   TIMER3 TRGO
 *   110   EXTI line 9
 *   111   Software trigger
 *
 * This driver does not support the EXTI trigger.
 */

/* DAC0 (OUT0) timer trigger configuration */

#ifdef CONFIG_GD32E11X_DAC0_DMA
#  if !defined(CONFIG_GD32E11X_DAC0_TIMER)
#    warning "A timer number must be specified in CONFIG_GD32E11X_DAC0_TIMER"
#    undef CONFIG_GD32E11X_DAC0_DMA
#    undef CONFIG_GD32E11X_DAC0_TIMER_FREQUENCY
#  elif !defined(CONFIG_GD32E11X_DAC0_TIMER_FREQUENCY)
#    warning "A timer frequency must be specified in CONFIG_GD32E11X_DAC0_TIMER_FREQUENCY"
#    undef CONFIG_GD32E11X_DAC0_DMA
#    undef CONFIG_GD32E11X_DAC0_TIMER
#  endif
#endif

#ifdef CONFIG_GD32E11X_DAC1_DMA
#  if !defined(CONFIG_GD32E11X_DAC1_TIMER)
#    warning "A timer number must be specified in CONFIG_GD32E11X_DAC1_TIMER"
#    undef CONFIG_GD32E11X_DAC1_DMA
#    undef CONFIG_GD32E11X_DAC1_TIMER_FREQUENCY
#  elif !defined(CONFIG_GD32E11X_DAC1_TIMER_FREQUENCY)
#    warning "A timer frequency must be specified in CONFIG_GD32E11X_DAC1_TIMER_FREQUENCY"
#    undef CONFIG_GD32E11X_DAC1_DMA
#    undef CONFIG_GD32E11X_DAC1_TIMER
#  endif
#endif

/* Timer trigger selection values based on configured timer number */

#ifdef CONFIG_GD32E11X_DAC0_DMA
#  if CONFIG_GD32E11X_DAC0_TIMER == 5
#    ifndef CONFIG_GD32E11X_TIMER5_DAC
#      error "CONFIG_GD32E11X_TIMER5_DAC required for DAC0"
#    endif
#    define DAC0_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC0_TIMER_BASE           GD32_TIMER5_BASE
#  elif CONFIG_GD32E11X_DAC0_TIMER == 2
#    ifndef CONFIG_GD32E11X_TIMER2_DAC
#      error "CONFIG_GD32E11X_TIMER2_DAC required for DAC0"
#    endif
#    define DAC0_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC0_TIMER_BASE           GD32_TIMER2_BASE
#  elif CONFIG_GD32E11X_DAC0_TIMER == 6
#    ifndef CONFIG_GD32E11X_TIMER6_DAC
#      error "CONFIG_GD32E11X_TIMER6_DAC required for DAC0"
#    endif
#    define DAC0_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC0_TIMER_BASE           GD32_TIMER6_BASE
#  elif CONFIG_GD32E11X_DAC0_TIMER == 4
#    ifndef CONFIG_GD32E11X_TIMER4_DAC
#      error "CONFIG_GD32E11X_TIMER4_DAC required for DAC0"
#    endif
#    define DAC0_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC0_TIMER_BASE           GD32_TIMER4_BASE
#  elif CONFIG_GD32E11X_DAC0_TIMER == 1
#    ifndef CONFIG_GD32E11X_TIMER1_DAC
#      error "CONFIG_GD32E11X_TIMER1_DAC required for DAC0"
#    endif
#    define DAC0_TSEL_VALUE           DAC_CR_TSEL_TIM1
#    define DAC0_TIMER_BASE           GD32_TIMER1_BASE
#  elif CONFIG_GD32E11X_DAC0_TIMER == 3
#    ifndef CONFIG_GD32E11X_TIMER3_DAC
#      error "CONFIG_GD32E11X_TIMER3_DAC required for DAC0"
#    endif
#    define DAC0_TSEL_VALUE           DAC_CR_TSEL_TIM3
#    define DAC0_TIMER_BASE           GD32_TIMER3_BASE
#  else
#    error "Unsupported CONFIG_GD32E11X_DAC0_TIMER"
#  endif
#else
#  define DAC0_TSEL_VALUE DAC_CR_TSEL_SW
#endif

#ifdef CONFIG_GD32E11X_DAC1_DMA
#  if CONFIG_GD32E11X_DAC1_TIMER == 5
#    ifndef CONFIG_GD32E11X_TIMER5_DAC
#      error "CONFIG_GD32E11X_TIMER5_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC1_TIMER_BASE           GD32_TIMER5_BASE
#  elif CONFIG_GD32E11X_DAC1_TIMER == 2
#    ifndef CONFIG_GD32E11X_TIMER2_DAC
#      error "CONFIG_GD32E11X_TIMER2_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC1_TIMER_BASE           GD32_TIMER2_BASE
#  elif CONFIG_GD32E11X_DAC1_TIMER == 6
#    ifndef CONFIG_GD32E11X_TIMER6_DAC
#      error "CONFIG_GD32E11X_TIMER6_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC1_TIMER_BASE           GD32_TIMER6_BASE
#  elif CONFIG_GD32E11X_DAC1_TIMER == 4
#    ifndef CONFIG_GD32E11X_TIMER4_DAC
#      error "CONFIG_GD32E11X_TIMER4_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC1_TIMER_BASE           GD32_TIMER4_BASE
#  elif CONFIG_GD32E11X_DAC1_TIMER == 1
#    ifndef CONFIG_GD32E11X_TIMER1_DAC
#      error "CONFIG_GD32E11X_TIMER1_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM1
#    define DAC1_TIMER_BASE           GD32_TIMER1_BASE
#  elif CONFIG_GD32E11X_DAC1_TIMER == 3
#    ifndef CONFIG_GD32E11X_TIMER3_DAC
#      error "CONFIG_GD32E11X_TIMER3_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM3
#    define DAC1_TIMER_BASE           GD32_TIMER3_BASE
#  else
#    error "Unsupported CONFIG_GD32E11X_DAC1_TIMER"
#  endif
#else
#  define DAC1_TSEL_VALUE DAC_CR_TSEL_SW
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the single GD32E11X DAC
 * block
 */

struct gd32_dac_s
{
  uint8_t    init   : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one GD32E11X DAC
 * channel
 */

struct gd32_chan_s
{
  uint8_t    inuse  : 1; /* True, the driver is in use and not available */
  uint8_t    intf;       /* DAC zero-based interface number (0=OUT0, 1=OUT1) */
  uint32_t   pin;        /* Pin configuration */
  uint32_t   dro;        /* Data output register */
  uint32_t   cr;         /* Control register */
  uint32_t   tsel;       /* CR trigger select value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* DAC methods */

static void dac_reset(struct dac_dev_s *dev);
static int  dac_setup(struct dac_dev_s *dev);
static void dac_shutdown(struct dac_dev_s *dev);
static void dac_txint(struct dac_dev_s *dev, bool enable);
static int  dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int  dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg);

/* Initialization */

static int  dac_chaninit(struct gd32_chan_s *chan);
static int  dac_blockinit(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = dac_reset,
  .ao_setup    = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint    = dac_txint,
  .ao_send     = dac_send,
  .ao_ioctl    = dac_ioctl,
};

#ifdef CONFIG_GD32E11X_DAC0

/* DAC OUT0 channel */

static struct gd32_chan_s g_dac0priv =
{
  .intf       = 0,
  .pin        = GPIO_DAC_OUT0,
  .dro        = GD32_DAC_OUT0_R12DH,
  .cr         = GD32_DAC_CTL0,
  .tsel       = DAC0_TSEL_VALUE,
};

static struct dac_dev_s g_dac0dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac0priv,
};

#endif /* CONFIG_GD32E11X_DAC0 */

#ifdef CONFIG_GD32E11X_DAC1

/* DAC OUT1 channel */

static struct gd32_chan_s g_dac1priv =
{
  .intf       = 1,
  .pin        = GPIO_DAC_OUT1,
  .dro        = GD32_DAC_OUT1_R12DH,
  .cr         = GD32_DAC_CTL0,
  .tsel       = DAC1_TSEL_VALUE,
};

static struct dac_dev_s g_dac1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1priv,
};

#endif /* CONFIG_GD32E11X_DAC1 */

static struct gd32_dac_s g_dacblock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dac_modify_cr
 *
 * Description:
 *   Modify the contents of the DAC control register.
 *
 *   Both DAC channels (OUT0 and OUT1) share the same DAC_CTL0 register.
 *   OUT0 uses bits [0:12] (shift = 0), OUT1 uses bits [16:28] (shift = 16).
 *
 * Input Parameters:
 *   chan      - A reference to the DAC channel state
 *   clearbits - Bits in the control register to be cleared
 *   setbits   - Bits in the control register to be set
 *
 ****************************************************************************/

static inline void gd32_dac_modify_cr(struct gd32_chan_s *chan,
                                      uint32_t clearbits, uint32_t setbits)
{
  unsigned int shift;

  /* OUT0: intf=0, shift=0; OUT1: intf=1, shift=16 */

  shift = (chan->intf & 1) << 4;
  modifyreg32(chan->cr, clearbits << shift, setbits << shift);
}

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC channel.  Called during dac_register() to initialize the
 *   hardware.  The DAC channel is already configured during board bringup
 *   (dac_chaninit), so no additional action is needed here.
 *
 ****************************************************************************/

static void dac_reset(struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.
 *
 ****************************************************************************/

static int dac_setup(struct dac_dev_s *dev)
{
  struct gd32_chan_s *chan = dev->ad_priv;

  /* Ensure the DAC channel is enabled when the device is opened */

  gd32_dac_modify_cr(chan, 0, DAC_CR_EN);
  return OK;
}

/****************************************************************************
 * Name: dac_shutdown
 *
 * Description:
 *   Disable the DAC. This method is called when the DAC device is closed.
 *
 *   Note: The DAC output is intentionally left enabled after close() so
 *   that the output voltage is maintained.  This matches the expected
 *   behavior where a user sets a DAC value and then measures the output.
 *
 ****************************************************************************/

static void dac_shutdown(struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: dac_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 *   GD32E11X DAC does not have a dedicated output interrupt.
 *   For non-DMA mode, dac_txdone() is called directly in dac_send().
 *
 ****************************************************************************/

static void dac_txint(struct dac_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: dac_send
 *
 * Description:
 *   Set the DAC output.
 *
 * Input Parameters:
 *   dev - DAC device
 *   msg - The DAC message with output value in am_data (12-bit, 0-4095)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
  struct gd32_chan_s *chan = dev->ad_priv;

  /* Enable DAC channel */

  gd32_dac_modify_cr(chan, 0, DAC_CR_EN);

  /* Write data to the 12-bit right-aligned data holding register.
   * The data will be transferred to the output register after one
   * APB1 clock cycle (if trigger is disabled) or after the trigger
   * event (if trigger is enabled).
   */

  putreg32((uint32_t)msg->am_data & 0xfff, chan->dro);

  /* Signal that the data has been sent */

  dac_txdone(dev);

  return OK;
}

/****************************************************************************
 * Name: dac_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: dac_chaninit
 *
 * Description:
 *   Initialize the DAC channel.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_chaninit(struct gd32_chan_s *chan)
{
  uint32_t clearbits;
  uint32_t setbits;

  /* Is the selected channel already in-use? */

  if (chan->inuse)
    {
      return -EBUSY;
    }

  /* Configure the DAC output pin:
   *
   * DAC - Once the DAC channel is enabled, the corresponding GPIO pin
   * (PA4 for OUT0 or PA5 for OUT1) is automatically connected to the
   * analog converter output.  In order to avoid parasitic consumption,
   * the pin should first be configured to analog input mode (AIN).
   */

  gd32_gpio_config(chan->pin);

  /* DAC channel configuration:
   *
   * - Set the trigger selection based upon the configuration.
   * - Set wave generation == None.
   * - Enable the output buffer (clear DBOFF bit).
   */

  /* Disable before change */

  gd32_dac_modify_cr(chan, DAC_CR_EN, 0);

  clearbits = DAC_CR_TSEL_MASK |
              DAC_CR_MAMP_MASK |
              DAC_CR_WAVE_MASK |
              DAC_CR_BOFF;

  /* Configure trigger selection, waveform amplitude, and wave mode. */

  setbits = chan->tsel |
            DAC_CR_MAMP_AMP1 |
            DAC_CR_WAVE_DISABLED;

  /* Output buffer control: DBOFF=0 enables buffer, DBOFF=1 disables it.
   * The clearbits above already clears DAC_CR_BOFF, which enables the
   * output buffer by default.  Only set BOFF if buffer is disabled.
   */

#ifdef CONFIG_GD32E11X_DAC0
  if (chan->intf == 0)
    {
#  ifndef CONFIG_GD32E11X_DAC0_OUTPUT_BUFFER
      setbits |= DAC_CR_BOFF;     /* Disable output buffer (DBOFF=1) */
#  endif
    }
#endif

#ifdef CONFIG_GD32E11X_DAC1
  if (chan->intf == 1)
    {
#  ifndef CONFIG_GD32E11X_DAC1_OUTPUT_BUFFER
      setbits |= DAC_CR_BOFF;     /* Disable output buffer (DBOFF=1) */
#  endif
    }
#endif

  gd32_dac_modify_cr(chan, clearbits, setbits);

  /* Enable the DAC channel output (DEN=1).  The channel is enabled
   * immediately after configuration so that it is ready for use when
   * the device node is opened and written to.  The shutdown callback
   * intentionally leaves DEN enabled so the output voltage persists.
   */

  gd32_dac_modify_cr(chan, 0, DAC_CR_EN);

  /* Mark the DAC channel "in-use" */

  chan->inuse = 1;
  return OK;
}

/****************************************************************************
 * Name: dac_blockinit
 *
 * Description:
 *   Initialize the DAC block.  Enable DAC clock via RCU and reset the
 *   peripheral.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_blockinit(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Has the DAC block already been initialized? */

  if (g_dacblock.init)
    {
      return OK;
    }

  flags = enter_critical_section();

  /* Enable DAC clock */

  modifyreg32(GD32_RCU_APB1EN, 0, RCU_APB1EN_DACEN);

  /* Reset DAC peripheral */

  regval  = getreg32(GD32_RCU_APB1RST);
  regval |= RCU_APB1RST_DACRST;
  putreg32(regval, GD32_RCU_APB1RST);

  /* Take the DAC out of reset */

  regval &= ~RCU_APB1RST_DACRST;
  putreg32(regval, GD32_RCU_APB1RST);

  leave_critical_section(flags);

  /* Mark the DAC block as initialized */

  g_dacblock.init = 1;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *          0 = DAC OUT0 (PA4)
 *          1 = DAC OUT1 (PA5)
 *
 * Returned Value:
 *   Valid DAC device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct dac_dev_s *gd32_dacinitialize(int intf)
{
  struct dac_dev_s    *dev;
  struct gd32_chan_s  *chan;
  int ret;

#ifdef CONFIG_GD32E11X_DAC0
  if (intf == 0)
    {
      ainfo("DAC OUT0 Selected\n");
      dev = &g_dac0dev;
    }
  else
#endif
#ifdef CONFIG_GD32E11X_DAC1
  if (intf == 1)
    {
      ainfo("DAC OUT1 Selected\n");
      dev = &g_dac1dev;
    }
  else
#endif
    {
      aerr("ERROR: No such DAC interface: %d\n", intf);
      return NULL;
    }

  /* Make sure that the DAC block has been initialized */

  ret = dac_blockinit();
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize the DAC block: %d\n", ret);
      return NULL;
    }

  /* Configure the selected DAC channel */

  chan = dev->ad_priv;
  ret = dac_chaninit(chan);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize DAC channel %d: %d\n", intf, ret);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_DAC */
