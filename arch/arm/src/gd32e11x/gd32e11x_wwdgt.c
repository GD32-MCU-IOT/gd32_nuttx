/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_wwdgt.c
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/gd32e11x_wdg.h"
#include "hardware/gd32e11x_rcu.h"
#include "hardware/gd32e11x_dbg.h"
#include "gd32e11x_rcu.h"
#include "gd32e11x_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_GD32E11X_WWDG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The minimum frequency of the WWDGT clock is:
 *
 *  Fmin = PCLK1 / 4096 / 8
 *
 * So the maximum delay (in milliseconds) is then:
 *
 *   1000 * (WWDGT_CTL_CNT_MAX+1) / Fmin
 *
 * For example, if PCLK1 = 60MHz, then the maximum delay is:
 *
 *   Fmin = 1831.05
 *   1000 * 64 / Fmin = 34.95 msec
 */

#define WWDGT_FMIN       (GD32_PCLK1_FREQUENCY / 4096 / 8)
#define WWDGT_MAXTIMEOUT (1000 * (WWDGT_CTL_CNT_MAX+1) / WWDGT_FMIN)

/* Configuration ************************************************************/

#ifndef CONFIG_GD32E11X_WWDG_DEFTIMOUT
#  define CONFIG_GD32E11X_WWDG_DEFTIMOUT WWDGT_MAXTIMEOUT
#endif

#ifndef CONFIG_GD32E11X_WWDG_WINDOW
#  define CONFIG_GD32E11X_WWDG_WINDOW    0x7f
#endif

#ifndef CONFIG_DEBUG_WATCHDOG_INFO
#  undef CONFIG_GD32E11X_WWDG_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct gd32_wwdgt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;    /* Lower half operations */
  xcpt_t   handler;                    /* Current EWI interrupt handler */
  uint32_t timeout;                    /* The actual timeout value */
  uint32_t fwwdgt;                     /* WWDGT clock frequency */
  bool     started;                    /* The timer has been started */
  uint8_t  reload;                     /* The 7-bit reload field reset value */
  uint8_t  window;                     /* The 7-bit window (W) field value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_GD32E11X_WWDG_REGDEBUG
static uint16_t gd32_getreg(uint32_t addr);
static void     gd32_putreg(uint16_t val, uint32_t addr);
#else
#  define       gd32_getreg(addr)     getreg32(addr)
#  define       gd32_putreg(val,addr) putreg32(val,addr)
#endif
static void     gd32_setwindow(struct gd32_wwdgt_lowerhalf_s *priv,
                  uint8_t window);

/* Interrupt handling *******************************************************/

static int      gd32_wwdgt_interrupt(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int      gd32_wwdgt_start(struct watchdog_lowerhalf_s *lower);
static int      gd32_wwdgt_stop(struct watchdog_lowerhalf_s *lower);
static int      gd32_wwdgt_keepalive(struct watchdog_lowerhalf_s *lower);
static int      gd32_wwdgt_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status);
static int      gd32_wwdgt_settimeout(struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);
static xcpt_t   gd32_wwdgt_capture(struct watchdog_lowerhalf_s *lower,
                  xcpt_t handler);
static int      gd32_wwdgt_ioctl(struct watchdog_lowerhalf_s *lower,
                  int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wwdgt_ops =
{
  .start      = gd32_wwdgt_start,
  .stop       = gd32_wwdgt_stop,
  .keepalive  = gd32_wwdgt_keepalive,
  .getstatus  = gd32_wwdgt_getstatus,
  .settimeout = gd32_wwdgt_settimeout,
  .capture    = gd32_wwdgt_capture,
  .ioctl      = gd32_wwdgt_ioctl,
};

/* "Lower half" driver state */

static struct gd32_wwdgt_lowerhalf_s g_wwdgt_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_getreg
 *
 * Description:
 *   Get the contents of a GD32 WWDGT register
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_WWDG_REGDEBUG
static uint16_t gd32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count = 0;
  static uint16_t preval = 0;

  /* Read the value from the register */

  uint16_t val = getreg16(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              wdinfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          wdinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  wdinfo("%08x->%04x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: gd32_putreg
 *
 * Description:
 *   Set the contents of a GD32 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_WWDG_REGDEBUG
static void gd32_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  wdinfo("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: gd32_setwindow
 *
 * Description:
 *   Set the CFG window value. The window value is compared to the down-
 *   counter when the counter is updated.  The WWDGT counter should be
 *   updated only when the counter is below this window value (and greater
 *   than 0x40) otherwise a reset will be generated
 *
 ****************************************************************************/

static void gd32_setwindow(struct gd32_wwdgt_lowerhalf_s *priv,
                           uint8_t window)
{
  uint16_t regval;

  /* Set WIN[6:0] bits according to selected window value */

  regval = gd32_getreg(GD32_WWDGT_CFG);
  regval &= ~WWDGT_CFG_WIN_MASK;
  regval |= window << WWDGT_CFG_WIN_SHIFT;
  gd32_putreg(regval, GD32_WWDGT_CFG);

  /* Remember the window setting */

  priv->window = window;
}

/****************************************************************************
 * Name: gd32_wwdgt_interrupt
 *
 * Description:
 *   WWDGT early warning interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int gd32_wwdgt_interrupt(int irq, void *context, void *arg)
{
  struct gd32_wwdgt_lowerhalf_s *priv = &g_wwdgt_dev;
  uint16_t regval;

  /* Check if the EWI interrupt is really pending */

  regval = gd32_getreg(GD32_WWDGT_STAT);
  if ((regval & WWDGT_STAT_EWIF) != 0)
    {
      /* Is there a registered handler? */

      if (priv->handler)
        {
          /* Yes... NOTE:  This interrupt service routine (ISR) must reload
           * the WWDGT counter to prevent the reset.  Otherwise, we will
           * reset upon return.
           */

          priv->handler(irq, context, arg);
        }

      /* The EWI interrupt is cleared by writing '0' to the EWIF bit in the
       * WWDGT_STAT register.
       */

      regval &= ~WWDGT_STAT_EWIF;
      gd32_putreg(regval, GD32_WWDGT_STAT);
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_wwdgt_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_wwdgt_start(struct watchdog_lowerhalf_s *lower)
{
  struct gd32_wwdgt_lowerhalf_s *priv =
    (struct gd32_wwdgt_lowerhalf_s *)lower;
  uint16_t cfg_reg;
  uint16_t ctl_reg;
  uint8_t win_value;
  uint8_t cnt_value;
  uint16_t threshold;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Read current window configuration before starting */

  cfg_reg = gd32_getreg(GD32_WWDGT_CFG);
  win_value = (cfg_reg & WWDGT_CFG_WIN_MASK) >> WWDGT_CFG_WIN_SHIFT;
  threshold = 0x40 + win_value;
  cnt_value = 0x40 | priv->reload;

  wdinfo("Before start: WIN=0x%02x threshold=0x%02x CNT=0x%02x\n",
         win_value, threshold, cnt_value);

  /* CRITICAL: Check if starting will violate window */

  if (win_value > 0 && cnt_value >= threshold)
    {
      wderr("ERROR: Window violation at start! "
            "CNT=0x%02x >= threshold=0x%02x\n",
            cnt_value, threshold);
      wderr("       This will cause immediate reset!\n");
      wderr("       WIN=0x%02x, mintime was too large for this "
            "timeout\n", win_value);
      wderr("       Hardware limitation: window mechanism requires "
            "timeout > 50ms\n");
      wderr("       For timeout=33ms, mintime must be 0 "
            "(window disabled)\n");
    }

  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGTEN bit in the WWDGT_CTL register, then it cannot be disabled
   * again except by a reset.
   */

  gd32_putreg(WWDGT_CTL_WDGTEN | (0x40 | priv->reload), GD32_WWDGT_CTL);
  priv->started = true;

  /* Read back to confirm what was written */

  ctl_reg = gd32_getreg(GD32_WWDGT_CTL);
  wdinfo("After start: CTL=0x%04x (enabled=%d CNT=0x%02x)\n",
         ctl_reg,
         (ctl_reg & WWDGT_CTL_WDGTEN) ? 1 : 0,
         ctl_reg & WWDGT_CTL_CNT_MASK);

  return OK;
}

/****************************************************************************
 * Name: gd32_wwdgt_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_wwdgt_stop(struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGTEN bit in the WWDGT_CTL register, then it cannot be disabled
   * again except by a reset.
   */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: gd32_wwdgt_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 *   The application program must write in the WWDGT_CTL register at
 *   regular intervals during normal operation to prevent an MCU reset.
 *   This operation must occur only when the counter value is lower than
 *   the window register value. The value to be stored in the WWDGT_CTL
 *   register must be between 0x7f and 0x40:
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_wwdgt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct gd32_wwdgt_lowerhalf_s *priv =
    (struct gd32_wwdgt_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Write to CNT[6:0] bits to configure the counter value, no need to do
   * a read-modify-write; writing a 0 to WDGTEN bit does nothing.
   */

  gd32_putreg((0x40 | priv->reload), GD32_WWDGT_CTL);
  return OK;
}

/****************************************************************************
 * Name: gd32_wwdgt_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_wwdgt_getstatus(struct watchdog_lowerhalf_s *lower,
                                struct watchdog_status_s *status)
{
  struct gd32_wwdgt_lowerhalf_s *priv =
    (struct gd32_wwdgt_lowerhalf_s *)lower;
  uint32_t elapsed;
  uint16_t reload;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  if (priv->handler)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds) */

  reload = (gd32_getreg(GD32_WWDGT_CTL) >> WWDGT_CTL_CNT_SHIFT) & 0x7f;
  elapsed = priv->reload - reload;
  status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", (unsigned)status->flags);
  wdinfo("  timeout  : %u\n", (unsigned)status->timeout);
  wdinfo("  timeleft : %u\n", (unsigned)status->flags);
  return OK;
}

/****************************************************************************
 * Name: gd32_wwdgt_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_wwdgt_settimeout(struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout)
{
  struct gd32_wwdgt_lowerhalf_s *priv =
    (struct gd32_wwdgt_lowerhalf_s *)lower;
  uint32_t fwwdgt;
  uint32_t reload;
  uint16_t regval;
  int psc;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%u\n", (unsigned)timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WWDGT_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%u > %lu\n",
            (unsigned)timeout, WWDGT_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Determine prescaler value.
   *
   * Fwwdgt = PCLK1/4096/prescaler.
   *
   * Where
   *  Fwwdgt is the frequency of the WWDGT clock
   *  psc is one of {1, 2, 4, or 8}
   */

  /* Select the smallest prescaler that will result in a reload field value
   * that is less than the maximum.
   */

  for (psc = 0; ; psc++)
    {
      /* PSC = 0 -> Divider = 1  = 1 << 0
       * PSC = 1 -> Divider = 2  = 1 << 1
       * PSC = 2 -> Divider = 4  = 1 << 2
       * PSC = 3 -> Divider = 8  = 1 << 3
       */

      /* Get the WWDGT counter frequency in Hz. */

      fwwdgt = (GD32_PCLK1_FREQUENCY / 4096) >> psc;

      /* The formula to calculate the timeout value is given by:
       *
       * timeout =  1000 * (reload + 1) / Fwwdgt, OR
       * reload = timeout * Fwwdgt / 1000 - 1
       *
       * Where
       *  timeout is the desired timeout in milliseconds
       *  reload is the contents of CNT[5:0]
       *  Fwwdgt is the frequency of the WWDGT clock
       */

       reload = timeout * fwwdgt / 1000 - 1;

      /* If this reload valid is less than the maximum or we are not ready
       * at the prescaler value, then break out of the loop to use these
       * settings.
       */

#if 0
      wdinfo("psc=%d fwwdgt=%d reload=%d timeout=%d\n",
             psc, fwwdgt, reload,  1000 * (reload + 1) / fwwdgt);
#endif
      if (reload <= 0x3f || psc == 3)
        {
          /* Note that we explicitly break out of the loop rather than using
           * the 'for' loop termination logic because we do not want the
           * value of psc to be incremented.
           */

          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > 0x3f)
    {
      reload = 0x3f;
    }

  /* Calculate and save the actual timeout value in milliseconds:
   *
   * timeout =  1000 * (reload + 1) / Fwwdgt
   */

  priv->timeout = 1000 * (reload + 1) / fwwdgt;

  /* Remember the selected values */

  priv->fwwdgt  = fwwdgt;
  priv->reload = reload;

  wdinfo("psc=%d fwwdgt=%u reload=%u timeout=%u\n",
         psc, (unsigned)fwwdgt, (unsigned)reload, (unsigned)priv->timeout);

  /* Set PSC[1:0] bits according to calculated value */

  regval = gd32_getreg(GD32_WWDGT_CFG);
  regval &= ~WWDGT_CFG_PSC_MASK;
  regval |= (uint16_t)psc << WWDGT_CFG_PSC_SHIFT;
  gd32_putreg(regval, GD32_WWDGT_CFG);

  /* NOTE: Window value should be configured separately via WDIOC_MINTIME.
   * We don't touch it here to allow dynamic window configuration.
   */

  return OK;
}

/****************************************************************************
 * Name: gd32_wwdgt_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t gd32_wwdgt_capture(struct watchdog_lowerhalf_s *lower,
                                 xcpt_t handler)
{
  struct gd32_wwdgt_lowerhalf_s *priv =
    (struct gd32_wwdgt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;
  uint16_t regval;

  DEBUGASSERT(priv);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags = enter_critical_section();
  oldhandler = priv->handler;

  /* Save the new handler */

  priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  regval = gd32_getreg(GD32_WWDGT_CFG);
  if (handler)
    {
      /* Attaching... Enable the EWI interrupt */

      regval |= WWDGT_CFG_EWIE;
      gd32_putreg(regval, GD32_WWDGT_CFG);

      up_enable_irq(GD32_IRQ_WWDGT);
    }
  else
    {
      /* Detaching... Disable the EWI interrupt */

      regval &= ~WWDGT_CFG_EWIE;
      gd32_putreg(regval, GD32_WWDGT_CFG);

      up_disable_irq(GD32_IRQ_WWDGT);
    }

  leave_critical_section(flags);
  return oldhandler;
}

/****************************************************************************
 * Name: gd32_wwdgt_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.\n *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_wwdgt_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                           unsigned long arg)
{
  struct gd32_wwdgt_lowerhalf_s *priv =
    (struct gd32_wwdgt_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  wdinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* WDIOC_MINTIME: Set the minimum ping time.  If two keepalive ioctls
   * are received within this time, a reset event will be generated.
   * Argument: A 32-bit time value in milliseconds.
   */

  if (cmd == WDIOC_MINTIME)
    {
      uint32_t mintime = (uint32_t)arg;

      /* The minimum time should be strictly less than the total delay
       * which, in turn, will be less than or equal to 0x3f
       */

      ret = -EINVAL;
      if (mintime < priv->timeout)
        {
          uint32_t window;

          /* Special case: mintime=0 means disable window (like official) */

          if (mintime == 0)
            {
              window = 0x7f;  /* Max value = disabled, feed anytime */
              wdinfo("Window disabled: mintime=0, window=0x7F\n");
            }
          else
            {
              /* Calculate WIN register value based on mintime
               *
               * Hardware behavior:
               *   - CNT starts at (0x40 + reload), counts down
               *   - When CNT > WIN, feeding causes reset
               *   - When WIN >= CNT > 0x40, feeding is allowed
               *   - When CNT <= 0x40, system resets
               *
               * So WIN = initial_cnt - (mintime * fwwdgt / 1000)
               *        = (0x40 + reload) - (mintime * fwwdgt / 1000)
               *
               * Example: reload=63, mintime=26ms, fwwdgt=1831Hz
               *   WIN = (64+63) - (26*1831/1000) = 127 - 47 = 80 (0x50)
               */

              uint32_t initial_cnt = 0x40 + priv->reload;
              uint32_t decrement = mintime * priv->fwwdgt / 1000;

              if (decrement >= initial_cnt - 0x40)
                {
                  /* mintime too large, window would be <= 0x40 */

                  window = 0x40;
                }
              else
                {
                  window = initial_cnt - decrement;
                }

              /* Window value must be in range 0x40-0x7F */

              if (window > 0x7f)
                {
                  window = 0x7f;
                }
              else if (window < 0x40)
                {
                  window = 0x40;
                }

              wdinfo("Setting window: mintime=%lu timeout=%lu "
                     "window=0x%02lx reload=%u initial_cnt=%lu\n",
                     (unsigned long)mintime,
                     (unsigned long)priv->timeout,
                     (unsigned long)window, priv->reload,
                     (unsigned long)initial_cnt);
            }

          /* Set window value (must be 0x40-0x7F for WWDGT) */

          gd32_setwindow(priv, window);
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_wwdg_initialize
 *
 * Description:
 *   Initialize the WWDGT watchdog timer.  The watchdog timer is initialized
 *   and registers as 'devpath'.  The initial state of the watchdog timer is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_wwdg_initialize(const char *devpath)
{
  struct gd32_wwdgt_lowerhalf_s *priv = &g_wwdgt_dev;
  uint32_t regval;

  wdinfo("Entry: devpath=%s\n", devpath);

  /* Enable WWDGT clock on APB1 bus */

  regval = getreg32(GD32_RCU_APB1EN);
  regval |= RCU_APB1EN_WWDGTEN;
  putreg32(regval, GD32_RCU_APB1EN);
  wdinfo("WWDGT clock enabled (RCU_APB1EN: 0x%08lx)\n", regval);

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_wwdgt_ops;

  /* Attach our EWI interrupt handler (But don't enable it yet) */

  irq_attach(GD32_IRQ_WWDGT, gd32_wwdgt_interrupt, NULL);

  /* Select an arbitrary initial timeout value.  But don't start the
   * watchdog yet. NOTE: If the "Hardware watchdog" feature is enabled
   * through the device option bits, the watchdog is automatically enabled
   * at power-on.
   */

  gd32_wwdgt_settimeout((struct watchdog_lowerhalf_s *)priv,
                        CONFIG_GD32E11X_WWDG_DEFTIMOUT);

  /* Set default window value (0x7f = disabled, allows feeding anytime) */

  gd32_setwindow(priv, CONFIG_GD32E11X_WWDG_WINDOW);
  wdinfo("Default window: 0x%02x\n", (unsigned)CONFIG_GD32E11X_WWDG_WINDOW);

  /* Register the watchdog driver as /dev/watchdogN */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);

  /* When the microcontroller enters debug mode (Cortex-M4F core halted),
   * the WWDGT counter either continues to work normally or stops, depending
   * on DBG_CTL WWDGT_HOLD configuration bit in DBG module.
   */

#ifdef CONFIG_GD32E11X_WWDG_DBGFREEZE
{
    uint32_t cr = getreg32(GD32_DBG_CTL);
    cr |= DBG_CTL_WWDGT_HOLD;
    putreg32(cr, GD32_DBG_CTL);
    wdinfo("Debug freeze enabled: WWDGT will stop during debug\n");
}
#else
  wdinfo("Debug freeze disabled: WWDGT runs during debug\n");
#endif
}

#endif /* CONFIG_WATCHDOG && CONFIG_GD32E11X_WWDG */
