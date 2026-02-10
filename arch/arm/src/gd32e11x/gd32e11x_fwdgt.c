/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_fwdgt.c
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

#include <inttypes.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "gd32e11x_rcu.h"
#include "hardware/gd32e11x_wdg.h"
#include "hardware/gd32e11x_dbg.h"
#include "gd32e11x_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_GD32E11X_FWDGT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The minimum frequency of the FWDGT clock is:
 *
 *  Fmin = FIRC40K / 256
 *
 * So the maximum delay (in milliseconds) is then:
 *
 *   1000 * FWDGT_RLD_MAX / Fmin
 *
 * For example, if FIRC40K = 40KHz (the nominal, uncalibrated value), then
 * the maximum delay is:
 *
 *   Fmin = 156.25
 *   1000 * 4095 / Fmin = 26,208 msec
 */

#define FWDGT_FMIN       (40000 / 256)  /* Nominal IRC40K = 40KHz */
#define FWDGT_MAXTIMEOUT (1000 * FWDGT_RLD_MAX / FWDGT_FMIN)

/* Configuration ************************************************************/

#ifndef CONFIG_GD32E11X_FWDGT_DEFTIMOUT
#  define CONFIG_GD32E11X_FWDGT_DEFTIMOUT FWDGT_MAXTIMEOUT
#endif

#ifndef CONFIG_DEBUG_WATCHDOG_INFO
#  undef CONFIG_GD32E11X_FWDGT_REGDEBUG
#endif

/* REVISIT:  It appears that you can only setup the prescaler and reload
 * registers once.  After that, the STAT register's PUD and RUD bits never
 * go to zero.  So we defer setting up these registers until the watchdog
 * is started, then refuse any further attempts to change timeout.
 */

#define CONFIG_GD32E11X_FWDGT_ONETIMESETUP 1

/* REVISIT:  Another possibility is that we CAN change the prescaler and
 * reload values after starting the timer.  This option is untested but the
 * implementation place conditioned on the following:
 */

#undef CONFIG_GD32E11X_FWDGT_DEFERREDSETUP

/* But you can only try one at a time */

#if defined(CONFIG_GD32E11X_FWDGT_ONETIMESETUP) && \
    defined(CONFIG_GD32E11X_FWDGT_DEFERREDSETUP)
#  error "Both CONFIG_GD32E11X_FWDGT_ONETIMESETUP and CONFIG_GD32E11X_FWDGT_DEFERREDSETUP are defined"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct gd32_fwdgt_lowerhalf_s
{
  const struct watchdog_ops_s  *ops;        /* Lower half operations */
  uint32_t irc40kfreq;                      /* The calibrated frequency of the IRC40K oscillator */
  uint32_t timeout;                         /* The (actual) selected timeout */
  uint32_t lastreset;                       /* The last reset time */
  bool     started;                         /* true: The watchdog timer has been started */
  uint8_t  prescaler;                       /* Clock prescaler value */
  uint16_t reload;                          /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_GD32E11X_FWDGT_REGDEBUG
static uint16_t gd32_getreg(uint32_t addr);
static void     gd32_putreg(uint16_t val, uint32_t addr);
#else
#  define       gd32_getreg(addr)     getreg16(addr)
#  define       gd32_putreg(val,addr) putreg16(val,addr)
#endif

static inline void gd32_setprescaler(
                     struct gd32_fwdgt_lowerhalf_s *priv);

/* "Lower half" driver methods **********************************************/

static int      gd32_fwdgt_start(struct watchdog_lowerhalf_s *lower);
static int      gd32_fwdgt_stop(struct watchdog_lowerhalf_s *lower);
static int      gd32_fwdgt_keepalive(struct watchdog_lowerhalf_s *lower);
static int      gd32_fwdgt_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status);
static int      gd32_fwdgt_settimeout(struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_fwdgt_ops =
{
  .start      = gd32_fwdgt_start,
  .stop       = gd32_fwdgt_stop,
  .keepalive  = gd32_fwdgt_keepalive,
  .getstatus  = gd32_fwdgt_getstatus,
  .settimeout = gd32_fwdgt_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct gd32_fwdgt_lowerhalf_s g_fwdgt_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_getreg
 *
 * Description:
 *   Get the contents of a GD32 FWDGT register
 *
 ****************************************************************************/

#ifdef CONFIG_GD32E11X_FWDGT_REGDEBUG
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

#ifdef CONFIG_GD32E11X_FWDGT_REGDEBUG
static void gd32_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  wdinfo("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: gd32_setprescaler
 *
 * Description:
 *   Set up the prescaler and reload values.  This seems to be something
 *   that can only be done one time.
 *
 * Input Parameters:
 *   priv   - A pointer the internal representation of the "lower-half"
 *             driver state structure.
 *
 ****************************************************************************/

static inline void gd32_setprescaler(
                     struct gd32_fwdgt_lowerhalf_s *priv)
{
  /* Enable write access to FWDGT_PSC and FWDGT_RLD registers */

  gd32_putreg(FWDGT_CTL_CMD_UNLOCK, GD32_FWDGT_CTL);

  /* Wait for the PUD and RUD bits to be reset by hardware.  These bits
   * were set the last time that the PSC register was written and may not
   * yet be cleared.
   *
   * If the setup is only permitted one time, then this wait should not
   * be necessary.
   */

#ifndef CONFIG_GD32E11X_FWDGT_ONETIMESETUP
  while ((gd32_getreg(GD32_FWDGT_STAT) &
         (FWDGT_STAT_PUD | FWDGT_STAT_RUD)) != 0);
#endif

  /* Set the prescaler */

  gd32_putreg((uint16_t)priv->prescaler << FWDGT_PSC_SHIFT,
              GD32_FWDGT_PSC);

  /* Set the reload value */

  gd32_putreg((uint16_t)priv->reload, GD32_FWDGT_RLD);

  /* Reload the counter (and disable write access) */

  gd32_putreg(FWDGT_CTL_CMD_RELOAD, GD32_FWDGT_CTL);
}

/****************************************************************************
 * Name: gd32_fwdgt_start
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

static int gd32_fwdgt_start(struct watchdog_lowerhalf_s *lower)
{
  struct gd32_fwdgt_lowerhalf_s *priv =
    (struct gd32_fwdgt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry: started\n");
  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      /* REVISIT: It appears that you can only setup the prescaler and
       * reload registers once. After that, the STAT register's PUD and RUD
       * bits never go to 0. So we defer setting up these registers until
       * the watchdog is started, then refuse any further attempts to
       * change timeout.
       */

      /* Set up prescaler and reload value for the selected timeout before
       * starting the watchdog timer.
       */

#if defined(CONFIG_GD32E11X_FWDGT_ONETIMESETUP) || \
    defined(CONFIG_GD32E11X_FWDGT_DEFERREDSETUP)
      gd32_setprescaler(priv);
#endif

      /* Enable FWDGT (the IRC40K oscillator will be enabled by hardware).
       * NOTE: If the "Hardware watchdog" feature is enabled through the
       * device option bits, the watchdog is automatically enabled at
       * power-on.
       */

      flags           = enter_critical_section();
      gd32_putreg(FWDGT_CTL_CMD_ENABLE, GD32_FWDGT_CTL);
      priv->lastreset = clock_systime_ticks();
      priv->started   = true;
      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_fwdgt_stop
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

static int gd32_fwdgt_stop(struct watchdog_lowerhalf_s *lower)
{
  /* There is no way to disable the FWDGT timer once it has been started */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: gd32_fwdgt_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_fwdgt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct gd32_fwdgt_lowerhalf_s *priv =
    (struct gd32_fwdgt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  /* Reload the FWDGT timer */

  flags = enter_critical_section();
  gd32_putreg(FWDGT_CTL_CMD_RELOAD, GD32_FWDGT_CTL);
  priv->lastreset = clock_systime_ticks();
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: gd32_fwdgt_getstatus
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

static int gd32_fwdgt_getstatus(struct watchdog_lowerhalf_s *lower,
                                struct watchdog_status_s *status)
{
  struct gd32_fwdgt_lowerhalf_s *priv =
    (struct gd32_fwdgt_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systime_ticks() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08" PRIx32 "\n", status->flags);
  wdinfo("  timeout  : %" PRId32 "\n", status->timeout);
  wdinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: gd32_fwdgt_settimeout
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

static int gd32_fwdgt_settimeout(struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout)
{
  struct gd32_fwdgt_lowerhalf_s *priv =
    (struct gd32_fwdgt_lowerhalf_s *)lower;
  uint32_t ffwdgt;
  uint64_t reload;
  int prescaler;
  int shift;

  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);
  DEBUGASSERT(priv);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > FWDGT_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%" PRId32 " > %d\n",
            timeout, FWDGT_MAXTIMEOUT);
      return -ERANGE;
    }

  /* REVISIT:  It appears that you can only setup the prescaler and reload
   * registers once.  After that, the STAT register's PUD and RUD bits
   * never go to zero.
   */

#ifdef CONFIG_GD32E11X_FWDGT_ONETIMESETUP
  if (priv->started)
    {
      wdwarn("WARNING: Timer is already started\n");
      return -EBUSY;
    }
#endif

  /* Select the smallest prescaler that will result in a reload value that
   * is less than the maximum.
   */

  for (prescaler = 0; ; prescaler++)
    {
      /* PSC = 0 -> Divider = 4   = 1 << 2
       * PSC = 1 -> Divider = 8   = 1 << 3
       * PSC = 2 -> Divider = 16  = 1 << 4
       * PSC = 3 -> Divider = 32  = 1 << 5
       * PSC = 4 -> Divider = 64  = 1 << 6
       * PSC = 5 -> Divider = 128 = 1 << 7
       * PSC = 6 -> Divider = 256 = 1 << 8
       * PSC = n -> Divider       = 1 << (n+2)
       */

      shift = prescaler + 2;

      /* Get the FWDGT counter frequency in Hz. For a nominal 40KHz IRC40K
       * clock, this is value in the range of 10000 and 156.25.
       */

      ffwdgt = priv->irc40kfreq >> shift;

      /* We want:
       *  1000 * reload / Ffwdgt = timeout
       * Or:
       *  reload = Ffwdgt * timeout / 1000
       */

      reload = (uint64_t)ffwdgt * (uint64_t)timeout / 1000;

      /* If this reload valid is less than the maximum or we are not ready
       * at the prescaler value, then break out of the loop to use these
       * settings.
       */

      if (reload <= FWDGT_RLD_MAX || prescaler == 6)
        {
          /* Note that we explicitly break out of the loop rather than
           * using the 'for' loop termination logic because we do not want
           * the value of prescaler to be incremented.
           */

          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > FWDGT_RLD_MAX)
    {
      reload = FWDGT_RLD_MAX;
    }

  /* Get the actual timeout value in milliseconds.
   *
   * We have:
   *  reload = Ffwdgt * timeout / 1000
   * So we want:
   *  timeout = 1000 * reload / Ffwdgt
   */

  priv->timeout = (1000 * (uint32_t)reload) / ffwdgt;

  /* Save setup values for later use */

  priv->prescaler = prescaler;
  priv->reload    = reload;

  /* Write the prescaler and reload values to the FWDGT registers.
   *
   * REVISIT:  It appears that you can only setup the prescaler and reload
   * registers once.  After that, the STAT register's PUD and RUD bits
   * never go to zero.
   */

#ifndef CONFIG_GD32E11X_FWDGT_ONETIMESETUP
  /* If CONFIG_GD32E11X_FWDGT_DEFERREDSETUP is selected, then perform the
   * register configuration only if the timer has been started.
   */

#ifdef CONFIG_GD32E11X_FWDGT_DEFERREDSETUP
  if (priv->started)
#endif
    {
      gd32_setprescaler(priv);
    }
#endif

  wdinfo("prescaler=%d ffwdgt=%" PRId32 " reload=%" PRId64 "\n",
         prescaler, ffwdgt, reload);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fwdgt_initialize
 *
 * Description:
 *   Initialize the FWDGT watchdog timer.  The watchdog timer is initialized
 *   and registers as 'devpath'.  The initial state of the watchdog timer is
 *   disabled.
 *
 * Input Parameters:
 *   devpath   - The full path to the watchdog.  This should be of the form
 *               /dev/watchdog0
 *   irc40kfreq - The calibrated IRC40K clock frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_fwdgt_initialize(const char *devpath, uint32_t irc40kfreq)
{
  struct gd32_fwdgt_lowerhalf_s *priv = &g_fwdgt_dev;

  wdinfo("Entry: devpath=%s irc40kfreq=%" PRId32 "\n",
         devpath, irc40kfreq);

  /* Initialize the driver state structure. */

  priv->ops         = &g_fwdgt_ops;
  priv->irc40kfreq  = irc40kfreq;
  priv->started     = false;

  /* Select an arbitrary initial timeout value.  But don't start the
   * watchdog yet. NOTE: If the "Hardware watchdog" feature is enabled
   * through the device option bits, the watchdog is automatically enabled
   * at power-on.
   */

  gd32_fwdgt_settimeout((struct watchdog_lowerhalf_s *)priv,
                        CONFIG_GD32E11X_FWDGT_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdogN */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);

  /* When the microcontroller enters debug mode (Cortex-M4F core halted),
   * the FWDGT counter either continues to work normally or stops, depending
   * on DBG_CTL FWDGT_HOLD configuration bit in DBG module.
   */

#ifdef CONFIG_GD32E11X_FWDGT_DBGFREEZE
{
    uint32_t cr = getreg32(GD32_DBG_CTL);
    cr |= DBG_CTL_FWDGT_HOLD;
    putreg32(cr, GD32_DBG_CTL);
    wdinfo("Debug freeze enabled: FWDGT will stop during debug\n");
}
#else
  wdinfo("Debug freeze disabled: FWDGT runs during debug\n");
#endif
}

#endif /* CONFIG_WATCHDOG && CONFIG_GD32E11X_FWDGT */
