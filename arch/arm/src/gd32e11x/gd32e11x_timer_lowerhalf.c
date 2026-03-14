/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_timer_lowerhalf.c
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

#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "gd32e11x.h"
#include "gd32e11x_timer.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_GD32E11X_TIMER0)  || defined(CONFIG_GD32E11X_TIMER1)  || \
     defined(CONFIG_GD32E11X_TIMER2)  || defined(CONFIG_GD32E11X_TIMER3)  || \
     defined(CONFIG_GD32E11X_TIMER4)  || defined(CONFIG_GD32E11X_TIMER5)  || \
     defined(CONFIG_GD32E11X_TIMER6)  || defined(CONFIG_GD32E11X_TIMER7)  || \
     defined(CONFIG_GD32E11X_TIMER8)  || defined(CONFIG_GD32E11X_TIMER9)  || \
     defined(CONFIG_GD32E11X_TIMER10) || defined(CONFIG_GD32E11X_TIMER11) || \
     defined(CONFIG_GD32E11X_TIMER12) || defined(CONFIG_GD32E11X_TIMER13))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All GD32E11X timers are 16-bit */

#define GD32_TIMER_RESOLUTION  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct gd32_lowerhalf_s
{
  const struct timer_ops_s  *ops;        /* Lower half operations */
  struct gd32_timer_dev_s   *tim;        /* GD32 timer driver */
  tccb_t                     callback;   /* Current user interrupt callback */
  void                      *arg;        /* Argument passed to callback */
  bool                       started;    /* True: Timer has been started */
  const uint8_t              resolution; /* Timer resolution in bits */
  uint32_t                   timeout;    /* Current timeout value (us) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gd32_timer_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods */

static int  gd32_timer_start(struct timer_lowerhalf_s *lower);
static int  gd32_timer_stop(struct timer_lowerhalf_s *lower);
static int  gd32_timer_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status);
static int  gd32_timer_settimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t timeout);
static void gd32_timer_setcallback(struct timer_lowerhalf_s *lower,
                                   tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = gd32_timer_start,
  .stop        = gd32_timer_stop,
  .getstatus   = gd32_timer_getstatus,
  .settimeout  = gd32_timer_settimeout,
  .setcallback = gd32_timer_setcallback,
  .ioctl       = NULL,
};

#ifdef CONFIG_GD32E11X_TIMER0
static struct gd32_lowerhalf_s g_timer0_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER1
static struct gd32_lowerhalf_s g_timer1_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER2
static struct gd32_lowerhalf_s g_timer2_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER3
static struct gd32_lowerhalf_s g_timer3_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER4
static struct gd32_lowerhalf_s g_timer4_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER5
static struct gd32_lowerhalf_s g_timer5_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER6
static struct gd32_lowerhalf_s g_timer6_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER7
static struct gd32_lowerhalf_s g_timer7_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER8
static struct gd32_lowerhalf_s g_timer8_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER9
static struct gd32_lowerhalf_s g_timer9_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER10
static struct gd32_lowerhalf_s g_timer10_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER11
static struct gd32_lowerhalf_s g_timer11_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER12
static struct gd32_lowerhalf_s g_timer12_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

#ifdef CONFIG_GD32E11X_TIMER13
static struct gd32_lowerhalf_s g_timer13_lowerhalf =
{
  .ops        = &g_timer_ops,
  .resolution = GD32_TIMER_RESOLUTION,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_timer_handler
 *
 * Description:
 *   Timer interrupt handler.  Called when the timer update interrupt fires.
 *   Acknowledges the interrupt and invokes the registered callback.
 *
 ****************************************************************************/

static int gd32_timer_handler(int irq, void *context, void *arg)
{
  struct gd32_lowerhalf_s *lower = (struct gd32_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;

  /* Acknowledge the update interrupt */

  GD32_TIMER_ACKINT(lower->tim, GD32_TIMER_INT_UP);

  /* Invoke the callback.  The callback returns true to continue the timer,
   * or false to stop it.  If next_interval_us is set to a non-zero value,
   * the timer period is updated.
   */

  if (lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          GD32_TIMER_SETPERIOD(lower->tim, next_interval_us);
        }
    }
  else
    {
      gd32_timer_stop((struct timer_lowerhalf_s *)lower);
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_getstatus
 *
 * Description:
 *   Get the current timer status.
 *
 ****************************************************************************/

static int gd32_timer_getstatus(struct timer_lowerhalf_s *lower,
                                struct timer_status_s *status)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  uint32_t period;
  uint32_t clock;
  uint32_t counter;

  DEBUGASSERT(priv != NULL && status != NULL);

  /* Return the status bit */

  status->flags = 0;

  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Return the timeout value in microseconds */

  status->timeout = priv->timeout;

  /* Compute the time remaining (in microseconds) */

  period  = GD32_TIMER_GETPERIOD(priv->tim);
  clock   = GD32_TIMER_GETCLOCK(priv->tim);
  counter = GD32_TIMER_GETCOUNTER(priv->tim);

  if (clock > 0 && period > 0)
    {
      /* timeleft = (period - counter) / clock * 1000000 */

      status->timeleft = (uint32_t)(((uint64_t)(period - counter)
                                     * 1000000ull) / clock);
    }
  else
    {
      status->timeleft = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_start
 *
 * Description:
 *   Start the timer, configuring the timer for up-counting mode and
 *   enabling the update interrupt if a callback has been registered.
 *
 ****************************************************************************/

static int gd32_timer_start(struct timer_lowerhalf_s *lower)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;

  if (!priv->started)
    {
      GD32_TIMER_SETMODE(priv->tim, GD32_TIMER_COUNTER_UP);

      if (priv->callback != NULL)
        {
          GD32_TIMER_SETISR(priv->tim, gd32_timer_handler, priv,
                            GD32_TIMER_INT_UP);
          GD32_TIMER_ENABLEINT(priv->tim, GD32_TIMER_INT_UP);
        }

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: gd32_timer_stop
 *
 * Description:
 *   Stop the timer and disable the update interrupt.
 *
 ****************************************************************************/

static int gd32_timer_stop(struct timer_lowerhalf_s *lower)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;

  if (priv->started)
    {
      GD32_TIMER_SETMODE(priv->tim, GD32_TIMER_MODE_DISABLED);
      GD32_TIMER_DISABLEINT(priv->tim, GD32_TIMER_INT_UP);
      GD32_TIMER_SETISR(priv->tim, NULL, NULL, GD32_TIMER_INT_UP);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: gd32_timer_settimeout
 *
 * Description:
 *   Set a new timeout value in microseconds.  The timer prescaler and
 *   period are calculated to achieve the requested timeout.
 *
 *   For a 16-bit timer at 1 MHz clock:
 *     max period = 65535 us (65.535 ms)
 *
 *   For longer timeouts, the clock frequency is reduced:
 *     freq = (max_count * 1000000) / timeout
 *
 * Input Parameters:
 *   lower   - Lower half timer driver
 *   timeout - New timeout value in microseconds
 *
 ****************************************************************************/

static int gd32_timer_settimeout(struct timer_lowerhalf_s *lower,
                                 uint32_t timeout)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  uint64_t maxtimeout;

  if (priv->started)
    {
      return -EPERM;
    }

  maxtimeout = (1ull << priv->resolution) - 1;

  if (timeout > maxtimeout)
    {
      /* For long timeouts, reduce the clock frequency */

      uint64_t freq = (maxtimeout * 1000000ull) / timeout;
      if (freq == 0)
        {
          freq = 1;
        }

      GD32_TIMER_SETCLOCK(priv->tim, (uint32_t)freq);
      GD32_TIMER_SETPERIOD(priv->tim, (uint32_t)maxtimeout);
    }
  else
    {
      /* For short timeouts, use 1 MHz clock (1 tick = 1 us) */

      GD32_TIMER_SETCLOCK(priv->tim, 1000000);
      GD32_TIMER_SETPERIOD(priv->tim, timeout);
    }

  /* Save the timeout value for getstatus() */

  priv->timeout = timeout;

  return OK;
}

/****************************************************************************
 * Name: gd32_timer_setcallback
 *
 * Description:
 *   Set the timer expiration callback function.  If callback is NULL,
 *   the interrupt is disabled.
 *
 ****************************************************************************/

static void gd32_timer_setcallback(struct timer_lowerhalf_s *lower,
                                   tccb_t callback, void *arg)
{
  struct gd32_lowerhalf_s *priv = (struct gd32_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Save the new callback and argument */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started)
    {
      GD32_TIMER_SETISR(priv->tim, gd32_timer_handler, priv,
                        GD32_TIMER_INT_UP);
      GD32_TIMER_ENABLEINT(priv->tim, GD32_TIMER_INT_UP);
    }
  else
    {
      GD32_TIMER_DISABLEINT(priv->tim, GD32_TIMER_INT_UP);
      GD32_TIMER_SETISR(priv->tim, NULL, NULL, GD32_TIMER_INT_UP);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device (e.g., /dev/timer0)
 *   timer   - The timer number (0-13 for GD32E11X)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int gd32_timer_initialize(const char *devpath, int timer)
{
  struct gd32_lowerhalf_s *lower;

  switch (timer)
    {
#ifdef CONFIG_GD32E11X_TIMER0
      case 0:
        lower = &g_timer0_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER1
      case 1:
        lower = &g_timer1_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER2
      case 2:
        lower = &g_timer2_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER3
      case 3:
        lower = &g_timer3_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER4
      case 4:
        lower = &g_timer4_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER5
      case 5:
        lower = &g_timer5_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER6
      case 6:
        lower = &g_timer6_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER7
      case 7:
        lower = &g_timer7_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER8
      case 8:
        lower = &g_timer8_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER9
      case 9:
        lower = &g_timer9_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER10
      case 10:
        lower = &g_timer10_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER11
      case 11:
        lower = &g_timer11_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER12
      case 12:
        lower = &g_timer12_lowerhalf;
        break;
#endif
#ifdef CONFIG_GD32E11X_TIMER13
      case 13:
        lower = &g_timer13_lowerhalf;
        break;
#endif
      default:
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->timeout  = 0;
  lower->tim      = gd32_timer_init(timer);

  if (lower->tim == NULL)
    {
      return -EINVAL;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   */

  void *drvr = timer_register(devpath,
                               (struct timer_lowerhalf_s *)lower);
  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * or perhaps a failure to register the timer driver (such as if the
       * 'devpath' were not unique).
       */

      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_TIMER */
