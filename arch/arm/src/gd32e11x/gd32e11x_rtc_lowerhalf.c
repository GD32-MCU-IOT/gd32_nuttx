/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_rtc_lowerhalf.c
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
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/rtc.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32e11x_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GD32_NALARMS 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct gd32_cbinfo_s
{
    volatile rtc_alarm_callback_t cb;
    volatile void *priv;
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct gd32_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;      /* Threads can only exclusively access the RTC */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

    struct gd32_cbinfo_s cbinfo[GD32_NALARMS];
#endif

#ifdef CONFIG_RTC_PERIODIC
  /* Periodic wakeup information */

  struct lower_setperiodic_s periodic;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int gd32_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime);
static int gd32_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime);
static bool gd32_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int gd32_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo);
static int gd32_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo);
static int gd32_cancelalarm(struct rtc_lowerhalf_s *lower,
                             int alarmid);
static int gd32_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int gd32_setperiodic(struct rtc_lowerhalf_s *lower,
                             const struct lower_setperiodic_s *alarminfo);
static int gd32_cancelperiodic(struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GD32 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
    .rdtime      = gd32_rdtime,
    .settime     = gd32_settime,
    .havesettime = gd32_havesettime,
#ifdef CONFIG_RTC_ALARM
    .setalarm    = gd32_setalarm,
    .setrelative = gd32_setrelative,
    .cancelalarm = gd32_cancelalarm,
    .rdalarm     = gd32_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = gd32_setperiodic,
  .cancelperiodic = gd32_cancelperiodic,
#endif
};

/* GD32 RTC device state */

static struct gd32_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_alarm_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the alarm
 *   goes off.  It just invokes the upper half drivers callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void gd32_alarm_callback(void)
{
  struct gd32_cbinfo_s *cbinfo = &g_rtc_lowerhalf.cbinfo[0];

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  rtc_alarm_callback_t cb = (rtc_alarm_callback_t)cbinfo->cb;
  void *arg               = (void *)cbinfo->priv;

  cbinfo->cb              = NULL;
  cbinfo->priv            = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(arg, 0);
    }
}

#endif

/****************************************************************************
 * Name: gd32_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int gd32_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime)
{
#if defined(CONFIG_RTC_DATETIME)
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return up_rtc_getdatetime((struct tm *)rtctime);

#elif defined(CONFIG_RTC_HIRES)
  struct timespec ts;
  int ret;

  /* Get the higher resolution time */

  ret = up_rtc_gettime(&ts);
  if (ret < 0)
    {
      goto errout;
    }

  /* Convert the one second epoch time to a struct tm.  This operation
   * depends on the fact that struct rtc_time and struct tm are cast
   * compatible.
   */

  if (!gmtime_r(&ts.tv_sec, (struct tm *)rtctime))
    {
      ret = -get_errno();
      goto errout;
    }

  return OK;

errout:
  DEBUGASSERT(ret < 0);
  return ret;

#else
  time_t timer;

  /* The resolution of time is only 1 second */

  timer = up_rtc_time();

  /* Convert the one second epoch time to a struct tm */

  if (!gmtime_r(&timer, (struct tm *)rtctime))
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  return OK;
#endif
}

/****************************************************************************
 * Name: gd32_settime
 *
 * Description:
 *   Implements the settime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The new time to set
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int gd32_settime(struct rtc_lowerhalf_s *lower,
                        const struct rtc_time *rtctime)
{
#ifdef CONFIG_RTC_DATETIME
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return gd32_rtc_setdatetime((const struct tm *)rtctime);

#else
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = timegm((struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  return up_rtc_settime(&ts);
#endif
}

/****************************************************************************
 * Name: gd32_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

static bool gd32_havesettime(struct rtc_lowerhalf_s *lower)
{
  return getreg16(RTC_MAGIC_REG) == RTC_MAGIC_TIME_SET;
}

/****************************************************************************
 * Name: gd32_setalarm
 *
 * Description:
 *   Set a new alarm.  This function implements the setalarm() method of the
 *   RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int gd32_setalarm(struct rtc_lowerhalf_s *lower,
                         const struct lower_setalarm_s *alarminfo)
{
  struct gd32_lowerhalf_s *priv;
  struct gd32_cbinfo_s *cbinfo;

  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (struct gd32_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == 0)
    {
      struct timespec ts;

      /* Convert the RTC time to a timespec (1 second accuracy) */

      ts.tv_sec   = timegm((struct tm *)&alarminfo->time);
      ts.tv_nsec  = 0;

      /* Remember the callback information */

      cbinfo            = &priv->cbinfo[0];
      cbinfo->cb        = alarminfo->cb;
      cbinfo->priv      = alarminfo->priv;

      /* And set the alarm */

      ret = gd32_rtc_setalarm(&ts, gd32_alarm_callback);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_setrelative
 *
 * Description:
 *   Set a new alarm relative to the current time.  This function implements
 *   the setrelative() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int gd32_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo)
{
  struct gd32_lowerhalf_s *priv;
  struct gd32_cbinfo_s *cbinfo;
#if defined(CONFIG_RTC_DATETIME)
  struct tm time;
#endif
  struct timespec ts;
  int ret = -EINVAL;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (struct gd32_lowerhalf_s *)lower;

  if (alarminfo->id == 0 && alarminfo->reltime > 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      flags = enter_critical_section();

      /* Get the current time in seconds */

#if defined(CONFIG_RTC_DATETIME)
      /* Get the broken out time and convert to seconds */

      ret = up_rtc_getdatetime(&time);
      if (ret < 0)
        {
          leave_critical_section(flags);
          return ret;
        }

      ts.tv_sec  = timegm(&time);
      ts.tv_nsec = 0;

#elif defined(CONFIG_RTC_HIRES)
      /* Get the higher resolution time */

      ret = up_rtc_gettime(&ts);
      if (ret < 0)
        {
          leave_critical_section(flags);
          return ret;
        }
#else
      /* The resolution of time is only 1 second */

      ts.tv_sec  = up_rtc_time();
      ts.tv_nsec = 0;
#endif

        ts.tv_sec   += alarminfo->reltime;

      /* Remember the callback information */

      cbinfo       = &priv->cbinfo[0];
      cbinfo->cb   = alarminfo->cb;
      cbinfo->priv = alarminfo->priv;

      /* And set the alarm */

      ret = gd32_rtc_setalarm(&ts, gd32_alarm_callback);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }

      leave_critical_section(flags);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int gd32_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  struct gd32_lowerhalf_s *priv;
  struct gd32_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (struct gd32_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo[0];
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  return gd32_rtc_cancelalarm();
}
#endif

/****************************************************************************
 * Name: gd32_rdalarm
 *
 * Description:
 *   Query the RTC alarm.
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to query the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int gd32_rdalarm(struct rtc_lowerhalf_s *lower,
                        struct lower_rdalarm_s *alarminfo)
{
  struct alm_rdalarm_s lowerinfo;
  int ret = -EINVAL;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);

  DEBUGASSERT(alarminfo->id >= 0 && alarminfo->id < CONFIG_RTC_NALARMS);

  if (alarminfo != NULL && alarminfo->id >= 0 &&
      alarminfo->id < CONFIG_RTC_NALARMS)

    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      flags = enter_critical_section();

      lowerinfo.ar_id   = alarminfo->id;
      lowerinfo.ar_time = alarminfo->time;

      ret = gd32_rtc_rdalarm(&lowerinfo);

      leave_critical_section(flags);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_periodic_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the
 *   periodic wakeup goes off.  It just invokes the upper half drivers
 *   callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int gd32_periodic_callback(void)
{
  struct gd32_lowerhalf_s *lower;
  struct lower_setperiodic_s *cbinfo;
  rtc_wakeup_callback_t cb;
  void *priv;

  lower = (struct gd32_lowerhalf_s *)&g_rtc_lowerhalf;

  cbinfo = &lower->periodic;
  cb     = (rtc_wakeup_callback_t)cbinfo->cb;
  priv   = (void *)cbinfo->priv;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, 0);
    }

  return OK;
}
#endif /* CONFIG_RTC_PERIODIC */

/****************************************************************************
 * Name: gd32_setperiodic
 *
 * Description:
 *   Set a new periodic wakeup relative to the current time, with a given
 *   period. This function implements the setperiodic() method of the RTC
 *   driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the wakeup activity
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int gd32_setperiodic(struct rtc_lowerhalf_s *lower,
                             const struct lower_setperiodic_s *alarminfo)
{
  struct gd32_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (struct gd32_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));
  ret = gd32_rtc_setperiodic(&alarminfo->period, gd32_periodic_callback);

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_cancelperiodic
 *
 * Description:
 *   Cancel the current periodic wakeup activity.  This function implements
 *   the cancelperiodic() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int gd32_cancelperiodic(struct rtc_lowerhalf_s *lower, int id)
{
  struct gd32_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (struct gd32_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_rtc_cancelperiodic();

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the GD32.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "gd32e11x_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = gd32e11x_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *gd32_rtc_lowerhalf(void)
{
  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
