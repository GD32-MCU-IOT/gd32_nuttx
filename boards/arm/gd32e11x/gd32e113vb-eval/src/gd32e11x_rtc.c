/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_rtc.c
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
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <syslog.h>
#include <sys/ioctl.h>

#include <nuttx/clock.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/kthread.h>

#include "gd32e113v_eval.h"

#ifdef CONFIG_GD32E113VB_EVAL_RTCTEST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTCTEST_DEVPATH         "/dev/rtc0"
#define RTCTEST_SIGNO           13
#define RTCTEST_COUNTER_SECS    2
#define RTCTEST_ALARM_SECS      5
#define RTCTEST_DEVIATION_MS    1500
#define RTCTEST_COUNTER_TOL     3
#define RTCTEST_THREAD_STACKSZ  4096
#define RTCTEST_THREAD_PRIO     100

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static time_t rtctest_to_time(FAR const struct rtc_time *rt)
{
  return timegm((FAR struct tm *)rt);
}

static void rtctest_add_seconds(FAR struct rtc_time *rt, time_t secs)
{
  time_t t;
  FAR struct tm *tmp;

  t   = rtctest_to_time(rt) + secs;
  tmp = gmtime(&t);

  rt->tm_sec   = tmp->tm_sec;
  rt->tm_min   = tmp->tm_min;
  rt->tm_hour  = tmp->tm_hour;
  rt->tm_mday  = tmp->tm_mday;
  rt->tm_mon   = tmp->tm_mon;
  rt->tm_year  = tmp->tm_year;
  rt->tm_wday  = tmp->tm_wday;
  rt->tm_yday  = tmp->tm_yday;
  rt->tm_isdst = tmp->tm_isdst;
}

static uint32_t rtctest_monotonic_ms(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void rtctest_print_time(FAR const char *tag,
                               FAR const struct rtc_time *rt)
{
  syslog(LOG_INFO, "[RTC Test] %s  %04d-%02d-%02d %02d:%02d:%02d\n",
         tag,
         rt->tm_year + TM_YEAR_BASE, rt->tm_mon + 1, rt->tm_mday,
         rt->tm_hour, rt->tm_min, rt->tm_sec);
}

static int rtctest_open(void)
{
  int fd;

  fd = open(RTCTEST_DEVPATH, O_RDWR);
  if (fd < 0)
    {
      syslog(LOG_ERR, "[RTC Test] open %s failed: %d\n",
             RTCTEST_DEVPATH, errno);
      return -errno;
    }

  return fd;
}

/****************************************************************************
 * Name: rtctest_run_counter
 *
 * Description:
 *   Test 1 - Counter test:  RTC_SET_TIME / RTC_HAVE_SET_TIME / RTC_RD_TIME
 *   and verify the counter advances over a 2-second sleep.
 ****************************************************************************/

static int rtctest_run_counter(void)
{
  struct rtc_time base;
  struct rtc_time rd;
  struct rtc_time later;
  time_t base_s;
  time_t rd_s;
  time_t later_s;
  long delta;
  bool have;
  int fd;
  int ret;

  syslog(LOG_INFO, "[RTC Test] Test 1: Counter read / write\n");

  fd = rtctest_open();
  if (fd < 0)
    {
      return fd;
    }

  /* Set base time to 2000-01-01 00:00:00 */

  memset(&base, 0, sizeof(base));
  base.tm_year = 2000 - TM_YEAR_BASE;
  base.tm_mon  = TM_JANUARY;
  base.tm_mday = 1;
  base.tm_wday = TM_SATURDAY;

  ret = ioctl(fd, RTC_SET_TIME, (unsigned long)((uintptr_t)&base));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_SET_TIME FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  syslog(LOG_INFO, "[RTC Test]   RTC_SET_TIME PASSED\n");

  /* Check RTC_HAVE_SET_TIME */

  ret = ioctl(fd, RTC_HAVE_SET_TIME, (unsigned long)((uintptr_t)&have));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_HAVE_SET_TIME ioctl FAILED: %d\n",
             errno);

      close(fd);
      return -errno;
    }

  if (!have)
    {
      syslog(LOG_ERR,
             "[RTC Test]   RTC_HAVE_SET_TIME returned false FAILED\n");
      close(fd);
      return -EIO;
    }

  syslog(LOG_INFO, "[RTC Test]   RTC_HAVE_SET_TIME PASSED\n");

  /* Read back and compare */

  ret = ioctl(fd, RTC_RD_TIME, (unsigned long)((uintptr_t)&rd));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_RD_TIME FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  rtctest_print_time("set ", &base);
  rtctest_print_time("read", &rd);

  base_s = rtctest_to_time(&base);
  rd_s   = rtctest_to_time(&rd);
  delta  = (long)(rd_s - base_s);

  if (delta < 0 || delta > 1)
    {
      syslog(LOG_ERR,
             "[RTC Test]   set-to-read delta=%ld FAILED (expect 0..1)\n",
             delta);
      close(fd);
      return -ERANGE;
    }

  syslog(LOG_INFO,
         "[RTC Test]   set-to-read delta=%ld PASSED\n", delta);

  /* Sleep and verify counter advances */

  sleep(RTCTEST_COUNTER_SECS);

  ret = ioctl(fd, RTC_RD_TIME, (unsigned long)((uintptr_t)&later));
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[RTC Test]   RTC_RD_TIME after sleep FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  rtctest_print_time("later", &later);

  later_s = rtctest_to_time(&later);
  delta   = (long)(later_s - rd_s);

  if (delta < RTCTEST_COUNTER_SECS - 1 ||
      delta > RTCTEST_COUNTER_SECS + RTCTEST_COUNTER_TOL)
    {
      syslog(LOG_ERR,
             "[RTC Test]   counter advance=%ld FAILED (expect %d..%d)\n",
             delta, RTCTEST_COUNTER_SECS - 1,
             RTCTEST_COUNTER_SECS + RTCTEST_COUNTER_TOL);
      close(fd);
      return -ERANGE;
    }

  syslog(LOG_INFO,
         "[RTC Test]   counter advance=%ld PASSED\n", delta);

  close(fd);
  syslog(LOG_INFO, "[RTC Test] Test 1: Counter test PASSED\n");
  return OK;
}

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: rtctest_run_alarm
 *
 * Description:
 *   Test 2 - Alarm test:  RTC_SET_ALARM / RTC_RD_ALARM / RTC_CANCEL_ALARM
 *   and RTC_SET_RELATIVE.  Verifies absolute alarm fires in expected range,
 *   cancel prevents delivery, and relative alarm fires correctly.
 ****************************************************************************/

static int rtctest_run_alarm(void)
{
  struct rtc_time base;
  struct rtc_time now;
  struct rtc_setalarm_s sa;
  struct rtc_rdalarm_s ra;
  struct rtc_setrelative_s sr;
  struct timespec timeout;
  sigset_t set;
  uint32_t start_ms;
  uint32_t elapsed;
  long lo;
  long hi;
  int fd;
  int ret;

  syslog(LOG_INFO, "[RTC Test] Test 2: Alarm\n");

  fd = rtctest_open();
  if (fd < 0)
    {
      return fd;
    }

  /* Prepare signal */

  signal(RTCTEST_SIGNO, SIG_IGN);
  sigemptyset(&set);
  sigaddset(&set, RTCTEST_SIGNO);
  ret = sigprocmask(SIG_BLOCK, &set, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   sigprocmask FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  /* Flush pending */

  timeout.tv_sec  = 0;
  timeout.tv_nsec = 0;
  while (sigtimedwait(&set, NULL, &timeout) >= 0);

  /* Set a known base time */

  memset(&base, 0, sizeof(base));
  base.tm_year = 2000 - TM_YEAR_BASE;
  base.tm_mon  = TM_JANUARY;
  base.tm_mday = 1;
  base.tm_wday = TM_SATURDAY;

  ret = ioctl(fd, RTC_SET_TIME, (unsigned long)((uintptr_t)&base));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_SET_TIME FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  ret = ioctl(fd, RTC_RD_TIME, (unsigned long)((uintptr_t)&now));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_RD_TIME FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  /* --- 2a: Absolute alarm --- */

  syslog(LOG_INFO, "[RTC Test]   2a: Absolute alarm\n");

  memset(&sa, 0, sizeof(sa));
  sa.id  = 0;
  sa.pid = getpid();
  sa.event.sigev_notify          = SIGEV_SIGNAL;
  sa.event.sigev_signo           = RTCTEST_SIGNO;
  sa.event.sigev_value.sival_int = 0;
  sa.time = now;
  rtctest_add_seconds(&sa.time, RTCTEST_ALARM_SECS + 1);

  ret = ioctl(fd, RTC_SET_ALARM, (unsigned long)((uintptr_t)&sa));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_SET_ALARM FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  /* Read-back */

  memset(&ra, 0, sizeof(ra));
  ra.id = 0;
  ret = ioctl(fd, RTC_RD_ALARM, (unsigned long)((uintptr_t)&ra));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_RD_ALARM FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  if (!ra.active)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_RD_ALARM inactive FAILED\n");
      close(fd);
      return -EIO;
    }

  if (rtctest_to_time(&ra.time) != rtctest_to_time(&sa.time))
    {
      syslog(LOG_ERR,
             "[RTC Test]   RTC_RD_ALARM readback mismatch FAILED\n");
      close(fd);
      return -EIO;
    }

  syslog(LOG_INFO, "[RTC Test]   RTC_RD_ALARM readback PASSED\n");

  /* Wait for alarm */

  start_ms = rtctest_monotonic_ms();
  ret = sigwaitinfo(&set, NULL);
  if (ret != RTCTEST_SIGNO)
    {
      syslog(LOG_ERR,
             "[RTC Test]   sigwaitinfo returned %d errno=%d FAILED\n",
             ret, errno);
      close(fd);
      return ret < 0 ? -errno : -EIO;
    }

  elapsed = rtctest_monotonic_ms() - start_ms;
  lo = (long)(RTCTEST_ALARM_SECS * 1000 - RTCTEST_DEVIATION_MS);
  hi = (long)((RTCTEST_ALARM_SECS + 1) * 1000 + RTCTEST_DEVIATION_MS);

  if ((long)elapsed < lo || (long)elapsed > hi)
    {
      syslog(LOG_ERR,
             "[RTC Test]   absolute alarm elapsed=%lu FAILED"
             " (expect %ld..%ld)\n",
             (unsigned long)elapsed, lo, hi);
      close(fd);
      return -ERANGE;
    }

  syslog(LOG_INFO,
         "[RTC Test]   absolute alarm elapsed=%lu PASSED\n",
         (unsigned long)elapsed);

  /* Flush */

  while (sigtimedwait(&set, NULL, &timeout) >= 0);

  /* --- 2b: Cancel alarm test --- */

  syslog(LOG_INFO, "[RTC Test]   2b: Cancel alarm\n");

  memset(&sr, 0, sizeof(sr));
  sr.id  = 0;
  sr.pid = getpid();
  sr.event.sigev_notify          = SIGEV_SIGNAL;
  sr.event.sigev_signo           = RTCTEST_SIGNO;
  sr.event.sigev_value.sival_int = 0;
  sr.reltime = RTCTEST_ALARM_SECS;

  ret = ioctl(fd, RTC_SET_RELATIVE, (unsigned long)((uintptr_t)&sr));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_SET_RELATIVE FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  ret = ioctl(fd, RTC_CANCEL_ALARM, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "[RTC Test]   RTC_CANCEL_ALARM FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  timeout.tv_sec  = RTCTEST_ALARM_SECS + 2;
  timeout.tv_nsec = 0;
  ret = sigtimedwait(&set, NULL, &timeout);
  if (ret >= 0)
    {
      syslog(LOG_ERR,
             "[RTC Test]   unexpected signal after cancel FAILED\n");
      close(fd);
      return -EIO;
    }

  if (errno != EAGAIN)
    {
      syslog(LOG_ERR, "[RTC Test]   sigtimedwait errno=%d FAILED\n", errno);
      close(fd);
      return -errno;
    }

  syslog(LOG_INFO, "[RTC Test]   cancel alarm PASSED\n");

  /* Flush */

  timeout.tv_sec  = 0;
  timeout.tv_nsec = 0;
  while (sigtimedwait(&set, NULL, &timeout) >= 0);

  /* --- 2c: Relative alarm --- */

  syslog(LOG_INFO, "[RTC Test]   2c: Relative alarm\n");

  ret = ioctl(fd, RTC_SET_RELATIVE, (unsigned long)((uintptr_t)&sr));
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[RTC Test]   RTC_SET_RELATIVE (2nd) FAILED: %d\n", errno);
      close(fd);
      return -errno;
    }

  start_ms = rtctest_monotonic_ms();
  ret = sigwaitinfo(&set, NULL);
  if (ret != RTCTEST_SIGNO)
    {
      syslog(LOG_ERR, "[RTC Test]   sigwaitinfo returned %d FAILED\n", ret);
      close(fd);
      return ret < 0 ? -errno : -EIO;
    }

  elapsed = rtctest_monotonic_ms() - start_ms;
  lo = (long)(RTCTEST_ALARM_SECS * 1000 - RTCTEST_DEVIATION_MS);
  hi = (long)((RTCTEST_ALARM_SECS + 1) * 1000 + RTCTEST_DEVIATION_MS);

  if ((long)elapsed < lo || (long)elapsed > hi)
    {
      syslog(LOG_ERR,
             "[RTC Test]   relative alarm elapsed=%lu FAILED"
             " (expect %ld..%ld)\n",
             (unsigned long)elapsed, lo, hi);
      close(fd);
      return -ERANGE;
    }

  syslog(LOG_INFO,
         "[RTC Test]   relative alarm elapsed=%lu PASSED\n",
         (unsigned long)elapsed);

  close(fd);
  syslog(LOG_INFO, "[RTC Test] Test 2: Alarm test PASSED\n");
  return OK;
}

#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: rtctest_thread
 *
 * Description:
 *   Kernel thread entry that runs the RTC self-test sequence.
 ****************************************************************************/

static int rtctest_thread(int argc, FAR char *argv[])
{
  int failures = 0;
  int ret;

  syslog(LOG_INFO, "[RTC Test] Starting RTC self-test ...\n");

  /* Test 1 - Counter */

  ret = rtctest_run_counter();
  if (ret < 0)
    {
      failures++;
    }

#ifdef CONFIG_RTC_ALARM
  /* Test 2 - Alarm */

  ret = rtctest_run_alarm();
  if (ret < 0)
    {
      failures++;
    }
#else
  syslog(LOG_INFO,
         "[RTC Test] CONFIG_RTC_ALARM not set, alarm test skipped\n");
#endif

  if (failures > 0)
    {
      syslog(LOG_ERR, "[RTC Test] DONE - %d failure(s)\n", failures);
    }
  else
    {
      syslog(LOG_INFO, "[RTC Test] All tests PASSED\n");
    }

  return failures > 0 ? ERROR : OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_rtc_test
 *
 * Description:
 *   Run a board-level RTC self-test.  A kernel thread is created so that
 *   blocking operations (sleep / signal-wait) do not stall bringup.
 *
 * Returned Value:
 *   OK on success, or a negative error code if the thread could not start.
 *
 ****************************************************************************/

int gd32_rtc_test(void)
{
  int pid;

  pid = kthread_create(
          "rtctest",
          RTCTEST_THREAD_PRIO,
          RTCTEST_THREAD_STACKSZ,
          rtctest_thread, NULL);
  if (pid < 0)
    {
      syslog(LOG_ERR,
             "[RTC Test] Failed to create test thread: %d\n", pid);
      return pid;
    }

  syslog(LOG_INFO, "[RTC Test] Test thread started (pid=%d)\n", pid);
  return OK;
}

#endif /* CONFIG_GD32E113VB_EVAL_RTCTEST */
