/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_wdt.c
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
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/watchdog.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/gd32e11x_wdg.h"
#include "hardware/gd32e11x_rcu.h"
#include "gd32e11x_gpio.h"
#include "gd32e113v_eval.h"

#ifdef CONFIG_GD32E113VB_EVAL_WDG_THREAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_GD32E113VB_EVAL_WDG_THREAD_DEVPATH
#  define CONFIG_GD32E113VB_EVAL_WDG_THREAD_DEVPATH "/dev/watchdog0"
#endif

#ifndef CONFIG_GD32E113VB_EVAL_WDG_THREAD_NAME
#  define CONFIG_GD32E113VB_EVAL_WDG_THREAD_NAME "wdog"
#endif

#ifndef CONFIG_GD32E113VB_EVAL_WDG_THREAD_INTERVAL
#  define CONFIG_GD32E113VB_EVAL_WDG_THREAD_INTERVAL 5000
#endif

#ifndef CONFIG_GD32E113VB_EVAL_WDG_THREAD_PRIORITY
#  define CONFIG_GD32E113VB_EVAL_WDG_THREAD_PRIORITY SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_GD32E113VB_EVAL_WDG_THREAD_STACKSIZE
#  define CONFIG_GD32E113VB_EVAL_WDG_THREAD_STACKSIZE 1024
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wdog_daemon
 *
 * Description:
 *   Watchdog daemon thread that periodically feeds the watchdog.
 *
 ****************************************************************************/

static int wdog_daemon(int argc, char *argv[])
{
  int ret;
  int led_state = 0;

  /* Start watchdog timer using the new API */

  ret = gd32_wdt_start(CONFIG_GD32E113VB_EVAL_WDG_THREAD_DEVPATH);
  if (ret < 0)
    {
      wderr("ERROR: Failed to start watchdog: %d\n", ret);
      return ret;
    }

  wdinfo("Watchdog daemon started, feeding %s every %d ms\n",
         CONFIG_GD32E113VB_EVAL_WDG_THREAD_DEVPATH,
         CONFIG_GD32E113VB_EVAL_WDG_THREAD_INTERVAL);

  /* Periodically feed the watchdog */

  while (1)
    {
      /* Toggle LED3 */

      led_state = !led_state;
      gd32_gpio_write(LED3, led_state);

      /* Feed the watchdog using the new API */

      ret = gd32_wdt_keepalive(CONFIG_GD32E113VB_EVAL_WDG_THREAD_DEVPATH);
      if (ret < 0)
        {
          wderr("ERROR: Failed to feed watchdog: %d\n", ret);
          break;
        }

      /* Sleep for the specified interval */

      up_udelay(CONFIG_GD32E113VB_EVAL_WDG_THREAD_INTERVAL * 1000);
    }

  /* Turn off LED3 on exit */

  gd32_gpio_write(LED3, false);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_watchdog_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *   This function starts a kernel thread to automatically feed the watchdog.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gd32_watchdog_initialize(void)
{
  int taskid;

  /* Start the watchdog daemon task */

  taskid = kthread_create(CONFIG_GD32E113VB_EVAL_WDG_THREAD_NAME,
                          CONFIG_GD32E113VB_EVAL_WDG_THREAD_PRIORITY,
                          CONFIG_GD32E113VB_EVAL_WDG_THREAD_STACKSIZE,
                          wdog_daemon,
                          NULL);
  if (taskid < 0)
    {
      wderr("ERROR: Failed to start watchdog daemon: %d\n", -taskid);
      return -taskid;
    }

  return OK;
}

#endif /* CONFIG_GD32E113VB_EVAL_WDG_THREAD */

/****************************************************************************
 * Public Watchdog Control Functions
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Name: gd32_wdt_start
 *
 * Description:
 *   Start the watchdog timer on the specified device.
 *
 ****************************************************************************/

int gd32_wdt_start(const char *devpath)
{
  struct file filestruct;
  int ret;

  /* Open watchdog device */

  ret = file_open(&filestruct, devpath, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: Failed to open %s: %d\n", devpath, ret);
      return ret;
    }

  /* Start watchdog timer */

  ret = file_ioctl(&filestruct, WDIOC_START, 0);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_START) failed: %d\n", ret);
    }

  file_close(&filestruct);
  return ret;
}

/****************************************************************************
 * Name: gd32_wdt_stop
 *
 * Description:
 *   Stop the watchdog timer on the specified device.
 *
 ****************************************************************************/

int gd32_wdt_stop(const char *devpath)
{
  struct file filestruct;
  int ret;

  /* Open watchdog device */

  ret = file_open(&filestruct, devpath, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: Failed to open %s: %d\n", devpath, ret);
      return ret;
    }

  /* Stop watchdog timer */

  ret = file_ioctl(&filestruct, WDIOC_STOP, 0);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_STOP) failed: %d\n", ret);
    }

  file_close(&filestruct);
  return ret;
}

/****************************************************************************
 * Name: gd32_wdt_keepalive
 *
 * Description:
 *   Feed (keepalive) the watchdog timer on the specified device.
 *
 ****************************************************************************/

int gd32_wdt_keepalive(const char *devpath)
{
  struct file filestruct;
  int ret;

  /* Open watchdog device */

  ret = file_open(&filestruct, devpath, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: Failed to open %s: %d\n", devpath, ret);
      return ret;
    }

  /* Send keep-alive ping */

  ret = file_ioctl(&filestruct, WDIOC_KEEPALIVE, 0);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_KEEPALIVE) failed: %d\n", ret);
    }

  file_close(&filestruct);
  return ret;
}

/****************************************************************************
 * Name: gd32_wdt_settimeout
 *
 * Description:
 *   Set the watchdog timeout value on the specified device.
 *
 ****************************************************************************/

int gd32_wdt_settimeout(const char *devpath, uint32_t timeout)
{
  struct file filestruct;
  int ret;

  /* Open watchdog device */

  ret = file_open(&filestruct, devpath, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: Failed to open %s: %d\n", devpath, ret);
      return ret;
    }

  /* Set timeout value */

  ret = file_ioctl(&filestruct, WDIOC_SETTIMEOUT, timeout);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_SETTIMEOUT) failed: %d\n", ret);
    }

  file_close(&filestruct);
  return ret;
}

/****************************************************************************
 * Name: gd32_wdt_setwindow
 *
 * Description:
 *   Set the watchdog window value (WWDGT only).
 *
 ****************************************************************************/

int gd32_wdt_setwindow(const char *devpath, uint32_t window)
{
  struct file filestruct;
  int ret;

  /* Open watchdog device */

  ret = file_open(&filestruct, devpath, O_RDONLY);
  if (ret < 0)
    {
      wderr("ERROR: Failed to open %s: %d\n", devpath, ret);
      return ret;
    }

  /* Set window value (WWDGT specific) */

  ret = file_ioctl(&filestruct, WDIOC_MINTIME, window);
  if (ret < 0)
    {
      wderr("ERROR: ioctl(WDIOC_MINTIME) failed: %d\n", ret);
    }

  file_close(&filestruct);
  return ret;
}

#ifdef CONFIG_GD32E11X_WWDG

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_wwdgt_thread_running = false;
static volatile bool g_wwdgt_test_complete = false;
static const char *g_wwdgt_devpath = "/dev/watchdog1";

/****************************************************************************
 * Name: wwdgt_feed_thread
 *
 * Description:
 *   Thread that feeds WWDGT within the safe window period using file API.
 *
 ****************************************************************************/

static int wwdgt_feed_thread(int argc, char *argv[])
{
  int feed_count = 0;
  int led_state = 0;
  int ret;

  /* Configure WWDGT using file operations API */

  /* Set timeout to 35ms (reload=63, CNT=127, actual timeout=34.9ms) */

  ret = gd32_wdt_settimeout(g_wwdgt_devpath, 35);
  if (ret < 0)
    {
      wderr("ERROR: Failed to set timeout: %d\n", ret);
      return ret;
    }

  /* Set window mintime to 24ms (converts to WIN=0x50)
   * Feed window: 24-35ms (CNT: 83-64)
   * Calculation: WIN = 127 - (24 * 1831 / 1000) = 83 (0x53)
   */

  ret = gd32_wdt_setwindow(g_wwdgt_devpath, 24);
  if (ret < 0)
    {
      wderr("ERROR: Failed to set window: %d\n", ret);
      return ret;
    }

  /* Start WWDGT */

  ret = gd32_wdt_start(g_wwdgt_devpath);
  if (ret < 0)
    {
      wderr("ERROR: Failed to start WWDGT: %d\n", ret);
      return ret;
    }

  /* Main loop - feed watchdog using file API */

  while (g_wwdgt_thread_running)
    {
      /* Toggle LED3 */

      if (feed_count % 50 == 0)
        {
            led_state = !led_state;
        }

      gd32_gpio_write(LED3, led_state);

      /* Wait 33ms (within feed window: 24-35ms)
       * Using up_udelay for microsecond precision
       */

      up_udelay(33000);

      /* Feed watchdog */

      ret = gd32_wdt_keepalive(g_wwdgt_devpath);
      if (ret < 0)
        {
          wderr("ERROR: Failed to feed watchdog: %d\n", ret);
          break;
        }

      feed_count++;

      /* Stop after 400 successful feeds */

      if (feed_count == 400)
        {
          g_wwdgt_test_complete = true;
          g_wwdgt_thread_running = false;
        }
    }

  gd32_gpio_write(LED3, false);

  return OK;
}

/****************************************************************************
 * Name: gd32_wwdgt_test
 *
 * Description:
 *   Test WWDGT using file operations API.
 *
 *   Configuration:
 *   - Timeout: 35ms (actual 34.9ms, CNT=127)
 *   - Mintime: 24ms (converts to WIN=0x53)
 *   - Feed interval: 33ms (within window 24-35ms)
 *
 *   Expected behavior:
 *   - Feed 400 times successfully
 *   - Stop feeding -> System resets after ~35ms
 *   - After reset: Detects WWDGT reset flag
 *
 ****************************************************************************/

int gd32_wwdgt_test(void)
{
  int thread_id;

  /* Check if system was reset by WWDGT timeout */

  uint32_t rstsck = getreg32(GD32_RCU_RSTSCK);

  if (rstsck & RCU_RSTSCK_WWDGTRSTF)
    {
      printf("\n*** WWDGT RESET DETECTED! Test PASS! ***\n");
      gd32_gpio_write(LED2, true);
      putreg32(rstsck | RCU_RSTSCK_RSTFC, GD32_RCU_RSTSCK);
      return OK;
    }
  else
    {
      gd32_gpio_write(LED2, false);
    }

  gd32_gpio_write(LED3, false);

  printf("\n*** WWDGT Test (File API) ***\n");
  printf("Timeout=35ms, Mintime=24ms, Feed=33ms\n");
  printf("WIN register=0x53, Feed window: 24-35ms\n");
  fflush(stdout);

  /* Create high-priority feed thread */

  g_wwdgt_thread_running = true;
  g_wwdgt_test_complete = false;

  thread_id = kthread_create("wwdgt_feed",
                             SCHED_PRIORITY_MAX - 1,
                             1024,
                             wwdgt_feed_thread,
                             NULL);
  if (thread_id < 0)
    {
      printf("ERROR: Failed to create thread\n");
      g_wwdgt_thread_running = false;
      return thread_id;
    }

  printf("Thread started (tid=%d)\n", thread_id);
  fflush(stdout);

  /* Wait for test completion */

  while (!g_wwdgt_test_complete && g_wwdgt_thread_running)
    {
      usleep(50000);
    }

  if (g_wwdgt_test_complete)
    {
      printf("\n*** WWDGT Test PASSED: 400 feeds OK! ***\n");
      fflush(stdout);
    }

  return OK;
}

#endif /* CONFIG_GD32E11X_WWDG */

#endif /* CONFIG_WATCHDOG */
