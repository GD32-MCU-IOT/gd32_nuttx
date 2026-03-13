/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_timer.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/timers/timer.h>
#include <arch/board/board.h>

#include "chip.h"
#include "gd32e11x_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer
 *
 * The GD32E113VB-EVAL board can be configured to use various timers as
 * NuttX timer character devices (/dev/timerX).
 *
 * Default configuration uses TIMER5 (basic timer, no channel pins needed)
 * for the timer example test.  TIMER5 and TIMER6 are ideal for this
 * purpose as they are basic timers with no GPIO pin conflicts.
 *
 * For more precise timing tests or tests that need channel I/O, use
 * TIMER1/TIMER3/TIMER4 (general-purpose timers).  Avoid using TIMER2
 * simultaneously with PWM testing (both would try to use the same timer).
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver.
 *
 *   Call chain:
 *     board_late_initialize() or boardctl(BOARDIOC_INIT)
 *       -> gd32_bringup()
 *         -> gd32_timer_driver_setup()   [this function]
 *           -> gd32_timer_initialize()   [arch timer lower-half driver]
 *             -> gd32_timer_init()       [arch low-level timer driver]
 *             -> timer_register()        [NuttX timer upper-half, creates
 *                                         /dev/timerX]
 *
 *   Then user space (e.g., examples/timer app):
 *     open("/dev/timer0")
 *       -> ioctl(TCIOC_SETTIMEOUT, interval_us)
 *       -> ioctl(TCIOC_NOTIFICATION, {sigev, pid, ...})
 *       -> ioctl(TCIOC_START)
 *       -> sigwaitinfo() / sleep()
 *       -> ioctl(TCIOC_STOP)
 *       -> close(fd)
 *
 ****************************************************************************/

int gd32_timer_driver_setup(const char *devpath, int timer)
{
  return gd32_timer_initialize(devpath, timer);
}
