/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_pwm.c
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

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "gd32e11x_pwm.h"
#include "gd32e113v_eval.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM
 *
 * The GD32E113VB-EVAL board can be configured to output a PWM signal on
 * various timer channels.  The default configuration uses:
 *
 *   TIMER2 CH0 - PA6 (no remap, "no remap" = _1 suffix in pinmap)
 *
 * This pin is freely available and does not conflict with LEDs, buttons,
 * or USART0 (serial console on PA9/PA10).
 *
 * To use a different timer, change GD32E113VBEVAL_PWMTIMER in the board
 * header and enable the corresponding timer/channel in Kconfig.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 *   Call chain:
 *     board_late_initialize() or boardctl(BOARDIOC_INIT)
 *       -> gd32_bringup()
 *         -> gd32_pwm_setup()           [this function]
 *           -> gd32_pwminitialize()      [arch PWM lower-half driver]
 *           -> pwm_register()            [NuttX PWM upper-half, creates
 *                                         /dev/pwm0]
 *
 *   Then user space (e.g., examples/pwm app):
 *     open("/dev/pwm0")
 *       -> ioctl(PWMIOC_SETCHARACTERISTICS, {freq, duty})
 *       -> ioctl(PWMIOC_START)
 *       -> sleep(duration)
 *       -> ioctl(PWMIOC_STOP)
 *       -> close(fd)
 *
 ****************************************************************************/

int gd32_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call gd32_pwminitialize() to get an instance of the PWM
       * lower-half driver interface for the specified timer.
       */

      pwm = gd32_pwminitialize(GD32E113VBEVAL_PWMTIMER);
      if (pwm == NULL)
        {
          pwmerr("ERROR: Failed to get the GD32 PWM lower half for "
                 "timer %d\n", GD32E113VBEVAL_PWMTIMER);
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          pwmerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}
