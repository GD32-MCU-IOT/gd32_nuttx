/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_rtcounter.c
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

/* The GD32 RTC Driver offers standard precision of 1 Hz or High Resolution
 * operating at rate up to 16384 Hz. It provides UTC time and alarm interface
 * with external output pin (for wake-up).
 *
 * RTC is based on hardware RTC module which is located in a separate power
 * domain. The 32-bit counter is extended by 16-bit registers in BKP domain
 * GD32 backup data register to provide system equivalent storage for time_t.
 * (time_t *).
 *
 * Notation:
 *  - clock refers to 32-bit hardware counter
 *  - time is a combination of clock and upper bits stored in backuped domain
 *    with unit of 1 [s]
 *
 * TODO:
 * Error Handling in case LSE fails during start-up or during operation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32e11x.h"
#include "gd32e11x_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* In hi-res mode, the RTC operates at 16384Hz.  Overflow interrupts are
 * handled when the 32-bit RTC counter overflows every 3 days and 43 minutes.
 * A BKP register is incremented on each overflow interrupt creating,
 * effectively, a 48-bit RTC counter.
 *
 * In the lo-res mode, the RTC operates at 1Hz.  Overflow interrupts are not
 * handled (because the next overflow is not expected until the year 2106).
 *
 * WARNING:
 * Overflow interrupts are lost whenever the GD32 is powered down.  The
 * overflow interrupt may be lost even if the GD32 is powered down only
 * momentarily. Therefore hi-res solution is only useful in systems where
 * the power is always on.
 */

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  elif CONFIG_RTC_FREQUENCY != 16384
#    error "Only hi-res CONFIG_RTC_FREQUENCY of 16384Hz is supported"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 1Hz is supported"
#  endif
#endif

#ifndef CONFIG_GD32E11X_BKP
#  error "CONFIG_GD32E11X_BKP is required for CONFIG_GD32E11X_RTC"
#endif

#ifndef CONFIG_GD32E11X_PMU
#  error "CONFIG_GD32E11X_PMU is required for CONFIG_GD32E11X_RTC"
#endif

#if defined(CONFIG_GD32_RTC_HXTALCLOCK)
#  error "RTC with HXTAL clock not yet implemented for GD32E11x"
#elif defined(CONFIG_GD32_RTC_LSICLOCK)
#  error "RTC with LSI clock not yet implemented for GD32E11x"
#endif

/* RTC/BKP Definitions ******************************************************/

/* GD32_RTC_PRESCALAR_VALUE
 *   RTC pre-scalar value.  The RTC is driven by a 32,768Hz input clock.
 *   This input value is divided by this value (plus one) to generate the
 *   RTC frequency.
 * RTC_TIMEMSB_REG
 *   The BKP module register used to hold the RTC overflow value.
 *   Overflows are only handled in hi-res mode.
 * RTC_CLOCKS_SHIFT
 *   The shift used to convert the hi-res timer LSB to one second.
 *   Not used with the lo-res timer.
 */

#ifdef CONFIG_RTC_HIRES
#  define GD32_RTC_PRESCALER_VALUE  GD32_RTC_PRESCALER_MIN
#  define RTC_TIMEMSB_REG           GD32_BKP_DATA1
#  define RTC_CLOCKS_SHIFT          14
#else
#  define GD32_RTC_PRESCALER_VALUE GD32_RTC_PRESCALER_SECOND
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rtc_regvals_s
{
  uint16_t cntl;
  uint16_t cnth;
#ifdef CONFIG_RTC_HIRES
  uint16_t ovf;
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_rtc_lock = SP_UNLOCKED;

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM

static alarmcb_t g_alarmcb;

/* Cached alarm value — ALRMH/ALRML are write-only registers on
 * GD32E11x (STM32F1-compatible), reading them returns undefined data.
 * We cache the values here so that gd32_rtc_rdalarm() can return them.
 */

static uint16_t g_alarm_cnth;
static uint16_t g_alarm_cntl;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Variable determines the state of the LSE oscillator.
 * Possible errors:
 *   - on start-up
 *   - during operation, reported by LSE interrupt
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void gd32_rtc_bkp_write_enable(void)
{
  modifyreg32(GD32_RCU_APB1EN, 0, RCU_APB1EN_PMUEN | RCU_APB1EN_BKPIEN);
  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_BKPWEN);
}

static inline void gd32_rtc_bkp_write_disable(void)
{
  modifyreg32(GD32_PMU_CTL, PMU_CTL_BKPWEN, 0);
}

/****************************************************************************
 * Name: gd32_rtc_waitlasttask
 *
 * Description:
 *   wait task done
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gd32_rtc_waitlasttask(void)
{
  /* Previous write is done? */

  while ((getreg32(GD32_RTC_CTL) & RTC_CTL_LWOFF) == 0)
    {
      gd32_waste();
    }
}

/****************************************************************************
 * Name: gd32_rtc_beginwr
 *
 * Description:
 *   Enter configuration mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gd32_rtc_beginwr(void)
{
    gd32_rtc_waitlasttask();

  /* Enter Config mode, Set Value and Exit */

    modifyreg32(GD32_RTC_CTL, 0, RTC_CTL_CMF);
}

/****************************************************************************
 * Name: gd32_rtc_endwr
 *
 * Description:
 *   Exit configuration mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gd32_rtc_endwr(void)
{
    modifyreg32(GD32_RTC_CTL, RTC_CTL_CMF, 0);
    gd32_rtc_waitlasttask();
}

/****************************************************************************
 * Name: gd32_rtc_wait4rsf
 *
 * Description:
 *   Wait for registers to synchronise with RTC module, call after power-up
 *   only
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gd32_rtc_wait4rsf(void)
{
  modifyreg32(GD32_RTC_CTL, RTC_CTL_RSYNF, 0);

  while ((getreg32(GD32_RTC_CTL) & RTC_CTL_RSYNF) == 0)
    {
      gd32_waste();
    }
}

/****************************************************************************
 * Name: gd32_rtc_breakout
 *
 * Description:
 *   Set the RTC to the provided time.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
static void gd32_rtc_breakout(const struct timespec *tp,
                              struct rtc_regvals_s *regvals)
{
  uint64_t frac;
  uint32_t cnt;
  uint16_t ovf;

  /* Break up the time in seconds + milleconds into the correct values for
   * our use
   */

  frac = ((uint64_t)tp->tv_nsec * CONFIG_RTC_FREQUENCY) / 1000000000;
  cnt  = ((uint32_t)tp->tv_sec << RTC_CLOCKS_SHIFT) |
         ((uint32_t)frac & (CONFIG_RTC_FREQUENCY - 1));
  ovf  = (tp->tv_sec >> (32 - RTC_CLOCKS_SHIFT));

  /* Then return the broken out time */

  regvals->cnth = (uint16_t)(cnt >> 16);
  regvals->cntl = (uint16_t)(cnt & 0xffff);
  regvals->ovf  = ovf;
}
#else
static inline void gd32_rtc_breakout(const struct timespec *tp,
                                     struct rtc_regvals_s *regvals)
{
  /* The low-res timer is easy... tv_sec holds exactly the value needed
   * by the CNTH/CNTL registers.
   */

  regvals->cnth = (uint16_t)((uint32_t)tp->tv_sec >> 16);
  regvals->cntl = (uint16_t)((uint32_t)tp->tv_sec & 0xffff);
}
#endif

/****************************************************************************
 * Name: gd32_rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_HIRES) || defined(CONFIG_RTC_ALARM)
static int gd32_rtc_interrupt(int irq, void *context, void *arg)
{
  uint32_t source = getreg32(GD32_RTC_CTL);

#ifdef CONFIG_RTC_HIRES
  if ((source & RTC_CTL_OVIF) != 0)
    {
      gd32_rtc_bkp_write_enable();
      putreg32(getreg32(RTC_TIMEMSB_REG) + 1, RTC_TIMEMSB_REG);
      gd32_rtc_bkp_write_disable();
    }
#endif

#ifdef CONFIG_RTC_ALARM
  if ((source & RTC_CTL_ALRMIF) != 0 && g_alarmcb != NULL)
    {
      /* Alarm callback */

      g_alarmcb();
      g_alarmcb = NULL;
    }
#endif

  /* Clear pending flags, leave RSF high */

  putreg32(RTC_CTL_RSYNF, GD32_RTC_CTL);

#ifdef CONFIG_RTC_ALARM
  /* Clear the EXTI line 17 pending bit */

  putreg32(EXTI_RTC_ALARM, GD32_EXTI_PD);
#endif
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  uint32_t regval;

  /* Enable write access to the backup domain (RTC registers, RTC backup data
   * registers and backup SRAM).
   */

  gd32_rtc_bkp_write_enable();

  regval = getreg32(RTC_MAGIC_REG);
  if (regval != RTC_MAGIC && regval != RTC_MAGIC_TIME_SET)
    {
      /* Reset backup domain if bad magic */

      modifyreg32(GD32_RCU_BDCTL, 0, RCU_BDCTL_BKPRST);
      modifyreg32(GD32_RCU_BDCTL, RCU_BDCTL_BKPRST, 0);

      modifyreg32(GD32_RCU_BDCTL, 0, RCU_BDCTL_LXTALEN);

      /* Wait for the LSE clock to be ready */

      while ((getreg32(GD32_RCU_BDCTL) & RCU_BDCTL_LXTALSTB) == 0)
        {
          gd32_waste();
        }

      /* Select the lower power external 32,768Hz (Low-Speed External, LSE)
       * oscillator as RTC Clock Source and enable the Clock.
       */

      modifyreg32(GD32_RCU_BDCTL, RCU_BDCTL_RTCSRC_MASK,
                  RCU_BDCTL_RTCSRC_LXTAL);

      /* Enable RTC and wait for RSF */

      modifyreg32(GD32_RCU_BDCTL, 0, RCU_BDCTL_RTCEN);
      gd32_rtc_waitlasttask();

      gd32_rtc_wait4rsf();
      gd32_rtc_waitlasttask();

      /* Configure prescaler, note that these are write-only registers */

      gd32_rtc_beginwr();
      putreg32(GD32_RTC_PRESCALER_VALUE >> 16, GD32_RTC_PSCH);
      putreg32(GD32_RTC_PRESCALER_VALUE & 0xffff, GD32_RTC_PSCL);
      gd32_rtc_endwr();

      gd32_rtc_wait4rsf();
      gd32_rtc_waitlasttask();

      /* Write the magic register after RTC initialization. */

      putreg32(RTC_MAGIC, RTC_MAGIC_REG);
    }
  else
    {
      /* Warm restart: RTC already configured but APB1/RTC clock domains
       * are not yet synchronized after reset.  Must wait for RSF before
       * any RTC register reads are valid.
       */

      gd32_rtc_wait4rsf();
    }

#ifdef CONFIG_RTC_HIRES
  /* Enable overflow interrupt - alarm interrupt is enabled in
   * gd32_rtc_setalarm().
   */

  modifyreg32(GD32_RTC_INTEN, 0, RTC_INTEN_OVIE);
#endif

#ifdef CONFIG_RTC_ALARM
  /* Clear stale alarm state from cold boot (counter=alarm=0 match) or
   * warm restart (previous session residual).  Disable ALRMIE and clear
   * ALRMIF so that the first gd32_rtc_setalarm() call starts clean.
   */

  modifyreg32(GD32_RTC_INTEN, RTC_INTEN_ALRMIE, 0);
  putreg32(RTC_CTL_RSYNF, GD32_RTC_CTL);
#endif

  g_rtc_enabled = true;

  /* Disable write access to the backup domain
   * (RTC registers, RTC backup data registers and backup SRAM).
   */

  gd32_rtc_bkp_write_disable();

  return OK;
}

/****************************************************************************
 * Name: gd32_rtc_irqinitialize
 *
 * Description:
 *   Initialize IRQs for RTC, not possible during up_rtc_initialize because
 *   up_irqinitialize is called later.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int gd32_rtc_irqinitialize(void)
{
#if defined(CONFIG_RTC_HIRES) || defined(CONFIG_RTC_ALARM)
  /* Configure RTC interrupt to catch overflow and alarm interrupts. */

    irq_attach(GD32_IRQ_RTC, gd32_rtc_interrupt, NULL);
    up_enable_irq(GD32_IRQ_RTC);
#endif

#ifdef CONFIG_RTC_ALARM
  /* Configure EXTI line 17 for RTC alarm interrupt.
   * Rising edge trigger, interrupt mode.
   */

  gd32_exti_alarm(true, false, false, gd32_rtc_interrupt, NULL);
#endif

  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.
 *   This is similar to the standard time() function.
 *   This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set but
 *   neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  irqstate_t flags;
  uint16_t cnth;
  uint16_t cntl;
  uint16_t tmp;

  /* The RTC counter is read from two 16-bit registers to form one 32-bit
   * value.  Because these are non-atomic operations, many things can happen
   * between the two reads:  This thread could get suspended or interrupted
   * or the lower 16-bit counter could rollover between reads.  Disabling
   * interrupts will prevent suspensions and interruptions:
   */

  flags = spin_lock_irqsave(&g_rtc_lock);

  /* And the following loop will handle any clock rollover events that may
   * happen between samples.  Most of the time (like 99.9%), the following
   * loop will execute only once.  In the rare rollover case, it should
   * execute no more than 2 times.
   */

  do
    {
      tmp  = (uint16_t)getreg32(GD32_RTC_CNTL);
      cnth = (uint16_t)getreg32(GD32_RTC_CNTH);
      cntl = (uint16_t)getreg32(GD32_RTC_CNTL);
    }

  /* The second sample of CNTL could be less than the first sample of CNTL
   * only if rollover occurred.  In that case, CNTH may or may not be out
   * of sync.  The best thing to do is try again until we know that no
   * rollover occurred.
   */

  while (cntl < tmp);
  spin_unlock_irqrestore(&g_rtc_lock, flags);

  /* Okay.. the samples should be as close together in time as possible and
   * we can be assured that no clock rollover occurred between the samples.
   *
   * Return the time in seconds.
   */

  return ((time_t)cnth << 16 | (time_t)cntl);
}
#endif

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation.
 *   It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(struct timespec *tp)
{
  irqstate_t flags;
  uint32_t ls;
  uint32_t ms;
  uint16_t ovf;
  uint16_t cnth;
  uint16_t cntl;
  uint16_t tmp;

  /* The RTC counter is read from two 16-bit registers to form one 32-bit
   * value.  Because these are non-atomic operations, many things can happen
   * between the two reads:  This thread could get suspended or interrupted
   * or the lower 16-bit counter could rollover between reads.  Disabling
   * interrupts will prevent suspensions and interruptions:
   */

  flags = spin_lock_irqsave(&g_rtc_lock);

  /* And the following loop will handle any clock rollover events that may
   * happen between samples.  Most of the time (like 99.9%), the following
   * loop will execute only once.  In the rare rollover case, it should
   * execute no more than 2 times.
   */

  do
    {
      tmp  = (uint16_t)getreg32(GD32_RTC_CNTL);
      cnth = (uint16_t)getreg32(GD32_RTC_CNTH);
      ovf  = (uint16_t)getreg32(RTC_TIMEMSB_REG);
      cntl = (uint16_t)getreg32(GD32_RTC_CNTL);
    }

  /* The second sample of CNTL could be less than the first sample of CNTL
   * only if rollover occurred.  In that case, CNTH may or may not be out
   * of sync.  The best thing to do is try again until we know that no
   * rollover occurred.
   */

  while (cntl < tmp);
  spin_unlock_irqrestore(&g_rtc_lock, flags);

  /* Okay.. the samples should be as close together in time as possible and
   * we can be assured that no clock rollover occurred between the samples.
   *
   * Create a 32-bit value from the LS and MS 16-bit RTC counter values and
   * from the MS and overflow 16-bit counter values.
   */

  ls = (uint32_t)cnth << 16 | (uint32_t)cntl;
  ms = (uint32_t)ovf  << 16 | (uint32_t)cnth;

  /* Then we can save the time in seconds and fractional seconds. */

  tp->tv_sec  = (ms << (32 - RTC_CLOCKS_SHIFT - 16)) |
                (ls >> (RTC_CLOCKS_SHIFT + 16));
  tp->tv_nsec = (ls & (CONFIG_RTC_FREQUENCY - 1)) *
                (1000000000 / CONFIG_RTC_FREQUENCY);
  return OK;
}
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *tp)
{
  struct rtc_regvals_s regvals;
  irqstate_t flags;

  /* Break out the time values */

  gd32_rtc_breakout(tp, &regvals);

  /* Enable write access to the backup domain */

  flags = spin_lock_irqsave(&g_rtc_lock);
  gd32_rtc_bkp_write_enable();

  /* Then write the broken out values to the RTC counter and BKP overflow
   * register (hi-res mode only)
   */

  gd32_rtc_beginwr();
  putreg32(regvals.cnth, GD32_RTC_CNTH);
  putreg32(regvals.cntl, GD32_RTC_CNTL);
  gd32_rtc_endwr();
  putreg32(RTC_MAGIC_TIME_SET, RTC_MAGIC_REG);

#ifdef CONFIG_RTC_HIRES
  putreg32(regvals.ovf, RTC_TIMEMSB_REG);
#endif

  gd32_rtc_bkp_write_disable();
  spin_unlock_irqrestore(&g_rtc_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: gd32_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int gd32_rtc_setalarm(const struct timespec *tp, alarmcb_t callback)
{
  struct rtc_regvals_s regvals;
  irqstate_t flags;
  uint16_t cr;
  int ret = -EBUSY;

  flags = spin_lock_irqsave(&g_rtc_lock);

  /* Is there already something waiting on the ALARM? */

  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* Break out the time values */

      gd32_rtc_breakout(tp, &regvals);

      gd32_rtc_bkp_write_enable();

      /* Clear any stale alarm flag before enabling, then enable alarm */

      putreg32(RTC_CTL_RSYNF, GD32_RTC_CTL);
      cr  = getreg32(GD32_RTC_INTEN);
      cr |= RTC_INTEN_ALRMIE;
      putreg32(cr, GD32_RTC_INTEN);

      /* The set the alarm */

      gd32_rtc_beginwr();
      putreg32(regvals.cnth, GD32_RTC_ALRMH);
      putreg32(regvals.cntl, GD32_RTC_ALRML);
      gd32_rtc_endwr();

      /* Cache alarm value — ALRMH/ALRML are write-only registers */

      g_alarm_cnth = regvals.cnth;
      g_alarm_cntl = regvals.cntl;

      gd32_rtc_bkp_write_disable();

      ret = OK;
    }

  spin_unlock_irqrestore(&g_rtc_lock, flags);

  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_rtc_cancelalarm
 *
 * Description:
 *   Cancel a pending alarm alarm
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int gd32_rtc_cancelalarm(void)
{
  irqstate_t flags;
  int ret = -ENODATA;

  flags = spin_lock_irqsave(&g_rtc_lock);

  if (g_alarmcb != NULL)
    {
      /* Cancel the global callback function */

      g_alarmcb = NULL;

      /* Disable alarm interrupt and unset the alarm */

      gd32_rtc_bkp_write_enable();
      modifyreg32(GD32_RTC_INTEN, RTC_INTEN_ALRMIE, 0);
      gd32_rtc_beginwr();
      putreg32(0xffff, GD32_RTC_ALRMH);
      putreg32(0xffff, GD32_RTC_ALRML);
      gd32_rtc_endwr();
      putreg32(RTC_CTL_RSYNF, GD32_RTC_CTL);
      gd32_rtc_bkp_write_disable();

      ret = OK;
    }

  spin_unlock_irqrestore(&g_rtc_lock, flags);

  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int gd32_rtc_rdalarm(FAR struct alm_rdalarm_s *alminfo)
{
  struct timespec tp;
  int ret = -EINVAL;

  DEBUGASSERT(alminfo != NULL);
  DEBUGASSERT(alminfo->ar_id == 0);

  switch (alminfo->ar_id)
    {
      case 0:
        {
          struct tm tmp;

          /* Use cached values — ALRMH/ALRML are write-only registers,
           * reading them returns undefined data.
           */

          tp.tv_sec = (time_t)g_alarm_cnth << 16 | g_alarm_cntl;
          gmtime_r(&tp.tv_sec, &tmp);
          memcpy(alminfo->ar_time, &tmp, sizeof(struct tm));
          ret = OK;
        }
        break;

      default:
        rtcerr("ERROR: Invalid ALARM%d\n", alminfo->ar_id);
        break;
    }

  return ret;
}
#endif