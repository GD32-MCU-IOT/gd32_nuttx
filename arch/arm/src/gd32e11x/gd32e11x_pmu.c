/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_pmu.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "nvic.h"
#include "gd32e11x_pmu.h"
#include "gd32e11x.h"

#if defined(CONFIG_GD32E11X_PMU)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t gd32_pmu_reg_snap[4];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pmu_wfi
 *
 * Description:
 *   Issue the WFI that parks the core in the selected low-power mode.
 *
 *   ARMv7-M (ARM DDI 0403, B1.5.18) only terminates a WFI on an
 *   asynchronous exception "at a priority that, if PRIMASK was set to 0,
 *   would preempt any currently active exceptions".  Only PRIMASK is
 *   neutralised that way, an interrupt that is masked by BASEPRI is *not*
 *   a wake-up event.
 *
 *   NuttX raises BASEPRI to NVIC_SYSH_DISABLE_PRIORITY in up_irq_save(),
 *   and up_idlepm() enters the low-power modes from inside a critical
 *   section, so a bare WFI here could never be woken by the EXTI line
 *   that the PM buttons are attached to.
 *
 *   Swap the BASEPRI mask for a PRIMASK mask across the WFI.  The core wakes
 *   up as expected while the pending handler stays deferred until the caller
 *   leaves its critical section, so the critical section semantics are kept.
 *
 ****************************************************************************/

static inline void gd32_pmu_wfi(void)
{
  uint8_t basepri = getbasepri();
  uint8_t primask = getprimask();
  uint32_t icsr;

  /* Block the handlers with PRIMASK first and only then drop the BASEPRI
   * mask, so that no interrupt can be taken in between.
   */

  setprimask(1);
  setbasepri(0);

  /* The caller runs with BASEPRI raised, so the SysTick that expired while
   * the critical section was held is still latched in ICSR.  A pending
   * exception that is only masked by PRIMASK is a valid WFI wake-up event,
   * which would make the WFI below return immediately.  Park the pending
   * SysTick across the WFI and re-post it afterwards, so that no tick is
   * lost: the handler runs as soon as the caller leaves its critical
   * section.
   */

  icsr = getreg32(NVIC_INTCTRL);
  if ((icsr & NVIC_INTCTRL_PENDSTSET) != 0)
    {
      putreg32(NVIC_INTCTRL_PENDSTCLR, NVIC_INTCTRL);
    }

  asm("wfi");

  if ((icsr & NVIC_INTCTRL_PENDSTSET) != 0)
    {
      putreg32(NVIC_INTCTRL_PENDSTSET, NVIC_INTCTRL);
    }

  /* Re-arm the BASEPRI mask before releasing PRIMASK, same reason. */

  setbasepri(basepri);
  setprimask(primask);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pmu_lvd_select
 *
 * Description:
 *   Select low voltage detector threshold.
 *
 * Input Parameters:
 *   lvdt_n - PMU_CTL_LVDT(n), LVD threshold level
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_lvd_select(uint32_t lvdt_n)
{
  uint32_t regval;

  /* Disable LVD */

  modifyreg32(GD32_PMU_CTL, PMU_CTL_LVDEN, 0);

  regval = getreg32(GD32_PMU_CTL);

  /* Clear LVDT bits */

  regval &= ~PMU_CTL_LVDT_MASK;

  /* Set LVDT bits according to lvdt_n */

  regval |= lvdt_n;
  putreg32(regval, GD32_PMU_CTL);

  /* Enable LVD */

  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_LVDEN);
}

/****************************************************************************
 * Name: gd32_pmu_lvd_enable
 *
 * Description:
 *   Enable LVD
 *
 ****************************************************************************/

void gd32_pmu_lvd_enable(void)
{
  /* Enable LVD */

  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_LVDEN);
}

/****************************************************************************
 * Name: gd32_pmu_lvd_disable
 *
 * Description:
 *   Disable LVD
 *
 ****************************************************************************/

void gd32_pmu_lvd_disable(void)
{
  /* Disable LVD */

  modifyreg32(GD32_PMU_CTL, PMU_CTL_LVDEN, 0);
}

/****************************************************************************
 * Name: gd32_pmu_ldo_output_select
 *
 * Description:
 *   Select the LDO output voltage.  This bit is set by software when the
 *   main PLL is closed; before closing the PLL, change the system clock to
 *   IRC16M or HXTAL.
 *
 * Input Parameters:
 *   ldo_output - PMU_CTL_LDOVS(n), PMU LDO output voltage select
 *    PMU_CTL_LDOVS(1): LDO output voltage select normal mode
 *    PMU_CTL_LDOVS(3): LDO output voltage select low mode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_ldo_output_select(uint32_t ldo_output)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CTL_LDOVS_MASK;
  regval |= ldo_output;
  putreg32(regval, GD32_PMU_CTL);
}

/****************************************************************************
 * Name: gd32_pmu_to_sleepmode
 *
 * Description:
 *   PMU work in sleep mode.
 *
 * Input Parameters:
 *   sleepmodecmd - PMU command constants
 *        - WFI_CMD: use WFI command
 *        - WFE_CMD: use WFE command
 *   sleeponexit
 *        - true:  SLEEPONEXIT bit is set when the WFI instruction is
 *                 executed, the MCU enters Sleep mode as soon as it
 *                 exits the lowest priority ISR.
 *        - false: SLEEPONEXIT bit is cleared, the MCU enters Sleep
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_sleepmode(uint8_t sleepmodecmd, bool sleeponexit)
{
  uint32_t regval;

  /* Clear SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;

  if (sleeponexit)
    {
      regval |= NVIC_SYSCON_SLEEPONEXIT;
    }
  else
    {
      regval &= ~NVIC_SYSCON_SLEEPONEXIT;
    }

  putreg32(regval, NVIC_SYSCON);

  /* Select WFI or WFE command to enter sleep mode */

  if (sleepmodecmd == WFI_CMD)
    {
      gd32_pmu_wfi();
    }
  else
    {
      asm("sev");
      asm("wfe");
      asm("wfe");
    }
}

/****************************************************************************
 * Name: gd32_pmu_to_deepsleepmode
 *
 * Description:
 *   PMU work in deep-sleep mode
 *
 * Input Parameters:
 *   ldo
 *       - PMU_LDO_NORMAL:   LDO normal work when pmu enter deep-sleep mode
 *       - PMU_LDO_LOWPOWER: LDO work at low power mode when pmu enter
 *                           deep-sleep mode
 *
 *   deepsleepmodecmd - PMU command constants
 *       - WFI_CMD: use WFI command
 *       - WFE_CMD: use WFE command
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_deepsleepmode(uint32_t ldo, uint8_t deepsleepmodecmd)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CTL);

  /* Clear stbmod and ldolp bits */

  regval &= ~(PMU_CTL_STBMOD | PMU_CTL_LDOLP);

  /* Set ldolp bit according to ldo */

  regval |= ldo;

  putreg32(regval, GD32_PMU_CTL);

  /* Set SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  gd32_pmu_reg_snap[0] = getreg32(NVIC_SYSTICK_CTRL);
  gd32_pmu_reg_snap[1] = getreg32(NVIC_IRQ0_31_ENABLE);
  gd32_pmu_reg_snap[2] = getreg32(NVIC_IRQ32_63_ENABLE);
  gd32_pmu_reg_snap[3] = getreg32(NVIC_IRQ64_95_ENABLE);

  putreg32((0x00010004u & gd32_pmu_reg_snap[0]), NVIC_SYSTICK_CTRL);
  putreg32(0xff7ff83du, NVIC_IRQ0_31_CLEAR);
  putreg32(0xfffff8ffu, NVIC_IRQ32_63_CLEAR);
  putreg32(0xffffffffu, NVIC_IRQ64_95_CLEAR);

  /* Select WFI or WFE command to enter deep-sleep mode */

  if (deepsleepmodecmd == WFI_CMD)
    {
      gd32_pmu_wfi();
    }
  else
    {
      asm("sev");
      asm("wfe");
      asm("wfe");
    }

  putreg32(gd32_pmu_reg_snap[0], NVIC_SYSTICK_CTRL);
  putreg32(gd32_pmu_reg_snap[1], NVIC_IRQ0_31_ENABLE);
  putreg32(gd32_pmu_reg_snap[2], NVIC_IRQ32_63_ENABLE);
  putreg32(gd32_pmu_reg_snap[3], NVIC_IRQ64_95_ENABLE);

  /* Reset SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);
}

/****************************************************************************
 * Name: gd32_pmu_to_standbymode
 *
 * Description:
 *   PMU work in standby mode
 *
 * Input Parameters:
 *   standbymodecmd - PMU command constants
 *                  WFI_CMD: use WFI command
 *                  WFE_CMD: use WFE command
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_standbymode(uint8_t standbymodecmd)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CTL);

  /* Set stbmod bit */

  regval |= PMU_CTL_STBMOD;

  /* Reset wakeup flag */

  regval |= PMU_CTL_WURST;
  putreg32(regval, GD32_PMU_CTL);

  /* Set SLEEPDEEP bit of Cortex-M4 System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  putreg32((0x00010004u & getreg32(NVIC_SYSTICK_CTRL)), NVIC_SYSTICK_CTRL);
  putreg32(0xfffffff7u, NVIC_IRQ0_31_CLEAR);
  putreg32(0xfffffdffu, NVIC_IRQ32_63_CLEAR);
  putreg32(0xffffffffu, NVIC_IRQ64_95_CLEAR);

  /* Select WFI or WFE command to enter standby mode */

  if (standbymodecmd == WFI_CMD)
    {
      asm("wfi");
    }
  else
    {
      asm("wfe");
      asm("wfe");
    }
}

/****************************************************************************
 * Name: gd32_pmu_wakeup_pin_enable
 *
 * Description:
 *   Enables PMU wakeup pin.
 *
 ****************************************************************************/

void gd32_pmu_wakeup_pin_enable(void)
{
  modifyreg32(GD32_PMU_CS, 0, PMU_CS_WUPEN);
}

/****************************************************************************
 * Name: gd32_pmu_wakeup_pin_disable
 *
 * Description:
 *   Disables PMU wakeup pin.
 *
 ****************************************************************************/

void gd32_pmu_wakeup_pin_disable(void)
{
  modifyreg32(GD32_PMU_CS, PMU_CS_WUPEN, 0);
}

/****************************************************************************
 * Name: gd32_pmu_backup_init
 *
 * Description:
 *   Insures the referenced count access to the backup domain
 *   (RTC registers, backup data registers) is consistent with the
 *   hardware state without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - set the initial state of the enable or disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_backup_init(bool writable)
{
  /* Make the hardware not writable */

  modifyreg32(GD32_PMU_CTL, PMU_CTL_BKPWEN, 0);

  if (writable)
    {
      gd32_pmu_backup_write_enable();
    }
  else
    {
      gd32_pmu_backup_write_disable();
    }
}

/****************************************************************************
 * Name: gd32_pmu_backup_write_enable
 *
 * Description:
 *   Enables write access to the registers in backup domain
 *
 ****************************************************************************/

void gd32_pmu_backup_write_enable(void)
{
  modifyreg32(GD32_PMU_CTL, 0, PMU_CTL_BKPWEN);
}

/****************************************************************************
 * Name: gd32_pmu_backup_write_disable
 *
 * Description:
 *   Disables write access to the registers in backup domain
 *
 ****************************************************************************/

void gd32_pmu_backup_write_disable(void)
{
  modifyreg32(GD32_PMU_CTL, PMU_CTL_BKPWEN, 0);
}

/****************************************************************************
 * Name: gd32_pmu_flag_get
 *
 * Description:
 *   Get flag state
 *
 * Input Parameters:
 *   flag - PMU_CS_WUF:  wakeup flag
 *        - PMU_CS_STBF: standby flag
 *        - PMU_CS_LVDF: lvd flag
 *
 ****************************************************************************/

bool gd32_pmu_flag_get(uint32_t flag)
{
  if (getreg32(GD32_PMU_CS) & flag)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: gd32_pmu_flag_clear
 *
 * Description:
 *   Clear the flag
 *
 * Input Parameters:
 *   flag - PMU_FLAG_RESET_WAKEUP: reset wakeup flag
 *        - PMU_FLAG_RESET_STANDBY: reset standby flag
 *
 ****************************************************************************/

void gd32_pmu_flag_clear(uint32_t flag)
{
  uint32_t regval;

  regval = getreg32(GD32_PMU_CTL);

  switch (flag)
    {
      case PMU_FLAG_RESET_WAKEUP:

        /* Reset wakeup flag */

        regval |= PMU_CTL_WURST;
        putreg32(regval, GD32_PMU_CTL);
        break;
      case PMU_FLAG_RESET_STANDBY:

        /* Reset standby flag */

        regval |= PMU_CTL_STBRST;
        putreg32(regval, GD32_PMU_CTL);
        break;
      default:
        break;
    }
}

/****************************************************************************
 * Name: gd32_pmsleep
 *
 * Description:
 *   Enter SLEEP mode.  This is the lightest low-power mode: only the CPU
 *   clock is stopped, all peripherals keep running and the MCU wakes on any
 *   interrupt (or event).  It maps directly to the NuttX PM_IDLE/PM_STANDBY
 *   handling used by up_idlepm().
 *
 * Input Parameters:
 *   sleeponexit - true:  the MCU re-enters Sleep as soon as it exits the
 *                        lowest priority ISR (SLEEPONEXIT set).
 *               - false: the MCU enters Sleep only on the WFI/WFE below.
 *
 * Returned Value:
 *   Zero (OK) after the MCU has been re-awakened.
 *
 ****************************************************************************/

int gd32_pmsleep(bool sleeponexit)
{
  gd32_pmu_to_sleepmode(WFI_CMD, sleeponexit);
  return OK;
}

/****************************************************************************
 * Name: gd32_pmdeepsleep
 *
 * Description:
 *   Enter DEEP-SLEEP mode.
 *   The CPU and most clocks are stopped while SRAM and register contents are
 *   retained.  The MCU is woken by an EXTI line (e.g. the WKUP button) or an
 *   RTC event, after which execution resumes from this function.
 *
 * Input Parameters:
 *   lpds - true:  keep the internal LDO in low-power mode while stopped to
 *                 further reduce consumption.
 *        - false: keep the LDO in normal mode for a faster wakeup.
 *
 * Returned Value:
 *   Zero (OK) after the MCU has been re-awakened.
 *
 ****************************************************************************/

int gd32_pmdeepsleep(bool lpds)
{
  gd32_pmu_to_deepsleepmode(lpds ? PMU_LDO_LOWPOWER : PMU_LDO_NORMAL,
                            WFI_CMD);

#ifdef CONFIG_PM
  /* Leaving Deep-sleep mode switches the system clock back to IRC8M and
   * turns the PLL off.  The clocking must be re-established here or every
   * clock-derived peripheral (SysTick, the serial console baud rate, ...)
   * would keep running from the wrong frequency.
   */

  gd32_clock_enable();
#endif

  return OK;
}

/****************************************************************************
 * Name: gd32_pmstandby
 *
 * Description:
 *   Enter STANDBY mode, the deepest low-power mode.  The 1.2V core domain is
 *   powered off; only the backup domain and standby circuitry remain alive.
 *   The MCU is woken by the WKUP pin, an RTC event or an external/watchdog
 *   reset, and resumes execution from reset (this function does not return).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned nominally, but STANDBY can only be terminated by a
 *   reset so this function does not normally return.
 *
 ****************************************************************************/

int gd32_pmstandby(void)
{
  /* Standby can only be left through the WKUP pin, an RTC event or a reset.
   * Make sure the WKUP pin (PA0) is armed before stopping the core domain,
   * otherwise the board could only be recovered with a reset.
   */

  gd32_pmu_wakeup_pin_enable();
  gd32_pmu_flag_clear(PMU_FLAG_RESET_WAKEUP);

  gd32_pmu_to_standbymode(WFI_CMD);
  return OK;
}

#endif /* CONFIG_GD32E11X_PMU */