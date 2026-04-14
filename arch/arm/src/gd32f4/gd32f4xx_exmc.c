/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_exmc.c
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

#include <assert.h>

#include "gd32f4xx.h"

#if defined(CONFIG_GD32F4_EXMC)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_exmc_sdram_wait
 *
 * Description:
 *   Wait for the SDRAM controller to be ready.
 *
 ****************************************************************************/

void gd32_exmc_sdram_wait(void)
{
  int timeout = 0xffff;
  while (timeout > 0)
    {
      if ((getreg32(GD32_EXMC_SDSTAT) & EXMC_SDSDAT_NRDY) == 0)
        {
          break;
        }

      timeout--;
    }

  DEBUGASSERT(timeout > 0);
}

/****************************************************************************
 * Name: gd32_exmc_enable
 *
 * Description:
 *   Enable clocking to the EXMC.
 *
 ****************************************************************************/

void gd32_exmc_enable(void)
{
  modifyreg32(GD32_RCU_AHB3EN, 0, RCU_AHB3EN_EXMCEN);
}

/****************************************************************************
 * Name: gd32_exmc_disable
 *
 * Description:
 *   Disable clocking to the EXMC.
 *
 ****************************************************************************/

void gd32_exmc_disable(void)
{
  modifyreg32(GD32_RCU_AHB3EN, RCU_AHB3EN_EXMCEN, 0);
}

/****************************************************************************
 * Name: gd32_exmc_sdram_write_protect
 *
 * Description:
 *   Enable/Disable writes to an SDRAM.
 *
 ****************************************************************************/

void gd32_exmc_sdram_write_protect(int bank, bool state)
{
  uint32_t val;
  uint32_t sdcr;

  DEBUGASSERT(bank == 1 || bank == 2);
  sdcr = (bank == 1) ? GD32_EXMC_SDCTL0 : GD32_EXMC_SDCTL1;

  gd32_exmc_sdram_wait();

  val = getreg32(sdcr);
  if (state)
    {
      val |= EXMC_SDCTL_WPEN;       /* wp == 1 */
    }
  else
    {
      val &= ~EXMC_SDCTL_WPEN;      /* wp == 0 */
    }

  putreg32(val, sdcr);
}

/****************************************************************************
 * Name: gd32_exmc_sdram_set_refresh_rate
 *
 * Description:
 *   Set the SDRAM refresh rate.
 *
 ****************************************************************************/

void gd32_exmc_sdram_set_refresh_rate(int count)
{
  uint32_t val;

  DEBUGASSERT(count <= 0x1fff && count >= 0x29);

  gd32_exmc_sdram_wait();

  val  = getreg32(GD32_EXMC_SDARI);
  val &= ~(0x1fff << 1);        /* preserve non-count bits */
  val |= (count << 1);
  putreg32(val, GD32_EXMC_SDARI);
}

/****************************************************************************
 * Name: gd32_exmc_sdram_set_timing
 *
 * Description:
 *   Set the SDRAM timing parameters.
 *
 ****************************************************************************/

void gd32_exmc_sdram_set_timing(int bank, uint32_t timing)
{
  uint32_t val;
  uint32_t sdtr;

  DEBUGASSERT((bank == 1) || (bank == 2));
  DEBUGASSERT((timing & EXMC_SDTCFG_RESERVED) == 0);

  sdtr = (bank == 1) ? GD32_EXMC_SDTCFG0 : GD32_EXMC_SDTCFG1;
  val  = getreg32(sdtr);
  val &= EXMC_SDTCFG_RESERVED;     /* preserve reserved bits */
  val |= timing;
  putreg32(val, sdtr);
}

/****************************************************************************
 * Name: gd32_exmc_sdram_set_control
 *
 * Description:
 *   Set the SDRAM control parameters.
 *
 ****************************************************************************/

void gd32_exmc_sdram_set_control(int bank, uint32_t ctrl)
{
  uint32_t val;
  uint32_t sdcr;

  DEBUGASSERT((bank == 1) || (bank == 2));
  DEBUGASSERT((ctrl & EXMC_SDCTL_RESERVED) == 0);

  sdcr = (bank == 1) ? GD32_EXMC_SDCTL0 : GD32_EXMC_SDCTL1;
  val  = getreg32(sdcr);
  val &= EXMC_SDCTL_RESERVED;     /* preserve reserved bits */
  val |= ctrl;
  putreg32(val, sdcr);
}

/****************************************************************************
 * Name: gd32_exmc_sdram_command
 *
 * Description:
 *   Send a command to the SDRAM.
 *
 ****************************************************************************/

void gd32_exmc_sdram_command(uint32_t cmd)
{
  uint32_t val;

  DEBUGASSERT((cmd & EXMC_SDCMD_RESERVED) == 0);

  /* Wait for the controller to be ready */

  gd32_exmc_sdram_wait();

  val  = getreg32(GD32_EXMC_SDCMD);
  val &= EXMC_SDCMD_RESERVED;    /* Preserve reserved bits */
  val |= cmd;
  putreg32(val, GD32_EXMC_SDCMD);
}

#endif /* CONFIG_GD32F4_EXMC */
