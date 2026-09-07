/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_fmc.c
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

/* Measured on GD32E113: a read of the main flash while the FMC is erasing
 * or programming stalls on the bus until the operation completes and then
 * returns correct data, so no special .ramfunc placement is needed here.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include <stdbool.h>
#include <errno.h>

#include <arch/barriers.h>

#include "gd32e11x.h"
#include "gd32e11x_fmc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_gd32_fmc_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_state_get
 *
 * Description:
 *   Get the FMC state
 *
 * Returned Value:
 *  State of FMC
 *
 ****************************************************************************/

static gd32_fmc_state_enum gd32_fmc_state_get(void)
{
  uint32_t regval = getreg32(GD32_FMC_STAT);

  if (regval & FMC_STAT_BUSY)
    {
      return FMC_BUSY;
    }
  else if (regval & FMC_STAT_WPERR)
    {
      return FMC_WPERR;
    }
  else if (regval & FMC_STAT_PGERR)
    {
      return FMC_PGERR;
    }
  else if (regval & FMC_STAT_PGAERR)
    {
      return FMC_PGAERR;
    }

  return FMC_READY;
}

/****************************************************************************
 * Name: gd32_fmc_ready_wait
 *
 * Description:
 *   Wait until the FMC leaves the busy state
 *
 * Returned Value:
 *  State of FMC, FMC_TOERR if the operation did not complete in time
 *
 ****************************************************************************/

static gd32_fmc_state_enum gd32_fmc_ready_wait(uint32_t timeout)
{
  gd32_fmc_state_enum fmc_state;

  do
    {
      fmc_state = gd32_fmc_state_get();
      timeout--;
    }
  while ((FMC_BUSY == fmc_state) && (timeout > 0));

  if (FMC_BUSY == fmc_state)
    {
      fmc_state = FMC_TOERR;
    }

  return fmc_state;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_wscnt_set
 *
 * Description:
 *   Set the wait state counter value
 *
 * Parameters:
 *   wscnt - Wait state counter value
 *
 ****************************************************************************/

void gd32_fmc_wscnt_set(uint32_t wscnt)
{
  uint32_t regval;

  regval = getreg32(GD32_FMC_WS);

  /* Set the wait state counter value */

  regval &= ~FMC_WS_WSCNT_MASK;
  regval |= (wscnt & FMC_WS_WSCNT_MASK);
  putreg32(regval, GD32_FMC_WS);
}

/****************************************************************************
 * Name: gd32_fmc_unlock
 *
 * Description:
 *   Unlock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_unlock(void)
{
  int ret;

  ret = nxmutex_lock(&g_gd32_fmc_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (getreg32(GD32_FMC_CTL) & FMC_CTL_LK)
    {
      /* Write the FMC unlock key */

      putreg32(FMC_UNLOCK_KEY0, GD32_FMC_KEY);
      putreg32(FMC_UNLOCK_KEY1, GD32_FMC_KEY);
    }

  nxmutex_unlock(&g_gd32_fmc_lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_fmc_lock
 *
 * Description:
 *   Lock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_lock(void)
{
  int ret;

  ret = nxmutex_lock(&g_gd32_fmc_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the LK bit */

  modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_LK);

  nxmutex_unlock(&g_gd32_fmc_lock);
  return ret;
}

/****************************************************************************
 * Name: gd32_fmc_page_erase
 *
 * Description:
 *   Erase one main flash page
 *
 * Parameters:
 *   page_addr - Any address inside the page to erase
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_page_erase(uint32_t page_addr)
{
  gd32_fmc_state_enum fmc_state;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start the page erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_PER);
      putreg32(page_addr, GD32_FMC_ADDR);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      UP_DSB();
      UP_ISB();

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PER bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PER, 0);
    }

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_mass_erase
 *
 * Description:
 *   Erase the whole main flash memory
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_mass_erase(void)
{
  gd32_fmc_state_enum fmc_state;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start the whole chip erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      UP_DSB();
      UP_ISB();

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the MER bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER, 0);
    }

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_doubleword_program
 *
 * Description:
 *   Program a double word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Double word to program
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum
gd32_fmc_doubleword_program(uint32_t address, uint64_t data)
{
  gd32_fmc_state_enum fmc_state;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Select the 64-bit program width and set the PG bit to start */

      modifyreg32(GD32_FMC_WS, 0, FMC_WS_PGW);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_PG);

      putreg32((uint32_t)data, address);
      putreg32((uint32_t)(data >> 32), address + 4);

      /* The flash is normal memory, so the writes have to be completed
       * before the busy flag is sampled.
       */

      UP_DSB();

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG and PGW bits */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PG, 0);
      modifyreg32(GD32_FMC_WS, FMC_WS_PGW, 0);
    }

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_word_program
 *
 * Description:
 *   Program a word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Word to program(0x00000000 - 0xFFFFFFFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_word_program(uint32_t address,
                                          uint32_t data)
{
  gd32_fmc_state_enum fmc_state;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Select the 32-bit program width and set the PG bit to start */

      modifyreg32(GD32_FMC_WS, FMC_WS_PGW, 0);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_PG);

      putreg32(data, address);

      /* The flash is normal memory, so the write has to be completed before
       * the busy flag is sampled.
       */

      UP_DSB();

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PG, 0);
    }

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_ob_unlock
 *
 * Description:
 *   Unlock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_unlock(void)
{
  if ((getreg32(GD32_FMC_CTL) & FMC_CTL_OBWEN) == 0)
    {
      /* Write the option bytes unlock key */

      putreg32(FMC_UNLOCK_KEY0, GD32_FMC_OBKEY);
      putreg32(FMC_UNLOCK_KEY1, GD32_FMC_OBKEY);
    }
}

/****************************************************************************
 * Name: gd32_ob_lock
 *
 * Description:
 *   Lock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_lock(void)
{
  /* Reset the OBWEN bit */

  modifyreg32(GD32_FMC_CTL, FMC_CTL_OBWEN, 0);
}

/****************************************************************************
 * Name: gd32_ob_erase
 *
 * Description:
 *   Erase the whole option bytes block and restore the current security
 *   protection level.  Every other option byte returns to its erased state.
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_ob_erase(void)
{
  gd32_fmc_state_enum fmc_state;
  uint32_t spc_user;

  /* Sample the security protection code before the block is erased.  The
   * erased state of the code means protected, so it has to be programmed
   * again.  Every other option byte is left in its erased state.
   */

  spc_user = GD32_OB_WORD_HIGH_ERASED |
             (gd32_ob_spc_get() ? FMC_USPC : FMC_NSPC);

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start the option bytes erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_OBER);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      modifyreg32(GD32_FMC_CTL, FMC_CTL_OBER, 0);

      if (FMC_READY == fmc_state)
        {
          /* Restore the security protection code */

          modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_OBPG);
          putreg32(spc_user, GD32_OB_WORD(GD32_OB_WORD_SPC_USER));

          fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

          modifyreg32(GD32_FMC_CTL, FMC_CTL_OBPG, 0);
        }
    }

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_ob_wp_get
 *
 * Description:
 *   Get the current write protection of the main flash sectors
 *
 ****************************************************************************/

uint32_t gd32_ob_wp_get(void)
{
  return getreg32(GD32_FMC_WP);
}

/****************************************************************************
 * Name: gd32_ob_spc_get
 *
 * Description:
 *   Check whether the security protection is active
 *
 ****************************************************************************/

bool gd32_ob_spc_get(void)
{
  return (getreg32(GD32_FMC_OBSTAT) & FMC_OBSTAT_SPC) != 0;
}

/****************************************************************************
 * Name: gd32_ob_wp_config
 *
 * Description:
 *   Erase the option bytes block and program it again with the given write
 *   protection value
 *
 * Parameters:
 *   ob_wp - Write protection value, a cleared bit protects its sectors
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

static gd32_fmc_state_enum gd32_ob_wp_config(uint32_t ob_wp)
{
  gd32_fmc_state_enum fmc_state;
  uint32_t obword[GD32_OB_NWORDS];
  int i;

  /* Keep the content of the option bytes that is not related to the write
   * protection, the block is erased as a whole.
   */

  for (i = 0; i < GD32_OB_NWORDS; i++)
    {
      obword[i] = getreg32(GD32_OB_WORD(i));
    }

  obword[GD32_OB_WORD_WP0] = (ob_wp & 0x000000ff) |
                             ((ob_wp & 0x0000ff00) << 8);
  obword[GD32_OB_WORD_WP1] = ((ob_wp & 0x00ff0000) >> 16) |
                             ((ob_wp & 0xff000000) >> 8);

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_OBER);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      modifyreg32(GD32_FMC_CTL, FMC_CTL_OBER, 0);

      if (FMC_READY == fmc_state)
        {
          modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_OBPG);

          for (i = 0; i < GD32_OB_NWORDS && FMC_READY == fmc_state; i++)
            {
              putreg32(obword[i], GD32_OB_WORD(i));
              fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);
            }

          modifyreg32(GD32_FMC_CTL, FMC_CTL_OBPG, 0);
        }
    }

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_ob_write_protection_enable
 *
 * Description:
 *   Enable write protection.  The new setting takes effect after the next
 *   power on reset.
 *
 * Parameters:
 *   ob_wp - Specify sector to be write protected
 *
 ****************************************************************************/

int gd32_ob_write_protection_enable(uint32_t ob_wp)
{
  gd32_fmc_state_enum fmc_state;

  if (gd32_ob_spc_get())
    {
      /* Refuse while the security protection is active: the exact
       * hardware behavior of erasing the option bytes in that state
       * has not been verified against the reference manual.
       */

      return -EACCES;
    }

  gd32_fmc_unlock();
  gd32_ob_unlock();

  fmc_state = gd32_ob_wp_config(gd32_ob_wp_get() & ~ob_wp);

  gd32_ob_lock();
  gd32_fmc_lock();

  return (FMC_READY == fmc_state) ? OK : -EIO;
}

/****************************************************************************
 * Name: gd32_ob_write_protection_disable
 *
 * Description:
 *   Disable write protection.  The new setting takes effect after the next
 *   power on reset.
 *
 * Parameters:
 *   ob_wp - Specify sector whose write protection is removed
 *
 ****************************************************************************/

int gd32_ob_write_protection_disable(uint32_t ob_wp)
{
  gd32_fmc_state_enum fmc_state;

  if (gd32_ob_spc_get())
    {
      return -EACCES;
    }

  gd32_fmc_unlock();
  gd32_ob_unlock();

  fmc_state = gd32_ob_wp_config(gd32_ob_wp_get() | ob_wp);

  gd32_ob_lock();
  gd32_fmc_lock();

  return (FMC_READY == fmc_state) ? OK : -EIO;
}

/****************************************************************************
 * Name: gd32_fmc_flag_clear
 *
 * Description:
 *   Clear the FMC pending flag
 *
 * Parameters:
 *   fmc_flag - FMC flag
 *
 ****************************************************************************/

void gd32_fmc_flag_clear(uint32_t fmc_flag)
{
  putreg32(fmc_flag, GD32_FMC_STAT);
}
