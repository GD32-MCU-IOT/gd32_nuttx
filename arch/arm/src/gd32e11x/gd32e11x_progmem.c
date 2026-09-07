/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_progmem.c
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
#include <string.h>
#include <limits.h>
#include <errno.h>

#include <nuttx/progmem.h>
#include <nuttx/mutex.h>

#include "gd32e11x_progmem.h"
#include "gd32e11x_fmc.h"
#include "gd32e11x.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_gd32_progmem_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_progmem_isprotected
 *
 * Description:
 *   Check whether the given progmem page is covered by an option bytes
 *   write protection group
 *
 * Input Parameters:
 *   page - The progmem page index
 *
 * Returned Value:
 *   True if the page cannot be erased or programmed
 *
 ****************************************************************************/

static bool gd32_progmem_isprotected(size_t page)
{
  size_t bit;

  bit = (GD32_PROGMEM_STARTPAGE + page) / FMC_WP_PAGES_PER_BIT;
  if (bit > 31)
    {
      /* The last protection bit covers all the remaining pages */

      bit = 31;
    }

  return (gd32_ob_wp_get() & (1ul << bit)) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_neraseblocks
 *
 * Description:
 *   Return number of erase blocks
 *
 ****************************************************************************/

size_t up_progmem_neraseblocks(void)
{
  return GD32_PROGMEM_NPAGES;
}

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   Is program memory uniform or page size differs?
 *
 ****************************************************************************/

bool up_progmem_isuniform(void)
{
  return true;
}

/****************************************************************************
 * Name: up_progmem_erasesize
 *
 * Description:
 *   Return erase block size
 *
 ****************************************************************************/

size_t up_progmem_erasesize(size_t block)
{
  if (block >= GD32_PROGMEM_NPAGES)
    {
      return 0;
    }

  return GD32_FLASH_PAGESIZE;
}

/****************************************************************************
 * Name: up_progmem_pagesize
 *
 * Description:
 *   Return read/write page size
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  return up_progmem_erasesize(page);
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to read/write page conversion
 *
 * Input Parameters:
 *   addr - Address with or without flash offset
 *          (absolute or aligned to page0)
 *
 * Returned Value:
 *   Page or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     -EFAULT: On invalid address
 *
 ****************************************************************************/

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr < GD32_PROGMEM_STARTADDR)
    {
      if (addr >= GD32_PROGMEM_SIZE)
        {
          return -EFAULT;
        }

      addr += GD32_PROGMEM_STARTADDR;
    }

  if (addr >= GD32_PROGMEM_ENDADDR)
    {
      return -EFAULT;
    }

  return (addr - GD32_PROGMEM_STARTADDR) / GD32_FLASH_PAGESIZE;
}

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Read/write page to address conversion
 *
 * Input Parameters:
 *   page - page index
 *
 * Returned Value:
 *   Base address of given page, SIZE_MAX if page index is not valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t page)
{
  if (page >= GD32_PROGMEM_NPAGES)
    {
      return SIZE_MAX;
    }

  return GD32_PROGMEM_STARTADDR + page * GD32_FLASH_PAGESIZE;
}

/****************************************************************************
 * Name: up_progmem_ispageerased
 *
 * Description:
 *   Checks whether page is erased
 *
 * Input Parameters:
 *   page - The erase page index to be checked.
 *
 * Returned Value:
 *   Returns number of bytes NOT erased or negative value on error. If it
 *   returns zero then complete page is erased.
 *
 *   The following errors are reported:
 *     -EFAULT: On invalid page
 *
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= GD32_PROGMEM_NPAGES)
    {
      return -EFAULT;
    }

  for (addr = up_progmem_getaddress(page), count = GD32_FLASH_PAGESIZE;
       count > 0; count--, addr++)
    {
      if (getreg8(addr) != GD32_PROGMEM_ERASEDVAL)
        {
          bwritten++;
        }
    }

  return bwritten;
}

/****************************************************************************
 * Name: up_progmem_eraseblock
 *
 * Description:
 *   Erase selected block.
 *
 * Input Parameters:
 *   block - The erase block index to be erased.
 *
 * Returned Value:
 *   block size or negative value on error.  The following errors are
 *   reported (errno is not set!):
 *
 *     -EFAULT: On invalid page
 *     -EIO:    On unsuccessful erase
 *     -EROFS:  On access to write protected area
 *     -EACCES: Insufficient permissions (read/write protected)
 *     -EPERM:  If operation is not permitted due to some other constraints
 *              (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_eraseblock(size_t block)
{
  gd32_fmc_state_enum fmc_state;
  size_t addr;
  int ret;

  if (block >= GD32_PROGMEM_NPAGES)
    {
      return -EFAULT;
    }

  if (gd32_progmem_isprotected(block))
    {
      return -EROFS;
    }

  addr = up_progmem_getaddress(block);

  ret = nxmutex_lock(&g_gd32_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Get flash ready and begin erasing the single page */

  ret = gd32_fmc_unlock();
  if (ret < 0)
    {
      nxmutex_unlock(&g_gd32_progmem_lock);
      return ret;
    }

  gd32_fmc_flag_clear(FMC_STAT_PERR);

  fmc_state = gd32_fmc_page_erase(addr);

  gd32_fmc_lock();
  nxmutex_unlock(&g_gd32_progmem_lock);

  if (FMC_READY != fmc_state)
    {
      return (FMC_WPERR == fmc_state) ? -EROFS : -EIO;
    }

  /* Verify */

  if (up_progmem_ispageerased(block) != 0)
    {
      return -EIO;
    }

  return GD32_FLASH_PAGESIZE;
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at given address
 *
 *   Note: this function is not limited to single page and nor it requires
 *   the address be aligned inside the page boundaries.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to page0)
 *   buf   - Pointer to buffer
 *   count - Number of bytes to write
 *
 * Returned Value:
 *   Bytes written or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EINVAL: If count is not aligned with the flash boundaries (i.e.
 *             some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO:    On unsuccessful write
 *     EROFS:  On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  const uint8_t *src = (const uint8_t *)buf;
  size_t written = 0;
  size_t page;
  size_t lastpage;
  uint32_t word;
  gd32_fmc_state_enum fmc_state;
  int ret;

  /* The FMC programs words only */

  if (((addr | count) & GD32_PROGMEM_BLOCKMASK) != 0)
    {
      return -EINVAL;
    }

  if (count == 0)
    {
      return 0;
    }

  /* Check for valid address range */

  if (addr < GD32_PROGMEM_STARTADDR)
    {
      if (addr >= GD32_PROGMEM_SIZE)
        {
          return -EFAULT;
        }

      addr += GD32_PROGMEM_STARTADDR;
    }

  if (addr >= GD32_PROGMEM_ENDADDR ||
      count > GD32_PROGMEM_ENDADDR - addr)
    {
      return -EFAULT;
    }

  lastpage = up_progmem_getpage(addr + count - 1);
  for (page = up_progmem_getpage(addr); page <= lastpage; page++)
    {
      if (gd32_progmem_isprotected(page))
        {
          return -EROFS;
        }
    }

  ret = nxmutex_lock(&g_gd32_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Get flash ready and begin flashing */

  ret = gd32_fmc_unlock();
  if (ret < 0)
    {
      nxmutex_unlock(&g_gd32_progmem_lock);
      return ret;
    }

  gd32_fmc_flag_clear(FMC_STAT_PERR);

  while (written < count)
    {
      /* The caller buffer has no alignment requirement */

      memcpy(&word, src, sizeof(word));

      fmc_state = gd32_fmc_word_program(addr, word);
      if (fmc_state != FMC_READY || getreg32(addr) != word)
        {
          ret = (FMC_WPERR == fmc_state) ? -EROFS : -EIO;
          break;
        }

      addr    += sizeof(word);
      src     += sizeof(word);
      written += sizeof(word);
    }

  gd32_fmc_lock();
  nxmutex_unlock(&g_gd32_progmem_lock);

  return ret < 0 ? ret : (ssize_t)written;
}

/****************************************************************************
 * Name: up_progmem_read
 *
 * Description:
 *   Read data at given address
 *
 *   Note: this function is not limited to single page and nor it requires
 *   the address be aligned inside the page boundaries.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to page0)
 *   buf   - Pointer to buffer
 *   count - Number of bytes to read
 *
 * Returned Value:
 *   Bytes read or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EFAULT: On invalid address
 *     EACCES: Insufficient permissions (read/write protected)
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_PROGMEM_READ
ssize_t up_progmem_read(size_t addr, void *buf, size_t count)
{
  int ret;

  if (addr < GD32_PROGMEM_STARTADDR)
    {
      if (addr > GD32_PROGMEM_SIZE)
        {
          return -EFAULT;
        }

      addr += GD32_PROGMEM_STARTADDR;
    }

  if (addr > GD32_PROGMEM_ENDADDR ||
      count > GD32_PROGMEM_ENDADDR - addr)
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_gd32_progmem_lock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buf, (const void *)addr, count);

  nxmutex_unlock(&g_gd32_progmem_lock);

  return count;
}
#endif

/****************************************************************************
 * Name: up_progmem_erasestate
 *
 * Description:
 *   Return value of erase state.
 *
 ****************************************************************************/

uint8_t up_progmem_erasestate(void)
{
  return GD32_PROGMEM_ERASEDVAL;
}
