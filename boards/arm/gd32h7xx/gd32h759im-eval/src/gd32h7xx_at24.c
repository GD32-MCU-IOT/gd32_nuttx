/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/src/gd32h7xx_at24.c
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>

#include "gd32h7xx_i2c.h"
#include "gd32h759im_eval.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AT24_I2C_ADDR     0x50     /* AT24 EEPROM I2C slave address */
#define AT24_I2C_FREQ     100000   /* 100 kHz I2C bus frequency */
#define AT24_PAGE_SIZE    8        /* AT24C02 page size: 8 bytes */
#define AT24_TEST_SIZE    256      /* Total test byte count (1 full AT24C02) */

#define AT24_NUM_PAGES    ((AT24_TEST_SIZE + AT24_PAGE_SIZE - 1) / AT24_PAGE_SIZE)

/* MTD write/read block size */

#define AT24_BUFF_SIZE    16
#define AT24_NBLOCK       (AT24_BUFF_SIZE / AT24_PAGE_SIZE)
#define AT24_START_BLOCK  0

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_write_buf[AT24_BUFF_SIZE] =
{
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef HAVE_AT24

/****************************************************************************
 * Name: gd32_at24_wr_test
 *
 * Description:
 *   Polled AT24 EEPROM write-then-read test using the MTD layer.
 *   Writes AT24_BUFF_SIZE bytes via MTD bwrite(), reads them back, and
 *   verifies data integrity.
 *
 * Input Parameters:
 *   minor - Minor device number (unused; present for API compatibility)
 *
 * Returned Value:
 *   OK on success; negative errno on failure.
 *
 ****************************************************************************/

int gd32_at24_wr_test(int minor)
{
  static struct i2c_master_s *i2c   = NULL;
  static struct mtd_dev_s    *at24  = NULL;
  static bool                 initialized = false;
  uint8_t *read_buf;
  ssize_t  nblocks;
  int      ret;

  if (!initialized)
    {
      finfo("Initializing I2C%d for AT24 test\n", AT24_BUS);

      i2c = gd32_i2cbus_initialize(AT24_BUS);
      if (i2c == NULL)
        {
          ferr("ERROR: Failed to initialize I2C%d\n", AT24_BUS);
          return -ENODEV;
        }

      at24 = at24c_initialize(i2c);
      if (at24 == NULL)
        {
          ferr("ERROR: Failed to initialize AT24 driver on I2C%d\n",
               AT24_BUS);
          gd32_i2cbus_uninitialize(i2c);
          return -ENODEV;
        }

      initialized = true;
    }

  /* Write AT24_BUFF_SIZE bytes starting at block AT24_START_BLOCK */

  nblocks = at24->bwrite(at24, AT24_START_BLOCK, AT24_NBLOCK, g_write_buf);
  if (nblocks < AT24_NBLOCK)
    {
      ferr("ERROR: AT24 bwrite failed: %zd\n", nblocks);
      ret = (nblocks < 0) ? (int)nblocks : -EIO;
      goto errout;
    }

  read_buf = kmm_malloc(AT24_BUFF_SIZE);
  if (read_buf == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Read back the data */

  nblocks = at24->bread(at24, AT24_START_BLOCK, AT24_NBLOCK, read_buf);
  if (nblocks < AT24_NBLOCK)
    {
      ferr("ERROR: AT24 bread failed: %zd\n", nblocks);
      ret = (nblocks < 0) ? (int)nblocks : -EIO;
      goto errout_free;
    }

  /* Verify */

  if (memcmp(read_buf, g_write_buf, AT24_BUFF_SIZE) != 0)
    {
      ferr("ERROR: AT24 read data mismatch\n");
      ret = -EIO;
      goto errout_free;
    }

  ret = OK;

errout_free:
  kmm_free(read_buf);

errout:
  gd32_i2cbus_uninitialize(i2c);
  initialized = false;
  i2c  = NULL;
  at24 = NULL;

  if (ret == OK)
    {
      syslog(LOG_INFO, "AT24 polled write/read test PASSED\n");
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_at24_multimsg_dma_test
 *
 * Description:
 *   AT24 EEPROM DMA test using i2c_transfer() directly.
 *
 *   Sequence:
 *     1. Write AT24_TEST_SIZE bytes in AT24_PAGE_SIZE chunks.
 *     2. Single-message RX DMA read of AT24_TEST_SIZE bytes (after setting
 *        the address pointer with a prior 1-byte TX transfer).
 *     3. Two-message Repeated-START read:
 *          msg[0]: write 1-byte address pointer (TX, no STOP)
 *          msg[1]: read AT24_TEST_SIZE bytes (RX)
 *        Both messages submitted in a single I2C_TRANSFER() call.
 *
 * Input Parameters:
 *   minor - Minor device number (unused)
 *
 * Returned Value:
 *   OK on success; negative errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32H7_I2C_DMA
int gd32_at24_multimsg_dma_test(int minor)
{
  struct i2c_master_s  *i2c;
  struct i2c_msg_s      msgs[2];
  uint8_t               addr_buf[1];
  uint8_t               tx_buf[AT24_PAGE_SIZE + 1];  /* EEPROM addr + data */
  uint8_t              *read_buf;
  uint8_t               small_buf[2];
  uint8_t              *unaligned_alloc;
  uint8_t              *unaligned_buf;
  char                  line[52];
  int                   page;
  int                   i;
  int                   pos;
  int                   ret;

  syslog(LOG_INFO, "AT24 DMA test: %u bytes (%u pages)\n",
         AT24_TEST_SIZE, AT24_NUM_PAGES);

  i2c = gd32_i2cbus_initialize(AT24_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d\n", AT24_BUS);
      return -ENODEV;
    }

  read_buf = kmm_malloc(AT24_TEST_SIZE);
  if (read_buf == NULL)
    {
      gd32_i2cbus_uninitialize(i2c);
      return -ENOMEM;
    }

  /* Step 1: Write the full EEPROM page by page
   */

  for (page = 0; page < AT24_NUM_PAGES; page++)
    {
      int bytes_left = AT24_TEST_SIZE - page * AT24_PAGE_SIZE;
      int bytes      = (bytes_left > AT24_PAGE_SIZE) ?
                        AT24_PAGE_SIZE : bytes_left;

      tx_buf[0] = (uint8_t)(page * AT24_PAGE_SIZE);  /* Internal address */
      for (i = 1; i <= bytes; i++)
        {
          tx_buf[i] = (uint8_t)((page * AT24_PAGE_SIZE + i) & 0xff);
        }

      msgs[0].frequency = AT24_I2C_FREQ;
      msgs[0].addr      = AT24_I2C_ADDR;
      msgs[0].flags     = 0;
      msgs[0].buffer    = tx_buf;
      msgs[0].length    = bytes + 1;  /* address byte + data */

      ret = I2C_TRANSFER(i2c, msgs, 1);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Write page %d failed: %d\n", page, ret);
          goto errout;
        }

      /* AT24C02 write cycle time: typical 5 ms, max 10 ms */

      nxsig_usleep(10000);
    }

  syslog(LOG_INFO, "Write %u bytes done\n", AT24_TEST_SIZE);

  /* Step 2: Single-message DMA read (TX addr pointer + RX
   * data, issued as separate calls).
   */

  addr_buf[0] = 0x00;

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 1;

  ret = I2C_TRANSFER(i2c, msgs, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Set address pointer failed: %d\n", ret);
      goto errout;
    }

  memset(read_buf, 0, AT24_TEST_SIZE);

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = I2C_M_READ;
  msgs[0].buffer    = read_buf;
  msgs[0].length    = AT24_TEST_SIZE;

  ret = I2C_TRANSFER(i2c, msgs, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Single-msg DMA read failed: %d\n", ret);
      goto errout;
    }

  /* Hex dump */

  syslog(LOG_INFO, "Single-msg DMA read %u bytes:\n", AT24_TEST_SIZE);
  for (i = 0; i < AT24_TEST_SIZE; i += 16)
    {
      int j;
      pos = 0;
      for (j = 0; j < 16 && (i + j) < AT24_TEST_SIZE; j++)
        {
          pos += snprintf(&line[pos], sizeof(line) - (size_t)pos,
                          "%02x ", read_buf[i + j]);
        }

      syslog(LOG_INFO, "  %02x: %s\n", i, line);
    }

  /* Verify */

  for (i = 0; i < AT24_TEST_SIZE; i++)
    {
      if (read_buf[i] != ((i + 1) & 0xff))
        {
          syslog(LOG_ERR,
                 "ERROR: Mismatch at %d: "
                 "exp 0x%02x got 0x%02x\n",
                 i, (i + 1) & 0xff, read_buf[i]);
          ret = -EIO;
          goto errout;
        }
    }

  syslog(LOG_INFO, "Single-msg DMA read PASSED\n");

  /* Step 3: Multi-message Repeated-START DMA read
   * msg[0]: TX 1-byte EEPROM address (no STOP)
   * msg[1]: RX AT24_TEST_SIZE bytes
   */

  addr_buf[0] = 0x00;

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = 0;            /* Write without stop */
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 1;

  memset(read_buf, 0, AT24_TEST_SIZE);

  msgs[1].frequency = AT24_I2C_FREQ;
  msgs[1].addr      = AT24_I2C_ADDR;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = read_buf;
  msgs[1].length    = AT24_TEST_SIZE;

  ret = I2C_TRANSFER(i2c, msgs, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Multi-msg DMA read failed: %d\n", ret);
      goto errout;
    }

  /* Hex dump */

  syslog(LOG_INFO, "Multi-msg DMA read %u bytes:\n", AT24_TEST_SIZE);
  for (i = 0; i < AT24_TEST_SIZE; i += 16)
    {
      int j;
      pos = 0;
      for (j = 0; j < 16 && (i + j) < AT24_TEST_SIZE; j++)
        {
          pos += snprintf(&line[pos], sizeof(line) - (size_t)pos,
                          "%02x ", read_buf[i + j]);
        }

      syslog(LOG_INFO, "  %02x: %s\n", i, line);
    }

  /* Verify */

  for (i = 0; i < AT24_TEST_SIZE; i++)
    {
      if (read_buf[i] != ((i + 1) & 0xff))
        {
          syslog(LOG_ERR,
                 "ERROR: Mismatch at %d: "
                 "exp 0x%02x got 0x%02x\n",
                 i, (i + 1) & 0xff, read_buf[i]);
          ret = -EIO;
          goto errout;
        }
    }

  syslog(LOG_INFO, "Multi-msg DMA read PASSED\n");

  /* Step 4: Small transfer below DMA threshold (should
   * use polling even with DMA enabled).  Read only 2 bytes
   * from offset 0.
   */

  syslog(LOG_INFO, "Small read (2 bytes, below DMA threshold) ...\n");

  addr_buf[0] = 0x00;

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 1;

  memset(small_buf, 0, sizeof(small_buf));

  msgs[1].frequency = AT24_I2C_FREQ;
  msgs[1].addr      = AT24_I2C_ADDR;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = small_buf;
  msgs[1].length    = 2;

  ret = I2C_TRANSFER(i2c, msgs, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Small read failed: %d\n", ret);
      goto errout;
    }

  if (small_buf[0] != 0x01 || small_buf[1] != 0x02)
    {
      syslog(LOG_ERR,
             "ERROR: Small read mismatch: "
             "got 0x%02x 0x%02x, expected 0x01 0x02\n",
             small_buf[0], small_buf[1]);
      ret = -EIO;
      goto errout;
    }

  syslog(LOG_INFO, "Small read PASSED (0x%02x 0x%02x)\n",
         small_buf[0], small_buf[1]);

  /* Step 5: DMA read without RELOAD (100 bytes, <= 255)
   * Tests dma_read() single-chunk path (no RELOAD/TCR needed).
   */

  syslog(LOG_INFO, "DMA read no-RELOAD (100 bytes) ...\n");

  addr_buf[0] = 0x00;

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 1;

  ret = I2C_TRANSFER(i2c, msgs, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Set address pointer failed: %d\n", ret);
      goto errout;
    }

  memset(read_buf, 0, AT24_TEST_SIZE);

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = I2C_M_READ;
  msgs[0].buffer    = read_buf;
  msgs[0].length    = 100;

  ret = I2C_TRANSFER(i2c, msgs, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: DMA read 100 bytes failed: %d\n", ret);
      goto errout;
    }

  for (i = 0; i < 100; i++)
    {
      if (read_buf[i] != ((i + 1) & 0xff))
        {
          syslog(LOG_ERR,
                 "ERROR: Mismatch at %d: "
                 "exp 0x%02x got 0x%02x\n",
                 i, (i + 1) & 0xff, read_buf[i]);
          ret = -EIO;
          goto errout;
        }
    }

  syslog(LOG_INFO, "DMA read no-RELOAD PASSED\n");

  /* Step 6: Non-aligned buffer DMA read (bounce buffer test)
   * Allocate buffer+1 and use offset to force cache-line misalignment.
   * This exercises the bounce buffer path in dma_read() when
   * CONFIG_ARMV7M_DCACHE is enabled.
   */

  syslog(LOG_INFO, "Non-aligned buffer DMA read (32 bytes) ...\n");

  unaligned_alloc = kmm_malloc(64);
  if (unaligned_alloc == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to allocate unaligned buffer\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Force misalignment: if already misaligned use as-is,
   * otherwise offset by 1.
   */

  if (((uintptr_t)unaligned_alloc & 0x1f) != 0)
    {
      unaligned_buf = unaligned_alloc;
    }
  else
    {
      unaligned_buf = unaligned_alloc + 1;
    }

  syslog(LOG_INFO, "  alloc=%p use=%p (aligned=%s)\n",
         unaligned_alloc, unaligned_buf,
         ((uintptr_t)unaligned_buf & 0x1f) == 0 ? "yes" : "no");

  addr_buf[0] = 0x00;

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 1;

  memset(unaligned_buf, 0, 32);

  msgs[1].frequency = AT24_I2C_FREQ;
  msgs[1].addr      = AT24_I2C_ADDR;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = unaligned_buf;
  msgs[1].length    = 32;

  ret = I2C_TRANSFER(i2c, msgs, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Non-aligned DMA read failed: %d\n", ret);
      kmm_free(unaligned_alloc);
      goto errout;
    }

  for (i = 0; i < 32; i++)
    {
      if (unaligned_buf[i] != ((i + 1) & 0xff))
        {
          syslog(LOG_ERR,
                 "ERROR: Mismatch at %d: "
                 "exp 0x%02x got 0x%02x\n",
                 i, (i + 1) & 0xff, unaligned_buf[i]);
          kmm_free(unaligned_alloc);
          ret = -EIO;
          goto errout;
        }
    }

  kmm_free(unaligned_alloc);
  syslog(LOG_INFO, "Non-aligned buffer DMA read PASSED\n");

  syslog(LOG_INFO, "=== All AT24 DMA tests PASSED ===\n");
  syslog(LOG_INFO, "  Step 1: Page write %u bytes - OK\n", AT24_TEST_SIZE);
  syslog(LOG_INFO, "  Step 2: Single-msg DMA read %u bytes (RELOAD) - OK\n",
         AT24_TEST_SIZE);
  syslog(LOG_INFO, "  Step 3: Multi-msg DMA read %u bytes (RELOAD) - OK\n",
         AT24_TEST_SIZE);
  syslog(LOG_INFO, "  Step 4: Small read 2 bytes (polling fallback) - OK\n");
  syslog(LOG_INFO, "  Step 5: DMA read 100 bytes (no RELOAD) - OK\n");
  syslog(LOG_INFO, "  Step 6: Non-aligned buffer DMA read (bounce) - OK\n");
  ret = OK;

errout:
  kmm_free(read_buf);
  gd32_i2cbus_uninitialize(i2c);
  return ret;
}
#endif /* CONFIG_GD32H7_I2C_DMA */

#endif /* HAVE_AT24 */
