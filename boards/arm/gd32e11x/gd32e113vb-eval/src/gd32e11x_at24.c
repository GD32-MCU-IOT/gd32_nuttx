/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_at24.c
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
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <string.h>

#include "gd32e11x.h"
#include "gd32e113v_eval.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AT24_I2C_ADDR     0x50    /* AT24 EEPROM I2C address */
#define AT24_I2C_FREQ     100000  /* 100kHz I2C frequency */
#define AT24_PAGE_SIZE    8       /* AT24C02 page size: 8 bytes */
#define AT24_TEST_SIZE    256     /* Test size in bytes (max 256 for AT24C02) */

/* Calculate number of pages (round up) */

#define AT24_NUM_PAGES    ((AT24_TEST_SIZE + AT24_PAGE_SIZE - 1) / AT24_PAGE_SIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_at24_wr_test
 *
 * Description:
 *   Write and read the AT24 serial EEPROM test.
 *
 ****************************************************************************/

#ifdef HAVE_AT24

#define BUFFSIZE  16
#define START_BLOCK 0

#if BUFFSIZE>=CONFIG_AT24XX_MTD_BLOCKSIZE
#  define NBLOCK    (BUFFSIZE/CONFIG_AT24XX_MTD_BLOCKSIZE)
#else
#  error "BUFFSIZE should bigger than CONFIG_AT24XX_MTD_BLOCKSIZE"
#endif

const uint8_t write_buf[BUFFSIZE] =
{
  0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
  0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf
};

int gd32_at24_wr_test(int minor)
{
  static struct i2c_master_s *i2c = NULL;
  static struct mtd_dev_s *at24 = NULL;
  static bool initialized = false;
  int ret;
  ssize_t nblocks;
  uint8_t *read_buf;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C port driver */

      finfo("Initialize TWI%d\n", AT24_BUS);
      i2c = gd32_i2cbus_initialize(AT24_BUS);
      if (!i2c)
        {
          ferr("ERROR: Failed to initialize TWI%d\n", AT24_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      finfo("Bind the AT24 EEPROM driver to TWI%d\n", AT24_BUS);
      at24 = at24c_initialize(i2c);
      if (!at24)
        {
          ferr("ERROR: Failed to bind TWI%d to the AT24 EEPROM driver\n",
               AT24_BUS);
          return -ENODEV;
        }

      /* Now we are initializeed */

      initialized = true;
    }

  /* Write start block is START_BLOCK, number of block is 2 */

  nblocks = at24->bwrite(at24, START_BLOCK, NBLOCK, write_buf);
  if (nblocks < NBLOCK)
    {
      ferr("ERROR: AT24 write failed: %zd\n", nblocks);
      gd32_i2cbus_uninitialize(i2c);
      return (int)nblocks;
    }

  read_buf = kmm_malloc(BUFFSIZE);

  /* Read the data write before */

  nblocks = at24->bread(at24, START_BLOCK, NBLOCK, read_buf);
  if (nblocks < NBLOCK)
    {
      ferr("ERROR: AT24 read failed: %zd\n", nblocks);
      gd32_i2cbus_uninitialize(i2c);
      return (int)nblocks;
    }

  if (memcmp(read_buf, write_buf, BUFFSIZE) != 0)
    {
      ferr("ERROR: Read buffer does not match write buffer\n");
      return -1;
    }

  /* Release the I2C instance.
   * REVISIT:  Need an interface to release the AT24 instance too
   */

  ret = gd32_i2cbus_uninitialize(i2c);
  if (ret < 0)
    {
      ferr("ERROR: Failed to release the I2C interface: %d\n", ret);
    }

  syslog(LOG_INFO, "INFO: I2C EEPROM write and read success: \
         %d\n", ret);

  return OK;
}

#ifdef CONFIG_GD32E11X_I2C_DMA
/****************************************************************************
 * Name: gd32_at24_multimsg_dma_test
 *
 * Description:
 *   Test I2C multi-message DMA using i2c_transfer() with 2 messages
 *   (write address + read data) in a single call with Repeated START.
 *
 ****************************************************************************/

int gd32_at24_multimsg_dma_test(int minor)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msgs[2];
  uint8_t addr_buf[1];
  uint8_t read_buf[AT24_TEST_SIZE];
  uint8_t tx_buf[AT24_PAGE_SIZE + 1];  /* addr + page data */
  char line[52];
  int ret;
  int i;
  int page;
  int pos;

  syslog(LOG_INFO, "DMA test: %d bytes (%d pages)\n",
         AT24_TEST_SIZE, AT24_NUM_PAGES);

  /* Get the I2C port driver */

  i2c = gd32_i2cbus_initialize(AT24_BUS);
  if (!i2c)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d\n", AT24_BUS);
      return -ENODEV;
    }

  /* Step 1: Write AT24_TEST_SIZE bytes */

  for (page = 0; page < AT24_NUM_PAGES; page++)
    {
      int bytes_remaining = AT24_TEST_SIZE - page * AT24_PAGE_SIZE;
      int bytes_this_page = (bytes_remaining > AT24_PAGE_SIZE) ?
                            AT24_PAGE_SIZE : bytes_remaining;

      tx_buf[0] = page * AT24_PAGE_SIZE;  /* EEPROM internal address */
      for (i = 1; i <= bytes_this_page; i++)
        {
          tx_buf[i] = (page * AT24_PAGE_SIZE + i) & 0xff;
        }

      msgs[0].frequency = AT24_I2C_FREQ;
      msgs[0].addr      = AT24_I2C_ADDR;
      msgs[0].flags     = 0;
      msgs[0].buffer    = tx_buf;
      msgs[0].length    = bytes_this_page + 1;  /* addr + data */

      ret = I2C_TRANSFER(i2c, msgs, 1);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Write page %d failed: %d\n", page, ret);
          goto errout;
        }

      /* Wait for EEPROM write cycle (5-10ms typical) */

      nxsig_usleep(10000);
    }

  syslog(LOG_INFO, "Write %d bytes done\n", AT24_TEST_SIZE);

  /* Step 2: Test single-message read DMA */

  addr_buf[0] = 0x00;  /* Start address */

  /* Write address pointer */

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 1;

  ret = I2C_TRANSFER(i2c, msgs, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Set address failed: %d\n", ret);
      goto errout;
    }

  /* Single-message read */

  memset(read_buf, 0, sizeof(read_buf));

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = I2C_M_READ;
  msgs[0].buffer    = read_buf;
  msgs[0].length    = AT24_TEST_SIZE;

  ret = I2C_TRANSFER(i2c, msgs, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Single read failed: %d\n", ret);
      goto errout;
    }

  /* Hex dump output (16 bytes per line) */

  syslog(LOG_INFO, "Single read %d bytes:\n", AT24_TEST_SIZE);
  for (i = 0; i < AT24_TEST_SIZE; i += 16)
    {
      pos = 0;
      for (int j = 0; j < 16 && (i + j) < AT24_TEST_SIZE; j++)
        {
          pos += snprintf(&line[pos], sizeof(line) - pos,
                          "%02x ", read_buf[i + j]);
        }

      syslog(LOG_INFO, "  %02x: %s\n", i, line);
    }

  /* Verify data */

  for (i = 0; i < AT24_TEST_SIZE; i++)
    {
      if (read_buf[i] != ((i + 1) & 0xff))
        {
          syslog(LOG_ERR, "ERROR: Mismatch at %d: exp 0x%02x got 0x%02x\n",
                 i, (i + 1) & 0xff, read_buf[i]);
          ret = -EIO;
          goto errout;
        }
    }

  syslog(LOG_INFO, "Single-message read PASSED!\n");

  /* Step 3: Multi-message read (write addr + read data with
   * Repeated START)
   */

  addr_buf[0] = 0x00;

  msgs[0].frequency = AT24_I2C_FREQ;
  msgs[0].addr      = AT24_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = addr_buf;
  msgs[0].length    = 1;

  msgs[1].frequency = AT24_I2C_FREQ;
  msgs[1].addr      = AT24_I2C_ADDR;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = read_buf;
  msgs[1].length    = AT24_TEST_SIZE;

  memset(read_buf, 0, sizeof(read_buf));

  ret = I2C_TRANSFER(i2c, msgs, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Multi-message read failed: %d\n", ret);
      goto errout;
    }

  /* Hex dump output */

  syslog(LOG_INFO, "Multi-msg read %d bytes:\n", AT24_TEST_SIZE);
  for (i = 0; i < AT24_TEST_SIZE; i += 16)
    {
      pos = 0;
      for (int j = 0; j < 16 && (i + j) < AT24_TEST_SIZE; j++)
        {
          pos += snprintf(&line[pos], sizeof(line) - pos,
                          "%02x ", read_buf[i + j]);
        }

      syslog(LOG_INFO, "  %02x: %s\n", i, line);
    }

  /* Verify data */

  for (i = 0; i < AT24_TEST_SIZE; i++)
    {
      if (read_buf[i] != ((i + 1) & 0xff))
        {
          syslog(LOG_ERR, "ERROR: Mismatch at %d: exp 0x%02x got 0x%02x\n",
                 i, (i + 1) & 0xff, read_buf[i]);
          ret = -EIO;
          goto errout;
        }
    }

  syslog(LOG_INFO, "Multi-message read PASSED!\n");
  syslog(LOG_INFO, "=== All DMA tests PASSED ===\n");
  ret = OK;

errout:
  gd32_i2cbus_uninitialize(i2c);
  return ret;
}
#endif /* CONFIG_GD32E11X_I2C_DMA */

#endif /* HAVE_AT24 */
