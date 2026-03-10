/****************************************************************************
 * boards/arm/gd32e11x/gd32e113vb-eval/src/gd32e11x_crc.c
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
#include <syslog.h>
#include <errno.h>

#include "gd32e11x_crc.h"
#include "gd32e113v_eval.h"

#ifdef CONFIG_GD32E11X_CRC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_crc_test
 *
 * Description:
 *   Test the CRC hardware by computing CRC-32 for known test vectors.
 *   The GD32E11x CRC unit uses the Ethernet CRC-32 polynomial (0x4C11DB7).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success or a negative error code on failure
 *
 ****************************************************************************/

int gd32_crc_test(void)
{
  uint32_t crc_result;
  int ret = OK;

  /* Test vector 1: Single word */

  static const uint32_t test1_data[] =
  {
    0x12345678
  };

  /* Test vector 2: Multiple words (typical use case) */

  static const uint32_t test2_data[] =
  {
    0x00000000,
    0x00000001,
    0x00000002,
    0x00000003,
    0x00000004,
    0x00000005,
    0x00000006,
    0x00000007
  };

  /* Test vector 3: All 0xFF pattern */

  static const uint32_t test3_data[] =
  {
    0xffffffff,
    0xffffffff,
    0xffffffff,
    0xffffffff
  };

  syslog(LOG_INFO, "[CRC Test] Initializing CRC hardware...\n");

  /* Initialize CRC hardware */

  gd32_crc_init();

  /* Test 1: Single word CRC */

  syslog(LOG_INFO, "[CRC Test] Test 1: Single word CRC\n");
  gd32_crc_reset();
  crc_result = gd32_crc_calculate_block(test1_data, 1);
  syslog(LOG_INFO, "[CRC Test]   Input: 0x%08lx\n",
         (unsigned long)test1_data[0]);
  syslog(LOG_INFO, "[CRC Test]   CRC32: 0x%08lx\n",
         (unsigned long)crc_result);

  /* Test 2: Multiple words CRC */

  syslog(LOG_INFO, "[CRC Test] Test 2: Multiple words CRC (8 words)\n");
  gd32_crc_reset();
  crc_result = gd32_crc_calculate_block(test2_data,
               sizeof(test2_data) / sizeof(test2_data[0]));
  syslog(LOG_INFO, "[CRC Test]   Input: 0x00000000 to 0x00000007\n");
  syslog(LOG_INFO, "[CRC Test]   CRC32: 0x%08lx\n",
         (unsigned long)crc_result);

  /* Test 3: All 0xFF pattern */

  syslog(LOG_INFO, "[CRC Test] Test 3: All 0xFF pattern CRC\n");
  gd32_crc_reset();
  crc_result = gd32_crc_calculate_block(test3_data,
               sizeof(test3_data) / sizeof(test3_data[0]));
  syslog(LOG_INFO, "[CRC Test]   Input: Four 0xFFFFFFFF words\n");
  syslog(LOG_INFO, "[CRC Test]   CRC32: 0x%08lx\n",
         (unsigned long)crc_result);

  /* Test 4: Incremental CRC calculation */

  syslog(LOG_INFO, "[CRC Test] Test 4: Incremental CRC calculation\n");
  gd32_crc_reset();

  /* Add data word by word */

  gd32_crc_calculate(0x11111111);
  gd32_crc_calculate(0x22222222);
  gd32_crc_calculate(0x33333333);
  crc_result = gd32_crc_get_value();
  syslog(LOG_INFO, "[CRC Test]   Data: 0x11111111, 0x22222222...\n");
  syslog(LOG_INFO, "[CRC Test]   CRC32: 0x%08lx\n",
         (unsigned long)crc_result);

  /* Test 5: Free data register test */

  syslog(LOG_INFO, "[CRC Test] Test 5: Free data register (FDATA) test\n");
  gd32_crc_fdata_write(0xa5);
  uint8_t fdata = gd32_crc_fdata_read();
  syslog(LOG_INFO, "[CRC Test]   Write: 0xa5, Read: 0x%02x\n", fdata);
  if (fdata == 0xa5)
    {
      syslog(LOG_INFO, "[CRC Test]   FDATA test PASSED\n");
    }
  else
    {
      syslog(LOG_ERR, "[CRC Test]   FDATA test FAILED\n");
      ret = -EIO;
    }

  /* Test 6: Reset verification */

  syslog(LOG_INFO, "[CRC Test] Test 6: Reset verification\n");
  gd32_crc_reset();
  crc_result = gd32_crc_get_value();
  syslog(LOG_INFO, "[CRC Test]   After reset, CRC_DATA: 0x%08lx\n",
         (unsigned long)crc_result);
  if (crc_result == 0xffffffff)
    {
      syslog(LOG_INFO, "[CRC Test]   Reset test PASSED\n");
    }
  else
    {
      syslog(LOG_ERR, "[CRC Test]   Reset FAILED (expected 0xffffffff)\n");
      ret = -EIO;
    }

  syslog(LOG_INFO, "[CRC Test] All tests completed.\n");

  return ret;
}

#endif /* CONFIG_GD32E11X_CRC */
