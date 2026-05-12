/****************************************************************************
 * boards/arm/gd32f4/gd32f470ik-eval/src/gd32f4xx_dci.c
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

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/video.h>

#include <arch/board/board.h>

#include "gd32f4xx_ov2640_init_table.h"

#include "gd32f4xx.h"
#include "gd32f4xx_dci.h"
#include "gd32f470i_eval.h"

#include "hardware/gd32f4xx_i2c.h"

#ifdef CONFIG_GD32F4_DCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_OV2640_FREQUENCY
#  define CONFIG_OV2640_FREQUENCY 100000
#endif

#define GD32_OV2640_NENTRIES(a)   (sizeof(a) / sizeof((a)[0]))

#define GD32_OV2640_REG_BANK_SEL  0xff
#define GD32_OV2640_REG_RESET     0x12
#define GD32_OV2640_REG_OUTW      0x5a
#define GD32_OV2640_REG_OUTH      0x5b
#define GD32_OV2640_REG_OUTSIZE   0x5c
#define GD32_OV2640_REG_CTRL0     0xe0

#define GD32_OV2640_MIDH_EXPECTED  0x7f
#define GD32_OV2640_MIDL_EXPECTED  0xa2
#define GD32_OV2640_PID_EXPECTED   0x26
#define GD32_OV2640_VER_EXPECTED   0x42

#define GD32_OV2640_SIZE_WIDTH    320
#define GD32_OV2640_SIZE_HEIGHT   240

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ov2640_sensor_s
{
  struct imgsensor_s sensor;
  FAR struct i2c_master_s *i2c;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool ov2640_is_available(FAR struct imgsensor_s *sensor);
static int  ov2640_sensor_init(FAR struct imgsensor_s *sensor);
static int  ov2640_sensor_uninit(FAR struct imgsensor_s *sensor);
static const char *ov2640_get_driver_name(FAR struct imgsensor_s *sensor);
static int  ov2640_validate_frame_setting(FAR struct imgsensor_s *sensor,
              imgsensor_stream_type_t type,
              uint8_t nr_datafmts,
              FAR imgsensor_format_t *datafmts,
              FAR imgsensor_interval_t *interval);
static int  ov2640_start_capture(FAR struct imgsensor_s *sensor,
              imgsensor_stream_type_t type,
              uint8_t nr_datafmts,
              FAR imgsensor_format_t *datafmts,
              FAR imgsensor_interval_t *interval);
static int  ov2640_stop_capture(FAR struct imgsensor_s *sensor,
              imgsensor_stream_type_t type);
static int  gd32_ov2640_initialize(FAR struct ov2640_sensor_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct v4l2_fmtdesc g_ov2640_fmtdescs[] =
{
  {
    .index       = 0,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = 0,
    .description = "YUYV 4:2:2",
    .pixelformat = V4L2_PIX_FMT_YUYV,
  },
  {
    .index       = 1,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = V4L2_FMT_FLAG_COMPRESSED,
    .description = "JPEG",
    .pixelformat = V4L2_PIX_FMT_JPEG,
  },
  {
    .index       = 2,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = 0,
    .description = "RGB565",
    .pixelformat = V4L2_PIX_FMT_RGB565,
  },
};

static const struct v4l2_frmsizeenum g_ov2640_frmsizes[] =
{
  {
    .index        = 0,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width  = 160,
        .height = 120
      },  /* QQVGA */
  },
  {
    .index        = 1,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width  = 320,
        .height = 240
      },  /* QVGA */
  },
  {
    .index        = 2,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width  = 640,
        .height = 480
      },  /* VGA */
  },
  {
    .index        = 3,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width  = 800,
        .height = 600
      },  /* SVGA */
  },
  {
    .index        = 4,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width  = 1024,
        .height = 768
      },  /* XGA */
  },
  {
    .index        = 5,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width  = 1280,
        .height = 1024
      }, /* SXGA */
  },
  {
    .index        = 6,
    .type         = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete     =
      {
        .width  = 1600,
        .height = 1200
      }, /* UXGA */
  },
};

static const struct imgsensor_ops_s g_ov2640_sensor_ops =
{
  .is_available           = ov2640_is_available,
  .init                   = ov2640_sensor_init,
  .uninit                 = ov2640_sensor_uninit,
  .get_driver_name        = ov2640_get_driver_name,
  .validate_frame_setting = ov2640_validate_frame_setting,
  .start_capture          = ov2640_start_capture,
  .stop_capture           = ov2640_stop_capture,
  .get_frame_interval     = NULL,
  .get_supported_value    = NULL,
  .get_value              = NULL,
  .set_value              = NULL,
};

static struct ov2640_sensor_s g_ov2640_priv;

#define OV2640_FMTDESCS_NUM  (sizeof(g_ov2640_fmtdescs) / \
                              sizeof(g_ov2640_fmtdescs[0]))
#define OV2640_FRMSIZES_NUM  (sizeof(g_ov2640_frmsizes) / \
                              sizeof(g_ov2640_frmsizes[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_is_available
 ****************************************************************************/

static bool ov2640_is_available(FAR struct imgsensor_s *sensor)
{
  return true;
}

/****************************************************************************
 * Name: ov2640_sensor_init
 ****************************************************************************/

static int ov2640_sensor_init(FAR struct imgsensor_s *sensor)
{
  FAR struct ov2640_sensor_s *priv =
    (FAR struct ov2640_sensor_s *)sensor;

  return gd32_ov2640_initialize(priv);
}

/****************************************************************************
 * Name: ov2640_sensor_uninit
 ****************************************************************************/

static int ov2640_sensor_uninit(FAR struct imgsensor_s *sensor)
{
  FAR struct ov2640_sensor_s *priv =
    (FAR struct ov2640_sensor_s *)sensor;

  priv->initialized = false;
  return OK;
}

/****************************************************************************
 * Name: ov2640_get_driver_name
 ****************************************************************************/

static const char *ov2640_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "OV2640";
}

/****************************************************************************
 * Name: ov2640_validate_frame_setting
 ****************************************************************************/

static int ov2640_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval)
{
  /* Accept all valid formats - actual validation done by imgdata side */

  return OK;
}

/****************************************************************************
 * Name: ov2640_start_capture
 ****************************************************************************/

static int ov2640_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval)
{
  /* OV2640 is always streaming after initialization.
   * The DCI hardware controls capture start/stop.
   */

  return OK;
}

/****************************************************************************
 * Name: ov2640_stop_capture
 ****************************************************************************/

static int ov2640_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  return OK;
}

/****************************************************************************
 * SCCB (Serial Camera Control Bus) low-level implementation
 *
 * The OV2640 uses SCCB protocol which differs from standard I2C:
 *   - Write: START + SlaveAddr(W) + RegAddr + Data + STOP
 *   - Read:  START + SlaveAddr(W) + RegAddr + STOP +
 *            START + SlaveAddr(R) + Data + NACK + STOP
 *
 * The NuttX I2C framework cannot properly generate this sequence on the
 * GD32F4 I2C peripheral. We therefore directly access the I2C0 registers
 * following the exact same procedure as the vendor sccb.c reference.
 ****************************************************************************/

#define SCCB_I2C_BASE         GD32_I2C0
#define SCCB_WRITE_ADDR       (BOARD_OV2640_I2C_ADDR << 1)        /* 0x60 */
#define SCCB_READ_ADDR        ((BOARD_OV2640_I2C_ADDR << 1) | 1)  /* 0x61 */
#define SCCB_TIMEOUT          2000000

static inline uint16_t sccb_getreg(uint8_t offset)
{
  return getreg16(SCCB_I2C_BASE + offset);
}

static inline void sccb_putreg(uint8_t offset, uint16_t value)
{
  putreg16(value, SCCB_I2C_BASE + offset);
}

static inline void sccb_modifyreg(uint8_t offset,
                                  uint16_t clearbits, uint16_t setbits)
{
  modifyreg16(SCCB_I2C_BASE + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: gd32_ov2640_putreg
 *
 * Description:
 *   Write one byte to an OV2640 register via SCCB.
 *   Protocol: S + 0x60 + reg + data + P
 ****************************************************************************/

static int gd32_ov2640_putreg(FAR struct i2c_master_s *i2c,
                              uint8_t regaddr, uint8_t regval)
{
  volatile uint32_t timeout;

  (void)i2c; /* We bypass the NuttX I2C framework */

  /* Wait until I2C bus is idle */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT1_OFFSET) & (1 << 1)) != 0)  /* BUSY */
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB WR: BUSY timeout reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Generate START */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_START);

  /* Wait for SBSEND (Start Bit Sent) */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_SBSEND) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB WR: SBSEND timeout reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Send slave write address */

  sccb_putreg(GD32_I2C_DATA_OFFSET, SCCB_WRITE_ADDR);

  /* Wait for ADDSEND (Address Sent + ACK) */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_ADDSEND) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB WR: ADDSEND timeout reg=0x%02x "
                 "STAT0=0x%04x\n", regaddr,
                 sccb_getreg(GD32_I2C_STAT0_OFFSET));
          return -ETIMEDOUT;
        }
    }

  /* Clear ADDSEND by reading STAT0 then STAT1 */

  (void)sccb_getreg(GD32_I2C_STAT0_OFFSET);
  (void)sccb_getreg(GD32_I2C_STAT1_OFFSET);

  /* Wait for transmit buffer empty */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_TBE) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB WR: transmit buffer empty timeout"
                 " reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Send register address */

  sccb_putreg(GD32_I2C_DATA_OFFSET, regaddr);

  /* Wait for BTC (Byte Transfer Complete) */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_BTC) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB WR: byte transfer complete timeout"
                 " reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Send register value */

  sccb_putreg(GD32_I2C_DATA_OFFSET, regval);

  /* Wait for transmit buffer empty */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_TBE) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB WR: transmit buffer empty timeout"
                 " reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Generate STOP */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_STOP);

  return OK;
}

/****************************************************************************
 * Name: gd32_ov2640_getreg
 *
 * Description:
 *   Read one byte from an OV2640 register via SCCB.
 *   Protocol: S + 0x60 + reg + P  then  S + 0x61 + [data] + NAK + P
 ****************************************************************************/

static int gd32_ov2640_getreg(FAR struct i2c_master_s *i2c,
                              uint8_t regaddr, FAR uint8_t *regval)
{
  volatile int wait_i;
  volatile uint32_t timeout;

  (void)i2c; /* We bypass the NuttX I2C framework */

  /* ---- Phase 1: Write register address (slave addr = 0x60) ---- */

  /* Clear AF (Acknowledge Failure) flag if set from previous transaction */

  sccb_putreg(GD32_I2C_STAT0_OFFSET,
              sccb_getreg(GD32_I2C_STAT0_OFFSET) | (1 << 10));

  /* Wait for bus idle before starting */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT1_OFFSET) & (1 << 1)) != 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: BUSY timeout reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Generate START */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_START);

  /* Wait for SBSEND */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_SBSEND) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: SBSEND1 timeout reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Send slave write address (0x60) */

  sccb_putreg(GD32_I2C_DATA_OFFSET, SCCB_WRITE_ADDR);

  /* Wait for ADDSEND */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_ADDSEND) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: ADDSEND1 timeout reg=0x%02x "
                 "STAT0=0x%04x\n", regaddr,
                 sccb_getreg(GD32_I2C_STAT0_OFFSET));
          return -ETIMEDOUT;
        }
    }

  /* Clear ADDSEND */

  (void)sccb_getreg(GD32_I2C_STAT0_OFFSET);
  (void)sccb_getreg(GD32_I2C_STAT1_OFFSET);

  /* Wait for Transmit buffer empty timeout */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_TBE) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: transmit buffer empty timeout"
                 " reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Send register address */

  sccb_putreg(GD32_I2C_DATA_OFFSET, regaddr);

  /* Wait for BTC */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_BTC) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: BTC timeout reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Clear AF flag */

  sccb_putreg(GD32_I2C_STAT0_OFFSET,
              sccb_getreg(GD32_I2C_STAT0_OFFSET) | (1 << 10));

  /* Generate STOP (end of write phase) */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_STOP);

  /* Small delay to allow STOP condition to complete on the bus */

  for (wait_i = 0; wait_i < 100; wait_i++);

  /* ---- Phase 2: Read data (slave addr = 0x61) ---- */

  /* Generate START */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_START);

  /* Wait for SBSEND */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_SBSEND) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: SBSEND2 timeout reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Send slave read address (0x61) */

  sccb_putreg(GD32_I2C_DATA_OFFSET, SCCB_READ_ADDR);

  /* Wait for ADDSEND */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_ADDSEND) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: ADDSEND2 timeout reg=0x%02x "
                 "STAT0=0x%04x\n", regaddr,
                 sccb_getreg(GD32_I2C_STAT0_OFFSET));
          return -ETIMEDOUT;
        }
    }

  /* Clear ADDSEND */

  (void)sccb_getreg(GD32_I2C_STAT0_OFFSET);
  (void)sccb_getreg(GD32_I2C_STAT1_OFFSET);

  /* Disable ACK (NACK after the single byte read) */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, I2C_CTL0_ACKEN, 0);

  /* Wait for RBNE (Receive Buffer Not Empty) */

  timeout = SCCB_TIMEOUT;
  while ((sccb_getreg(GD32_I2C_STAT0_OFFSET) & I2C_STAT0_RBNE) == 0)
    {
      if (--timeout == 0)
        {
          syslog(LOG_ERR, "SCCB RD: RBNE timeout reg=0x%02x\n", regaddr);
          return -ETIMEDOUT;
        }
    }

  /* Generate STOP */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_STOP);

  /* Read the data byte */

  *regval = (uint8_t)(sccb_getreg(GD32_I2C_DATA_OFFSET) & 0xff);

  /* Clear AF flag */

  sccb_putreg(GD32_I2C_STAT0_OFFSET,
              sccb_getreg(GD32_I2C_STAT0_OFFSET) | (1 << 10));

  /* Re-enable ACK for next transaction */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_ACKEN);

  return OK;
}

/****************************************************************************
 * Name: gd32_ov2640_putreglist
 ****************************************************************************/

static int gd32_ov2640_putreglist(FAR struct i2c_master_s *i2c,
                                  FAR const char reglist[][2],
                                  size_t nentries)
{
  size_t index;
  int ret;

  for (index = 0; index < nentries; index++)
    {
      ret = gd32_ov2640_putreg(i2c,
                               (uint8_t)reglist[index][0],
                               (uint8_t)reglist[index][1]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_ov2640_outsize_set
 ****************************************************************************/

static int gd32_ov2640_outsize_set(FAR struct i2c_master_s *i2c,
                                   uint16_t width, uint16_t height)
{
  uint16_t outw;
  uint16_t outh;
  uint8_t temp;
  int ret;

  if ((width % 4) != 0 || (height % 4) != 0)
    {
      return -EINVAL;
    }

  outw = width / 4;
  outh = height / 4;

  ret = gd32_ov2640_putreg(i2c, GD32_OV2640_REG_BANK_SEL, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_ov2640_putreg(i2c, GD32_OV2640_REG_CTRL0, 0x04);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_ov2640_putreg(i2c, GD32_OV2640_REG_OUTW, outw & 0xff);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_ov2640_putreg(i2c, GD32_OV2640_REG_OUTH, outh & 0xff);
  if (ret < 0)
    {
      return ret;
    }

  temp  = (outw >> 8) & 0x03;
  temp |= (outh >> 6) & 0x04;

  ret = gd32_ov2640_putreg(i2c, GD32_OV2640_REG_OUTSIZE, temp);
  if (ret < 0)
    {
      return ret;
    }

  return gd32_ov2640_putreg(i2c, GD32_OV2640_REG_CTRL0, 0x00);
}

/****************************************************************************
 * Name: gd32_ov2640_verify_id
 ****************************************************************************/

static int gd32_ov2640_verify_id(FAR struct i2c_master_s *i2c,
                                 FAR const char *stage)
{
  uint8_t midh;
  uint8_t midl;
  uint8_t ver;
  uint8_t pid;
  int ret;

  ret = gd32_ov2640_putreg(i2c, GD32_OV2640_REG_BANK_SEL, 0x01);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_ov2640_getreg(i2c, 0x1c, &midh);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_ov2640_getreg(i2c, 0x1d, &midl);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_ov2640_getreg(i2c, 0x0b, &ver);
  if (ret < 0)
    {
      return ret;
    }

  ret = gd32_ov2640_getreg(i2c, 0x0a, &pid);
  if (ret < 0)
    {
      return ret;
    }

  syslog(LOG_INFO, "OV2640 ID[%s] MID=%02x%02x VER=%02x PID=%02x\n",
         stage, midh, midl, ver, pid);

  if (midh != GD32_OV2640_MIDH_EXPECTED ||
      midl != GD32_OV2640_MIDL_EXPECTED ||
      ver  != GD32_OV2640_VER_EXPECTED  ||
      pid  != GD32_OV2640_PID_EXPECTED)
    {
      syslog(LOG_ERR,
             "ERROR: OV2640 ID mismatch at %s, "
             "expect MID=%02x%02x VER=%02x PID=%02x\n",
             stage,
             GD32_OV2640_MIDH_EXPECTED,
             GD32_OV2640_MIDL_EXPECTED,
             GD32_OV2640_VER_EXPECTED,
             GD32_OV2640_PID_EXPECTED);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_ov2640_initialize
 ****************************************************************************/

static int gd32_ov2640_initialize(FAR struct ov2640_sensor_s *priv)
{
  int ret;

  if (priv->initialized)
    {
      return OK;
    }

  syslog(LOG_INFO, "OV2640: Sending software reset...\n");

  ret = gd32_ov2640_putreg(priv->i2c, GD32_OV2640_REG_BANK_SEL, 0x01);
  if (ret < 0)
    {
      syslog(LOG_ERR, "OV2640: BANK_SEL write failed: %d\n", ret);
      return ret;
    }

  ret = gd32_ov2640_putreg(priv->i2c, GD32_OV2640_REG_RESET, 0x80);
  if (ret < 0)
    {
      syslog(LOG_ERR, "OV2640: soft reset write failed: %d\n", ret);
      return ret;
    }

  up_mdelay(50);

  ret = gd32_ov2640_putreglist(priv->i2c, gd32_ov2640_svga_init_reg_tbl,
                               GD32_OV2640_NENTRIES(
                                 gd32_ov2640_svga_init_reg_tbl));
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(100);

  ret = gd32_ov2640_putreglist(priv->i2c, gd32_ov2640_rgb565_reg_tbl,
                               GD32_OV2640_NENTRIES(
                                 gd32_ov2640_rgb565_reg_tbl));
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(100);

  ret = gd32_ov2640_outsize_set(priv->i2c,
                                GD32_OV2640_SIZE_WIDTH,
                                GD32_OV2640_SIZE_HEIGHT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "OV2640: outsize_set failed: %d\n", ret);
      return ret;
    }

  up_mdelay(10);

  /* Read ID after full initialization (matches vendor demo order) */

  ret = gd32_ov2640_verify_id(priv->i2c, "after-init");
  if (ret < 0)
    {
      /* Dump I2C register state for debugging */

      syslog(LOG_ERR, "OV2640: I2C0 CTL0=0x%04x CTL1=0x%04x "
             "STAT0=0x%04x STAT1=0x%04x\n",
             sccb_getreg(GD32_I2C_CTL0_OFFSET),
             sccb_getreg(GD32_I2C_CTL1_OFFSET),
             sccb_getreg(GD32_I2C_STAT0_OFFSET),
             sccb_getreg(GD32_I2C_STAT1_OFFSET));
      return ret;
    }

  priv->initialized = true;
  return OK;
}

/****************************************************************************
 * Name: gd32_ov2640_clock_enable
 *
 * Description:
 *   Provide the OV2640 XCLK from CKOUT0 on PA8, following the vendor demo.
 *
 ****************************************************************************/

static void gd32_ov2640_clock_enable(void)
{
  uint32_t regval;

  gd32_gpio_config(BOARD_OV2640_MCLK_PIN);

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_CKOUT0SEL_MASK | RCU_CFG0_CKOUT0DIV_MASK);
  regval |= BOARD_OV2640_MCLK_SOURCE | BOARD_OV2640_MCLK_DIVIDER;
  putreg32(regval, GD32_RCU_CFG0);

  up_mdelay(10);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dci_setup
 *
 * Description:
 *   Initialize the DCI (Digital Camera Interface) and OV2640 camera sensor
 *   on the GD32F470IK-EVAL board. This function:
 *     1. Enables the OV2640 XCLK
 *     2. Initializes the I2C bus for OV2640 SCCB control
 *     3. Initializes the OV2640 sensor driver
 *     4. Registers the imgsensor interface
 *     5. Gets the DCI imgdata interface and registers it
 *     6. Creates the capture device at /dev/video0
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gd32_dci_setup(void)
{
  struct imgdata_s *imgdata;
  struct i2c_master_s *i2c;
  int ret;

  /* Drive OV2640 XCLK before sensor reset and SCCB accesses. */

  gd32_ov2640_clock_enable();

  /* Initialize the I2C bus for OV2640 SCCB communication.
   * We use gd32_i2cbus_initialize to set up clocks, GPIO, and I2C speed,
   * but then disable all I2C interrupts since our SCCB implementation
   * bypasses the NuttX I2C transfer framework and uses polling.
   */

  i2c = gd32_i2cbus_initialize(BOARD_DCI_I2C_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d for OV2640\n",
             BOARD_DCI_I2C_BUS);
      return -ENODEV;
    }

  /* Disable I2C event/error/buffer interrupts to prevent NuttX ISR from
   * interfering with our direct register-level SCCB operations.
   */

  sccb_modifyreg(GD32_I2C_CTL1_OFFSET, I2C_CTL1_INTS_MASK, 0);

  /* Enable the I2C peripheral and ACK (needed for SCCB) */

  sccb_modifyreg(GD32_I2C_CTL0_OFFSET, 0, I2C_CTL0_ACKEN);

  /* Initialize OV2640 sensor via I2C */

  g_ov2640_priv.i2c         = i2c;
  g_ov2640_priv.initialized = false;

  ret = gd32_ov2640_initialize(&g_ov2640_priv);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize OV2640 with GD32 SCCB flow: %d\n",
             ret);
      gd32_i2cbus_uninitialize(i2c);
      return ret;
    }

  /* Set up the OV2640 imgsensor interface */

  g_ov2640_priv.sensor.ops              = &g_ov2640_sensor_ops;
  g_ov2640_priv.sensor.fmtdescs_num     = OV2640_FMTDESCS_NUM;
  g_ov2640_priv.sensor.fmtdescs         = g_ov2640_fmtdescs;
  g_ov2640_priv.sensor.frmsizes_num     = OV2640_FRMSIZES_NUM;
  g_ov2640_priv.sensor.frmsizes         = g_ov2640_frmsizes;
  g_ov2640_priv.sensor.frmintervals_num = 0;
  g_ov2640_priv.sensor.frmintervals     = NULL;

  /* Register the OV2640 as an imgsensor */

  ret = imgsensor_register(&g_ov2640_priv.sensor);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imgsensor_register failed: %d\n", ret);
      gd32_i2cbus_uninitialize(i2c);
      return ret;
    }

  /* Initialize the DCI driver and get the imgdata interface */

  imgdata = gd32_dci_initialize();
  if (imgdata == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize DCI driver\n");
      gd32_i2cbus_uninitialize(i2c);
      return -ENODEV;
    }

  /* Register the DCI as imgdata backend */

  imgdata_register(imgdata);

  /* Create the V4L2 capture device */

  ret = capture_initialize("/dev/video0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: capture_initialize failed: %d\n", ret);
      gd32_dci_uninitialize();
      gd32_i2cbus_uninitialize(i2c);
      return ret;
    }

  syslog(LOG_INFO, "DCI: /dev/video0 registered successfully\n");
  return OK;
}

#endif /* CONFIG_GD32F4_DCI */
