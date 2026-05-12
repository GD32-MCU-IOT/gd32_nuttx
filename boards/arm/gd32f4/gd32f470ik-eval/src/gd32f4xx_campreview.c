/****************************************************************************
 * boards/arm/gd32f4/gd32f470ik-eval/src/gd32f4xx_campreview.c
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
 * Description:
 *   Board-level camera preview task for GD32F470IK-EVAL.
 *
 *   Opens /dev/video0 (OV2640, 320x240 RGB565) and /dev/fb0
 *   (TLI LCD, 480x272 RGB565).  Each captured frame is copied
 *   row-by-row into the framebuffer centred on the LCD:
 *
 *       X offset = (480 - 320) / 2 = 80 pixels
 *       Y offset = (272 - 240) / 2 = 16 pixels
 *
 *   This avoids the NX graphics server and fixes the 6-split artefact
 *   that occurs when the source stride (320*2 = 640 B) is confused with
 *   the LCD stride (480*2 = 960 B) by the upstream NX nx_bitmap() call.
 *
 *   Started from gd32_bringup() after both /dev/fb0 and /dev/video0
 *   have been registered.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/video.h>

#include <arch/board/board.h>

#ifdef CONFIG_GD32F4_CAMPREVIEW

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OV2640 capture size (QVGA) */

#define PREVIEW_W          320
#define PREVIEW_H          240
#define PREVIEW_BPP        2                          /* RGB565 */
#define PREVIEW_FRAME_SIZE (PREVIEW_W * PREVIEW_H * PREVIEW_BPP)

/* LCD (TLI) full-screen size */

#define LCD_W              BOARD_TLI_WIDTH            /* 480 */
#define LCD_H              BOARD_TLI_HEIGHT           /* 272 */
#define LCD_STRIDE         (LCD_W * PREVIEW_BPP)      /* 960 bytes/line */

/* Centre the 320x240 image on the 480x272 LCD */

#define FB_XOFF            ((LCD_W - PREVIEW_W) / 2) /* 80 px  */
#define FB_YOFF            ((LCD_H - PREVIEW_H) / 2) /* 16 px  */

/* Byte offset of the top-left pixel of the centred image in the FB */

#define FB_START_BYTES     (FB_YOFF * LCD_STRIDE + FB_XOFF * PREVIEW_BPP)

/* Number of V4L2 userptr buffers to cycle */

#define CAMPREVIEW_BUFNUM  3

/* Device paths */

#define CAMPREVIEW_VIDEODEV  "/dev/video0"
#define CAMPREVIEW_FBDEV     "/dev/fb0"

/* Retry delay if open() fails (device not yet ready) */

#define CAMPREVIEW_OPEN_DELAY_MS  100

/* Preview run duration in seconds (0 = run forever).
 * Provide a fallback so the file compiles cleanly even when the Kconfig
 * symbol has not yet been regenerated into nuttx/config.h.
 */

#ifdef CONFIG_GD32F4_CAMPREVIEW_DURATION
#  define CAMPREVIEW_RUN_SECONDS  CONFIG_GD32F4_CAMPREVIEW_DURATION
#else
#  define CAMPREVIEW_RUN_SECONDS  10   /* default: 10 s */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct campreview_buf_s
{
  FAR void    *start;
  uint32_t     length;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: campreview_open_wait
 *
 * Description:
 *   Open a device node, retrying until it appears (up to ~5 s).
 ****************************************************************************/

static int campreview_open_wait(FAR const char *path, int flags)
{
  int fd;
  int tries = 50; /* 50 × 100 ms = 5 s */

  while (tries-- > 0)
    {
      fd = open(path, flags);
      if (fd >= 0)
        {
          return fd;
        }

      up_mdelay(CAMPREVIEW_OPEN_DELAY_MS);
    }

  syslog(LOG_ERR, "campreview: cannot open %s: %d\n", path, errno);
  return -1;
}

/****************************************************************************
 * Name: campreview_v4l2_setup
 *
 * Description:
 *   Configure V4L2 format, allocate userptr buffers, enqueue them and
 *   start streaming.
 *
 * Returned Value:
 *   0 on success; -1 on failure.
 ****************************************************************************/

static int campreview_v4l2_setup(int v_fd,
                                 FAR struct campreview_buf_s *bufs,
                                 int nbuf)
{
  struct v4l2_format        fmt;
  struct v4l2_requestbuffers req;
  struct v4l2_buffer         buf;
  enum v4l2_buf_type         type;
  int                        i;
  int                        ret;

  /* Set capture format: 320x240 RGB565 */

  memset(&fmt, 0, sizeof(fmt));
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = PREVIEW_W;
  fmt.fmt.pix.height      = PREVIEW_H;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;

  ret = ioctl(v_fd, VIDIOC_S_FMT, (uintptr_t)&fmt);
  if (ret < 0)
    {
      syslog(LOG_ERR, "campreview: VIDIOC_S_FMT failed: %d\n", errno);
      return -1;
    }

  /* Request userptr buffers */

  memset(&req, 0, sizeof(req));
  req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = nbuf;
  req.mode   = V4L2_BUF_MODE_RING;

  ret = ioctl(v_fd, VIDIOC_REQBUFS, (uintptr_t)&req);
  if (ret < 0)
    {
      syslog(LOG_ERR, "campreview: VIDIOC_REQBUFS failed: %d\n", errno);
      return -1;
    }

  /* Allocate and enqueue each buffer */

  for (i = 0; i < nbuf; i++)
    {
      bufs[i].length = PREVIEW_FRAME_SIZE;
      bufs[i].start  = memalign(32, PREVIEW_FRAME_SIZE);
      if (bufs[i].start == NULL)
        {
          syslog(LOG_ERR, "campreview: OOM for buffer %d\n", i);

          while (i--)
            {
              free(bufs[i].start);
              bufs[i].start = NULL;
            }

          return -1;
        }

      memset(&buf, 0, sizeof(buf));
      buf.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory    = V4L2_MEMORY_USERPTR;
      buf.index     = i;
      buf.m.userptr = (uintptr_t)bufs[i].start;
      buf.length    = bufs[i].length;

      ret = ioctl(v_fd, VIDIOC_QBUF, (uintptr_t)&buf);
      if (ret < 0)
        {
          syslog(LOG_ERR, "campreview: VIDIOC_QBUF %d failed: %d\n",
                 i, errno);
          return -1;
        }
    }

  /* Start streaming */

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ret = ioctl(v_fd, VIDIOC_STREAMON, (uintptr_t)&type);
  if (ret < 0)
    {
      syslog(LOG_ERR, "campreview: VIDIOC_STREAMON failed: %d\n", errno);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: campreview_thread
 *
 * Description:
 *   Kernel thread entry point.  Opens video and framebuffer devices,
 *   then loops: dequeue frame → blit centred into FB → requeue frame.
 ****************************************************************************/

static int campreview_thread(int argc, FAR char *argv[])
{
  struct campreview_buf_s    bufs[CAMPREVIEW_BUFNUM];
  struct fb_fix_screeninfo   finfo;
  struct v4l2_buffer         vbuf;
#if CAMPREVIEW_RUN_SECONDS > 0
  struct timeval             start;
  struct timeval             now;
  struct timeval             delta;
#endif
  FAR const uint8_t         *src;
  FAR uint8_t               *dst;
  FAR uint8_t               *fbmem;
  enum v4l2_buf_type         type;
  int                        v_fd;
  int                        fb_fd;
  int                        y;
  int                        ret;

  memset(bufs, 0, sizeof(bufs));

  /* ------------------------------------------------------------------
   * Open framebuffer
   * ------------------------------------------------------------------
   */

  fb_fd = campreview_open_wait(CAMPREVIEW_FBDEV, O_RDWR);
  if (fb_fd < 0)
    {
      return EXIT_FAILURE;
    }

  ret = ioctl(fb_fd, FBIOGET_FSCREENINFO, (uintptr_t)&finfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "campreview: FBIOGET_FSCREENINFO failed: %d\n", errno);
      close(fb_fd);
      return EXIT_FAILURE;
    }

  fbmem = mmap(NULL, finfo.smem_len,
               PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
  if (fbmem == MAP_FAILED)
    {
      syslog(LOG_ERR, "campreview: mmap fb failed: %d\n", errno);
      close(fb_fd);
      return EXIT_FAILURE;
    }

  /* Clear the framebuffer to black */

  memset(fbmem, 0, finfo.smem_len);

  /* ------------------------------------------------------------------
   * Open video capture device
   * ------------------------------------------------------------------
   */

  v_fd = campreview_open_wait(CAMPREVIEW_VIDEODEV, O_RDWR);
  if (v_fd < 0)
    {
      munmap(fbmem, finfo.smem_len);
      close(fb_fd);
      return EXIT_FAILURE;
    }

  /* Configure V4L2 and start streaming */

  ret = campreview_v4l2_setup(v_fd, bufs, CAMPREVIEW_BUFNUM);
  if (ret < 0)
    {
      close(v_fd);
      munmap(fbmem, finfo.smem_len);
      close(fb_fd);
      return EXIT_FAILURE;
    }

#if CAMPREVIEW_RUN_SECONDS > 0
  syslog(LOG_INFO,
         "campreview: streaming %dx%d -> LCD centre (%d,%d), "
         "will stop after %d s\n",
         PREVIEW_W, PREVIEW_H, FB_XOFF, FB_YOFF, CAMPREVIEW_RUN_SECONDS);
#else
  syslog(LOG_INFO,
         "campreview: streaming %dx%d -> LCD centre (%d,%d),"
         " running forever\n",
         PREVIEW_W, PREVIEW_H, FB_XOFF, FB_YOFF);
#endif

#if CAMPREVIEW_RUN_SECONDS > 0
  gettimeofday(&start, NULL);
#endif

  /* ------------------------------------------------------------------
   * Main capture loop
   * ------------------------------------------------------------------
   */

  while (1)
    {
      /* Dequeue a completed frame */

      memset(&vbuf, 0, sizeof(vbuf));
      vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      vbuf.memory = V4L2_MEMORY_USERPTR;

      ret = ioctl(v_fd, VIDIOC_DQBUF, (uintptr_t)&vbuf);
      if (ret < 0)
        {
          syslog(LOG_ERR, "campreview: VIDIOC_DQBUF failed: %d\n", errno);
          break;
        }

      /* Blit frame into FB centred position, row by row.
       *
       * Source stride = PREVIEW_W * PREVIEW_BPP  (640 bytes)
       * Dest   stride = LCD_STRIDE               (960 bytes)
       *
       * This is the fix for the 6-split problem: we never use the LCD
       * width as the source stride.
       */

      src = (FAR const uint8_t *)vbuf.m.userptr;
      dst = fbmem + FB_START_BYTES;

      for (y = 0; y < PREVIEW_H; y++)
        {
          memcpy(dst, src, PREVIEW_W * PREVIEW_BPP);
          src += PREVIEW_W * PREVIEW_BPP;
          dst += LCD_STRIDE;
        }

      /* Requeue the buffer for next capture */

      ret = ioctl(v_fd, VIDIOC_QBUF, (uintptr_t)&vbuf);
      if (ret < 0)
        {
          syslog(LOG_ERR, "campreview: VIDIOC_QBUF failed: %d\n", errno);
          break;
        }

      /* Check elapsed time and exit when duration is reached */

#if CAMPREVIEW_RUN_SECONDS > 0
      gettimeofday(&now, NULL);
      timersub(&now, &start, &delta);
      if (delta.tv_sec >= CAMPREVIEW_RUN_SECONDS)
        {
          syslog(LOG_INFO, "campreview: %d s elapsed, stopping\n",
                 CAMPREVIEW_RUN_SECONDS);
          break;
        }
#endif
    }

  /* Clean up on exit (timeout or error) */

  (void)ret; /* suppress unused-variable warning */

  syslog(LOG_INFO, "campreview: cleaning up\n");
  memset(fbmem, 0, finfo.smem_len);

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(v_fd, VIDIOC_STREAMOFF, (uintptr_t)&type);

  for (int i = 0; i < CAMPREVIEW_BUFNUM; i++)
    {
      free(bufs[i].start);
    }

  close(v_fd);
  munmap(fbmem, finfo.smem_len);
  close(fb_fd);

  syslog(LOG_INFO, "campreview: done\n");

  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_campreview_start
 *
 * Description:
 *   Create the camera-preview kernel thread.  Must be called after both
 *   fb_register() and gd32_dci_setup() have succeeded in gd32_bringup().
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 ****************************************************************************/

int gd32_campreview_start(void)
{
  int pid;

  pid = kthread_create("campreview",
                       CONFIG_GD32F4_CAMPREVIEW_PRIORITY,
                       CONFIG_GD32F4_CAMPREVIEW_STACKSIZE,
                       campreview_thread,
                       NULL);
  if (pid < 0)
    {
      syslog(LOG_ERR, "campreview: kthread_create failed: %d\n", pid);
      return pid;
    }

  syslog(LOG_INFO, "campreview: thread started (pid=%d)\n", pid);
  return OK;
}

#endif /* CONFIG_GD32F4_CAMPREVIEW */
