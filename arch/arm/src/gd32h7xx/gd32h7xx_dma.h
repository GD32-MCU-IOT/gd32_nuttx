/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_dma.h
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

#ifndef __ARCH_ARM_SRC_GD32H7XX_GD32H7XX_DMA_H
#define __ARCH_ARM_SRC_GD32H7XX_GD32H7XX_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"
#include "hardware/gd32h7xx_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA transfer status definitions (using flags from hardware header) */

#define DMA_STATUS_ERROR     DMA_INTF_ERRIF
#define DMA_STATUS_SUCCESS   (DMA_INTF_HTFIF | DMA_INTF_FTFIF)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA_HANDLE provides an opaque reference that can be used to
 * represent a DMA channel.
 */

typedef void *DMA_HANDLE;

/* This is the type of the callback that is used to inform
 * the user of the completion of the DMA.
 *
 * Input Parameters:
 *   handle - Refers to the DMA channel
 *   status - A bit encoded value that provides the
 *            completion status.
 *   arg    - A user-provided value that was provided when
 *            gd32_dma_start() was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle,
              uint16_t status, void *arg);

#ifdef CONFIG_DEBUG_DMA_INFO
struct gd32_dmaregs_s
{
  uint32_t intf0;
  uint32_t intf1;
  uint32_t chctl;
  uint32_t chcnt;
  uint32_t chpaddr;
  uint32_t chm0addr;
  uint32_t chm1addr;
  uint32_t chfctl;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* DMA configuration structure.
 *
 * Fields periph_width, memory_width, priority, direction
 * use direct register bit values (e.g. DMA_PERIPH_WIDTH_8BIT).
 */

struct gd32_dma_config_s
{
  uint32_t periph_addr;     /* Peripheral address */
  uint32_t memory_addr;     /* Memory address */
  uint32_t direction;       /* DMA_PERIPH_TO_MEMORY, etc. */
  uint32_t number;          /* Transfer count */
  uint32_t periph_width;    /* DMA_PERIPH_WIDTH_8BIT, etc. */
  uint32_t memory_width;    /* DMA_MEMORY_WIDTH_8BIT, etc. */
  uint32_t priority;        /* DMA_PRIORITY_xxx */
  bool     periph_inc;      /* Peripheral address increment */
  bool     memory_inc;      /* Memory address increment */
  bool     circular;        /* Circular mode */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gd32_dma_channel_alloc
 *
 * Description:
 *   Allocate a free DMA channel. Returns a DMA handle on success.
 *   The dmamux_reqid is used to configure DMAMUX request routing.
 *
 * Input Parameters:
 *   dmamux_reqid - DMAMUX request ID (DMA_REQUEST_xxx)
 *
 * Returned Value:
 *   Non-NULL DMA handle on success, NULL on failure (no free channel)
 *
 ****************************************************************************/

DMA_HANDLE gd32_dma_channel_alloc(uint8_t dmamux_reqid);

/****************************************************************************
 * Name: gd32_dma_channel_free
 *
 * Description:
 *   Release a DMA channel.
 *
 ****************************************************************************/

void gd32_dma_channel_free(DMA_HANDLE handle);

/****************************************************************************
 * Name: gd32_dma_setup
 *
 * Description:
 *   Configure DMA channel for transfer.
 *
 ****************************************************************************/

void gd32_dma_setup(DMA_HANDLE handle,
                    const struct gd32_dma_config_s *config);

/****************************************************************************
 * Name: gd32_dma_start
 *
 * Description:
 *   Start the DMA transfer.
 *
 ****************************************************************************/

void gd32_dma_start(DMA_HANDLE handle,
                    dma_callback_t callback,
                    void *arg, uint32_t interrupt);

/****************************************************************************
 * Name: gd32_dma_stop
 *
 * Description:
 *   Cancel the DMA. After gd32_dma_stop() is called, the
 *   DMA channel is reset and gd32_dma_setup() must be called
 *   before gd32_dma_start() can be called again.
 *
 ****************************************************************************/

void gd32_dma_stop(DMA_HANDLE handle);

/****************************************************************************
 * Name: gd32_dma_transnum_get
 *
 * Description:
 *   Get the number of remaining data to be transferred.
 *
 ****************************************************************************/

size_t gd32_dma_transnum_get(DMA_HANDLE handle);

/****************************************************************************
 * Name: gd32_dma_residual
 *
 * Description:
 *   Alias for gd32_dma_transnum_get().
 *
 ****************************************************************************/

size_t gd32_dma_residual(DMA_HANDLE handle);

/****************************************************************************
 * Name: gd32_dma_sample
 *
 * Description:
 *   Sample DMA register contents.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_sample(DMA_HANDLE handle,
                     struct gd32_dmaregs_s *regs);
void gd32_dma_dump(DMA_HANDLE handle,
                   const struct gd32_dmaregs_s *regs,
                   const char *msg);
#else
#  define gd32_dma_sample(handle, regs)
#  define gd32_dma_dump(handle, regs, msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_GD32H7XX_GD32H7XX_DMA_H */
