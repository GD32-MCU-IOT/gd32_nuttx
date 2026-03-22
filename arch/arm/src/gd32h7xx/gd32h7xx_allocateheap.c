/****************************************************************************
 * arch/arm/src/gd32h7xx/gd32h7xx_allocateheap.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"

#include "hardware/gd32h7xx_memorymap.h"

#ifdef CONFIG_ARM_MPU
#  include "mpu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* At startup the kernel will invoke arm_addregion() so that platform code
 * may register available memories for use as part of system heap.
 * The global configuration option CONFIG_MM_REGIONS defines the maximal
 * number of non-contiguous memory ranges that may be registered with the
 * system heap. You must make sure it is large enough to hold all memory
 * regions you intend to use.
 *
 * The following memory types can be used for heap on GD32H7 platform:
 *
 * - DTCM SRAM is a 128KB memory area at 0x20000000. This will be
 *      automatically registered with the system heap in up_allocate_heap.
 *      So, CONFIG_MM_REGIONS must be at least 1 to use DTCM SRAM.
 *
 * - AXI SRAM is a 512KB memory area at 0x24000000. This is always
 *      registered with system heap if CONFIG_MM_REGIONS > 1.
 *      So, add 1 more to CONFIG_MM_REGIONS.
 *
 * - SRAM0+SRAM1 are two contiguous 16KB memory areas at 0x30000000.
 *      These can be added to system heap if CONFIG_MM_REGIONS > 2.
 *      So, add 1 more to CONFIG_MM_REGIONS.
 *
 * - Tightly Coupled Memory (ITCM RAM) at 0x00000000 is instruction-only
 *      and cannot be used for system heap.
 *
 * - DTCM has limitations (DMA transfers to/from DTCM may be restricted).
 *      Define CONFIG_GD32H7_DTCMEXCLUDE to exclude the DTCM from heap and
 *      use AXI SRAM as primary heap instead.
 *
 * - External SDRAM can be connected to the EXMC peripheral.
 *      - CONFIG_HEAP2_BASE: External RAM base address
 *      - CONFIG_HEAP2_SIZE: External RAM size
 *      - +1 to CONFIG_MM_REGIONS
 */

/* Set the start and end of the primary SRAM regions */

#ifndef CONFIG_GD32H7_DTCMEXCLUDE
#  define SRAM_START GD32_DTCMSRAM_BASE
#  define SRAM_END   (SRAM_START + (128 * 1024))
#else
#  define SRAM_START GD32_AXISRAM_BASE
#  define SRAM_END   (SRAM_START + (512 * 1024))
#endif

/* Set the range of AXI SRAM (512KB at 0x24000000) */

#define AXISRAM_START GD32_AXISRAM_BASE
#define AXISRAM_END   (AXISRAM_START + (512 * 1024))

/* Set the range of SRAM0+SRAM1 (32KB total at 0x30000000) */

#define SRAM01_START  GD32_SRAM0_BASE
#define SRAM01_END    (SRAM01_START + (16 + 16) * 1024)

/* DTCM SRAM */

#undef HAVE_DTCM
#define HAVE_DTCM 1

#if !defined(GD32_DTCMSRAM_BASE)
#  undef HAVE_DTCM
#endif

/* DTCM to be excluded from the main heap */

#ifdef CONFIG_GD32H7_DTCMEXCLUDE
#  undef HAVE_DTCM
#endif

/* Check if we will have additional memory regions */

#if !defined(CONFIG_GD32H7_DTCMEXCLUDE) || defined(CONFIG_ARCH_HAVE_HEAP2)
#  define HAVE_MMREGIONS 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_ARM_MPU)
/****************************************************************************
 * Name: gd32_mpu_uheap
 *
 * Description:
 *  Map the user heap region (placeholder for future MPU implementation)
 *
 ****************************************************************************/

static inline void gd32_mpu_uheap(uintptr_t start, size_t size)
{
  /* TODO: Implement MPU configuration for GD32H7 */
}
#else
#  define gd32_mpu_uheap(start, size)
#endif

/****************************************************************************
 * Name: up_heap_color
 *
 * Description:
 *   Set heap memory to a known, non-zero state to checking heap usage.
 *
 ****************************************************************************/

#ifdef CONFIG_HEAP_COLORATION
static inline void up_heap_color(void *start, size_t size)
{
  memset(start, HEAP_COLOR, size);
}
#else
#  define up_heap_color(start,size)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 *   The following memory map is assumed for the flat build:
 *
 *     .data region.  Size determined at link time.
 *     .bss  region  Size determined at link time.
 *     IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Heap.  Extends to the end of SRAM.
 *
 *   The following memory map is assumed for the kernel build:
 *
 *     Kernel .data region.       Size determined at link time.
 *     Kernel .bss region         Size determined at link time.
 *     Kernel IDLE thread stack.  Size determined by
 *                                CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.         Size determined at link time.
 *     User .bss region           Size determined at link time.
 *     Kernel heap.               Size determined by
 *                                CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.                 Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = AXISRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)AXISRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the AXISRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((AXISRAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = AXISRAM_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((void *)ubase, usize);

  /* Allow user-mode access to the user heap memory */

  gd32_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = AXISRAM_END - g_idle_topstack;

  /* Colorize the heap for debug */

  up_heap_color(*heap_start, *heap_size);
#endif

  /* Display memory ranges to help debugging */

  minfo("%uKb of SRAM at %p\n", *heap_size / 1024, *heap_start);
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel-space heap.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = AXISRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)AXISRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the AXISRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((AXISRAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = AXISRAM_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

#if (CONFIG_MM_REGIONS > 1)
/****************************************************************************
 * Name: addregion
 *
 * Description:
 *   Make a range of memory available for allocation from system heap.
 *   If debug is disabled, compiler should optimize out the "desc" strings.
 *
 ****************************************************************************/

static void addregion(uintptr_t start, uint32_t size, const char *desc)
{
  /* Display memory ranges to help debugging */

  minfo("%" PRIu32 "Kb of %s at %p\n", size / 1024, desc, (void *)start);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the heap */

  gd32_mpu_uheap(start, size);
#endif

  /* Colorize the heap for debug */

  up_heap_color((void *)start, size);

  /* Add the user heap region */

  kumm_addregion((void *)start, size);
}

/****************************************************************************
 * Name: arm_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

void arm_addregion(void)
{
  /* At this point there is already one region allocated for heap */

  unsigned mm_regions = 1;

#ifdef SRAM01_START
  /* Add SRAM01 */

  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion(SRAM01_START, SRAM01_END - SRAM01_START, "SRAM0,1");
      mm_regions++;
    }
#endif

#ifdef HAVE_DTCM
  /* Add DTCM SRAM when AXI is used as primary heap */

  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion(GD32_DTCMSRAM_BASE, 128 * 1024, "DTCM");
      mm_regions++;
    }
#endif

  /* Add SRAM0+SRAM1 region (32KB total) */

  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion(SRAM01_START, SRAM01_END - SRAM01_START, "SRAM0,1");
      mm_regions++;
    }

#ifdef CONFIG_ARCH_HAVE_HEAP2
  /* Add external EXMC SRAM or other external memory */

  if (mm_regions < CONFIG_MM_REGIONS)
    {
      addregion(CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE, "HEAP2");
      mm_regions++;
    }
#endif
}
#endif