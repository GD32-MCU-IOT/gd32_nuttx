/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_allocateheap.c
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

#include "mpu.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32E11x contains System SRAM on the chip.
 * The following definitions must be provided to specify the size and
 * location of System SRAM:
 *
 * CONFIG_RAM_END                  :  SRAM end address
 *
 * In addition to internal SRAM, external RAM may also be available through
 * the EXMC. When external RAM is want to be used, then the following
 * definitions should to be provided:
 *
 * CONFIG_GD32E11X_EXMC=y          : Enable the EXMC
 * CONFIG_GD32E11X_EXTERNAL_RAM=y  : Indicates that via the EXMC, external
 *                                    RAM can be used.
 * CONFIG_HEAP2_BASE               : External RAM base address
 * CONFIG_HEAP2_SIZE               : External RAM size
 * CONFIG_MM_REGIONS               : Must be set to the value match to how
 *                                    many RAMs you have used.
 */

#if !defined(CONFIG_GD32E11X_EXMC)
#  undef CONFIG_GD32E11X_EXTERNAL_RAM
#endif

/* The heap is in one contiguous block starting at g_idle_topstack and
 * extending through CONFIG_RAM_END.
 */

#if defined(CONFIG_GD32E11X_GD32E11X)

/* Set the end of system SRAM */

#  if defined(CONFIG_GD32E11X_GD32E113XB)
     /* GD32E113VB: 32KB SRAM at 0x20000000-0x20007fff */
#    define SRAM_END 0x20008000

#  elif defined(CONFIG_GD32E11X_GD32E113X8)
     /* GD32E113X8: 20KB SRAM at 0x20000000-0x20004fff */
#    define SRAM_END 0x20005000
#  else
#    error "Unsupported GD32E11x chip"
#endif

/* There are 2 possible SRAM configuration cases:
 *
 * Case 0.     System SRAM only
 *             CONFIG_MM_REGIONS define as 1
 *             CONFIG_GD32E11X_EXTERNAL_RAM NOT defined
 *
 * Case 1.     System SRAM and EXMC SRAM
 *             CONFIG_MM_REGIONS define as 2
 *             CONFIG_GD32E11X_EXTERNAL_RAM defined
 *
 * Make sure that all definitions are consistent before doing anything else
 */

#  if CONFIG_MM_REGIONS < 1

#    error "There is at least one memory region"
#    define CONFIG_MM_REGIONS 1
#    undef CONFIG_GD32E11X_EXTERNAL_RAM

#  elif CONFIG_MM_REGIONS < 2
/* Only one memory region.  Force Case 0 */

#    warning "EXMC SRAM excluded from the heap"
#    undef CONFIG_GD32E11X_EXTERNAL_RAM

#  elif CONFIG_MM_REGIONS == 2
/* Two memory regions.  Case 1 - System SRAM and EXMC SRAM */

#    ifndef CONFIG_GD32E11X_EXTERNAL_RAM
#      error "CONFIG_MM_REGIONS is 2, CONFIG_GD32E11X_EXTERNAL_RAM should be selected"
#    endif

#  else
#    error "CONFIG_MM_REGIONS > 2 but there are no more regions than SRAM and EXMC RAM"
#    undef CONFIG_MM_REGIONS
#    define CONFIG_MM_REGIONS 2

#  endif /* CONFIG_MM_REGIONS */

#else
#  error "Unsupported GD32 chip"
#endif

/* If EXMC SRAM is going to be used as heap, then verify that the starting
 * address and size of the external SRAM region has been provided in the
 * configuration (as CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE).
 */

#ifdef CONFIG_GD32E11X_EXTERNAL_RAM
#  if !defined(CONFIG_HEAP2_BASE) || !defined(CONFIG_HEAP2_SIZE)
#    error "When use EXMC RAM CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE must be provided"
#    undef CONFIG_GD32E11X_EXTERNAL_RAM
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
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
 *     Kernel .data region       Size determined at link time
 *     Kernel .bss  region       Size determined at link time
 *     Kernel IDLE thread stack  Size determined by
 *                                CONFIG_IDLETHREAD_STACKSIZE
 *     Padding for alignment
 *     User .data region         Size determined at link time
 *     User .bss region          Size determined at link time
 *     Kernel heap               Size determined by
 *                                CONFIG_MM_KERNEL_HEAPSIZE
 *     User heap                 Extends to the end of SRAM
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
  size_t    usize = SRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);

  usize = (1 << log2);
  ubase = SRAM_END - usize;

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
  *heap_size  = SRAM_END - g_idle_topstack;

  /* Colorize the heap for debug */

  up_heap_color(*heap_start, *heap_size);
#endif
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
  size_t    usize = SRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);

  usize = (1 << log2);
  ubase = SRAM_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

/****************************************************************************
 * Name: arm_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
#ifdef CONFIG_GD32E11X_EXTERNAL_RAM
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the EXMC SRAM user heap memory */

  gd32_mpu_uheap((uintptr_t)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

#endif

  /* Colorize the heap for debug */

  up_heap_color((void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

  /* Add the external EXMC SRAM user heap region. */

  kumm_addregion((void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);
#endif
}
#endif
