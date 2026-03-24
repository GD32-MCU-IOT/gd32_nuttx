/****************************************************************************
 * boards/arm/gd32h7xx/gd32h759im-eval/scripts/memory.ld
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

/* The GD32H759IMK6 has 3840KB of FLASH beginning at address 0x08000000 and
 * 1058KB of SRAM. SRAM is split up into multiple blocks:
 *
 * 1)  64KB of ITCM SRAM beginning at address 0x00000000
 * 2) 128KB of DTCM SRAM beginning at address 0x20000000
 * 3) 512KB of AXI SRAM beginning at address 0x24000000
 * 4)  16KB of SRAM0 beginning at address 0x30000000
 * 5)  16KB of SRAM1 beginning at address 0x30004000
 * 6)   4KB of Backup SRAM beginning at address 0x38800000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x00000000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x08000000 address range.
 */

MEMORY
{
  /* 3840KB FLASH */

  kflash (rx)      : ORIGIN = 0x08000000, LENGTH = 128K
  uflash (rx)      : ORIGIN = 0x08020000, LENGTH = 128K
  xflash (rx)      : ORIGIN = 0x08040000, LENGTH = 1792K

  /* 32KB of SRAM01 */

  ksram (rwx)      : ORIGIN = 0x30000000, LENGTH = 16K
  usram (rwx)      : ORIGIN = 0x30004000, LENGTH = 16K


  /* 512Kb of contiguous AXI SRAM */

  sram (rwx)       : ORIGIN = 0x24000000, LENGTH = 512K

  /* DTCM SRAM */

  dtcm1 (rwx)      : ORIGIN = 0x20000000, LENGTH = 64K
  dtcm2 (rwx)      : ORIGIN = 0x20010000, LENGTH = 64K

}
