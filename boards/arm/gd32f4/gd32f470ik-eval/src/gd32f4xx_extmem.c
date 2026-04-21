/****************************************************************************
 * boards/arm/gd32f4/gd32f470ik-eval/src/gd32f4xx_extmem.c
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
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "gd32f4xx.h"
#include "gd32f470i_eval.h"
#include "gd32f4xx_exmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_GD32F4_EXMC
#warning "EXMC is not enabled"
#endif

#define GD32_SDRAM_CLKEN        EXMC_SDRAM_CLOCK_ENABLE | EXMC_SDCMD_DS0 | EXMC_SDCMD_NARF(1)

#define GD32_SDRAM_PALL         EXMC_SDRAM_PRECHARGE_ALL | EXMC_SDCMD_DS0 | EXMC_SDCMD_NARF(1)

#define GD32_SDRAM_REFRESH      EXMC_SDRAM_AUTO_REFRESH | EXMC_SDCMD_DS0 |\
                                EXMC_SDCMD_NARF(8)

#define GD32_SDRAM_MODEREG      EXMC_SDRAM_LOAD_MODE_REGISTER | EXMC_SDCMD_DS0 |\
                                EXMC_SDCMR_MDR_BURST_LENGTH_1 | \
                                EXMC_SDCMR_MDR_BURST_TYPE_SEQUENTIAL |\
                                EXMC_SDCMR_MDR_CAS_LATENCY_3 |\
                                EXMC_SDCMR_MDR_WBL_SINGLE | EXMC_SDCMD_NARF(1) |\
                                EXMC_SDRAM_NORMAL_OPERATION

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIO configurations common to most external memories */

static const uint32_t g_sdram_config[] =
{
  /* 16 data lines
   *
   * GPIO_EXMC_D0, GPIO_EXMC_D1, GPIO_EXMC_D2, GPIO_EXMC_D3,
   * GPIO_EXMC_D4, GPIO_EXMC_D5, GPIO_EXMC_D6, GPIO_EXMC_D7,
   * GPIO_EXMC_D8, GPIO_EXMC_D9, GPIO_EXMC_D10, GPIO_EXMC_D11,
   * GPIO_EXMC_D12, GPIO_EXMC_D13, GPIO_EXMC_D14, GPIO_EXMC_D15,
   *
   * 12 address lines
   *
   * GPIO_EXMC_A0, GPIO_EXMC_A1, GPIO_EXMC_A2, GPIO_EXMC_A3,
   * GPIO_EXMC_A4, GPIO_EXMC_A5, GPIO_EXMC_A6, GPIO_EXMC_A7,
   * GPIO_EXMC_A8, GPIO_EXMC_A9, GPIO_EXMC_A10, GPIO_EXMC_A11,
   *
   * control lines
   *
   * GPIO_EXMC_SDCKE0, GPIO_EXMC_SDNE0, GPIO_EXMC_SDNWE, GPIO_EXMC_NBL0,
   * GPIO_EXMC_NRAS, GPIO_EXMC_NBL1, GPIO_EXMC_A14(BA0), GPIO_EXMC_A15(BA1),
   * GPIO_EXMC_SDCLK, GPIO_EXMC_SDNCAS,
   */

  GPIO_EXMC_SDNE0_1, GPIO_EXMC_SDCKE0_2,

  GPIO_EXMC_D2, GPIO_EXMC_D3, GPIO_EXMC_D0, GPIO_EXMC_D1,
  GPIO_EXMC_D13, GPIO_EXMC_D14, GPIO_EXMC_D15,

  GPIO_EXMC_NBL0, GPIO_EXMC_NBL1, GPIO_EXMC_D4, GPIO_EXMC_D5,
  GPIO_EXMC_D6, GPIO_EXMC_D7, GPIO_EXMC_D8, GPIO_EXMC_D9,
  GPIO_EXMC_D10, GPIO_EXMC_D11, GPIO_EXMC_D12,

  GPIO_EXMC_A0, GPIO_EXMC_A1, GPIO_EXMC_A2, GPIO_EXMC_A3, GPIO_EXMC_A4,
  GPIO_EXMC_A5, GPIO_EXMC_NRAS, GPIO_EXMC_A6, GPIO_EXMC_A7, GPIO_EXMC_A8,
  GPIO_EXMC_A9,

  GPIO_EXMC_A10, GPIO_EXMC_A11, GPIO_EXMC_A12, GPIO_EXMC_A14,
  GPIO_EXMC_A15, GPIO_EXMC_SDCLK, GPIO_EXMC_SDNCAS, GPIO_EXMC_SDNWE_3,
};

#define NUM_SDRAM_GPIOS (sizeof(g_sdram_config) / sizeof(uint32_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_sdram_initialize
 *
 * Description:
 *   Called from gd32_board_initialize to initialize external SDRAM access.
 *
 ****************************************************************************/

void gd32_sdram_initialize(void)
{
  uint32_t val;
  int i;
  volatile int count;

  /* Enable GPIOs as FMC / memory pins */

  for (i = 0; i < NUM_SDRAM_GPIOS; i++)
    {
      gd32_gpio_config(g_sdram_config[i]);
    }

  /* Enable AHB clocking to the FMC */

  gd32_exmc_enable();

  /* Configure and enable the SDRAM bank1
   *
   *   FMC clock = 180MHz/2 = 90MHz
   *   90MHz = 11,11 ns
   *   All timings from the datasheet for Speedgrade -7 (=7ns)
   */

  val = EXMC_PIPELINE_DELAY_1_HCLK |    /* rpipe = 1 hclk */
        EXMC_SDCLK_PERIODS_3_HCLK |     /* sdclk = 3 hclk */
        EXMC_CAS_LATENCY_3_SDCLK |      /* cas latency = 3 cycles */
        EXMC_SDCTL_NBK_4 |              /* 4 internal banks */
        EXMC_SDRAM_DATABUS_WIDTH_16B |  /* width = 16 bits */
        EXMC_SDCTL_BRSTRD |             /* burst read */
        EXMC_SDRAM_ROW_ADDRESS_13 |     /* numrows = 13 */
        EXMC_SDRAM_COL_ADDRESS_9;       /* numcols = 9 bits */
  gd32_exmc_sdram_set_control(1, val);

  /* gd32_exmc_sdram_set_control(2, val); */

  val = EXMC_SDTCFG_RCD(2) |            /* tRCD min = 15ns */
        EXMC_SDTCFG_RPD(2) |            /* tRP  min = 15ns */
        EXMC_SDTCFG_WRD(2) |            /* tWR      = 2CLK */
        EXMC_SDTCFG_ARFD(6) |           /* tRC  min = 63ns */
        EXMC_SDTCFG_RASD(5) |           /* tRAS min = 42ns */
        EXMC_SDTCFG_XSRD(7) |           /* tXSR min = 70ns */
        EXMC_SDTCFG_LMRD(2);            /* tMRD     = 2CLK */
  gd32_exmc_sdram_set_timing(1, val);

  /* SDRAM Initialization sequence */

  gd32_exmc_sdram_command(GD32_SDRAM_CLKEN);   /* Clock enable command */
  for (count = 0; count < 10000; count++);     /* Delay */
  gd32_exmc_sdram_command(GD32_SDRAM_PALL);    /* Precharge ALL command */
  gd32_exmc_sdram_command(GD32_SDRAM_REFRESH); /* Auto refresh command */
  gd32_exmc_sdram_command(GD32_SDRAM_MODEREG); /* Mode Register program */

  /* Set refresh count
   *
   * FMC_CLK = 90MHz
   * Refresh_Rate = 7.81us
   * Counter = (FMC_CLK * Refresh_Rate) - 20
   */

  gd32_exmc_sdram_set_refresh_rate(761);

  /* Disable write protection */

  gd32_exmc_sdram_write_protect(1, false);
}
