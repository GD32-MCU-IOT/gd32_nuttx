/****************************************************************************
 * arch/arm/src/gd32e11x/gd32e11x_pmu.h
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

#ifndef __ARCH_ARM_SRC_GD32E11X_GD32E11X_PMU_H
#define __ARCH_ARM_SRC_GD32E11X_GD32E11X_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "hardware/gd32e11x_pmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Need TODO */

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pmu_to_sleepmode
 *
 * Description:
 *   PMU work in sleep mode.
 *
 * Input Parameters:
 *   sleepmodecmd - PMU command constants
 *        - WFI_CMD: use WFI command
 *        - WFE_CMD: use WFE command
 *   sleeponexit
 *        - true:  SLEEPONEXIT bit is set when the WFI instruction is
 *                 executed, the MCU enters Sleep mode as soon as it
 *                 exits the lowest priority ISR.
 *        - false: SLEEPONEXIT bit is cleared, the MCU enters Sleep
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_sleepmode(uint8_t sleepmodecmd, bool sleeponexit);

/****************************************************************************
 * Name: gd32_pmu_to_deepsleepmode
 *
 * Description:
 *   PMU work in deep-sleep mode
 *
 * Input Parameters:
 *   ldo
 *       - PMU_LDO_NORMAL:   LDO normal work when pmu enter deep-sleep mode
 *       - PMU_LDO_LOWPOWER: LDO work at low power mode when pmu enter
 *                           deep-sleep mode
 *   lowdrive
 *       - PMU_LOWDRIVER_DISABLE: Low-driver mode disable in deep-sleep mode
 *       - PMU_LOWDRIVER_ENABLE:  Low-driver mode enable in deep-sleep mode
 *
 *   deepsleepmodecmdd - PMU command constants
 *       - WFI_CMD: use WFI command
 *       - WFE_CMD: use WFE command
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_deepsleepmode(uint32_t ldo, uint32_t lowdrive,
                               uint8_t deepsleepmodecmd);

/****************************************************************************
 * Name: gd32_pmu_to_standbymode
 *
 * Description:
 *   PMU work in standby mode
 *
 * Input Parameters:
 *   standbymodecmd - PMU command constants
 *                  WFI_CMD: use WFI command
 *                  WFE_CMD: use WFE command
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_to_standbymode(uint8_t standbymodecmd);

/****************************************************************************
 * Name: gd32_pmu_wakeup_pin_enable
 *
 * Description:
 *   Enables PMU wakeup pin.
 *
 ****************************************************************************/

void gd32_pmu_wakeup_pin_enable(void);

/****************************************************************************
 * Name: gd32_pmu_wakeup_pin_disable
 *
 * Description:
 *   Disables PMU wakeup pin.
 *
 ****************************************************************************/

void gd32_pmu_wakeup_pin_disable(void);

/****************************************************************************
 * Name: gd32_pmu_backup_ldo_config
 *
 * Description:
 *   Enables the backup ldo, to open Backup SRAM LDO for data protection of
 *   backup SRAM when VDD shut down. When VDD shut down and this bit is
 *   cleared, the data in Backup SRAM will be lost.
 *   Once set, the application should wait that the Backup SRAM LDO flag
 *   (BLDORF) is set to indicate that the data written into the RAM will be
 *   maintained when VDD shut down.
 *
 * Input Parameters:
 *   bkp_ldo - state to set it to
 *           - PMU_BLDOON_OFF: backup SRAM LDO closed
 *           - PMU_BLDOON_ON: backup SRAM LDO closed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_backup_ldo_config(bool bkp_ldo);

/****************************************************************************
 * Name: gd32_pmu_backup_init
 *
 * Description:
 *   Insures the referenced count access to the backup domain
 *   (RTC registers, RTC backup data registers and backup SRAM is consistent
 *   with the hardware state without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - set the initial state of the enable and the
 *              bkp_writable_counter
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_pmu_backup_init(bool writable);

/****************************************************************************
 * Name: gd32_pmu_backup_write_enable
 *
 * Description:
 *   Enableswrite access to the registers in backup domain
 *
 ****************************************************************************/

void gd32_pmu_backup_write_enable(void);

/****************************************************************************
 * Name: gd32_pmu_backup_write_disable
 *
 * Description:
 *   DIsables write access to the registers in backup domain
 *
 ****************************************************************************/

void gd32_pmu_backup_write_disable(void);

/****************************************************************************
 * Name: gd32_pmu_flag_get
 *
 * Description:
 *   Get flag state
 *
 * Input Parameters:
 *   flag - PMU_CS_WUF: wakeup flag
 *        - PMU_CS_STBF: standby flag
 *        - PMU_CS_LVDF: lvd flag
 *        - PMU_CS_BLDORF: backup SRAM LDO ready flag
 *        - PMU_CS_LDOVSRF: LDO voltage select ready flag
 *        - PMU_CS_HDRF: high-driver ready flag
 *        - PMU_CS_HDSRF: high-driver switch ready flag
 *        - PMU_CS_LDRF: low-driver mode ready flag
 *
 ****************************************************************************/

bool gd32_pmu_flag_get(uint32_t flag);

/****************************************************************************
 * Name: gd32_pmu_backup_write_disable
 *
 * Description:
 *   Clear the flag
 *
 * Input Parameters:
 *   flag - PMU_FLAG_RESET_WAKEUP: reset wakeup flag
 *        - PMU_FLAG_RESET_STANDBY: reset standby flag
 *
 ****************************************************************************/

void gd32_pmu_flag_clear(uint32_t flag);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32E11X_GD32E11X_PMU_H */
