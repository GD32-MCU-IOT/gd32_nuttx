############################################################################
# boards/arm/gd32h7xx/gd32h759im-eval/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = gd32h7xx_boot.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += gd32h7xx_appinit.c gd32h7xx_bringup.c
else ifeq ($(CONFIG_BOARD_LATE_INITIALIZE),y)
CSRCS += gd32h7xx_bringup.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += gd32h7xx_autoleds.c
else
CSRCS += gd32h7xx_userleds.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += gd32h7xx_gpio.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += gd32h7xx_buttons.c
endif


ifeq ($(CONFIG_ARCH_BOARD_GD32H7XX_CUSTOM_CLOCKCONFIG),y)
CSRCS += gd32h7xx_clockconfig.c
endif

ifeq ($(CONFIG_GD32H7_I2C),y)
CSRCS += gd32h7xx_i2c.c
endif

ifeq ($(CONFIG_MTD_AT24XX),y)
ifeq ($(CONFIG_GD32H759IM_EVAL_AT24_TEST),y)
CSRCS += gd32h7xx_at24.c
endif
endif

include $(TOPDIR)/boards/Board.mk
