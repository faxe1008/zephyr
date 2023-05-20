/* ac057tc1_regs.h - Registers definition for AC057TC1 compatible controller */

/*
 * Copyright (c) 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AC057TC1_REGS_H__
#define __AC057TC1_REGS_H__

#define AC057TC1_PANEL_SET          0x00
#define AC057TC1_POWER_SET          0x01
#define AC057TC1_POWER_OFF_SEQ_SET  0x03
#define AC057TC1_POWER_OFF          0x04
#define AC057TC1_BOOSTER_SOFTSTART  0x06
#define AC057TC1_DEEP_SLEEP         0x07
#define AC057TC1_DATA_START_TRANS   0x10
#define AC057TC1_DATA_STOP          0x11
#define AC057TC1_DISPLAY_REF        0x12
#define AC057TC1_IMAGE_PROCESS      0x13
#define AC057TC1_PLL_CONTROL        0x30
#define AC057TC1_TEMP_SENSOR        0x40
#define AC057TC1_TEMP_SENSOR_EN     0x41
#define AC057TC1_TEMP_SENSOR_WR     0x42
#define AC057TC1_TEMP_SENSOR_RD     0x43
#define AC057TC1_VCOM_DATA_INTERVAL 0x50
#define AC057TC1_LOW_POWER_DETECT   0x51
#define AC057TC1_RESOLUTION_SET     0x61
#define AC057TC1_STATUS             0x71
#define AC057TC1_VCOM_VALUE         0x81
#define AC057TC1_VCM_DC_SET         0x02

/* time constants in ms */
#define AC057TC1_RESET_DELAY 1
#define AC057TC1_BUSY_DELAY  1

#endif /* __AC057TC1_REGS_H__ */