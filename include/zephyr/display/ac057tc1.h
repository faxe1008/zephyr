/*
 * Copyright 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DISPLAY_AC057TC1_DISPLAY_H_
#define ZEPHYR_INCLUDE_DISPLAY_AC057TC1_DISPLAY_H_

#include <zephyr/drivers/display.h>

enum display_pixel_format_ac057tc1 {
	/** Fixed palette with 6 available colors */
	PIXEL_FORMAT_FIXED_PALETTE_6 = PIXEL_FORMAT_PRIV_START,
};

enum ac057tc1_color {
	AC057TC1_COLOR_BLACK = 0x0,
	AC057TC1_COLOR_WHITE = 0x1,
	AC057TC1_COLOR_GREEN = 0x2,
	AC057TC1_COLOR_BLUE = 0x3,
	AC057TC1_COLOR_RED = 0x4,
	AC057TC1_COLOR_YELLOW = 0x5,
	AC057TC1_COLOR_ORANGE = 0x6
};

#endif /* ZEPHYR_INCLUDE_DISPLAY_AC057TC1_DISPLAY_H_ */
