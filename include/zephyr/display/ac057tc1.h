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

#endif /* ZEPHYR_INCLUDE_DISPLAY_AC057TC1_DISPLAY_H_ */
