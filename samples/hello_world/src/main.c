/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/display/ac057tc1.h>

#define DISPLAY_WIDTH  600
#define DISPLAY_HEIGHT 448

/* Framebuffer: 4 bits per pixel, 2 pixels per byte */
static uint8_t framebuffer[AC057TC1_BUFFER_SIZE(DISPLAY_WIDTH, DISPLAY_HEIGHT)];

static void fill_test_pattern(void)
{
	/* Create horizontal color stripes - 7 colors, each stripe ~64 pixels tall */
	const uint8_t colors[] = {
		AC057TC1_COLOR_BLACK,
		AC057TC1_COLOR_WHITE,
		AC057TC1_COLOR_GREEN,
		AC057TC1_COLOR_BLUE,
		AC057TC1_COLOR_RED,
		AC057TC1_COLOR_YELLOW,
		AC057TC1_COLOR_ORANGE,
	};
	const int num_colors = ARRAY_SIZE(colors);
	const int stripe_height = DISPLAY_HEIGHT / num_colors;

	for (int y = 0; y < DISPLAY_HEIGHT; y++) {
		int color_idx = y / stripe_height;
		if (color_idx >= num_colors) {
			color_idx = num_colors - 1;
		}
		uint8_t color = colors[color_idx];
		uint8_t packed = AC057TC1_PACK_PIXELS(color, color);

		for (int x = 0; x < DISPLAY_WIDTH / 2; x++) {
			framebuffer[y * (DISPLAY_WIDTH / 2) + x] = packed;
		}
	}
}

int main(void)
{
	const struct device *display;
	struct display_buffer_descriptor desc;
	int ret;

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	display = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_display));
	if (display == NULL) {
		printf("No display device found\n");
		return 0;
	}

	if (!device_is_ready(display)) {
		printf("Display device not ready\n");
		return 0;
	}

	printf("Display device: %s\n", display->name);

	/* Fill framebuffer with test pattern */
	fill_test_pattern();

	/* Set up buffer descriptor */
	desc.buf_size = sizeof(framebuffer);
	desc.width = DISPLAY_WIDTH;
	desc.height = DISPLAY_HEIGHT;
	desc.pitch = DISPLAY_WIDTH;

	/* Turn off blanking to enable display refresh */
	display_blanking_off(display);

	/* Write the framebuffer to display */
	printf("Writing test pattern to display...\n");
	ret = display_write(display, 0, 0, &desc, framebuffer);
	if (ret < 0) {
		printf("Failed to write display: %d\n", ret);
		return 0;
	}

	printf("Display updated successfully!\n");
	printf("You should see 7 horizontal color stripes:\n");
	printf("  Black, White, Green, Blue, Red, Yellow, Orange\n");

	return 0;
}
