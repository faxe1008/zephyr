/*
 * Copyright (c) 2024 Thomas Stenersen <thomas.stenersen@easee.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>

#include <lvgl.h>
#include <stdio.h>
#include <stdint.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

static const uint8_t lv_example_lottie_approve[] =
{
#include "./lv_example_lottie_approve.txt"
};

/* See https://github.com/lvgl/lvgl/blob/c903c1dc0fdfae20bce69d0a48894bdebae278ff/examples/widgets/lottie/lv_example_lottie_1.c#L8-L28 */
static void lv_example_lottie_1(void)
{
	lv_obj_t * lottie = lv_lottie_create(lv_screen_active());
	lv_lottie_set_src_data(lottie, lv_example_lottie_approve, sizeof(lv_example_lottie_approve));

#if LV_DRAW_BUF_ALIGN == 4 && LV_DRAW_BUF_STRIDE_ALIGN == 1
	/*If there are no special requirements, just declare a buffer
	  x4 because the Lottie is rendered in ARGB8888 format*/
	static uint8_t buf[64 * 64 * 4];
	lv_lottie_set_buffer(lottie, 64, 64, buf);
#else
	/*For GPUs and special alignment/strid setting use a draw_buf instead*/
	LV_DRAW_BUF_DEFINE(draw_buf, 64, 64, LV_COLOR_FORMAT_ARGB8888);
	lv_lottie_set_draw_buf(lottie, &draw_buf);
#endif

	lv_obj_center(lottie);
}

int main(void)
{
	const struct device *display_dev;
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

	lv_example_lottie_1();

	lv_timer_handler();
	display_blanking_off(display_dev);

	while (1) {
		lv_timer_handler();
		k_sleep(K_MSEC(10));
	}
	return 0;
}
