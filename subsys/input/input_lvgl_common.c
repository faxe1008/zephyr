/*
 * Copyright 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>

#include "input_lvgl_common.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(input_lvgl, CONFIG_INPUT_LOG_LEVEL);

/* The lvgl module intializes its dependencies (memory, display driver) with
 * CONFIG_APPLICATION_INIT_PRIORITY. To successfully register drivers, the heap has to be
 * initialized. Also since this device relies on the input subsystem, ensure that this one is ready
 * too.
 */
BUILD_ASSERT(CONFIG_APPLICATION_INIT_PRIORITY < CONFIG_INPUT_LVGL_INIT_PRIORITY);
BUILD_ASSERT(CONFIG_INPUT_INIT_PRIORITY < CONFIG_INPUT_LVGL_INIT_PRIORITY);

static void lvgl_input_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
	const struct device *dev = drv->user_data;
	const struct input_lvgl_common_config *cfg = dev->config;

	k_msgq_get(cfg->event_msgq, data, K_NO_WAIT);
	data->continue_reading = k_msgq_num_used_get(cfg->event_msgq) > 0;
}

int register_lvgl_indev_driver(lv_indev_type_t indev_type, const struct device *dev)
{
	/* Currently no indev binding has its dedicated data
	 * if that ever changes ensure that `input_lvgl_common_data`
	 * remains the first member
	 */
	struct input_lvgl_common_data *common_data = dev->data;

	if (common_data == NULL) {
		return -EINVAL;
	}

	lv_indev_drv_init(&common_data->indev_drv);
	common_data->indev_drv.type = indev_type;
	common_data->indev_drv.read_cb = lvgl_input_read_cb;
	common_data->indev_drv.user_data = dev;
	common_data->indev = lv_indev_drv_register(&common_data->indev_drv);

	if (common_data->indev == NULL) {
		return -EINVAL;
	}

	return 0;
}
