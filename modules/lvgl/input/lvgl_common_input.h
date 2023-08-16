/*
 * Copyright 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_LVGL_LVGL_COMMON_INPUT_H_
#define ZEPHYR_MODULES_LVGL_LVGL_COMMON_INPUT_H_

#include <lvgl.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>

#ifdef __cplusplus
extern "C" {
#endif

struct lvgl_common_input_config {
	const struct device *input_dev;
	struct k_msgq *event_msgq;
};

struct lvgl_common_input_data {
	lv_indev_drv_t indev_drv;
	lv_indev_t *indev;
	lv_indev_data_t pending_event;
};

int register_lvgl_indev_driver(lv_indev_type_t indev_type, const struct device *dev);

#define LVGL_INPUT_EVENT_MSGQ(_inst, _type) event_msgq_##_type##_inst
#define LVGL_INPUT_DEVICE(_inst)            DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(_inst, input))

#define LVGL_COORD_VALID(_coord) (_coord >= LV_COORD_MIN && _coord <= LV_COORD_MAX)
#define LVGL_KEY_VALID(_key)     (_key >= 0 && _key <= 255)

#define LVGL_INPUT_DECLARE(_inst, _type, _msgq_size)                                               \
	static void process_input_event(const struct device *dev, struct input_event *evt);        \
	static void lvgl_input_cb_##_inst(struct input_event *evt)                                 \
	{                                                                                          \
		process_input_event(DEVICE_DT_INST_GET(_inst), evt);                               \
	}                                                                                          \
	INPUT_CALLBACK_DEFINE(LVGL_INPUT_DEVICE(_inst), lvgl_input_cb_##_inst);                    \
	K_MSGQ_DEFINE(event_msgq_##_type##_inst, sizeof(lv_indev_data_t), _msgq_size, 4)

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* ZEPHYR_MODULES_LVGL_LVGL_COMMON_INPUT_H_ */
