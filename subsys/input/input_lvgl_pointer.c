/*
 * Copyright 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_input_lvgl_pointer

#include "input_lvgl_common.h"
#include <zephyr/drivers/display.h>
#include <lvgl_display.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(input_lvgl);

struct input_lvgl_pointer_config {
	struct input_lvgl_common_config common_config; /* Needs to be first member */
	bool swap_xy;
	bool invert_x;
	bool invert_y;
};

static void process_input_event(const struct device *dev, struct input_event *evt)
{
	const struct input_lvgl_pointer_config *cfg = dev->config;
	struct input_lvgl_common_data *data = dev->data;
	lv_disp_t *disp = lv_disp_get_default();
	struct lvgl_disp_data *disp_data = disp->driver->user_data;
	struct display_capabilities *cap = &disp_data->cap;

	switch (evt->code) {
	case INPUT_ABS_X:
		data->pending_event.point.x = evt->value;
		break;
	case INPUT_ABS_Y:
		data->pending_event.point.y = evt->value;
		break;
	case INPUT_BTN_TOUCH:
		data->pending_event.state = evt->value ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
		break;
	}

	if (!evt->sync) {
		return;
	}

	/* adjust coordinates */
	if (cfg->swap_xy) {
		lv_coord_t x;

		x = data->pending_event.point.x;
		data->pending_event.point.x = data->pending_event.point.y;
		data->pending_event.point.y = x;
	}

	if (cfg->invert_x) {
		if (cap->current_orientation == DISPLAY_ORIENTATION_NORMAL ||
		    cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_180) {
			data->pending_event.point.x =
				cap->x_resolution - data->pending_event.point.x;
		} else {
			data->pending_event.point.x =
				cap->y_resolution - data->pending_event.point.x;
		}
	}

	if (cfg->invert_y) {
		if (cap->current_orientation == DISPLAY_ORIENTATION_NORMAL ||
		    cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_180) {
			data->pending_event.point.y =
				cap->y_resolution - data->pending_event.point.y;
		} else {
			data->pending_event.point.y =
				cap->x_resolution - data->pending_event.point.y;
		}
	}

	/* rotate touch point to match display rotation */
	if (cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_90) {
		lv_coord_t x;

		x = data->pending_event.point.x;
		data->pending_event.point.x = data->pending_event.point.y;
		data->pending_event.point.y = cap->y_resolution - x;
	} else if (cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		data->pending_event.point.x = cap->x_resolution - data->pending_event.point.x;
		data->pending_event.point.y = cap->y_resolution - data->pending_event.point.y;
	} else if (cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_270) {
		lv_coord_t x;

		x = data->pending_event.point.x;
		data->pending_event.point.x = cap->x_resolution - data->pending_event.point.y;
		data->pending_event.point.y = x;
	}

	/* filter readings within display */
	if (data->pending_event.point.x <= 0) {
		data->pending_event.point.x = 0;
	} else if (data->pending_event.point.x >= cap->x_resolution) {
		data->pending_event.point.x = cap->x_resolution - 1;
	}

	if (data->pending_event.point.y <= 0) {
		data->pending_event.point.y = 0;
	} else if (data->pending_event.point.y >= cap->y_resolution) {
		data->pending_event.point.y = cap->y_resolution - 1;
	}

	if (k_msgq_put(cfg->common_config.event_msgq, &data->pending_event, K_NO_WAIT) != 0) {
		LOG_DBG("Could not put input data into queue");
	}
}

int input_lvgl_pointer_init(const struct device *dev)
{
	return register_lvgl_indev_driver(LV_INDEV_TYPE_POINTER, dev);
}

#define INPUT_LVGL_POINTER_DEFINE(_inst)                                                           \
	LVGL_INPUT_DECLARE(_inst, pointer, CONFIG_INPUT_LVGL_POINTER_MSGQ_COUNT);                  \
	static const struct input_lvgl_pointer_config input_lvgl_pointer_config##_inst = {         \
		.common_config.input_dev = INPUT_LVGL_DEVICE(_inst),                               \
		.common_config.event_msgq = &INPUT_LVGL_EVENT_MSGQ(_inst, pointer),                \
		.swap_xy = DT_INST_PROP(_inst, swap_xy),                                           \
		.invert_x = DT_INST_PROP(_inst, invert_x),                                         \
		.invert_y = DT_INST_PROP(_inst, invert_y),                                         \
	};                                                                                         \
	static struct input_lvgl_common_data input_lvgl_common_data##_inst;                        \
	DEVICE_DT_INST_DEFINE(_inst, input_lvgl_pointer_init, NULL,                                \
			      &input_lvgl_common_data##_inst, &input_lvgl_pointer_config##_inst,   \
			      APPLICATION, CONFIG_INPUT_LVGL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(INPUT_LVGL_POINTER_DEFINE)
