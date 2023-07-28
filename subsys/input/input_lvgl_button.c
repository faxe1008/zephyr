/*
 * Copyright 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_input_lvgl_button

#include "input_lvgl_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(input_lvgl);

struct input_lvgl_button_config {
	struct input_lvgl_common_config common_config; /* Needs to be first member */
	const uint16_t *input_codes;
	uint8_t num_codes;
	lv_key_t key;
	lv_point_t coordinate;
};

static void process_input_event(const struct device *dev, struct input_event *evt)
{
	struct input_lvgl_common_data *data = dev->data;
	const struct input_lvgl_button_config *cfg = dev->config;
	uint8_t i;

	for (i = 0; i < cfg->num_codes; i++) {
		if (evt->code == cfg->input_codes[i]) {
			break;
		}
	}

	if (i == cfg->num_codes) {
		LOG_DBG("Ignored input event: %u", evt->code);
		return;
	}

	data->pending_event.key = cfg->key;
	data->pending_event.state = evt->value ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;

	if (k_msgq_put(cfg->common_config.event_msgq, &data->pending_event, K_NO_WAIT) != 0) {
		LOG_DBG("Could not put input data into queue");
	}
}

int input_lvgl_button_init(const struct device *dev)
{
	struct input_lvgl_common_data *data = dev->data;
	const struct input_lvgl_button_config *cfg = dev->config;
	int ret;

	ret = register_lvgl_indev_driver(LV_INDEV_TYPE_BUTTON, dev);
	if (ret < 0) {
		return ret;
	}

	if (cfg->coordinate.x >= 0 && cfg->coordinate.y >= 0) {
		lv_indev_set_button_points(data->indev, &cfg->coordinate);
	}

	return ret;
}

#define ASSERT_PROPERTIES(inst)                                                                    \
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(inst, x_coordinate) ==                                  \
			     DT_INST_NODE_HAS_PROP(inst, y_coordinate),                            \
		     "Property x-coordinate and y-coordinate must both be empty or assigned.");    \
	BUILD_ASSERT(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, x_coordinate),                        \
				 LVGL_COORD_VALID(DT_INST_PROP(inst, x_coordinate)), (1)),         \
		     "Invalid x coordinate value.");                                               \
	BUILD_ASSERT(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, y_coordinate),                        \
				 LVGL_COORD_VALID(DT_INST_PROP(inst, y_coordinate)), (1)),         \
		     "Invalid y coordinate value.");                                               \
	BUILD_ASSERT(LVGL_KEY_VALID(DT_INST_PROP(inst, lvgl_key)),                                 \
		     "Property lvgl-key needs to be in range [0,255]")

#define INPUT_LVGL_BUTTON_DEFINE(_inst)                                                            \
	ASSERT_PROPERTIES(_inst);                                                                  \
	LVGL_INPUT_DECLARE(_inst, button, CONFIG_INPUT_LVGL_BUTTON_MSGQ_COUNT);                    \
	static const uint16_t input_lvgl_button_codes_##_inst[] =                                  \
		DT_INST_PROP(_inst, input_codes);                                                  \
	static const struct input_lvgl_button_config input_lvgl_button_config##_inst = {           \
		.common_config.input_dev = INPUT_LVGL_DEVICE(_inst),                               \
		.common_config.event_msgq = &INPUT_LVGL_EVENT_MSGQ(_inst, button),                 \
		.input_codes = input_lvgl_button_codes_##_inst,                                    \
		.num_codes = DT_INST_PROP_LEN(_inst, input_codes),                                 \
		.key = DT_INST_PROP(_inst, lvgl_key),                                              \
		.coordinate.x = DT_INST_PROP_OR(_inst, x_coordinate, -1),                          \
		.coordinate.y = DT_INST_PROP_OR(_inst, y_coordinate, -1),                          \
	};                                                                                         \
	static struct input_lvgl_common_data input_lvgl_common_data##_inst;                        \
	DEVICE_DT_INST_DEFINE(_inst, input_lvgl_button_init, NULL, &input_lvgl_common_data##_inst, \
			      &input_lvgl_button_config##_inst, APPLICATION,                       \
			      CONFIG_INPUT_LVGL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(INPUT_LVGL_BUTTON_DEFINE)
