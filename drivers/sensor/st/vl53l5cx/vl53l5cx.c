/*
 * Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l5cx

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>

#include "vl53l5cx_api.h"
#include "vl53l5cx_platform.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(vl53l5cx, CONFIG_SENSOR_LOG_LEVEL);

struct vl53l5cx_data {
	VL53L5CX_Configuration vl53l5cx;
	VL53L5CX_ResultsData result_data;
	struct gpio_callback int_gpio_cb;
	struct k_sem data_ready_sem;
	const struct device *dev;
};

struct vl53l5cx_config {
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec rst_gpio;
};

#define DATA_READY_TIMEOUT 1000

static void vl53l5cx_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				   uint32_t pins)
{
	struct vl53l5cx_data *data = CONTAINER_OF(cb, struct vl53l5cx_data, int_gpio_cb);

	k_sem_give(&data->data_ready_sem);
}

static int vl53l5cx_init_interrupt(const struct device *dev)
{
	struct vl53l5cx_data *drv_data = dev->data;
	const struct vl53l5cx_config *config = dev->config;
	int ret;

	drv_data->dev = dev;

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("%s: device %s is not ready", dev->name, config->int_gpio.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to configure GPIO interrupt", dev->name);
		return -EIO;
	}

	gpio_init_callback(&drv_data->int_gpio_cb, vl53l5cx_gpio_callback,
			   BIT(config->int_gpio.pin));

	ret = gpio_add_callback(config->int_gpio.port, &drv_data->int_gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to set gpio callback!");
		return -EIO;
	}
	return 0;
}

static int vl53l5cx_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct vl53l5cx_data *data = dev->data;
	uint8_t ready = 0;

	vl53l5cx_start_ranging(&data->vl53l5cx);

	k_sleep(K_MSEC(1000));
	vl53l5cx_check_data_ready(&data->vl53l5cx, &ready);
	LOG_ERR("Ready %d", ready);

	vl53l5cx_get_ranging_data(&data->vl53l5cx, &data->result_data);

	vl53l5cx_stop_ranging(&data->vl53l5cx);

	return 0;
}

static int vl53l5cx_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_DISTANCE) {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api vl53l5cx_driver_api = {
	.sample_fetch = vl53l5cx_sample_fetch,
	.channel_get = vl53l5cx_channel_get,
};

static int vl53l5cx_dev_init(const struct device *dev)
{
	const struct vl53l5cx_config *config = dev->config;
	struct vl53l5cx_data *data = dev->data;

	if (!i2c_is_ready_dt(&data->vl53l5cx.platform.i2c)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("Reset gpio not ready");
		return -ENODEV;
	}

	vl53l5cx_init_interrupt(dev);
	k_sem_init(&data->data_ready_sem, 0, 1);

	vl53l5cx_init(&data->vl53l5cx);

	return 0;
}

#define VL53L5CX_INIT(i)                                                                           \
	static const struct vl53l5cx_config vl53l5cx_config_##i = {                                \
		.int_gpio = GPIO_DT_SPEC_INST_GET(i, int_gpios),                                   \
		.rst_gpio = GPIO_DT_SPEC_INST_GET(i, rst_gpios),                                   \
	};                                                                                         \
	static struct vl53l5cx_data vl53l5cx_data_##i = {                                          \
		.vl53l5cx.platform.i2c = I2C_DT_SPEC_INST_GET(i),                                  \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(i, vl53l5cx_dev_init, NULL, &vl53l5cx_data_##i,               \
				     &vl53l5cx_config_##i, POST_KERNEL,                            \
				     CONFIG_SENSOR_INIT_PRIORITY, &vl53l5cx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(VL53L5CX_INIT)
