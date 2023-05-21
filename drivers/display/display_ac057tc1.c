/*
 * Copyright (c) 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT eink_ac057tc1

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/display/ac057tc1.h>
#include <zephyr/sys/byteorder.h>
#include "ac057tc1_regs.h"

LOG_MODULE_REGISTER(display_ac057tc1, CONFIG_DISPLAY_LOG_LEVEL);


struct ac057tc1_display_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec busy_gpio;
	struct gpio_dt_spec cmd_data_gpio;

	enum display_pixel_format current_pixel_format;
	uint16_t height;
	uint16_t width;
};

struct ac057tc1_display_data {
};

static inline void ac057tc1_busy_wait(const struct device *dev)
{
	const struct ac057tc1_display_config *config = dev->config;
	int pin = gpio_pin_get_dt(&config->busy_gpio);

	while (pin > 0) {
		__ASSERT(pin >= 0, "Failed to get pin level");
		k_msleep(AC057TC1_BUSY_DELAY);
		pin = gpio_pin_get_dt(&config->busy_gpio);
	}
}

static inline int ac057tc1_write_cmd(const struct device *dev, uint8_t cmd, const uint8_t *data,
				     size_t len)
{
	const struct ac057tc1_display_config *config = dev->config;
	struct spi_buf buf = {.buf = &cmd, .len = sizeof(cmd)};
	struct spi_buf_set buf_set = {.buffers = &buf, .count = 1};
	int err = 0;

	ac057tc1_busy_wait(dev);

	err = gpio_pin_set_dt(&config->cmd_data_gpio, 1);
	if (err < 0) {
		return err;
	}

	err = spi_write_dt(&config->spi, &buf_set);
	if (err < 0) {
		goto spi_out;
	}

	if (data != NULL) {
		buf.buf = (void *)data;
		buf.len = len;

		err = gpio_pin_set_dt(&config->cmd_data_gpio, 0);
		if (err < 0) {
			goto spi_out;
		}

		err = spi_write_dt(&config->spi, &buf_set);
		if (err < 0) {
			goto spi_out;
		}
	}

spi_out:
	spi_release_dt(&config->spi);
	return err;
}

static int ac057tc1_set_resolution(const struct device *dev)
{
	const struct ac057tc1_display_config *config = dev->config;
	uint8_t res_set_data[4] = {0};

	sys_put_be16(config->width, &res_set_data[0]);
	sys_put_be16(config->height, &res_set_data[2]);
	return ac057tc1_write_cmd(dev, AC057TC1_RESOLUTION_SET, res_set_data, sizeof(res_set_data));
}

static int ac057tc1_controller_init(const struct device *dev)
{
	struct ac057tc1_display_data *data = dev->data;
	const struct ac057tc1_display_config *config = dev->config;
	int error;

	/* Reset the panel */
	error = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (error < 0) {
		return error;
	}
	k_sleep(K_MSEC(AC057TC1_RESET_DELAY));
	error = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (error < 0) {
		return error;
	}
	k_sleep(K_MSEC(200));

	/* Wait for to be ready by reading busy high signal */
	ac057tc1_busy_wait(dev);

	uint8_t panel_set_data[] = {0xEF, 0x08};

	ac057tc1_write_cmd(dev, AC057TC1_PANEL_SET, panel_set_data, sizeof(panel_set_data));

	uint8_t power_set_data[] = {0x37, 0x00, 0x05, 0x05};

	ac057tc1_write_cmd(dev, AC057TC1_POWER_SET, power_set_data, sizeof(power_set_data));

	ac057tc1_write_cmd(dev, AC057TC1_POWER_OFF_SEQ_SET, NULL, 0);

	uint8_t booster_softstart_data[] = {0xC7, 0xC7, 0x1D};

	ac057tc1_write_cmd(dev, AC057TC1_BOOSTER_SOFTSTART, booster_softstart_data,
			   sizeof(booster_softstart_data));

	uint8_t temp_sensor_enable_data = 0x00;

	ac057tc1_write_cmd(dev, AC057TC1_TEMP_SENSOR_EN, &temp_sensor_enable_data,
			   sizeof(temp_sensor_enable_data));

	uint8_t vcom_data = 0x37;

	ac057tc1_write_cmd(dev, AC057TC1_VCOM_DATA_INTERVAL, &vcom_data, sizeof(vcom_data));

	/* TODO: WTF is this*/
	uint8_t strange_data = 0x20;

	ac057tc1_write_cmd(dev, 0x60, &strange_data, sizeof(strange_data));

	ac057tc1_set_resolution(dev);

	/* TODO: WTF is this*/
	uint8_t strange_data2 = 0xAA;

	ac057tc1_write_cmd(dev, 0xE3, &strange_data2, sizeof(strange_data2));

	k_sleep(K_MSEC(100));

	return 0;
}

static int ac057tc1_display_init(const struct device *dev)
{
	struct ac057tc1_display_data *data = dev->data;
	const struct ac057tc1_display_config *config = dev->config;
	int error;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s not ready", config->spi.bus->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_ERR("Reset GPIO device not ready");
		return -ENODEV;
	}

	error = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (error < 0) {
		LOG_ERR("Failed to configure reset GPIO");
		return error;
	}

	if (!gpio_is_ready_dt(&config->busy_gpio)) {
		LOG_ERR("Busy GPIO device not ready");
		return -ENODEV;
	}

	error = gpio_pin_configure_dt(&config->busy_gpio, GPIO_OUTPUT_INACTIVE);
	if (error < 0) {
		LOG_ERR("Failed to configure busy GPIO");
		return error;
	}

	if (!gpio_is_ready_dt(&config->cmd_data_gpio)) {
		LOG_ERR("Command data GPIO device not ready");
		return -ENODEV;
	}

	error = gpio_pin_configure_dt(&config->cmd_data_gpio, GPIO_OUTPUT_INACTIVE);
	if (error < 0) {
		LOG_ERR("Failed to configure command data GPIO");
		return error;
	}

	return ac057tc1_controller_init(dev);
}

static int ac057tc1_display_write(const struct device *dev, const uint16_t x, const uint16_t y,
				  const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ac057tc1_display_config *config = dev->config;
	int error;

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
	__ASSERT(desc->pitch <= config->width, "Pitch in descriptor is larger than screen size");
	__ASSERT(desc->height <= config->height, "Height in descriptor is larger than screen size");
	__ASSERT(x == 0 && y == 0, "Partial redraw not available");

	error = ac057tc1_set_resolution(dev);
	if (error < 0) {
		return error;
	}

	error = ac057tc1_write_cmd(dev, AC057TC1_DATA_START_TRANS, (uint8_t *)buf, desc->buf_size);
	if (error < 0) {
		return error;
	}

	error = ac057tc1_write_cmd(dev, AC057TC1_DISPLAY_REF, NULL, 0);
	if (error < 0) {
		return error;
	}

	error = ac057tc1_write_cmd(dev, AC057TC1_POWER_OFF, NULL, 0);
	if (error < 0) {
		return error;
	}

	return 0;
}

static int ac057tc1_display_read(const struct device *dev, const uint16_t x, const uint16_t y,
				 const struct display_buffer_descriptor *desc, void *buf)
{
	return -ENOTSUP;
}

static void *ac057tc1_display_get_framebuffer(const struct device *dev)
{
	return NULL;
}

static int ac057tc1_display_blanking_off(const struct device *dev)
{
	return 0;
}

static int ac057tc1_display_blanking_on(const struct device *dev)
{
	return 0;
}

static int ac057tc1_display_set_brightness(const struct device *dev, const uint8_t brightness)
{
	return -ENOTSUP;
}

static int ac057tc1_display_set_contrast(const struct device *dev, const uint8_t contrast)
{
	return -ENOTSUP;
}

static void ac057tc1_display_get_capabilities(const struct device *dev,
					      struct display_capabilities *capabilities)
{
	const struct ac057tc1_display_config *config = dev->config;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = config->width;
	capabilities->y_resolution = config->height;
	capabilities->supported_pixel_formats =
		PIXEL_FORMAT_MONO01 | PIXEL_FORMAT_MONO10 | PIXEL_FORMAT_FIXED_PALETTE_6;
	capabilities->current_pixel_format = config->current_pixel_format;
	capabilities->screen_info = SCREEN_INFO_MONO_MSB_FIRST | SCREEN_INFO_EPD;
}

static int ac057tc1_display_set_pixel_format(const struct device *dev,
					     const enum display_pixel_format pixel_format)
{
	if (pixel_format == PIXEL_FORMAT_FIXED_PALETTE_6) {
		return 0;
	}
	LOG_ERR("Pixel format change not implemented");
	return -ENOTSUP;
}

static int ac057tc1_display_set_orientation(const struct device *dev,
					    const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Orientation change not supported");
	return -ENOTSUP;
}

static const struct display_driver_api ac057tc1_display_api = {
	.blanking_on = ac057tc1_display_blanking_on,
	.blanking_off = ac057tc1_display_blanking_off,
	.write = ac057tc1_display_write,
	.read = ac057tc1_display_read,
	.get_framebuffer = ac057tc1_display_get_framebuffer,
	.set_brightness = ac057tc1_display_set_brightness,
	.set_contrast = ac057tc1_display_set_contrast,
	.get_capabilities = ac057tc1_display_get_capabilities,
	.set_pixel_format = ac057tc1_display_set_pixel_format,
	.set_orientation = ac057tc1_display_set_orientation,
};

#define AC057TC1_DEFINE(n)                                                                         \
	static const struct ac057tc1_display_config ac057tc1_config_##n = {                        \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),           \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),                               \
		.cmd_data_gpio = GPIO_DT_SPEC_INST_GET(n, dc_gpios),                               \
		.busy_gpio = GPIO_DT_SPEC_INST_GET(n, busy_gpios),                                 \
		.height = DT_INST_PROP(n, height),                                                 \
		.width = DT_INST_PROP(n, width),                                                   \
	};                                                                                         \
                                                                                                   \
	static struct ac057tc1_display_data ac057tc1_data_##n;                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &ac057tc1_display_init, NULL, &ac057tc1_data_##n,                 \
			      &ac057tc1_config_##n, APPLICATION, CONFIG_DISPLAY_INIT_PRIORITY,     \
			      &ac057tc1_display_api);

DT_INST_FOREACH_STATUS_OKAY(AC057TC1_DEFINE)
