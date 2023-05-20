/*
 * Copyright (c) 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT eink_ac057tc1

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/display.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

struct ac057tc1_display_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec busy_gpio;
	struct gpio_dt_spec cmd_data_gpio;

	uint16_t height;
	uint16_t width;
};

struct ac057tc1_display_data {
	enum display_pixel_format current_pixel_format;
};

static int ac057tc1_display_init(const struct device *dev)
{
	struct ac057tc1_display_data *disp_data = dev->data;

	disp_data->current_pixel_format = PIXEL_FORMAT_ARGB_8888;

	return 0;
}

static int ac057tc1_display_write(const struct device *dev, const uint16_t x, const uint16_t y,
				  const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ac057tc1_display_config *config = dev->config;

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
	__ASSERT(desc->pitch <= config->width, "Pitch in descriptor is larger than screen size");
	__ASSERT(desc->height <= config->height, "Height in descriptor is larger than screen size");
	__ASSERT(x + desc->pitch <= config->width,
		 "Writing outside screen boundaries in horizontal direction");
	__ASSERT(y + desc->height <= config->height,
		 "Writing outside screen boundaries in vertical direction");

	if (desc->width > desc->pitch || x + desc->pitch > config->width ||
	    y + desc->height > config->height) {
		return -EINVAL;
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
	return 0;
}

static int ac057tc1_display_set_contrast(const struct device *dev, const uint8_t contrast)
{
	return 0;
}

static void ac057tc1_display_get_capabilities(const struct device *dev,
					      struct display_capabilities *capabilities)
{
	const struct ac057tc1_display_config *config = dev->config;
	struct ac057tc1_display_data *disp_data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = config->width;
	capabilities->y_resolution = config->height;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_ARGB_8888 | PIXEL_FORMAT_RGB_888 |
						PIXEL_FORMAT_MONO01 | PIXEL_FORMAT_MONO10;
	capabilities->current_pixel_format = disp_data->current_pixel_format;
	capabilities->screen_info = SCREEN_INFO_MONO_VTILED | SCREEN_INFO_MONO_MSB_FIRST;
}

static int ac057tc1_display_set_pixel_format(const struct device *dev,
					     const enum display_pixel_format pixel_format)
{
	struct ac057tc1_display_data *disp_data = dev->data;

	disp_data->current_pixel_format = pixel_format;
	return 0;
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
};

#define DISPLAY_AC057TC1_DEFINE(n)                                                                 \
	static const struct ac057tc1_display_config ac057tc1_config_##n = {                        \
		.height = DT_INST_PROP(n, height),                                                 \
		.width = DT_INST_PROP(n, width),                                                   \
	};                                                                                         \
                                                                                                   \
	static struct ac057tc1_display_data ac057tc1_data_##n;                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &ac057tc1_display_init, NULL, &ac057tc1_data_##n,                 \
			      &ac057tc_config_##n, APPLICATION, CONFIG_DISPLAY_INIT_PRIORITY,      \
			      &ac057tc1_display_api);

DT_INST_FOREACH_STATUS_OKAY(DISPLAY_AC057TC1_DEFINE)
