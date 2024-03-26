/*
 * Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT pwm_backlight

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/backlight.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(backlight_pwm, CONFIG_BACKLIGHT_LOG_LEVEL);

struct pwm_backlight_config {
	struct backlight_common_config common;
	struct pwm_dt_spec pwms;
	struct gpio_dt_spec enable_gpios;
	const struct device *regulator;
	uint32_t post_pwm_on_delay_ms;
	uint32_t pwm_off_delay_ms;
	uint32_t low_threshold_brightness;
};

static int pwm_backlight_set_brightness(const struct device *dev, uint32_t brightness)
{
	const struct pwm_backlight_config *cfg = dev->config;
	int ret;
	uint32_t pulse_width;
	uint32_t period = cfg->pwms.period;
	uint32_t lth = cfg->low_threshold_brightness;

	pulse_width = (brightness * period) / (100 - lth);

	ret = pwm_set_pulse_dt(&cfg->pwms, pulse_width);
	if (ret < 0) {
		LOG_ERR("failed to set pwm pulse width: %d", ret);
	}

	return ret;
}


static int pwm_backlight_enable(const struct device *dev)
{
	const struct pwm_backlight_config *cfg = dev->config;
	const struct backlight_common_config *common_cfg = &cfg->common;
	int ret = 0;

#ifdef CONFIG_REGULATOR
	if (cfg->regulator) {
		ret = regulator_enable(cfg->regulator);
		if (ret < 0) {
			LOG_ERR("failed to enable power supply");
			return ret;
		}
	}
#endif /* CONFIG_REGULATOR */

	if (cfg->post_pwm_on_delay_ms) {
		k_msleep(cfg->post_pwm_on_delay_ms);
	}

	if (common_cfg->default_brightness) {
		ret = pwm_backlight_set_brightness(dev, common_cfg->default_brightness);

		if (ret < 0) {
			LOG_ERR("failed to set brightness");
		}
		return ret;
	}

	if (gpio_is_ready_dt(&cfg->enable_gpios)) {
		gpio_pin_set_dt(&cfg->enable_gpios, 1);
	}

	return ret;
}

static int pwm_backlight_disable(const struct device *dev)
{
	const struct pwm_backlight_config *cfg = dev->config;
	int ret = 0;

	if (gpio_is_ready_dt(&cfg->enable_gpios)) {
		gpio_pin_set_dt(&cfg->enable_gpios, 0);
	}

	if (cfg->pwm_off_delay_ms) {
		k_msleep(cfg->pwm_off_delay_ms);
	}

	ret = pwm_set_pulse_dt(&cfg->pwms, 0);
	if (ret < 0) {
		LOG_ERR("failed to disable pwm");
		return ret;
	}

#ifdef CONFIG_REGULATOR
	if (cfg->regulator) {
		ret = regulator_disable(cfg->regulator);
		if (ret < 0) {
			LOG_ERR("failed to disable power supply");
		}
	}
#endif /* CONFIG_REGULATOR */

	return ret;
}


static int pwm_backlight_init(const struct device *dev)
{
	const struct pwm_backlight_config *cfg = dev->config;
	const struct backlight_common_config *common_cfg = &cfg->common;
	int ret = 0;

	if (!pwm_is_ready_dt(&cfg->pwms)) {
		LOG_ERR("pwm not ready");
		return -ENODEV;
	}

	if (cfg->enable_gpios.port) {

		if (gpio_is_ready_dt(&cfg->enable_gpios)) {
			LOG_ERR("enable gpios configured but not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->enable_gpios, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("could not configure enable-gpios");
			return ret;
		}
	}

	if (common_cfg->default_brightness) {
		ret = pwm_backlight_set_brightness(dev, common_cfg->default_brightness);
	}

	return ret;
}

static const struct backlight_driver_api pwm_backlight_api = {
	.enable = pwm_backlight_enable,
	.disable = pwm_backlight_disable,
	.set_brightness = pwm_backlight_set_brightness,
};

#define ASSERT_PROPERTIES(inst)                                                                    \
	BUILD_ASSERT(!DT_NODE_HAS_PROP(DT_DRV_INST(inst), power_supply) ||                         \
			     IS_ENABLED(CONFIG_REGULATOR),                                         \
		     "Assigning a power-supply phandle requires enabling of CONFIG_REGULATOR")

#define PWM_BACKLIGHT_INST(inst)                                                                   \
	ASSERT_PROPERTIES(inst);                                                                   \
	static const struct pwm_backlight_config pwm_backlight_config_##inst = {                   \
		.common = BACKLIGHT_DT_INST_COMMON_CONFIG_INIT(inst),                              \
		.pwms = PWM_DT_SPEC_INST_GET(inst),                                                \
		.enable_gpios = GPIO_DT_SPEC_INST_GET_OR(inst, enable_gpios, {0}),                 \
		.regulator = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, power_supply)),           \
		.post_pwm_on_delay_ms = DT_PROP_OR(inst, post_pwm_on_delay_ms, 0),                 \
		.pwm_off_delay_ms = DT_PROP_OR(inst, pwm_off_delay_ms, 0),                         \
		.low_threshold_brightness = DT_PROP_OR(inst, low_threshold_brightness, 0),         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &pwm_backlight_init, NULL, NULL, &pwm_backlight_config_##inst, \
			      POST_KERNEL, CONFIG_BACKLIGHT_INIT_PRIORITY, &pwm_backlight_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_BACKLIGHT_INST)
