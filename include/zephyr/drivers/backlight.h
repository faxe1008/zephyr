/*
 * Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public backlight driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BACKLIGHT_H_
#define ZEPHYR_INCLUDE_DRIVERS_BACKLIGHT_H_

/**
 * @brief Backlight Interface
 * @defgroup backlight_interface Backlight Interface
 * @since 3.7
 * @version 0.1.0
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Backlight common config structure
 *
 * This structure contains configuration common to all backlight drivers.
 */
struct backlight_common_config {
	/** Default brightness applied on start-up */
	uint32_t default_brightness;
	/** Maximum brightness */
	uint32_t max_brightness;
};

/**
 * @brief Initialize common backlight config from devicetree.
 *
 * @param node_id The devicetree node identifier.
 */
#define BACKLIGHT_DT_COMMON_CONFIG_INIT(node_id)                                                   \
	{                                                                                          \
		.default_brightness = DT_PROP_OR(node_id, default_brightness, 0),                  \
		.max_brightness = DT_PROP_OR(node, max_brightness, 0),                             \
	}

/**
 * @brief Initialize common backlight config from devicetree instance.
 *
 * @param inst Instance.
 */
#define BACKLIGHT_DT_INST_COMMON_CONFIG_INIT(inst)                                                 \
	BACKLIGHT_DT_COMMON_CONFIG_INIT(DT_DRV_INST(inst))

/**
 * @typedef backlight_api_enable()
 * @brief Callback API for enabling a backlight
 *
 * @see backlight_enable() for argument descriptions.
 */
typedef int (*backlight_api_enable)(const struct device *dev);

/**
 * @typedef backlight_api_disable()
 * @brief Callback API for disabling a backlight
 *
 * @see backlight_disable() for argument descriptions.
 */
typedef int (*backlight_api_disable)(const struct device *dev);

/**
 * @typedef backlight_api_set_brightness()
 * @brief Callback API for setting brightness of a backlight
 *
 * @see backlight_set_brightness() for argument descriptions.
 */
typedef int (*backlight_api_set_brightness)(const struct device *dev, uint32_t brightness);

/**
 * @typedef backlight_api_get_brightness()
 * @brief Callback API for getting brightness of a backlight
 *
 * @see backlight_get_brightness() for argument descriptions.
 */
typedef int (*backlight_api_get_brightness)(const struct device *dev, uint32_t *brightness);

/**
 * @brief Backlight driver API
 */
__subsystem struct backlight_driver_api {
	/* Mandatory callbacks. */
	backlight_api_enable enable;
	backlight_api_disable disable;
	backlight_api_set_brightness set_brightness;
	backlight_api_get_brightness get_brightness;
	/* Optional callbacks. */
};

/**
 * @brief Enable a backlight
 *
 * This routine enables a backlight
 *
 * @param dev Backlight device
 * @return 0 on success, negative on error
 */
__syscall int backlight_enable(const struct device *dev);

static inline int z_impl_backlight_enable(const struct device *dev)
{
	const struct backlight_driver_api *api = (const struct backlight_driver_api *)dev->api;

	return api->enable(dev);
}

/**
 * @brief Disable a backlight
 *
 * This routine disables a backlight
 *
 * @param dev Backlight device
 * @return 0 on success, negative on error
 */
__syscall int backlight_disable(const struct device *dev);

static inline int z_impl_backlight_disable(const struct device *dev)
{
	const struct backlight_driver_api *api = (const struct backlight_driver_api *)dev->api;

	return api->disable(dev);
}

/**
 * @brief Set the brightness of a backlight
 *
 * This routine sets the brightness of a backlight
 *
 * @param dev Backlight device
 * @return 0 on success, negative on error
 */
__syscall int backlight_set_brightness(const struct device *dev, uint32_t brightness);

static inline int z_impl_backlight_set_brightness(const struct device *dev, uint32_t brightness)
{
	const struct backlight_driver_api *api = (const struct backlight_driver_api *)dev->api;

	return api->set_brightness(dev, brightness);
}

/**
 * @brief Get the brightness of a backlight
 *
 * This routine gets the brightness of a backlight
 *
 * @param dev Backlight device
 * @return 0 on success, negative on error
 */
__syscall int backlight_get_brightness(const struct device *dev, uint32_t *brightness);

static inline int z_impl_backlight_get_brightness(const struct device *dev, uint32_t *brightness)
{
	const struct backlight_driver_api *api = (const struct backlight_driver_api *)dev->api;

	return api->get_brightness(dev, brightness);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <syscalls/backlight.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_BACKLIGHT_H_ */
