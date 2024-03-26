/*
 * Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/backlight.h>
#include <zephyr/syscall_handler.h>

static inline int z_vrfy_backlight_enable(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_BACKLIGHT));
	return z_impl_backlight_enable(dev);
}
#include <syscalls/backlight_enable_mrsh.c>

static inline int z_vrfy_backlight_disable(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_BACKLIGHT));
	return z_impl_backlight_disable(dev);
}
#include <syscalls/backlight_disable_mrsh.c>

static inline int z_vrfy_backlight_set_brightness(const struct device *dev, uint32_t brightness)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_BACKLIGHT));
	return z_impl_backlight_set_brightness(dev, brightness);
}
#include <syscalls/backlight_set_brightness_mrsh.c>

static inline int z_vrfy_backlight_get_brightness(const struct device *dev, uint32_t *brightness)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_BACKLIGHT));
	return z_impl_backlight_get_brightness(dev, brightness);
}
#include <syscalls/backlight_get_brightness_mrsh.c>
