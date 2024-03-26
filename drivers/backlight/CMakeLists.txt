# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/backlight.h)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_USERSPACE			backlight_handlers.c)
zephyr_library_sources_ifdef(CONFIG_PWM_BACKLIGHT		backlight_pwm.c)
