/*
 * Copyright (c) 2025 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>

#define EXPANDER_VBATT_GPIO_SPEC	GPIO_DT_SPEC_GET(DT_NODELABEL(vbatt), power_gpios)

static inline void configure_vbatt_measure(bool on)
{
	struct gpio_dt_spec vbatt_gpio = EXPANDER_VBATT_GPIO_SPEC;

	if (!gpio_is_ready_dt(&vbatt_gpio)) {
		return;
	}

	gpio_pin_configure_dt(&vbatt_gpio, (on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE));
}

static int board_inkplate_6color_init(void)
{

	configure_vbatt_measure(false);

	return 0;
}

/* needs to be done after I2C driver init, which is at
 * POST_KERNEL:CONFIG_GPIO_PCAL64XXA_INIT_PRIORITY.
 * but before POST_KERNEL:CONFIG_SENSOR_INIT_PRIORITY for the voltage-divider
 */
SYS_INIT(board_inkplate_6color_init, POST_KERNEL, 80);
