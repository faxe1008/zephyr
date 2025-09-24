/*
 * Copyright (c) 2025 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#define EXPANDER_VBATT_GPIO_SPEC       GPIO_DT_SPEC_GET(DT_NODELABEL(vbatt), power_gpios)
#define EXPANDER_SDCARD_EN_GPIO_SPEC   GPIO_DT_SPEC_GET(DT_NODELABEL(sdhc_spi), pwr_gpios)
#define GPIO_ON_EXPANDER_INIT_PRIORITY 80
#define I2C_RECOVER_INIT_PRIORITY      69

static inline void configure_vbatt_measure(bool on)
{
	struct gpio_dt_spec vbatt_gpio = EXPANDER_VBATT_GPIO_SPEC;

	if (!gpio_is_ready_dt(&vbatt_gpio)) {
		return;
	}

	gpio_pin_configure_dt(&vbatt_gpio, (on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE));
}

static inline void configure_sdcard_power(bool on)
{
	struct gpio_dt_spec sdcard_en_gpio = EXPANDER_SDCARD_EN_GPIO_SPEC;

	if (!gpio_is_ready_dt(&sdcard_en_gpio)) {
		return;
	}

	gpio_pin_configure_dt(&sdcard_en_gpio, (on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE));
}

static int board_inkplate_6color_gpio_init(void)
{
	configure_vbatt_measure(false);
	configure_sdcard_power(false);

	return 0;
}

/* needs to be done after I2C driver init, which is at
 * POST_KERNEL:CONFIG_GPIO_PCAL64XXA_INIT_PRIORITY.
 * but before POST_KERNEL:CONFIG_SENSOR_INIT_PRIORITY for the voltage-divider
 */
BUILD_ASSERT(GPIO_ON_EXPANDER_INIT_PRIORITY > CONFIG_GPIO_PCAL64XXA_INIT_PRIORITY);
#ifdef CONFIG_SENSOR
BUILD_ASSERT(GPIO_ON_EXPANDER_INIT_PRIORITY < CONFIG_SENSOR_INIT_PRIORITY);
#endif /* CONFIG_SENSOR */
SYS_INIT(board_inkplate_6color_gpio_init, POST_KERNEL, GPIO_ON_EXPANDER_INIT_PRIORITY);

#ifdef CONFIG_I2C

static int board_inkplate_6color_i2c_recover(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	return i2c_recover_bus(i2c_dev);
}

/* needs to be done before PCAL64XXA init, avoid SCL glitch on first write */
BUILD_ASSERT(I2C_RECOVER_INIT_PRIORITY < CONFIG_GPIO_PCAL64XXA_INIT_PRIORITY);
SYS_INIT(board_inkplate_6color_i2c_recover, POST_KERNEL, I2C_RECOVER_INIT_PRIORITY);

#endif /* CONFIG_I2C*/
