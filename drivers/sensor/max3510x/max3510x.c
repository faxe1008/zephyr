/*
 * Copyright (c) 2023 Fabian Blatz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max3510x

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor/max3510x.h>
#include "max3510x_regs.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(max3510x, CONFIG_SENSOR_LOG_LEVEL);

#define SRV_REG_TIMEOUT 500

struct max3510x_fixed_float {
	uint16_t integral_part;
	uint16_t frac_part;
} __packed;

struct max3510x_config {
	struct spi_dt_spec spi;
	const struct gpio_dt_spec srv_irq_gpio;
	int ref_freq;
	int transducer_distance;
};

struct max3510x_data {
	const struct device *dev;
	uint16_t sample;
	struct gpio_callback srv_cb;
	struct k_sem srv_sem;

	double tof_up;
	double tof_down;
	double tof_diff;
};

static int32_t max3510x_fixed_to_time(const struct max3510x_fixed_float *p_fixed)
{
	return (p_fixed->integral_part << 16) | p_fixed->frac_part;
}

static double max3510x_time_to_double(int32_t time, double ref_freq)
{
	const double c = 1.0 / (ref_freq * 65536.0);
	return (double)time * c;
}

static double max3510x_fixed_to_double(const struct max3510x_fixed_float *p_fixed, double ref_freq)
{
	return max3510x_time_to_double(max3510x_fixed_to_time(p_fixed), ref_freq);
}

static void srv_irq_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct max3510x_data *drv_data = CONTAINER_OF(cb, struct max3510x_data, srv_cb);
	const struct max3510x_config *const drv_cfg = drv_data->dev->config;

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(&drv_cfg->srv_irq_gpio, GPIO_INT_DISABLE);
	k_sem_give(&drv_data->srv_sem);
}

static int max3510x_write_register(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	const struct max3510x_config *const cfg = dev->config;

	const struct spi_buf bufs[] = {{
					       .buf = &reg,
					       .len = 1,
				       },
				       {.buf = data, .len = len}};
	const struct spi_buf_set tx = {.buffers = bufs, .count = 2};

	return spi_write_dt(&cfg->spi, &tx);
}

static int max3510x_send_opcode(const struct device *dev, uint8_t opcode)
{
	return max3510x_write_register(dev, opcode, NULL, 0);
}

static int max3510x_read_register(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	const struct max3510x_config *cfg = dev->config;
	uint8_t read_reg = MAX3510X_OPCODE_READ_REG(reg);

	const struct spi_buf tx_bufs[] = {{
		.buf = &read_reg,
		.len = 1,
	}};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};
	const struct spi_buf rx_bufs[] = {{
		.buf = data,
		.len = len,
	}};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = 1};

	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int max3510x_write_reg_bitfield(const struct device *dev, uint8_t reg, uint8_t shift,
				       uint8_t width, uint16_t value)
{
	int ret;
	uint16_t current_reg_val;
	uint16_t mask;

	ret = max3510x_read_register(dev, reg, (uint8_t *)&current_reg_val,
				     sizeof(current_reg_val));
	if (ret < 0) {
		LOG_DBG("Reading register failed");
		return ret;
	}
	LOG_DBG("Cur Register[%u]=%u", reg, current_reg_val);

	mask = ((1 << width) - 1) << shift;
	value = (value << shift) & mask;
	/* Set bitfield value */
	value = (current_reg_val & ~mask) | value;

	ret = max3510x_write_register(dev, reg, (uint8_t *)&value, sizeof(value));
	if (ret < 0) {
		LOG_DBG("Error writing register value");
	}

	return ret;
}

static int max3510x_enable_service_interrupt(const struct device *dev, bool enable)
{
	return max3510x_write_reg_bitfield(
		dev, MAX3510X_REG_CALIBRATION_CONTROL,
		MAX3510X_REG_CALIBRATION_CONTROL_INT_EN_SHIFT,
		MAX3510X_REG_CALIBRATION_CONTROL_INT_EN_WIDTH,
		enable ? MAX3510X_REG_CALIBRATION_CONTROL_INT_EN_ENABLED
		       : MAX3510X_REG_CALIBRATION_CONTROL_INT_EN_DISABLED);
}

static int max3510x_wait_for_reset_complete(const struct device *dev)
{
	uint16_t status_reg_val = MAX3510X_REG_INTERRUPT_STATUS_INVALID;
	size_t retries = 3;

	while (status_reg_val == MAX3510X_REG_INTERRUPT_STATUS_INVALID && retries > 0) {
		if (max3510x_read_register(dev, MAX3510X_REG_INTERRUPT_STATUS,
					   (uint8_t *)&status_reg_val,
					   sizeof(status_reg_val)) == 0) {
			LOG_DBG("Interrupt status register is valid");
			return 0;
		}
		LOG_DBG("Interrupt status register is still invalid");
		k_sleep(K_MSEC(10));
		retries--;
	}
	return -ETIMEDOUT;
}

static int max3510x_read_double_reg(const struct device *dev, uint8_t reg_int, double *reg_result)
{
	const struct max3510x_config *cfg = dev->data;
	struct max3510x_fixed_float fixed_float_reg_val;
	int ret;

	ret = max3510x_read_register(dev, reg_int, (uint8_t *)&fixed_float_reg_val,
				     sizeof(fixed_float_reg_val));
	if (ret < 0) {
		LOG_DBG("Error reading reg");
		return ret;
	}

	if (fixed_float_reg_val.integral_part == MAX3510X_REG_TOF_INVALID ||
	    fixed_float_reg_val.frac_part == MAX3510X_REG_TOF_INVALID) {
		/* TODO: only for TOF regs ??? */
		LOG_DBG("Reg contains invalid values");
	}

	*reg_result = max3510x_fixed_to_double(&fixed_float_reg_val, cfg->ref_freq);

	return ret;
}

static int max3510x_fetch_tofs(const struct device *dev)
{
	struct max3510x_data *data = dev->data;
	const struct max3510x_config *cfg = dev->data;
	int ret;
	uint16_t status_reg_val;

	ret = gpio_pin_interrupt_configure_dt(&cfg->srv_irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Error enabling srv interrupt");
		return ret;
	}

	ret = max3510x_send_opcode(dev, MAX3510X_OPCODE_TOF_DIFF);
	if (ret < 0) {
		LOG_ERR("Error sending TOF_DIFF opcode");
		return ret;
	}

	ret = k_sem_take(&data->srv_sem, K_MSEC(SRV_REG_TIMEOUT));
	if (ret != 0) {
		LOG_ERR("Error taking srv semaphore");
		return ret;
	}

	ret = max3510x_read_register(dev, MAX3510X_REG_INTERRUPT_STATUS, (uint8_t *)&status_reg_val,
				     sizeof(status_reg_val));
	if (ret < 0) {
		LOG_ERR("Error reading interrupt status register");
		return ret;
	}

	LOG_DBG("IRQ Status Register Value: %u", status_reg_val);

	if ((status_reg_val & MAX3510X_REG_INTERRUPT_STATUS_TOF) == 0) {
		return (status_reg_val & MAX3510X_REG_INTERRUPT_STATUS_TO) ? -ETIMEDOUT : -EIO;
	}

	ret = max3510x_read_double_reg(dev, MAX3510X_REG_TOF_DIFF_AVGINT, &data->tof_diff);
	if (ret < 0) {
		LOG_WRN("Error reading TOF diff");
	}

	ret = max3510x_read_double_reg(dev, MAX3510X_REG_AVGUPINT, &data->tof_up);
	if (ret < 0) {
		LOG_WRN("Error reading TOF up");
	}

	ret = max3510x_read_double_reg(dev, MAX3510X_REG_AVGDNINT, &data->tof_down);
	if (ret < 0) {
		LOG_WRN("Error reading TOF down");
	}

	return 0;
}

static int max3510x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;

	switch ((enum sensor_channel_max3510x)chan) {
	case SENSOR_CHAN_MAX3510X_TOF_DIFF:
	case SENSOR_CHAN_MAX3510X_TOF_UP:
	case SENSOR_CHAN_MAX3510X_TOF_DOWN:
		ret = max3510x_fetch_tofs(dev);
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}

static int max3510x_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	return -ENOTSUP;
}

static const struct sensor_driver_api max3510x_api = {
	.sample_fetch = &max3510x_sample_fetch,
	.channel_get = &max3510x_channel_get,
};

static int max3510x_init(const struct device *dev)
{
	const struct max3510x_config *config = dev->config;
	struct max3510x_data *data = dev->data;
	int ret;

	data->dev = dev;
	k_sem_init(&data->srv_sem, 0, 1);

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->srv_irq_gpio)) {
		LOG_ERR("GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->srv_irq_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure gpio");
		return ret;
	}

	gpio_init_callback(&data->srv_cb, srv_irq_gpio_callback, BIT(config->srv_irq_gpio.pin));

	if (gpio_add_callback(config->srv_irq_gpio.port, &data->srv_cb) < 0) {
		LOG_ERR("Could not set gpio callback");
		return -EIO;
	}

	ret = max3510x_wait_for_reset_complete(dev);
	if (ret < 0) {
		LOG_ERR("Error waiting for initial reset to complete");
		return ret;
	}

	ret = max3510x_enable_service_interrupt(dev, true);
	if (ret < 0) {
		LOG_ERR("Error enabling service interrupt.");
		return ret;
	}

	return 0;
}

#define MAX3510X_INIT(n)                                                                           \
	static struct max3510x_data max3510x_data_##n;                                             \
	static const struct max3510x_config max3510x_config_##n = {                                \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),             \
		.srv_irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                               \
		.ref_freq = DT_INST_PROP(n, ref_frequency),                                        \
		.transducer_distance = DT_INST_PROP(n, transducer_distance),                       \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(n, &max3510x_init, NULL, &max3510x_data_##n,                  \
				     &max3510x_config_##n, POST_KERNEL,                            \
				     CONFIG_SENSOR_INIT_PRIORITY, &max3510x_api);

DT_INST_FOREACH_STATUS_OKAY(MAX3510X_INIT)
