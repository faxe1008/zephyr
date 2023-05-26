/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hilink_ld2410

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/sensor/ld2410.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LD2410, CONFIG_SENSOR_LOG_LEVEL);

#define CFG_LD2410_MAX_FRAME_SIZE    50
#define CFG_LD2410_BUFFER_BATCH_SIZE 5
#define CFG_LD2410_SERIAL_TIMEOUT    100

struct ld2410_config {
	const struct device *uart_dev;
};

struct ld2410_data {
	struct ring_buf *rx_rb;

	struct k_sem tx_sem;
	struct k_sem rx_sem;
};

enum ld2410_command {
	ENTER_CONFIG_MODE = 0xFF00,
	LEAVE_CONFIG_MODE = 0xFE00,
	SET_MAX_DISTANCE_AND_DURATION = 0x6000,
	READ_SETTINGS = 0x6100,
	ENTER_ENGINEERING_MODE = 0x6200,
	LEAVE_ENGINEERING_MODE = 0x6300,
	SET_GATE_SENSITIVITY_CONFIG = 0x6400,
	READ_FIRMWARE_VERSION = 0xA000,
	SET_BAUDRATE = 0xA100,
	FACTORY_RESET = 0xA200,
	RESTART = 0xA300,
	SET_DISTANCE_RESOLUTION = 0xAA00
};

static void uart_cb_handler(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct ld2410_data *data = dev->data;
	uint8_t *rx_claim = NULL;
	uint32_t rx_claim_size;

	if (!uart_dev || !uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {

		while (uart_irq_rx_ready(uart_dev)) {
			rx_claim_size = ring_buf_get_claim(data->rx_rb, &rx_claim, CFG_LD2410_BUFFER_BATCH_SIZE);
			if(rx_claim_size == 0){
				break;
			}
			uart_fifo_read(uart_dev, rx_claim, rx_claim_size);
		}

		uart_irq_rx_disable(uart_dev);
		k_sem_give(&data->rx_sem);

		if (data->has_rsp) {
			k_sem_give(&data->tx_sem);
		}
	}

	if (uart_irq_tx_ready(uart_dev)) {
		data->xfer_bytes +=
			uart_fifo_fill(uart_dev, &mhz19b_cmds[data->cmd_idx][data->xfer_bytes],
				       MHZ19B_BUF_LEN - data->xfer_bytes);

		if (data->xfer_bytes == MHZ19B_BUF_LEN) {
			data->xfer_bytes = 0;
			uart_irq_tx_disable(uart_dev);
			if (!data->has_rsp) {
				k_sem_give(&data->tx_sem);
			}
		}
	}
}

static void ld2410_uart_flush(const struct device *dev)
{
	uint8_t c;

	while (uart_fifo_read(dev, &c, 1) > 0) {
		continue;
	}
}

int ld2410_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		    const struct sensor_value *val)
{

	struct ld2410_config *cfg = dev->config;
	struct ld2410_data *drv_data = dev->data;

	return 0;
}

int ld2410_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		    struct sensor_value *val)
{
	struct ld2410_data *drv_data = dev->data;
	const struct ld2410_config *cfg = dev->config;

	return 0;
}

static int ld2410_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

static int ld2410_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ld2410_data *drv_data = dev->data;

	return 0;
}

static const struct sensor_driver_api ld2410_api = {.sample_fetch = &ld2410_sample_fetch,
						    .channel_get = &ld2410_channel_get,
						    .attr_get = &ld2410_attr_get,
						    .attr_set = &ld2410_attr_set};

static int ld2410_init(const struct device *dev)
{
	struct ld2410_config *cfg = dev->config;
	struct ld2410_data *data = dev->data;

	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	uart_irq_rx_disable(cfg->uart_dev);
	uart_irq_tx_disable(cfg->uart_dev);

	ld2410_uart_flush(cfg->uart_dev);

	k_sem_init(&data->rx_sem, 0, 1);
	k_sem_init(&data->tx_sem, 1, 1);

	uart_irq_callback_user_data_set(cfg->uart_dev, uart_cb_handler, (void *)dev);

	return 0;
}

#define LD2410_DEFINE(inst)                                                                        \
	RING_BUF_DECLARE(ld2410_##inst##_rx_rb, CFG_LD2410_MAX_FRAME_SIZE);                        \
	RING_BUF_DECLARE(ld2410_##inst##_tx_rb, CFG_LD2410_MAX_FRAME_SIZE);                        \
	static struct ld2410_data ld2410_data_##inst = {                                           \
		.rx_rb = &ld2410_##inst##_rx_rb,                                                   \
	};                                                                                         \
                                                                                                   \
	static const struct ld2410_config ld2410_config_##inst = {                                 \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst))};                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &ld2410_init, NULL, &ld2410_data_##inst,                       \
			      &ld2410_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,     \
			      &ld2410_api);

DT_INST_FOREACH_STATUS_OKAY(LD2410_DEFINE)
