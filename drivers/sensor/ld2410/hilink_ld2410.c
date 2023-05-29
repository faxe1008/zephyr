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

const uint8_t DATA_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t DATA_FRAME_FOOTER[] = {0xF8, 0xF7, 0xF6, 0xF5};

const uint8_t CMD_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t CMD_FRAME_FOOTER[] = {0x04, 0x03, 0x02, 0x01};

#define CFG_LD2410_MAX_FRAME_SIZE    50
#define CFG_LD2410_BUFFER_BATCH_SIZE 5
#define CFG_LD2410_SERIAL_TIMEOUT    100

#define LD2410_MAX_FRAME_BODYLEN 35
#define LD2410_CMD_ID_SIZE       sizeof(uint16_t)
#define FRAME_FOOTER_SIZE        sizeof(DATA_FRAME_FOOTER)
#define FRAME_HEADER_SIZE        sizeof(DATA_FRAME_HEADER)

enum ld2410_frame_type {
	DATA_FRAME = 0,
	ACK_FRAME
};

struct ld2410_frame {
	uint32_t header;
	uint16_t body_len;
	uint8_t body[LD2410_MAX_FRAME_BODYLEN];
	uint32_t footer;
} __packed;

struct ld2410_rx_frame {
	size_t total_bytes_read;
	union {
		struct ld2410_frame frame;
		uint8_t raw[2 * sizeof(struct ld2410_frame)];
	} data;
} __packed;

#define FRAME_HEADER_AND_SIZE_LENGTH (offsetof(struct ld2410_frame, body))

struct ld2410_tx_frame {
	size_t bytes_remaining;
	union {
		struct ld2410_frame frame;
		uint8_t raw_data[sizeof(struct ld2410_frame)];
	};
};

struct ld2410_config {
	const struct device *uart_dev;
};

struct ld2410_data {
	enum ld2410_frame_type awaited_rx_frame;
	struct ld2410_rx_frame rx_frame;

	struct ld2410_tx_frame tx_frame;

	struct k_sem tx_sem;
	struct k_sem rx_sem;
};

enum ld2410_command {
	ENTER_CONFIG_MODE = 0x00FF,
	LEAVE_CONFIG_MODE = 0x00FE,
	SET_MAX_DISTANCE_AND_DURATION = 0x0060,
	READ_SETTINGS = 0x0061,
	ENTER_ENGINEERING_MODE = 0x0062,
	LEAVE_ENGINEERING_MODE = 0x0063,
	SET_GATE_SENSITIVITY_CONFIG = 0x0064,
	READ_FIRMWARE_VERSION = 0x00A0,
	SET_BAUDRATE = 0x00A1,
	FACTORY_RESET = 0x00A2,
	RESTART = 0x00A3,
	SET_DISTANCE_RESOLUTION = 0x00AA
};

static int find_rx_frame_start(struct ld2410_rx_frame *rx_frame)
{
	size_t frame_start = 0;
	size_t frame_length;
	enum ld2410_frame_type frame_type;

	/* Ensure at least frame header info is read */
	if (rx_frame->total_bytes_read < FRAME_HEADER_AND_SIZE_LENGTH + FRAME_FOOTER_SIZE) {
		return -EAGAIN;
	}

	for (;;) {
		if (memcmp(&rx_frame->data.raw[frame_start], DATA_FRAME_HEADER,
			   sizeof(DATA_FRAME_HEADER)) == 0) {
			frame_type = DATA_FRAME;
			break;
		}

		if (memcmp(&rx_frame->data.raw[frame_start], CMD_FRAME_HEADER,
			   sizeof(CMD_FRAME_HEADER)) == 0) {
			frame_type = ACK_FRAME;
			break;
		}

		if (frame_start >= rx_frame->total_bytes_read) {
			/* If the buffer is full shift but leave at least header bytes in buffer */
			if (frame_start >= sizeof(rx_frame->data.raw)) {
				memmove(&rx_frame->data.raw[0],
					&rx_frame->data.raw[sizeof(rx_frame->data.raw) -
							    FRAME_HEADER_AND_SIZE_LENGTH - 1],
					FRAME_HEADER_AND_SIZE_LENGTH);
				rx_frame->total_bytes_read = FRAME_HEADER_AND_SIZE_LENGTH;
			}
			return -EAGAIN;
		}
		frame_start++;
	}

	if (frame_start != 0) {
		/* Shift frame to start of the buffer */
		frame_length = rx_frame->total_bytes_read - frame_start;
		memmove(&rx_frame->data.raw[0], &rx_frame->data.raw[frame_start], frame_length);
		rx_frame->total_bytes_read -= frame_start;
	}

	if (rx_frame->data.frame.body_len >= LD2410_MAX_FRAME_BODYLEN) {
		/* Length information is implausible, discard buffer*/
		rx_frame->total_bytes_read = 0;
		return -EBADMSG;
	}

	if (rx_frame->total_bytes_read <
	    FRAME_HEADER_AND_SIZE_LENGTH + rx_frame->data.frame.body_len + FRAME_FOOTER_SIZE) {
		return -EAGAIN;
	}

	if (frame_type == DATA_FRAME &&
	    memcmp(&rx_frame->data.frame.body[rx_frame->data.frame.body_len], DATA_FRAME_FOOTER,
		   FRAME_FOOTER_SIZE) != 0) {
		rx_frame->total_bytes_read = 0;
		return -EBADMSG;
	}

	if (frame_type == ACK_FRAME &&
	    memcmp(&rx_frame->data.frame.body[rx_frame->data.frame.body_len], CMD_FRAME_FOOTER,
		   FRAME_FOOTER_SIZE) != 0) {
		rx_frame->total_bytes_read = 0;

		return -EBADMSG;
	}

	return frame_type;
}

static void uart_tx_cb_handler(const struct device *dev)
{
	const struct ld2410_config *config = dev->config;
	struct ld2410_data *drv_data = dev->data;
	int sent = 0;
	uint8_t retries = 3;

	while (drv_data->tx_frame.bytes_remaining > 0) {
		sent = uart_fifo_fill(config->uart_dev, &drv_data->tx_frame.raw_data[sent],
				      drv_data->tx_frame.bytes_remaining);
		drv_data->tx_frame.bytes_remaining -= sent;
	}

	while (retries--) {
		if (uart_irq_tx_complete(config->uart_dev)) {
			uart_irq_tx_disable(config->uart_dev);
			k_sem_give(&drv_data->tx_sem);
			break;
		}
	}
}

static void uart_cb_handler(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct ld2410_data *drv_data = dev->data;
	size_t rx_available_space;
	int found_frame;

	if (!uart_dev || !uart_irq_update(uart_dev) || !uart_irq_is_pending(uart_dev)) {
		return;
	}

	if (uart_irq_tx_ready(uart_dev)) {
		uart_tx_cb_handler(dev);
	}

	rx_available_space = sizeof(struct ld2410_rx_frame) - drv_data->rx_frame.total_bytes_read;

	while (uart_irq_rx_ready(uart_dev) && rx_available_space > 0) {
		drv_data->rx_frame.total_bytes_read += uart_fifo_read(
			uart_dev, drv_data->rx_frame.data.raw[drv_data->rx_frame.total_bytes_read],
			rx_available_space);

		found_frame = find_rx_frame_start(&drv_data->rx_frame);
		if (found_frame > 0 && found_frame == drv_data->awaited_rx_frame) {
			LOG_HEXDUMP_DBG(drv_data->rx_frame.data,
					drv_data->rx_frame.total_bytes_read, "RX");
			k_sem_give(&drv_data->rx_sem);
			break;
		}

		rx_available_space =
			sizeof(drv_data->rx_frame.data) - drv_data->rx_frame.total_bytes_read;
	}
}

static void ld2410_uart_flush(const struct device *dev)
{
	uint8_t c;

	while (uart_fifo_read(dev, &c, 1) > 0) {
		continue;
	}
}

static int ld2410_tranceive_command(const struct device *dev, enum ld2410_command command,
				    uint8_t *data, uint16_t data_len)
{
	struct ld2410_data *drv_data = dev->data;
	struct ld2410_config *cfg = dev->config;
	int ret;

	if (LD2410_CMD_ID_SIZE + data_len >= LD2410_MAX_FRAME_BODYLEN) {
		return -EINVAL;
	}

	/* Make sure last command has been transferred */
	ret = k_sem_take(&drv_data->tx_sem, K_MSEC(CFG_LD2410_SERIAL_TIMEOUT));
	if (ret) {
		return ret;
	}

	drv_data->tx_frame.bytes_remaining =
		FRAME_HEADER_AND_SIZE_LENGTH + LD2410_CMD_ID_SIZE + data_len + FRAME_FOOTER_SIZE;
	drv_data->awaited_rx_frame = ACK_FRAME;

	memcpy(&drv_data->tx_frame.frame.header, CMD_FRAME_HEADER, FRAME_HEADER_SIZE);
	drv_data->tx_frame.frame.body_len = data_len + LD2410_CMD_ID_SIZE;
	sys_put_le16(command, &drv_data->tx_frame.frame.body[0]);
	memcpy(&drv_data->tx_frame.frame.body[LD2410_CMD_ID_SIZE], data, data_len);
	memcpy(&drv_data->tx_frame.frame.body[LD2410_CMD_ID_SIZE + data_len], CMD_FRAME_FOOTER,
	       FRAME_FOOTER_SIZE);

	k_sem_reset(&drv_data->rx_sem);

	uart_irq_tx_enable(cfg->uart_dev);
	uart_irq_rx_enable(cfg->uart_dev);

	ret = k_sem_take(&drv_data->rx_sem, K_MSEC(CFG_LD2410_SERIAL_TIMEOUT));
	if (ret) {
		LOG_DBG("Awaiting rx message timedout");
		return ret;
	}

	/* Verify command id is contained */
	if (sys_get_le16(drv_data->rx_frame.data.frame.body[0]) != (command | 0x0100)) {
		LOG_DBG("Message did not contain expected command|0x0100");
		return -EIO;
	}

	/* Check return value */
	if (sys_get_le16(drv_data->rx_frame.data.frame.body[LD2410_CMD_ID_SIZE])) {
		LOG_DBG("Non zero ack state");
		return -EIO;
	}

	return 0;
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
	static struct ld2410_data ld2410_data_##inst = {};                                         \
                                                                                                   \
	static const struct ld2410_config ld2410_config_##inst = {                                 \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst))};                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &ld2410_init, NULL, &ld2410_data_##inst,                       \
			      &ld2410_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,     \
			      &ld2410_api);

DT_INST_FOREACH_STATUS_OKAY(LD2410_DEFINE)
