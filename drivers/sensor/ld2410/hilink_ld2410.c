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

#include <zephyr/drivers/sensor/ld2410.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LD2410, CONFIG_SENSOR_LOG_LEVEL);

#define CFG_LD2410_MAX_FRAME_SIZE 40
#define CFG_LD2410_GATE_COUNT	  9
#define CFG_LD2410_SERIAL_TIMEOUT 100

struct ld2410_cyclic_data {
	bool in_engineering_mode;

	enum ld2410_target_state state;
	uint16_t moving_target_distance;
	uint8_t moving_target_energy;
	uint16_t stationary_target_distance;
	uint8_t stationary_target_energy;
	uint8_t detection_distance;

	uint8_t max_moving_gate;
	uint8_t max_moving_energy;
	uint8_t max_stationary_gate;
	uint8_t max_stationary_energy;
	uint8_t moving_energy_per_gate[CFG_LD2410_GATE_COUNT];
	uint8_t stationary_energy_per_gate[CFG_LD2410_GATE_COUNT];
};

struct ld2410_firmware_version {
	uint8_t major;
	uint8_t minor;
	uint32_t bugfix;
};

struct ld2410_config {
	const struct device *uart_dev;
};

enum ld2410_parsing_state {
	FIND_HEADER = 0,
	RECEIVE_DATA_LENGTH,
	RECEIVE_DATA
};

struct ld2410_settings {
	uint8_t max_gate;
	uint8_t max_moving_gate;
	uint8_t max_stationary_gate;
	uint8_t moving_sensitivity[CFG_LD2410_GATE_COUNT];
	uint8_t stationary_sensitivity[CFG_LD2410_GATE_COUNT];
	uint16_t detection_time;
};

struct ld2410_data {
	struct ld2410_cyclic_data cyclic_data;
	struct ld2410_firmware_version firmware_version;
	struct ld2410_settings settings;
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

enum ld2410_rx_packet_type {
	UNKNOWN,
	COMMAND_RESPONSE,
	DATA_PACKET
};

struct ld2410_rx_command_response {
	enum ld2410_command command;
	bool ack;
};

struct ld2410_rx_packet {
	uint8_t buf[CFG_LD2410_MAX_FRAME_SIZE];
	size_t received_bytes;
	size_t expected_length;
	enum ld2410_parsing_state state;
	enum ld2410_rx_packet_type packet_type;

	union {
		struct ld2410_cyclic_data cyclic_data;
		struct ld2410_rx_command_response response;
		struct ld2410_settings settings;
		struct ld2410_firmware_version version;
	};
};

static const uint8_t data_header[4] = {0XF4, 0xF3, 0XF2, 0xF1};
static const uint8_t data_tail[4] = {0xF8, 0xF7, 0xF6, 0xF5};
static const uint8_t command_header[4] = {0XFD, 0xFC, 0XFB, 0xFA};
static const uint8_t command_tail[4] = {0x04, 0x03, 0x02, 0x01};

#define UPPER_BYTE(x) ((x & 0xFF00) >> 8)
#define LOWER_BYTE(x) (x & 0xFF)

static int ld2410_parse_data_frame(struct ld2410_rx_packet *rx_packet)
{
	uint8_t *data_buffer = rx_packet->buf;
	// tail not found
	if (memcmp(&data_buffer[rx_packet->expected_length], data_tail, sizeof(data_tail))) {
		LOG_DBG("Tail not in data frame");
		rx_packet->state = FIND_HEADER;
		return -EBADMSG;
	}
	rx_packet->packet_type = DATA_PACKET;

	// engineering mode active
	rx_packet->cyclic_data.in_engineering_mode = (data_buffer[0] == 0x01);

	// cyclic header 0xAA
	if (data_buffer[1] != 0xAA) {
		LOG_DBG("Failed cyclic header check");
		rx_packet->state = FIND_HEADER;
		return -EBADMSG;
	}

	rx_packet->cyclic_data.state = (enum ld2410_target_state)data_buffer[2];
	rx_packet->cyclic_data.moving_target_distance = sys_get_le16(&data_buffer[3]);
	rx_packet->cyclic_data.moving_target_energy = data_buffer[5];
	rx_packet->cyclic_data.stationary_target_distance = sys_get_le16(&data_buffer[6]);
	rx_packet->cyclic_data.stationary_target_energy = data_buffer[8];
	rx_packet->cyclic_data.detection_distance = sys_get_le16(&data_buffer[9]);

	if (rx_packet->cyclic_data.in_engineering_mode) {
		LOG_DBG("In engineering mode");
		rx_packet->cyclic_data.max_moving_gate = data_buffer[11];
		rx_packet->cyclic_data.max_stationary_gate = data_buffer[12];

		for (size_t gate = 0; gate < CFG_LD2410_GATE_COUNT; gate++) {
			rx_packet->cyclic_data.moving_energy_per_gate[gate] =
				data_buffer[13 + gate];
			rx_packet->cyclic_data.stationary_energy_per_gate[gate] =
				data_buffer[22 + gate];
		}

		rx_packet->cyclic_data.max_moving_energy = data_buffer[31];
		rx_packet->cyclic_data.max_stationary_energy = data_buffer[32];

		// 0x55 cyclicData tail and check (0x00)
		if (data_buffer[33] != 0x55 || data_buffer[34] != 0x00) {
			LOG_DBG("cyclic data: engineering mode: tail check failed");
			rx_packet->state = FIND_HEADER;
			return -EBADMSG;
		}
	} else {
		if (data_buffer[11] != 0x55 || data_buffer[12] != 0x00) {
			LOG_DBG("cyclic data: tail check failed");
			rx_packet->state = FIND_HEADER;
			return -EBADMSG;
		}
	}
	rx_packet->state = FIND_HEADER;
	return 0;
}

static int ld2410_parse_command_frame(struct ld2410_rx_packet *rx_packet)
{
	uint8_t *data_buffer = rx_packet->buf;

	if (memcmp(&data_buffer[rx_packet->expected_length], command_tail, sizeof(command_tail))) {
		LOG_DBG("Did not find tail in command frame");
		rx_packet->state = FIND_HEADER;
		return -EBADMSG;
	}

	rx_packet->response.command = sys_get_be16(&data_buffer[0]) - 1;
	rx_packet->response.ack = sys_get_le16(&data_buffer[2]) == 0;

	LOG_DBG("Command frame, id=%u, success=%i", rx_packet->response.command,
		rx_packet->response.ack);

	switch (rx_packet->response.command) {
	case READ_SETTINGS:
		LOG_DBG("Command frame read parameter");
		if (data_buffer[4] != 0xAA) {
			LOG_DBG("Did not find parameter header");
			return -EBADMSG;
		}
		rx_packet->settings.max_gate = data_buffer[5];
		rx_packet->settings.max_moving_gate = data_buffer[6];
		rx_packet->settings.max_stationary_gate = data_buffer[7];

		for (size_t gate = 0; gate < CFG_LD2410_GATE_COUNT; gate++) {
			rx_packet->settings.moving_sensitivity[gate] = data_buffer[8 + gate];
			rx_packet->settings.stationary_sensitivity[gate] = data_buffer[17 + gate];
		}
		rx_packet->settings.detection_time = sys_get_le16(&data_buffer[26]);

	case READ_FIRMWARE_VERSION:
		LOG_DBG("Command frame read firmware version");
		rx_packet->version.minor = data_buffer[6];
		rx_packet->version.major = data_buffer[7];
		rx_packet->version.bugfix = sys_get_le32(&data_buffer[8]);
		break;
	}
	rx_packet->state = FIND_HEADER;
	return 0;
}

static int ld2410_receive_data(const struct ld2410_config *cfg, struct ld2410_rx_packet *rx_packet)
{
	uint8_t *data_buffer = rx_packet->buf;
	unsigned char read_byte = 0;
	while (uart_poll_in(cfg->uart_dev, &read_byte) == 0) {
		switch (rx_packet->state) {
		case FIND_HEADER:
			memmove(&data_buffer[0], &data_buffer[1], sizeof(data_header) - 1);
			data_buffer[3] = read_byte;

			// check for data header
			if (!memcmp(data_buffer, data_header, sizeof(data_header))) {
				LOG_DBG("FIND_HEADER: Found data frame header");
				rx_packet->packet_type = DATA_PACKET;
				rx_packet->state = RECEIVE_DATA_LENGTH;
				rx_packet->received_bytes = 0;
			}

			if (!memcmp(data_buffer, command_header, sizeof(command_header))) {
				LOG_DBG("FIND_HEADER: Found command frame header");
				rx_packet->packet_type = COMMAND_RESPONSE;
				rx_packet->state = RECEIVE_DATA_LENGTH;
				rx_packet->received_bytes = 0;
			}
			break;
		case RECEIVE_DATA_LENGTH:
			data_buffer[rx_packet->received_bytes++] = read_byte;
			if (rx_packet->received_bytes >= 2) {
				rx_packet->expected_length = sys_get_le16(&data_buffer[0]);
				LOG_DBG("RECEIVE_DATA_LENGTH: %u", rx_packet->expected_length);
				if (rx_packet->expected_length > CFG_LD2410_MAX_FRAME_SIZE) {
					rx_packet->state = FIND_HEADER;
					return 0;
				}
				rx_packet->state = RECEIVE_DATA;
				rx_packet->received_bytes = 0;
			}
			break;
		case RECEIVE_DATA:
			data_buffer[rx_packet->received_bytes++] = read_byte;
			if (rx_packet->received_bytes ==
			    rx_packet->expected_length + sizeof(data_tail)) {
				if (rx_packet->packet_type == DATA_PACKET) {
					return ld2410_parse_data_frame(rx_packet);
				} else {
					return ld2410_parse_command_frame(rx_packet);
				}
			}
		}
	}
	return -EAGAIN;
}

static int ld2410_send_request(const struct ld2410_config *cfg, struct ld2410_rx_packet *rx_packet,
			       enum ld2410_command command, const uint8_t *cmd_data, size_t len)
{
	size_t i;
	// send header
	for (i = 0; i < sizeof(command_header); i++) {
		uart_poll_out(cfg->uart_dev, command_header[i]);
	}

	// send frame data length
	uint16_t frame_data_len = sizeof(uint16_t) + len; // TODO: error if > 2 bytes
	uart_poll_out(cfg->uart_dev, LOWER_BYTE(frame_data_len));
	uart_poll_out(cfg->uart_dev, UPPER_BYTE(frame_data_len));

	// send command
	uart_poll_out(cfg->uart_dev, UPPER_BYTE(command));
	uart_poll_out(cfg->uart_dev, LOWER_BYTE(command));

	// send frame data
	for (i = 0; i < len; i++) {
		uart_poll_out(cfg->uart_dev, cmd_data[i]);
	}

	// send tail
	for (i = 0; i < sizeof(command_tail); i++) {
		uart_poll_out(cfg->uart_dev, command_tail[i]);
	}

	int64_t timeout_time = k_uptime_get() + CFG_LD2410_SERIAL_TIMEOUT;
	int response = 0;
	while (k_uptime_get() < timeout_time) {
		response = ld2410_receive_data(cfg, rx_packet);
		// LOG_DBG("res %i, ty %i, cmd %u", response, rx_packet->packet_type,
		// rx_packet->response.command);
		if (response == 0 && rx_packet->packet_type == COMMAND_RESPONSE &&
		    rx_packet->response.command == command) {
			return rx_packet->response.ack ? 0 : -EIO;
		}
	}
	return -ETIMEDOUT;
}

static int ld2410_enable_config_mode(struct ld2410_config *cfg, struct ld2410_rx_packet *data)
{
	LOG_DBG("Enable config mode");
	uint8_t payload[2] = {0x01, 0x00};
	struct ld2410_rx_packet rx_packet = {0};
	return ld2410_send_request(cfg, &rx_packet, ENTER_CONFIG_MODE, payload, sizeof(payload));
}

static int ld2410_disable_config_mode(struct ld2410_config *cfg, struct ld2410_rx_packet *data)
{
	LOG_DBG("Leave config mode");
	struct ld2410_rx_packet rx_packet = {0};
	return ld2410_send_request(cfg, &rx_packet, LEAVE_CONFIG_MODE, NULL, 0);
}

static int ld2410_send_command(struct ld2410_config *cfg, struct ld2410_rx_packet *rx_packet,
			       enum ld2410_command command, const uint8_t *cmd_data, size_t len)
{
	if (!ld2410_enable_config_mode(cfg, rx_packet)) {
		if (!ld2410_send_request(cfg, rx_packet, command, cmd_data, len)) {
			if (command == RESTART) {
				return 0;
			}
			return ld2410_disable_config_mode(cfg, rx_packet);
		}
		ld2410_disable_config_mode(cfg, rx_packet);
	}
	return -EIO;
}

static int ld2410_set_max_distance_and_duration(struct ld2410_config *cfg, uint8_t max_moving_range,
						uint8_t max_stationary_range, uint16_t duration)
{
	LOG_DBG("Set max distance and duration");
	uint8_t payload[18] = {0x00,
			       0x00,
			       max_moving_range,
			       0x00,
			       0x00,
			       0x00,
			       0x01,
			       0x00,
			       max_stationary_range,
			       0x00,
			       0x00,
			       0x00,
			       0x02,
			       0x00,
			       LOWER_BYTE(duration),
			       UPPER_BYTE(duration),
			       0x00,
			       0x00};
	struct ld2410_rx_packet rx_packet = {0};

	return ld2410_send_command(cfg, &rx_packet, SET_MAX_DISTANCE_AND_DURATION, payload,
				   sizeof(payload));
}

static int ld2410_read_settings(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("read parameter");
	struct ld2410_rx_packet rx_packet = {0};
	int rc;

	rc = ld2410_send_command(cfg, &rx_packet, READ_SETTINGS, NULL, 0);
	if (!rc) {
		memcpy(&data->settings, &rx_packet.settings, sizeof(struct ld2410_settings));
	}
	return rc;
}

static int ld2410_enter_engineering_mode(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("enter engineering mode");
	struct ld2410_rx_packet rx_packet = {0};
	int rc;
	rc = ld2410_send_command(cfg, &rx_packet, ENTER_ENGINEERING_MODE, NULL, 0);

	if (!rc) {
		data->cyclic_data.in_engineering_mode = true;
	}
	return rc;
}

static int ld2410_leave_engineering_mode(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("leave engineering mode");
	struct ld2410_rx_packet rx_packet = {0};
	int rc;

	rc = ld2410_send_command(cfg, &rx_packet, LEAVE_CONFIG_MODE, NULL, 0);
	if (!rc) {
		data->cyclic_data.in_engineering_mode = false;
	}
	return rc;
}

static int ld2410_set_gate_sensitivity_config(struct ld2410_config *cfg, struct ld2410_data *data,
					      uint8_t gate, uint8_t moving_sensitivity,
					      uint8_t stationary_sensitivity)
{
	LOG_DBG("Setting gate sensitivity conf");
	uint8_t payload[18] = {0x00, 0x00, gate,
			       0x00, 0x00, 0x00,
			       0x01, 0x00, moving_sensitivity,
			       0x00, 0x00, 0x00,
			       0x02, 0x00, stationary_sensitivity,
			       0x00, 0x00, 0x00};
	struct ld2410_rx_packet rx_packet = {0};

	return ld2410_send_command(cfg, &rx_packet, SET_GATE_SENSITIVITY_CONFIG, payload,
				   sizeof(payload));
}

static int ld2410_set_distance_resolution(struct ld2410_config *cfg, uint8_t resolution)
{
	if (resolution != 75 || resolution != 20) {
		return -EINVAL;
	}
	uint8_t payload[2] = {0x01, 0x00};
	struct ld2410_rx_packet rx_packet = {0};
	return ld2410_send_command(cfg, &rx_packet, SET_DISTANCE_RESOLUTION, payload,
				   sizeof(payload));
}

static int ld2410_factory_reset(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("perform factory reset");
	struct ld2410_rx_packet rx_packet = {0};
	return ld2410_send_command(cfg, &rx_packet, FACTORY_RESET, NULL, 0);
}

static int ld2410_restart(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("perform restart");
	struct ld2410_rx_packet rx_packet = {0};
	return ld2410_send_command(cfg, &rx_packet, RESTART, NULL, 0);
}

static int ld2410_read_firmware_version(struct ld2410_config *cfg, struct ld2410_data *data)
{
	struct ld2410_rx_packet rx_packet = {0};
	int rc;

	rc = ld2410_send_command(cfg, &rx_packet, READ_FIRMWARE_VERSION, NULL, 0);
	if (!rc) {
		memcpy(&data->firmware_version, &rx_packet.version,
		       sizeof(struct ld2410_firmware_version));
	}
	return rc;
}

int ld2410_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		    const struct sensor_value *val)
{

	struct ld2410_config *cfg = dev->config;
	struct ld2410_data *drv_data = dev->data;

	if (attr == SENSOR_ATTR_LD2410_ENGINEERING_MODE) {
		if (val->val1) {
			return ld2410_enter_engineering_mode(cfg, drv_data);
		} else {
			return ld2410_leave_engineering_mode(cfg, drv_data);
		}
	} else if (attr == SENSOR_ATTR_LD2410_DISTANCE_RESOLUTION) {
		return ld2410_set_distance_resolution(cfg, val->val1);
	} else if (attr >= SENSOR_ATTR_LD2410_MOVING_SENSITIVITY_GATE_0 &&
		   attr <= SENSOR_ATTR_LD2410_MOVING_SENSITIVITY_GATE_8) {
		uint8_t gate = attr - SENSOR_ATTR_LD2410_MOVING_SENSITIVITY_GATE_0;
		drv_data->settings.moving_sensitivity[gate] = val->val1;
		return ld2410_set_gate_sensitivity_config(
			cfg, drv_data, gate, drv_data->settings.moving_sensitivity[gate],
			drv_data->settings.stationary_sensitivity[gate]);
	} else if (attr >= SENSOR_ATTR_LD2410_STATIONARY_SENSITIVITY_GATE_0 &&
		   attr <= SENSOR_ATTR_LD2410_STATIONARY_SENSITIVITY_GATE_8) {
		uint8_t gate = attr - SENSOR_ATTR_LD2410_STATIONARY_SENSITIVITY_GATE_0;
		drv_data->settings.stationary_sensitivity[gate] = val->val1;
		return ld2410_set_gate_sensitivity_config(
			cfg, drv_data, gate, drv_data->settings.moving_sensitivity[gate],
			drv_data->settings.stationary_sensitivity[gate]);
	}

	return -ENOTSUP;
}

int ld2410_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		    struct sensor_value *val)
{
	const struct ld2410_data *drv_data = dev->data;

	if (attr == SENSOR_ATTR_LD2410_ENGINEERING_MODE) {
		val->val1 = drv_data->cyclic_data.in_engineering_mode;
		val->val2 = 0;
	} else if (attr >= SENSOR_ATTR_LD2410_MOVING_SENSITIVITY_GATE_0 &&
		   attr <= SENSOR_ATTR_LD2410_MOVING_SENSITIVITY_GATE_8) {
		uint8_t gate = attr - SENSOR_ATTR_LD2410_MOVING_SENSITIVITY_GATE_0;
		val->val1 = drv_data->settings.moving_sensitivity[gate];
		val->val2 = 0;
	} else if (attr >= SENSOR_ATTR_LD2410_STATIONARY_SENSITIVITY_GATE_0 &&
		   attr <= SENSOR_ATTR_LD2410_STATIONARY_SENSITIVITY_GATE_8) {
		uint8_t gate = attr - SENSOR_ATTR_LD2410_STATIONARY_SENSITIVITY_GATE_0;
		val->val1 = drv_data->settings.stationary_sensitivity[gate];
		val->val2 = 0;
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static int ld2410_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ld2410_data *drv_data = dev->data;
	struct ld2410_config *cfg = dev->config;

	int64_t timeout_time = k_uptime_get() + CFG_LD2410_SERIAL_TIMEOUT;
	int response = 0;
	struct ld2410_rx_packet rx_packet = {0};
	while (k_uptime_get() < timeout_time) {
		response = ld2410_receive_data(cfg, &rx_packet);
		if (rx_packet.packet_type == DATA_PACKET) {
			memcpy(&drv_data->cyclic_data, &rx_packet.cyclic_data,
			       sizeof(struct ld2410_cyclic_data));
			return 0;
		}
	}
	return -ETIMEDOUT;
}

static int ld2410_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ld2410_data *drv_data = dev->data;
	val->val2 = 0;

	if (chan == SENSOR_CHAN_LD2410_MOVING_TARGET_DISTANCE) {
		val->val1 = drv_data->cyclic_data.moving_target_distance;
	} else if (chan == SENSOR_CHAN_LD2410_MOVING_TARGET_ENERGY) {
		val->val1 = drv_data->cyclic_data.moving_target_energy;
	} else if (chan == SENSOR_CHAN_LD2410_STATIONARY_TARGET_DISTANCE) {
		val->val1 = drv_data->cyclic_data.stationary_target_distance;
	} else if (chan == SENSOR_CHAN_LD2410_STATIONARY_TARGET_ENERGY) {
		val->val1 = drv_data->cyclic_data.stationary_target_energy;
	} else if(chan == SENSOR_CHAN_LD2410_TARGET_TYPE){
		val->val1 = drv_data->cyclic_data.state;
	} else if (chan >= SENSOR_CHAN_LD2410_MOVING_ENERGY_GATE_0 &&
		   chan <= SENSOR_CHAN_LD2410_MOVING_ENERGY_GATE_8) {
		uint8_t gate = chan - SENSOR_CHAN_LD2410_MOVING_ENERGY_GATE_0;
		val->val1 = drv_data->cyclic_data.moving_energy_per_gate[gate];
	} else if (chan >= SENSOR_CHAN_LD2410_STATIONARY_ENERGY_GATE_0 &&
		   chan <= SENSOR_CHAN_LD2410_STATIONARY_ENERGY_GATE_8) {
		uint8_t gate = chan - SENSOR_CHAN_LD2410_STATIONARY_ENERGY_GATE_0;
		val->val1 = drv_data->cyclic_data.stationary_energy_per_gate[gate];
	} else {
		return -ENOTSUP;
	}
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

	return 0;
}

#define LD2410_DEFINE(inst)                                                                        \
	static struct ld2410_data ld2410_data_##inst;                                              \
                                                                                                   \
	static const struct ld2410_config ld2410_config_##inst = {                                 \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst))};                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &ld2410_init, NULL, &ld2410_data_##inst,                       \
			      &ld2410_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,     \
			      &ld2410_api);

DT_INST_FOREACH_STATUS_OKAY(LD2410_DEFINE)
