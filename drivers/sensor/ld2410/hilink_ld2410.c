/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hilink_ld2410

#include <errno.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

LOG_MODULE_REGISTER(LD2410, CONFIG_SENSOR_LOG_LEVEL);

#include <zephyr/drivers/sensor/ld2410.h>

/* wait serial output with 1000ms timeout */
#define CFG_LD2410_MAX_FRAME_SIZE 40
#define CFG_LD2410_GATE_COUNT	  9
#define CFG_LD2410_SERIAL_TIMEOUT 100

enum ld2410_target_state {
	NO_TAGET = 0,
	MOVING_TARGET,
	STATIONARY_TARGET,
	MOVING_AND_STATIONARY_TARGET
};

struct ld2410_cyclic_data {
	enum ld2410_target_state state;
	uint16_t moving_target_distance;
	uint8_t moving_target_energy;
	uint16_t stationary_target_distance;
	uint8_t stationary_target_energy;
	uint8_t detection_distance;
};

struct ld2410_engineering_data {
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
	bool in_engineering_mode;

	uint8_t max_gate;
	uint8_t max_moving_gate;
	uint8_t max_stationary_gate;
	uint8_t moving_sensitivity[CFG_LD2410_GATE_COUNT];
	uint8_t stationary_sensitivity[CFG_LD2410_GATE_COUNT];
	uint16_t detection_time;
};

enum ld2410_parsing_state {
	FIND_HEADER = 0,
	RECEIVE_DATA_LENGTH,
	RECEIVE_DATA
};

struct ld2410_scratch_data {
	bool data_payload;
	enum ld2410_parsing_state state;
	size_t data_length;
	size_t received_bytes;
	uint8_t rx_buffer[CFG_LD2410_MAX_FRAME_SIZE];
};

struct ld2410_data {
	struct ld2410_scratch_data scratch_data;
	struct ld2410_cyclic_data cyclic_data;
	struct ld2410_engineering_data engineering_data;
	struct ld2410_firmware_version firmware_version;
};

enum ld2410_command {
	ENTER_CONFIG_MODE = 0xFF00,
	LEAVE_CONFIG_MODE = 0xFE00,
	SET_MAX_DISTANCE_AND_DURATION = 0x6000,
	READ_PARAMETER = 0x6100,
	ENTER_ENGINEERING_MODE = 0x6200,
	LEAVE_ENGINEERING_MODE = 0x6300,
	SET_GATE_SENSITIVITY_CONFIG = 0x6400,
	READ_FIRMWARE_VERSION = 0xA000,
	SET_BAUDRATE = 0xA100,
	FACTORY_RESET = 0xA200,
	RESTART = 0xA300
};

typedef uint32_t ld2410_response;
#define RESPONSE_CMD_ID(response)    (response & 0xFFFF)
#define RESPONSE_SUCCESS(response)   ((response & 0xFF0000) >> 16)
#define TO_RESPONSE(cmd_id, success) ((ld2410_response)cmd_id | ((ld2410_response)success << 16))

static const uint8_t data_header[4] = {0XF4, 0xF3, 0XF2, 0xF1};
static const uint8_t data_tail[4] = {0xF8, 0xF7, 0xF6, 0xF5};
static const uint8_t command_header[4] = {0XFD, 0xFC, 0XFB, 0xFA};
static const uint8_t command_tail[4] = {0x04, 0x03, 0x02, 0x01};

#define UPPER_BYTE(x)	     ((x & 0xFF00) >> 8)
#define LOWER_BYTE(x)	     (x & 0xFF)
#define BYTES_TO_SHORT(x, y) (x | (y << 8))

static ld2410_response ld2410_parse_data_frame(struct ld2410_config *cfg, struct ld2410_data *data)
{
	uint8_t *data_buffer = data->scratch_data.rx_buffer;

	// tail not found
	if (memcmp(&data_buffer[data->scratch_data.data_length], data_tail, sizeof(data_tail))) {
		LOG_DBG("Tail not in data frame");
		data->scratch_data.state = FIND_HEADER;
		return 0;
	}

	// engineering mode active
	cfg->in_engineering_mode = (data_buffer[0] == 0x01);

	// cyclic header 0xAA
	if (data_buffer[1] != 0xAA) {
		LOG_DBG("Failed cyclic header check");
		data->scratch_data.state = FIND_HEADER;
		return 0;
	}

	data->cyclic_data.state = (enum ld2410_target_state)data_buffer[2];
	data->cyclic_data.moving_target_distance = BYTES_TO_SHORT(data_buffer[3], data_buffer[4]);
	data->cyclic_data.moving_target_energy = data_buffer[5];
	data->cyclic_data.stationary_target_distance =
		BYTES_TO_SHORT(data_buffer[6], data_buffer[7]);
	data->cyclic_data.stationary_target_energy = data_buffer[8];
	data->cyclic_data.detection_distance = BYTES_TO_SHORT(data_buffer[9], data_buffer[10]);

	if (cfg->in_engineering_mode) {
		LOG_DBG("In engineering mode");
		data->engineering_data.max_moving_gate = data_buffer[11];
		data->engineering_data.max_stationary_gate = data_buffer[12];

		for (size_t gate = 0; gate < CFG_LD2410_GATE_COUNT; gate++) {
			data->engineering_data.moving_energy_per_gate[gate] =
				data_buffer[13 + gate];
			data->engineering_data.stationary_energy_per_gate[gate] =
				data_buffer[22 + gate];
		}

		data->engineering_data.max_moving_energy = data_buffer[31];
		data->engineering_data.max_stationary_energy = data_buffer[32];

		// 0x55 cyclicData tail and check (0x00)
		if (data_buffer[33] == 0x55 && data_buffer[34] == 0x00) {
			LOG_DBG("cyclic data: engineering mode: tail check failed");
			data->scratch_data.state = FIND_HEADER;
			return 1;
		}
	} else {
		LOG_DBG("cyclic data: tail check failed");
		memset(&data->engineering_data, 0, sizeof(data->engineering_data));
		if (data_buffer[11] == 0x55 && data_buffer[12] == 0x00) {
			data->scratch_data.state = FIND_HEADER;
			return 1;
		}
	}
	data->scratch_data.state = FIND_HEADER;
	return 0;
}

static ld2410_response ld2410_parse_command_frame(struct ld2410_config *cfg,
						  struct ld2410_data *data)
{
	uint8_t *data_buffer = data->scratch_data.rx_buffer;

	if (memcmp(&data_buffer[data->scratch_data.data_length], command_tail,
		   sizeof(command_tail))) {
		LOG_DBG("Did not find tail in command frame");
		data->scratch_data.state = FIND_HEADER;
		return 0;
	}
	uint16_t cmd_id = BYTES_TO_SHORT(data_buffer[1], data_buffer[0]) - 1;
	bool failed = BYTES_TO_SHORT(data_buffer[2], data_buffer[3]) != 0;
	LOG_DBG("Command frame, id=%u, failed=%i", cmd_id, failed);

	switch (cmd_id) {
	case READ_PARAMETER:
		LOG_DBG("Command frame read parameter");
		if (data_buffer[4] != 0xAA) {
			LOG_DBG("Did not find parameter header");
			return 0;
		}
		cfg->max_gate = data_buffer[5];
		cfg->max_moving_gate = data_buffer[6];
		cfg->max_stationary_gate = data_buffer[7];

		for (size_t gate = 0; gate < CFG_LD2410_GATE_COUNT; gate++) {
			cfg->moving_sensitivity[gate] = data_buffer[8 + gate];
			cfg->stationary_sensitivity[gate] = data_buffer[17 + gate];
		}
		cfg->detection_time = BYTES_TO_SHORT(data_buffer[26], data_buffer[27]);

	case READ_FIRMWARE_VERSION:
		LOG_DBG("Command frame read firmware version");
		data->firmware_version.minor = data_buffer[6];
		data->firmware_version.major = data_buffer[7];
		data->firmware_version.bugfix =
			(uint32_t)data_buffer[8] | ((uint32_t)data_buffer[9] << 8) |
			((uint32_t)data_buffer[10] << 16) | ((uint32_t)data_buffer[11] << 24);
		break;
	}
	data->scratch_data.state = FIND_HEADER;
	return TO_RESPONSE(cmd_id, failed);
}

static ld2410_response ld2410_parse(struct ld2410_config *cfg, struct ld2410_data *data)
{
	uint8_t *data_buffer = data->scratch_data.rx_buffer;
	unsigned char read_byte = 0;
	while (uart_poll_in(cfg->uart_dev, &read_byte) == 0) {
		LOG_DBG("Received byte: %u", read_byte);
		switch (data->scratch_data.state) {
		case FIND_HEADER:
			memmove(&data_buffer[0], &data_buffer[1], sizeof(data_header) - 1);
			data_buffer[3] = read_byte;

			// check for data header
			if (!memcmp(data_buffer, data_header, sizeof(data_header))) {
				LOG_DBG("FIND_HEADER: Found data frame header");
				data->scratch_data.data_payload = true;
				data->scratch_data.state = RECEIVE_DATA_LENGTH;
				data->scratch_data.received_bytes = 0;
			}

			if (!memcmp(data_buffer, command_header, sizeof(command_header))) {
				LOG_DBG("FIND_HEADER: Found command frame header");
				data->scratch_data.data_payload = false;
				data->scratch_data.state = RECEIVE_DATA_LENGTH;
				data->scratch_data.received_bytes = 0;
			}
			break;
		case RECEIVE_DATA_LENGTH:
			data_buffer[data->scratch_data.received_bytes++] = read_byte;
			if (data->scratch_data.received_bytes >= 2) {
				data->scratch_data.data_length =
					BYTES_TO_SHORT(data_buffer[0], data_buffer[1]);
				LOG_DBG("RECEIVE_DATA_LENGTH: %u", data->scratch_data.data_length);
				if (data->scratch_data.data_length > CFG_LD2410_MAX_FRAME_SIZE) {
					data->scratch_data.state = FIND_HEADER;
					return 0;
				}
				data->scratch_data.state = RECEIVE_DATA;
				data->scratch_data.received_bytes = 0;
			}
			break;
		case RECEIVE_DATA:
			data_buffer[data->scratch_data.received_bytes++] = read_byte;
			LOG_DBG("RECEIVE_DATA: current len: %u", data->scratch_data.received_bytes);
			if (data->scratch_data.received_bytes ==
			    data->scratch_data.data_length + sizeof(data_tail)) {
				if (data->scratch_data.data_payload) {
					return ld2410_parse_data_frame(cfg, data);
				} else {
					return ld2410_parse_command_frame(cfg, data);
				}
			}
		}
	}
	return 0;
}

static int ld2410_send_request(struct ld2410_config *cfg, struct ld2410_data *data,
			       enum ld2410_command command, const uint8_t *cmd_data, size_t len)
{
	size_t i;
	// send header
	for (i = 0; i < sizeof(command_header); i++) {
		uart_poll_out(cfg->uart_dev, command_header[i]);
	}

	// send frame data length
	size_t frame_data_len = sizeof(uint16_t) + len; // TODO: error if > 2 bytes
	uart_poll_out(cfg->uart_dev, UPPER_BYTE(frame_data_len));
	uart_poll_out(cfg->uart_dev, LOWER_BYTE(frame_data_len));

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
	while (k_uptime_get() < timeout_time) {
		LOG_DBG("Trying to parse data");
		ld2410_response response = ld2410_parse(cfg, data);
		if (RESPONSE_CMD_ID(response) == command) {
			return RESPONSE_SUCCESS(response) ? -EINVAL
							  : 0; // TODO: clean this error code
		}
	}
	return -ETIMEDOUT;
}

static int ld2410_enable_config_mode(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("Enable config mode");
	uint8_t payload[2] = {0x01, 0x00};
	return ld2410_send_request(cfg, data, ENTER_CONFIG_MODE, payload, sizeof(payload));
}

static int ld2410_disable_config_mode(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("Leave config mode");
	return ld2410_send_request(cfg, data, LEAVE_CONFIG_MODE, NULL, 0);
}

static int ld2410_send_command(struct ld2410_config *cfg, struct ld2410_data *data,
			       enum ld2410_command command, const uint8_t *cmd_data, size_t len)
{
	LOG_DBG("Sending command %i", (int)command);

	if (!ld2410_enable_config_mode(cfg, data)) {
		if (!ld2410_send_request(cfg, data, command, cmd_data, len)) {
			if (command == RESTART) {
				return 0;
			}
			return ld2410_disable_config_mode(cfg, data);
		}
		ld2410_disable_config_mode(cfg, data);
	}
	return -EIO;
}

static int ld2410_set_max_distance_and_duration(struct ld2410_config *cfg, struct ld2410_data *data,
						uint8_t max_moving_range,
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
	return ld2410_send_command(cfg, data, SET_MAX_DISTANCE_AND_DURATION, payload,
				   sizeof(payload));
}

static int ld2410_read_parameter(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("read parameter");
	return ld2410_send_command(cfg, data, READ_PARAMETER, NULL, 0);
}

static int ld2410_enter_engineering_mode(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("enter engineering mode");
	return ld2410_send_command(cfg, data, ENTER_ENGINEERING_MODE, NULL, 0);
}

static int ld2410_leave_engineering_mode(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("leave engineering mode");
	return ld2410_send_command(cfg, data, LEAVE_CONFIG_MODE, NULL, 0);
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
	return ld2410_send_command(cfg, data, SET_GATE_SENSITIVITY_CONFIG, payload,
				   sizeof(payload));
}

static int ld2410_factory_reset(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("perform factory reset");
	return ld2410_send_command(cfg, data, FACTORY_RESET, NULL, 0);
}

static int ld2410_restart(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("perform restart");
	return ld2410_send_command(cfg, data, RESTART, NULL, 0);
}

static int ld2410_read_firmware_version(struct ld2410_config *cfg, struct ld2410_data *data)
{
	LOG_DBG("request firmware_version");
	return ld2410_send_command(cfg, data, READ_FIRMWARE_VERSION, NULL, 0);
}

int ld2410_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		    const struct sensor_value *val)
{

	struct ld2410_config *cfg = dev->config;
	struct ld2410_data *drv_data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_LD2410_ENGINEERING_MODE:
		if (val->val1) {
			return ld2410_enter_engineering_mode(cfg, drv_data);
		} else {
			return ld2410_leave_engineering_mode(cfg, drv_data);
		}
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

int ld2410_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr,
		    struct sensor_value *val)
{
	const struct ld2410_config *cfg = dev->config;

	switch (attr) {
	case SENSOR_ATTR_LD2410_ENGINEERING_MODE:
		val->val1 = cfg->in_engineering_mode;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int ld2410_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ld2410_data *drv_data = dev->data;
	struct ld2410_config *cfg = dev->config;
	return ld2410_read_parameter(cfg, drv_data);
}

static int ld2410_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ld2410_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_LD2410_MT_DISTANCE:
		val->val1 = drv_data->cyclic_data.moving_target_distance;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_LD2410_MT_ENERGY:
		val->val1 = drv_data->cyclic_data.moving_target_energy;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_LD2410_ST_DISTANCE:
		val->val1 = drv_data->cyclic_data.stationary_target_distance;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_LD2410_ST_ENERGY:
		val->val1 = drv_data->cyclic_data.stationary_target_energy;
		val->val2 = 0;
		break;
	default:
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

	if (!ld2410_read_firmware_version(cfg, data)) {
		LOG_DBG("Firmware version %u.%u.%u", data->firmware_version.major,
			data->firmware_version.minor, data->firmware_version.bugfix);
	}

	if (cfg->in_engineering_mode) {
		ld2410_enter_engineering_mode(cfg, data);
	}

	return 0;
}

#define LD2410_DEFINE(inst)                                                                                             \
	static struct ld2410_data ld2410_data_##inst;                                                                   \
	\ 
                                                                                                    static const struct \
		ld2410_config ld2410_config_##inst = {                                                                  \
			.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                                   \
			.in_engineering_mode = IS_ENABLED(DT_INST_PROP(inst, engineering_mode))};                       \
                                                                                                                        \
	DEVICE_DT_INST_DEFINE(inst, &ld2410_init, NULL, &ld2410_data_##inst,                                            \
			      &ld2410_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                          \
			      &ld2410_api);

DT_INST_FOREACH_STATUS_OKAY(LD2410_DEFINE)
