/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for MAX3510X Time-To-Digital Converter
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX3510X_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX3510X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>


enum sensor_channel_max3510x {
	SENSOR_CHAN_MAX3510X_TOF_UP = SENSOR_CHAN_PRIV_START, /* in nanoseconds */
    SENSOR_CHAN_MAX3510X_TOF_DOWN,
    SENSOR_CHAN_MAX3510X_TOF_DIFF,
};


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX3510X_H_ */