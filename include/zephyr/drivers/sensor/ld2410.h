/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for LD2410 Radar Sensor
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_LD2410_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_LD2410_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum sensor_channel_ld2410 {
    //
	SENSOR_CHAN_LD2410_MT_DISTANCE = SENSOR_CHAN_PRIV_START,
    SENSOR_CHAN_LD2410_MT_ENERGY,
    SENSOR_CHAN_LD2410_ST_DISTANCE,
    SENSOR_CHAN_LD2410_ST_ENERGY
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_LD2410_H_ */
