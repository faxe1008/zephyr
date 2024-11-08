/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

const struct device* sensor = DEVICE_DT_GET(DT_NODELABEL(test_i2c_vl53l5cx));


int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    sensor_sample_fetch_chan(sensor, SENSOR_CHAN_DISTANCE);

	return 0;
}
