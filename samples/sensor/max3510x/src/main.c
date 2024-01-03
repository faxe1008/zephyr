#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/max3510x.h>


int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(maxim_max3510x);

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}
    printk("Sensor int done\n");

    sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAX3510X_TOF_DIFF);

	struct sensor_value tof_diff;
    sensor_channel_get(dev, SENSOR_CHAN_MAX3510X_TOF_DIFF, &tof_diff);

    printk("TOF Diff, int: %u, frac: %u\n", tof_diff.val1, tof_diff.val2);

    return 0;
}
