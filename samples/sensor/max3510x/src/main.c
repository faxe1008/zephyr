#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>


int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(maxim_max3510x);
	struct sensor_value val;

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

    return 0;
}
