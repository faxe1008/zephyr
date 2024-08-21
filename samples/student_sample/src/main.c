#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

/* Configuration for the GPIO button using DT_ALIAS */
#define SW0_NODE    DT_ALIAS(sw0)

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;
static atomic_t shared_counter = ATOMIC_INIT(0);  // Define the shared counter as an atomic variable

/* ISR for button press */
void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    atomic_inc(&shared_counter);  // Atomically increment the counter
    printk("ISR incremented counter to:         %ld\n", atomic_get(&shared_counter));
}

/* Thread to periodically increment and print the counter value */
void increment_thread_entry(void)
{
    while (1) {
        atomic_inc(&shared_counter);  // Atomically increment the counter
        printk("Main thread incremented counter to: %ld\n", atomic_get(&shared_counter));
        k_sleep(K_SECONDS(1));
    }
}

int main(void)
{
    int ret;

    if (!gpio_is_ready_dt(&button)) {
        printk("Error: button device %s is not ready\n", button.port->name);
        return 1;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, button.port->name, button.pin);
        return 1;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on %s pin %d\n",
               ret, button.port->name, button.pin);
        return 1;
    }

    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    printk("Set up button at %s pin %d\n", button.port->name, button.pin);

    while (1) {
        k_sleep(K_FOREVER); /* Main thread does nothing, waits forever */
    }
}

/* Define the increment thread */
K_THREAD_DEFINE(increment_thread, 1024, increment_thread_entry, NULL, NULL, NULL, 3, 0, 0);
