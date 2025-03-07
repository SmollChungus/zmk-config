#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zmk/activity.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led_power, LOG_LEVEL_INF);

#define LED_POWER_NODE DT_ALIAS(led_power)
#define TIMEOUT_MS CONFIG_ZMK_ACTIVITY_TRIGGER_TIME_MS

static const struct gpio_dt_spec led_power = GPIO_DT_SPEC_GET(LED_POWER_NODE, gpios);
static struct k_work_delayable timeout_work;

static void power_timeout_handler(struct k_work *work)
{
    LOG_DBG("LED power timeout - turning off");
    gpio_pin_set_dt(&led_power, 0);  // Turn off LED power
}

static void activity_callback(void)
{
    LOG_DBG("Activity detected - turning on LED power");
    gpio_pin_set_dt(&led_power, 1);  // Turn on LED power
    k_work_reschedule(&timeout_work, K_MSEC(TIMEOUT_MS));
}

void led_power_init(void)
{
    int ret;

    if (!device_is_ready(led_power.port)) {
        LOG_ERR("LED power control device not ready");
        return;
    }

    ret = gpio_pin_configure_dt(&led_power, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED power control pin (err %d)", ret);
        return;
    }

    k_work_init_delayable(&timeout_work, power_timeout_handler);
    
    // Turn on LED power initially
    gpio_pin_set_dt(&led_power, 1);
    
    // Start initial timeout
    k_work_schedule(&timeout_work, K_MSEC(TIMEOUT_MS));

    // Register activity callback
    zmk_activity_register_listener(activity_callback);
    
    LOG_INF("LED power control initialized");
}