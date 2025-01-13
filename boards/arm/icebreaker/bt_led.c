#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <zmk/ble.h>
#include <zephyr/kernel.h>


//profile led
LOG_MODULE_REGISTER(bt_led, LOG_LEVEL_INF);


#define BLUE_LED_NODE DT_ALIAS(blue_led)

#if DT_NODE_HAS_STATUS(BLUE_LED_NODE, okay)
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(BLUE_LED_NODE, gpios);
#else
#error "Blue LED not defined in device tree"
#endif

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth initialization failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    // Start advertising
    bt_le_adv_start(BT_LE_ADV_CONN, NULL, 0, NULL, 0);
}

static void bt_connected(struct bt_conn *conn, uint8_t err)
{
    if (err == 0) {
        LOG_INF("Bluetooth connected");
        gpio_pin_set_dt(&blue_led, 0); // Turn OFF LED (Active High)
    } else {
        LOG_ERR("Bluetooth connection failed (err %d)", err);
    }
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Bluetooth disconnected");
    gpio_pin_set_dt(&blue_led, 1); // Turn ON LED (Active High)
}

// Define and register connection callbacks
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = bt_connected,
    .disconnected = bt_disconnected,
};

void bt_led_init(void)
{
    int ret;

    if (!device_is_ready(blue_led.port)) {
        LOG_ERR("Blue LED device not ready");
        return;
    }

    ret = gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure blue LED pin");
        return;
    }

    ret = bt_enable(bt_ready);
    if (ret < 0) {
        LOG_ERR("Bluetooth initialization failed (err %d)", ret);
        return;
    }

    gpio_pin_set_dt(&blue_led, 1); // Turn ON LED (Active High, advertising)
}

// profile LED stufdf
static void blink_led(int count) {
    for (int i = 0; i < count; i++) {
        gpio_pin_set_dt(&blue_led, 1); // Turn ON LED
        k_msleep(200);                // Wait 200ms
        gpio_pin_set_dt(&blue_led, 0); // Turn OFF LED
        k_msleep(200);                // Wait 200ms
    }
}

void bt_profile_led_blink(int profile) {
    LOG_INF("Blinking LED for profile %d", profile);
    blink_led(profile + 1); // Blink LED (Profile 0 = 1 blink, Profile 1 = 2 blinks, etc.)
}