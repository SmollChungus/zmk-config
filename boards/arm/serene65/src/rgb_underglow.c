/*
 * Copyright (c) 2025 Matthijs Muller
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/rgb_underglow.h>
#include "rgb_driver.h"

LOG_MODULE_REGISTER(rgb_underglow, CONFIG_ZMK_LOG_LEVEL);

#define NUM_PIXELS 1
static struct led_rgb pixels[NUM_PIXELS];
static bool initialized = false;
static int64_t last_update = 0;
#define UPDATE_INTERVAL_MS 50  // 20 updates per second maximum

/* Called by ZMK core to initialize RGB underglow */
int zmk_rgb_underglow_init(void) {
    LOG_INF("Initializing custom RGB underglow for Serene65");
    
    int rc = rgb_driver_init();
    if (rc != 0) {
        LOG_ERR("Failed to initialize RGB driver: %d", rc);
        return rc;
    }

    /* Initialize all LEDs to off */
    for (int i = 0; i < NUM_PIXELS; i++) {
        pixels[i].r = 0;
        pixels[i].g = 0;
        pixels[i].b = 0;
    }
    
    initialized = true;
    LOG_INF("RGB underglow initialized");
    return 0;
}

/* Called by ZMK core to update RGB underglow */
void zmk_rgb_underglow_update(void) {
    if (!initialized) {
        return;
    }
    
    /* Throttle updates to avoid overloading USB */
    int64_t now = k_uptime_get();
    if ((now - last_update) < UPDATE_INTERVAL_MS) {
        return; // Skip update if too frequent
    }
    last_update = now;
    
    /* Update the LED strip with our pixel data */
    int rc = rgb_driver_update(pixels, NUM_PIXELS);
    if (rc) {
        LOG_ERR("Failed to update RGB strip: %d", rc);
    }
}

/* Called by ZMK core to set RGB underglow pixel */
void zmk_rgb_underglow_set_color(size_t index, struct led_rgb *color) {
    if (index < NUM_PIXELS) {
        pixels[index] = *color;
    }
} 