/*
 * Copyright (c) 2024 Matthijs Muller
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include "rgb_driver.h"

LOG_MODULE_REGISTER(rgb_driver, CONFIG_ZMK_LOG_LEVEL);

#define NUM_LEDS 1

/* Forward declarations from rgb_direct.c */
extern void rgb_direct_init(void);
extern void rgb_direct_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
extern void rgb_direct_update(void);
extern void rgb_direct_set_all(uint8_t r, uint8_t g, uint8_t b);
extern void rgb_direct_test(void);

/**
 * @brief Update the RGB LED strip with the provided colors
 * 
 * @param pixels Array of color data for each LED
 * @param count Number of LEDs to update
 * @return int 0 on success, negative errno on failure
 */
int rgb_driver_update(struct led_rgb *pixels, size_t count) {
    if (count > NUM_LEDS) {
        LOG_WRN("Requested to update %d LEDs, but only %d are available", count, NUM_LEDS);
        count = NUM_LEDS;
    }
    
    LOG_DBG("Updating RGB strip with %d LEDs", count);
    
    /* Update each LED using our direct driver */
    for (int i = 0; i < count; i++) {
        rgb_direct_set_led(i, pixels[i].r, pixels[i].g, pixels[i].b);
    }
    
    /* Send the update to the LEDs */
    rgb_direct_update();
    
    return 0;
}

/**
 * @brief Initialize the RGB driver
 * 
 * @return int 0 on success, negative errno on failure
 */
int rgb_driver_init(void) {
    /* Initialize our custom direct RGB driver */
    rgb_direct_init();
    
    /* Run a quick test to verify the driver is working */
    rgb_direct_test();
    
    LOG_INF("RGB driver initialized with %d LEDs", NUM_LEDS);
    return 0;
}

/* LED mapping - physical layout to logical mapping */
static const uint8_t led_map[] = {
    // Map of 16 LEDs in physical order
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
    30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
    58,     57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45,
    59, 60, 61,         62,         63, 64, 65, 66, 67
};

/* Custom LED update function */
int rgb_driver_update_all_leds(struct led_rgb *values) {
    /* Update each LED using our direct driver */
    for (int i = 0; i < NUM_LEDS; i++) {
        rgb_direct_set_led(i, values[i].r, values[i].g, values[i].b);
    }
    
    /* Send the update to the LEDs */
    rgb_direct_update();
    
    return 0;
}

/* Custom LED mapping function */
void rgb_driver_apply_mapping(struct led_rgb *values, struct led_rgb *mapped_values) {
    /* Apply the LED mapping */
    for (int i = 0; i < ARRAY_SIZE(led_map); i++) {
        mapped_values[led_map[i]] = values[i];
    }
} 