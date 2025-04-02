/*
 * Copyright (c) 2024 Matthijs Muller
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/types.h>
#include <zephyr/drivers/led_strip.h>

/* Number of LEDs in the strip */
#define RGB_DRIVER_LED_COUNT 1

/**
 * @brief Initialize the RGB driver
 * 
 * @return int 0 on success, negative errno on failure
 */
int rgb_driver_init(void);

/**
 * @brief Update the RGB LED strip with the provided colors
 * 
 * @param pixels Array of color data for each LED
 * @param count Number of LEDs to update
 * @return int 0 on success, negative errno on failure
 */
int rgb_driver_update(struct led_rgb *pixels, size_t count);

/* Function provided to ZMK for RGB underglow updates */
int zmk_rgb_underglow_driver_update(struct led_rgb *pixels, size_t count);

/**
 * @brief Update all LEDs at once
 * 
 * @param values Array of LED values in physical order
 * @return int 0 on success, negative errno on failure
 */
int rgb_driver_update_all_leds(struct led_rgb *values);

/**
 * @brief Apply physical to logical LED mapping
 * 
 * @param values Input LED values in logical order
 * @param mapped_values Output LED values in physical order
 */
void rgb_driver_apply_mapping(struct led_rgb *values, struct led_rgb *mapped_values); 