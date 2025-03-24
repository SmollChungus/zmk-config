/*
 * Copyright (c) 2025 Matthijs Muller
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

#include <zmk/matrix.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zephyr/drivers/kscan.h>

#include "hall_effect.h"

LOG_MODULE_REGISTER(matrix_hall_effect, CONFIG_ZMK_LOG_LEVEL);

/* Matrix state */
static bool matrix_state[MATRIX_ROWS][MATRIX_COLS] = {0};

/* Hall effect device */
static const struct device *hall_effect_dev;

/* Callback function */
static kscan_callback_t callback;

/* Timer for periodic scanning */
static struct k_timer scan_timer;
static bool is_scanning = false;

/* Read matrix state */
bool zmk_matrix_read_state(uint8_t row, uint8_t col) {
    if (row >= MATRIX_ROWS || col >= MATRIX_COLS) {
        return false;
    }
    
    return matrix_state[row][col];
}

/* Set matrix state */
void zmk_matrix_set_state(uint8_t row, uint8_t col, bool state) {
    if (row >= MATRIX_ROWS || col >= MATRIX_COLS) {
        return;
    }
    
    if (matrix_state[row][col] != state) {
        LOG_DBG("Key state changed: row=%d, col=%d, state=%d", row, col, state);
        matrix_state[row][col] = state;
    }
}

/* Matrix scan callback */
static void matrix_hall_effect_scan_cb(struct k_timer *timer) {
    LOG_DBG("Matrix scan callback triggered");
    bool matrix_changed = hall_effect_matrix_scan(hall_effect_dev);
    
    if (matrix_changed && callback != NULL) {
        LOG_INF("Matrix changed, notifying ZMK");
        // Notify ZMK about key state changes
        for (int row = 0; row < MATRIX_ROWS; row++) {
            for (int col = 0; col < MATRIX_COLS; col++) {
                bool state = zmk_matrix_read_state(row, col);
                if (state) {
                    LOG_INF("Key pressed: row=%d, col=%d", row, col);
                }
                callback(hall_effect_dev, row, col, state);
            }
        }
    }
}

/* Configure callback */
static int matrix_hall_effect_config(const struct device *dev, kscan_callback_t cb) {
    LOG_DBG("Configuring hall effect matrix callback");
    callback = cb;
    return 0;
}

/* Enable callback */
static int matrix_hall_effect_enable_callback(const struct device *dev) {
    LOG_DBG("Enabling hall effect matrix callback");
    if (!is_scanning) {
        is_scanning = true;
        LOG_INF("Starting matrix scan timer");
        k_timer_start(&scan_timer, K_MSEC(10), K_MSEC(10)); // 10ms interval
    }
    return 0;
}

/* Disable callback */
static int matrix_hall_effect_disable_callback(const struct device *dev) {
    LOG_DBG("Disabling hall effect matrix callback");
    if (is_scanning) {
        is_scanning = false;
        LOG_INF("Stopping matrix scan timer");
        k_timer_stop(&scan_timer);
    }
    return 0;
}

/* Matrix init */
static int matrix_hall_effect_init(const struct device *dev) {
    LOG_INF("Initializing hall effect matrix");
    
    /* Store hall effect device for later use */
    hall_effect_dev = dev;
    
    /* Initialize hall effect driver */
    int ret = hall_effect_init(hall_effect_dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize hall effect driver: %d", ret);
        return ret;
    }
    LOG_INF("Hall effect driver initialized successfully");
    
    /* Initialize timer for regular scanning */
    k_timer_init(&scan_timer, matrix_hall_effect_scan_cb, NULL);
    
    LOG_INF("Hall effect matrix initialized successfully");
    return 0;
}

/* Define the kscan driver API */
static const struct kscan_driver_api matrix_hall_effect_api = {
    .config = matrix_hall_effect_config,
    .enable_callback = matrix_hall_effect_enable_callback,
    .disable_callback = matrix_hall_effect_disable_callback,
};

/* Define the matrix device */
DEVICE_DEFINE(zmk_matrix_hall_effect, "KSCAN_HALL_EFFECT", matrix_hall_effect_init,
             NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, 
             &matrix_hall_effect_api); 