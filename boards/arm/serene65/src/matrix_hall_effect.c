/*
 * Copyright (c) 2025 Matthijs Muller
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/kscan.h>

#include "hall_effect.h"

LOG_MODULE_REGISTER(matrix_hall_effect, CONFIG_ZMK_LOG_LEVEL);

static struct k_thread scanner_thread;
static K_KERNEL_STACK_DEFINE(scanner_stack, 1024);
static kscan_callback_t hall_effect_callback;
static const struct device *dev;

/* Scanner thread function */
static void scanner_loop(void *p1, void *p2, void *p3) {
    printk("MATRIX: Scanner thread starting\n");
    
    while (1) {
        // Call the hall effect matrix scan
        hall_effect_matrix_scan(dev);
        
        k_sleep(K_MSEC(20)); // Scan every 20ms
    }
}

/* Initialize hall effect matrix */
static int hall_effect_kscan_init(const struct device *_dev) {
    int ret;
    
    printk("MATRIX: Initializing hall effect matrix (device: %p)\n", _dev);
    dev = _dev;
    
    // Initialize the hall effect driver
    ret = hall_effect_init(_dev);
    if (ret != 0) {
        printk("MATRIX: Failed to initialize hall effect driver: %d\n", ret);
        return ret;
    }
    
    // Start scanner thread
    k_thread_create(&scanner_thread, scanner_stack, K_KERNEL_STACK_SIZEOF(scanner_stack),
                    scanner_loop, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(15), 0, K_NO_WAIT);
    
    printk("MATRIX: Hall effect matrix initialized successfully\n");
    return 0;
}

/* Configure KSCAN callback */
static int hall_effect_kscan_configure(const struct device *_dev,
                                      kscan_callback_t callback) {
    printk("MATRIX: Configuring hall effect kscan callback\n");
    hall_effect_callback = callback;
    return 0;
}

/* KSCAN driver API structure */
static const struct kscan_driver_api hall_effect_kscan_api = {
    .config = hall_effect_kscan_configure,
};

// We need to explicitly expose the callback to hall_effect.c
kscan_callback_t *get_hall_effect_callback_ptr(void) {
    return &hall_effect_callback;
}

/* Register our driver for the kscan0 node */
DEVICE_DEFINE(hall_effect_kscan, "HALL_EFFECT_MATRIX", hall_effect_kscan_init,
              NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
              &hall_effect_kscan_api);

/* Verify these values match your keyboard design */
#define MATRIX_ROWS 5  // Number of rows in your keyboard
#define MATRIX_COLS 15 // Number of columns in your keyboard 