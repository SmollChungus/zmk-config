/*
 * Copyright (c) 2025 Matthijs Muller
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>

#include <zmk/matrix.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

#include "hall_effect.h"

LOG_MODULE_REGISTER(hall_effect, CONFIG_ZMK_LOG_LEVEL);

/* Global variables */
static he_config_t he_config = {
    .calibration_mode = false,
    .post_flash_flag = false,
    .actuation_mode = ACTUATION_MODE_NORMAL
};

static he_key_config_t he_key_configs[SENSOR_COUNT];
static he_key_rapid_trigger_config_t he_key_rapid_trigger_configs[SENSOR_COUNT];
static key_debounce_t debounce_matrix[MATRIX_ROWS][MATRIX_COLS] = {{{0, 0}}};

/* ADC configuration */
static const struct device *adc_dev;
static struct adc_sequence adc_sequence;
static uint16_t adc_raw_value;

/* GPIO pin definitions for multiplexers */
static const struct {
    const struct device *port;
    gpio_pin_t pin;
} mux_sel_pins[MUX_SEL_PIN_COUNT] = {
    {DEVICE_DT_GET(DT_NODELABEL(gpiob)), 3},
    {DEVICE_DT_GET(DT_NODELABEL(gpiob)), 4},
    {DEVICE_DT_GET(DT_NODELABEL(gpiob)), 6},
    {DEVICE_DT_GET(DT_NODELABEL(gpiob)), 5}
};

static const struct {
    const struct device *port;
    gpio_pin_t pin;
} mux_en_pins[MUX_COUNT] = {
    {DEVICE_DT_GET(DT_NODELABEL(gpiob)), 0},
    {DEVICE_DT_GET(DT_NODELABEL(gpioa)), 7},
    {DEVICE_DT_GET(DT_NODELABEL(gpioa)), 6},
    {DEVICE_DT_GET(DT_NODELABEL(gpioa)), 5},
    {DEVICE_DT_GET(DT_NODELABEL(gpioa)), 4}
};

/* Sensor to matrix mapping */
static const sensor_to_matrix_map_t sensor_to_matrix_map[SENSOR_COUNT] = {
    {0,0,0,0,3},  {0,1,1,0,5},  {0,2,2,0,6},  {0,3,3,0,2},  {0,4,4,1,6},  {0,5,5,1,5},  {0,6,6,1,4},  {0,7,7,1,3},  {0,8,8,2,5},  {0,9,9,2,4},  {0,10,10,2,3},{0,11,11,2,2}, {0,12,12,3,3}, {0,13,13,3,2},{0,14,14,4,5},
    {1,0,15,0,4}, {1,1,16,0,7}, {1,2,17,0,0}, {1,3,18,0,1}, {1,4,19,1,7}, {1,5,20,1,0}, {1,6,21,1,1}, {1,7,22,1,2}, {1,8,23,2,6}, {1,9,24,2,0}, {1,10,25,2,1},{1,11,26,3,5}, {1,12,27,3,0}, {1,13,28,3,1},{1,14,29,4,2},
    {2,0,30,0,8}, {2,1,31,0,10},{2,2,32,0,15},{2,3,33,1,8}, {2,4,34,1,10},{2,5,35,1,15},{2,6,36,1,14},{2,7,37,2,8}, {2,8,38,2,9}, {2,9,39,2,15},{2,10,40,3,8},{2,11,41,3,9}, {2,12,42,3,14},{2,14,43,4,14},
    {3,0,44,0,9}, {3,1,45,0,13},{3,2,46,0,14},{3,3,47,1,11},{3,4,48,1,12},{3,5,49,2,10},{3,6,50,2,11},{3,7,51,2,12},{3,8,52,2,13},{3,9,53,2,14},{3,10,54,3,10},{3,11,55,3,13},{3,12,56,4,10},{3,13,57,4,13},{3,14,58,4,4},
    {4,0,59,0,11},{4,1,60,0,12},{4,2,61,1,9}, {4,6,62,1,13},{4,10,63,3,11},{4,11,64,3,12},{4,12,65,4,9}, {4,13,66,4,11},{4,14,67,4,12},{4,9,68,0,0}
};

/* Add at the top with other global variables */
bool matrix_state[MATRIX_ROWS][MATRIX_COLS] = {0};

/* Add at the top of the file, with other global variables */
static kscan_callback_t hall_effect_callback;

/* Initialize multiplexers */
static int mux_init(void) {
    int ret;

    LOG_INF("Initializing multiplexers");

    /* Initialize multiplexer select pins */
    for (int i = 0; i < MUX_SEL_PIN_COUNT; i++) {
        if (!device_is_ready(mux_sel_pins[i].port)) {
            LOG_ERR("GPIO device for MUX SEL %d not ready", i);
            return -ENODEV;
        }
        LOG_INF("MUX SEL %d GPIO device ready", i);

        ret = gpio_pin_configure(mux_sel_pins[i].port, mux_sel_pins[i].pin, 
                                GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Failed to configure MUX SEL pin %d: %d", i, ret);
            return ret;
        }
        LOG_INF("MUX SEL %d pin configured", i);
    }

    /* Initialize multiplexer enable pins */
    for (int i = 0; i < MUX_COUNT; i++) {
        if (!device_is_ready(mux_en_pins[i].port)) {
            LOG_ERR("GPIO device for MUX EN %d not ready", i);
            return -ENODEV;
        }
        LOG_INF("MUX EN %d GPIO device ready", i);

        ret = gpio_pin_configure(mux_en_pins[i].port, mux_en_pins[i].pin, 
                                GPIO_OUTPUT_ACTIVE);
        if (ret != 0) {
            LOG_ERR("Failed to configure MUX EN pin %d: %d", i, ret);
            return ret;
        }
        LOG_INF("MUX EN %d pin configured", i);
    }

    LOG_INF("Multiplexers initialized successfully");
    return 0;
}

/* Select multiplexer channel */
static void select_mux(uint8_t sensor_id) {
    uint8_t mux_id = sensor_to_matrix_map[sensor_id].mux_id;
    uint8_t mux_channel = sensor_to_matrix_map[sensor_id].mux_channel;
    
    LOG_DBG("select_mux: Selecting mux %d, channel %d for sensor %d", mux_id, mux_channel, sensor_id);

    /* Disable all multiplexers first */
    for (int i = 0; i < MUX_COUNT; i++) {
        gpio_pin_set(mux_en_pins[i].port, mux_en_pins[i].pin, 1);
    }
    LOG_DBG("select_mux: All multiplexers disabled");
    
    /* Set multiplexer channel */
    for (int i = 0; i < MUX_SEL_PIN_COUNT; i++) {
        gpio_pin_set(mux_sel_pins[i].port, mux_sel_pins[i].pin, 
                   (mux_channel >> i) & 0x01);
    }
    LOG_DBG("select_mux: Channel select pins set");
    
    /* Enable the selected multiplexer */
    gpio_pin_set(mux_en_pins[mux_id].port, mux_en_pins[mux_id].pin, 0);
    LOG_DBG("select_mux: Multiplexer %d enabled", mux_id);

    /* Small delay to allow the multiplexer to settle */
    k_busy_wait(5);
}

/* Read raw ADC value from a hall effect sensor */
uint16_t hall_effect_read_raw(uint8_t sensor_id) {
    int ret;
    
    LOG_DBG("hall_effect_read_raw: Reading sensor %d", sensor_id);
    select_mux(sensor_id);

    LOG_DBG("hall_effect_read_raw: Starting ADC read for sensor %d", sensor_id);
    ret = adc_read(adc_dev, &adc_sequence);
    if (ret != 0) {
        printk("HALL EFFECT: hall_effect_read_raw: Failed to read ADC for sensor %d: %d\n", sensor_id, ret);
        LOG_ERR("hall_effect_read_raw: Failed to read ADC for sensor %d: %d", sensor_id, ret);
        return 0;
    }
    
    LOG_DBG("hall_effect_read_raw: Sensor %d raw value: %d", sensor_id, adc_raw_value);
    return adc_raw_value;
}

/* Modify the rescale function to better handle your sensor values */
static uint8_t rescale(uint16_t sensor_value, uint8_t sensor_id) {
    // The resting value for most sensors is around 2020-2050
    // Most false presses show values around 2070-2100
    uint16_t noise_floor = 2050;    // Base value when key is not pressed
    uint16_t noise_ceiling = 3000;  // Maximum expected when fully pressed
    
    // Use the configured values if they're sensible
    if (he_key_configs[sensor_id].noise_ceiling > he_key_configs[sensor_id].noise_floor) {
        noise_floor = he_key_configs[sensor_id].noise_floor;
        noise_ceiling = he_key_configs[sensor_id].noise_ceiling;
    }

    // If the value is below or very close to the noise floor, return 0
    if (sensor_value <= noise_floor + 20) return 0;
    
    // If the value is above noise ceiling, return 100
    if (sensor_value >= noise_ceiling) return 100;
    
    // Otherwise, scale to 0-100 range
    return (uint8_t)(((uint32_t)(sensor_value - noise_floor - 20) * 100) / 
                    (noise_ceiling - noise_floor - 20));
}

/* Update key state using normal actuation mode */
static bool update_key_normal(uint8_t row, uint8_t col, uint8_t sensor_id, 
                            uint16_t sensor_value) {
    bool changed = false;
    uint8_t scaled_value = rescale(sensor_value, sensor_id);
    uint8_t actuation_threshold = he_key_configs[sensor_id].actuation_threshold;
    uint8_t release_threshold = he_key_configs[sensor_id].release_threshold;
    
    /* Get current state */
    bool current_state = zmk_matrix_read_state(row, col);
    
    /* Determine new state based on thresholds */
    bool new_state = current_state;
    if (current_state) {
        if (scaled_value < release_threshold) {
            new_state = false;
        }
    } else {
        if (scaled_value > actuation_threshold) {
            new_state = true;
        }
    }
    
    /* Apply debouncing */
    if (new_state != debounce_matrix[row][col].debounced_state) {
        if (debounce_matrix[row][col].debounce_counter < DEBOUNCE_THRESHOLD) {
            debounce_matrix[row][col].debounce_counter++;
        } else {
            debounce_matrix[row][col].debounced_state = new_state;
            debounce_matrix[row][col].debounce_counter = 0;
            
            /* Update matrix state */
            zmk_matrix_set_state(row, col, new_state);
            changed = true;
        }
    } else {
        debounce_matrix[row][col].debounce_counter = 0;
    }
    
    return changed;
}

/* Update key state using rapid trigger mode */
static bool update_key_rapid_trigger(uint8_t row, uint8_t col, uint8_t sensor_id, 
                                  uint16_t sensor_value) {
    bool changed = false;
    uint8_t scaled_value = rescale(sensor_value, sensor_id);
    uint8_t deadzone = he_key_rapid_trigger_configs[sensor_id].deadzone;
    uint8_t actuation_point = he_key_rapid_trigger_configs[sensor_id].rt_actuation_point;
    uint8_t engage_distance = he_key_rapid_trigger_configs[sensor_id].engage_distance;
    uint8_t disengage_distance = he_key_rapid_trigger_configs[sensor_id].disengage_distance;
    
    /* Get current state */
    bool current_state = zmk_matrix_read_state(row, col);
    
    /* Determine new state based on rapid trigger logic */
    bool new_state = current_state;
    
    if (scaled_value <= deadzone) {
        new_state = false;
    } else if (current_state) {
        if (scaled_value < actuation_point - disengage_distance) {
            new_state = false;
        }
    } else {
        if (scaled_value > actuation_point + engage_distance) {
            new_state = true;
        }
    }
    
    /* Apply debouncing */
    if (new_state != debounce_matrix[row][col].debounced_state) {
        if (debounce_matrix[row][col].debounce_counter < DEBOUNCE_THRESHOLD) {
            debounce_matrix[row][col].debounce_counter++;
        } else {
            debounce_matrix[row][col].debounced_state = new_state;
            debounce_matrix[row][col].debounce_counter = 0;
            
            /* Update matrix state */
            zmk_matrix_set_state(row, col, new_state);
            changed = true;
        }
    } else {
        debounce_matrix[row][col].debounce_counter = 0;
    }
    
    return changed;
}

/* Calibrate noise floor (minimum sensor values) */
void hall_effect_calibrate_noise_floor(void) {
    for (uint8_t sensor_id = 0; sensor_id < SENSOR_COUNT; sensor_id++) {
        uint16_t min_value = UINT16_MAX;
        
        for (uint8_t sample = 0; sample < NOISE_FLOOR_SAMPLE_COUNT; sample++) {
            hall_effect_read_raw(sensor_id);
            if (adc_raw_value < min_value) {
                min_value = adc_raw_value;
            }
            k_busy_wait(5);
        }
        
        he_key_configs[sensor_id].noise_floor = min_value;
    }
    
    hall_effect_save_calibration();
}

/* Calibrate noise ceiling (maximum sensor values) */
void hall_effect_calibrate_noise_ceiling(void) {
    for (uint8_t sensor_id = 0; sensor_id < SENSOR_COUNT; sensor_id++) {
        uint16_t max_value = 0;
        
        for (uint8_t sample = 0; sample < NOISE_CEILING_SAMPLE_COUNT; sample++) {
            hall_effect_read_raw(sensor_id);
            if (adc_raw_value > max_value) {
                max_value = adc_raw_value;
            }
            k_busy_wait(5);
        }
        
        he_key_configs[sensor_id].noise_ceiling = max_value;
    }
    
    hall_effect_save_calibration();
}

/* Save calibration data to settings */
void hall_effect_save_calibration(void) {
    /* Save calibration data to settings */
    settings_save_one("hall_effect/config", &he_config, sizeof(he_config));
    settings_save_one("hall_effect/key_configs", he_key_configs, 
                     sizeof(he_key_configs));
    settings_save_one("hall_effect/rt_configs", he_key_rapid_trigger_configs, 
                     sizeof(he_key_rapid_trigger_configs));
}

/* Settings direct load callback */
static int settings_direct_loader(const char *name, size_t len, settings_read_cb read_cb,
                                 void *cb_arg, void *param) {
    const void *data = param;
    size_t data_len = len;
    int rc;

    rc = read_cb(cb_arg, data, data_len);
    if (rc < 0) {
        return rc;
    }

    return 0;
}

/* Load calibration data from settings */
static int hall_effect_load_calibration(void) {
    int ret;
    
    /* Load hall effect configuration */
    ret = settings_load_subtree_direct("hall_effect/config", settings_direct_loader, 
                                      &he_config);
    if (ret < 0 && ret != -ENOENT) {
        LOG_ERR("Failed to load hall effect config: %d", ret);
        return ret;
    }
    
    /* Load key-specific configurations */
    ret = settings_load_subtree_direct("hall_effect/key_configs", settings_direct_loader, 
                                      he_key_configs);
    if (ret < 0 && ret != -ENOENT) {
        LOG_ERR("Failed to load hall effect key configs: %d", ret);
        return ret;
    }
    
    /* Load rapid trigger configurations */
    ret = settings_load_subtree_direct("hall_effect/rt_configs", settings_direct_loader, 
                                      he_key_rapid_trigger_configs);
    if (ret < 0 && ret != -ENOENT) {
        LOG_ERR("Failed to load hall effect RT configs: %d", ret);
        return ret;
    }
    
    return 0;
}

/* Add this function to your hall_effect.c file for debugging */
static void debug_adc_devices(void) {
    // List of possible ADC device paths to check
    const char* adc_paths[] = {
        "ADC_1",
        "ADC1",
        "adc@0",
        "adc@40012000",
        "adc1@40012000",
        "ST_STM32_ADC_1",
        "adc"
    };
    
    printk("HALL EFFECT DEBUG: Checking for available ADC devices\n");
    
    for (int i = 0; i < sizeof(adc_paths)/sizeof(char*); i++) {
        const struct device *dev = device_get_binding(adc_paths[i]);
        printk("HALL EFFECT DEBUG: Checking %s: %s\n", 
               adc_paths[i], 
               dev ? "FOUND" : "NOT FOUND");
    }
}

/* Initialize hall effect driver */
int hall_effect_init(const struct device *dev) {
    int err;
    k_sleep(K_MSEC(1000));

    LOG_INF("HALL EFFECT: Initializing hall effect driver");
    printk("HALL EFFECT: Initializing hall effect driver\n");

    // Add a small delay to ensure ADC subsystem is ready
    k_sleep(K_MSEC(100));

    // Get the ADC device
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        printk("HALL EFFECT: ADC device not ready\n");
        
        // Print more detailed error information
        printk("HALL EFFECT: ADC device pointer: %p\n", adc_dev);
        printk("HALL EFFECT: ADC device name: %s\n", adc_dev ? adc_dev->name : "NULL");
        
        return -ENODEV;
    }

    printk("HALL EFFECT: ADC device is ready\n");
    
    // Set up ADC channel configuration with explicit error checking
    struct adc_channel_cfg channel_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = 3, // Use channel 3 (ADC1_IN3)
        .differential = 0
    };
    
    err = adc_channel_setup(adc_dev, &channel_cfg);
    if (err != 0) {
        LOG_ERR("Failed to setup ADC channel: %d", err);
        printk("HALL EFFECT: Failed to setup ADC channel: %d\n", err);
        return err;
    }

    printk("HALL EFFECT: ADC channel setup complete\n");

    // Set up ADC sequence with explicit buffer
    adc_sequence.channels = BIT(3); // Channel 3
    adc_sequence.buffer = &adc_raw_value;
    adc_sequence.buffer_size = sizeof(adc_raw_value);
    adc_sequence.resolution = 12;
    adc_sequence.oversampling = 0;
    adc_sequence.calibrate = false;

    printk("HALL EFFECT: ADC sequence configuration complete\n");

    // Initialize multiplexers
    err = mux_init();
    if (err != 0) {
        LOG_ERR("Failed to initialize multiplexers: %d", err);
        printk("HALL EFFECT: Failed to initialize multiplexers: %d\n", err);
        return err;
    }

    // Initialize key configurations with default values
    for (int i = 0; i < SENSOR_COUNT; i++) {
        he_key_configs[i].noise_floor = 2050;     // Resting value
        he_key_configs[i].noise_ceiling = 3000;   // Fully pressed value
        he_key_configs[i].actuation_threshold = 80; // 80% to actuate (high to prevent false triggers)
        he_key_configs[i].release_threshold = 60;   // 60% to release (provides hysteresis)
        
        he_key_rapid_trigger_configs[i].deadzone = 30; // 30% deadzone - increase to avoid accidental triggers
        he_key_rapid_trigger_configs[i].rt_actuation_point = 80; // 80% actuation - higher threshold
        he_key_rapid_trigger_configs[i].engage_distance = 10;    // 10% engage distance
        he_key_rapid_trigger_configs[i].disengage_distance = 10; // 10% disengage distance
    }

    // Load calibration data from settings
    settings_subsys_init();
    settings_load();

    // Set post-flash flag to true on first boot
    if (!he_config.post_flash_flag) {
        LOG_INF("Setting post-flash flag for first-time calibration");
        he_config.post_flash_flag = true;
        settings_save_one("hall_effect/config", &he_config, sizeof(he_config));
    }

    // If this is the first boot after flashing, calibrate the sensors
    if (he_config.post_flash_flag) {
        LOG_INF("Post-flash flag set, calibrating sensors");
        hall_effect_calibrate_noise_floor();
        hall_effect_calibrate_noise_ceiling();
        he_config.post_flash_flag = false;
        settings_save_one("hall_effect/config", &he_config, sizeof(he_config));
    }

    // Add threshold debugging at the end of initialization
    hall_effect_debug_thresholds();

    // Add this at the end of the function
    printk("HALL EFFECT: Sampling all sensors to determine thresholds\n");
    hall_effect_sample_all_values();
    
    LOG_INF("Hall effect driver initialized successfully");
    printk("HALL EFFECT: Hall effect driver initialized successfully\n");
    return 0;
}

/* Update the matrix scan function to use the callback */
bool hall_effect_matrix_scan(const struct device *dev) {
    bool matrix_changed = false;
    static uint32_t scan_count = 0;
    
    // Get the callback pointer
    kscan_callback_t *callback_ptr = get_hall_effect_callback_ptr();
    
    // Log scan count occasionally for debugging
    if (scan_count++ % 100 == 0) {
        printk("HALL EFFECT: Matrix scan %u\n", scan_count);
    }
    
    // Scan each sensor
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint8_t row = sensor_to_matrix_map[i].row;
        uint8_t col = sensor_to_matrix_map[i].col;
        
        // Skip sensors outside the valid matrix range
        if (row >= MATRIX_ROWS || col >= MATRIX_COLS) {
            continue;
        }
        
        // Read the sensor value
        uint16_t raw_value = hall_effect_read_raw(i);
        uint8_t scaled_value = rescale(raw_value, i);
        
        // Determine the new state
        bool was_pressed = matrix_state[row][col];
        bool is_pressed = scaled_value >= he_key_configs[i].actuation_threshold;
        
        // If the state changed, update and report it
        if (is_pressed != was_pressed) {
            matrix_state[row][col] = is_pressed;
            
            // Debug output
            printk("KEY CHANGE: row %d, col %d, sensor %d, value %d, scaled %d, is_pressed: %s\n",
                   row, col, i, raw_value, scaled_value,
                   is_pressed ? "TRUE" : "FALSE");
            
            // Notify ZMK of the key state change using the registered callback
            if (callback_ptr && *callback_ptr) {
                (*callback_ptr)(dev, row, col, is_pressed);
                printk("HALL EFFECT: Called callback for key change\n");
            } else {
                printk("HALL EFFECT: Warning - callback not registered!\n");
            }
            
            matrix_changed = true;
        }
    }
    
    return matrix_changed;
}

bool hall_effect_is_pressed(int sensor) {
    int32_t value = hall_effect_get_value(sensor);
    bool pressed = false;
    
    if (value < 0) {
        // Error reading value
        LOG_ERR("Error reading hall effect value for sensor %d: %d", sensor, value);
        printk("HALL EFFECT: Error reading value for sensor %d: %d\n", sensor, value);
        return false;
    }
    
    if (IS_ENABLED(CONFIG_ZMK_HALL_EFFECT_RAPID_TRIGGER)) {
        // Rapid trigger mode
        if (he_key_states[sensor].pressed) {
            // Key is already pressed, check for release
            if (value <= he_key_configs[sensor].release_threshold) {
                he_key_states[sensor].pressed = false;
                pressed = false;
                LOG_DBG("Sensor %d RELEASED (RT): value %d <= threshold %d", 
                       sensor, value, he_key_configs[sensor].release_threshold);
                printk("HALL EFFECT: Sensor %d RELEASED (RT): %d <= %d\n", 
                       sensor, value, he_key_configs[sensor].release_threshold);
            } else {
                pressed = true;
            }
        } else {
            // Key is released, check for press
            if (value >= he_key_configs[sensor].actuation_threshold) {
                he_key_states[sensor].pressed = true;
                pressed = true;
                LOG_DBG("Sensor %d PRESSED (RT): value %d >= threshold %d", 
                       sensor, value, he_key_configs[sensor].actuation_threshold);
                printk("HALL EFFECT: Sensor %d PRESSED (RT): %d >= %d\n", 
                       sensor, value, he_key_configs[sensor].actuation_threshold);
            } else {
                pressed = false;
            }
        }
    } else {
        // Standard mode
        if (he_key_states[sensor].pressed) {
            // Key is already pressed, check for release
            if (value <= he_key_configs[sensor].release_threshold) {
                he_key_states[sensor].pressed = false;
                pressed = false;
                LOG_DBG("Sensor %d RELEASED: value %d <= threshold %d", 
                       sensor, value, he_key_configs[sensor].release_threshold);
                printk("HALL EFFECT: Sensor %d RELEASED: %d <= %d\n", 
                       sensor, value, he_key_configs[sensor].release_threshold);
            } else {
                pressed = true;
            }
        } else {
            // Key is released, check for press
            if (value >= he_key_configs[sensor].actuation_threshold) {
                he_key_states[sensor].pressed = true;
                pressed = true;
                LOG_DBG("Sensor %d PRESSED: value %d >= threshold %d", 
                       sensor, value, he_key_configs[sensor].actuation_threshold);
                printk("HALL EFFECT: Sensor %d PRESSED: %d >= %d\n", 
                       sensor, value, he_key_configs[sensor].actuation_threshold);
            } else {
                pressed = false;
            }
        }
    }
    
    return pressed;
}

// Add this new debugging function to check all thresholds
void hall_effect_debug_thresholds(void) {
    printk("HALL EFFECT DEBUG: Checking all key thresholds\n");
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
        printk("HALL EFFECT DEBUG: Sensor %d - noise_floor: %d, noise_ceiling: %d, actuation: %d, release: %d\n",
               i,
               he_key_configs[i].noise_floor,
               he_key_configs[i].noise_ceiling,
               he_key_configs[i].actuation_threshold,
               he_key_configs[i].release_threshold);
    }
}

// Add near the top with other global variables
he_key_state_t he_key_states[SENSOR_COUNT] = {0};

// Add this missing function implementation
int32_t hall_effect_get_value(int sensor) {
    if (sensor < 0 || sensor >= SENSOR_COUNT) {
        LOG_ERR("Invalid sensor ID: %d", sensor);
        return -EINVAL;
    }
    
    uint16_t raw_value = hall_effect_read_raw(sensor);
    uint8_t scaled_value = rescale(raw_value, sensor);
    
    LOG_DBG("Sensor %d: raw=%d, scaled=%d", sensor, raw_value, scaled_value);
    return scaled_value;
}

/* Make sure this function is defined before it's used */
void hall_effect_sample_all_values(void) {
    printk("===== HALL EFFECT SENSOR SAMPLING =====\n");
    uint16_t min_value = 65535;
    uint16_t max_value = 0;
    
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint16_t value = hall_effect_read_raw(i);
        
        printk("Sensor %d: raw=%d\n", i, value);
        
        if (value < min_value) min_value = value;
        if (value > max_value) max_value = value;
    }
    
    printk("Min value: %d, Max value: %d\n", min_value, max_value);
    printk("Suggested noise_floor: %d\n", min_value + 10);
    printk("Suggested noise_ceiling: %d\n", max_value + 500);
    printk("===== SAMPLING COMPLETE =====\n");
}

/* Add this function to register the callback */
int hall_effect_set_callback(const struct device *dev, kscan_callback_t callback) {
    hall_effect_callback = callback;
    return 0;
} 