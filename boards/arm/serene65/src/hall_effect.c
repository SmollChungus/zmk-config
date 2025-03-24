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
    
    printk("HALL EFFECT: select_mux: Selecting mux %d, channel %d for sensor %d\n", mux_id, mux_channel, sensor_id);
    LOG_INF("select_mux: Selecting mux %d, channel %d for sensor %d", mux_id, mux_channel, sensor_id);

    /* Disable all multiplexers first */
    for (int i = 0; i < MUX_COUNT; i++) {
        gpio_pin_set(mux_en_pins[i].port, mux_en_pins[i].pin, 1);
    }
    printk("HALL EFFECT: select_mux: All multiplexers disabled\n");
    LOG_INF("select_mux: All multiplexers disabled");
    
    /* Set multiplexer channel */
    for (int i = 0; i < MUX_SEL_PIN_COUNT; i++) {
        gpio_pin_set(mux_sel_pins[i].port, mux_sel_pins[i].pin, 
                   (mux_channel >> i) & 0x01);
    }
    printk("HALL EFFECT: select_mux: Channel select pins set\n");
    LOG_INF("select_mux: Channel select pins set");
    
    /* Enable the selected multiplexer */
    gpio_pin_set(mux_en_pins[mux_id].port, mux_en_pins[mux_id].pin, 0);
    printk("HALL EFFECT: select_mux: Multiplexer %d enabled\n", mux_id);
    LOG_INF("select_mux: Multiplexer %d enabled", mux_id);

    /* Small delay to allow the multiplexer to settle */
    k_busy_wait(5);
}

/* Read raw ADC value from a hall effect sensor */
uint16_t hall_effect_read_raw(uint8_t sensor_id) {
    int ret;
    
    printk("HALL EFFECT: hall_effect_read_raw: Reading sensor %d\n", sensor_id);
    LOG_INF("hall_effect_read_raw: Reading sensor %d", sensor_id);
    select_mux(sensor_id);

    printk("HALL EFFECT: hall_effect_read_raw: Starting ADC read for sensor %d\n", sensor_id);
    LOG_INF("hall_effect_read_raw: Starting ADC read for sensor %d", sensor_id);
    ret = adc_read(adc_dev, &adc_sequence);
    if (ret != 0) {
        printk("HALL EFFECT: hall_effect_read_raw: Failed to read ADC for sensor %d: %d\n", sensor_id, ret);
        LOG_ERR("hall_effect_read_raw: Failed to read ADC for sensor %d: %d", sensor_id, ret);
        return 0;
    }
    
    printk("HALL EFFECT: hall_effect_read_raw: Sensor %d raw value: %d\n", sensor_id, adc_raw_value);
    LOG_INF("hall_effect_read_raw: Sensor %d raw value: %d", sensor_id, adc_raw_value);
    return adc_raw_value;
}

/* Rescale sensor value to 0-100 range */
static uint8_t rescale(uint16_t sensor_value, uint8_t sensor_id) {
    uint16_t noise_floor = he_key_configs[sensor_id].noise_floor;
    uint16_t noise_ceiling = he_key_configs[sensor_id].noise_ceiling;

    if (noise_ceiling > noise_floor) {
        if (sensor_value <= noise_floor) return 0;
        if (sensor_value >= noise_ceiling) return 100;
        return (uint8_t)(((uint32_t)(sensor_value - noise_floor) * 100) / 
                        (noise_ceiling - noise_floor));
    }
    return 0;
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
    uint16_t samples[NOISE_FLOOR_SAMPLE_COUNT];
    
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
    uint16_t samples[NOISE_CEILING_SAMPLE_COUNT];
    
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
        he_key_configs[i].noise_floor = 0;
        he_key_configs[i].noise_ceiling = EXPECTED_NOISE_CEILING;
        he_key_configs[i].actuation_threshold = DEFAULT_ACTUATION_LEVEL;
        he_key_configs[i].release_threshold = DEFAULT_RELEASE_LEVEL;
        
        he_key_rapid_trigger_configs[i].deadzone = DEFAULT_DEADZONE_RT;
        he_key_rapid_trigger_configs[i].rt_actuation_point = 50;
        he_key_rapid_trigger_configs[i].engage_distance = DEFAULT_RELEASE_DISTANCE_RT;
        he_key_rapid_trigger_configs[i].disengage_distance = DEFAULT_RELEASE_DISTANCE_RT;
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

    LOG_INF("Hall effect driver initialized successfully");
    printk("HALL EFFECT: Hall effect driver initialized successfully\n");
    return 0;
}

/* Scan the matrix and update key states */
bool hall_effect_matrix_scan(const struct device *dev) {
    bool matrix_changed = false;
    
    LOG_INF("Starting hall effect matrix scan");
    
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint8_t row = sensor_to_matrix_map[i].row;
        uint8_t col = sensor_to_matrix_map[i].col;
        uint8_t sensor_id = sensor_to_matrix_map[i].sensor_id;
        
        /* Skip the encoder sensor (last one) */
        if (sensor_id == 68) {
            continue;
        }
        
        LOG_INF("Scanning sensor %d (row %d, col %d)", sensor_id, row, col);
        
        uint16_t sensor_value = hall_effect_read_raw(sensor_id);
        uint8_t scaled_value = rescale(sensor_value, sensor_id);
        
        LOG_INF("Sensor %d (row %d, col %d): raw=%d, scaled=%d", 
               sensor_id, row, col, sensor_value, scaled_value);
        
        /* Update key state based on actuation mode */
        bool changed = false;
        switch (he_config.actuation_mode) {
            case ACTUATION_MODE_RAPID_TRIGGER:
                LOG_INF("Using rapid trigger mode for sensor %d", sensor_id);
                changed = update_key_rapid_trigger(row, col, sensor_id, sensor_value);
                break;
            case ACTUATION_MODE_NORMAL:
            default:
                LOG_INF("Using normal mode for sensor %d", sensor_id);
                changed = update_key_normal(row, col, sensor_id, sensor_value);
                break;
        }
        
        if (changed) {
            LOG_INF("Key state changed at row %d, col %d", row, col);
            matrix_changed = true;
        }
    }
    
    LOG_INF("Matrix scan complete, changed: %d", matrix_changed);
    return matrix_changed;
} 