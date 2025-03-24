/*
 * Copyright (c) 2025 Matthijs Muller
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once 

#ifndef _HALL_EFFECT_H_
#define _HALL_EFFECT_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zmk/matrix.h>

/* Matrix dimensions */
#define MATRIX_ROWS    5
#define MATRIX_COLS    15
#define SENSOR_COUNT   69

/* Multiplexer setup */
#define MUX_COUNT       5
#define MUX_SEL_PIN_COUNT 4

/* Calibration setup */
#define NOISE_FLOOR_SAMPLE_COUNT 5
#define NOISE_CEILING_SAMPLE_COUNT 5

/* Debounce settings */
#define DEBOUNCE_THRESHOLD 3

/* Default values */
#define DEFAULT_ACTUATION_LEVEL 2500
#define DEFAULT_RELEASE_LEVEL 2400
#define EXPECTED_NOISE_CEILING 4000

/* Rapid trigger settings */
#define DEFAULT_RELEASE_DISTANCE_RT 50
#define DEFAULT_DEADZONE_RT 50

/* Actuation modes */
typedef enum {
    ACTUATION_MODE_NORMAL = 0,
    ACTUATION_MODE_RAPID_TRIGGER = 1
} he_actuation_mode_t;

/* Sensor to matrix mapping structure */
typedef struct {
    uint8_t row;
    uint8_t col;
    uint8_t sensor_id;
    uint8_t mux_id;
    uint8_t mux_channel;
} sensor_to_matrix_map_t;

/* Hall effect configuration structure */
typedef struct {
    bool calibration_mode;
    bool post_flash_flag;
    he_actuation_mode_t actuation_mode;
} he_config_t;

/* Key configuration structure */
typedef struct {
    uint16_t noise_floor;
    uint16_t noise_ceiling;
    uint8_t actuation_threshold;
    uint8_t release_threshold;
} he_key_config_t;

/* Rapid trigger configuration structure */
typedef struct {
    uint8_t engage_distance;
    uint8_t disengage_distance;
    uint8_t deadzone;
    uint8_t rt_actuation_point;
} he_key_rapid_trigger_config_t;

/* Key debounce structure */
typedef struct {
    bool debounced_state;
    uint8_t debounce_counter;
} key_debounce_t;

/* Debug sample structure */
#define DEBUG_SAMPLE_COUNT 15
typedef struct {
    uint16_t samples[DEBUG_SAMPLE_COUNT];
    uint8_t index;
} sensor_data_t;

/* Key state structure */
typedef struct {
    bool pressed;
} he_key_state_t;

/* Global variables that need to be visible externally */
extern he_key_state_t he_key_states[SENSOR_COUNT];

/* Matrix state - externally visible */
extern bool matrix_state[MATRIX_ROWS][MATRIX_COLS];

/* Function prototypes */
int hall_effect_init(const struct device *dev);
uint16_t hall_effect_read_raw(uint8_t sensor_id);
bool hall_effect_matrix_scan(const struct device *dev);
bool hall_effect_is_pressed(int sensor);
int32_t hall_effect_get_value(int sensor);
void hall_effect_calibrate_noise_floor(void);
void hall_effect_calibrate_noise_ceiling(void);
void hall_effect_save_calibration(void);
void hall_effect_debug_thresholds(void);
void hall_effect_sample_all_values(void);

/* Matrix interface */
bool zmk_matrix_read_state(uint8_t row, uint8_t col);
void zmk_matrix_set_state(uint8_t row, uint8_t col, bool state);

/* Callback registration */
int hall_effect_set_callback(const struct device *dev, kscan_callback_t callback);

/* Add this declaration to your hall_effect.h file */
extern kscan_callback_t *get_hall_effect_callback_ptr(void);

#endif /* _HALL_EFFECT_H_ */ 