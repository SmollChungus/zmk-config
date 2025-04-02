/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: MIT
 *
 * Custom WS2812 GPIO driver for Serene65 keyboard
 * Based on the Zephyr WS2812 drivers
 */

#define DT_DRV_COMPAT worldsemi_ws2812_custom

#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/led/led.h>

#if defined(CONFIG_SOC_SERIES_STM32F4X)
#include <stm32f4xx.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_gpio_custom, CONFIG_LED_STRIP_LOG_LEVEL);

/* 
 * WS2812 timing (in ns) - Carefully tuned for STM32F4 PA8
 * These values are adjusted based on the STM32F4 running at 72MHz
 */
#define T0H    350    /* Zero high time */
#define T1H    900    /* One high time */
#define T0L    900    /* Zero low time */
#define T1L    350    /* One low time */
#define RESET  80000  /* Reset time */

struct ws2812_gpio_cfg {
    struct gpio_dt_spec in_gpio;
    uint8_t num_leds;
    const uint8_t *color_mapping;
    uint8_t num_colors;
};

struct ws2812_gpio_data {
    uint8_t *led_buf;
    size_t buf_size;
    /* Static buffer for LED data */
    uint8_t buffer[1 * 3]; /* 16 LEDs × 3 colors = 48 bytes */
};

/* Simple routine to convert the microsecond timing values into instruction cycles */
static inline uint32_t ns_to_cycles(uint32_t ns)
{
#if defined(SystemCoreClock)
    return (ns * (SystemCoreClock / 1000000U)) / 1000;
#else
    /* Use the configured system clock frequency if SystemCoreClock is not available */
    return (ns * (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000U)) / 1000;
#endif
}

/* Convert an RGB value to the PWM bits for the strip */
static inline void ws2812_gpio_write_byte(const struct device *dev, uint8_t byte)
{
    struct ws2812_gpio_cfg *config = (struct ws2812_gpio_cfg *)dev->config;
    const struct gpio_dt_spec *gpio = &config->in_gpio;
    uint32_t cyc_t0h, cyc_t1h, cyc_t0l, cyc_t1l;
    uint8_t mask;
    uint32_t irq_state;
    uint32_t start_cycles, cycles_to_wait;
    
    cyc_t0h = ns_to_cycles(T0H);
    cyc_t1h = ns_to_cycles(T1H);
    cyc_t0l = ns_to_cycles(T0L);
    cyc_t1l = ns_to_cycles(T1L);

    /* Process each bit with minimal critical section per bit */
    for (mask = 0x80; mask; mask >>= 1) {
        /* Lock interrupts only for the precise timing of each bit transition */
        irq_state = irq_lock();
        
        if (byte & mask) {
            /* Send 1 */
            start_cycles = k_cycle_get_32();
            gpio_pin_set_dt(gpio, 1);
            cycles_to_wait = cyc_t1h;
            while ((k_cycle_get_32() - start_cycles) < cycles_to_wait) {
                /* Busy wait */
            }
            
            gpio_pin_set_dt(gpio, 0);
            cycles_to_wait = cyc_t1l;
            while ((k_cycle_get_32() - start_cycles) < (cyc_t1h + cycles_to_wait)) {
                /* Busy wait */
            }
        } else {
            /* Send 0 */
            start_cycles = k_cycle_get_32();
            gpio_pin_set_dt(gpio, 1);
            cycles_to_wait = cyc_t0h;
            while ((k_cycle_get_32() - start_cycles) < cycles_to_wait) {
                /* Busy wait */
            }
            
            gpio_pin_set_dt(gpio, 0);
            cycles_to_wait = cyc_t0l;
            while ((k_cycle_get_32() - start_cycles) < (cyc_t0h + cycles_to_wait)) {
                /* Busy wait */
            }
        }
        
        /* Unlock interrupts after each bit to allow USB interrupts to be serviced */
        irq_unlock(irq_state);
    }
}

static int ws2812_gpio_update_rgb(const struct device *dev, struct led_rgb *pixels,
                              size_t num_pixels)
{
    struct ws2812_gpio_cfg *config = (struct ws2812_gpio_cfg *)dev->config;
    struct ws2812_gpio_data *data = (struct ws2812_gpio_data *)dev->data;
    const struct gpio_dt_spec *gpio = &config->in_gpio;
    uint8_t *ptr = data->led_buf;
    size_t i;

    if (num_pixels > config->num_leds) {
        num_pixels = config->num_leds;
    }

    /* Convert RGB data to the format needed by the strip */
    for (i = 0; i < num_pixels; i++) {
        uint8_t j;

        for (j = 0; j < config->num_colors; j++) {
            switch (config->color_mapping[j]) {
            case LED_COLOR_ID_RED:
                *ptr = pixels[i].r;
                break;
            case LED_COLOR_ID_GREEN:
                *ptr = pixels[i].g;
                break;
            case LED_COLOR_ID_BLUE:
                *ptr = pixels[i].b;
                break;
            default:
                *ptr = 0;
                break;
            }
            ptr++;
        }
    }

    /* Add significant delay before LED update to allow USB processing */
    k_sleep(K_MSEC(5));

    /* Send data to the LED strip - one byte at a time with opportunities for USB interrupts */
    ptr = data->led_buf;
    for (i = 0; i < (num_pixels * config->num_colors); i++) {
        /* Each byte gets processed with its own critical sections at the bit level */
        ws2812_gpio_write_byte(dev, *ptr);
        
        /* Small delay between bytes to allow USB interrupts */
        k_busy_wait(10);
        
        ptr++;
        
        /* Every 8 bytes, add a longer delay to ensure USB gets processing time */
        if ((i & 0x7) == 0x7) {
            k_sleep(K_MSEC(1));
        }
    }

    /* Send reset pulse */
    gpio_pin_set_dt(gpio, 0);
    k_busy_wait(RESET / 1000); /* Convert ns to us for k_busy_wait */

    /* Add additional delay after LED updates to ensure USB has time */
    k_sleep(K_MSEC(5));

    return 0;
}

static int ws2812_gpio_update_channels(const struct device *dev, uint8_t *channels,
                                   size_t num_channels)
{
    /* Not implemented */
    return -ENOTSUP;
}

static int ws2812_gpio_init(const struct device *dev)
{
    struct ws2812_gpio_cfg *config = (struct ws2812_gpio_cfg *)dev->config;
    struct ws2812_gpio_data *data = (struct ws2812_gpio_data *)dev->data;
    const struct gpio_dt_spec *gpio = &config->in_gpio;
    int ret;

    /* Initialize the GPIO pin */
    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    /* 
     * Configure PA8 as output with high drive strength
     * Important to use ACTIVE_HIGH with OUTPUT_INIT_LOW
     */
    ret = gpio_pin_configure_dt(gpio, GPIO_OUTPUT_ACTIVE | GPIO_OUTPUT_INIT_LOW);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO pin: %d", ret);
        return ret;
    }

    /* Initial low pulse to reset the strip */
    gpio_pin_set_dt(gpio, 0);
    k_busy_wait(RESET / 1000);

    /* Allocate memory for LED data */
    data->buf_size = config->num_leds * config->num_colors;
    data->led_buf = data->buffer;

    /* Initialize the buffer with zeros */
    memset(data->led_buf, 0, data->buf_size);

    LOG_INF("WS2812 GPIO driver initialized on pin PA8, %d LEDs", config->num_leds);
    return 0;
}

static const struct led_strip_driver_api ws2812_gpio_api = {
    .update_rgb = ws2812_gpio_update_rgb,
    .update_channels = ws2812_gpio_update_channels,
};

/* LED strip color mapping - GRB ordering (typical for WS2812) */
static const uint8_t ws2812_color_mapping[] = {
    LED_COLOR_ID_GREEN,
    LED_COLOR_ID_RED,
    LED_COLOR_ID_BLUE,
};

#define WS2812_CUSTOM_DEVICE(id)                                        \
    static struct ws2812_gpio_data ws2812_gpio_##id##_data;             \
    static const struct ws2812_gpio_cfg ws2812_gpio_##id##_cfg = {      \
        .in_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(id), in_gpios),         \
        .num_leds = DT_INST_PROP(id, chain_length),                     \
        .color_mapping = ws2812_color_mapping,                          \
        .num_colors = 3,                                                \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(id,                                           \
                        ws2812_gpio_init,                               \
                        NULL,                                           \
                        &ws2812_gpio_##id##_data,                       \
                        &ws2812_gpio_##id##_cfg,                        \
                        POST_KERNEL,                                    \
                        90, /* Higher priority than default */          \
                        &ws2812_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_CUSTOM_DEVICE) 