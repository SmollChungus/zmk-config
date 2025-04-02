/*
 * Copyright (c) 2024 Matthijs Muller
 * SPDX-License-Identifier: MIT
 *
 * WS2812 PWM+DMA driver for STM32F4
 * This driver uses Timer1 with DMA to efficiently drive WS2812 LEDs
 * without CPU involvement during the transmission
 */

#define DT_DRV_COMPAT worldsemi_ws2812_strip

#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SOC_SERIES_STM32F4X)
#include <stm32f4xx.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_tim.h>
#endif

LOG_MODULE_REGISTER(ws2812_pwm_dma, CONFIG_LED_STRIP_LOG_LEVEL);

/* WS2812 timing - for PWM frequency of ~800kHz (1.25us period) */
#define WS2812_PERIOD    60      /* 60 PWM ticks = 1.25us @72MHz/1.5 = 48MHz timer clock */
#define WS2812_ONE_DUTY  40      /* ~0.83us high (duty cycle ~67%) */
#define WS2812_ZERO_DUTY 20      /* ~0.42us high (duty cycle ~33%) */
#define WS2812_RESET_LEN 60      /* Number of reset bits (all zero duty) - 50us of zeros */

/* For 72MHz clock, with APB2 div=1 and TIM1 on APB2, timer clock is 72MHz
 * We need to use prescaler to get close to 800kHz for WS2812 */
#define TIM_PRESCALER    1       /* 72MHz/1.5 = 48MHz timer clock for TIM1 */

/* Maximum number of LEDs supported */
#define MAX_LEDS         100
#define BYTES_PER_LED    3       /* GRB format - 3 bytes per LED */
#define BITS_PER_BYTE    8       /* 8 bits per byte */
#define BITS_PER_LED     (BYTES_PER_LED * BITS_PER_BYTE)

/* Buffer to hold the PWM duty cycle values for WS2812 protocol
 * Each LED needs 24 PWM periods (8 bits × 3 colors)
 * Plus reset period of ~50us (60 periods at 800kHz) */
#define PWM_BUFFER_SIZE (MAX_LEDS * BITS_PER_LED + WS2812_RESET_LEN)

/* Configuration and data structures */
struct ws2812_pwm_dma_data {
    uint16_t pwm_buffer[PWM_BUFFER_SIZE];
    size_t buf_size;
    struct dma_config dma_cfg;
    struct dma_block_config dma_blk_cfg;
    uint8_t *led_buf;
    size_t led_count;
    struct k_sem sync_sem;
    const uint8_t *color_mapping;
    uint8_t num_colors;
    volatile bool transfer_complete;
    const struct device *dma_dev;
};

struct ws2812_pwm_dma_cfg {
    uint8_t dma_channel;
    uint8_t dma_slot;
    uint8_t num_leds;
    const uint8_t *color_mapping;
    uint8_t num_colors;
};

/* GPIO to use for LED strip */
#define TIM1_CH1_PIN PA8

/* Buffer to store LED RGB values */
static uint8_t led_buffer[MAX_LEDS * 3];

/* DMA completion callback */
static void ws2812_dma_callback(const struct device *dma_dev, void *user_data,
                               uint32_t channel, int status)
{
    struct ws2812_pwm_dma_data *data = (struct ws2812_pwm_dma_data *)user_data;
    
    /* Set flag to indicate transfer is complete */
    data->transfer_complete = true;
    
    /* Signal any waiting threads */
    k_sem_give(&data->sync_sem);
    
    LOG_DBG("DMA transfer completed with status %d", status);
}

/* Convert an LED color byte to PWM duty cycles for WS2812 protocol */
static void byte_to_pwm_duty_cycles(uint8_t byte, uint16_t *pwm_values)
{
    /* Convert each bit of the byte to corresponding PWM duty cycles */
    for (int i = 0; i < 8; i++) {
        if (byte & (0x80 >> i)) {
            pwm_values[i] = WS2812_ONE_DUTY;
        } else {
            pwm_values[i] = WS2812_ZERO_DUTY;
        }
    }
}

/* Update the LED strip using PWM with DMA */
static int ws2812_pwm_dma_update_rgb(const struct device *dev,
                                   struct led_rgb *pixels,
                                   size_t num_pixels)
{
    struct ws2812_pwm_dma_data *data = (struct ws2812_pwm_dma_data *)dev->data;
    const struct ws2812_pwm_dma_cfg *config = dev->config;
    uint16_t *pwm_buf_ptr = data->pwm_buffer;
    uint8_t *led_buf_ptr = data->led_buf;
    int i, j, ret;
    
    if (num_pixels > config->num_leds) {
        num_pixels = config->num_leds;
    }
    
    /* Wait for any ongoing DMA transfer to complete */
    if (!data->transfer_complete) {
        LOG_DBG("Waiting for previous DMA transfer to complete");
        if (k_sem_take(&data->sync_sem, K_MSEC(50)) != 0) {
            LOG_ERR("Timeout waiting for DMA transfer to complete");
            return -ETIMEDOUT;
        }
    }
    
    /* Convert RGB values to LED buffer with correct color mapping */
    for (i = 0; i < num_pixels; i++) {
        for (j = 0; j < config->num_colors; j++) {
            uint8_t value = 0;
            switch (config->color_mapping[j]) {
            case LED_COLOR_ID_RED:
                value = pixels[i].r;
                break;
            case LED_COLOR_ID_GREEN:
                value = pixels[i].g;
                break;
            case LED_COLOR_ID_BLUE:
                value = pixels[i].b;
                break;
            default:
                value = 0;
                break;
            }
            *led_buf_ptr++ = value;
        }
    }
    
    /* Reset the buffer pointer */
    led_buf_ptr = data->led_buf;
    
    /* Convert LED RGB values to PWM duty cycles for WS2812 protocol */
    for (i = 0; i < num_pixels * BYTES_PER_LED; i++) {
        byte_to_pwm_duty_cycles(*led_buf_ptr++, pwm_buf_ptr);
        pwm_buf_ptr += 8;
    }
    
    /* Add reset period (50 or more microseconds of zeros) */
    for (i = 0; i < WS2812_RESET_LEN; i++) {
        *pwm_buf_ptr++ = 0;
    }
    
    /* Set up DMA transfer */
    data->dma_blk_cfg.block_size = num_pixels * BITS_PER_LED + WS2812_RESET_LEN;
    data->dma_blk_cfg.source_address = (uint32_t)data->pwm_buffer;
    
    /* Configure DMA */
    ret = dma_config(data->dma_dev, config->dma_channel, &data->dma_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure DMA: %d", ret);
        return ret;
    }
    
    /* Mark transfer as started */
    data->transfer_complete = false;
    
    /* Start DMA transfer */
    ret = dma_start(data->dma_dev, config->dma_channel);
    if (ret < 0) {
        LOG_ERR("Failed to start DMA: %d", ret);
        return ret;
    }
    
    return 0;
}

/* Not implemented for PWM/DMA mode */
static int ws2812_pwm_dma_update_channels(const struct device *dev,
                                        uint8_t *channels,
                                        size_t num_channels)
{
    return -ENOTSUP;
}

/* Setup PWM hardware and DMA for WS2812 */
static int ws2812_pwm_dma_init(const struct device *dev)
{
    struct ws2812_pwm_dma_data *data = dev->data;
    const struct ws2812_pwm_dma_cfg *config = dev->config;
    int ret;
    
    /* Initialize the semaphore for DMA sync */
    k_sem_init(&data->sync_sem, 0, 1);
    
    /* Set up the data buffer */
    data->led_buf = led_buffer;
    data->transfer_complete = true;
    
    /* Directly configure STM32 timer for PWM output */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    /* Get timer instance - TIM1 is an advanced timer on STM32F4 */
    TIM_TypeDef *TIMx = TIM1;
    
    /* Configure the timer for PWM output */
    LL_TIM_SetPrescaler(TIMx, TIM_PRESCALER);
    LL_TIM_SetAutoReload(TIMx, WS2812_PERIOD - 1);
    LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);
    
    /* Enable PWM mode 1 and output compare preload on channel 1 */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
    
    /* Set initial duty cycle to zero */
    LL_TIM_OC_SetCompareCH1(TIMx, 0);
    
    /* Enable TIM1 outputs and set as master for DMA */
    LL_TIM_EnableAllOutputs(TIMx);
    LL_TIM_EnableDMAReq_UPDATE(TIMx);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);
    
    /* Enable the timer */
    LL_TIM_EnableCounter(TIMx);
#endif
    
    /* Get the DMA device */
    const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma2));
    if (!device_is_ready(dma)) {
        LOG_ERR("DMA device not ready");
        return -ENODEV;
    }
    data->dma_dev = dma;
    
    /* Configure DMA for MEMORY -> TIM1_CH1 */
    data->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
    data->dma_cfg.source_data_size = 2;  /* 16-bit */
    data->dma_cfg.dest_data_size = 2;    /* 16-bit */
    data->dma_cfg.source_burst_length = 1;
    data->dma_cfg.dest_burst_length = 1;
    data->dma_cfg.dma_callback = ws2812_dma_callback;
    data->dma_cfg.user_data = data;
    data->dma_cfg.complete_callback_en = true;
    data->dma_cfg.error_callback_en = true;
    data->dma_cfg.block_count = 1;
    data->dma_cfg.head_block = &data->dma_blk_cfg;
    
    /* Configure the DMA block */
    data->dma_blk_cfg.source_address = (uint32_t)data->pwm_buffer;
    data->dma_blk_cfg.dest_address = (uint32_t)&TIM1->CCR1;  /* Channel 1 compare register */
    data->dma_blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    data->dma_blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    
    LOG_INF("WS2812 PWM-DMA driver initialized for %d LEDs", config->num_leds);
    
    return 0;
}

/* LED strip driver API */
static const struct led_strip_driver_api ws2812_pwm_dma_api = {
    .update_rgb = ws2812_pwm_dma_update_rgb,
    .update_channels = ws2812_pwm_dma_update_channels,
};

/* WS2812 color mapping - usually GRB ordering (WS2812B) */
static const uint8_t ws2812_color_mapping[] = {
    LED_COLOR_ID_GREEN,
    LED_COLOR_ID_RED,
    LED_COLOR_ID_BLUE,
};

/* Define the device */
#define WS2812_PWM_DMA_DEVICE(id)                                          \
    static struct ws2812_pwm_dma_data ws2812_pwm_dma_##id##_data = {       \
        .pwm_buffer = { 0 },                                               \
    };                                                                     \
                                                                           \
    static const struct ws2812_pwm_dma_cfg ws2812_pwm_dma_##id##_cfg = {   \
        .dma_channel = 2,                                                  \
        .dma_slot = 6,                                                     \
        .num_leds = DT_INST_PROP(id, chain_length),                       \
        .color_mapping = ws2812_color_mapping,                            \
        .num_colors = 3,                                                  \
    };                                                                     \
                                                                           \
    DEVICE_DT_INST_DEFINE(id,                                             \
                        ws2812_pwm_dma_init,                              \
                        NULL,                                             \
                        &ws2812_pwm_dma_##id##_data,                      \
                        &ws2812_pwm_dma_##id##_cfg,                       \
                        POST_KERNEL,                                      \
                        90,                                               \
                        &ws2812_pwm_dma_api);

/* Create the device for each instance in the device tree */
DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DMA_DEVICE) 