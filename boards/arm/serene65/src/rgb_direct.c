/*
 * Copyright (c) 2024 Matthijs Muller
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/dma.h>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_tim.h>
#include <stm32f4xx_ll_dma.h>

LOG_MODULE_REGISTER(rgb_direct, CONFIG_ZMK_LOG_LEVEL);

/* Number of LEDs in the strip (hardcoded) */
#define NUM_LEDS 1

/* WS2812 timing for 72MHz clock */
#define WS2812_PERIOD  90     /* 72MHz / 90 = 800kHz (1.25us period) */
#define WS2812_ONE     65     /* ~0.9us high (duty cycle ~72%) */
#define WS2812_ZERO    24     /* ~0.32us high (duty cycle ~26%) */
#define WS2812_RESET   6000   /* Number of zero pulses for reset (~80us) */

/* Buffer for LED data - 24 bits per LED plus reset period */
#define PWM_BUF_SIZE   ((NUM_LEDS * 24) + WS2812_RESET)
static uint16_t pwm_buf[PWM_BUF_SIZE];

/* Array for storing LED color data */
static uint8_t led_data[NUM_LEDS * 3];

/* DMA completion semaphore */
static struct k_sem dma_sem;
static volatile bool dma_busy = false;

/* Forward declarations */
void rgb_direct_init(void);
void rgb_direct_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void rgb_direct_update(void);
void rgb_direct_set_all(uint8_t r, uint8_t g, uint8_t b);
void rgb_direct_test(void);
static void prepare_pwm_data(void);

/* DMA completion callback */
static void dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
    LOG_DBG("DMA transfer complete with status %d", status);
    if (status != 0) {
        LOG_ERR("DMA transfer failed with status %d", status);
    }
    
    /* Ensure TIM1 and CCR1 are reset properly */
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_OC_SetCompareCH1(TIM1, 0);
    
    /* Wait a short time to ensure all data is sent */
    k_busy_wait(100);
    
    dma_busy = false;
    k_sem_give(&dma_sem);
}

/* Debug function to display DMA status */
static void debug_dma_status(const struct device *dma_dev, uint32_t channel)
{
    struct dma_status status;
    int result = dma_get_status(dma_dev, channel, &status);
    
    if (result == 0) {
        LOG_INF("DMA channel %d status: busy=%d, dir=%d, pending_len=%d", 
               channel, status.busy, status.dir, status.pending_length);
    } else {
        LOG_ERR("Failed to get DMA status: %d", result);
    }
}

/* Initialize the LED strip hardware */
void rgb_direct_init(void)
{
    LOG_INF("Initializing direct RGB control for %d LEDs", NUM_LEDS);
    
    /* Initialize semaphore for DMA synchronization */
    k_sem_init(&dma_sem, 0, 1);
    
    /* Clear LED data buffer */
    memset(led_data, 0, sizeof(led_data));
    
    /* Get DMA device */
    const struct device *dma_dev = DEVICE_DT_GET(DT_NODELABEL(dma2));
    if (!device_is_ready(dma_dev)) {
        LOG_ERR("DMA device not ready");
        return;
    }
    
    /* Print clock information */
    LOG_INF("SYSCLK_FREQ: %d Hz", SystemCoreClock);
    
    /* Configure GPIO PA8 for TIM1_CH1 - matching ChibiOS WS2812_PWM_PAL_MODE 1 */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
    
    /* First stop timer if it's running */
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_DisableAllOutputs(TIM1);
    
    /* Configure TIM1 for PWM output - Exactly matching ChibiOS PWMD1 */
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetAutoReload(TIM1, WS2812_PERIOD - 1);
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_SetCompareCH1(TIM1, 0);
    
    /* Enable TIM1 outputs and configure DMA */
    LL_TIM_EnableARRPreload(TIM1);
    
    /* Crucial: Enable DMA request for CAPTURE COMPARE 1 (not update) */
    LL_TIM_EnableDMAReq_CC1(TIM1);
    
    /* Configure DMA2 Stream5, Channel 6 for TIM1_CH1 - match ChibiOS exactly */
    LL_DMA_DeInit(DMA2, LL_DMA_STREAM_5);
    
    /* Clear all flags first */
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_TE5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);
    LL_DMA_ClearFlag_FE5(DMA2);
    LL_DMA_ClearFlag_DME5(DMA2);
    
    /* Explicit configuration matching STM32_DMA2_STREAM5, channel 6 */
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);  /* ChibiOS uses LINEAR mode, not circular */
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD);
    
    /* Configure NVIC for DMA */
    NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    
    /* Register the Zephyr DMA callback */
    struct dma_config dma_cfg = {0};
    
    dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
    dma_cfg.source_data_size = 2;  /* 16-bit */
    dma_cfg.dest_data_size = 2;    /* 16-bit */
    dma_cfg.source_burst_length = 1;
    dma_cfg.dest_burst_length = 1;
    dma_cfg.dma_callback = dma_callback;
    dma_cfg.user_data = NULL;
    dma_cfg.complete_callback_en = true;
    dma_cfg.error_callback_en = true;
    dma_cfg.block_count = 1;
    dma_cfg.dma_slot = 6;  /* Channel 6 for TIM1_CH1 */
    dma_cfg.channel_priority = 3; /* High priority */
    
    struct dma_block_config dma_block = {0};
    dma_block.block_size = PWM_BUF_SIZE;
    dma_block.source_address = (uint32_t)pwm_buf;
    dma_block.dest_address = (uint32_t)&TIM1->CCR1;
    dma_block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    dma_block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    
    dma_cfg.head_block = &dma_block;
    
    /* Register the DMA configuration */
    int ret = dma_config(dma_dev, 5, &dma_cfg);
    if (ret != 0) {
        LOG_ERR("Failed to configure DMA: %d", ret);
    } else {
        LOG_INF("DMA configured successfully");
    }
    
    /* Initialize LED buffer */
    for (int i = 0; i < PWM_BUF_SIZE; i++) {
        pwm_buf[i] = 0; /* Start with all LEDs off */
    }
    
    /* Initialize LED buffer with an immediate test pattern */
    /* First create a test pattern in the LED data buffer */
    for (int i = 0; i < NUM_LEDS; i++) {
        if (i % 4 == 0) {
            led_data[i * 3 + 0] = 0;    /* Red */
            led_data[i * 3 + 1] = 0;    /* Green */
            led_data[i * 3 + 2] = 255;  /* Blue */
        } else if (i % 4 == 1) {
            led_data[i * 3 + 0] = 0;    /* Red */
            led_data[i * 3 + 1] = 255;  /* Green */
            led_data[i * 3 + 2] = 0;    /* Blue */
        } else if (i % 4 == 2) {
            led_data[i * 3 + 0] = 255;  /* Red */
            led_data[i * 3 + 1] = 0;    /* Green */
            led_data[i * 3 + 2] = 0;    /* Blue */
        } else {
            led_data[i * 3 + 0] = 255;  /* Red */
            led_data[i * 3 + 1] = 255;  /* Green */
            led_data[i * 3 + 2] = 255;  /* Blue */
        }
    }
    
    /* Then prepare the PWM data based on the pattern */
    prepare_pwm_data();
    
    /* Configure DMA addresses and size directly */
    LL_DMA_ConfigAddresses(
        DMA2,
        LL_DMA_STREAM_5,
        (uint32_t)pwm_buf,           /* Source */
        (uint32_t)&TIM1->CCR1,       /* Destination */
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH
    );
    
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, PWM_BUF_SIZE);
    
    /* Reset timer counter */
    LL_TIM_SetCounter(TIM1, 0);
    
    /* Enable DMA transfer complete interrupt */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
    
    /* Start DMA and Timer */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    
    /* Output a single pulse to initialize the LEDs */
    rgb_direct_set_all(0, 0, 0);
    rgb_direct_update();
    
    LOG_INF("RGB direct control initialized - SYSCLK = %d Hz", SystemCoreClock);
}

/* Convert LED data to PWM values */
static void prepare_pwm_data(void)
{
    int i, bit, led_idx = 0, pwm_idx = 0;
    
    /* Convert RGB data to PWM values - GRB order for WS2812 */
    for (i = 0; i < NUM_LEDS; i++) {
        /* Process green first */
        for (bit = 7; bit >= 0; bit--) {
            pwm_buf[pwm_idx++] = (led_data[led_idx] & (1 << bit)) ? WS2812_ONE : WS2812_ZERO;
        }
        led_idx++;
        
        /* Process red next */
        for (bit = 7; bit >= 0; bit--) {
            pwm_buf[pwm_idx++] = (led_data[led_idx] & (1 << bit)) ? WS2812_ONE : WS2812_ZERO;
        }
        led_idx++;
        
        /* Process blue last */
        for (bit = 7; bit >= 0; bit--) {
            pwm_buf[pwm_idx++] = (led_data[led_idx] & (1 << bit)) ? WS2812_ONE : WS2812_ZERO;
        }
        led_idx++;
    }
    
    /* Add reset period - at least 50us of zeros */
    for (i = 0; i < WS2812_RESET; i++) {
        pwm_buf[pwm_idx++] = 0;
    }
}

/* Set LED color */
void rgb_direct_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < NUM_LEDS) {
        led_data[index * 3 + 0] = g;  /* WS2812 uses GRB color order */
        led_data[index * 3 + 1] = r;
        led_data[index * 3 + 2] = b;
    }
}

/* Update all LEDs with current data */
void rgb_direct_update(void)
{
    const struct device *dma_dev = DEVICE_DT_GET(DT_NODELABEL(dma2));
    
    /* Wait for any ongoing DMA transfer to complete */
    if (dma_busy) {
        LOG_DBG("Waiting for previous DMA transfer to complete");
        if (k_sem_take(&dma_sem, K_MSEC(100)) != 0) {
            LOG_ERR("Timeout waiting for DMA transfer to complete");
            /* Force reset of DMA busy flag if we timeout */
            dma_busy = false;
        }
    }
    
    /* Prepare PWM data */
    prepare_pwm_data();
    
    /* Temporarily stop DMA and timer */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
    LL_TIM_DisableCounter(TIM1);
    
    /* Mark transfer as started */
    dma_busy = true;
    
    /* Clear DMA flags */
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_TE5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);
    LL_DMA_ClearFlag_FE5(DMA2);
    LL_DMA_ClearFlag_DME5(DMA2);
    
    /* Make TIM1_CH1 output a zero */
    LL_TIM_OC_SetCompareCH1(TIM1, 0);
    
    /* Configure DMA addresses and size directly */
    LL_DMA_ConfigAddresses(
        DMA2,
        LL_DMA_STREAM_5,
        (uint32_t)pwm_buf,           /* Source */
        (uint32_t)&TIM1->CCR1,       /* Destination */
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH
    );
    
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, PWM_BUF_SIZE);
    
    /* Reset timer counter */
    LL_TIM_SetCounter(TIM1, 0);
    
    /* Enable DMA transfer complete interrupt */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
    
    /* Enable DMA stream and start timer */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    
    LOG_DBG("DMA transfer started: %d bytes", PWM_BUF_SIZE);
}

/* Set all LEDs to the same color */
void rgb_direct_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < NUM_LEDS; i++) {
        rgb_direct_set_led(i, r, g, b);
    }
}

/* Test function that cycles through some colors */
void rgb_direct_test(void)
{
    const struct device *dma_dev = DEVICE_DT_GET(DT_NODELABEL(dma2));
    int result;
    
    LOG_INF("RGB TEST STARTING - Initial DMA status:");
    debug_dma_status(dma_dev, 5);
    
    LOG_INF("RGB test - Single LED RED");
    rgb_direct_set_all(0, 0, 0);  // Clear all LEDs
    rgb_direct_set_led(0, 255, 0, 0);  // Set only first LED to red
    rgb_direct_update();
    k_sleep(K_MSEC(1000));
    
    LOG_INF("DMA status after single LED:");
    debug_dma_status(dma_dev, 5);
    
    /* Basic operation check - output additional diagnostic info */
    LOG_INF("TIM1 Status: ARR=%d, CCR1=%d, CNT=%d", 
           (int)LL_TIM_GetAutoReload(TIM1),
           (int)LL_TIM_OC_GetCompareCH1(TIM1),
           (int)LL_TIM_GetCounter(TIM1));
    
    LOG_INF("RGB test - ALL RED");
    rgb_direct_set_all(255, 0, 0);
    rgb_direct_update();
    k_sleep(K_MSEC(500));
    
    LOG_INF("RGB test - ALL GREEN");
    rgb_direct_set_all(0, 255, 0);
    rgb_direct_update();
    k_sleep(K_MSEC(500));
    
    LOG_INF("RGB test - ALL BLUE");
    rgb_direct_set_all(0, 0, 255);
    rgb_direct_update();
    k_sleep(K_MSEC(500));
    
    LOG_INF("RGB test - Alternating pattern");
    for (int i = 0; i < NUM_LEDS; i++) {
        if (i % 3 == 0) {
            rgb_direct_set_led(i, 255, 0, 0);  // Red
        } else if (i % 3 == 1) {
            rgb_direct_set_led(i, 0, 255, 0);  // Green
        } else {
            rgb_direct_set_led(i, 0, 0, 255);  // Blue
        }
    }
    rgb_direct_update();
    k_sleep(K_MSEC(1000));
    
    LOG_INF("RGB test - ALL WHITE");
    rgb_direct_set_all(255, 255, 255);
    rgb_direct_update();
    k_sleep(K_MSEC(1000));
    
    LOG_INF("RGB test - ALL OFF");
    rgb_direct_set_all(0, 0, 0);
    rgb_direct_update();
    
    LOG_INF("Final DMA status:");
    debug_dma_status(dma_dev, 5);
    
    LOG_INF("RGB TEST COMPLETE");
} 