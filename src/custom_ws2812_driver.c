#include <zephyr.h>
#include <device.h>
#include <drivers/spi.h>

#define SPI_BUS_LABEL   "SPI_3"
#define SPI_MAX_FREQUENCY 2400000U

static const struct spi_config spi_cfg = {
    .frequency = SPI_MAX_FREQUENCY,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
    .cs = NULL,
};

void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue, uint16_t num_leds) {
    const struct device *spi_dev = device_get_binding(SPI_BUS_LABEL);
    if (!spi_dev) {
        printk("SPI device not found\n");
        return;
    }

    size_t buffer_size = 3 * num_leds;
    uint8_t buffer[buffer_size];

    for (int i = 0; i < num_leds; i++) {
        buffer[3 * i]     = green; // WS2812 expects colors in GRB order
        buffer[3 * i + 1] = red;
        buffer[3 * i + 2] = blue;
    }

    struct spi_buf tx_buf = {
        .buf = buffer,
        .len = buffer_size,
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    if (ret) {
        printk("SPI write failed: %d\n", ret);
    }
}

void ws2812_turn_all_blue(uint16_t num_leds) {
    ws2812_set_color(0x00, 0x00, 0xFF, num_leds);
}

void ws2812_turn_all_off(uint16_t num_leds) {
    ws2812_set_color(0x00, 0x00, 0x00, num_leds);
}