#include <zephyr.h>
#include <device.h>
#include <drivers/spi.h>
#include <string.h>

#define SPI_BUS_LABEL   "SPI_3"
#define SPI_MAX_FREQUENCY 4000000U

static const struct spi_config spi_cfg = {
    .frequency = SPI_MAX_FREQUENCY,
    .operation = SPI_OP_MODE_MASTER |    // Master mode
                 SPI_WORD_SET(8)      |  // 8 bits per word
                 SPI_TRANSFER_MSB    |   // MSB first
                 SPI_MODE_0,             // SPI mode 0
    .slave = 0,
    .cs = NULL,
};

void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue, uint16_t num_leds) {
    const struct device *spi_dev = device_get_binding(SPI_BUS_LABEL);
    if (!spi_dev) {
        printk("SPI device not found\n");
        return;
    }

    // Each color component is 8 bits, total of 24 bits per LED
    // After encoding, each bit becomes one SPI byte
    size_t buffer_size = 24 * num_leds;
    uint8_t buffer[buffer_size];

    for (int i = 0; i < num_leds; i++) {
        // WS2812 expects colors in GRB order
        uint8_t colors[3] = {green, red, blue};

        for (int color_idx = 0; color_idx < 3; color_idx++) {
            uint8_t color = colors[color_idx];
            for (int bit = 7; bit >= 0; bit--) {
                uint8_t spi_byte;
                if (color & (1 << bit)) {
                    // Logical "1"
                    spi_byte = 0x70; // 01110000
                } else {
                    // Logical "0"
                    spi_byte = 0x40; // 01000000
                }
                buffer[i * 24 + color_idx * 8 + (7 - bit)] = spi_byte;
            }
        }
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

    // Reset pulse
    k_busy_wait(80); // Wait for 80µs to latch the data
}

void ws2812_turn_all_blue(uint16_t num_leds) {
    ws2812_set_color(0x00, 0x00, 0xFF, num_leds);
}

void ws2812_turn_all_off(uint16_t num_leds) {
    ws2812_set_color(0x00, 0x00, 0x00, num_leds);
}