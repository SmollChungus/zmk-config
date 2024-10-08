#include <init.h>

extern void ws2812_turn_all_blue(uint16_t num_leds);

static int ws2812_init(const struct device *dev) {
    ARG_UNUSED(dev);

    // Turn all LEDs blue
    ws2812_turn_all_blue(3);

    return 0;
}

SYS_INIT(ws2812_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);