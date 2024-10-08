# Add your custom driver and initialization source files
include($ENV{ZMK_BASE}/app/CMakeLists.txt)
target_sources(app PRIVATE src/custom_ws2812_driver.c src/init_ws2812.c)