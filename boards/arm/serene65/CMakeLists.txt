zephyr_library()

zephyr_library_sources(src/hall_effect.c)
zephyr_library_sources(src/matrix_hall_effect.c)

# Add ZMK module includes - use correct absolute paths
zephyr_include_directories(${ZEPHYR_BASE}/../app/module/include)