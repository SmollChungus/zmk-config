# SPDX-License-Identifier: MIT

# STM32F411 configuration
board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse")
include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
