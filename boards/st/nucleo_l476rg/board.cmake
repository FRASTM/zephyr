# SPDX-License-Identifier: Apache-2.0

# keep first
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
board_runner_args(jlink "--device=STM32L476RG" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
# keep first
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
