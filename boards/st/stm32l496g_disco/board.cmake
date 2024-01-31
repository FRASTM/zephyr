# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32L496AG" "--speed=4000")

if(CONFIG_STM32_MEMMAP)
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
board_runner_args(stm32cubeprogrammer "--hex-file=${ZEPHYR_BASE}/build/zephyr/zephyr.hex")
board_runner_args(stm32cubeprogrammer "--extload=MX25R6435F_STM32L496G-DISCO.stldr")
else()
board_runner_args(stm32cubeprogrammer "--erase" "--port=swd" "--reset-mode=hw" )
endif()

include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
