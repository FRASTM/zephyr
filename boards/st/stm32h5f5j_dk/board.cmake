# SPDX-License-Identifier: Apache-2.0

# keep first
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
if(CONFIG_STM32_MEMMAP OR (CONFIG_XIP AND CONFIG_BOOTLOADER_MCUBOOT))
  board_runner_args(stm32cubeprogrammer "--extload=MX25LM51245G_STM32H5F5J-DK.stldr")
endif()

# keep first
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
# FIXME: official openocd runner not yet available.
