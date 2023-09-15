# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(code_relocation_nocopy)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
target_sources_ifdef(CONFIG_NRFX_QSPI app PRIVATE boards/nrf5340dk_nrf5340_cpuapp/ext_mem_init.c)

# Run ext_code from the external flash (XIP). Place ext_rodata (const vars) in external flash. No need to copy.
# Note that external flash must be loaded with a capable external loader
if(CONFIG_FLASH_STM32_OSPI)
zephyr_code_relocate(FILES src/ext_code.c LOCATION XSPI_TEXT NOCOPY)
zephyr_code_relocate(FILES src/ext_code.c LOCATION XSPI_RODATA NOCOPY)
else()
zephyr_code_relocate(FILES src/ext_code.c LOCATION EXTFLASH_TEXT NOCOPY)
endif()

# But still relocate (copy) the data to RAM
zephyr_code_relocate(FILES src/ext_code.c LOCATION RAM_DATA)

# sram_code instead runs entirely from SRAM after being copied there.
zephyr_code_relocate(FILES src/sram_code.c LOCATION RAM)
