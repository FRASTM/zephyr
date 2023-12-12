# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(spi_flash)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# flash_stm32_ospi instead runs entirely from SRAM after being copied there.
zephyr_code_relocate(FILES $ENV{ZEPHYR_BASE}/drivers/flash/flash_stm32_ospi.c LOCATION RAM)
zephyr_code_relocate(FILES $ENV{ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32u5xx/drivers/src/stm32u5xx_hal_ospi.c LOCATION RAM)
zephyr_code_relocate(FILES $ENV{ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32u5xx/drivers/src/stm32u5xx_hal_dma.c LOCATION RAM)
zephyr_code_relocate(FILES $ENV{ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32u5xx/drivers/src/stm32u5xx_hal_dma_ex.c LOCATION RAM)
