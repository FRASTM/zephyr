/*
 * Copyright (c) 2022 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_OSPI_STM32_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_OSPI_STM32_H_

/**
 * @name custom OSPI definition for the STM32 OctoSPI peripherals
 * Note that the possible combination is
 *  SPI mode in STR transfer rate
 *  OPI mode in STR transfer rate
 *  OPI mode in DTR transfer rate
 */

/* OSPI mode operating on 1 line or 8 lines */
/* 1 Cmd Line, 1 Address Line and 1 Data Line    */
#define STM32_OSPI_SPI_MODE                     1U
/* 8 Cmd Lines, 8 Address Lines and 8 Data Lines */
#define STM32_OSPI_OPI_MODE                     8U

/* OSPI mode operating on Single or Double Transfer Rate */
/* Single Transfer Rate */
#define STM32_OSPI_STR_TRANSFER                 1U
/* Double Transfer Rate */
#define STM32_OSPI_DTR_TRANSFER                 2U

/* Max Time value during reset or erase operation */
#define STM32_OSPI_RESET_MAX_TIME               100U
#define STM32_OSPI_BULK_ERASE_MAX_TIME          460000U
#define STM32_OSPI_SECTOR_ERASE_MAX_TIME        1000U
#define STM32_OSPI_SUBSECTOR_4K_ERASE_MAX_TIME  400U
#define STM32_OSPI_WRITE_REG_MAX_TIME           40U

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_OSPI_STM32_H_ */
