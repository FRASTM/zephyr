/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NOR_H__
#define __SPI_NOR_H__

#include <sys/util.h>

#define SPI_NOR_MAX_ID_LEN	3

/* Status register bits */
#define SPI_NOR_WIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NOR_WEL_BIT         BIT(1)  /* Write enable latch */

/* Flash opcodes */
#define SPI_NOR_CMD_WRSR        0x01    /* Write status register */
#define SPI_NOR_CMD_RDSR        0x05    /* Read status register */
#define SPI_NOR_CMD_READ        0x03    /* Read data */
#define SPI_NOR_CMD_WREN        0x06    /* Write enable */
#define SPI_NOR_CMD_WRDI        0x04    /* Write disable */
#define SPI_NOR_CMD_PP          0x02    /* Page program */
#define SPI_NOR_CMD_SE          0x20    /* Sector erase */
#define SPI_NOR_CMD_BE_32K      0x52    /* Block erase 32KB */
#define SPI_NOR_CMD_BE          0xD8    /* Block erase */
#define SPI_NOR_CMD_CE          0xC7    /* Chip erase */
#define SPI_NOR_CMD_RDID        0x9F    /* Read JEDEC ID */
#define SPI_NOR_CMD_ULBPR       0x98    /* Global Block Protection Unlock */
#define SPI_NOR_CMD_4BA         0xB7    /* Enter 4-Byte Address Mode */
#define SPI_NOR_CMD_DPD         0xB9    /* Deep Power Down */
#define SPI_NOR_CMD_RDPD        0xAB    /* Release from Deep Power Down */
#define SPI_NOR_CMD_CFGREG2     0x72    /* Read config register 2 */
#define SPI_NOR_OCMD_RDSR       0x05FA  /* Octal Read status register */
#define SPI_NOR_OCMD_WREN       0x06F9  /* Octal Write enable */

/* Page, sector, and block size are standard, not configurable. */
#define SPI_NOR_PAGE_SIZE    0x0100U
#define SPI_NOR_SECTOR_SIZE  0x1000U
#define SPI_NOR_BLOCK_SIZE   0x10000U

/* Auto-polling values */
#define SPI_NOR_WREN_MATCH    0x02
#define SPI_NOR_WREN_MASK     0x02

#define SPI_NOR_MEM_RDY_MATCH 0x00
#define SPI_NOR_MEM_RDY_MASK  0x01

#define SPI_NOR_AUTO_POLLING_INTERVAL       0x10

/* Memory registers address */
#define SPI_NOR_REG2_ADDR1    0x0000000
#define SPI_NOR_CR2_STR_OPI_EN          0x01

#define SPI_NOR_REG2_ADDR3           0x00000300
#define SPI_NOR_CR2_DUMMY_CYCLES_66MHZ      0x07

/* Test whether offset is aligned to a given number of bits. */
#define SPI_NOR_IS_ALIGNED(_ofs, _bits) (((_ofs) & BIT_MASK(_bits)) == 0)
#define SPI_NOR_IS_SECTOR_ALIGNED(_ofs) SPI_NOR_IS_ALIGNED(_ofs, 12)

#endif /*__SPI_NOR_H__*/
