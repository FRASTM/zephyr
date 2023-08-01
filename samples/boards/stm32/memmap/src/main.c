/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>

#define SPI_FLASH_TEST_REGION_OFFSET_0 0x0000
#define SPI_FLASH_TEST_REGION_OFFSET 0x10000
#define SPI_FLASH_SECTOR_SIZE        4096

#if defined(CONFIG_FLASH_STM32_OSPI) || defined(CONFIG_FLASH_STM32_QSPI)
#define SPI_FLASH_MULTI_SECTOR_TEST
#endif

uint8_t buf[32];
uint8_t buf1[4];
uint8_t buf2[4];
uint8_t buf3[4];

const struct device *const flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

static void *memmap_setup(void)
{
	zassert_true(device_is_ready(flash_dev));

	TC_PRINT("\n%s SPI flash testing in MemoryMapped\n", flash_dev->name);
	TC_PRINT("=================================\n");

	return NULL;
}

ZTEST(memmap, test_stm32_memmap_read_sector)
{
	uint32_t len = sizeof(buf);
	int rc;

	TC_PRINT("\nPerform test on single sector\n\n");
	memset(buf, 0, len);

	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET_0, buf, len);

	zassert_equal(rc, 0, "Flash read failed!");

	const uint8_t *rp = buf;
	const uint8_t *rpe = rp + len;

	while (rp < rpe) {
		TC_PRINT("%08x read %02x\n",
			(uint32_t)(SPI_FLASH_TEST_REGION_OFFSET_0 + (rp - buf)),
			*rp);
		++rp;
	}
}

#if defined SPI_FLASH_MULTI_SECTOR_TEST
ZTEST(memmap, test_stm32_memmap_multi_sector)
{
	uint32_t len1 = sizeof(buf1);
	uint32_t len2 = sizeof(buf2);
	uint32_t len3 = sizeof(buf3);
	int rc;
	size_t offs = 0;

	TC_PRINT("\nPerform test on multiple consecutive sectors\n\n");

	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */

	memset(buf1, 0, len1);
	memset(buf2, 0, len2);

	rc = flash_read(flash_dev, offs, buf1, len1);
	zassert_equal(rc, 0, "Flash read failed!");

	const uint8_t *rp = buf1;
	const uint8_t *rpe = rp + len1;

	TC_PRINT("Data read :\n");
	while (rp < rpe) {
		TC_PRINT(" %08x read %02x\n",
			(uint32_t)(offs + (rp - buf1)),
				*rp);
		++rp;
	}

	offs += SPI_FLASH_SECTOR_SIZE;

	rc = flash_read(flash_dev, offs, buf2, len2);
	zassert_equal(rc, 0, "Flash read failed!");

	const uint8_t *rp2 = buf2;
	const uint8_t *rpe2 = rp2 + len2;

	TC_PRINT("Data read :\n");
	while (rp2 < rpe2) {
		TC_PRINT(" %08x read %02x\n",
			(uint32_t)(offs + (rp2 - buf2)),
				*rp2);
		++rp2;
	}
		
	offs += SPI_FLASH_TEST_REGION_OFFSET;

	rc = flash_read(flash_dev, offs, buf3, len3);
	zassert_equal(rc, 0, "Flash read failed!");

	const uint8_t *rp3 = buf3;
	const uint8_t *rpe3 = rp3 + len3;

	TC_PRINT("Data read :\n");
	while (rp3 < rpe3) {
		TC_PRINT(" %08x read %02x\n",
			(uint32_t)(offs + (rp3 - buf3)),
				*rp3);
		++rp3;
	}
}
#endif

ZTEST_SUITE(memmap, NULL, memmap_setup, NULL, NULL, NULL);
