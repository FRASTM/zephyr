/*
 * Copyright (c) 2024 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>


#define SPI_FLASH_TEST_REGION_OFFSET 0xff000
//#define SPI_FLASH_TEST_REGION_OFFSET 0x10000
//#define SPI_FLASH_TEST_REGION_OFFSET 0x0000

#define SPI_FLASH_SECTOR_SIZE        4096

#define SPI_FLASH_MULTI_SECTOR_TEST

int single_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	printf("\nPerform test on single sector");
	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase at 0x%x\n", SPI_FLASH_TEST_REGION_OFFSET);

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET,
			 SPI_FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		printf("Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write at 0x%x\n", SPI_FLASH_TEST_REGION_OFFSET);

	printf("Attempting to write %zu bytes\n", len);
	rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printf("Flash write failed! %d\n", rc);
		return 1;
	}

	memset(buf, 0, len);
	printf("\nTest 3: Flash read at 0x%x\n", SPI_FLASH_TEST_REGION_OFFSET);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printf("Flash read failed! %d\n", rc);
		return 1;
	}

	if (memcmp(expected, buf, len) == 0) {
		printf("Data read matches data written. Good!!\n");
	} else {
		const uint8_t *wp = expected;
		const uint8_t *rp = buf;
		const uint8_t *rpe = rp + len;

		printf("Data read does not match data written!!\n");
		while (rp < rpe) {
			printf("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
	}
	return rc;
}


int single_sector_read_test(const struct device *flash_dev)
{
	const size_t len = 16;
	uint8_t buf[len];
	int rc;

	printf("\nPerform test read on single sector at 0x%x\n", SPI_FLASH_TEST_REGION_OFFSET);

	memset(buf, 0, len);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printf("Flash read failed! %d\n", rc);
		return 1;
	}

	const uint8_t *rp = buf;
	const uint8_t *rpe = rp + len;

	printf("Data read at is \n");
	while (rp < rpe) {
		printf("Read at %08x read %02x\n",
		       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *rp);
		++rp;
	}

	return rc;
}

#if defined SPI_FLASH_MULTI_SECTOR_TEST
int multi_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	printf("\nPerform test on multiple consecutive sectors");

	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase from 0x%x\n", SPI_FLASH_TEST_REGION_OFFSET);

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 * Erase 2 sectors for check for erase of consecutive sectors
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, SPI_FLASH_SECTOR_SIZE * 2);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		/* Read the content and check for erased */
		memset(buf, 0, len);
		size_t offs = SPI_FLASH_TEST_REGION_OFFSET;

		while (offs < SPI_FLASH_TEST_REGION_OFFSET + 2 * SPI_FLASH_SECTOR_SIZE) {
			rc = flash_read(flash_dev, offs, buf, len);
			if (rc != 0) {
				printf("Flash read failed! %d\n", rc);
				return 1;
			}
			if (buf[0] != 0xff) {
				printf("Flash erase failed at offset 0x%x got 0x%x\n",
				offs, buf[0]);
				return 1;
			}
			offs += SPI_FLASH_SECTOR_SIZE;
		}
		printf("Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write from 0x%x\n", SPI_FLASH_TEST_REGION_OFFSET);
	size_t offs = SPI_FLASH_TEST_REGION_OFFSET;

	while (offs < SPI_FLASH_TEST_REGION_OFFSET + 2 * SPI_FLASH_SECTOR_SIZE) {
		printf("Attempting to write %zu bytes at offset 0x%x\n", len, offs);
		rc = flash_write(flash_dev, offs, expected, len);
		if (rc != 0) {
			printf("Flash write failed! %d\n", rc);
			return 1;
		}

		memset(buf, 0, len);
		printf("Attempting to read %zu bytes at offset 0x%x\n", len, offs);
		rc = flash_read(flash_dev, offs, buf, len);
		if (rc != 0) {
			printf("Flash read failed! %d\n", rc);
			return 1;
		}

		if (memcmp(expected, buf, len) == 0) {
			printf("Data read matches data written. Good!!\n");
		} else {
			const uint8_t *wp = expected;
			const uint8_t *rp = buf;
			const uint8_t *rpe = rp + len;

			printf("Data read does not match data written!!\n");
			while (rp < rpe) {
				printf("%08x wrote %02x read %02x %s\n",
					(uint32_t)(offs + (rp - buf)),
					*wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
				++rp;
				++wp;
			}
		}
		offs += SPI_FLASH_SECTOR_SIZE;
	}
	return rc;
}
#endif

int main(void)
{
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(flash0));

	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);
		return 1;
	}

	printf("\n%s SPI flash testing\n", flash_dev->name);
	printf("==========================\n");

	if (single_sector_read_test(flash_dev)) {
		return 1;
	}

	if (single_sector_test(flash_dev)) {
		return 1;
	}
#if defined SPI_FLASH_MULTI_SECTOR_TEST
	if (multi_sector_test(flash_dev)) {
		return 1;
	}
#endif
	printf("==========================\n");
	return 0;
}
