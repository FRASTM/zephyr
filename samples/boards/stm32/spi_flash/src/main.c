/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>

#define SPI_FLASH_TEST_REGION_OFFSET 0xf0000
#define SPI_FLASH_TEST_REGION_OFFSET_1 (SPI_FLASH_TEST_REGION_OFFSET + 0x1000)
#define SPI_FLASH_SECTOR_SIZE        4096

#if defined(CONFIG_FLASH_STM32_OSPI) || defined(CONFIG_FLASH_STM32_QSPI)
#define SPI_FLASH_MULTI_SECTOR_TEST
#endif

volatile uint8_t buf[4];
//volatile uint8_t expected[17*1024];
//uint16_t len = sizeof(expected);

//const uint8_t *re = (uint8_t *)0x008010000;

const uint8_t expected[] = { 0x55, 0xaa, 0xa5, 0x5a };
const size_t len = sizeof(expected);


void single_sector_test(const struct device *flash_dev)
{

	int rc;
/*
	for (uint16_t index = 0; index < len; index++) {
		expected[index] = *re;
		re++;
	}

*/

#if 1

	printf("\nPerform test on single sector");
	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	rc = flash_erase(flash_dev, 0x100000, 425984);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		printf("Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write\n");

	printf("Attempting to write %zu bytes\n", len);
	rc = flash_write(flash_dev, 0, expected, len);
	if (rc != 0) {
		printf("Flash write failed! %d\n", rc);
		return;
	}

#endif
	memset(buf, 0, 4);
	rc = flash_read(flash_dev, 0, buf, 4);
	if (rc != 0) {
		printf("Flash read failed! %d\n", rc);
		return;
	}

	if (memcmp(expected, buf, 4) == 0) {
		printf("Data read matches data written. Good!!\n");
	} else {
		const uint8_t *wp = expected;
		const uint8_t *rp = buf;
//		const uint8_t *rpe = rp + len;
		const uint8_t *rpe = rp + 4;

		printf("Data read does not match data written!!\n");
		while (rp < rpe) {
			printf("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(0 + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
	}
}

#if defined SPI_FLASH_MULTI_SECTOR_TEST
void multi_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = { 0xaa, 0x5a, 0xa5, 0x55 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	printf("\nPerform test on multiple consecutive sectors");

	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET_1 = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 * Erase 2 sectors for check for erase of consequtive sectors
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET_1, SPI_FLASH_SECTOR_SIZE * 2);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		/* Read the content and check for erased */
		memset(buf, 0, len);
		size_t offs = SPI_FLASH_TEST_REGION_OFFSET_1;

		while (offs < SPI_FLASH_TEST_REGION_OFFSET_1 + 2 * SPI_FLASH_SECTOR_SIZE) {
			rc = flash_read(flash_dev, offs, buf, len);
			if (rc != 0) {
				printf("Flash read failed! %d\n", rc);
				return;
			}
			if (buf[0] != 0xff) {
				printf("Flash erase failed at offset 0x%x got 0x%x\n",
				offs, buf[0]);
				return;
			}
			offs += SPI_FLASH_SECTOR_SIZE;
		}
		printf("Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write\n");

	size_t offs = SPI_FLASH_TEST_REGION_OFFSET_1;

	while (offs < SPI_FLASH_TEST_REGION_OFFSET_1 + 2 * SPI_FLASH_SECTOR_SIZE) {
		printf("Attempting to write %zu bytes at offset 0x%x\n", len, offs);
		rc = flash_write(flash_dev, offs, expected, len);
		if (rc != 0) {
			printf("Flash write failed! %d\n", rc);
			return;
		}

		memset(buf, 0, len);
		rc = flash_read(flash_dev, offs, buf, len);
		if (rc != 0) {
			printf("Flash read failed! %d\n", rc);
			return;
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
}
#endif

int main(void)
{
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);
		return 0;
	}

	printf("\n%s SPI flash testing\n", flash_dev->name);
	printf("==========================\n");

	single_sector_test(flash_dev);

#if defined SPI_FLASH_MULTI_SECTOR_TEST
//	multi_sector_test(flash_dev);
#endif
	return 0;
}
