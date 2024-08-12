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

#if defined(CONFIG_FLASH_STM32_OSPI) || \
	defined(CONFIG_FLASH_STM32_QSPI) || \
	defined(CONFIG_FLASH_STM32_XSPI)
#define SPI_FLASH_MULTI_SECTOR_TEST
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(jedec_spi_nor)
#define SPI_FLASH_COMPAT jedec_spi_nor
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_qspi_nor)
#define SPI_FLASH_COMPAT st_stm32_qspi_nor
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_ospi_nor)
#define SPI_FLASH_COMPAT st_stm32_ospi_nor
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_xspi_nor)
#define SPI_FLASH_COMPAT st_stm32_xspi_nor
#else
#define SPI_FLASH_COMPAT invalid
#endif

#define SPI_FLASH_TEST_REGION_OFFSET	0x000
#define SPI_FLASH_SECTOR_SIZE		4096
#define RD_LEN				256

void single_sector_test(const struct device *flash_dev)
{
	uint8_t buf[RD_LEN];
	int rc;

	printf("MemoryMapped read on single sector\n");

	/* Check erased pattern */
	memset(buf, 0, RD_LEN);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, RD_LEN);
	if (rc != 0) {
		printf("Flash read failed! %d\n", rc);
		return;
	}

	const uint8_t *rp = buf;
	const uint8_t *rpe = rp + 0x10; /* Read the first 16 bytes */

	while ((rp < rpe)) {
		printf("%02x read %02x\n",
		       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
		       *rp);
		++rp;
	}
}



int main(void)
{
	const struct device *flash_dev = DEVICE_DT_GET_ONE(SPI_FLASH_COMPAT);

	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);
		return 0;
	}

	printf("\n%s SPI flash testing in Memory Mapped mode\n", flash_dev->name);
	printf("==========================\n");
	printf("(use STM32CubeProgrammer to fill the external flash memory)\n");

	single_sector_test(flash_dev);

	return 0;
}
