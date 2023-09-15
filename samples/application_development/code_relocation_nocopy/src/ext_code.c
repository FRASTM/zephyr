/*
 * Copyright (c) 2022 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

uint32_t var_ext_sram_data = 10U;
const uint8_t ext_rodata[] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};


void function_in_ext_flash(void)
{
    int i;

	printk("Address of %s %p\n", __func__, &function_in_ext_flash);
	printk("Address of var_ext_sram_data %p (%d)\n", &var_ext_sram_data, var_ext_sram_data);
	printk("Address of ext_rodata %p:", &ext_rodata);

    for (i = 0; i < sizeof(ext_rodata) / sizeof(ext_rodata[0]); i++) {
        if(i % 8 == 0) printk("\n");
        printk("0x%02X ", ext_rodata[i]);
    }

    printk("\n");
}
