/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <power.h>
#include <misc/printk.h>
#include <string.h>
#include <device.h>
#include <gpio.h>

#define WAIT_DELAY_S			(3)
#define BUSY_WAIT_DELAY_US		(WAIT_DELAY_S * USEC_PER_SEC)

static char *state_name_table[] = {
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	STRINGIFY(SYS_POWER_STATE_SLEEP_1),
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	STRINGIFY(SYS_POWER_STATE_SLEEP_2),
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	STRINGIFY(SYS_POWER_STATE_SLEEP_3),
#endif
};

static s32_t delay_table[] = {
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_1 + 1,
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_2 + 1,
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_3 + 1,
#endif
};

struct device *gpio_port0;
#define PORT0		DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED0		DT_ALIAS_LED0_GPIOS_PIN

#define LOW		0
#define HIGH		1
#define LED_ON		HIGH
#define LED_OFF		LOW

void sys_pm_notify_power_state_entry(enum power_states state)
{
	printk("Entering Power State %s\n", state_name_table[state]);
	k_busy_wait(200);
}

void sys_pm_notify_power_state_exit(enum power_states state)
{
	k_busy_wait(200);
	printk("Exiting Power State %s\n", state_name_table[state]);
}

void main(void)
{
	/* Configure LEDs */
	gpio_port0 = device_get_binding(PORT0);
	gpio_pin_configure(gpio_port0, LED0, GPIO_DIR_OUT);
	gpio_pin_write(gpio_port0, LED0, LED_OFF);

	printk("\n*** OS Power Management Demo on %s ***\n", CONFIG_SOC_SERIES);

	for (int i = 0; i < ARRAY_SIZE(delay_table); i++) {
		gpio_pin_write(gpio_port0, LED0, LED_ON);
		printk("\nApp doing busy wait for %d s...\n", WAIT_DELAY_S);
		k_busy_wait(BUSY_WAIT_DELAY_US);

		/* Create Idleness to make Idle thread run */
		s32_t delay = delay_table[i];

		printk("Going to sleep for %d ms\n", delay);
		gpio_pin_write(gpio_port0, LED0, LED_OFF);
		k_sleep(delay);
	}

	printk("OS managed Power Management Test completed\n");
	gpio_pin_write(gpio_port0, LED0, LED_ON);
	/* infinite loop to avoid a further sleep entry
	 * resulting in a BusFault
	 */
	for ( ; ; ) {
	}
}

