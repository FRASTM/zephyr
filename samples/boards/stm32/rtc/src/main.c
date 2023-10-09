/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/counter.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct device* my_real_time_clk = DEVICE_DT_GET(DT_NODELABEL(rtc));

/*void main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);
	}
}*/

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		printk( "LED device not ready\r\n");
		return -1;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

	if (device_is_ready(my_real_time_clk))
	{
		printk("RTC device OK\r\n");
	}
	else
	{
		printk("RTC device NOT OK\r\n");
		return -1;
	}
	
	uint32_t ticks = 0U;

	while(1)
	{
		gpio_pin_toggle_dt(&led);
		counter_get_value( my_real_time_clk, &ticks );
		printk("ticks=%d\n",ticks);
		k_busy_wait(10000000);
//		k_msleep(10000);
	}
}













