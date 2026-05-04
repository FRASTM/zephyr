/*
 * Copyright (c) 2024, Muhammad Waleed Badar
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/counter.h>

const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));

static int set_date_time(const struct device *rtc)
{
	int ret = 0;
	struct rtc_time tm = {
		.tm_year = 2026 - 1900,
		.tm_mon = 5,
		.tm_mday = 19,
		.tm_hour = 4,
		.tm_min = 19,
		.tm_sec = 0,
	};

	ret = rtc_set_time(rtc, &tm);
	if (ret < 0) {
		printk("Cannot write date time: %d\n", ret);
		return ret;
	}
	return ret;
}

static int get_date_time(const struct device *rtc)
{
	int ret = 0;
	struct rtc_time tm;

	ret = rtc_get_time(rtc, &tm);
	if (ret < 0) {
		printk("Cannot read date time: %d\n", ret);
		return ret;
	}

	printk("ticks=%02d\n",tm.tm_sec);

	return ret;
}

int main(void)
{
	/* Check if the RTC is ready */
	if (!device_is_ready(rtc)) {
		printk("Device is not ready\n");
		return 0;
	}
uint32_t time = 0;
	set_date_time(rtc);

	/* Continuously read the current date and time from the RTC */
	while(1) {
		(void)counter_get_value(rtc, &time);
		printk("ticks=%d\n",time);
		k_busy_wait( 10000000 );
	};

	return 0;
}

