/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/rtc.h>

/* Fri Jan 01 2021 13:29:50 GMT+0000 */
#define RTC_TEST_SET_TIME		      1609507790

static const struct device *rtc = DEVICE_DT_GET(DT_NODELABEL(rtc));

static void set_calendar(void)
{
	int ret;
	time_t timer_set;
	struct rtc_time time_set;

	/* Initialize RTC time to set */
	timer_set = RTC_TEST_SET_TIME;

	gmtime_r(&timer_set, (struct tm *)(&time_set));

	time_set.tm_isdst = -1;
	time_set.tm_nsec = 0;

	/* Set RTC time */
	ret = rtc_set_time(rtc, &time_set);
	if (ret) {
		printk("Failed to set calendar\n");
	}
}


int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(rtc));
	struct rtc_time calendar_get;

	if (!device_is_ready(dev)) {
		printf("RTC device not ready\n");
		return -1;
	}
	
	/* Get RTC time after first init */
	if (rtc_get_time(rtc, &calendar_get)) {
		set_calendar();
		printk("\nSetting calendar\n\n");
	}

	memset(&calendar_get, 0xFF, sizeof(calendar_get));
	
	while(1) {
		/* Wait for a while */
		k_sleep(K_MSEC(500));

		/* Get RTC time */
		if (rtc_get_time(rtc, &calendar_get)) {
			printk("Failed to get calendar\n");
			return -1;
		}
		printk("  %02d:%02d:%02d - \r", calendar_get.tm_hour, calendar_get.tm_min, calendar_get.tm_sec);

	};

	return 0;
}
