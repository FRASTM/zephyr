/*
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>

static const struct device *my_real_time_clk = DEVICE_DT_GET(DT_ALIAS(rtc));

int main(void)
{
	if (!device_is_ready(my_real_time_clk)) {
		printk("RTC device not ready\n");
		return 0;
	}
	
	printk( "RTC device OK\r\n" );

	/* Continuously read the current date and time from the RTC */

        uint32_t ticks = 0U;
	struct rtc_time tm;

        while(1)
        {       
                rtc_get_time( my_real_time_clk, &tm );
                printk("ticks=%02d\n",tm.tm_sec);
                k_busy_wait( 10000000 );
        }
        
        return 0;
}
