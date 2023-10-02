/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>


#if DT_NODE_HAS_STATUS(DT_ALIAS(i2c_0), okay)
#define I2C_DEV_NODE	DT_ALIAS(i2c_0)
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_1), okay)
#define I2C_DEV_NODE	DT_ALIAS(i2c_1)
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_2), okay)
#define I2C_DEV_NODE	DT_ALIAS(i2c_2)
#else
#error "Please set the correct I2C device"
#endif

struct i2c_config_timing {
	/* i2c peripheral clock in Hz */
	uint32_t periph_clock;
	/* i2c bus speed in Hz */
	uint32_t i2c_speed;
	/* I2C_TIMINGR register value of i2c v2 peripheral */
	uint32_t timing_setting;
};

int main(void)
{
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
	uint32_t i2c_cfg_tmp = 0;
	struct i2c_config_timing cfg_timings;

	if (!device_is_ready(i2c_dev)) {
		printk("I2C device is not ready\n");
		return -1;
	}

	/* 2. Verify i2c_get_config() */
	if (i2c_get_config(i2c_dev, (uint32_t *)&cfg_timings)) {
		printk("I2C get_config failed\n");
		return -1;
	}

	if (cfg_timings.i2c_speed != DT_PROP(I2C_DEV_NODE, clock_frequency)) {
		printk("I2C speed mismatch\n");
		return -1;
	}
	printk("I2C timing value, report to the DTS : \n");
	printk("timings = <%d", cfg_timings.periph_clock);

	/* I2C BIT RATE */
	if (DT_PROP(I2C_DEV_NODE, clock_frequency) == 100000) {
		printk(" I2C_BITRATE_STANDARD ");
	}
	else if (DT_PROP(I2C_DEV_NODE, clock_frequency) == 400000) {
		printk(" I2C_BITRATE_FAST ");
	}
	else if (DT_PROP(I2C_DEV_NODE, clock_frequency) == 1000000) {
		printk(" I2C_SPEED_FAST_PLUS ");
	}

	printk(" 0x%X>;\n", cfg_timings.timing_setting);


	return 0;
}
