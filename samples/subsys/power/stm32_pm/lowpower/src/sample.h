/*
 * Copyright (c) 2018 Intel Corporation.
 * Copyright (c) 2019 STMicroelectronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <power/power.h>
#include <string.h>
#include <soc.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#ifdef CONFIG_SOC_SERIES_STM32L4X
 #include <stm32l4xx_ll_pwr.h>
 #include <stm32l4xx_ll_bus.h>
#endif
#ifdef CONFIG_SOC_SERIES_STM32WBX
#include <stm32wbxx_ll_pwr.h>
#include <stm32wbxx_ll_bus.h>
#endif

/* change this to use another GPIO port */
#define PORT		DT_ALIAS_SW0_GPIOS_CONTROLLER
#define SW		DT_ALIAS_SW0_GPIOS_PIN
#define PORT0		DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED0		DT_ALIAS_LED0_GPIOS_PIN

#define LOW		0
#define HIGH		1
#define LED_ON		HIGH
#define LED_OFF		LOW
