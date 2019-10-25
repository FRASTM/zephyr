/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2019 STMicroelectronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample application which demonstrates the low power modes.
 * The low power state 'SYS_POWER_STATE_DEEP_SLEEP_1' can be changed
 * in the sys_pm_force_power_state(), to access sleep or deep sleep modes
 * The user push button is configures to exit from deep sleep mode.
 */

#include "sample.h"

#define WITHIN_ERROR(var, target, epsilon)       \
		(((var) >= (target)) && ((var) <= (target) + (epsilon)))

struct device *gpio_port, *gpio_port0;

/* 1000 msec = 1 sec */
#define WAIT_TIME 3000
#define SLEEP_TIME 5000

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
		LL_PWR_ClearFlag_WU();
		gpio_pin_write(gpio_port0, LED0, LED_ON);
		k_busy_wait(100);
		gpio_pin_write(gpio_port0, LED0, LED_OFF);
}

static struct gpio_callback gpio_cb;


/* Application main Thread */
void main(void)
{

	printk("\n\n*** Low Power Demo %s ***\n", CONFIG_BOARD);

	gpio_port = device_get_binding(PORT);

	/* Configure Button to exit lower power mode
	 * (no Pull up on PA0)
	 */
	gpio_pin_configure(gpio_port, SW,
			   GPIO_DIR_IN | GPIO_INT |
			   GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);

	gpio_init_callback(&gpio_cb, button_pressed, BIT(SW));

	gpio_add_callback(gpio_port, &gpio_cb);
	gpio_pin_enable_callback(gpio_port, SW);

	/* Configure LEDs */
	gpio_port0 = device_get_binding(PORT0);
	gpio_pin_configure(gpio_port0, LED0, GPIO_DIR_OUT);
	gpio_pin_write(gpio_port0, LED0, LED_ON);

	/*
	 * Start the demo.
	 */

	while (1) {

		k_busy_wait(WAIT_TIME);

		printk("<-- Enabling %s state --->\n",
				       STRINGIFY(SYS_POWER_STATE_SLEEP_2));
		sys_pm_force_power_state(SYS_POWER_STATE_SLEEP_2);

		k_sleep(SLEEP_TIME);

	}
}

/*
 * Application defined function for doing any target specific operations
 * for low power entry.
 */
void sys_pm_notify_power_state_entry(enum power_states state)
{

	switch (state) {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	case SYS_POWER_STATE_SLEEP_1:
		printk("--> Entering to SYS_POWER_STATE_SLEEP_1 state.\n");
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	case SYS_POWER_STATE_SLEEP_2:
		printk("--> Entering to SYS_POWER_STATE_SLEEP_2 state.\n");
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	case SYS_POWER_STATE_SLEEP_3:
		printk("--> Entering to SYS_POWER_STATE_SLEEP_3 state.\n");
		break;
 #endif
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */
#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_1
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		printk("--> Entering to SYS_POWER_STATE_DEEP_SLEEP_1 state.\n");

#if defined(LL_APB1_GRP1_PERIPH_PWR)
		/*
		 * Enable Wakeup Pin PWR_WAKEUP_PIN2 connected to PC.13.
		 */
		if (LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_PWR) == 0) {
			LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
		}
#endif
		LL_PWR_ClearFlag_WU();
		LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PIN2);
		LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN2);
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_2
	case SYS_POWER_STATE_DEEP_SLEEP_2:
		printk("--> Entering to SYS_POWER_STATE_DEEP_SLEEP_2 state.\n");

#if defined(LL_APB1_GRP1_PERIPH_PWR)
		/*
		 * Enable Wakeup Pin PWR_WAKEUP_PIN2 connected to PC.13.
		 */
		if (LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_PWR) == 0) {
			LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
		}
#endif
		LL_PWR_ClearFlag_WU();
		LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PIN2);
		LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN2);
		/*
		 * This power state is implemented by this sample.
		 * Perform enter to the low power state by turning off LD.
		 */
		gpio_pin_write(gpio_port0, LED0, LED_OFF);
		break;
 #endif
#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
	default:
		printk("Unsupported power state entry %u", state);
		break;
	}
	gpio_pin_write(gpio_port0, LED0, LED_OFF);
	/* add short delay before going to Lowpower mode*/
	k_busy_wait(200);
}

/*
 * Application defined function for doing any target specific operations
 * for low power exit.
 */
void sys_pm_notify_power_state_exit(enum power_states state)
{
	k_busy_wait(200);
	switch (state) {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	case SYS_POWER_STATE_SLEEP_1:
		printk("--> Exited from SYS_POWER_STATE_SLEEP_1 state.\n");
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	case SYS_POWER_STATE_SLEEP_2:
		printk("--> Exited from SYS_POWER_STATE_SLEEP_2 state.\n");
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	case SYS_POWER_STATE_SLEEP_3:
		printk("--> Exited from SYS_POWER_STATE_SLEEP_3 state.\n");
		break;
 #endif
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */
#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_1
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		printk("--> Exited from SYS_POWER_STATE_DEEP_SLEEP_1 state.\n");
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_2
	case SYS_POWER_STATE_DEEP_SLEEP_2:
		printk("--> Exited from SYS_POWER_STATE_DEEP_SLEEP_2 state.\n");
		break;
 #endif
#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
	default:
		printk("Unsupported power state exit %u", state);
		break;
	}
	/* Perform exit from the low power state by turning on LEDs */
	gpio_pin_write(gpio_port0, LED0, LED_ON);
}
