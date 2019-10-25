/*
 * Copyright (c) 2019 STMicroelectronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sample.h"

#define BUSY_WAIT_DELAY_US	(10 * 1000000)

#define LPS1_STATE_ENTER_TO		10
#define LPS2_STATE_ENTER_TO		30
#define DEEP_SLEEP_STATE_ENTER_TO	90

#define DEMO_DESCRIPTION	\
	"Demo Description\n"	\
	"Application creates idleness, due to which System Idle Thread is\n"\
	"scheduled and it enters into various Low Power States.\n"

struct device *gpio_port, *gpio_port0;

/* Application main Thread */
void main(void)
{
	u32_t level = 0U;

	printk("\n\n*** Power Management Demo on %s ***\n", CONFIG_BOARD);
	printk(DEMO_DESCRIPTION);

	gpio_port = device_get_binding(PORT);

	/* Configure Button as deep sleep trigger event */
	/* and Configure Button as wake source from deep sleep */
	gpio_pin_configure(gpio_port, SW,
			   GPIO_DIR_IN | GPIO_INT |
			   GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);

	gpio_pin_enable_callback(gpio_port, SW);

	/* Configure LEDs */
	/* LED 0 code the low power sleep mode 1, 2, 3 */
	gpio_port0 = device_get_binding(PORT0);
	gpio_pin_configure(gpio_port0, LED_0, GPIO_DIR_OUT);
	gpio_pin_write(gpio_port0, LED_0, LED_ON);

	/*
	 * Start the demo.
	 */
	for (int i = 1; i <= 8; i++) {
		unsigned int sleep_seconds;

		switch (i) {
		case 3:
			printk("\n<-- Disabling %s state --->\n",
					STRINGIFY(SYS_POWER_STATE_SLEEP_3));
			sys_pm_ctrl_disable_state(SYS_POWER_STATE_SLEEP_3);
			break;

		case 5:
			printk("\n<-- Enabling %s state --->\n",
				       STRINGIFY(SYS_POWER_STATE_SLEEP_3));
			sys_pm_ctrl_enable_state(SYS_POWER_STATE_SLEEP_3);

			printk("<-- Disabling %s state --->\n",
					STRINGIFY(SYS_POWER_STATE_SLEEP_2));
			sys_pm_ctrl_disable_state(SYS_POWER_STATE_SLEEP_2);
			break;

		case 7:
			printk("\n<-- Enabling %s state --->\n",
				       STRINGIFY(SYS_POWER_STATE_SLEEP_2));
			sys_pm_ctrl_enable_state(SYS_POWER_STATE_SLEEP_2);

			printk("<-- Forcing %s state --->\n",
				       STRINGIFY(SYS_POWER_STATE_SLEEP_3));
			sys_pm_force_power_state(SYS_POWER_STATE_SLEEP_3);
			break;

		default:
			/* Do nothing. */
			break;
		}

		printk("\n<-- App doing busy wait for 10 Sec -->\n");
		k_busy_wait(BUSY_WAIT_DELAY_US);

		sleep_seconds = (i % 2 != 0) ? LPS1_STATE_ENTER_TO :
					       LPS2_STATE_ENTER_TO;

		printk("\n<-- App going to sleep for %u Sec -->\n",
							sleep_seconds);
		k_sleep(K_SECONDS(sleep_seconds));
	}

	/* Restore automatic power management. */
	printk("\n<-- Forcing %s state --->\n",
		       STRINGIFY(SYS_POWER_STATE_AUTO));
	sys_pm_force_power_state(SYS_POWER_STATE_AUTO);

	printk("\nPress BUTTON to enter into Deep Sleep state. "
			"Press BUTTON to exit Deep Sleep state\n");
	while (1) {
		gpio_pin_read(gpio_port, SW, &level);
		if (level == LOW) {
			k_sleep(K_SECONDS(DEEP_SLEEP_STATE_ENTER_TO));
		}
		k_busy_wait(1000);
	}
}

/*
 * Application defined function for doing any target specific operations
 * for low power entry.
 */
void sys_pm_notify_power_state_entry(enum power_states state)
{

	gpio_pin_write(gpio_port0, LED_0, LED_OFF);

	switch (state) {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	case SYS_POWER_STATE_SLEEP_1:
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	case SYS_POWER_STATE_SLEEP_2:
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	case SYS_POWER_STATE_SLEEP_3:
		break;
 #endif
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */
#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_1
	case SYS_POWER_STATE_DEEP_SLEEP_1:
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
#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
	default:
		printk("Unsupported power state entry %u", state);
		break;
	}
	k_busy_wait(200);
}

/*
 * Application defined function for doing any target specific operations
 * for low power exit.
 */
void sys_pm_notify_power_state_exit(enum power_states state)
{

	k_busy_wait(200);
	gpio_pin_write(gpio_port0, LED_0, LED_ON);

	switch (state) {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	case SYS_POWER_STATE_SLEEP_1:
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	case SYS_POWER_STATE_SLEEP_2:
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	case SYS_POWER_STATE_SLEEP_3:
		break;
 #endif
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */
#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_1
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_2
	case SYS_POWER_STATE_DEEP_SLEEP_2:
		break;
 #endif
#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
	default:
		printk("Unsupported power state exit %u", state);
		break;
	}
}
