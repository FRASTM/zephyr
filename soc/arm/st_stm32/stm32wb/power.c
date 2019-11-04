/*
 * Copyright (c) 2019 STMicroelectronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <power/power.h>
#include <stm32wbxx_ll_hsem.h>
#include <stm32wbxx_ll_cortex.h>
#include <stm32wbxx_ll_pwr.h>
#include <stm32wbxx_ll_rcc.h>

/* Common BLE file where BLE shared resources are defined */
#include "hw_conf.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

static void switch_on_HSI(void)
{
	LL_RCC_HSI_Enable();
	while (!LL_RCC_HSI_IsReady()) {
	}
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
	}
}

/* Invoke Low Power/System Off specific Tasks */
void sys_set_power_state(enum power_states state)
{
#ifdef CONFIG_DEBUG
	/* Enable the Debug Module during STOP mode */
	LL_DBGMCU_EnableDBGStopMode();
#endif
	/* take the RCC semaphore */
	while (LL_HSEM_1StepLock(HSEM, CFG_HW_RCC_SEMID)) {
	}
	/*
	 * Ensure HSI is the system clock source after Wake Up from Stop mode
	 */
	LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);

	if (!LL_HSEM_1StepLock(HSEM, CFG_HW_ENTRY_STOP_MODE_SEMID)) {
		if (LL_PWR_IsActiveFlag_C2DS()) {
			/* Release ENTRY_STOP_MODE semaphore */
			LL_HSEM_ReleaseLock(HSEM, CFG_HW_ENTRY_STOP_MODE_SEMID, 0);

			switch_on_HSI();
		}
	} else {
		switch_on_HSI();
	}

	/* Release RCC semaphore */
	LL_HSEM_ReleaseLock(HSEM, CFG_HW_RCC_SEMID, 0);

	switch (state) {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	case SYS_POWER_STATE_SLEEP_1:
		/* this corresponds to the STOP0 mode: */
		/* enter STOP0 mode for CPU1 */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
		LL_LPM_EnableDeepSleep();
		/* enter SLEEP mode : WFE or WFI */
		k_cpu_idle();
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	case SYS_POWER_STATE_SLEEP_2:
		/* this corresponds to the STOP1 mode: */
		/* enter STOP1 mode for CPU1 */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
		LL_C2_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
		LL_LPM_EnableDeepSleep();
		k_cpu_idle();
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	case SYS_POWER_STATE_SLEEP_3:
		/* this corresponds to the STOP2 mode: */
		/* boot CPU2 after stop mode*/
		LL_PWR_EnableBootC2();
		/* enter STOP2 mode for CPU1 and CPU2 */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);
		LL_C2_PWR_SetPowerMode(LL_PWR_MODE_STOP2);
		LL_LPM_EnableDeepSleep();
		k_cpu_idle();
		break;
 #endif
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */
#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_1
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		/* this corresponds to the STANDBY mode: */
		/* clear WUFx bits in the power status register 1 */
		LL_PWR_ClearFlag_WU();
		/* set all GPIO to analog mode and disable GPIO clocks */
		/* TODO */
		/* program the wakeUp source RTC or wdg or wakeUp pin */
		/* TODO */
		/* enter STANDBY mode */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
		LL_LPM_EnableDeepSleep();
		k_cpu_idle();
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_2
	case SYS_POWER_STATE_DEEP_SLEEP_2:
		/* this corresponds to the SHUTDOWN mode */
		/* clear WUFx bits in the power status register 1 */
		LL_PWR_ClearFlag_WU();
		/* set all GPIO to analog mode and disable GPIO clocks */
		/* TODO */
		/* program the wakeUp source RTC or wdg or wakeUp pin */
		/* TODO */
		/* enter SHUTDOWN mode */
		LL_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
		LL_LPM_EnableDeepSleep();
		k_cpu_idle();
		break;
 #endif
#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
void _sys_pm_power_state_exit_post_ops(enum power_states state)
{
	/* after wakeUp from Stop Mode, the system clock is HSI
	 * after wakeUp from Deep Sleep we reboot, so we never come here
	 */
	switch (state) {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	case SYS_POWER_STATE_SLEEP_1:
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	case SYS_POWER_STATE_SLEEP_2:
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	case SYS_POWER_STATE_SLEEP_3:
 #endif
		/* release ENTRY_STOP_MODE semaphore */
		LL_HSEM_ReleaseLock(HSEM, CFG_HW_ENTRY_STOP_MODE_SEMID, 0);

		if ((LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
		|| (LL_PWR_IsActiveFlag_C1STOP() != 0)) {

			LL_PWR_ClearFlag_C1STOP_C1STB();

			while (LL_HSEM_1StepLock(HSEM, CFG_HW_RCC_SEMID)) {
			}
			/* re-configure system clock after wake-up from STOP:
			 * enable HSE as system clock source
			 */
			if (LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
				LL_RCC_HSE_Enable();
				while (!LL_RCC_HSE_IsReady()) {
				}
				LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
				while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE) {
				}
			} else {
				/*
				 * As long as the current application is fine with HSE as system clock source,
				 * there is nothing to do here
				 */
			}

			LL_HSEM_ReleaseLock(HSEM, CFG_HW_RCC_SEMID, 0);
		}
		break;
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */
#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_1
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		/* nothing special to do as reboot occurred */
		LL_PWR_ClearFlag_C1STOP_C1STB();
		break;
 #endif
 #ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_2
	case SYS_POWER_STATE_DEEP_SLEEP_2:
		/* nothing special to do as reboot occurred */
		LL_PWR_ClearFlag_C1STOP_C1STB();
		break;
 #endif
#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	/*
	 * System is now in active mode.
	 * Reenable interrupts which were disabled
	 * when OS started idling code.
	 */
	irq_unlock(0);
}
