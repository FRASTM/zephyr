/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PWM_STM32_H_
#define PWM_STM32_H_

#include <soc.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_lptim.h>


#if DT_NODE_HAS_COMPAT_STATUS(DT_INST_PARENT(0), st_stm32_lptimers, okay)

#define LL_PWM_InitTypeDef LL_LPTIM_InitTypeDef
#define LL_PWM_Init LL_LPTIM_Init
#define LL_PWM_StructInit LL_LPTIM_StructInit

#else

#define LL_PWM_InitTypeDef LL_TIM_InitTypeDef
#define LL_PWM_Init LL_TIM_Init
#define LL_PWM_StructInit LL_TIM_StructInit

#endif

#endif /* PWM_STM32_H_*/
