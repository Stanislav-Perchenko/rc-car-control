/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_ll_iwdg.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Board_LED_Pin LL_GPIO_PIN_13
#define Board_LED_GPIO_Port GPIOC
#define LOG_OUT_Pin LL_GPIO_PIN_2
#define LOG_OUT_GPIO_Port GPIOA
#define U2RX_Pin LL_GPIO_PIN_3
#define U2RX_GPIO_Port GPIOA
#define F_R_Pin LL_GPIO_PIN_5
#define F_R_GPIO_Port GPIOA
#define PWM_SPEED_Pin LL_GPIO_PIN_6
#define PWM_SPEED_GPIO_Port GPIOA
#define PWM_RUDE_Pin LL_GPIO_PIN_7
#define PWM_RUDE_GPIO_Port GPIOA
#define PWM_CAM_YAW_Pin LL_GPIO_PIN_0
#define PWM_CAM_YAW_GPIO_Port GPIOB
#define PWM_CAM_PITCH_Pin LL_GPIO_PIN_1
#define PWM_CAM_PITCH_GPIO_Port GPIOB
#define OUT1_Pin LL_GPIO_PIN_13
#define OUT1_GPIO_Port GPIOB
#define U1TX_Pin LL_GPIO_PIN_9
#define U1TX_GPIO_Port GPIOA
#define sBUS_Pin LL_GPIO_PIN_10
#define sBUS_GPIO_Port GPIOA
#define CALIBRATE_Pin LL_GPIO_PIN_9
#define CALIBRATE_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

#define USART_LOG	USART2

#define SBUS_CH_NUM_THROTTLE	3
#define SBUS_CH_NUM_RUDDER		4
#define SBUS_CH_NUM_CAM_YAW		1
#define SBUS_CH_NUM_CAM_PITCH	2
#define SBUS_CH_NUM_OUT1		6

#define ABS_MIN_CHANNEL_VALUE_FOR_CALIBRATION		150
#define ABS_MAX_CHANNEL_VALUE_FOR_CALIBRATION		2150
#define CLIP_CENTER_PROTECTION_INTERVAL_PERCENT		6

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
