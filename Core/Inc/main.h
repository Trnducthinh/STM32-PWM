/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Dir_Dc_Pin GPIO_PIN_4
#define Dir_Dc_GPIO_Port GPIOA
#define Dir_DcA5_Pin GPIO_PIN_5
#define Dir_DcA5_GPIO_Port GPIOA
#define BT1_Pin GPIO_PIN_7
#define BT1_GPIO_Port GPIOA
#define BT1_EXTI_IRQn EXTI9_5_IRQn
#define BT2_Pin GPIO_PIN_0
#define BT2_GPIO_Port GPIOB
#define BT2_EXTI_IRQn EXTI0_IRQn
#define BT3_Pin GPIO_PIN_1
#define BT3_GPIO_Port GPIOB
#define BT3_EXTI_IRQn EXTI1_IRQn
#define BT4_Pin GPIO_PIN_10
#define BT4_GPIO_Port GPIOB
#define BT4_EXTI_IRQn EXTI15_10_IRQn
#define BT5_Pin GPIO_PIN_11
#define BT5_GPIO_Port GPIOB
#define BT5_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
