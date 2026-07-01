/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Hearthbeat_Pin GPIO_PIN_13
#define LED_Hearthbeat_GPIO_Port GPIOC
#define Led_Debug_P14_Pin GPIO_PIN_14
#define Led_Debug_P14_GPIO_Port GPIOC
#define Led_Debug_P15_Pin GPIO_PIN_15
#define Led_Debug_P15_GPIO_Port GPIOC
#define Led_LV_Pin GPIO_PIN_10
#define Led_LV_GPIO_Port GPIOB
#define WaterPump_Pin GPIO_PIN_15
#define WaterPump_GPIO_Port GPIOA
#define AMS_Pin GPIO_PIN_4
#define AMS_GPIO_Port GPIOB
#define Radiator_Pin GPIO_PIN_6
#define Radiator_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
