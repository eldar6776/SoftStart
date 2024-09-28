/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "LuxNET.h"
#include "TinyFrame.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc;
extern CRC_HandleTypeDef hcrc;
extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart1;
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
#define ADDRESS_0_Pin GPIO_PIN_0
#define ADDRESS_0_GPIO_Port GPIOF
#define ADDRESS_1_Pin GPIO_PIN_1
#define ADDRESS_1_GPIO_Port GPIOF
#define TRIAC_TEMP_Pin GPIO_PIN_0
#define TRIAC_TEMP_GPIO_Port GPIOA
#define TRAFO_TEMP_Pin GPIO_PIN_4
#define TRAFO_TEMP_GPIO_Port GPIOA
#define TRAFO_VOLT_Pin GPIO_PIN_5
#define TRAFO_VOLT_GPIO_Port GPIOA
#define RELAY_CTRL_Pin GPIO_PIN_6
#define RELAY_CTRL_GPIO_Port GPIOA
#define TRIAC_CTRL_Pin GPIO_PIN_7
#define TRIAC_CTRL_GPIO_Port GPIOA
#define ZEROCROSS_Pin GPIO_PIN_1
#define ZEROCROSS_GPIO_Port GPIOB
#define ZEROCROSS_EXTI_IRQn EXTI0_1_IRQn
#define CTRL_IN_Pin GPIO_PIN_9
#define CTRL_IN_GPIO_Port GPIOA
#define STATUS_OUT_Pin GPIO_PIN_10
#define STATUS_OUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
