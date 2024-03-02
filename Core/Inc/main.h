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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdarg.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**
  * @brief  HAL_BoolState structures definition
  */
typedef enum
{
  HAL_NOT_SET = 0U,
  HAL_SET = !HAL_NOT_SET
} HAL_BoolState;

/**
  * @brief  HAL_PinConfig structures definition
  */
typedef enum
{
  HAL_PIN_NOT_CONFIG = 0U,
  HAL_PIN_CONFIG = !HAL_PIN_NOT_CONFIG
} HAL_PinConfig;

/**
  * @brief  HAL_PinState structures definition
  */
typedef enum
{
  //HAL_PIN_X=-1,
  HAL_PIN_LOW = 0U,
  HAL_PIN_HIGH = !HAL_PIN_LOW,
} HAL_PinState;

/**
  * @brief  HAL_ApiState structures definition
  */
typedef enum
{
  HAL_FAIL = 0U,
  HAL_SUCCESS = !HAL_FAIL
} HAL_ApiState;

/**
  * @brief  HAL_FunState structures definition
  */
typedef enum
{
  HAL_DISABLE = 0U,
  HAL_ENABLE = !HAL_DISABLE
} HAL_FunState;

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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
