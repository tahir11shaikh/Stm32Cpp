/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_rtc.hpp
  * Created on      : 02-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_RTC_H_
#define CLASS_INC_HAL_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32g4xx_hal.h"
/* USER CODE END Includes */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Global variable -----------------------------------------------------------*/
/* USER CODE BEGIN GV */
class RTC_CLASS
{
	public:
		// Internal Declaration
		RTC_CLASS();
		virtual ~RTC_CLASS();

		// Variable Declaration
		RTC_TimeTypeDef stSetTime;
		RTC_TimeTypeDef stGetTime;
		RTC_DateTypeDef stSetDate;
		RTC_DateTypeDef stGetDate;

		struct
		{
			HAL_ApiState enSetTime;
			HAL_ApiState enGetTime;
			HAL_ApiState enSetDate;
			HAL_ApiState enGetDate;
		} stState;

		// Methods Declaration
		HAL_ApiState RTC_vSetTimeDate();
		HAL_ApiState RTC_vGetTimeDate();
};
/* USER CODE END GV */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_RTC_H_ */
