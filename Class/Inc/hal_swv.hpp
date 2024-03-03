/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_swv.hpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_SWV_H_
#define CLASS_INC_HAL_SWV_H_

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
class SWV_CLASS
{
	public:
		// Internal Declaration
		SWV_CLASS();
		virtual ~SWV_CLASS();

		// Methods Declaration
		uint32_t SWV_Print(const char *format, ...);
};
/* USER CODE END GV */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_SWV_H_ */
