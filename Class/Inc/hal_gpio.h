/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_gpio.h
  * Created on      : 02-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_GPIO_H_
#define CLASS_INC_HAL_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32g4xx_hal.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	GPI_PIN_PC13=0,

	GPI_PIN_MaxCnt,
} GPI_PinTypeDef;

typedef enum
{
	GPO_PIN_PA5=0,

	GPO_PIN_MaxCnt,
} GPO_PinTypeDef;

class GPIO_CLASS
{
	public:
		// Internal Declaration
		GPIO_CLASS();
		virtual ~GPIO_CLASS();

		// Variable Declaration
		struct
		{
		    struct
		    {
		    	HAL_PinState PinState[GPI_PIN_MaxCnt];
		    } stDIn;

			struct
			{
		    	HAL_PinState PinState[GPO_PIN_MaxCnt];
			} stDout;
		} stIO;

		// Methods Declaration
		HAL_PinState GPO_vPinSetLevel(GPO_PinTypeDef enPinTypeDef, HAL_PinState enPinState);
		HAL_PinState GPI_bPinGetLevel(GPI_PinTypeDef enPinTypeDef);
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_GPIO_H_ */
