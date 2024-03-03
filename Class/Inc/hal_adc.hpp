/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_adc.hpp
  * Created on      : 02-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_ADC_H_
#define CLASS_INC_HAL_ADC_H_

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
typedef enum
{
	ADC_Sensor_1,

	ADC_PIN_MaxCnt,
} ADC_PinTypeDef;
/* USER CODE END ET */

/* Global variable -----------------------------------------------------------*/
/* USER CODE BEGIN GV */
class ADC_CLASS
{
	public:
		// Internal Declaration
		ADC_CLASS();
		virtual ~ADC_CLASS();

		// Variable Declaration
		struct
		{
			struct
			{
				HAL_PinState enPinState[ADC_PIN_MaxCnt];
			} stAin;
		} stStatus;

		struct
		{
			uint32_t u32ChannelValue[ADC_PIN_MaxCnt];
		} stVar;

		struct
		{
			struct
			{
				uint8_t u8ChannelCnt;
				HAL_FunState enAdcDmaStatus;
				uint32_t u32ChannelValue[ADC_PIN_MaxCnt];
			} stADC;
		} stDMA;

		// Methods Declaration
		HAL_FunState ADC_StartDMA(void);
		uint32_t ADC_ReadSingleChannelDMA(ADC_PinTypeDef enPinTypeDef);
		uint32_t ADC_ReadSingleChannelPoll(ADC_PinTypeDef enPinTypeDef);
};
/* USER CODE END GV */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_ADC_H_ */
