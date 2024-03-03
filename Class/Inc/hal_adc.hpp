/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_adc.hpp
  * Created on      : 03-03-2024
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

typedef class ADC_CLASS
{
    public:
        // Internal Declaration
        explicit ADC_CLASS(ADC_HandleTypeDef *hadc);
        virtual ~ADC_CLASS();

        // Variable Declaration
        struct
        {
            uint32_t u32ChannelValue[ADC_PIN_MaxCnt];
        } stVar;

        struct
        {
            uint8_t u8ChannelCnt;
            HAL_FunState enAdcDmaStatus;
            uint32_t u32ChannelValue[ADC_PIN_MaxCnt];
        } stDMA;

        // Methods Declaration
        HAL_FunState ADC_StartDMA(void);
        uint32_t ADC_ReadSingleChannelDMA(ADC_PinTypeDef enPinTypeDef);
        uint32_t ADC_ReadSingleChannelPoll(ADC_PinTypeDef enPinTypeDef);
    private:
        ADC_HandleTypeDef *hadc;
} ADC_CLASS;
/* USER CODE END ET */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_ADC_H_ */
