/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_gpio.hpp
  * Created on      : 03-03-2024
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

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

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

typedef class GPIO_CLASS
{
    public:
        // Internal Declaration
        explicit GPIO_CLASS();
        virtual ~GPIO_CLASS();

        // Variable Declaration
        struct
        {
            struct
            {
                HAL_PinState enPinState[GPI_PIN_MaxCnt];
            } stDIn;

            struct
            {
                HAL_PinState enPinState[GPO_PIN_MaxCnt];
            } stDout;
        } stStatus;

        // Methods Declaration
        HAL_PinState GPO_PinSetLevel(GPO_PinTypeDef enPinTypeDef, HAL_PinState enPinState);
        HAL_PinState GPO_PinToggle(GPO_PinTypeDef enPinTypeDef);
        HAL_PinState GPI_PinGetLevel(GPI_PinTypeDef enPinTypeDef);
} GPIO_CLASS;
/* USER CODE END ET */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_GPIO_H_ */
