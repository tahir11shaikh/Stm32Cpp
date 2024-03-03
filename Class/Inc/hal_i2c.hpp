/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_i2c.hpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_I2C_H_
#define CLASS_INC_HAL_I2C_H_

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
typedef class I2C_CLASS
{
    public:
        // Internal Declaration
        explicit I2C_CLASS(I2C_HandleTypeDef *hi2c);
        virtual ~I2C_CLASS();

        // Variable Declaration
        struct
        {
            HAL_ApiState enMemWrite;
            HAL_ApiState enMemRead;
        } stStatus;

        // Methods Declaration
        HAL_ApiState I2C_MemWrite(uint16_t u16DevAddr, uint16_t u16MemAddr, uint16_t u16Data);
        HAL_ApiState I2C_MemRead(uint16_t u16DevAddr, uint16_t u16MemAddr, uint16_t *p16Data);

    private:
        I2C_HandleTypeDef *hi2c;
} I2C_CLASS;
/* USER CODE END ET */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_I2C_H_ */
