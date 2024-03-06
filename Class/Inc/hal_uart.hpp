/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_uart.hpp
  * Created on      : 04-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_UART_H_
#define CLASS_INC_HAL_UART_H_

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
typedef class UART_CLASS
{
    public:
        // Internal Declaration
        explicit UART_CLASS(UART_HandleTypeDef *huart);
        virtual ~UART_CLASS();

        // Variable Declaration
        struct
        {
            HAL_ApiState enTx,enTxIT,enTxDMA;
            HAL_ApiState enRx,enRxIT,enRxDMA;
        } stStatus;

        // Methods Declaration
        HAL_ApiState UART_Transmit(const uint8_t *pData, uint16_t Size);
        HAL_ApiState UART_Transmit_IT(const uint8_t *pData, uint16_t Size);
        HAL_ApiState UART_Transmit_DMA(const uint8_t *pData, uint16_t Size);

        HAL_ApiState UART_Receive(uint8_t *pData, uint16_t Size);
        HAL_ApiState UART_Receive_IT(uint8_t *pData, uint16_t Size);
        HAL_ApiState UART_Receive_DMA(uint8_t *pData, uint16_t Size);

        uint32_t UART_TransmitMsg(const char *format, ...);
    private:
        UART_HandleTypeDef *huart;
} UART_CLASS;
/* USER CODE END ET */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_UART_H_ */
