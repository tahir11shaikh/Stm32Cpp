/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_uart.cpp
  * Created on      : 04-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_uart.hpp>
#include <cstdio>
#include <cstdarg>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
UART_CLASS::UART_CLASS(UART_HandleTypeDef *huart)
    :huart(huart)
{
    // Initialize class members by Constructor
    this->stStatus.enTx = this->stStatus.enTxIT = HAL_ApiState::HAL_FAIL;
    this->stStatus.enRx = this->stStatus.enRxIT = HAL_ApiState::HAL_FAIL;
}

UART_CLASS::~UART_CLASS()
{
    // TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* functions --------------------------------------------------------*/

/**
  * @brief  This function is used to transmit data to UART(Blocking)
  * @param  pData, Size
  * @retval HAL_ApiState
  */
HAL_ApiState UART_CLASS::UART_Transmit(const uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef enState = HAL_UART_Transmit(this->huart, pData, Size,100);
	if (enState != HAL_OK) {
		return this->stStatus.enTx = HAL_FAIL;
	} else {
		return this->stStatus.enTx = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to transmit data to UART(Non_Blocking)
  * @param  pData, Size
  * @retval HAL_ApiState
  */
HAL_ApiState UART_CLASS::UART_Transmit_IT(const uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef enState = HAL_UART_Transmit_IT(this->huart, pData, Size);
	if (enState != HAL_OK) {
		return this->stStatus.enTxIT = HAL_FAIL;
	} else {
		return this->stStatus.enTxIT = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to transmit data to UART(Non_Blocking)
  * @param  pData, Size
  * @retval HAL_ApiState
  */
HAL_ApiState UART_CLASS::UART_Transmit_DMA(const uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef enState = HAL_UART_Transmit_DMA(this->huart, pData, Size);
	if (enState != HAL_OK) {
		return this->stStatus.enTxDMA = HAL_FAIL;
	} else {
		return this->stStatus.enTxDMA = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to read data from UART(Blocking)
  * @param  pData, Size
  * @retval HAL_ApiState
  */
HAL_ApiState UART_CLASS::UART_Receive(uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef enState = HAL_UART_Receive(this->huart, pData, Size, 1000);
	if (enState != HAL_OK) {
		return this->stStatus.enRx = HAL_FAIL;
	} else {
		return this->stStatus.enRx = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to read data from UART(Non_Blocking)
  * @param  pData, Size
  * @retval HAL_ApiState
  */
HAL_ApiState UART_CLASS::UART_Receive_IT(uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef enState = HAL_UART_Receive_IT(this->huart, pData, Size);
	if (enState != HAL_OK) {
		return this->stStatus.enRxIT = HAL_FAIL;
	} else {
		return this->stStatus.enRxIT = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to read data from UART(Non_Blocking)
  * @param  pData, Size
  * @retval HAL_ApiState
  */
HAL_ApiState UART_CLASS::UART_Receive_DMA(uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef enState = HAL_UART_Receive_DMA(this->huart, pData, Size);
	if (enState != HAL_OK) {
		return this->stStatus.enRxDMA = HAL_FAIL;
	} else {
		return this->stStatus.enRxDMA = HAL_SUCCESS;
	}
}


/**
  * @brief  This function is used to transmit Data
  * @param  const char *format, ...
  * @retval uint32_t
  */
uint32_t UART_CLASS::UART_TransmitMsg(const char *format, ...)
{
    char buffer[64];
    va_list args;
    va_start(args, format);

    // std::vsnprintf returns the number of characters that would have been written
    int ret = std::vsnprintf(buffer, sizeof(buffer), format, args);
    if (ret > 0) {
        // Correctly pass the buffer and its length to UART_Transmit_DMA
        // Assuming UART_Transmit_DMA is corrected to take const uint8_t* and size
        this->UART_Transmit(reinterpret_cast<const uint8_t*>(buffer), static_cast<uint16_t>(ret));
    }
    va_end(args);

    return static_cast<uint32_t>(ret);
}
