/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_i2c.cpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_i2c.hpp>
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
I2C_CLASS::I2C_CLASS(I2C_HandleTypeDef *hi2c)
    :hi2c(hi2c)
{
    // Initialize class members by Constructor
    this->stStatus.enMemWrite = HAL_ApiState::HAL_FAIL;
    this->stStatus.enMemRead = HAL_ApiState::HAL_FAIL;
}

I2C_CLASS::~I2C_CLASS()
{
    // TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* functions --------------------------------------------------------*/

/**
  * @brief  This function is used to write data on memory register
  * @param  uint16_t, uint16_t, uint8_t*
  * @retval HAL_ApiState
  */
HAL_ApiState I2C_CLASS::I2C_MemWrite(uint16_t u16DevAddr, uint16_t u16MemAddr, uint16_t u16Data)
{
    uint8_t p8Data[2];

    p8Data[0] = (u16Data >> 8) & 0xFF; // MSB
    p8Data[1] = u16Data & 0xFF;        // LSB

    // I2C Mem Write
    if (HAL_I2C_Mem_Write(this->hi2c, u16DevAddr<<1, u16MemAddr, I2C_MEMADD_SIZE_8BIT, p8Data, 2, 100) != HAL_OK)
    {
        return this->stStatus.enMemWrite = HAL_FAIL;
    } else {
        return this->stStatus.enMemWrite = HAL_SUCCESS;
    }
}

/**
  * @brief  This function is used to read data from memory register
  * @param  uint16_t, uint16_t, uint8_t*
  * @retval HAL_ApiState
  */
HAL_ApiState I2C_CLASS::I2C_MemRead(uint16_t u16DevAddr, uint16_t u16MemAddr, uint16_t *p16Data)
{
    uint8_t p8Data[2];

    // I2C Mem Read
    if (HAL_I2C_Mem_Read(this->hi2c, u16DevAddr<<1, u16MemAddr, I2C_MEMADD_SIZE_8BIT, p8Data, 2, 100) != HAL_OK)
    {
        return this->stStatus.enMemRead = HAL_FAIL;
    } else {
        *p16Data = ((p8Data[0] << 8) | p8Data[1]);
        return this->stStatus.enMemRead = HAL_SUCCESS;
    }
}

