/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_swv.cpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_swv.hpp>
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
SWV_CLASS::SWV_CLASS()
{
	// TODO Auto-generated constructor stub
}

SWV_CLASS::~SWV_CLASS()
{
	// TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* functions --------------------------------------------------------*/

/**
  * @brief  This function is used to print on SWV console
  * @param  const char *format, ...
  * @retval uint32_t
  */
uint32_t SWV_CLASS::SWV_Print(const char *format, ...)
{
    char buffer[64];
    va_list args;
    va_start(args, format);

    // std::vsnprintf returns the number of characters that would have been written
    int ret = std:: vsnprintf(buffer, sizeof(buffer), format, args);
    if (ret > 0) {
        for (int i = 0; i < ret && buffer[i] != '\0'; i++) {
            ITM_SendChar(buffer[i]);
        }
    }
    va_end(args);

    return ret;
}
