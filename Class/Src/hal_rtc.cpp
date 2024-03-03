/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_rtc.cpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_rtc.hpp>
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
RTC_CLASS::RTC_CLASS()
{
    // Initialize class members by Constructor
    this->stStatus.enSetTime = HAL_ApiState::HAL_FAIL;
    this->stStatus.enGetTime = HAL_ApiState::HAL_FAIL;
    this->stStatus.enSetDate = HAL_ApiState::HAL_FAIL;
    this->stStatus.enGetDate = HAL_ApiState::HAL_FAIL;
}

RTC_CLASS::~RTC_CLASS()
{
	// TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern RTC_HandleTypeDef hrtc;
/* USER CODE END EV */

/* functions --------------------------------------------------------*/

/**
  * @brief  This function is used to set date and time to RTC.
  * @param  GPIO_PinTypeDef
  * @retval HAL_PinState
  */
HAL_ApiState RTC_CLASS::RTC_vSetTimeDate()
{
	// Set the RTC new Time
	this->stVar.stSetTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	this->stVar.stSetTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &this->stVar.stSetTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		this->stStatus.enSetTime = HAL_FAIL;
	} else {
		this->stStatus.enSetTime = HAL_SUCCESS;
	}

	// Set the RTC new Time
	this->stVar.stSetDate.WeekDay = RTC_WEEKDAY_MONDAY;
	if (HAL_RTC_SetDate(&hrtc, &this->stVar.stSetDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		this->stStatus.enSetDate = HAL_FAIL;
	} else {
		this->stStatus.enSetDate = HAL_SUCCESS;
	}

	if (this->stStatus.enSetTime != HAL_FAIL && this->stStatus.enSetDate != HAL_FAIL)
	{
		return HAL_SUCCESS;
	} else {
		return HAL_FAIL;
	}
}

/**
  * @brief  This function is used to get date and time from RTC.
  * @param  GPIO_PinTypeDef
  * @retval HAL_PinState
  */
HAL_ApiState RTC_CLASS::RTC_vGetTimeDate()
{
	// Get the RTC current Time
	if (HAL_RTC_GetTime(&hrtc, &this->stVar.stGetTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		this->stStatus.enGetTime = HAL_FAIL;
	} else {
		this->stStatus.enGetTime = HAL_SUCCESS;
	}

	// Get the RTC current Date
	if (HAL_RTC_GetDate(&hrtc, &this->stVar.stGetDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		this->stStatus.enGetDate = HAL_FAIL;
	} else {
		this->stStatus.enGetDate = HAL_SUCCESS;
	}

	if (this->stStatus.enSetTime != HAL_FAIL && this->stStatus.enGetDate != HAL_FAIL)
	{
		return HAL_SUCCESS;
	} else {
		return HAL_FAIL;
	}
}
