/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_gpio.cpp
  * Created on      : 24-09-2023
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_pwm.hpp>
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
// Instance Declaration
PWM_CLASS clPWM;

PWM_CLASS::PWM_CLASS() {
	// TODO Auto-generated constructor stub
}

PWM_CLASS::~PWM_CLASS() {
	// TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
/* USER CODE END EV */

/* functions --------------------------------------------------------*/

/**
  * @brief  This function is used to start PWM
  * @param  PWM_PinTypeDef
  * @retval HAL_ApiState
  */
HAL_ApiState PWM_CLASS::PWM_Start(PWM_PinTypeDef enPinTypeDef)
{
	HAL_StatusTypeDef enStatus = HAL_ERROR;

	switch(enPinTypeDef)
	{
		case PWM_PIN_1:
			enStatus = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			break;
		default:
		case PWM_PIN_MaxCnt:
			break;
	}
	if (enStatus != HAL_OK)
	{
		stStatus.enPwmPinStart[enPinTypeDef] = HAL_FAIL;
	} else {
		stStatus.enPwmPinStart[enPinTypeDef] = HAL_SUCCESS;
	}

	return stStatus.enPwmPinStart[enPinTypeDef];
}

/**
  * @brief  This function is used to stop PWM
  * @param  PWM_PinTypeDef
  * @retval HAL_ApiState
  */
HAL_ApiState PWM_CLASS::PWM_Stop(PWM_PinTypeDef enPinTypeDef)
{
	HAL_StatusTypeDef enStatus = HAL_ERROR;

	switch(enPinTypeDef)
	{
		case PWM_PIN_1:
			enStatus = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			break;
		default:
		case PWM_PIN_MaxCnt:
			break;
	}
	if (enStatus != HAL_OK)
	{
		stStatus.enPwmPinStop[enPinTypeDef] = HAL_FAIL;
	} else {
		stStatus.enPwmPinStop[enPinTypeDef] = HAL_SUCCESS;
	}

	return stStatus.enPwmPinStop[enPinTypeDef];
}

/**
  * @brief  This function is used to set duty cycle.
  * @param  PWM_PinTypeDef
  * @param  u8DutyCycle:0-100%
  * @retval HAL_ApiState
  */
HAL_ApiState PWM_CLASS::PWM_SetDutyCycle(PWM_PinTypeDef enPinTypeDef, uint8_t u8DutyCycle)
{
	uint8_t _u8DutyCycle;
	_u8DutyCycle = (uint8_t)(u8DutyCycle < MIN_DUTY_CYCLE) ? MIN_DUTY_CYCLE : ((u8DutyCycle > MAX_DUTY_CYCLE) ? MAX_DUTY_CYCLE : u8DutyCycle);

	switch(enPinTypeDef)
	{
		case PWM_PIN_1:
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, _u8DutyCycle);
		  break;
		default:
		case PWM_PIN_MaxCnt:
			break;
	}

	return HAL_SUCCESS;
}
