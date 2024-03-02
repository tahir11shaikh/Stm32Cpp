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
#include <hal_adc.h>
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
extern ADC_HandleTypeDef hadc1;

// Instance Declaration
ADC_CLASS clADC;

ADC_CLASS::ADC_CLASS() {
    // Initialize all PinConfig in stDIn with HAL_PIN_LOW
    for (int i = 0; i < ADC_PIN_MaxCnt; i++) {
        stIO.stAin.enPinState[i] = HAL_PIN_LOW;
    }
}

ADC_CLASS::~ADC_CLASS() {
	// TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* functions --------------------------------------------------------*/

/**
  * @brief  This function is used to set PIN level high/low.
  * @param  void
  * @retval HAL_FunState
  */
HAL_FunState ADC_CLASS::ADC_StartDMA(void)
{
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) == HAL_OK)
	{
		// Evaluate number of ADC channels
		this->stDMA.stADC.u8AdcDmaChannelCnt = sizeof(stDMA.stADC.u32AdcDmaResult)/sizeof(stDMA.stADC.u32AdcDmaResult[0]);

		// Start ADC DMA
		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) stDMA.stADC.u32AdcDmaResult, stDMA.stADC.u8AdcDmaChannelCnt)!= HAL_OK)
		{
			stDMA.stADC.enAdcDmaStatus = HAL_DISABLE;
		} else {
			stDMA.stADC.enAdcDmaStatus = HAL_ENABLE;
		}
	} else {
		stDMA.stADC.enAdcDmaStatus = HAL_DISABLE;
	}
	return stDMA.stADC.enAdcDmaStatus;
}

/**
  * @brief  This function is used read value from Single channel
  * @param  ADC_PinTypeDef
  * @retval uint32_t
  */
uint32_t ADC_CLASS::ADC_ReadSingleChannel(ADC_PinTypeDef enPinTypeDef)
{
	uint32_t u32AdcValue = 0;
	if (enPinTypeDef < ADC_PIN_MaxCnt)
	{
		return stDMA.stADC.u32AdcDmaResult[enPinTypeDef];
	}
	return u32AdcValue;
}
