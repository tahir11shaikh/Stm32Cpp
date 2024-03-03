/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_gpio.cpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_adc.hpp>
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
ADC_CLASS::ADC_CLASS(ADC_HandleTypeDef *hadc)
    :hadc(hadc)
{
    // Initialize class members by Constructor
    for (int i = 0; i < ADC_PIN_MaxCnt; ++i) {
         this->stVar.u32ChannelValue[i] = 0;
         this->stDMA.u32ChannelValue[i] = 0;
    }
    this->stDMA.u8ChannelCnt = 0;
    this->stDMA.enAdcDmaStatus = HAL_FunState::HAL_DISABLE;

}

ADC_CLASS::~ADC_CLASS()
{
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
    if (HAL_ADCEx_Calibration_Start(this->hadc, ADC_SINGLE_ENDED) == HAL_OK)
    {
        // Evaluate number of ADC channels
        this->stDMA.u8ChannelCnt = sizeof(this->stDMA.u32ChannelValue)/sizeof(this->stDMA.u32ChannelValue[0]);

        // Start ADC DMA
        if (HAL_ADC_Start_DMA(this->hadc, (uint32_t*) this->stDMA.u32ChannelValue, this->stDMA.u8ChannelCnt)!= HAL_OK)
        {
            this->stDMA.enAdcDmaStatus = HAL_DISABLE;
        } else {
            this->stDMA.enAdcDmaStatus = HAL_ENABLE;
        }
    } else {
        this->stDMA.enAdcDmaStatus = HAL_DISABLE;
    }
    return this->stDMA.enAdcDmaStatus;
}

/**
  * @brief  This function is used read value from Single channel By DMA
  * @param  ADC_PinTypeDef
  * @retval uint32_t
  */
uint32_t ADC_CLASS::ADC_ReadSingleChannelDMA(ADC_PinTypeDef enPinTypeDef)
{
    uint32_t u32AdcValue = 0;
    if (enPinTypeDef < ADC_PIN_MaxCnt)
    {
        return this->stDMA.u32ChannelValue[enPinTypeDef];
    }
    return u32AdcValue;
}

/**
  * @brief  This function is used read value from Single channel by Polling
  * @param  ADC_PinTypeDef
  * @retval uint32_t
  */
uint32_t ADC_CLASS::ADC_ReadSingleChannelPoll(ADC_PinTypeDef enPinTypeDef)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    switch(enPinTypeDef)
    {
        case ADC_Sensor_1:
            sConfig.Channel = ADC_CHANNEL_1;
            sConfig.Rank = ADC_REGULAR_RANK_1;
            sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
            break;

        case ADC_PIN_MaxCnt:
        default:
            break;
    }

    /* Configure Regular Channel */
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(this->hadc, &sConfig) == HAL_OK)
    {
        HAL_ADC_Start(this->hadc); // Start ADC
        HAL_ADC_PollForConversion(this->hadc, 100); // Poll For Conversation
        this->stVar.u32ChannelValue[enPinTypeDef] = HAL_ADC_GetValue(this->hadc); // Get ADC Value
        HAL_ADC_Stop(this->hadc); // Stop ADC
    }
    return this->stVar.u32ChannelValue[enPinTypeDef];
}
