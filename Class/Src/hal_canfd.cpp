/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_canfd.cpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_canfd.hpp>
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
CAN_CLASS::CAN_CLASS()
{
    // Initialize class members by Constructor
    this->stStatus.enCanFilterConfig = HAL_ApiState::HAL_FAIL;
    this->stStatus.enCanStart = HAL_ApiState::HAL_FAIL;
    this->stStatus.enCanActivateTxNotification = HAL_ApiState::HAL_FAIL;
    this->stStatus.enCanActivateRxNotification = HAL_ApiState::HAL_FAIL;
    this->stStatus.enCanAddTxMessage = HAL_ApiState::HAL_FAIL;
}

CAN_CLASS::~CAN_CLASS()
{
	// TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE END EV */

/* functions --------------------------------------------------------*/

/**
  * @brief  This function is used Configure CAN filter.
  * @param  void
  * @retval HAL_ApiState
  */
HAL_ApiState CAN_CLASS::CAN_FilterConfig(void)
{
	FDCAN_FilterTypeDef stFilterConfig;

	stFilterConfig.IdType = FDCAN_STANDARD_ID;
	stFilterConfig.FilterIndex = 0;
	stFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	stFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	stFilterConfig.FilterID1 = 0x123;
	stFilterConfig.FilterID2 = 0x123;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &stFilterConfig) != HAL_OK)
	{
		return this->stStatus.enCanFilterConfig = HAL_FAIL;
	} else {
		return this->stStatus.enCanFilterConfig = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to start CAN peripheral
  * @param  void
  * @retval HAL_ApiState
  */
HAL_ApiState CAN_CLASS::CAN_Start(void)
{
	// Start CAN
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		return this->stStatus.enCanStart = HAL_FAIL;
	} else {
		return this->stStatus.enCanStart = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to Activate CAN TX notification
  * @param  void
  * @retval HAL_ApiState
  */
HAL_ApiState CAN_CLASS::CAN_ActivateTxNotification(void)
{
	// Activate CAN TX notification
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE|FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER0) != HAL_OK)
	{
		return this->stStatus.enCanActivateTxNotification = HAL_FAIL;
	} else {
		return this->stStatus.enCanActivateTxNotification = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to Activate CAN RX notification
  * @param  void
  * @retval HAL_ApiState
  */
HAL_ApiState CAN_CLASS::CAN_ActivateRxNotification(void)
{
	// Activate CAN RX notification
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		return this->stStatus.enCanActivateRxNotification = HAL_FAIL;
	} else {
		return this->stStatus.enCanActivateRxNotification = HAL_SUCCESS;
	}
}

/**
  * @brief  This function is used to send CAN message frame
  * @param  void
  * @retval HAL_ApiState
  */
HAL_ApiState CAN_CLASS::CAN_AddTxMessage(CanMsg_st_t *stCanTxMsg)
{
	stCanTxMsg->TxHeader.Identifier = (uint32_t)stCanTxMsg->u32CanId;
	stCanTxMsg->TxHeader.IdType = FDCAN_STANDARD_ID;
	stCanTxMsg->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	stCanTxMsg->TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	stCanTxMsg->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	stCanTxMsg->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	stCanTxMsg->TxHeader.FDFormat = FDCAN_FD_CAN;
	stCanTxMsg->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	stCanTxMsg->TxHeader.MessageMarker = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &stCanTxMsg->TxHeader, 	stCanTxMsg->u8CanData) != HAL_OK)
	{
		return this->stStatus.enCanAddTxMessage = HAL_FAIL;
	} else {
		return this->stStatus.enCanAddTxMessage = HAL_SUCCESS;
	}
}
