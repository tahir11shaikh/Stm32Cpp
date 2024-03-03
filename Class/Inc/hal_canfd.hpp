/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_canfd.hpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_CANFD_H_
#define CLASS_INC_HAL_CANFD_H_

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
typedef struct CanMsg_st
{
  uint32_t u32CanId;
  uint32_t u32CanDlc;
  uint8_t  u8CanData[8];
  uint64_t u64TxSentCounter;
  uint64_t u64RxSentCounter;
  FDCAN_TxHeaderTypeDef TxHeader;
  FDCAN_RxHeaderTypeDef RxHeader;
} CanMsg_st_t;

typedef class CAN_CLASS
{
	public:
		// Internal Declaration
		CAN_CLASS();
		virtual ~CAN_CLASS();

		// Variable Declaration
		struct
		{
			HAL_ApiState enCanFilterConfig;
			HAL_ApiState enCanStart;
			HAL_ApiState enCanActivateTxNotification;
			HAL_ApiState enCanActivateRxNotification;
			HAL_ApiState enCanAddTxMessage;
		} stStatus;

		struct
		{
			CanMsg_st_t stCanTxMsg;
			CanMsg_st_t stCanRxMsg;
		}stVar;

		// Methods Declaration
		HAL_ApiState CAN_FilterConfig(void);
		HAL_ApiState CAN_Start(void);
		HAL_ApiState CAN_ActivateTxNotification(void);
		HAL_ApiState CAN_ActivateRxNotification(void);
		HAL_ApiState CAN_AddTxMessage(CanMsg_st_t *stCanTxMsg);
}CAN_CLASS;
/* USER CODE END ET */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_CANFD_H_ */
