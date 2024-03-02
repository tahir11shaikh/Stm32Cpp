/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_flash.hpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CLASS_INC_HAL_FLASH_H_
#define CLASS_INC_HAL_FLASH_H_

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
#define FLASH_BANK1_START	0x08000000U  // Start address of Bank 1
#define FLASH_BANK1_END     0x0803FFFFU  // End address of Bank 1
#define FLASH_BANK2_START   0x08040000U  // Start address of Bank 2
#define FLASH_BANK2_END     0x0807FFFFU  // End address of Bank 2
/* USER CODE END EM */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Global variable -----------------------------------------------------------*/
/* USER CODE BEGIN GV */
class FLASH_CLASS
{
	public:
		// Internal Declaration
		FLASH_CLASS();
		virtual ~FLASH_CLASS();

		// Variable Declaration
		struct
		{
			HAL_ApiState enMemErase;
			struct
			{
				HAL_ApiState enMemRead;
			}stMemRead;
			struct
			{
				HAL_ApiState enMemWrite;
			}stMemWrite;
		} stState;

		// Methods Declaration
		HAL_ApiState FLASH_MemEraseByPage(const uint32_t u32MemAdd);
		HAL_ApiState FLASH_MemWriteData(const uint32_t u32MemAdd, uint8_t *pData, uint16_t u16DataLen);
		HAL_ApiState FLASH_MemReadData(const uint32_t u32MemAdd, uint8_t *pData, uint16_t u16DataLen);

};
/* USER CODE END GV */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* CLASS_INC_HAL_FLASH_H_ */
