/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hal_flash.cpp
  * Created on      : 03-03-2024
  * Author          : Tahir.Shaikh
  * @brief          : Source/Header file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <hal_flash.hpp>
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
FLASH_CLASS::FLASH_CLASS() {
	// TODO Auto-generated constructor stub
}

FLASH_CLASS::~FLASH_CLASS() {
	// TODO Auto-generated destructor stub
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* functions --------------------------------------------------------*/
/**
  * @brief  This function is used to erase memory by page
  * @param  u32MemAdd
  * @retval HAL_ApiState
  */
HAL_ApiState FLASH_CLASS::FLASH_MemEraseByPage(const uint32_t u32MemAdd)
{
    uint32_t u32PageError = 0;
    FLASH_EraseInitTypeDef stEraseInit = {0};

    // Unlock the Flash to enable the flash control register access
    HAL_FLASH_Unlock();

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    // Fill EraseInit structure
    stEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    stEraseInit.Banks = (u32MemAdd < FLASH_BANK2_START) ? FLASH_BANK_1 : FLASH_BANK_2;

    // Calculate the page number within the bank
    uint32_t bankOffset = (stEraseInit.Banks == FLASH_BANK_1) ? FLASH_BANK1_START : FLASH_BANK2_START;
    stEraseInit.Page = (u32MemAdd - bankOffset) / FLASH_PAGE_SIZE;
    stEraseInit.NbPages = 1; // Assuming we want to erase only one page

    // Check if the address is within a valid range
    if ((u32MemAdd < FLASH_BANK1_START) || (u32MemAdd >= FLASH_BANK2_END))
    {
        // Address is out of range
        HAL_FLASH_Lock();
        return HAL_FAIL;
    }

    // Erase the Flash by Page
    if (HAL_FLASHEx_Erase(&stEraseInit, &u32PageError) != HAL_OK)
    {
        // Error occurred while page erase
        HAL_FLASH_GetError();
        HAL_FLASH_Lock();
        return HAL_FAIL;
    }

    // Lock the Flash to disable the flash control register access
    HAL_FLASH_Lock();

    return HAL_SUCCESS;
}

/**
  * @brief  This function is used write data to memory address
  * @param  u32MemAdd,pData,u16DataLen
  * @retval HAL_ApiState
  */
HAL_ApiState FLASH_CLASS::FLASH_MemWriteData(uint32_t u32MemAdd, uint8_t *pData, uint16_t u16DataLen)
{
	uint32_t i = 0;
	uint8_t u8Data[sizeof(uint64_t)] = {0xFF};

    // Erase the specified Flash page
	this->FLASH_MemEraseByPage(u32MemAdd);

	// Unlock the Flash to enable the flash control register access.
	HAL_FLASH_Unlock();

	// Program the user Flash area word by word
	while( i < u16DataLen)
	{
		u8Data[i%sizeof(uint64_t)] = pData[i];
		i++;
		if (i%sizeof(uint64_t) == 0)
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, u32MemAdd+(i-sizeof(uint64_t)), *((uint64_t*)u8Data)) != HAL_OK)
			{
				// Error occurred while page write.
				//uint32_t u32ErrCode = HAL_FLASH_GetError();
				HAL_FLASH_GetError();
				HAL_FLASH_Lock();
				return HAL_FAIL;
			}
			memset(u8Data, 0xFF, sizeof(u8Data)); // Reset buffer with 0xFF for next write
		}
	}
	// Remaining Data?
	if (i%sizeof(uint64_t) != 0)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, u32MemAdd+(i-(i%sizeof(uint64_t))), *((uint64_t*)u8Data)) != HAL_OK)
		{
			// Error occurred while page write.
			//uint32_t u32ErrCode = HAL_FLASH_GetError();
			HAL_FLASH_GetError();
			HAL_FLASH_Lock();
			return HAL_FAIL;
		}
	}

	// Lock the Flash to disable the flash control register access.
	HAL_FLASH_Lock();
	return HAL_SUCCESS;
}
/**
  * @brief  This function is used read data from memory address
  * @param  u32MemAdd,pData,u16DataLen
  * @retval HAL_ApiState
  */
HAL_ApiState FLASH_CLASS::FLASH_MemReadData(uint32_t u32MemAdd, uint8_t *pData, uint16_t u16DataLen)
{
    for (uint32_t i = 0; i < u16DataLen; i++)
    {
        pData[i] = *(__IO uint8_t*)(u32MemAdd + i);
    }
    return HAL_SUCCESS;
}
