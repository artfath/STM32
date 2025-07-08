/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal_def.h"
#include "stm32h743xx.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t MyRAMCounter;
extern uint32_t MyFLASHCounter;
extern uint32_t uwTickPrio;
//extern uint32_t uwTickFreq;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define USARTx                           USART2
/* Maximum Flash latency at given voltage range */
#define MAX_FLASH_LATENCY FLASH_LATENCY_7

/* legacy definitions */
#define IWDG IWDG1
#define WWDG WWDG1
#define RCC_FLAG_IWDGRST RCC_FLAG_IWDG1RST
#define RCC_FLAG_WWDGRST RCC_FLAG_WWDG1RST
#define RCC_FLAG_LPWRRST RCC_FLAG_LPWR1RST
#define __HAL_RCC_CLEAR_FLAG  __HAL_RCC_CLEAR_RESET_FLAGS
#define TIM_TIM16_LSI  TIM_TIM16_TI1_RCC_LSI
#define __DBGMCU_FREEZE_IWDG  __HAL_DBGMCU_FREEZE_IWDG1
#define __DBGMCU_FREEZE_WWDG  __HAL_DBGMCU_FREEZE_WWDG1

/* Exported functions ------------------------------------------------------- */
void STL_StartUp(void);
void SystemInit (void);
void SystemClock_Config(void);
void StartUpClock_Config(void);
void TIM16_IRQHandler(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
