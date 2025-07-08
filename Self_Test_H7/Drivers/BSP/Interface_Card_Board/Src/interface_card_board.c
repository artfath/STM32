/*
 * interface_card_board.c
 *
 *  Created on: Sep 27, 2024
 *      Author: Muhammad Fatahila
 */

/* Includes ------------------------------------------------------------------*/
#include "interface_card_board.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;

void MainMPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
/* ---------------------------------------------------------------------------*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI - 16MHz / HSE - 8MHz)
  *            SYSCLK(MHz)                    = 84 MHz (CPU Clock)
  *            HCLK(MHz)                      = 84 MHz (AHBs Clock)
  *            AHB Prescaler                  = 1
  *            D1 APB1 Prescaler              = 2
  *            D2 APB2 Prescaler              = 1
  *            PLL_M                          = 4
  *            PLL_N                          = 84
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 2
  * @retval None
  */
void MainSystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Supply configuration update enable
	  */
	  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	  /** Configure LSE Drive Capability
	  */
	  HAL_PWR_EnableBkUpAccess();
	  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 5;
	  RCC_OscInitStruct.PLL.PLLN = 192;
	  RCC_OscInitStruct.PLL.PLLP = 2;
	  RCC_OscInitStruct.PLL.PLLQ = 20;
	  RCC_OscInitStruct.PLL.PLLR = 2;
	  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	  RCC_OscInitStruct.PLL.PLLFRACN = 0;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
//	    Error_Handler();
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
	                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
/* ---------------------------------------------------------------------------*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI - 16MHz)
  *            SYSCLK(MHz)                    = 84 MHz (CPU Clock)
  *            HCLK(MHz)                      = 84 MHz (AHBs Clock)
  *            AHB Prescaler                  = 1
  *            D1 APB1 Prescaler              = 2
  *            D2 APB2 Prescaler              = 1
  *            PLL_M                          = 16
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 2
  * @retval None
  */
void StartUpClock_Config(void)
{
//	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	  /** Supply configuration update enable
//	  */
//	  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//
//	  /** Configure the main internal regulator output voltage
//	  */
//	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
//
//	  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//	  /** Initializes the RCC Oscillators according to the specified parameters
//	  * in the RCC_OscInitTypeDef structure.
//	  */
//	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
//	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//	  RCC_OscInitStruct.PLL.PLLM = 5;
//	  RCC_OscInitStruct.PLL.PLLN = 192;
//	  RCC_OscInitStruct.PLL.PLLP = 2;
//	  RCC_OscInitStruct.PLL.PLLQ = 2;
//	  RCC_OscInitStruct.PLL.PLLR = 2;
//	  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
//	  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//	  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	  {
////	    Error_Handler();
//	  }
//
//	  /** Initializes the CPU, AHB and APB buses clocks
//	  */
//	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//	                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//	  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
//	  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//	  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//	  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//	  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//
//	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//	  {
////	    Error_Handler();
//	  }
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	  RCC_OscInitTypeDef RCC_OscInitStruct;

	  /*!< Supply configuration update enable */
	  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

	  /* voltage scaling */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
	  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	  /* upload the init structure by default values prior any its change to prevenet ovewrite of HSITRIM & CSITRIM */
	  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	  /* Enable HSI Oscillator and activate PLL with HSI as source */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
	  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	  RCC_OscInitStruct.PLL.PLLM = 4;
	  RCC_OscInitStruct.PLL.PLLN = 50;
	  RCC_OscInitStruct.PLL.PLLP = 2;
	  RCC_OscInitStruct.PLL.PLLR = 2;
	  RCC_OscInitStruct.PLL.PLLQ = 4;

	  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;

	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {

	  }

	/* Select PLL as system clock source and configure  bus clocks dividers */
	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
	                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

	  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {

	  }
}
/* ---------------------------------------------------------------------------*/
/**
 * @brief  Configure the UART peripheral
 * @param  None
 * @retval None
 */
void USART_Configuration(void) {
	  UartHandle.Instance = USART2;
	  UartHandle.Init.BaudRate = 115200;
	  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	  UartHandle.Init.StopBits = UART_STOPBITS_1;
	  UartHandle.Init.Parity = UART_PARITY_NONE;
	  UartHandle.Init.Mode = UART_MODE_TX_RX;
	  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	  UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  UartHandle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  UartHandle.FifoMode = UART_FIFOMODE_DISABLE;
	  UartHandle.NbRxDataToProcess = UART_RXFIFO_THRESHOLD_1_8;
	  UartHandle.NbTxDataToProcess = UART_TXFIFO_THRESHOLD_1_8;
	  __HAL_UART_RESET_HANDLE_STATE(&UartHandle);
	  if (HAL_UART_Init(&UartHandle) != HAL_OK)
	  {

	  }

	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  /* USER CODE BEGIN USART2_MspInit 0 */

	  /* USER CODE END USART2_MspInit 0 */

	  /** Initializes the peripherals clock
	  */
	    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
	    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    /* USART2 clock enable */
	    __HAL_RCC_USART2_CLK_ENABLE();

	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    /**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA3     ------> USART2_RX
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void USART_Reconfiguration(void) {
  UartHandle.Instance        = USART2;
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  UartHandle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  UartHandle.FifoMode = UART_FIFOMODE_DISABLE;
  UartHandle.NbRxDataToProcess = UART_RXFIFO_THRESHOLD_1_8;
  UartHandle.NbTxDataToProcess = UART_TXFIFO_THRESHOLD_1_8;
  __HAL_UART_RESET_HANDLE_STATE(&UartHandle);
  HAL_UART_Init(&UartHandle);
}

/* -------------------------------------------------------------------------*/
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


