/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
 uint8_t SLV_ADDR = 0x28<<1; // Use 8-bit address

static const uint8_t REG_ST_RDY = 0x01<<0;//status ready
static const uint8_t REG_WR_CF = 0x01<<1;//write config
static const uint8_t REG_RD_CF = 0x01<<1 | 0x01;//read config
static const uint8_t REG_MD_LP = 0x01<<2;//mode loop
static const uint8_t REG_RD_PW = 0x01<<3;//read power
static const uint8_t REG_MF_ID = 0x01<<4;//manufacture id
static const uint8_t REG_VR_ID = 0x01<<5;//read power
uint8_t data_config[2]={0};
uint8_t rxbuf[2]={0};
uint8_t rx_uart;
int i=0;
bool state = true;
uint8_t buffer[12] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Task_action(char *message);
void Task_Send_Hex(char *message);
void Send_data(char *message);
void Send_data_hex(char *message);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, &rx_uart, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//		  buffer[1] = 0b01100101;//input ctrl on
//		  if(i==0){
//			  buffer[0] = REG_ST_RDY;
//			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);
//		  }
//		  else if(i==1){
//			  buffer[2] = 0b11100111; //write config
//			  buffer[0] = REG_WR_CF;
////			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 3);
//		  }else if(i==2){
//			  buffer[2] = 0b11100110; //read config
//			  buffer[0] = REG_WR_CF;
////			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 3);
//		  }else if(i==3){
//
//			  buffer[0] = REG_RD_PW;
////			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);
//		  }else if(i==4){
//			  buffer[0] = REG_MF_ID;
////			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);
//		  }else if(i==5){
//			  buffer[0] = REG_VR_ID;
////			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);
//		  }else{
//
//		  }
//
//		  HAL_Delay(700);
//		  i++;
//	  			if(i>5){
//	  				i=2;
//	  			}
//	  buffer[3] = REG_4;
//	  if(i == 1){
//		  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, &buf, 1);
//	  }else if(i == 2){
//		  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, &buf1, 1);
//	  }else if(i == 3){
//		  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, &buf2, 1);
//	  }else{
////		  Task_action("id");
//	  }
//
//	  HAL_Delay(1000);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 42000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 25;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
void Task_action(char *message)
{
	while(*message){
		ITM_SendChar(*message);
		message++;
	}

	ITM_SendChar('\n');
}
void Task_Send_Hex(char *message)
{
	char hex_digits[] = "0123456789ABCDEF";
	while(*message){

		ITM_SendChar(hex_digits[(*message >> 4) & 0x0F]);
		ITM_SendChar(hex_digits[(*message & 0x0F)]);
		message++;
		ITM_SendChar(' ');
	}

	ITM_SendChar('\n');
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
	HAL_I2C_Master_Receive_DMA(&hi2c1, SLV_ADDR, rxbuf, 2);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	Task_Send_Hex((char *)rxbuf);
	memset(rxbuf, 0, 2*sizeof(char));

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == B1_Pin && state == true){
		HAL_TIM_Base_Start_IT(&htim1);
		state = false;
	}
	else{
		__NOP();
	}
}
void Send_data(char *message)
{
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)message, strlen(message));
}

void Send_data_hex(char *message)
{
	char hex_digits[] = "0123456789ABCDEF";
	char *hex = malloc(4);
	int i=0;
	int n=0;
		while(*message){
			i+=n;
			hex[i] = hex_digits[(*message >> 4) & 0x0F];
			i++;
			hex[i] = hex_digits[(*message & 0x0F)];
			message++;
			n++;
		}
		HAL_UART_Transmit_DMA(&huart2,(uint8_t *)hex,4);
}
/*event callback when uart finish receive data/idle*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	Send_data(&rx_uart);
//	if(huart->Instance == USART2){
		  buffer[1] = 0b01100101;//input ctrl on
		  if(rx_uart== 0x30){
			  buffer[0] = REG_ST_RDY;
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);//0x65 101
		  }else if(rx_uart== 0x31){
			  buffer[2] = 0b11100111; //write config
			  buffer[0] = REG_WR_CF;
			  HAL_I2C_Master_Transmit(&hi2c1, SLV_ADDR, buffer, 3, 500); //0xFF 255
		  }else if(rx_uart== 0x32){
			  buffer[0] = REG_RD_CF;
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);//0xFF 255
		  }else if(rx_uart== 0x33){
			  buffer[0] = REG_RD_PW;
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);//0x65 101
		  }else if(rx_uart== 0x34){
			  buffer[0] = REG_MF_ID;
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);//0x65 101
		  }else if(rx_uart== 0x35){
			  buffer[0] = REG_VR_ID;
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 1);//0x65 101
		  }else{

		  }
		/*Receive command*/
//		rx_uart = 0;
		  HAL_UART_Receive_DMA(&huart2, &rx_uart, 1);
//		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

//	}

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance == TIM1){
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET){
			HAL_I2C_Master_Transmit_DMA(&hi2c1, SLV_ADDR, buffer, 3);
			i++;
			if(i>5){
				i=2;
			}
			state = true;
			HAL_TIM_Base_Stop_IT(&htim1);
		}
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
