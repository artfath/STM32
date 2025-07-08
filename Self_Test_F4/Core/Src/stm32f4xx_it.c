/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include <stm32fxx_STLparam.h>
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32fxx_STLlib.h"
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
   volatile static uint32_t tmpCC4_last;	/* keep last TIM5/Chn4 captured value */
   uint32_t MyRAMCounter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
//#ifdef STL_VERBOSE
//  if (__HAL_RCC_GET_IT(RCC_IT_CSS))
//  {
//    while(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TC) == 0)
//    {   /* Wait previous transmission completion */
//    }
//    /* Re-configure USART baud rate to have 115200 bds with HSI clock (8MHz) */
//    USART_Configuration();
//    printf("\n\r Clock Source failure (Clock Security System)\n\r");
//  }
//  else
//  {
//    printf("\n\r NMI Exception\n\r");
//  }
//#endif /* STL_VERBOSE */


  FailSafePOR();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
//   while (1)
//  {
//  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
#ifdef STL_VERBOSE
  printf("\n\r Hard fault Exception \n\r");
#endif /* STL_VERBOSE */

  FailSafePOR();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  if ((TickCounter ^ TickCounterInv) == 0xFFFFFFFFuL)
    {
      TickCounter++;
      TickCounterInv = ~TickCounter;

      if (TickCounter >= SYSTICK_10ms_TB)
      {
        uint32_t RamTestResult;

        #if defined STL_EVAL_MODE
          /* Toggle LED2 for debug purposes */
//          BSP_LED_Toggle(LED2);
        #endif  /* STL_EVAL_MODE */

        /* Reset timebase counter */
        TickCounter = 0u;
        TickCounterInv = 0xFFFFFFFFuL;

        /* Set Flag read in main loop */
        TimeBaseFlag = 0xAAAAAAAAuL;
        TimeBaseFlagInv = 0x55555555uL;

        ISRCtrlFlowCnt += RAM_MARCHC_ISR_CALLER;
        __disable_irq();
        RamTestResult = STL_TranspMarch();
        __enable_irq();
        ISRCtrlFlowCntInv -= RAM_MARCHC_ISR_CALLER;

        switch ( RamTestResult )
        {
          case TEST_RUNNING:
            break;
          case TEST_OK:
            #ifdef STL_VERBOSE
            /* avoid any long string output here in the interrupt, '#' marks ram test completed ok */
              #ifndef __GNUC__
                putchar((int16_t)'#');
              #else
                __io_putchar((int16_t)'#');
              #endif /* __GNUC__ */
            #endif  /* STL_VERBOSE */
            #ifdef STL_EVAL_MODE
              /* Toggle LED2 for debug purposes */
//              BSP_LED_Toggle(LED2);
            #endif /* STL_EVAL_MODE */
            #if defined(STL_EVAL_LCD)
              ++MyRAMCounter;
            #endif /* STL_EVAL_LCD */
            break;
          case TEST_FAILURE:
          case CLASS_B_DATA_FAIL:
          default:
            #ifdef STL_VERBOSE
              printf("\n\r >>>>>>>>>>>>>>>>>>>  RAM Error (March C- Run-time check)\n\r");
            #endif  /* STL_VERBOSE */
            FailSafePOR();
            break;
        } /* End of the switch */

        /* Do we reached the end of RAM test? */
        /* Verify 1st ISRCtrlFlowCnt integrity */
        if ((ISRCtrlFlowCnt ^ ISRCtrlFlowCntInv) == 0xFFFFFFFFuL)
        {
          if (RamTestResult == TEST_OK)
          {
    /* ==============================================================================*/
    /* MISRA violation of rule 17.4 - pointer arithmetic is used to check RAM test control flow */
  	#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
  		#pragma diag_suppress=Pm088
  	#endif /* __IAR_SYSTEMS_ICC__ */
            if (ISRCtrlFlowCnt != RAM_TEST_COMPLETED)
  	#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
  		#pragma diag_default=Pm088
  	#endif /* __IAR_SYSTEMS_ICC__ */
    /* ==============================================================================*/
            {
            #ifdef STL_VERBOSE
              printf("\n\r Control Flow error (RAM test) \n\r");
            #endif  /* STL_VERBOSE */
            FailSafePOR();
            }
            else  /* Full RAM was scanned */
            {
              ISRCtrlFlowCnt = 0u;
              ISRCtrlFlowCntInv = 0xFFFFFFFFuL;
            }
          } /* End of RAM completed if*/
        } /* End of control flow monitoring */
        else
        {
        #ifdef STL_VERBOSE
          printf("\n\r Control Flow error in ISR \n\r");
        #endif  /* STL_VERBOSE */
        FailSafePOR();
        }
        #if defined STL_EVAL_MODE
          /* Toggle LED2 for debug purposes */
//          BSP_LED_Toggle(LED2);
        #endif  /* STL_EVAL_MODE */
      } /* End of the 10 ms timebase interrupt */
    }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */
  uint32_t tmpCC4_last_cpy;

  if ((TIM5->SR & TIM_SR_CC4IF) != 0u )
  {
    /* store previous captured value */
    tmpCC4_last_cpy = tmpCC4_last;
    /* get currently captured value */
    tmpCC4_last = TIM5->CCR4;
    /* The CC4IF flag is already cleared here be reading CCR4 register */

    /* overight results only in case the data is required */
    if (LSIPeriodFlag == 0u)
    {
      /* take correct measurement only */
      if ((TIM5->SR & TIM_SR_CC4OF) == 0u)
      {
        /* Compute period length */
        PeriodValue = tmpCC4_last - tmpCC4_last_cpy;
        PeriodValueInv = ~PeriodValue;

        /* Set Flag tested at main loop */
        LSIPeriodFlag = 0xAAAAAAAAuL;
        LSIPeriodFlagInv = 0x55555555uL;
      }
      else
      {
        /* ignore computation in case of IC overflow */
        TIM5->SR &= (uint16_t)(~TIM_SR_CC4OF);
      }
    }
    /* ignore computation in case data is not required */
  }
  /* USER CODE END TIM5_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/******************************************************************************/
/**
  * @brief Configure TIM5 to measure LSI period
  * @param  : None
  * @retval : ErrorStatus = (ERROR, SUCCESS)
  */
ErrorStatus STL_InitClock_Xcross_Measurement(void)
{
  ErrorStatus result = SUCCESS;
  TIM_HandleTypeDef  tim_capture_handle;
  TIM_IC_InitTypeDef tim_input_config;

  /*## Enable peripherals and GPIO Clocks ####################################*/
  /* TIMx Peripheral clock enable */
  __TIM5_CLK_ENABLE();

  /*## Configure the NVIC for TIMx ###########################################*/
  HAL_NVIC_SetPriority(TIM5_IRQn, 4u, 0u);

  /* Enable the TIM5 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  /* TIM5 configuration: Input Capture mode ---------------------
  The LSI oscillator is connected to TIM5 CH4.
  The Rising edge is used as active edge, ICC input divided by 8
  The TIM5 CCR4 is used to compute the frequency value.
  ------------------------------------------------------------ */
  tim_capture_handle.Instance = TIM5;
  tim_capture_handle.Init.Prescaler         = 0u;
  tim_capture_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  tim_capture_handle.Init.Period            = 0xFFFFFFFFul;
  tim_capture_handle.Init.ClockDivision     = 0u;
  tim_capture_handle.Init.RepetitionCounter = 0u;
  /* define internal HAL driver status here as handle structure is defined locally */
  __HAL_RESET_HANDLE_STATE(&tim_capture_handle);
  if(HAL_TIM_IC_Init(&tim_capture_handle) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  /* Connect internally the TIM5_CH4 Input Capture to the LSI clock output */
  HAL_TIMEx_RemapConfig(&tim_capture_handle, TIM_TIM5_LSI);

  /* Configure the TIM5 Input Capture of channel 4 */
  tim_input_config.ICPolarity  = TIM_ICPOLARITY_RISING;
  tim_input_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim_input_config.ICPrescaler = TIM_ICPSC_DIV8;
  tim_input_config.ICFilter    = 0u;
  if(HAL_TIM_IC_ConfigChannel(&tim_capture_handle, &tim_input_config, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }

  /* Reset the flags */
  tim_capture_handle.Instance->SR = 0u;
  LSIPeriodFlag = 0u;

  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT(&tim_capture_handle, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  return(result);
}
/* USER CODE END 1 */
