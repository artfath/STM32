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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "qpc.h"                 // QP/C real-time embedded framework
#include "blinky.h"              // Blinky Application interface
#include "bsp.h"                 // Board Support Package

#include "qp_port.h"
#include "qsafe.h"        // QP Functional Safety (FuSa) Subsystem
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCB_SYSPRI   ((uint32_t volatile *)0xE000ED18U)
#define NVIC_EN      ((uint32_t volatile *)0xE000E100U)
#define NVIC_IP      ((uint32_t volatile *)0xE000E400U)
#define SCB_CPACR   *((uint32_t volatile *)0xE000ED88U)
#define FPU_FPCCR   *((uint32_t volatile *)0xE000EF34U)
#define NVIC_PEND    0xE000E200
#define SCB_ICSR     0xE000ED04
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VAL(x) #x
#define STRINGIFY(x) VAL(x)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
//void NMI_Handler(void)
//{
//  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
//////
//  /* USER CODE END NonMaskableInt_IRQn 0 */
//  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
//////   while (1)
//////  {
//////  }
//  /* USER CODE END NonMaskableInt_IRQn 1 */
//}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

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
//void PendSV_Handler(void)
//{
//  /* USER CODE BEGIN PendSV_IRQn 0 */
//////
//  /* USER CODE END PendSV_IRQn 0 */
//  /* USER CODE BEGIN PendSV_IRQn 1 */
//////
//  /* USER CODE END PendSV_IRQn 1 */
//}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */
	    QK_ISR_ENTRY();   // inform QK about entering an ISR

	    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); // time events at rate 0
	    // Perform the debouncing of buttons. The algorithm for debouncing
	        // adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
	        // and Michael Barr, page 71.
	        //
	        // state of the button debouncing
	        static struct {
	            uint32_t depressed;
	            uint32_t previous;
	        } buttons = { 0U, 0U };

	        uint32_t current = ~GPIOC->IDR; // Port C with state of Button B1
	        uint32_t tmp = buttons.depressed; // save the depressed buttons
	        buttons.depressed |= (buttons.previous & current); // set depressed
	        buttons.depressed &= (buttons.previous | current); // clear released
	        buttons.previous   = current; // update the history
	        tmp ^= buttons.depressed;     // changed debounced depressed
	        current = buttons.depressed;

//	        if ((tmp & (1U << 13U)) != 0U) { // debounced BTN state changed?
//	            if ((current & (1U << 13U)) != 0U) { // is BTN depressed?
//	                static QEvt const pauseEvt = QEVT_INITIALIZER(TEST_SIG);
//	                QACTIVE_PUBLISH(&pauseEvt, &l_SysTick_Handler);
//	            }
//	            else { // the button is released
//	                static QEvt const serveEvt = QEVT_INITIALIZER(TEST_SIG);
//	                QACTIVE_PUBLISH(&serveEvt, &l_SysTick_Handler);
//	            }
//	        }

	    #ifdef Q_SPY
	        tmp = SysTick->CTRL; // clear CTRL_COUNTFLAG
	        QS_tickTime_ += QS_tickPeriod_; // account for the clock rollover
	    #endif
	    QK_ISR_EXIT();    // inform QK about exiting an ISR
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  static QEvt const testEvt = QEVT_INITIALIZER(TEST_SIG);
        QACTIVE_POST(AO_Blinky, &testEvt, (void *)0);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
