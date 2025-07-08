/*
 * interface_card_board.h
 *
 *  Created on: Sep 27, 2024
 *      Author: Muhammad fatahila
 */

#ifndef INC_INTERFACE_CARD_BOARD_H_
#define INC_INTERFACE_CARD_BOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "interface_card_conf.h"

#include <stdint.h>
#include <stdio.h>

extern UART_HandleTypeDef UartHandle;

void MainMPU_Config(void);
void MainSystemClock_Config(void);
void StartUpClock_Config(void);
void USART_Configuration(void);


#endif /* INC_INTERFACE_CARD_BOARD_H_ */
