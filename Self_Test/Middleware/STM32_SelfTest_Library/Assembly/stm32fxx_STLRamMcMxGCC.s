/**
  ******************************************************************************
  * @file      stm32fxx_STLRamMcMxGCC.s
  * @author    MCD Application Team
  * @version   V2.1.0
  * @date      19-Dec-2016
  * @brief     This file contains procedures written in assembler for full and partial
  *            transparent Marching RAM tests to be called during start-up and run time
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

  /* Reference to the FailSafe routine to be executed in case of non-recoverable
     failure */
  .extern FailSafePOR

  /* reference to C variables for control flow monitoring */
  .extern CtrlFlowCnt
  .extern CtrlFlowCntInv

  .global  STL_FullRamMarchC
  .global  STL_TranspRamMarchCXStep

/* tables with offsets of physical order of address in RAM */
  .section  .text.__STANDARD_RAM_ORDER
  .align 4
  .type  __STANDARD_RAM_ORDER, %object
  .size  __STANDARD_RAM_ORDER, .-__STANDARD_RAM_ORDER
__STANDARD_RAM_ORDER:
        .word  -4
        .word   0
        .word   4
        .word   8
        .word   12
        .word   16
        .word   20
        .word   24
        .word   28

  .section  .text.__ARTISAN_RAM_ORDER
  .align 4
  .type  __ARTISAN_RAM_ORDER, %object
  .size  __ARTISAN_RAM_ORDER, .-__ARTISAN_RAM_ORDER
__ARTISAN_RAM_ORDER:
        .word  -8
        .word   0
        .word   4
        .word   12
        .word   8
        .word   16
        .word   20
        .word   28
        .word   24

/**
 ******************************************************************************
 * @brief   Full RAM MarchC test for start-up
 *          WARNING > All the RAM area including stack is destroyed during this test
 * Compilation paramater : ARTISAN - changes order of the sequence of tested
 *                                   addresses to respect their physical order
 * @param   R0 .. RAM begin (first address to check), 
 *          R1 .. RAM end (last address to check)
 *          R2 .. Background pattern
 *          > R3 .. Inverted background pattern (local)
 *          > R4 .. result status (local)
 *          > R5  .. pointer to RAM (local)
 *          > R6  .. content RAM to compare (local)
 * @retval : TEST_SUCCESSFULL(=1) TEST_ERROR(=0)
*/

  .section  .text.STL_FullRamMarchC
  .type  STL_FullRamMarchC, %function
  .size  STL_FullRamMarchC, .-STL_FullRamMarchC

STL_FullRamMarchC:
  //-----R4 - R7 BACKUP SEQUENCE-----
  //backup R2 and R3, used in following DMA backup procedure
/*  PUSH {R1-R3}
  //store R4-R7
  //enable DMA1 to store backup of registers
  LDR R1, =0x40023830 //AHB1ENR register address
  LDR R3, [R1]          //R1 backup of AHBENR
  //LDR R3, [R3]
  //MOVS R2, #(1<<21) 	  //DMA1EN clock enable bit position
  ORRS R3, R3, #(1<<21)	  //set DMA1EN bit, default after MCU reset
  LDR R2, =0x40023830
  STR R3, [R2] /*R3 -> AHB1ENR*/
  //store registers R4-R7
 /* LDR R3, =0x40026000     // DMA1 base address -> R3
  STR R4, [R3, #+0x1C]    // R4 -> DMA CMAR1
  STR R5, [R3, #+0x34]    // R5 -> DMA CMAR2
  STR R6, [R3, #+0x4C]    // R6 -> DMA CMAR3
  STR R7, [R3, #+0x64]    // R7 -> DMA CMAR4
  STR R1, [R3, #+0x7C]    // R1 -> DMA CMAR5
  //restore R1 - R3 backup
  POP {R1-R3}*/
  //---------------------------------

  MOVS  R4, #0x1       /* Test success status by default */
  
  MOVS  R3,R2          /* setup inverted background pattern */
  RSBS  R3, R3, #0
  SUBS  R3,R3, #1
  
/* *** Step 1 *** */
/* Write background pattern with addresses increasing */
  MOVS  R5,R0
__FULL1_LOOP:
  CMP   R5,R1
  BHI   __FULLSTEP_2
  STR   R2,[R5, #+0]
  ADDS  R5,R5,#+4
  B     __FULL1_LOOP
    
/* *** Step 2 *** */
/* Verify background and write inverted background with addresses increasing */
__FULLSTEP_2:
  MOVS  R5,R0
__FULL2_LOOP:
  CMP   R5,R1
  BHI   __FULLSTEP_3
  LDR   R6,[R5,#+0]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+0]
  LDR   R6,[R5,#+4]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+4]
.ifdef ARTISAN
  LDR   R6,[R5,#+12]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+12]
  LDR   R6,[R5,#+8]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+8]
 .else
  LDR   R6,[R5,#+8]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+8]
  LDR   R6,[R5,#+12]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+12]
 .endif /* ARTISAN */
  ADDS  R5,R5,#+16
  B     __FULL2_LOOP
  
/* *** Step 3 *** */
/* Verify inverted background and write background with addresses increasing   */
__FULLSTEP_3:
  MOVS  R5,R0
__FULL3_LOOP:
  CMP   R5,R1
  BHI   __FULLSTEP_4  
  LDR   R6,[R5,#+0]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+0]
  LDR   R6,[R5,#+4]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+4]
.ifdef ARTISAN
  LDR   R6,[R5,#+12]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+12]
  LDR   R6,[R5,#+8]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+8]
.else
  LDR   R6,[R5,#+8]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+8]
  LDR   R6,[R5,#+12]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+12]
.endif /* ARTISAN */
  ADDS  R5,R5,#+16
  B     __FULL3_LOOP

/* *** Step 4 *** */
/* Verify background and write inverted background with addresses decreasing */
__FULLSTEP_4:
  MOVS  R5,R1
  SUBS  R5,R5,#+15
__FULL4_LOOP:
  CMP   R5,R0
  BLO   __FULLSTEP_5
.ifdef ARTISAN
  LDR   R6,[R5,#+8]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+8]
  LDR   R6,[R5,#+12]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+12]
 .else
  LDR   R6,[R5,#+12]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+12]
  LDR   R6,[R5,#+8]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+8]
 .endif /* ARTISAN */
  LDR   R6,[R5,#+4]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+4]
  LDR   R6,[R5,#+0]
  CMP   R6,R2
  BNE   __FULL_ERR
  STR   R3,[R5,#+0]
  SUBS  R5,R5,#+16
  B     __FULL4_LOOP
  
/* *** Step 5 *** */
/* Verify inverted background and write background with addresses decreasing */
__FULLSTEP_5:
  MOVS  R5,R1
  SUBS  R5,R5,#+15
__FULL5_LOOP:
  CMP   R5,R0
  BLO   __FULLSTEP_6
.ifdef ARTISAN
  LDR   R6,[R5,#+8]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+8]
  LDR   R6,[R5,#+12]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+12]
.else
  LDR   R6,[R5,#+12]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+12]
  LDR   R6,[R5,#+8]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+8]
 .endif /* ARTISAN */
  LDR   R6,[R5,#+4]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+4]
  LDR   R6,[R5,#+0]
  CMP   R6,R3
  BNE   __FULL_ERR
  STR   R2,[R5,#+0]
  SUBS  R5,R5,#+16
  B     __FULL5_LOOP

/* *** Step 6 *** */
/* Verify background with addresses increasing */
__FULLSTEP_6:
  MOVS  R5,R0
__FULL6_LOOP:
  CMP   R5,R1
  BHI   __FULL_RET
  LDR   R6,[R5,#+0]
  CMP   R6,R2
  BNE   __FULL_ERR
  ADDS  R5,R5,#+4
  B     __FULL6_LOOP

__FULL_ERR:
  MOVS  R4,#0       /* error result */

__FULL_RET:
  MOVS  R0,R4

  //-----R4 - R7 RESTORE SEQUENCE------
  //backup R2 and R3, used in following DMA backup procedure
/*  PUSH {R2-R3}
  //restore R4-R7
  LDR R3, =0x40026000   // DMA1 base address -> R3
  MOVS R2, #0 			//default CMAR value by reference manual
  LDR R4, [R3, #+0x1C]    // DMA CMAR1 -> R4
  LDR R5, [R3, #+0x34]    // DMA CMAR2 -> R5
  LDR R6, [R3, #+0x4C]    // DMA CMAR3 -> R6
  LDR R7, [R3, #+0x64]    // DMA CMAR4 -> R7
  LDR R2, [R3, #+0x7C]    // DMA CMAR5 -> R2
  //restore AHBENR
  LDR R3, =0x40023830 //AHBENR register address
  STR R2, [R3]
  //restore R2 and R3 backup
  POP {R2-R3}*/
  //------------------------------------

  BX    LR          /* return to the caller */

/**
 ******************************************************************************
 * @brief   Transparent RAM MarchC-/March X test for run time
 *          WARNING - original content of the area under test is not valid during the test!
 *          Neighbour addresses (first-1 or -2 and last+1) are tested, too.
 * Compilation paramaters : ARTISAN - changes order of the sequence of tested
 *                                    addresses to respect their physical order
 *                  USE_MARCHX_TEST - Skip step 3 and 4 of March C- to make the test
 *                                    shorter and faster but less efficient overall
 * @param   R0 .. RAM begin (first address to check), 
 *          R1 .. Buffer begin (First address of backup buffer)
 *          R2 .. Background pattern
 *          > R4 .. pointer to physical address order (local)
 * @retval : TEST_SUCCESSFULL(=1) TEST_ERROR(=0)
*/

  .section  .text.STL_TranspRamMarchCXStep
  .type  STL_TranspRamMarchCXStep, %function
  .size  STL_TranspRamMarchCXStep, .-STL_TranspRamMarchCXStep

STL_TranspRamMarchCXStep:
  PUSH  {R4-R7}

  LDR   R5,=ISRCtrlFlowCnt  /* Control flow control */
  LDR   R6,[R5]
  ADDS  R6,R6,#11
  STR   R6,[R5]
  
  MOVS  R3,R2               /* setup inverted background pattern (R3) */
  RSBS  R3, R3, #0
  SUBS  R3,R3, #1  

.ifdef ARTISAN
  LDR   R4, =__ARTISAN_RAM_ORDER /* setup pointer to physical order of the addresses (R4) */
.else
  LDR   R4, =__STANDARD_RAM_ORDER
.endif /* ARTISAN */

  MOVS  R5,R0       /* backup buffer to be tested? */
  CMP   R5,R1
  BEQ   __BUFF_TEST
  
/* ***************** test of the RAM slice ********************* */
  MOVS  R5, #0       /* NO - save content of the RAM slice into the backup buffer */
__SAVE_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* load data from RAM */
  ADD   R5,R5,#4     /* original data are stored starting from second item of the buffer */
  STR   R7,[R1, R5]  /* (first and last items are used for testing purpose exclusively) */
  CMP   R5, #20
  BLE   __SAVE_LOOP
  
/* *** Step 1 *** */
/* Write background pattern with addresses increasing */
  MOVS  R5, #0
__STEP1_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  STR   R2,[R0, R6]  /* store background pattern */
  ADD   R5,R5,#4
  CMP   R5, #20
  BLE   __STEP1_LOOP
  
/* *** Step 2 *** */
/* Verify background and write inverted background with addresses increasing */
  MOVS  R5, #0
__STEP2_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify background pattern */
  CMP   R7, R2
  BNE   __STEP_ERR
  STR   R3,[R0, R6]  /* store inverted background pattern */
  ADD   R5,R5,#4
  CMP   R5, #20
  BLE   __STEP2_LOOP

.ifndef USE_MARCHX_TEST
/* *** Step 3 *** (not used at March-X test)  */
/* Verify inverted background and write background with addresses increasing */
  MOVS  R5, #0
__STEP3_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify inverted background pattern */
  CMP   R7, R3
  BNE   __STEP_ERR
  STR   R2,[R0, R6]  /* store background pattrern */
  ADD   R5,R5,#4
  CMP   R5, #20
  BLE   __STEP3_LOOP
  
/* *** Step 4 *** (not used at March-X test) */
/* Verify background and write inverted background with addresses decreasing */
  MOVS  R5, #24
__STEP4_LOOP:
  SUBS  R5,R5,#4
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify background pattern */
  CMP   R7, R2
  BNE   __STEP_ERR
  STR   R3,[R0, R6]  /* store inverted background pattrern */
  CMP   R5, #0      
  BHI   __STEP4_LOOP
.endif /* March-X  */
  
/* *** Step 5 *** */
/* Verify inverted background and write background with addresses decreasing  */
  MOVS  R5, #24
__STEP5_LOOP:
  SUBS  R5,R5,#4
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify inverted background pattern */
  CMP   R7, R3
  BNE   __STEP_ERR
  STR   R2,[R0, R6]  /* store background pattrern */
  CMP   R5, #0
  BHI   __STEP5_LOOP

/* *** Step 6 *** */
/* Verify background with addresses increasing */
  MOVS  R5, #0
__STEP6_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify background pattern */
  CMP   R7, R2
  BNE   __STEP_ERR
  ADD   R5,R5,#4
  CMP   R5, #20
  BLE   __STEP6_LOOP

  MOVS  R5, #24      /* restore content of the RAM slice back from the backup buffer */
__RESTORE_LOOP:
  LDR   R7,[R1, R5]  /* (first and last items are used for testing purpose exclusively) */
  SUB   R5,R5,#4     /* original data are stored starting from second item of the buffer */
  LDR   R6,[R4, R5]  /* load data offset */
  STR   R7,[R0, R6]  /* load data from RAM */
  CMP   R5, #0
  BHI   __RESTORE_LOOP
  
  B     __MARCH_RET

/* ************** test of the buffer itself ******************** */
__BUFF_TEST:
/* *** Step 1 ***  */
/* Write background pattern with addresses increasing */
  MOVS  R5, #4
__BUFF1_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  STR   R2,[R0, R6]  /* store background pattern */
  ADD   R5,R5,#4
  CMP   R5, #32
  BLE   __BUFF1_LOOP
  
/* *** Step 2 *** */
/* Verify background and write inverted background with addresses increasing */
  MOVS  R5, #4
__BUFF2_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify background pattern */
  CMP   R7, R2
  BNE   __STEP_ERR
  STR   R3,[R0, R6]  /* store inverted background pattern */
  ADD   R5,R5,#4
  CMP   R5, #32
  BLE   __BUFF2_LOOP
  
.ifndef USE_MARCHX_TEST
/* *** Step 3 *** (not used at March-X test) */
/* Verify inverted background and write background with addresses increasing */
  MOVS  R5, #4
__BUFF3_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify inverted background pattern */
  CMP   R7, R3
  BNE   __STEP_ERR
  STR   R2,[R0, R6]  /* store  background pattern */
  ADD   R5,R5,#4
  CMP   R5, #32
  BLE   __BUFF3_LOOP

/* *** Step 4 *** (not used at March-X test) */
/* Verify background and write inverted background with addresses decreasing */
  MOVS  R5, #36
__BUFF4_LOOP:
  SUBS  R5,R5,#4
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify background pattern */
  CMP   R7, R2
  BNE   __STEP_ERR
  STR   R3,[R0, R6]  /* store inverted background pattrern */
  CMP   R5, #4
  BHI   __BUFF4_LOOP
.endif /* March-X  */

/* *** Step 5 *** */
/* Verify inverted background and write background with addresses decreasing */
  MOVS  R5, #36
__BUFF5_LOOP:
  SUBS  R5,R5,#4
  LDR   R6,[R4, R5]  /* load data offset  */
  LDR   R7,[R0, R6]  /* verify inverted background pattern */
  CMP   R7, R3
  BNE   __STEP_ERR
  STR   R2,[R0, R6]  /* store background pattrern */
  CMP   R5, #4
  BHI   __BUFF5_LOOP

/* *** Step 6 *** */
/* Verify background with addresses increasing */
  MOVS  R5, #4
__BUFF6_LOOP:
  LDR   R6,[R4, R5]  /* load data offset */
  LDR   R7,[R0, R6]  /* verify background pattern */
  CMP   R7, R2
  BNE   __STEP_ERR
  ADD   R5,R5,#4
  CMP   R5, #32
  BLE   __BUFF6_LOOP

__MARCH_RET:
  LDR   R4,=ISRCtrlFlowCntInv  /* Control flow control */
  LDR   R5,[R4]
  SUBS  R5,R5,#11
  STR   R5,[R4]
  
  MOVS  R0, #1       /* Correct return */
  B     __STEP_RET
  
__STEP_ERR:
  MOVS  R0, #0       /* error result */
  
__STEP_RET:
  POP   {R4-R7}
  BX    LR           /* return to the caller */
	
/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE*****/
