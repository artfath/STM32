## add self test library STM32 to a STM32 NUCLEO-F401RE:

1. preparing self test library version 2.2.0 from st.com [link ](https://www.st.com/en/embedded-software/x-cube-classb.html)
2. create new project: STM32CubeIDE -> file -> new -> STM32 Project 
3. select board: board Manager -> NUCLEO-F401RE
4. in the CubeMX (.ioc file) Project Manager settings check “Generate peripheral initialization as a pair of c./.h files per peripheral”
5. in .ioc add IWDG, WWDG and CRC activated and click "device configuration tool code generation"
6. in .ioc add timer 5 channel 4 remap , enable nvic/interupt and click "device configuration tool code generation"
7. create Middleware folder: right click in nucleo project -> new -> source folder
8. copy STM32_SelfTest_Library folder to middleware folder (en.x-cube_classb\Middlewares\STM32_SelfTest_Library)
9. copy  assembly folder to STM32_SelfTest_Library folder (en.x-cube_classb\Projects\STM324xG_EVAL\SW4STM32\Assembly)
10. edit file startup.s(Core/Startup) same with startup.s in assembly folder
11. add file stl_param.h (en.x-cube_classb\Projects\STM324xG_EVAL\Inc) to folder core (Core/Inc)
12. modification it.c (Core/Src) fit to it.c (en.x-cube_classb\Projects\STM324xG_EVAL\Src\stm32f4xx_it.c)
13. edit USART to UART adjust with stm32 type
14. generate post build crc to check flash integrity
	- download srerecord (https://srecord.sourceforge.net/download.html)
	- install it
	- copy srec_cat.exe (location C:\Program Files\srecord\bin) to project folder root
15. modification Flash.Id fit to flash.id (en.x-cube_classb\Projects\STM324xG_EVAL\SW4STM32\STM324xG_EVAL\STM32F417IGHx_FLASH.ld), add CRC post build to _Check_Sum

16. CPU Test
17. Flash Test
18. RAM Test
19. Clock Test
	- check if use HSE/external clock or not, if not comment in STLparam.h
	- setting clock value



6. copy main code from example file main.c to nucleo main.c
7. copy philo.c, table.c and bsp.c to Core\Src. changing and customizing microcontroller types and HAL library
8. copy dpp.h and bsp.h to Core\Inc
9. create QPC folder: right click in nucleo project -> new -> source folder
10. Copy QPC\include from qpc file to QPC folder nucleo
11. Copy QPC\src (just qf and qk folder) from qpc file to QPC folder nucleo
12. Copy QPC\ports\arm-cm\qk qpc file to QPC folder nucleo (just gnu and config folder)
13. move .h file from gnu and config folder to QPC\include
14. Now in CubeIDE Project Settings -> C/C++ General -> Paths and Symbols -> Includes:
Add QPC\include: IMPORTANT: THESE ARE ONLY THE PATHS THAT INCLUDE THE REQUIRED .h FILES (not the .c files). 
15. code organization:
```bash
├── Core
├── Drivers
└── QPC
   ├── include
   ├── ports 
   │    └── arm-cm
   │        └── qk
   │           └── gnu/
   └── src
       ├── qf/
       └── qk/
 ```


16. comment SysTick_Handler() and EXTI15_10_IRQHandler() in stm32f4xx_it.c because its defined in QPC
17. comment NMI_Handler() PendSV_Handler() in stm32f4xx_it.c because its defined in QPC
18. build project and run