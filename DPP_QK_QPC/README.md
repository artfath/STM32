## Porting the QPC DPP to a STM32 NUCLEO-F401RE:

1. preparing example file dpp from github [link ](https://github.com/QuantumLeaps/qpc-examples.git)(qpc-examples\arm-cm\dpp_nucleo-c031c6)
2. preparing qpc file from github [link ](https://github.com/QuantumLeaps/qpc.git)
3. create new project: STM32CubeIDE -> file -> new -> STM32 Project 
4. select board: board Manager -> NUCLEO-F401RE
5. in the CubeMX (.ioc file) Project Manager settings check “Generate peripheral initialization as a pair of c./.h files per peripheral”
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