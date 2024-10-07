## Porting the QPC DPP to a STM32 NUCLEO-F401RE:

1. preparing example file dpp from github [link ](https://github.com/QuantumLeaps/qpc-examples.git)(qpc-examples\arm-cm\dpp_nucleo-c031c6)
2. preparing qpc file from github [link ](https://github.com/QuantumLeaps/qpc.git)
    * add embedded software package. help -> embedded software packs -> from url
    * add link ``` https://raw.githubusercontent.com/QuantumLeaps/cmsis-packs/refs/heads/main/QuantumLeaps.pidx ```
3. create new project: STM32CubeIDE -> file -> new -> STM32 Project 
4. select board: board Manager -> NUCLEO-F401RE
5. in the CubeMX (.ioc file) 
	* Project Manager settings check “Generate peripheral initialization as a pair of c./.h files per peripheral”
	* add qpc in middleware dan project software
	* in system core "code generation" unchecked non maskable interrupt (NMI), pendable request for system service and systick timer
	* enable interrupt in USART
6. copy main code from example file main.c to nucleo main.c
7. copy philo.c, table.c and bsp.c to Core\Src. changing and customizing microcontroller types and HAL library
8. copy dpp.h and bsp.h to Core\Inc
9. create QPC folder: right click in nucleo project -> new -> source folder
10. code organization:
```bash
├── Core
├── Drivers
└── Middlewares
   ├── include
   ├── ports 
   │    └── arm-cm
   │        └── qk
   │           └── gnu/
   └── src
       ├── qf/
       └── qk/
 ```

11. build project and run

### Using Qtools

1. Copy qview-dpp from qpc file
2. add preprocessor Q-SPY in properties -> C/C++ Build -> settings -> MCU GCC compiler -> preprocessor
3. open cmd in folder project. run qspy ``` qspy -c <COM number> ```
4. open cmd in qview folder. run qview ``` python3 %QTOOLS%\qview\qview.py dpp.py ```