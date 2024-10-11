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
10. add qutest
11. code organization:
```bash
├── Core
├── Drivers
├── Middlewares
│   ├── include
│   ├── ports 
│   │    ├── arm-cm
│   │    │    └── qk
│   │    │      └── gnu/
│   │    └── windows-qutest
│   └── src
│       ├── qf/
│       └── qk/
├── qview
└── Test
    └── dpp
        └── src
        └── test_dpp
        └── test_philo
        └── test_table
 ```

11. build project and run

### Using Qtools

1. Copy qview-dpp from qpc file
2. add preprocessor Q-SPY in properties -> C/C++ Build -> settings -> MCU GCC compiler -> preprocessor
3. open cmd in folder project. run qspy ``` qspy -c <COM number> ```
4. open cmd in qview folder. run qview ``` python3 %QTOOLS%\qview\qview.py dpp.py ```
5. add Qutest. test in windows host
    * add test qutest dpp in folder Test
    * add win32-qutest in qpc ports
    * edit makefile document, edit qpc path according with the project
    * open qspy in terminal directory (test_dpp/test_philo/test_table). run qspy ```qspy```
    * open terminal in test directory(test_dpp/test_philo/test_table). run makefile ```make```
6. result qutest in host windows
    * test_dpp
    ```  
    QUTest unit testing front-end 7.3.4 running on Python 3.10.1
    Copyright (c) 2005-2024 Quantum Leaps, www.state-machine.com
    Attaching to QSpy (localhost:7701)... OK

    Run ID    : 241011_111127
    Target    : build/test_dpp.exe,localhost:6601

    ==================================[Group  1]==================================
    test_init.py
    [ 1]--------------------------------------------------------------------------
    DPP init
                                                                [ PASS (  0.2s) ]

    ==================================[Group  2]==================================
    test_tick.py
    [ 2]--------------------------------------------------------------------------
    tick
                                                                [ PASS (  0.2s) ]

    ==================================[ SUMMARY ]=================================

    Target ID : 241011_105800 (QP-Ver=734)
    Log file  :
    Groups    : 2
    Tests     : 2
    Skipped   : 0
    Failed    : 0

    ==============================[  OK  (  2.5s) ]===============================
    ```

    * test_philo
    ```  

    QUTest unit testing front-end 7.3.4 running on Python 3.10.1
    Copyright (c) 2005-2024 Quantum Leaps, www.state-machine.com
    Attaching to QSpy (localhost:7701)... OK

    Run ID    : 241011_110925
    Target    : build/test_philo.exe,localhost:6601

    ==================================[Group  1]==================================
    test_init.py
    [ 1]--------------------------------------------------------------------------
    init
                                                                [ PASS (  0.3s) ]

    ==================================[Group  2]==================================
    test_philo.py
    [ 2]--------------------------------------------------------------------------
    Philo-thinking tick
                                                                [ PASS (  0.2s) ]
    [ 3]^-------------------------------------------------------------------------
    Philo-hungry publish EAT
                                                                [ PASS (  0.0s) ]
    [ 4]^-------------------------------------------------------------------------
    Philo-eating tick
                                                                [ PASS (  0.0s) ]
    [ 5]^-------------------------------------------------------------------------
    Philo-thinking tick(2)
                                                                [ PASS (  0.0s) ]

    ==================================[ SUMMARY ]=================================

    Target ID : 241011_105508 (QP-Ver=734)
    Log file  :
    Groups    : 2
    Tests     : 5
    Skipped   : 0
    Failed    : 0

    ==============================[  OK  (  2.8s) ]===============================
    ```

    * test_table
    ```  

    QUTest unit testing front-end 7.3.4 running on Python 3.10.1
    Copyright (c) 2005-2024 Quantum Leaps, www.state-machine.com
    Attaching to QSpy (localhost:7701)... OK

    Run ID    : 241011_105952
    Target    : build/test_table.exe,localhost:6601

    ==================================[Group  1]==================================
    test_init.py
    [ 1]--------------------------------------------------------------------------
    init
                                                                [ PASS (  0.3s) ]

    ==================================[Group  2]==================================
    test_table.py
    [ 2]--------------------------------------------------------------------------
    post HUNGRY[2]
    on_setup
    on_teardown
                                                                [ PASS (  0.2s) ]
    [ 3]^-------------------------------------------------------------------------
    post HUNGRY[1]
    on_setup
    on_teardown
                                                                [ PASS (  0.0s) ]
    [ 4]^-------------------------------------------------------------------------
    publish DONE[2]
    on_setup
    on_teardown
                                                                [ PASS (  0.0s) ]

    ==================================[ SUMMARY ]=================================

    Target ID : 241011_105951 (QP-Ver=734)
    Log file  :
    Groups    : 2
    Tests     : 4
    Skipped   : 0
    Failed    : 0

    ==============================[  OK  (  2.8s) ]===============================
    ```