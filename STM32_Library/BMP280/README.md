# BMP280 Library for STM32

## Overview
this library has been tested with Nucleo-F401RE. the interface between STM32 and BMP280 is SPI 4 wire. Nucleo-F401RE Use SPI1 and 8 Mbps (Max 10 Mhz for BMP280) for baudrate.

## Hardware Required

- 1 x BMP280
- 1 x Nucleo-F401RE
- 6 x female-female jumper
- 1 x mini USB to USB Male

## Wiring Diagram

| Pin BMP280    | Pin Nucleo-F401RE  | 
| :---:         | :---:              | 
| VCC           | VCC (3.3 V)        | 
| GND           | GND                | 
| SCL           | PA5 (SCK)          | 
| SDA           | PA7 (MOSI)         | 
| CSB           | PA4 (CS)           |
| SDO           | PA6 (MISO)         |

## Software Configuration

the system clock is set at 16 Mhz. the prescaler is set 2 to 8 Mhz clock SPI. this is SPI parameter:
```
hspi1.Instance = SPI1;
hspi1.Init.Mode = SPI_MODE_MASTER;
hspi1.Init.Direction = SPI_DIRECTION_2LINES;
hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
hspi1.Init.NSS = SPI_NSS_SOFT;
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
hspi1.Init.CRCPolynomial = 10;
```

CS (chip select) is driven by GPIOA. output level is **High**.
this is GPIO Parameter:
```
GPIO_InitStruct.Pin = GPIO_PIN_4;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
```  

## Documentation

![Nucleo-F401RE](/Assets/nucleo.jpg)

![Live Expression](/Assets/expression.PNG)

![SVM Output](/Assets/svm.PNG)