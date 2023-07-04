/*
 * BMP280.h
 *
 *  Created on: Jul 4, 2023
 *      Author: Muhammad Fatahila
 */

#ifndef BMP280_H_
#define BMP280_H_

//#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdbool.h"


/**/
#define CHIP_ID					0x58
/* Register */
#define REGISTER_ID 			0xD0 /* Chip id address */
#define REGISTER_RESET 			0xE0 /* Reset address */
#define REGISTER_STATUS 		0xF3 /* Status address */
#define REGISTER_CTRL_MEAS 		0xF4 /* Control measurement address */
#define REGISTER_CONFIG 		0xF5 /* Configuration address */
#define REGISTER_PRESS_MSB 		0xF7 /* Pressure MSB address */
#define REGISTER_PRESS_LSB 		0xF8 /* Pressure LSB address */
#define REGISTER_PRESS_XLSB 	0xF9 /* Pressure XLSB address */
#define REGISTER_TEMP_MSB 		0xFA /* Temperature MSB address */
#define REGISTER_TEMP_LSB 		0xFB /* Temperature LSB address */
#define REGISTER_TEMP_XLSB 		0xFC /* Temperature XLSB address */


#define RESET_VALUE 			0xB6 /* Reset value */

/*
 * Register Status 0xF3
 *
 * Data Receive 8 bit :
 * ----------------------------------------------------------------------------------------------
 * 	bit 7	|	bit 6	|	bit 5	|	bit 4	|	bit 3	 |	bit 2	|	bit 1	|	bit 0	|
 * ----------------------------------------------------------------------------------------------
 * 	XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX|measuring[0]|XXXXXXXXXXXXXXXXXXXXXX|im_update[0]
 * ----------------------------------------------------------------------------------------------
 */

/*
 * --------------------------------------------------------
 * measuring[3]		| 	description
 * --------------------------------------------------------
 * 		0			|	measuring done
 * 		1			|	measuring running
 * --------------------------------------------------------
 */
#define MEAS_DONE 				0x00
#define MEAS_RUNNING			0x80

/*
 * --------------------------------------------------------
 * im-update[0]		| 	description
 * --------------------------------------------------------
 * 		0			|	image update done
 * 		1			|	image update running
 * --------------------------------------------------------
 */
#define IM_UPDATE_DONE 			0x00
#define IM_UPDATE_RUNNING		0x01


/*
 * Register Control Measurement 0xF4
 *
 * Data Value 8 bit :
 * ----------------------------------------------------------------------------------------------
 * 	bit 7	|	bit 6	|	bit 5	|	bit 4	|	bit 3	|	bit 2	|	bit 1	|	bit 0	|
 * ----------------------------------------------------------------------------------------------
 * 			osrs_t[2:0]				| 				osrs_p[2:0]			| 		mode[1:0]
 * ----------------------------------------------------------------------------------------------
 */

/*
 *
 * temperature oversampling osrs_t
 * --------------------------------------------------------
 * osrs_t[7,6,5]	| temp oversampling | temp resolution
 * --------------------------------------------------------
 * 		000			|		skip		| -
 * 		001			|		x1			| 16 bit/0.0050 °C
 * 		010			|		x2			| 17 bit/0.0025 °C
 * 		011			|		x4			| 18 bit/0.0012 °C
 * 		100			|		x8			| 19 bit/0.0006 °C
 * 101, 110, 111	|		x16			| 20 bit/0.0003 °C
 * --------------------------------------------------------
 */
#define CTRL_MEAS_TEMP_0 		0x00
#define CTRL_MEAS_TEMP_1 		0x20
#define CTRL_MEAS_TEMP_2 		0x40
#define CTRL_MEAS_TEMP_4 		0x60
#define CTRL_MEAS_TEMP_8 		0x80
#define CTRL_MEAS_TEMP_16 		0xA0

/*
 * Pressure oversampling osrs_p
 * --------------------------------------------------------
 * osrs_p[4,3,2]	| press oversampling| press resolution
 * --------------------------------------------------------
 * 		000			|		skip		| -
 * 		001			|		x1			| 16 bit/2.62 Pa
 * 		010			|		x2			| 17 bit/1.31 Pa
 * 		011			|		x4			| 18 bit/0.66 Pa
 * 		100			|		x8			| 19 bit/0.33 Pa
 * 101, 110, 111	|		x16			| 20 bit/0.16 Pa
 * --------------------------------------------------------
 */
#define CTRL_MEAS_PRES_0 		0x00
#define CTRL_MEAS_PRES_1 		0x04
#define CTRL_MEAS_PRESS_2		0x08
#define CTRL_MEAS_PRESS_4 		0x0C
#define CTRL_MEAS_PRESS_8 		0x10
#define CTRL_MEAS_PRESS_16 		0x14

/*
 * Power mode
 * ----------------------------
 * mode[1,0]	| Mode
 * ----------------------------
 * 		00		| Sleep mode
 * 	01 & 10		| Forced Mode
 * 		11		| Normal Mode
 * ----------------------------
 */
#define PWR_SLEEP 				0x00
#define PWR_FORCED 				0x01
#define PWR_NORMAL 				0x03

/*
 * Register Config 0xF5
 *
 * Data Value 8 bit :
 * --------------------------------------------------------------------------------------------------
 * 	bit 7	|	bit 6	|	bit 5	|	bit 4	|	bit 3	|	bit 2	|	bit 1	|	bit 0		|
 * --------------------------------------------------------------------------------------------------
 * 			t_sb[2:0]				| 				filter[2:0]			| XXXXXXXX	|	spi3w_en[0]
 * --------------------------------------------------------------------------------------------------
 */

/*
 * Standby time in Normal mode t_sb
 * ----------------------------
 * mode[7,6,5]	| Tstandby(ms)
 * ----------------------------
 * 		000		| 0.5
 * 		001		| 62.5
 * 		010		| 125
 * 		011		| 250
 *		100		| 500
 * 		101		| 1000
 *		110		| 2000
 *		111		| 4000
 * ----------------------------
 */
#define STANDBY_0_5 			0x00
#define STANDBY_62_5			0x20
#define STANDBY_125				0x40
#define STANDBY_250				0x60
#define STANDBY_500				0x80
#define STANDBY_1000			0xA0
#define STANDBY_2000			0xC0
#define STANDBY_4000			0xE0

/*
 * IIR filter
 * -------------------------------------------------------
 * filter[4,3,2]| filter coeff	|	sample to reach > 75%
 * -------------------------------------------------------
 * 		000		| 	filter off	|			1
 * 		001		| 		2		|			2
 * 		010		| 		4		|			5
 * 		011		| 		8		|			11
 *		100		| 		16		|			22
 * -------------------------------------------------------
 */
#define FILTER_0 				0x00
#define FILTER_2				0x04
#define FILTER_4				0x08
#define FILTER_8				0x0C
#define FILTER_16				0x10

/*
 * Enable 3 Wire SPI mode spi3w_en
 * ----------------------------
 * spi3w_en[0]	| Enable 3 wire SPI
 * ----------------------------
 * 		0		| 	4 wire SPI
 * 		1		| 	3 wire SPI
 * ----------------------------
 */
#define SPI_3W_DISABLE 			0x00
#define SPI_3W_ENABLE			0x01

/*
 *
 * Compensation parameter storage
 * --------------------------------------------------------
 * Register address	| Register content	| Data Type
 * 		(LSB/MSB)	|					|
 * --------------------------------------------------------
 * 		0x88/0x89	|		dig_T1		| unsigned short
 * 		0x8A/0x8B	|		dig_T2		| signed short
 * 		0x8C/0x8D	|		dig_T3		| signed short
 * 		0x8E/0x8F	|		dig_P1		| unsigned short
 * 		0x90/0x91	|		dig_P2		| signed short
 * 		0x92/0x93	|		dig_P3		| signed short
 * 		0x94/0x95	|		dig_P4		| signed short
 * 		0x96/0x97	|		dig_P5		| signed short
 * 		0x98/0x99	|		dig_P6		| signed short
 * 		0x9A/0x9B	|		dig_P7		| signed short
 * 		0x9C/0x9D	|		dig_P8		| signed short
 * 		0x9E/0x9F	|		dig_P9		| signed short
 * 		0xA0/0xA1	|		reserved	| reserved
 * --------------------------------------------------------
 */
#define DIG_T1_LSB				0x88
#define DIG_T1_MSB 				0x89
#define DIG_T2_LSB				0x8A
#define DIG_T2_MSB 				0x8B
#define DIG_T3_LSB				0x8C
#define DIG_T3_MSB 				0x8D
#define DIG_P1_LSB				0x8E
#define DIG_P1_MSB 				0x8F
#define DIG_P2_LSB				0x90
#define DIG_P2_MSB 				0x91
#define DIG_P3_LSB				0x92
#define DIG_P3_MSB 				0x93
#define DIG_P4_LSB				0x94
#define DIG_P4_MSB 				0x95
#define DIG_P5_LSB				0x96
#define DIG_P5_MSB 				0x97
#define DIG_P6_LSB				0x98
#define DIG_P6_MSB 				0x99
#define DIG_P7_LSB				0x9A
#define DIG_P7_MSB 				0x9B
#define DIG_P8_LSB				0x9C
#define DIG_P8_MSB 				0x9D
#define DIG_P9_LSB				0x9E
#define DIG_P9_MSB 				0x9F

/*
 *  BMP280 Status Structure
 */
typedef struct __BMP280_StatusTypeDef
{
	uint8_t measuring;
	uint8_t im_update;

}BMP280_StatusTypeDef;

/*
 *  BMP280 Control Measurement Structure
 */
typedef struct __BMP280_CtrlMeasTypeDef
{
	uint8_t osr_t;
	uint8_t osr_p;
	uint8_t mode;

}BMP280_CtrlMeasTypeDef;

/*
 *  BMP280 Configuration Structure
 */
typedef struct __BMP280_ConfigTypeDef
{
	uint8_t t_sb;
	uint8_t filter;
	uint8_t spi3w_en;

}BMP280_ConfigTypeDef;

/*
 *  BMP280 Calibration Parameter Structure
 */
typedef struct __BMP280_Calib_ParamsTypeDef
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	int32_t t_fine;

}BMP280_Calib_ParamsTypeDef;

/*
 *  BMP280 Handle Structure
 */
typedef struct __BMP280_HandleTypeDef
{
	uint8_t id;

	BMP280_CtrlMeasTypeDef ctrlmeas;

	BMP280_ConfigTypeDef config;

	BMP280_Calib_ParamsTypeDef calibparams;

	GPIO_TypeDef *gpioport;

	uint16_t gpiopin;

	SPI_HandleTypeDef *hspi;

}BMP280_HandleTypeDef;



bool BMP280_Init(BMP280_HandleTypeDef *dev);
double BMP280_Compensate_Temperature(BMP280_HandleTypeDef *dev, int32_t adc_T);
double BMP280_Compensate_Pressure(BMP280_HandleTypeDef *dev, int32_t adc_P);
bool BMP280_Get_Temperature_Pressure(BMP280_HandleTypeDef *dev, double *temperature, double *pressure);

#endif /* BMP280_H_ */
