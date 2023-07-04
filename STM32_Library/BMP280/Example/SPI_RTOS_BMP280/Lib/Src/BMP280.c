/*
 * BMP280.c
 *
 *  Created on: Jul 4, 2023
 *      Author: Muhammad Fatahila
 */

#include "BMP280.h"


/*
 * Read Register BP280
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * address -> 8 bit address register
 * value -> pointer to received data
 * len -> amount data to be received
 * --------------------
 * Return value boolean
 */
static bool BMP280_Read_Register(BMP280_HandleTypeDef *dev, uint8_t address, uint8_t *value, uint16_t len)
{
	uint8_t txbuff[1];
	txbuff[0]=address | 0x80; /*bits 7 Read Command '1'*/

	HAL_GPIO_WritePin(dev->gpioport, dev->gpiopin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, txbuff, 1, 5000);
	if(HAL_SPI_Receive(dev->hspi, value, len,5000) == HAL_OK)
	{
		HAL_GPIO_WritePin(dev->gpioport, dev->gpiopin, GPIO_PIN_SET);
		return true;
	} else {
		return false;
	}
}

/*
 * Write Register BP280
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * address -> 8 bit address value
 * cmd -> 8 bit command value
 * --------------------
 * Return value boolean
 */
static bool BMP280_Write_Register(BMP280_HandleTypeDef *dev, uint8_t address, uint8_t cmd)
{
	uint8_t txbuff[2];
	txbuff[0] = address & 0x7F; /*bits 7 Write Command '0'*/
	txbuff[1]=cmd;

	HAL_GPIO_WritePin(dev->gpioport, dev->gpiopin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(dev->hspi, txbuff, 2, 5000) == HAL_OK)
	{
		HAL_GPIO_WritePin(dev->gpioport, dev->gpiopin, GPIO_PIN_SET);
		HAL_Delay(100);
		return true;
	} else
	{
		return false;
	}
}

/*
 * Get Id BMP280
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * id -> pointer to received data
 * --------------------
 * Return value boolean
 */
static bool BMP280_Get_Id(BMP280_HandleTypeDef *dev, uint8_t *id)
{
	if (BMP280_Read_Register(dev, REGISTER_ID, id, 1) == true){
		HAL_Delay(100);
		return true;
	} else {
		return false;
	}
}

/*
 * Soft Reset BMP280
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * --------------------
 * Return value boolean
 */
static bool BMP280_Reset(BMP280_HandleTypeDef *dev)
{
	if(BMP280_Write_Register(dev, REGISTER_RESET, RESET_VALUE) == true)
	{
		return true;
	} else {
		return false;
	}
}

/*
 * Get Calibrations Parameter BMP280
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * --------------------
 * Return value boolean
 */
static bool BMP280_Compesation_Params(BMP280_HandleTypeDef *dev)
{
	uint8_t temp[24]={0};

	if(BMP280_Read_Register(dev, DIG_T1_LSB, temp, 24) == true)
	{
		dev->calibparams.dig_T1 = (uint16_t)(temp[1]<<8 | temp[0]);
		dev->calibparams.dig_T2 = (temp[3]<<8 | temp[2]);
		dev->calibparams.dig_T3 = (temp[5]<<8 | temp[4]);
		dev->calibparams.dig_P1 = (uint16_t)(temp[7]<<8 | temp[6]);

		dev->calibparams.dig_P2 = (temp[9]<<8 | temp[8]);
		dev->calibparams.dig_P3 = (temp[11]<<8 | temp[10]);
		dev->calibparams.dig_P4 = (temp[13]<<8 | temp[12]);
		dev->calibparams.dig_P5 = (temp[15]<<8 | temp[14]);
		dev->calibparams.dig_P6 = (temp[17]<<8 | temp[16]);
		dev->calibparams.dig_P7 = (temp[19]<<8 | temp[18]);
		dev->calibparams.dig_P8 = (temp[21]<<8 | temp[20]);
		dev->calibparams.dig_P9 = (temp[23]<<8 | temp[22]);

		return true;
	} else  {
		return false;
	}
}
/*
 * Set Control Measurement for oversampling temperature, oversampling pressure and mode
 */
static bool BMP280_Ctrl_Meas(BMP280_HandleTypeDef *dev, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode)
{
	uint8_t config;
	config = osrs_t | osrs_p |mode ;
	if (BMP280_Write_Register(dev, REGISTER_CTRL_MEAS, config) == true){
		return true;
	} else {
		return false;
	}
}

/*
 * Set Configuration BMP280 for standby time, filter and SPI 3 Wire
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * standby -> selecting standby time
 * filter -> selecting filter
 * spi3 -> enable SPI 3 Wire
 * --------------------
 * Return value boolean
 */
static bool BMP280_Config(BMP280_HandleTypeDef *dev, uint8_t standby, uint8_t filter, uint8_t spi3)
{
	uint8_t config;
	config = standby | filter |spi3;
	if (BMP280_Write_Register(dev, REGISTER_CONFIG, config) == true){
			return true;
		} else {
			return false;
		}
}

/*
 * Initialization BMP280
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * --------------------
 * Return value boolean
 */
bool BMP280_Init(BMP280_HandleTypeDef *dev)
{

	if(BMP280_Reset(dev) != true){
		return false;
	}

	HAL_Delay(500);


	if(!BMP280_Get_Id(dev, &dev->id)){
		return false;
		}

	if(dev->id != CHIP_ID){
			return false;
		}

	if(!BMP280_Compesation_Params(dev)){
		return false;
	}

	/*ultra high resolution indoor navigation*/
	/*default control measurement */
	dev->ctrlmeas.mode = PWR_NORMAL;
	dev->ctrlmeas.osr_t =CTRL_MEAS_TEMP_2;
	dev->ctrlmeas.osr_p =CTRL_MEAS_PRESS_16;

	/*default configurations */
	dev->config.t_sb = STANDBY_125;
	dev->config.filter = FILTER_4;
	dev->config.spi3w_en = SPI_3W_DISABLE;

	if(BMP280_Ctrl_Meas(dev, dev->ctrlmeas.osr_t, dev->ctrlmeas.osr_p, dev->ctrlmeas.mode) != true)
	{
		return false;
	}

	if(BMP280_Config(dev, dev->config.t_sb, dev->config.filter, dev->config.spi3w_en) != true){
		return false;
	}

	HAL_Delay(50);
	return true;

}

/*
 * Compensation Temperature algorithm
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * adc_T -> Raw temperature value
 * --------------------
 * Returns temperature in DegC, resolution is 0.01 DegC
 */
double BMP280_Compensate_Temperature(BMP280_HandleTypeDef *dev, int32_t adc_T)
{
	int32_t var1, var2, T;

	var1 = ((((adc_T / 8) - ((int32_t)dev->calibparams.dig_T1 * 2))) * ((int32_t)dev->calibparams.dig_T2)) /2048;
	var2 = (((((adc_T /16) - ((int32_t)dev->calibparams.dig_T1)) * ((adc_T / 16) - ((int32_t)dev->calibparams.dig_T1))) / 4096) *
				((int32_t)dev->calibparams.dig_T3)) / 16384;
	dev->calibparams.t_fine = var1 + var2;
	T = (dev->calibparams.t_fine * 5 + 128) / 256;
	return (double)T / 100.0;
}

/*
 * Compensation Pressure algorithm
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * adc_P -> Raw Pressure value
 * --------------------
 * Returns Pressure in Pa
 */
double BMP280_Compensate_Pressure(BMP280_HandleTypeDef *dev, int32_t adc_P)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)dev->calibparams.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dev->calibparams.dig_P6;
	var2 = var2 + ((var1*(int64_t)dev->calibparams.dig_P5) *131072);
	var2 = var2 + (((int64_t)dev->calibparams.dig_P4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t)dev->calibparams.dig_P3)/256) + ((var1 * (int64_t)dev->calibparams.dig_P2) *4096);
	var1 = (((((int64_t)1) << 47) + var1))*((int64_t)dev->calibparams.dig_P1) / 8589934592;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - adc_P;
	p = (((p<<31) - var2) * 3125) / var1;
	var1 = (((int64_t)dev->calibparams.dig_P9) * (p/8192) * (p/8192)) /33554432;
	var2 = (((int64_t)dev->calibparams.dig_P8) * p) /524288;
	p = ((p + var1 + var2) /256) + (((int64_t)dev->calibparams.dig_P7)*16);
	return (double)p / 256.0;
}

/*
 * Get Temperature and Pressure Value
 * --------------------
 * dev -> pointer to BMP280 Handle structure that contains the information BMP280 module
 * temperature -> pointer to data buffer temperature
 * pressure -> pointer to data buffer pressure
 * --------------------
 * Return value boolean
 */
bool BMP280_Get_Temperature_Pressure(BMP280_HandleTypeDef *dev, double *temperature, double *pressure)
{
	uint8_t temp_p[6];
	int32_t adc_P, adc_T;

	if(BMP280_Read_Register(dev, REGISTER_PRESS_MSB, temp_p, 6) == true)
	{
		adc_P = (int32_t)((temp_p[0] << 12) | (temp_p[1] << 4) | (temp_p[2] >> 4));
		adc_T = (int32_t)((temp_p[3] << 12) | (temp_p[4] << 4) | (temp_p[5] >> 4));

		*temperature = BMP280_Compensate_Temperature(dev,adc_T);
		*pressure = BMP280_Compensate_Pressure(dev, adc_P);

		return true;
	} else {
		return false;
	}

}
