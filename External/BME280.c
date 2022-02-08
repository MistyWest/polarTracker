/*
 * BME280.c
 *
 *  Created on: Sep 13, 2018
 *      Author: klockwood
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "nrf_error.h"
#include "nrf_delay.h"

#include "mw_spi_master.h"
#include "BME280.h"
#include "../CLI_Logging/mw_logging.h"


bme280_calibration_data_t sensor_calibration;

static bool	bme280_initialized = false;

static uint8_t BME280_SPI_ID;

#define BME280_SUCCESS			0
#define BME280_PARAM_ERROR	99
#define BME280_STATE_ERROR	4


#define BME280_OPERATIONAL	if(!bme280_initialized) return BME280_STATE_ERROR;

static void BME280_set_measurement_control( uint8_t config );

/*
static void BME280_handler( mw_spi_device_evt_t const * spi_event )
{
	//TODO do stuff here
	MW_LOG_INFO("Received SPI data %d", spi_event->rx_buffer[0]);
}*/


void BME280_sleep()
{
	uint8_t config = BME280_read_measurement_control();
	config = config & 0xFC; // Set Sleep bits
	BME280_set_measurement_control(config);
}

void BME280_wake()
{
	uint8_t config = BME280_read_measurement_control();
	config = config | 0x03;	// Set Wake bits
	BME280_set_measurement_control(config);

	config = BME280_read_measurement_control();
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
static inline void parse_temp_press_calib_data( uint8_t * reg_data, bme280_calibration_data_t * calib_data)
{
	calib_data->dig_T1 = (int16_t)BME280_CONCAT_BYTES(reg_data[2], reg_data[1]);
	calib_data->dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[4], reg_data[3]);
	calib_data->dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[6], reg_data[5]);
	calib_data->dig_P1 = (int16_t)BME280_CONCAT_BYTES(reg_data[8], reg_data[7]);
	calib_data->dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[10], reg_data[9]);
	calib_data->dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[12], reg_data[11]);
	calib_data->dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[14], reg_data[13]);
	calib_data->dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[16], reg_data[15]);
	calib_data->dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[18], reg_data[17]);
	calib_data->dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[20], reg_data[19]);
	calib_data->dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[22], reg_data[21]);
	calib_data->dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[24], reg_data[23]);
	calib_data->dig_H1 = reg_data[26];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
static void parse_humidity_calib_data( uint8_t * reg_data, bme280_calibration_data_t * calib_data)
{
	calib_data->dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[2], reg_data[1]);
	calib_data->dig_H3 = reg_data[3];

	calib_data->dig_H4 = ((int16_t)(int8_t)reg_data[4] * 16) | ((int16_t)(reg_data[5] & 0x0F));

	calib_data->dig_H5 = ((int16_t)(int8_t)reg_data[6] * 16) | ((int16_t)(reg_data[5] >> 4));
	calib_data->dig_H6 = (int8_t)reg_data[7];
}

static void BME280_parse_sensor_data( uint8_t * raw_data, bme280_sensor_data_t * sensor_data )
{
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)raw_data[1] << 12;
	data_lsb = (uint32_t)raw_data[2] << 4;
	data_xlsb = (uint32_t)raw_data[3] >> 4;
	sensor_data->pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)raw_data[4] << 12;
	data_lsb = (uint32_t)raw_data[5] << 4;
	data_xlsb = (uint32_t)raw_data[6] >> 4;
	sensor_data->temperature = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for humidity data */
	data_lsb = (uint32_t)raw_data[7] << 8;
	data_msb = (uint32_t)raw_data[8];
	sensor_data->humidity = data_msb | data_lsb;
}

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
static void BME280_get_calibration_data( bme280_calibration_data_t * calib_data )
{
	uint8_t tx_data[BME280_TEMP_PRESS_CALIB_DATA_LEN];
	uint8_t rx_data[BME280_TEMP_PRESS_CALIB_DATA_LEN];

	/* Read the calibration data from the sensor */
	tx_data[0] = BME280_TEMP_PRESS_CALIB_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));
	parse_temp_press_calib_data(rx_data, calib_data);

	memset( tx_data, 0, sizeof(tx_data) );

	/* Read the calibration data from the sensor */
	tx_data[0] = BME280_HUIMD_CALIB_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));
	parse_humidity_calib_data(rx_data, calib_data);
}

float BME280_compensate_temperature(	long int temperature_data )
{
	float var1;
	float var2;

	var1 = ( ( (temperature_data) / 16384.0 - (sensor_calibration.dig_T1) / 1024.0 ) * (sensor_calibration.dig_T2) );

	var2 = 	((temperature_data / 131072.0) - (sensor_calibration.dig_T1 / 8192.0 )) *
					((temperature_data / 131072.0) - (sensor_calibration.dig_T1 / 8192.0 )) *
				  (sensor_calibration.dig_T3);

	sensor_calibration.t_fine = (int32_t)(var1 + var2);
	var1 = (var1 + var2) / 5120.0;

	if (var1 < -40) return -40;
	if (var1 > 85) return 85;
	return var1;
}


/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in float data type.
 */
float BME280_compensate_pressure(	long int pressure_data  )
{

	long long int var1;
	long long int var2;
	long long int p;

	var1 = ((long long int)sensor_calibration.t_fine) - 128000;
	var2 = var1 * var1 * (long long int)sensor_calibration.dig_P6;
	var2 = var2 + ((var1*(long long int)sensor_calibration.dig_P5) << 17);
	var2 = var2 + (((long long int)sensor_calibration.dig_P4) << 35);
	var1 = ((var1 * var1 * (long long int)sensor_calibration.dig_P3) >> 8) + ((var1 * (long long int)sensor_calibration.dig_P2) << 12);
	var1 = (((((long long int)1) << 47) + var1))*((long long int)sensor_calibration.dig_P1) >> 33;
	p = 1048576 - pressure_data;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((long long int)sensor_calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((long long int)sensor_calibration.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((long long int)sensor_calibration.dig_P7) << 4);

	return (float)p / 256000;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in float data type.
 */
float BME280_compensate_humidity(	long int humidity_data  )
{
	float var1;
	float var2;
	float var3;
	float var4;
	float var5;
	float var6;

	var1 = ((float)sensor_calibration.t_fine) - 76800.0;
	var2 = (((float)sensor_calibration.dig_H4) * 64.0 + (((float)sensor_calibration.dig_H5) / 16384.0) * var1);
	var3 = humidity_data - var2;
	var4 = ((float)sensor_calibration.dig_H2) / 65536.0;
	var5 = (1.0 + (((float)sensor_calibration.dig_H3) / 67108864.0) * var1);
	var6 = 1.0 + (((float)sensor_calibration.dig_H6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	var5 = var6 * (1.0 - ((float)sensor_calibration.dig_H1) * var6 / 524288.0);

	if (var5 > 100)
		var5 = 100;
	else if (var5 < 0)
		var5 = 0;

	return var5;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in float data type.
 */
void BME280_compensate_data( bme280_sensor_data_t * sensor_data )
{
	sensor_data->temperature = BME280_compensate_temperature(sensor_data->temperature);
	sensor_data->pressure 	 = BME280_compensate_pressure(sensor_data->pressure);
	sensor_data->humidity 	 = BME280_compensate_humidity(sensor_data->humidity);
}


//@read
void BME280_read_all_sensors( bme280_sensor_data_t * sensor_data )
{
	/*uint8_t data[8];
	data[0] = BME280_PRESSURE_MSB_REG;*/
	uint8_t tx_data[2];
	uint8_t rx_data[BME280_ALL_SENSOR_DATA_LEN];

	tx_data[0] = BME280_PRESSURE_MSB_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	BME280_parse_sensor_data(rx_data, sensor_data);
}

long int BME280_read_temperature(void)
{
	/*uint8_t data[3];
	data[0] = BME280_TEMPERATURE_MSB_REG;
  return 	data[0] << 12 | data[1] << 4 | data[2] >> 4;*/

	uint8_t tx_data[4];
	uint8_t rx_data[4];
	memset( tx_data, 0, sizeof(tx_data) );
	memset( rx_data, 0, sizeof(rx_data) );

	tx_data[0] = BME280_TEMPERATURE_MSB_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[0] << 12 | rx_data[1] << 4 | rx_data[2] >> 4;
}

long int BME280_read_pressure(void)
{
/*	uint8_t data[3];
	data[0] = BME280_PRESSURE_MSB_REG;
	return 	data[0] << 12 | data[1] << 4 | data[2] >> 4;*/

	uint8_t tx_data[2];
	uint8_t rx_data[3];

	tx_data[0] = BME280_TEMPERATURE_MSB_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}

long int BME280_read_humidity(void)
{
	//return 	data[0] << 8 | data[1];

	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = BME280_HUMIDITY_MSB_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ,  tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}

uint8_t	BME280_read_measurement_control(void)
{
	uint8_t tx_data[2] = {0,0};
	uint8_t rx_data[2] = {0,0};

	tx_data[0] = BME280_CTRL_MEAS_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}

uint8_t	BME280_read_humidity_control(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = BME280_CTRL_HUMIDITY_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}


uint8_t	BME280_read_configuration(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = BME280_CONFIG_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}


uint8_t	BME280_read_status(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = BME280_STAT_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}


// Should return a value of 0x60
uint8_t	BME280_read_ID(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = BME280_CHIP_ID_REG;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	//MW_LOG_DEBUG("BME280 Device ID: %x", rx_data[1]);
	return rx_data[1];
}

static void BME280_set_humidity_control( uint8_t config )
{
	uint8_t tx_data[2];
	memset( tx_data, 0, sizeof(tx_data) );

	tx_data[0] = BME280_CTRL_HUMIDITY_REG;
	tx_data[1] = config;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), NULL, 0);
}

static void BME280_set_measurement_control( uint8_t config )
{
	uint8_t data[2];
	memset( data, 0, sizeof(data) );

	data[0] = BME280_CTRL_MEAS_REG;
	data[1] = config;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_WRITE, data, sizeof(data), NULL, 0);
}

static void BME280_set_configuration( uint8_t config )
{
	uint8_t data[2];
	memset( data, 0, sizeof(data) );

	data[0] =  BME280_CONFIG_REG;
	data[1] = config;
	mw_spi_master_transfer(BME280_SPI_ID, SPI_WRITE, data, sizeof(data), NULL, 0);
}

void BME280_initialize()
{
	mw_spim_device_config_t BME280_device;
	spi_device_init_return_t return_value;

	BME280_device.spi_instance 		= (2);
	BME280_device.mode 						= NRF_SPIM_MODE_0;
	BME280_device.bit_order 			= NRF_SPI_BIT_ORDER_MSB_FIRST;
	BME280_device.frequency				= NRF_SPI_FREQ_8M;
	BME280_device.miso_pin 				= 22;
	BME280_device.mosi_pin 				= 23;
	BME280_device.ss_pin 					=	24;
	BME280_device.sck_pin 				= 25;
	BME280_device.ss_active_high 	= false;
	BME280_device.handler 				= NULL; //BME280_handler;
	BME280_device.irq_priority   	= NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY;
	return_value = mw_spi_master_device_init(BME280_device);

	if(return_value.err_code != NRF_SUCCESS) APP_ERROR_CHECK(return_value.err_code);

	BME280_SPI_ID = return_value.device_id;

	MW_LOG_INFO("BME280_device SPI Initialized");

	bme280_initialized = true;

	uint8_t config;

	config 		= BME280_read_ID();			//should be 0x05
	if( config != 0x60 ) MW_LOG_INFO("BME280_device Chip Id Error");

	BME280_set_humidity_control(0x05);		//Set oversampling x16
	config 		= BME280_read_humidity_control();			//should be 0x05
	if( config != 0x05 ) MW_LOG_INFO("BME280_device Device Initialization Error");

	BME280_set_measurement_control(0xB7);	//Temperature & Pressure oversampling x16, Normal Mode
	config 		= BME280_read_measurement_control();	//should be 0xB7
	if( config != 0xB7 ) MW_LOG_INFO("BME280_device Device Initialization Error");

	BME280_set_configuration(0x00); 			//Filter off, t_sb setting 0.5msec
	config 		= BME280_read_configuration();				//should be 0x00
	if( config != 0x00 ) MW_LOG_INFO("BME280_device Device Initialization Error");

	//Get calibration
	BME280_get_calibration_data(&sensor_calibration);
}
