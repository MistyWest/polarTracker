/*
 * BME280.h
 *
 *  Created on: Aug 24, 2018
 *      Author: klockwood
 */

#ifndef SENSORS_BME280_H_
#define SENSORS_BME280_H_

#define I2C_MODE 											1
#define SPI_MODE 											0

#define DEVICE_ID											0x76  //SDO tied to gnd

#define BME280_TEMP_PRESS_CALIB_DATA_LEN	26 +1 // add one for SPI control byte
#define BME280_HUMIDITY_CALIB_DATA_LEN		7  +1 // add one for SPI control byte
#define BME280_ALL_SENSOR_DATA_LEN				8  +1 // add pme fpr SPI control byte

//Register names:
#define BME280_TEMP_PRESS_CALIB_REG		0x88
#define BME280_DIG_H1_REG							0xA1

#define BME280_CHIP_ID_REG						0xD0 //Chip ID
#define BME280_RST_REG								0xE0 //Softreset Reg

#define BME280_HUIMD_CALIB_REG				0xE1

#define BME280_CTRL_HUMIDITY_REG			0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG								0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG					0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG							0xF5 //Configuration Reg
#define BME280_PRESSURE_MSB_REG				0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG				0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG			0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG				0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG				0xFE //Humidity LSB

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
							(bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

typedef struct
{
	float temperature;
	float humidity;
	float pressure;
}bme280_sensor_data_t;

/*!
 * @brief Calibration data
 */
typedef struct
{
 /**
 * @ Trim Variables
 */
/**@{*/
	uint16_t 	dig_T1;
	int16_t 	dig_T2;
	int16_t 	dig_T3;
	uint16_t 	dig_P1;
	int16_t 	dig_P2;
	int16_t 	dig_P3;
	int16_t 	dig_P4;
	int16_t 	dig_P5;
	int16_t 	dig_P6;
	int16_t 	dig_P7;
	int16_t 	dig_P8;
	int16_t 	dig_P9;
	uint8_t  	dig_H1;
	int16_t 	dig_H2;
	uint8_t  	dig_H3;
	int16_t		dig_H4;
	int16_t 	dig_H5;
	int8_t  	dig_H6;
	int32_t 	t_fine;
}bme280_calibration_data_t;

void BME280_sleep(void);
void BME280_wake(void);

float BME280_compensate_temperature(	long int temperature_data );
float BME280_compensate_pressure(	long int presssure_data );
float BME280_compensate_humidity(	long int humidity_data );

long int BME280_read_temperature(void);
long int BME280_read_pressure(void);
long int BME280_read_humidity(void);

void BME280_read_all_sensors( bme280_sensor_data_t * sensor_data );
void BME280_compensate_data( bme280_sensor_data_t * sensor_data );

uint8_t	BME280_read_ID(void);

uint8_t BME280_read_status(void);
uint8_t	BME280_read_configuration(void);
uint8_t	BME280_read_measurement_control(void);
uint8_t	BME280_read_humidity_control(void);

void BME280_initialize();


#endif /* SENSORS_BME280_H_ */
