/*
 * MPU9250.h
 *
 *  Created on: Feb 14, 2019
 *      Author: klockwood
 */

#ifndef PROSKIDA_SENSOR_COMM_H_
#define PROSKIDA_SENSOR_COMM_H_


#define TWI_DONT_STOP				1
#define TWI_ISSUE_STOP				0

#define ONE_DATA_BYTE								1
#define TWO_DATA_BYTES							2
#define THREE_DATA_BYTES						3
#define SIX_DATA_BYTES							6
#define FOURTEEN_DATA_BYTES					14


#define SENSOR_TWI_PRIORITY_LOW			6
#define SENSOR_TWI_FREQUENCY_HIGH		NRF_TWI_FREQ_400K

#define SENSOR_ACCEL_RANGE					ACCEL_SCALE_16G // ACCEL_SCALE_8G
#define SENSOR_GYRO_RANGE						GYRO_SCALE_2000dps

#define USER_LPF_FILTER							0x03

#define SENSOR_RES									(32767.5f)
#define GRAVITY											(9.807f)
#define DEG2RAD											(3.14159265359f/180.0f)

#define RESULTANT_THRESHOLD					(0.06f)

#define ADO 												0
#if ADO
#define MPU9250_ADDRESS 						0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 						0x68//<<1  // Device address when ADO = 0
#endif

#define		WHO_AM_I_VALUE						0x71

#define   WAKE_ON_MOTION_INT				0x40
#define   FIFO_OVERFLOW_INT					0x10
#define   RAW_DATA_READY_INT				0x01
#define   DISABLE_INTERRUPTS				0x00

#define		ENABLE_INTELLIGENCE_MODE  0xC0
#define		DISABLE_INTELLIGENCE_MODE 0x00

#define		ACCEL_SELF_TEST						0xE0

#define		ACCEL_SCALE_2G						0x00
#define		ACCEL_SCALE_4G						0x04
#define		ACCEL_SCALE_8G						0x10
#define		ACCEL_SCALE_16G						0x18

#define		ACCEL_LP_ODR_1HZ					0x02
#define		ACCEL_LP_ODR_62HZ					0x08
#define		ACCEL_LP_ODR_125HZ				0x09
#define		ACCEL_LP_ODR_250HZ				0x0A

#define		ACCEL_LPF_AND_BW_99HZ			0x0A
#define		ACCEL_LPF_AND_BW_10HZ			0x08

#define		ACCEL_FCHOICE_SET					0x08
#define		ACCEL_FCHOICE_CLEAR				0x00
#define		ACCEL_DLPFCFG_10HZ				0x05
#define		ACCEL_DLPFCFG_218HZ				0x01
#define		ACCEL_DLPFCFG_420HZ				0x07

#define		WAKE_THESHOLD_A						0xCF
#define		WAKE_THESHOLD_B						0xDF
#define		WAKE_THESHOLD_C						0xEF
#define		WAKE_THESHOLD_D						0xFF

#define		ACCEL_RESET								0x80
#define		PWR_MGMT_1_LP							0x00
#define		ENABLE_CYCLE_MODE					0x20
#define		PWR_MGMT_2_DEFAULT				0x00
#define		PWR_MGMT_2_LP							0x03

#define   DISABLE_ACCEL_XYZ					0x38
#define   DISABLE_GYRO_XYZ					0x07

#define		GYRO_SCALE_250dps					0x00
#define		GYRO_SCALE_500dps					0x04
#define		GYRO_SCALE_1000dps				0x10
#define		GYRO_SCALE_2000dps				0x18

//****************************************
//**************  Registers **************
#define 	SELF_TEST_X_GYRO					0x00
#define 	SELF_TEST_Y_GYRO					0x01
#define 	SELF_TEST_Z_GYRO					0x02
#define 	SELF_TEST_X_ACCEL					0x0D
#define 	SELF_TEST_Y_ACCEL					0x0E
#define 	SELF_TEST_Z_ACCEL					0x0F
#define 	XG_OFFSET_H								0x13
#define 	XG_OFFSET_L								0x14
#define 	YG_OFFSET_H								0x15
#define 	YG_OFFSET_L								0x16
#define 	ZG_OFFSET_H								0x17
#define 	ZG_OFFSET_L								0x18
#define 	SMPLRT_DIV								0x19
#define 	MPU_CONFIG								0x1A
#define 	GYRO_CONFIG								0x1B
#define 	ACCEL_CONFIG							0x1C
#define 	ACCEL_CONFIG_2						0x1D
#define 	LP_ACCEL_ODR							0x1E
#define 	WOM_THR										0x1F
#define 	FIFO_EN										0x23
#define 	I2C_MST_CTRL							0x24
#define 	I2C_SLV0_ADDR							0x25
#define 	I2C_SLV0_REG							0x26
#define 	I2C_SLV0_CTRL							0x27
#define 	I2C_SLV1_ADDR							0x28
#define 	I2C_SLV1_REG							0x29
#define 	I2C_SLV1_CTRL							0x2A
#define 	I2C_SLV2_ADDR							0x2B
#define 	I2C_SLV2_REG							0x2C
#define 	I2C_SLV2_CTRL							0x2D
#define 	I2C_SLV3_ADDR							0x2E
#define 	I2C_SLV3_REG							0x2F
#define 	I2C_SLV3_CTRL							0x30
#define 	I2C_SLV4_ADDR							0x31
#define 	I2C_SLV4_REG							0x32
#define 	I2C_SLV4_DO								0x33
#define 	I2C_SLV4_CTRL							0x34
#define 	I2C_SLV4_DI								0x35
#define 	I2C_MST_STATUS						0x36
#define 	INT_PIN_CFG								0x37
#define 	INT_ENABLE								0x38
#define 	INT_STATUS								0x3A
#define 	ACCEL_XOUT_H							0x3B
#define 	ACCEL_XOUT_L							0x3C
#define 	ACCEL_YOUT_H							0x3D
#define 	ACCEL_YOUT_L							0x3E
#define 	ACCEL_ZOUT_H							0x3F
#define 	ACCEL_ZOUT_L							0x40
#define 	TEMP_OUT_H								0x41
#define 	TEMP_OUT_L								0x42
#define 	GYRO_XOUT_H								0x43
#define 	GYRO_XOUT_L								0x44
#define 	GYRO_YOUT_H								0x45
#define 	GYRO_YOUT_L								0x46
#define 	GYRO_ZOUT_H								0x47
#define 	GYRO_ZOUT_L								0x48
#define 	EXT_SENS_DATA_00					0x49
#define 	EXT_SENS_DATA_01					0x4A
#define 	EXT_SENS_DATA_02					0x4B
#define 	EXT_SENS_DATA_03					0x4C
#define 	EXT_SENS_DATA_04					0x4D
#define 	EXT_SENS_DATA_05					0x4E
#define 	EXT_SENS_DATA_06					0x4F
#define 	EXT_SENS_DATA_07					0x50
#define 	EXT_SENS_DATA_08					0x51
#define 	EXT_SENS_DATA_09					0x52
#define 	EXT_SENS_DATA_10					0x53
#define 	EXT_SENS_DATA_11					0x54
#define 	EXT_SENS_DATA_12					0x55
#define 	EXT_SENS_DATA_13					0x56
#define 	EXT_SENS_DATA_14					0x57
#define 	EXT_SENS_DATA_15					0x58
#define 	EXT_SENS_DATA_16					0x59
#define 	EXT_SENS_DATA_17					0x5A
#define 	EXT_SENS_DATA_18					0x5B
#define 	EXT_SENS_DATA_19					0x5C
#define 	EXT_SENS_DATA_20					0x5D
#define 	EXT_SENS_DATA_21					0x5E
#define 	EXT_SENS_DATA_22					0x5F
#define 	EXT_SENS_DATA_23					0x60
#define 	I2C_SLV0_DO								0x63
#define 	I2C_SLV1_DO								0x64
#define 	I2C_SLV2_DO								0x65
#define 	I2C_SLV3_DO								0x66
#define 	I2C_MST_DELAY_CTRL				0x67
#define 	SIGNAL_PATH_RESET					0x68
#define 	MOT_DETECT_CTRL						0x69
#define 	USER_CTRL									0x6A
#define 	PWR_MGMT_1								0x6B
#define 	PWR_MGMT_2								0x6C
#define 	FIFO_COUNTH								0x72
#define 	FIFO_COUNTL								0x73
#define 	FIFO_R_W									0x74
#define 	WHO_AM_I									0x75
#define 	XA_OFFSET_H								0x77
#define 	XA_OFFSET_L								0x78
#define 	YA_OFFSET_H								0x7A
#define 	YA_OFFSET_L								0x7B
#define 	ZA_OFFSET_H								0x7D
#define 	ZA_OFFSET_L								0x7E
//****************************************
//****************************************


typedef struct
{
	uint8_t					data_lo;
	uint8_t					data_hi;
}axis_data_t;

//Data structure for sensor data - 6 bytes
typedef struct
{
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
}mpu9250_sensor_t;

typedef struct
{
	int16_t data;
}temperature_data_t;

typedef struct
{
	mpu9250_sensor_t		 			accelerometer_data;
	mpu9250_sensor_t 				gyroscope_data;
	temperature_data_t 		temperature_reading;
}sensor_fusion_data_t;

uint32_t sensor_go_to_sleep();

uint32_t reset_accelerometer();
uint32_t start_up_1_accelerometer();
uint32_t enable_cycle_mode_accelerometer();
uint32_t disable_cycle_mode_accelerometer();
uint32_t start_up_2_accelerometer();

uint32_t default_config_1_sensors_proskida();
uint32_t default_config_2_sensors_proskida();
uint32_t default_config_3_sensors_proskida();
uint32_t default_set_accel_range();
uint32_t enable_sensors_proskida();

uint32_t disable_intelligence_accelerometer();
uint32_t wake_on_motion_accelerometer_proskida();
uint32_t enable_interrupts_proskida();
uint32_t disable_interrupts_proskida();
uint32_t config_interrupts_proskida();
uint32_t unconfig_interrupts_proskida();
uint32_t clear_sleep_accel();
uint32_t request_reset_proskida();

uint32_t lp_config_1_sensors_proskida();
uint32_t lp_config_2_sensors_proskida();
uint32_t lp_config_3_sensors_proskida();

uint32_t low_power_odr_accelerometer_proskida();
uint32_t set_lpf_and_bw_accelerometer_proskida();
uint32_t enable_intelligence_accelerometer();
uint32_t start_up_2_lp_accelerometer();
uint32_t lp_setup_sensors_proskida();


uint32_t start_accel_self_test();
uint32_t end_accel_self_test();
uint32_t get_accel_bias_values();

uint32_t request_sensor_data();

sensor_fusion_data_t * get_sensor_data();

uint32_t request_accel_data();
mpu9250_sensor_t get_accel_data();

uint32_t request_gyro_data();
mpu9250_sensor_t get_gyro_data();

uint32_t request_temperature_data();
temperature_data_t get_temperature_data();

uint32_t read_power_managment_2();

float get_accel_scale_factor( uint8_t accel_scale );
float get_gyro_scale_factor( uint8_t gyro_scale );

void sensor_interrupt_enable();
void sensor_interrupt_disable();
void MPU9250_disable_communication();
void MPU9250_enable_communication();

//******************************************


uint32_t MPU9250_who_am_i();


void MPU9250_initialize( external_device_config_t device_confi );

#endif /* PROSKIDA_SENSOR_COMM_H_ */
