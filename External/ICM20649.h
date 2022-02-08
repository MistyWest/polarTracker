/*
 * ICM20649.h
 *
 *  Created on: Jan 30, 2019
 *      Author: klockwood
 */

#ifndef ICM20649_H_
#define ICM20649_H_

#include "nordic_common.h"

//******************************************
//*** Values
#define ICM20649_WHO_AM_I_CONST       	0xE1

#define ICM20649_PWR_MGMT_1_RESET_VALUE	0x41

#define ICM20649_BIT_RESET            	0x80
#define ICM20649_ENABLE_ACCEL_AXIS			0x67  // bit 7=0
#define ICM20649_ENABLE_GYRO_AXIS				0x78  // bit 7=0

#define ICM20649_ENABLE_FSYNC_INT(x)		SET_BIT(x,7)
#define ICM20649_ENABLE_DMP_INT2(x)			SET_BIT(x,4)
#define ICM20649_ENABLE_WAKE_INT(x)			SET_BIT(x,3)
#define ICM20649_ENABLE_PLL_READY(x)		SET_BIT(x,2)
#define ICM20649_ENABLE_DMP_INT1(x)			SET_BIT(x,1)

#define ICM20649_ENABLE_ACCEL_FIFO			0x10
#define ICM20649_ENABLE_GYRO_X_FIFO			0x08
#define ICM20649_ENABLE_GYRO_Y_FIFO			0x04
#define ICM20649_ENABLE_GYRO_Z_FIFO			0x02

#define ICM20649_ACCEL_RANGE_4G					0x00
#define ICM20649_ACCEL_RANGE_8G					0x02
#define ICM20649_ACCEL_RANGE_16G				0x04
#define ICM20649_ACCEL_RANGE_30G				0x06

#define ICM20649_ACCEL_RANGE_4G_SCALE		8192
#define ICM20649_ACCEL_RANGE_8G_SCALE		4096
#define ICM20649_ACCEL_RANGE_16G_SCALE	2048
#define ICM20649_ACCEL_RANGE_30G_SCALE	1024

#define ICM20649_ACCEL_AVERAGE_FC				0x00
#define ICM20649_ACCEL_AVERAGE_8				0x01
#define ICM20649_ACCEL_AVERAGE_16				0x02
#define ICM20649_ACCEL_AVERAGE_32				0x03

#define ICM20649_ACCEL_SELF_TEST_ENABLE  0x1C
#define ICM20649_ACCEL_SELF_TEST_DISABLE 0x00


#define ICM20649_GYRO_RANGE_500DPS			0x00
#define ICM20649_GYRO_RANGE_1000DPS			0x02
#define ICM20649_GYRO_RANGE_2000DPS			0x04
#define ICM20649_GYRO_RANGE_4000DPS			0x06

#define ICM20649_GYRO_RANGE_500DPS_SCALE		(65.5f)
#define ICM20649_GYRO_RANGE_1000DPS_SCALE		(32.8f)
#define ICM20649_GYRO_RANGE_2000DPS_SCALE		(16.4f)
#define ICM20649_GYRO_RANGE_4000DPS_SCALE		(8.2f)

#define ICM20649_GYRO_DLPFCFG_BW_230HZ	0x00
#define ICM20649_GYRO_DLPFCFG_BW_188HZ  0x08
#define ICM20649_GYRO_DLPFCFG_BW_154HZ  0x10
#define ICM20649_GYRO_DLPFCFG_BW_73HZ   0x18
#define ICM20649_GYRO_DLPFCFG_BW_36HZ   0x20
#define ICM20649_GYRO_DLPFCFG_BW_18HZ   0x28
#define ICM20649_GYRO_DLPFCFG_BW_9HZ    0x30
#define ICM20649_GYRO_DLPFCFG_BW_376HZ	0x38

#define ICM20649_GYRO_DLPF_ENABLED			0x01
#define ICM20649_GYRO_DLPF_DISABLED			0x00

#define ICM20649_GYRO_AVERAGE_1X				0x00
#define ICM20649_GYRO_AVERAGE_2X				0x01
#define ICM20649_GYRO_AVERAGE_4X				0x02
#define ICM20649_GYRO_AVERAGE_8X				0x03
#define ICM20649_GYRO_AVERAGE_16X				0x04
#define ICM20649_GYRO_AVERAGE_32X				0x05
#define ICM20649_GYRO_AVERAGE_64X				0x06
#define ICM20649_GYRO_AVERAGE_128X			0x07

#define ICM20649_GYRO_SELF_TEST_ENABLE  0x38
#define ICM20649_GYRO_SELF_TEST_DISABLE 0x00

//******************************************
//*** Registers
#define ICM20649_RA_REG_BANK_SEL       	0x7F

// BANK 0
#define ICM20649_RA_WHO_AM_I      	    0x00
#define ICM20649_RA_PWR_MGMT_1     	    0x06
#define ICM20649_RA_PWR_MGMT_2     	    0x07
#define ICM20649_RA_INT_PIN_CFG    	    0x0F
#define ICM20649_RA_INT_ENABLE      		0x10
#define ICM20649_RA_INT_ENABLE_1    	  0x11
#define ICM20649_RA_INT_ENABLE_2       	0x12
#define ICM20649_RA_INT_ENABLE_3      	0x13
#define ICM20649_RA_INT_STATUS	       	0x19
#define ICM20649_RA_INT_STATUS_1       	0x1A
#define ICM20649_RA_INT_STATUS_2       	0x1B
#define ICM20649_RA_INT_STATUS_3       	0x1C

#define ICM20649_RA_ACCEL_XOUT_H       	0x2D
#define ICM20649_RA_ACCEL_YOUT_H       	0x2F
#define ICM20649_RA_ACCEL_ZOUT_H       	0x31

#define ICM20649_RA_GYRO_XOUT_H        	0x33
#define ICM20649_RA_GYRO_YOUT_H        	0x35
#define ICM20649_RA_GYRO_ZOUT_H        	0x37

#define ICM20649_RA_TEMP_OUT_H					0x39

#define ICM20649_RA_FIFO_EN_2						0x67

// BANK 1
#define ICM20649_RA_SELF_TEST_X_GYRO		0x02
#define ICM20649_RA_OFFSET_X_ACCEL			0x14

// BANK 2
#define ICM20649_RA_GYRO_SMPLRT_DIV    	0x00
#define ICM20649_RA_GYRO_CONFIG_1      	0x01
#define ICM20649_RA_GYRO_CONFIG_2      	0x02
#define ICM20649_RA_OFFSET_X_GYRO				0x03

#define ICM20649_RA_ACCEL_SMPLRT_DIV_1 	0x10
#define ICM20649_RA_ACCEL_SMPLRT_DIV_2 	0x11
#define ICM20649_RA_ACCEL_INTEL_CTRL	 	0x12
#define ICM20649_RA_ACCEL_WAKE_THRESH	 	0x13
#define ICM20649_RA_ACCEL_CONFIG       	0x14
#define ICM20649_RA_ACCEL_CONFIG_2     	0x15

#define IMC20649_RA_TEMP_CONFIG					0x53
#define IMC20649_RA_MOD_CTRL_USR				0x54

//******************************************

enum icm20649_gyro_fsr_e
{
  ICM20649_FSR_500DPS = 0,
  ICM20649_FSR_1000DPS,
  ICM20649_FSR_2000DPS,
  ICM20649_FSR_4000DPS,
  NUM_ICM20649_GYRO_FSR
};

enum icm20649_accel_fsr_e
{
  ICM20649_FSR_4G = 0,
  ICM20649_FSR_8G,
  ICM20649_FSR_16G,
  ICM20649_FSR_30G,
  NUM_ICM20649_ACCEL_FSR
};


typedef enum
{
	ICM20649_USER_BANK_0 = 0,
	ICM20649_USER_BANK_1,
	ICM20649_USER_BANK_2,
	ICM20649_USER_BANK_3,
}icm20649_reg_bank_t;

typedef struct
{
	uint8_t gyro_range;
	uint8_t accel_range;
}icm20649_sensor_range_settings_t;

typedef struct
{
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
}icm20649_three_axis_t;

typedef struct
{
	icm20649_three_axis_t accel;
	icm20649_three_axis_t gyro;
}icm20649_all_imu_sensors_t;

typedef struct
{
	int32_t x_axis;
	int32_t y_axis;
	int32_t z_axis;
}icm20649_three_axis_self_test_t;

typedef struct
{
	icm20649_three_axis_self_test_t accel;
	icm20649_three_axis_self_test_t gyro;
}icm20649_all_imu_sensors_self_test_t;


icm20649_all_imu_sensors_t	ICM20649_get_self_test_imu( uint8_t config );

float ICM20649_convert_acceleration_to_g( int16_t axis );
float ICM20649_convert_gyro_to_angular_rate( int16_t axis );


void ICM20649_set_gyro_config_1( uint8_t config );
void ICM20649_set_gyro_config_2( uint8_t config );
void ICM20649_set_gyro_smplrt_div( uint8_t config );

void ICM20649_set_accel_config( uint8_t config );
void ICM20649_set_accel_config_2( uint8_t config );
void ICM20649_set_accel_smplrt_div( uint16_t config );

icm20649_three_axis_t	ICM20649_get_accel_offset_values( uint8_t config );
icm20649_three_axis_t	ICM20649_get_gyro_offset_values( uint8_t config );

void ICM20649_set_pwr_mgmt_1( uint8_t config );
void ICM20649_set_pwr_mgmt_2( uint8_t config );

icm20649_all_imu_sensors_t	ICM20649_read_all_imu_sensors(void);

int16_t	ICM20649_read_temp(void);
uint8_t	ICM20649_read_ID(void);

void ICM20649_initialize( external_device_config_t device_config );


#endif /* ICM20649 */
