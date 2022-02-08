/*
 * LSM6DS3.h
 *
 *  Created on: Mar 21, 2019
 *      Author: klockwood
 */

#ifndef EXTERNAL_LSM6DS3_H_
#define EXTERNAL_LSM6DS3_H_

#include "_mw_external_device.h"

/***************************************************************************/
/***************************************************************************/
/* LSM6DS3 Conversion Values */
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */

#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_245DPS   08.750  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

/************** Who am I  *******************/
#define LSM6DS3_WHO_AM_I         0x69


/************** Common Values  *******************/
#define LSM6DS3_FIFO_STEP_MODE     (1<<6)
#define LSM6DS3_FIFO_DATA_MODE     (0)

#define LSM6DS3_FIFO_DEC_NONE      1
#define LSM6DS3_FIFO_DEC_2         2
#define LSM6DS3_FIFO_DEC_3         3
#define LSM6DS3_FIFO_DEC_4         4
#define LSM6DS3_FIFO_DEC_8         5
#define LSM6DS3_FIFO_DEC_16        6
#define LSM6DS3_FIFO_DEC_32        7

#define LSM6DS3_ODR_OFF            0
#define LSM6DS3_ODR_12_5HZ         1
#define LSM6DS3_ODR_26HZ           2
#define LSM6DS3_ODR_52HZ           3
#define LSM6DS3_ODR_104HZ          4
#define LSM6DS3_ODR_208HZ          5
#define LSM6DS3_ODR_416HZ          6
#define LSM6DS3_ODR_833HZ          7
#define LSM6DS3_ODR_1660HZ         8
#define LSM6DS3_ODR_3330HZ         9
#define LSM6DS3_ODR_6660HZ         10

#define LSM6DS3_ACCEL_SCALE_MASK   0x0C

#define LSM6DS3_GYRO_SCALE_MASK    0x0C

#define LSM6DS3_ACCEL_SCALE_2G     0
#define LSM6DS3_ACCEL_SCALE_4G     2
#define LSM6DS3_ACCEL_SCALE_8G     3
#define LSM6DS3_ACCEL_SCALE_16G    1

#define LSM6DS3_ACCEL_2G_SCALE_VALUE     (2.0 / 32768.0f)
#define LSM6DS3_ACCEL_4G_SCALE_VALUE     (4.0 / 32768.0f)
#define LSM6DS3_ACCEL_8G_SCALE_VALUE     (8.0 / 32768.0f)
#define LSM6DS3_ACCEL_16G_SCALE_VALUE    (16.0 / 32768.0f)

#define LSM6DS3_GYRO_SCALE_250DPS  0
#define LSM6DS3_GYRO_SCALE_500DPS  1
#define LSM6DS3_GYRO_SCALE_1000DPS 2
#define LSM6DS3_GYRO_SCALE_2000DPS 3

#define LSM6DS3_GYRO_250DPS_SCALE_VALUE  (8.75 / 1000.0f)
#define LSM6DS3_GYRO_500DPS_SCALE_VALUE  (17.5 / 1000.0f)
#define LSM6DS3_GYRO_1000DPS_SCALE_VALUE (35.0 / 1000.0f)
#define LSM6DS3_GYRO_2000DPS_SCALE_VALUE (70.0 / 1000.0f)

#define LSM6DS2_ACCEL_ODR(x)       (x << 4)
#define LSM6DS2_ACCEL_SCALE(x)     (x << 2)

#define LSM6DS2_GYRO_ODR(x)        (x << 4)
#define LSM6DS2_GYRO_SCALE(x)      (x << 2)

#define LSM6DS3_ENABLED_ACCEL_BW   (1 << 7)

#define LSM6DS3_BW_400HZ           0
#define LSM6DS3_BW_200HZ           1
#define LSM6DS3_BW_100HZ           2
#define LSM6DS3_BW_50HZ            3


#define LSM6DS3_FIFO_ODR(x)        (x << 3)
#define LSM6DS3_ODR_OFF            0
#define LSM6DS3_ODR_104HZ          4
#define LSM6DS3_ODR_208HZ          5
#define LSM6DS3_ODR_416HZ          6
#define LSM6DS3_ODR_833HZ          7
#define LSM6DS3_ODR_1660HZ         8
#define LSM6DS3_ODR_3330HZ         9
#define LSM6DS3_ODR_6660HZ         10


#define LSM6DS3_FIFO_MODE_DISABLED  0
#define LSM6DS3_FIFO_MODE_STOP_FULL 1
#define LSM6DS3_FIFO_MODE_CONT_TRIG 3
#define LSM6DS3_FIFO_MODE_BYPASS    4
#define LSM6DS3_FIFO_MODE_CONT_FULL 6


#define LSM6DS3_INT_FIFO_THRES     (1<<3)
#define LSM6DS3_INT_FIFO_OVERFLOW  (1<<4)
#define LSM6DS3_INT_FIFO_FULL      (1<<5)

#define LSM6DS3_CTRL3_DEFAULT      (1<<6) | (1<<2) | (1<<1)  /*Enable BDU, MSB Addressing and Auto address incrementing */


#define LSM6DS3_ACCEL_SELF_TEST_MIN (90.0/1000.0f)
#define LSM6DS3_ACCEL_SELF_TEST_MAX (1700.0/1000.0f)
#define LSM6DS3_GYRO_SELF_TEST_MIN  (150.0f)
#define LSM6DS3_GYRO_SELF_TEST_MAX  (700.0f)


/***************************************************************************/
/***************************************************************************/

/************** Device Register  *******************/
#define LSM6DS3_FUNC_CFG_ACCESS    0X01
#define LSM6DS3_SENSOR_SYNC_TIME   0X04

#define LSM6DS3_FIFO_CTRL1         0X06
#define LSM6DS3_FIFO_CTRL2         0X07
#define LSM6DS3_FIFO_CTRL3         0X08
#define LSM6DS3_FIFO_CTRL4         0X09
#define LSM6DS3_FIFO_CTRL5         0X0A

#define LSM6DS3_ORIENT_CFG_G       0X0B
#define LSM6DS3_INT1_CTRL          0X0D
#define LSM6DS3_INT2_CTRL          0X0E
#define LSM6DS3_WHO_AM_I_REG       0X0F
#define LSM6DS3_CTRL1_XL           0X10
#define LSM6DS3_CTRL2_G            0X11
#define LSM6DS3_CTRL3_C            0X12
#define LSM6DS3_CTRL4_C            0X13
#define LSM6DS3_CTRL5_C            0X14
#define LSM6DS3_CTRL6_C            0X15
#define LSM6DS3_CTRL7_G            0X16
#define LSM6DS3_CTRL8_XL           0X17
#define LSM6DS3_CTRL9_XL           0X18
#define LSM6DS3_CTRL10_C           0X19
#define LSM6DS3_MASTER_CONFIG      0X1A
#define LSM6DS3_WAKE_UP_SRC        0X1B
#define LSM6DS3_TAP_SRC            0X1C
#define LSM6DS3_D6D_SRC            0X1D
#define LSM6DS3_STATUS_REG         0X1E
#define LSM6DS3_OUT_TEMP_L         0X20
#define LSM6DS3_OUT_TEMP_H         0X21
#define LSM6DS3_OUTX_L_G           0X22
#define LSM6DS3_OUTX_H_G           0X23
#define LSM6DS3_OUTY_L_G           0X24
#define LSM6DS3_OUTY_H_G           0X25
#define LSM6DS3_OUTZ_L_G           0X26
#define LSM6DS3_OUTZ_H_G           0X27
#define LSM6DS3_OUTX_L_XL          0X28
#define LSM6DS3_OUTX_H_XL          0X29
#define LSM6DS3_OUTY_L_XL          0X2A
#define LSM6DS3_OUTY_H_XL          0X2B
#define LSM6DS3_OUTZ_L_XL          0X2C
#define LSM6DS3_OUTZ_H_XL          0X2D
#define LSM6DS3_SENSORHUB1_REG     0X2E
#define LSM6DS3_SENSORHUB2_REG     0X2F
#define LSM6DS3_SENSORHUB3_REG     0X30
#define LSM6DS3_SENSORHUB4_REG     0X31
#define LSM6DS3_SENSORHUB5_REG     0X32
#define LSM6DS3_SENSORHUB6_REG     0X33
#define LSM6DS3_SENSORHUB7_REG     0X34
#define LSM6DS3_SENSORHUB8_REG     0X35
#define LSM6DS3_SENSORHUB9_REG     0X36
#define LSM6DS3_SENSORHUB10_REG    0X37
#define LSM6DS3_SENSORHUB11_REG    0X38
#define LSM6DS3_SENSORHUB12_REG    0X39
#define LSM6DS3_FIFO_STATUS1       0X3A
#define LSM6DS3_FIFO_STATUS2       0X3B
#define LSM6DS3_FIFO_STATUS3       0X3C
#define LSM6DS3_FIFO_STATUS4       0X3D
#define LSM6DS3_FIFO_DATA_OUT_L    0X3E
#define LSM6DS3_FIFO_DATA_OUT_H    0X3F
#define LSM6DS3_TIMESTAMP0_REG     0X40
#define LSM6DS3_TIMESTAMP1_REG     0X41
#define LSM6DS3_TIMESTAMP2_REG     0X42

#define LSM6DS3_TIMESTAMP_L        0X49
#define LSM6DS3_TIMESTAMP_H        0X4A

#define LSM6DS3_STEP_COUNTER_L     0X4B
#define LSM6DS3_STEP_COUNTER_H     0X4C

#define LSM6DS3_SENSORHUB13_REG    0X4D
#define LSM6DS3_SENSORHUB14_REG    0X4E
#define LSM6DS3_SENSORHUB15_REG    0X4F
#define LSM6DS3_SENSORHUB16_REG    0X50
#define LSM6DS3_SENSORHUB17_REG    0X51
#define LSM6DS3_SENSORHUB18_REG    0X52

#define LSM6DS3_FUNC_SRC           0X53
#define LSM6DS3_TAP_CFG1           0X58
#define LSM6DS3_TAP_THS_6D         0X59
#define LSM6DS3_INT_DUR2           0X5A
#define LSM6DS3_WAKE_UP_THS        0X5B
#define LSM6DS3_WAKE_UP_DUR        0X5C
#define LSM6DS3_FREE_FALL          0X5D
#define LSM6DS3_MD1_CFG            0X5E
#define LSM6DS3_MD2_CFG            0X5F

/***************************************************************************/
/***************************************************************************/
/************** Embedded functions register mapping  *******************/
#define LSM6DS3_SLV0_ADD                     0x02
#define LSM6DS3_SLV0_SUBADD                  0x03
#define LSM6DS3_SLAVE0_CONFIG                0x04
#define LSM6DS3_SLV1_ADD                     0x05
#define LSM6DS3_SLV1_SUBADD                  0x06
#define LSM6DS3_SLAVE1_CONFIG                0x07
#define LSM6DS3_SLV2_ADD                     0x08
#define LSM6DS3_SLV2_SUBADD                  0x09
#define LSM6DS3_SLAVE2_CONFIG                0x0A
#define LSM6DS3_SLV3_ADD                     0x0B
#define LSM6DS3_SLV3_SUBADD                  0x0C
#define LSM6DS3_SLAVE3_CONFIG                0x0D
#define LSM6DS3_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DS3_CONFIG_PEDO_THS_MIN          0x0F

#define LSM6DS3_SM_STEP_THS                  0x13
#define LSM6DS3_PEDO_DEB_REG                 0x14
#define LSM6DS3_STEP_COUNT_DELTA             0x15

#define LSM6DS3_MAG_SI_XX                    0x24
#define LSM6DS3_MAG_SI_XY                    0x25
#define LSM6DS3_MAG_SI_XZ                    0x26
#define LSM6DS3_MAG_SI_YX                    0x27
#define LSM6DS3_MAG_SI_YY                    0x28
#define LSM6DS3_MAG_SI_YZ                    0x29
#define LSM6DS3_MAG_SI_ZX                    0x2A
#define LSM6DS3_MAG_SI_ZY                    0x2B
#define LSM6DS3_MAG_SI_ZZ                    0x2C
#define LSM6DS3_MAG_OFFX_L                   0x2D
#define LSM6DS3_MAG_OFFX_H                   0x2E
#define LSM6DS3_MAG_OFFY_L                   0x2F
#define LSM6DS3_MAG_OFFY_H                   0x30
#define LSM6DS3_MAG_OFFZ_L                   0x31
#define LSM6DS3_MAG_OFFZ_H                   0x32

/***************************************************************************/
/***************************************************************************/

typedef struct
{
  int16_t x_axis;
  int16_t y_axis;
  int16_t z_axis;
}lsm6ds3_three_axis_t;

typedef struct
{
  lsm6ds3_three_axis_t gyro;
  lsm6ds3_three_axis_t accel;
}lsm6ds3_all_imu_sensors_t;

/**
 * @brief - Convert Raw Acceleormeter Data to G's
 */
void LSM6DS3_convert_accelerometer_to_g( int16_t axis, float * float_value );


/**
 * @brief - Convert Raw Gyroscope Data to dps's
 */
void LSM6DS3_convert_gyroscope_to_angular_rate( int16_t axis, float * float_value );


/**
 * @brief - Check FIFO Count
 *
 * @return - Value is multiplied by 2 because device returns value in
 *           as represented by 1 LSB = 2bytes
 */
uint16_t LSM6DS3_check_FIFO_count();


/**
 * @brief - Read FIFO
 *
 */
void LSM6DS3_read_fifo( uint8_t * data, uint16_t length );


/**
 * @brief - Read Gyroscope
 *
 */
//@read
void LSM6DS3_read_gyroscope( lsm6ds3_three_axis_t * gyro );


/**
 * @brief - Read Accelerometer
 *
 */
//@read
void LSM6DS3_read_accelerometer( lsm6ds3_three_axis_t * accel );



/**
 * @brief - Read Accelerometer and Gyroscope
 *
 */
void LSM6DS3_read_imu( lsm6ds3_all_imu_sensors_t * imu );


/**
 * @brief - Enable/Disable Gyroscope
 *
 */
void LSM6DS3_enable_gyro( bool gyro );


/**
 * @brief - Enable/Disable Accelerometer
 *
 */
void LSM6DS3_enable_accel( bool accel );


/**
 * @brief - Set Gyro High Performance Mode
 *
 */
void LSM6DS3_set_gyro_high_performance_mode( bool enabled );


/**
 * @brief - Set Accel High Performance Mode
 *
 */
void LSM6DS3_set_accel_high_performance_mode( bool enabled );


/**
 * @brief - Set Self Test
 *
 */
void LSM6DS3_set_self_test( uint8_t value);


/**
 * @brief - Get Ctrl 4 Register
 *
 */
uint8_t LSM6DS3_get_reg_ctrl4(void);

/**
 * @brief - Set Ctrl 4 Register
 *
 */
void LSM6DS3_set_reg_ctrl4( uint8_t ctrl );


/**
 * @brief - Set Ctrl 3 Register
 *
 */
void LSM6DS3_set_reg_ctrl3( uint8_t ctrl );


/**
 * @brief - Get Gyro Ctrl2 Register
 *
 */
uint8_t LSM6DS3_get_gyro_ctrl2(void);

/**
 * @brief - Set Gyro Ctrl2 Register
 *
 */
void LSM6DS3_set_gyro_ctrl2( uint8_t ctrl );

/**
 * @brief - Get INT2 Ctrl Register
 *
 */
uint8_t LSM6DS3_get_accel_ctrl1(void);

/**
 * @brief - Set INT2 Ctrl Register
 *
 */
void LSM6DS3_set_accel_ctrl1( uint8_t ctrl );


/**
 * @brief - Get INT2 Ctrl Register
 *
 */
uint8_t LSM6DS3_get_int2_ctrl(void);

/**
 * @brief - Set INT2 Ctrl Register
 *
 */
void LSM6DS3_set_int2_ctrl( uint8_t ctrl );

/**
 * @brief - Get INT1 Ctrl Register
 *
 */
uint8_t LSM6DS3_get_int1_ctrl(void);

/**
 * @brief - Set INT1 Ctrl Register
 *
 */
void LSM6DS3_set_int1_ctrl( uint8_t ctrl );



/**
 * @brief - Read FIFO
 *
 */
//@read
void LSM6DS3_read_fifo( uint8_t * data, uint16_t length );


/**
 * @brief - Flush FIFO
 */
void LSM6DS3_flush_fifo();


/**
 * @brief - Get FIFO Mode
 *
 */
uint8_t LSM6DS3_get_fifo_mode(void);

/**
 * @brief - Set FIFO Mode
 *
 */
void LSM6DS3_set_fifo_mode( uint8_t mode );



/**
 * @brief - Get FIFO Status
 *
 */
uint8_t LSM6DS3_get_fifo_status( void );


/**
 * @brief - Get FIFO ODR
 *
 */
uint8_t LSM6DS3_get_fifo_odr( void );

/**
 * @brief - Set FIFO ODR
 *
 */
void LSM6DS3_set_fifo_odr( uint8_t fifo_odr );


/**
 * @brief - Get FIFO Decimation
 *
 */
uint8_t LSM6DS3_get_fifo_decimation(void);

/**
 * @brief - Set FIFO Decimation
 *
 * @param - Decimation Factor
 */
void LSM6DS3_set_fifo_decimation( uint8_t decimation );


/**
 * @brief - Set FIFO Write Mode
 *
 * @param - STEP_MODE: data written on step detection
 *          DATA_RDY_MODE: data written as its measureed
 */
void LSM6DS3_set_fifo_write_mode( uint8_t mode );



/**
 * @brief - Get FIFO Threshold
 *
 * @return - Value is multiplied by 2 because device returns value in
 *           as represented by 1 LSB = 2bytes
 */
uint16_t LSM6DS3_get_fifo_threshold(void);


/**
 * @brief - Set FIFO Threshold
 *
 * @param - Absolute threshold, divided by 2 as representation
 *          on the device is 1 LSB = 2 bytes
 */
void LSM6DS3_set_fifo_threshold( uint16_t level );


/**
 * @brief - Reaset device
 */
void LSM6DS3_reset_device();


/**
 * @brief - Reads the device ID, used to verify communication
 */
uint8_t LSM6DS3_read_device_ID(void);


/**
 * @brief - Wrapper Function for Reading from a Register
 */
//@read
void LSM6DS3_read( uint8_t address, uint8_t * rx_data, uint16_t length );


/**
 * @brief - Wrapper Function for Writing to a Register
 */
//@write
void LSM6DS3_write( uint8_t address, uint8_t value );


/**
 * @brief - Wrapper Function for Writing to a Register, two bytes
 */
//@write
void LSM6DS3_write_two_bytes(  uint8_t address, uint16_t value );


/**
 * @brief - Wrapper Function for Writing to a Register, multiple bytes
 *
 * @param - Address must be passed in the first of the data[]
 */
//@write
void LSM6DS3_write_bytes(uint8_t * data, uint16_t length );


/**
 * @brief - LSM6DS3 Driver Place in Low Power Mode
 */
void LSM6DS3_suspend();


/**
 * @brief - LSM6DS3 Driver Initialization
 *
 * @param - Driver Initialzation parameter contains all interface configurations
 */
void LSM6DS3_initialize( external_device_config_t device_config );

#endif /* EXTERNAL_LSM6DS3_H_ */
