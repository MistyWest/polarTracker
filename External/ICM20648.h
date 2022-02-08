/*
 * ICM20648.h
 *
 *  Created on: Feb 21, 2019
 *      Author: klockwood
 */

#ifndef ICM20648_H_
#define ICM20648_H_

#include "nordic_common.h"

#include "_mw_external_device.h"

/***************************************************************************/
/***************************************************************************/
/* Register common for all banks */
#define ICM20648_REG_BANK_SEL           0x7F
#define ICM20648_DEVICE_ID              0xE0

/* ICM20648 register banks */
#define ICM20648_BANK_0                  ( 0 << 7 )
#define ICM20648_BANK_1                  ( 1 << 7 )
#define ICM20648_BANK_2                  ( 2 << 7 )
#define ICM20648_BANK_3                  ( 3 << 7 )


/* ICM20648 Conversion Values */
#define ICM20648_TEMPERATURE_SCALE_VALUE  (333.87f)
#define ICM20648_TEMPERATURE_OFFSET       (21.0f)

#define ICM20648_ACCEL_2G_SCALE_VALUE     (2.0 / 32768.0f)
#define ICM20648_ACCEL_4G_SCALE_VALUE     (4.0 / 32768.0f)
#define ICM20648_ACCEL_8G_SCALE_VALUE     (8.0 / 32768.0f)
#define ICM20648_ACCEL_16G_SCALE_VALUE    (16.0 / 32768.0f)

#define ICM20648_GYRO_250DPS_SCALE_VALUE  (250.0 / 32768.0f)
#define ICM20648_GYRO_500DPS_SCALE_VALUE  (500.0 / 32768.0f)
#define ICM20648_GYRO_1000DPS_SCALE_VALUE (1000.0 / 32768.0f)
#define ICM20648_GYRO_2000DPS_SCALE_VALUE (2000.0 / 32768.0f)

/***************************************************************************/
/* Register and associated bit definitions */
/***************************************************************************/
/* Bank 0 register map */
/***************************************************************************/
#define ICM20648_REG_WHO_AM_I           0x00
#define ICM20648_REG_USER_CTRL          0x03
#define ICM20648_REG_LP_CONFIG          0x05
#define ICM20648_REG_PWR_MGMT_1         0x06
#define ICM20648_REG_PWR_MGMT_2         0x07
#define ICM20648_REG_INT_PIN_CFG        0x0F
#define ICM20648_REG_INT_ENABLE         0x10
#define ICM20648_REG_INT_ENABLE_1       0x11
#define ICM20648_REG_INT_ENABLE_2       0x12
#define ICM20648_REG_INT_ENABLE_3       0x13
#define ICM20648_REG_I2C_MST_STATUS     0x17
#define ICM20648_REG_INT_STATUS         0x19
#define ICM20648_REG_INT_STATUS_1       0x1A
#define ICM20648_REG_INT_STATUS_2       0x1B
#define ICM20648_REG_INT_STATUS_3       0x1C

#define ICM20648_REG_DELAY_TIME_H       0x28
#define ICM20648_REG_DELAY_TIME_L       0x29

#define ICM20648_REG_ACCEL_XOUT_H_SH    0x2D
#define ICM20648_REG_ACCEL_XOUT_L_SH    0x2E
#define ICM20648_REG_ACCEL_YOUT_H_SH    0x2F
#define ICM20648_REG_ACCEL_YOUT_L_SH    0x30
#define ICM20648_REG_ACCEL_ZOUT_H_SH    0x31
#define ICM20648_REG_ACCEL_ZOUT_L_SH    0x32
#define ICM20648_REG_GYRO_XOUT_H_SH     0x33
#define ICM20648_REG_GYRO_XOUT_L_SH     0x34
#define ICM20648_REG_GYRO_YOUT_H_SH     0x35
#define ICM20648_REG_GYRO_YOUT_L_SH     0x36
#define ICM20648_REG_GYRO_ZOUT_H_SH     0x37
#define ICM20648_REG_GYRO_ZOUT_L_SH     0x38
#define ICM20648_REG_TEMPERATURE_H      0x39
#define ICM20648_REG_TEMPERATURE_L      0x3A

#define ICM20648_REG_FIFO_EN_1          0x66
#define ICM20648_REG_FIFO_EN_2          0x67
#define ICM20648_REG_FIFO_RST           0x68
#define ICM20648_REG_FIFO_MODE          0x69
#define ICM20648_REG_FIFO_COUNT_H       0x70
#define ICM20648_REG_FIFO_COUNT_L       0x71
#define ICM20648_REG_FIFO_R_W           0x72
#define ICM20648_REG_DATA_RDY_STATUS    0x74
#define ICM20648_REG_FIFO_CFG           0x76

#define ICM20648_REG_MEM_START_ADDR     0x7C
#define ICM20648_REG_MEM_R_W            0x7D
#define ICM20648_REG_MEM_BANK_SELECT    0x7E

/*******************************/
/* Bank 0 - Value/Bit settings */
/*******************************/
#define ICM20648_BIT_PWR_ALL_OFF        0x7F
#define ICM20648_BIT_DMP_EN             0x80
#define ICM20648_BIT_FIFO_EN            0x40
#define ICM20648_BIT_I2C_MST_EN         0x20
#define ICM20648_BIT_I2C_IF_DIS         0x10
#define ICM20648_BIT_DMP_RST            0x08
#define ICM20648_BIT_DIAMOND_DMP_RST    0x04
#define ICM20648_BIT_I2C_MST_CYCLE      0x40
#define ICM20648_BIT_ACCEL_CYCLE        0x20
#define ICM20648_BIT_GYRO_CYCLE         0x10
#define ICM20648_BIT_H_RESET            0x80
#define ICM20648_BIT_SLEEP              0x40
#define ICM20648_BIT_LP_EN              0x20
#define ICM20648_BIT_TEMP_DIS           0x08
#define ICM20648_BIT_CLK_PLL            0x01
#define ICM20648_BIT_PWR_ACCEL_STBY     0x38
#define ICM20648_BIT_PWR_GYRO_STBY      0x07
#define ICM20648_BIT_INT_ACTL           0x80
#define ICM20648_BIT_INT_OPEN           0x40
#define ICM20648_BIT_INT_LATCH_EN       0x20
#define ICM20648_BIT_WOM_INT_EN         0x08
#define ICM20648_BIT_RAW_DATA_0_RDY_EN  0x01
#define ICM20648_BIT_FIFO_OVERFLOW_EN_0 0x01
#define ICM20648_BIT_WOM_INT            0x08
#define ICM20648_BIT_PLL_RDY            0x04
#define ICM20648_BIT_RAW_DATA_0_RDY_INT 0x01
#define ICM20648_BIT_ACCEL_FIFO_EN      0x10
#define ICM20648_BITS_GYRO_FIFO_EN      0x0E
#define ICM20648_BIT_RAW_DATA_0_RDY     0x01
#define ICM20648_BIT_MULTI_FIFO_CFG     0x01
#define ICM20648_BIT_SINGLE_FIFO_CFG    0x00


/***************************************************************************/
/* Bank 1 register map */
/***************************************************************************/
#define ICM20648_REG_SELF_TEST_X_GYRO  0x02
#define ICM20648_REG_SELF_TEST_X_ACCEL 0x0E
#define ICM20648_REG_XA_OFFSET_H       0x14
#define ICM20648_REG_XA_OFFSET_L       0x15
#define ICM20648_REG_YA_OFFSET_H       0x17
#define ICM20648_REG_YA_OFFSET_L       0x18
#define ICM20648_REG_ZA_OFFSET_H       0x1A
#define ICM20648_REG_ZA_OFFSET_L       0x1B
#define ICM20648_REG_TIMEBASE_CORR_PLL 0x28

/***************************************************************************/
/* Bank 2 register map */
/***************************************************************************/
#define ICM20648_REG_GYRO_SMPLRT_DIV    0x00
#define ICM20648_REG_GYRO_CONFIG_1      0x01
#define ICM20648_REG_GYRO_CONFIG_2      0x02

#define ICM20648_REG_XG_OFFS_USRH       0x03
#define ICM20648_REG_XG_OFFS_USRL       0x04
#define ICM20648_REG_YG_OFFS_USRH       0x05
#define ICM20648_REG_YG_OFFS_USRL       0x06
#define ICM20648_REG_ZG_OFFS_USRH       0x07
#define ICM20648_REG_ZG_OFFS_USRL       0x08

#define ICM20648_REG_ODR_ALIGN_EN       0x09
#define ICM20648_REG_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20648_REG_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20648_REG_ACCEL_INTEL_CTRL   0x12
#define ICM20648_BIT_ACCEL_INTEL_EN     0x02
#define ICM20648_BIT_ACCEL_INTEL_MODE   0x01
#define ICM20648_REG_ACCEL_WOM_THR      0x13
#define ICM20648_REG_ACCEL_CONFIG       0x14
#define ICM20648_REG_ACCEL_CONFIG_2     0x15

#define ICM20648_REG_FSYNCH_CONFIG      0x52
#define ICM20648_REG_TEMP_CONFIG        0x53
#define ICM20648_REG_MOD_CTRL_USR       0x54

/*******************************/
/* Bank 2 - Value/Bit settings */
/*******************************/
#define ICM20648_BIT_GYRO_FCHOICE       0x01
#define ICM20648_SHIFT_GYRO_FS_SEL      1
#define ICM20648_SHIFT_GYRO_DLPCFG      3
#define ICM20648_MASK_GYRO_FULLSCALE    0x06
#define ICM20648_MASK_GYRO_BW           0x39
#define ICM20648_GYRO_FULLSCALE_250DPS  ( 0x00 << ICM20648_SHIFT_GYRO_FS_SEL )
#define ICM20648_GYRO_FULLSCALE_500DPS  ( 0x01 << ICM20648_SHIFT_GYRO_FS_SEL )
#define ICM20648_GYRO_FULLSCALE_1000DPS ( 0x02 << ICM20648_SHIFT_GYRO_FS_SEL )
#define ICM20648_GYRO_FULLSCALE_2000DPS ( 0x03 << ICM20648_SHIFT_GYRO_FS_SEL )
#define ICM20648_GYRO_BW_12100HZ        ( 0x00 << ICM20648_SHIFT_GYRO_DLPCFG )
#define ICM20648_GYRO_BW_360HZ          ( ( 0x07 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )
#define ICM20648_GYRO_BW_200HZ          ( ( 0x00 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )
#define ICM20648_GYRO_BW_197HZ          ( ( 0x01 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )
#define ICM20648_GYRO_BW_120HZ          ( ( 0x02 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )
#define ICM20648_GYRO_BW_51HZ           ( ( 0x03 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )
#define ICM20648_GYRO_BW_24HZ           ( ( 0x04 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )
#define ICM20648_GYRO_BW_12HZ           ( ( 0x05 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )
#define ICM20648_GYRO_BW_6HZ            ( ( 0x06 << ICM20648_SHIFT_GYRO_DLPCFG ) | ICM20648_BIT_GYRO_FCHOICE )

#define ICM20648_BIT_GYRO_SELF_TEST_EN  0x38

#define ICM20648_BIT_ACCEL_FCHOICE      0x01
#define ICM20648_SHIFT_ACCEL_FS         1
#define ICM20648_SHIFT_ACCEL_DLPCFG     3
#define ICM20648_MASK_ACCEL_FULLSCALE   0x06
#define ICM20648_MASK_ACCEL_BW          0x39
#define ICM20648_ACCEL_FULLSCALE_2G     ( 0x00 << ICM20648_SHIFT_ACCEL_FS )
#define ICM20648_ACCEL_FULLSCALE_4G     ( 0x01 << ICM20648_SHIFT_ACCEL_FS )
#define ICM20648_ACCEL_FULLSCALE_8G     ( 0x02 << ICM20648_SHIFT_ACCEL_FS )
#define ICM20648_ACCEL_FULLSCALE_16G    ( 0x03 << ICM20648_SHIFT_ACCEL_FS )
#define ICM20648_ACCEL_BW_1210HZ        ( 0x00 << ICM20648_SHIFT_ACCEL_DLPCFG )
#define ICM20648_ACCEL_BW_470HZ         ( ( 0x07 << ICM20648_SHIFT_ACCEL_DLPCFG ) | ICM20648_BIT_ACCEL_FCHOICE )
#define ICM20648_ACCEL_BW_246HZ         ( ( 0x00 << ICM20648_SHIFT_ACCEL_DLPCFG ) | ICM20648_BIT_ACCEL_FCHOICE )
#define ICM20648_ACCEL_BW_111HZ         ( ( 0x02 << ICM20648_SHIFT_ACCEL_DLPCFG ) | ICM20648_BIT_ACCEL_FCHOICE )
#define ICM20648_ACCEL_BW_50HZ          ( ( 0x03 << ICM20648_SHIFT_ACCEL_DLPCFG ) | ICM20648_BIT_ACCEL_FCHOICE )
#define ICM20648_ACCEL_BW_24HZ          ( ( 0x04 << ICM20648_SHIFT_ACCEL_DLPCFG ) | ICM20648_BIT_ACCEL_FCHOICE )
#define ICM20648_ACCEL_BW_12HZ          ( ( 0x05 << ICM20648_SHIFT_ACCEL_DLPCFG ) | ICM20648_BIT_ACCEL_FCHOICE )
#define ICM20648_ACCEL_BW_6HZ           ( ( 0x06 << ICM20648_SHIFT_ACCEL_DLPCFG ) | ICM20648_BIT_ACCEL_FCHOICE )

#define ICM20648_BIT_ACCEL_SELF_TEST_EN 0x1C


/***************************************************************************/
/* Bank 3 register map */
/***************************************************************************/
#define ICM20648_REG_I2C_MST_ODR_CONFIG 0x00
#define ICM20648_REG_I2C_MST_CTRL       0x01
#define ICM20648_REG_I2C_MST_DELAY_CTRL 0x02
#define ICM20648_REG_I2C_SLV0_ADDR      0x03
#define ICM20648_REG_I2C_SLV0_REG       0x04
#define ICM20648_REG_I2C_SLV0_CTRL      0x05
#define ICM20648_REG_I2C_SLV0_DO        0x06
#define ICM20648_REG_I2C_SLV1_ADDR      0x07
#define ICM20648_REG_I2C_SLV1_REG       0x08
#define ICM20648_REG_I2C_SLV1_CTRL      0x09
#define ICM20648_REG_I2C_SLV1_DO        0x0A
#define ICM20648_REG_I2C_SLV2_ADDR      0x0B
#define ICM20648_REG_I2C_SLV2_REG       0x0C
#define ICM20648_REG_I2C_SLV2_CTRL      0x0D
#define ICM20648_REG_I2C_SLV2_DO        0x0E
#define ICM20648_REG_I2C_SLV3_ADDR      0x0F
#define ICM20648_REG_I2C_SLV3_REG       0x10
#define ICM20648_REG_I2C_SLV3_CTRL      0x11
#define ICM20648_REG_I2C_SLV3_DO        0x12
#define ICM20648_REG_I2C_SLV4_ADDR      0x13
#define ICM20648_REG_I2C_SLV4_REG       0x14
#define ICM20648_REG_I2C_SLV4_CTRL      0x15
#define ICM20648_REG_I2C_SLV4_DO        0x16
#define ICM20648_REG_I2C_SLV4_DI        0x17

/*******************************/
/* Bank 3 - Value/Bit settings */
/*******************************/
#define ICM20648_BIT_I2C_MST_P_NSR      0x10

#define ICM20648_BIT_SLV0_DLY_EN        0x01
#define ICM20648_BIT_SLV1_DLY_EN        0x02
#define ICM20648_BIT_SLV2_DLY_EN        0x04
#define ICM20648_BIT_SLV3_DLY_EN        0x08

#define ICM20648_BIT_I2C_SLV_EN         0x80
#define ICM20648_BIT_I2C_BYTE_SW        0x40
#define ICM20648_BIT_I2C_REG_DIS        0x20
#define ICM20648_BIT_I2C_GRP            0x10
#define ICM20648_BIT_I2C_READ           0x80

/***************************************************************************/
// Table for list of results for factory self-test value equation
// st_otp = 2620/2^FS * 1.01^(st_value - 1)
// for gyro and accel FS = 0 so 2620 * 1.01^(st_value - 1)
// st_value = 1 => 2620
// st_value = 2 => 2620 * 1.01 = 2646
// etc../
static const uint16_t inv_Self_Test_Equation[256] = {
  2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
  2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
  3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
  3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
  3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
  3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
  4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
  4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
  4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
  5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
  5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
  6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
  6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
  7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
  7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
  8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
  9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
  10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
  10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
  11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
  12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
  13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
  15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
  16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
  17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
  19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
  20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
  22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
  24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
  26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
  28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
  30903, 31212, 31524, 31839, 32157, 32479, 32804
};
/***************************************************************************/

enum icm20648_gyroscope_fsr_e
{
    ICM20648_FSR_500DPS = 0,
    ICM20648_FSR_1000DPS,
    ICM20648_FSR_2000DPS,
    ICM20648_FSR_4000DPS,
    NUM_ICM20648_GYRO_FSR
};

enum icm20648_accelerometer_fsr_e
{
    ICM20648_FSR_4G = 0,
    ICM20648_FSR_8G,
    ICM20648_FSR_16G,
    ICM20648_FSR_30G,
    NUM_ICM20648_ACCEL_FSR
};


typedef enum
{
	ICM20648_USER_BANK_0=0,
	ICM20648_USER_BANK_1,
	ICM20648_USER_BANK_2,
	ICM20648_USER_BANK_3,
}icm20648_reg_bank_t;

typedef struct
{
	uint8_t gyro_range;
	uint8_t accel_range;
}icm20648_sensor_range_settings_t;

typedef struct
{
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
}icm20648_three_axis_t;

typedef struct
{
	icm20648_three_axis_t accel;
	icm20648_three_axis_t gyro;
}icm20648_all_imu_sensors_t;

typedef struct
{
	int32_t x_axis;
	int32_t y_axis;
	int32_t z_axis;
}icm20648_three_axis_self_test_t;

typedef struct
{
	icm20648_three_axis_self_test_t accel;
	icm20648_three_axis_self_test_t gyro;
}icm20648_all_imu_sensors_self_test_t;


//**********************


/**
 * @brief - Write Sample Rate to ACCEL_SMPLRT_DIV_1 & 2.  It is
 *          automatically calculated from the pass sample_rate
 */
void ICM20648_set_accelerometer_sample_rate( float sample_rate );



/**
 * @brief - Read Sample Rate from ACCEL_SMPLRT_DIV_1 & 2
 */
uint8_t ICM20648_get_gyroscope_sample_rate();


/**
 * @brief - Write Sample Rate to GYRO_SMPLRT_DIV.  It is
 *          automatically calculated from the pass sample_rate
 */
void ICM20648_set_gyroscope_sample_rate( float sample_rate );


/**
 * @brief - Read Sample Rate from GYRO_SMPLRT_DIV
 */
uint16_t ICM20648_get_accelerometer_sample_rate();



/**
 * @brief - Write accelerometer Scale to ACCEL_CONFIG
 */
void ICM20648_set_accelerometer_scale( uint8_t scale );

/**
 * @brief - Read accelerometer Scale to ACCEL_CONFIG
 */
uint8_t ICM20648_get_accelerometer_scale();


/**
 * @brief - Write gyroscope Scale to GYRO_CONFIG_1
 */
void ICM20648_set_gyroscope_scale( uint8_t scale );

/**
 * @brief - Read gyroscope Scale from GYRO_CONFIG_1
 */
uint8_t ICM20648_get_gyroscope_scale();


/**
 * @brief - Write LPF Value to ACCEL_CONFIG
 */
void ICM20648_set_accelerometer_lpf( uint8_t bandwidth );

/**
 * @brief - Read LPF Value from ACCEL_CONFIG
 */
uint8_t ICM20648_get_accelerometer_lpf();


/**
 * @brief - Write LPF Value to GYRO_CONFIG_1
 */
void ICM20648_set_gyroscope_lpf( uint8_t bandwidth );

/**
 * @brief - Read LPF Value from GYRO_CONFIG_1
 */
uint8_t ICM20648_get_gyroscope_lpf();


/**
 * @brief - Converts gyroscope axis value in radians/sec float value
 */
void ICM20648_convert_accelerometer_to_g( int16_t axis, float * float_value );

/**
 * @brief - Converts gyroscope axis value in radians/sec float value
 */
void ICM20648_convert_gyroscope_to_angular_rate( int16_t axis, float * float_value );



/**
 * @brief - Reads current measurement in all sensors
 */
void ICM20648_read_all_imu_sensors( icm20648_all_imu_sensors_t * imu );


/**
 * @brief - Reset the FIFO
 */
void ICM20648_reset_FIFO();


/**
 * @brief - Enable/Disable FIFO bit in USER_CTRL
 */
void ICM20648_enable_FIFO( bool enable );


/**
 * @brief - Set FIFO mode
 */
void ICM20648_FIFO_mode( bool snapshot);


/**
 * @brief - Enable/Disable FIFO Overflow Interrupt INT2
 */
void ICM20648_enable_FIFO_overflow_interrupt( bool enable);


/**
 * @brief - Enable/Disable FIFO Watermark Interrupt INT2
 */
void ICM20648_enable_FIFO_watermark_interrupt( bool enable);


/**
 * @brief - Check FIFO Watermark Interrupt INT3 status
 */
uint8_t ICM20648_check_FIFO_watermark_interrupt();

/**
 * @brief - Check FIFO Count
 */
uint16_t ICM20648_check_FIFO_count();


/**
 * @brief - Write to FIFO_EN_2 register which controls assignment of
 *          Sensors to the FIFO
 */
void ICM20648_set_FIFO_sensor_enable( uint8_t config );


/*
 * @brief - Read FIFO Contents
 */
void ICM20648_read_FIFO( icm20648_all_imu_sensors_t * imu );


/*
 * @brief - Read FIFO Contents in Burst.  Max Read size is 256bytes
 */
uint32_t ICM20648_read_FIFO_burst( uint8_t * data, uint16_t length );


/**
 * @brief - Enable/Disable DMP bit in USER_CTRL
 */
void ICM20648_enable_DMP( bool enable);


/**
 * @brief - Set DMP Watermark level
 */
void ICM20648_config_DMP_watermark( uint16_t watermark );

/**
 * @brief - Set Interrupt Configuration Register
 */
void  ICM20648_set_int_pin_cgf( uint8_t config );


/**
 * @brief - Set Interrupt Enable Register
 */
void  ICM20648_set_int_enable( uint8_t config );


/**
 * @brief - Set Interrupt Enable 1 Register
 */
void  ICM20648_set_int_enable_1( uint8_t config );


/**
 * @brief - Set Interrupt Enable 2 Register
 */
void  ICM20648_set_int_enable_2( uint8_t config );


/**
 * @brief - Read Interrupt Status Register
 */
uint8_t ICM20648_read_int_status(void);

/**
 * @brief - Read Interrupt 1 Status
 */
uint8_t ICM20648_read_interrupt_status_1(void);

/**
 * @brief - Read Interrupt 2 Status
 */
uint8_t ICM20648_read_interrupt_status_2(void);

/**
 * @brief - Read Interrupt 3 Status
 */
uint8_t ICM20648_read_interrupt_status_3(void);


/**
 * @brief - Enable/Disable Sleep bit in PWR_MGMT_1
 */
void ICM20648_enable_sleep( bool enable);

/**
 * @brief - Writes contents of PWR_MGMT_1
 */
void ICM20648_set_pwr_mgmt_1( uint8_t config );


/**
 * @brief - Writes contents of PWR_MGMT_2
 */
void ICM20648_set_pwr_mgmt_2( uint8_t config );


/**
 * @brief - Writes contents of LP_CONFIG
 */
void ICM20648_set_lp_config( uint8_t config );



/**
 * @brief - Writes contents of ACCEL_CONFIG
 */
void ICM20648_set_accelerometer_config( uint8_t config );


/**
 * @brief - Reads contents of ACCEL_CONFIG
 */
uint8_t ICM20648_get_accelerometer_config();


/**
 * @brief - Writes contents of GYRO_CONFIG_1
 */
void ICM20648_set_gyroscope_config_1( uint8_t config );

/**
 * @brief - Reads contents of GYRO_CONFIG_1
 */
uint8_t ICM20648_get_gyroscope_config_1();


/**
 * @brief - Writes contents of GYRO_CONFIG_2
 */
void ICM20648_set_gyroscope_config_2( uint8_t config );


/**
 * @brief - Writes contents of ACCEL_CONFIG_2
 */
void ICM20648_set_accelerometer_config_2( uint8_t config );


/**
 * @brief - Reads current temperature measurement
 */
void ICM20648_read_temperature( float *temperature );


/**
 * @brief - Enable/Disable Accelerometer, Gyroscope, Temperature Sensors
 */
void ICM20648_sensor_enable( bool accel, bool gyro, bool temp );



/**
 * @brief - Retreives the Self Test Stored Values
 */
void ICM20648_get_factory_self_test( icm20648_all_imu_sensors_t * self_test );

/**
 * @brief - Runs Device SELF-TEST to ensure proper device operation
 */
bool ICM20648_run_self_test();

/**
 * @brief - Performs the ICM20648 Calibration Routine.
 *
 * IMPORTANT: Called Application should store the accel_bias_scaled for application level
 *            offset subtraction. Not done well ON-DEVICE.  Gyro bias is handled internally
 */
void ICM20648_calibrate( float *accel_bias_scaled, float *gyro_bias_scaled );

/**
 * @brief - Resets device
 */
void ICM20648_reset( void );

/**
 * @brief - Reads the device ID, used to verify communication
 */
uint8_t ICM20648_read_device_ID(void);


/**
 * @brief - Select Register Bank to Access
 */
void ICM20648_select_bank( icm20648_reg_bank_t bank_no );

/**
 * @brief - Wrapper Function for Reading from a Register
 */
//@read
void ICM20648_read( uint8_t address, uint8_t * rx_data, uint16_t length );


/**
 * @brief - Wrapper Function for Writing to a Register
 */
//@write
void ICM20648_write( uint8_t address, uint8_t value );


/**
 * @brief - Wrapper Function for Writing to a Register, two bytes
 */
//@write
void ICM20648_write_two_bytes(  uint8_t address, uint16_t value );


/**
 * @brief - Wrapper Function for Writing to a Register, multiple bytes
 */
//@write
void ICM20648_write_bytes( uint8_t address, uint8_t * data, uint16_t length );


/**
 * @brief - TMP57C Driver Initialization
 *
 * @param - Driver Initialzation parameter contains all interface configurations
 */
void ICM20648_initialize( external_device_config_t device_config );


#endif /* ICM20648 */
