/*
 * ICM20649.c
 *
 *  Created on: Jan 30, 2019
 *      Author: klockwood
 */

#include "ICM20649.h"
#include "../CLI_Logging/mw_logging.h"
#include "_mw_external_device.h"

#define ICM20649_TEST_UNITS_ENABLED				1

static bool	m_icm20649_initialized = false;

static external_driver_spi_config_t m_ICM20649_SPI_ID;

static icm20649_sensor_range_settings_t	m_icm20649_range_settings;

static void ICM20649_select_bank( icm20649_reg_bank_t bank_no );

static void ICM20649_write( uint8_t address, uint8_t value );
static void ICM20649_read( uint8_t address, uint8_t * rx_data, uint8_t length );

/**@brief Unit Test for each register write
 *
 */
static void IMU_READ_TEST( icm20649_all_imu_sensors_t readings )
{
#if !ICM20649_TEST_UNITS_ENABLED
	return;
#endif

	if( ( readings.accel.x_axis == 0 ) &&
			( readings.accel.y_axis == 0 ) &&
			( readings.accel.z_axis == 0 ) &&
			( readings.gyro.x_axis == 0 ) &&
			( readings.gyro.y_axis == 0 ) &&
			( readings.gyro.z_axis == 0 ) )
	{
		MW_LOG_INFO("Error: Blank Sensor Reading");
	}
}


/**@brief Unit Test for each register write
 *
 */
static void UNIT_TEST( uint8_t bank_no, uint8_t register_to_check, uint8_t verify_value )
{
#if !ICM20649_TEST_UNITS_ENABLED
	return;
#endif

	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(bank_no);

	tx_data[0] = register_to_check;
	if(tx_data[0] == 0x06 )
	{
		MW_LOG_INFO("ICM20649 Error: Super Error");
	}
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	if( verify_value != rx_data[1] )
	{
		MW_LOG_INFO("Register: 0x%X shows unexpected value of 0x%X", register_to_check, verify_value)
	}
}


float ICM20649_convert_acceleration_to_g( int16_t axis )
{
	uint16_t accel_scale_factor = 0;

	switch(m_icm20649_range_settings.accel_range)
	{
	case ICM20649_ACCEL_RANGE_4G:
		accel_scale_factor = ICM20649_ACCEL_RANGE_4G_SCALE;
		break;
	case ICM20649_ACCEL_RANGE_8G:
		accel_scale_factor = ICM20649_ACCEL_RANGE_8G_SCALE;
		break;
	case ICM20649_ACCEL_RANGE_16G:
		accel_scale_factor = ICM20649_ACCEL_RANGE_16G_SCALE;
		break;
	case ICM20649_ACCEL_RANGE_30G:
		accel_scale_factor = ICM20649_ACCEL_RANGE_30G_SCALE;
		break;
	}
	return (float) axis / accel_scale_factor;
}


float ICM20649_convert_gyro_to_angular_rate( int16_t axis )
{
	float gyro_scale_factor = 0;

	switch(m_icm20649_range_settings.accel_range)
	{
	case ICM20649_GYRO_RANGE_500DPS:
		gyro_scale_factor = ICM20649_GYRO_RANGE_500DPS_SCALE;
		break;
	case ICM20649_GYRO_RANGE_1000DPS:
		gyro_scale_factor = ICM20649_GYRO_RANGE_1000DPS_SCALE;
		break;
	case ICM20649_GYRO_RANGE_2000DPS:
		gyro_scale_factor = ICM20649_GYRO_RANGE_2000DPS_SCALE;
		break;
	case ICM20649_GYRO_RANGE_4000DPS:
		gyro_scale_factor = ICM20649_GYRO_RANGE_4000DPS_SCALE;
		break;
	}
	return (float)  axis / gyro_scale_factor;
}


static void update_gyro_range_settings( uint8_t config )
{
	config = config & 0x06; // Isolate Range bits

	m_icm20649_range_settings.gyro_range = config;
}


static void update_accel_range_settings( uint8_t config )
{
	config = config & 0x06; // Isolate Range bits

	m_icm20649_range_settings.accel_range = config;
}


/**@brief Select Register Bank
 *
 */
static void ICM20649_select_bank( icm20649_reg_bank_t bank_no )
{
	uint8_t update = 0;

	switch(bank_no)
	{
	// Clear bits 4:5
	case ICM20649_USER_BANK_0:
		CLR_BIT( update, 4);
		CLR_BIT( update, 5);
		break;

	// Set bits 4:5 1:0
	case ICM20649_USER_BANK_1:
		SET_BIT( update, 4);
		CLR_BIT( update, 5);
		break;

	// Set bits 4:5 0:1
	case ICM20649_USER_BANK_2:
		CLR_BIT( update, 4);
		SET_BIT( update, 5);
		break;

	// Set bits 4:5 1:1
	case ICM20649_USER_BANK_3:
		SET_BIT( update, 4);
		SET_BIT( update, 5);
		break;

	default:
		CLR_BIT( update, 4);
		CLR_BIT( update, 5);
		break;
	}

	ICM20649_write( ICM20649_RA_REG_BANK_SEL, update );
}


/**@brief Config PWR_MGMT_1 Register
 *
 */
void	ICM20649_set_pwr_mgmt_1( uint8_t config )
{

	ICM20649_write( ICM20649_RA_PWR_MGMT_1, config);

	// If Device Reset bit set, expect the Reset Value 0x41
	if( IS_SET(config, 7) )
	{
		NRFX_DELAY_US(10000); // Add delay to allow device to reset
		UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_PWR_MGMT_1, ICM20649_PWR_MGMT_1_RESET_VALUE );
	}
	else
	{
		UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_PWR_MGMT_1, config );
	}

}


/**@brief Config PWR_MGMT_1 Register
 *
 */
void	ICM20649_set_pwr_mgmt_2( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_PWR_MGMT_2;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	//******************************************
	// Section 9.8 of ICM20649 Datasheet - requires a 22usec delay after disabling GYRO before any
	// subsequent SPI writes
	if( (config & 0x07) == 0x07 ) NRFX_DELAY_US(22);
	//******************************************

	UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_PWR_MGMT_2, config );
}


void	ICM20649_set_int_pin_cgf( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_PIN_CFG;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_INT_PIN_CFG, config );
}


void	ICM20649_set_int_enable( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_ENABLE;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_INT_ENABLE, config );
}


void	ICM20649_set_int_enable_1( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_ENABLE_1;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_INT_ENABLE_1, config );
}


void	ICM20649_set_int_enable_2( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_ENABLE_2;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_INT_ENABLE_2, config );
}



void	ICM20649_set_int_enable_3( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_ENABLE_3;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_INT_ENABLE_3, config );
}


void ICM20649_set_fifo_2( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_FIFO_EN_2;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_FIFO_EN_2, config );
}


void ICM20649_set_gyro_smplrt_div( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_GYRO_SMPLRT_DIV;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_GYRO_SMPLRT_DIV, config );
}


void ICM20649_set_gyro_config_1( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_GYRO_CONFIG_1;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	update_gyro_range_settings(config); // track new Gyro range setting

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_GYRO_CONFIG_1, config );
}


void ICM20649_set_gyro_config_2( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_GYRO_CONFIG_2;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(uint8_t), rx_data, sizeof(rx_data));

	tx_data[0] = ICM20649_RA_GYRO_CONFIG_2;
	tx_data[1] = config | ( rx_data[1] & (BIT_7 | BIT_6) ); //	Ignore bits 6 and 7
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_GYRO_CONFIG_2, config );
}


/**@brief Sets the SMPLRT DIV setting for accelerometer. 12bit number passed as uint16_t
 *        Parameter is split and written over two registers as per ICM20649 datasheet
 *        ICM20649_RA_ACCEL_SMPLRT_DIV_1 -> ACCEL_SMPLRT_DIV[11:8]
 *        ICM20649_RA_ACCEL_SMPLRT_DIV_2 -> ACCEL_SMPLRT_DIV[7:0]
 */
void ICM20649_set_accel_smplrt_div( uint16_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_ACCEL_SMPLRT_DIV_1;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	tx_data[0] = ICM20649_RA_ACCEL_SMPLRT_DIV_1;
	tx_data[1] = ( config >> 8 ) & 0xFFF;  // Mask out any upper bits accidently written
	tx_data[1] = tx_data[1] | ( rx_data[1] & (BIT_7 | BIT_6 | BIT_5 | BIT_4) );  //	Ignore bits 4,5,6 and 7
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	tx_data[0] = ICM20649_RA_ACCEL_SMPLRT_DIV_2;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_ACCEL_SMPLRT_DIV_1, config );
}


void ICM20649_set_accel_smplrt_div_2( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_ACCEL_SMPLRT_DIV_2;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_ACCEL_SMPLRT_DIV_2, config );
}


void ICM20649_set_accel_intel_ctrl( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_ACCEL_INTEL_CTRL;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_ACCEL_INTEL_CTRL, config );
}


void ICM20649_set_accel_wake_threshold( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_ACCEL_WAKE_THRESH;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_ACCEL_WAKE_THRESH, config );
}



void ICM20649_set_accel_config( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_ACCEL_CONFIG;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	update_accel_range_settings(config); // track new Accel range setting

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_ACCEL_CONFIG, config );
}


void ICM20649_set_accel_config_2( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);
	tx_data[0] = ICM20649_RA_ACCEL_CONFIG_2;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(uint8_t), rx_data, sizeof(rx_data));

	tx_data[0] = ICM20649_RA_ACCEL_CONFIG_2;
	tx_data[1] = config | ( rx_data[1] & (BIT_7 | BIT_6 | BIT_5) ); //	Ignore bits 5,6 and 7
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, ICM20649_RA_ACCEL_CONFIG_2, config );
}


void ICM20649_set_temp_config( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = IMC20649_RA_TEMP_CONFIG;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, IMC20649_RA_TEMP_CONFIG, config );
}



void ICM20649_set_mod_ctrl_user( uint8_t config )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = IMC20649_RA_MOD_CTRL_USR;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	UNIT_TEST( ICM20649_USER_BANK_2, IMC20649_RA_MOD_CTRL_USR, config );
}

//*********************************************************************************************
//*                            SELF TESTS AND OFFSETS
//*********************************************************************************************

icm20649_all_imu_sensors_t	ICM20649_get_self_test_imu( uint8_t config )
{
	icm20649_all_imu_sensors_t self_test;
	uint8_t tx_data[2];
	uint8_t rx_data[7];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_1);

	tx_data[0] = ICM20649_RA_SELF_TEST_X_GYRO;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	self_test.gyro.x_axis = rx_data[1];
	self_test.gyro.y_axis = rx_data[2];
	self_test.gyro.z_axis = rx_data[3];
	self_test.accel.x_axis = rx_data[4];
	self_test.accel.y_axis = rx_data[5];
	self_test.accel.z_axis = rx_data[6];
	return self_test;
}


icm20649_three_axis_t	ICM20649_get_accel_offset_values( uint8_t config )
{
	icm20649_three_axis_t accel_offsets;
	uint8_t tx_data[2];
	uint8_t rx_data[7];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_1);

	tx_data[0] = ICM20649_RA_OFFSET_X_ACCEL;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	// Remove last bit in lower reg and first bit in upper reg
	accel_offsets.x_axis = (uint16_t)((rx_data[1] & 0x7F) << 8) | rx_data[2] >> 1;
	accel_offsets.y_axis = (uint16_t)((rx_data[3] & 0x7F) << 8) | rx_data[4] >> 1;
	accel_offsets.z_axis = (uint16_t)((rx_data[5] & 0x7F) << 8)  | rx_data[6] >> 1;

	return accel_offsets;
}

icm20649_three_axis_t	ICM20649_get_gyro_offset_values( uint8_t config )
{
	icm20649_three_axis_t gyro_offsets;
	uint8_t tx_data[2];
	uint8_t rx_data[7];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_2);

	tx_data[0] = ICM20649_RA_OFFSET_X_GYRO;
	tx_data[1] = config;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	gyro_offsets.x_axis = rx_data[1] << 8 | rx_data[2];
	gyro_offsets.y_axis = rx_data[3] << 8 | rx_data[4];
	gyro_offsets.z_axis = rx_data[5] << 8 | rx_data[6];

	return gyro_offsets;
}

//*********************************************************************************************
//*                            INTERRUPT STATUS READS
//*********************************************************************************************

uint8_t	ICM20649_read_int_status(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_STATUS;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}


uint8_t	ICM20649_read_int_status_1(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_STATUS_1;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}


uint8_t	ICM20649_read_int_status_2(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_STATUS_2;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}


uint8_t	ICM20649_read_int_status_3(void)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	tx_data[0] = ICM20649_RA_INT_STATUS_3;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));

	return rx_data[1];
}


//*********************************************************************************************
//*                            INTERRUPT SENSOR READS
//*********************************************************************************************
/**@brief read both accelerometer and gyroscope sensors
 *
 */
icm20649_all_imu_sensors_t	ICM20649_read_all_imu_sensors(void)
{
	icm20649_all_imu_sensors_t sensor_reading;
	uint8_t axis[13];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_ACCEL_XOUT_H, axis, 13);

	sensor_reading.accel.x_axis = axis[1] << 8 | axis[2];
	sensor_reading.accel.y_axis = axis[3] << 8 | axis[4];
	sensor_reading.accel.z_axis = axis[5] << 8 | axis[6];
	sensor_reading.gyro.x_axis	= axis[7] << 8 | axis[8];
	sensor_reading.gyro.y_axis 	= axis[9] << 8 | axis[10];
	sensor_reading.gyro.z_axis 	= axis[11] << 8 | axis[12];

	IMU_READ_TEST(sensor_reading);

	return sensor_reading;
}

/**@brief read accelerometer sensor
 *
 */
icm20649_three_axis_t	ICM20649_read_accel_all_axis(void)
{
	icm20649_three_axis_t accel_reading;
	uint8_t axis[7];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_ACCEL_XOUT_H, axis, 7);

	accel_reading.x_axis = axis[1] << 8 | axis[2];
	accel_reading.y_axis = axis[3] << 8 | axis[4];
	accel_reading.z_axis = axis[5] << 8 | axis[6];

	return accel_reading;
}


/**@brief read accelerometer x-axis
 *
 */
uint16_t	ICM20649_read_accel_x_axis(void)
{
	uint8_t axis[3];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_ACCEL_XOUT_H, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read accelerometer y-axis
 *
 */
uint16_t	ICM20649_read_accel_y_axis(void)
{
	uint8_t axis[3];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_ACCEL_YOUT_H, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read accelerometer z-axis
 *
 */
uint16_t	ICM20649_read_accel_z_axis(void)
{
	uint8_t axis[3];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_ACCEL_ZOUT_H, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read gyroscope sensor
 *
 */
icm20649_three_axis_t	ICM20649_read_gyro_all_axis(void)
{
	icm20649_three_axis_t gyro_reading;
	uint8_t axis[7];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_GYRO_XOUT_H, axis, 7);

	gyro_reading.x_axis = axis[1] << 8 | axis[2];
	gyro_reading.y_axis = axis[3] << 8 | axis[4];
	gyro_reading.z_axis = axis[5] << 8 | axis[6];

	return gyro_reading;
}


/**@brief read gyroscope x-axis
 *
 */
uint16_t	ICM20649_read_gyro_x_axis(void)
{
	uint8_t axis[3];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_GYRO_XOUT_H, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read gyroscope y-axis
 *
 */
uint16_t	ICM20649_read_gyro_y_axis(void)
{
	uint8_t axis[3];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_GYRO_YOUT_H, axis, 3);

	return axis[1] << 8 | axis[2];
}

/**@brief read gyroscope z-axis
 *
 */
uint16_t	ICM20649_read_gyro_z_axis(void)
{
	uint8_t axis[3];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_GYRO_ZOUT_H, axis, 3);

	return axis[1] << 8 | axis[2];
}

/**@brief read temperature
 *
 */
int16_t	ICM20649_read_temp(void)
{
	uint8_t temperature[3];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_TEMP_OUT_H, temperature, 3);

	return temperature[1] << 8 | temperature[2];
}


//*********************************************************************************************
//*********************************************************************************************


uint8_t	ICM20649_read_ID(void)
{
	uint8_t chip_id[2];

	ICM20649_select_bank(ICM20649_USER_BANK_0);

	ICM20649_read(ICM20649_RA_WHO_AM_I, chip_id, 2);

	//UNIT_TEST( ICM20649_USER_BANK_0, ICM20649_RA_WHO_AM_I, ICM20649_WHO_AM_I_CONST );
	return chip_id[1];
}


//*********************************************************************************************
//*********************************************************************************************

static void ICM20649_read( uint8_t address, uint8_t * rx_data, uint8_t length )
{
	uint8_t tx_data[28];

	memset(rx_data, 0, length);
	memset(tx_data, 0, sizeof(tx_data));

	tx_data[0] = address;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_READ, tx_data, length, rx_data, length);
}

static void ICM20649_write( uint8_t address, uint8_t value )
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));


	tx_data[0] = address;
	tx_data[1] = value;
	mw_spi_master_transfer(m_ICM20649_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));
}

//*********************************************************************************************
//*********************************************************************************************


void ICM20649_initialize( external_device_config_t device_config )
{
	if( device_config.communication == SPI_COMMUNICATION )
	{
		spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

		if(return_value.err_code != NRF_SUCCESS) APP_ERROR_CHECK(return_value.err_code);

		m_ICM20649_SPI_ID = return_value.device_id;

		MW_LOG_INFO("ICM20649_device SPI Initialized");
	}

	if ( device_config.communication == TWI_COMMUNICATION )
	{
		mw_twi_master_init(device_config.twi_config);

		MW_LOG_INFO("ICM20649_device TWI Initialized");
	}

	m_icm20649_initialized = true;
}
