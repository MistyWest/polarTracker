/*
 * ICM20648.c
 *
 *  Created on: Feb 21, 2019
 *      Author: klockwood
 */

#include <nrf_delay.h>

#include "ICM20648.h"

#include "../CLI_Logging/mw_logging.h"
#include "_mw_external_device.h"
//#include "ICM20648_dmp_fw_image.h"

#define ICM20648_TEST_UNITS_ENABLED				1

static bool	m_icm20648_initialized = false;

static external_driver_spi_config_t m_ICM20648_SPI_ID;


/**@brief Unit Test for each register write
 *
 */
static void IMU_READ_TEST( icm20648_all_imu_sensors_t readings )
{
#if !ICM20648_TEST_UNITS_ENABLED
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
#if !ICM20648_TEST_UNITS_ENABLED
	return;
#endif

	uint8_t tx_data[2];
	uint8_t rx_data[2];
	memset(tx_data, 0, sizeof(tx_data));
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(bank_no);

	tx_data[0] = register_to_check;
	ICM20648_read (register_to_check, rx_data, sizeof(rx_data));

	if( verify_value != rx_data[1] )
	{
		MW_LOG_INFO("Register: 0x%X shows unexpected value of 0x%X", register_to_check, verify_value)
	}
}

/***************************************************************************/
static void ICM20648_get_accelerometer_resolution( float *accel_res )
{
  uint8_t reg;

  /* Read the actual acceleration full scale setting */
  reg = ICM20648_get_accelerometer_config();
  reg &= ICM20648_MASK_ACCEL_FULLSCALE;

  /* Calculate the resolution */
  switch ( reg )
  {
  case ICM20648_ACCEL_FULLSCALE_2G:
  *accel_res = ICM20648_ACCEL_2G_SCALE_VALUE;
  break;

  case ICM20648_ACCEL_FULLSCALE_4G:
  *accel_res = ICM20648_ACCEL_4G_SCALE_VALUE;
  break;

  case ICM20648_ACCEL_FULLSCALE_8G:
  *accel_res = ICM20648_ACCEL_8G_SCALE_VALUE;
  break;

  case ICM20648_ACCEL_FULLSCALE_16G:
  *accel_res = ICM20648_ACCEL_16G_SCALE_VALUE;
  break;
  }
}

/***************************************************************************/
static void ICM20648_get_gyroscope_resolution( float * gyro_res )
{
  uint8_t reg;

  /* Read the actual gyroscope full scale setting */
  reg = ICM20648_get_gyroscope_config_1();
  reg &= ICM20648_MASK_ACCEL_FULLSCALE;

  /* Calculate the resolution */
  switch ( reg )
  {
  case ICM20648_GYRO_FULLSCALE_250DPS:
  *gyro_res = (ICM20648_GYRO_250DPS_SCALE_VALUE);
  break;

  case ICM20648_GYRO_FULLSCALE_500DPS:
  *gyro_res = (ICM20648_GYRO_500DPS_SCALE_VALUE);
  break;

  case ICM20648_GYRO_FULLSCALE_1000DPS:
  *gyro_res = (ICM20648_GYRO_1000DPS_SCALE_VALUE);
  break;

  case ICM20648_GYRO_FULLSCALE_2000DPS:
  *gyro_res = (ICM20648_GYRO_2000DPS_SCALE_VALUE);
  break;
  }
}

void ICM20648_convert_accelerometer_to_g( int16_t axis, float * float_value )
{
  ICM20648_get_accelerometer_resolution(float_value); // get resolution value
  *float_value = (float)axis * *float_value;          // calculate float value
}


void ICM20648_convert_gyroscope_to_angular_rate( int16_t axis, float * float_value )
{
  ICM20648_get_gyroscope_resolution(float_value); // get resolution value
  *float_value = (float) axis * *float_value;     // calculate float value
}

/**@brief Select Register Bank
 *
 */
void ICM20648_select_bank( icm20648_reg_bank_t bank_no )
{
  uint8_t update = 0;

  switch(bank_no)
  {
  // Clear bits 4:5
  case ICM20648_USER_BANK_0:
    CLR_BIT( update, 4);
    CLR_BIT( update, 5);
    break;

  // Set bits 4:5 1:0
  case ICM20648_USER_BANK_1:
    SET_BIT( update, 4);
    CLR_BIT( update, 5);
    break;

  // Set bits 4:5 0:1
  case ICM20648_USER_BANK_2:
    CLR_BIT( update, 4);
    SET_BIT( update, 5);
    break;

  // Set bits 4:5 1:1
  case ICM20648_USER_BANK_3:
    SET_BIT( update, 4);
    SET_BIT( update, 5);
    break;

  default:
    CLR_BIT( update, 4);
    CLR_BIT( update, 5);
    break;
  }

  ICM20648_write( ICM20648_REG_BANK_SEL, update );
}



/***************************************************************************/
void ICM20648_read_accelerometer_data( float *accel )
{
  uint8_t raw_data[6];
  float accel_res = 0; //TODO fix
  int16_t temp;

  /* Retrieve the current resolution */
  ICM20648_get_accelerometer_resolution(&accel_res);

  /* Read the six raw data registers into data array */
  ICM20648_read( ICM20648_REG_ACCEL_XOUT_H_SH, &raw_data[0], 6);

  /* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the G value */
  temp = ((int16_t) raw_data[0] << 8) | raw_data[1];
  accel[0] = (float) temp * accel_res;
  temp = ((int16_t) raw_data[2] << 8) | raw_data[3];
  accel[1] = (float) temp * accel_res;
  temp = ((int16_t) raw_data[4] << 8) | raw_data[5];
  accel[2] = (float) temp * accel_res;
}

/***************************************************************************/
void ICM20648_read_gyroscope_data( float *gyro )
{
  uint8_t rawData[6];
  float gyroRes = 0; //TODO fix
  int16_t temp;

  /* Retrieve the current resolution */
  ICM20648_get_gyroscope_resolution(&gyroRes);

  /* Read the six raw data registers into data array */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_GYRO_XOUT_H_SH, &rawData[0], 6);

  /* Convert the MSB and LSB into a signed 16-bit value and multiply by the resolution to get the dps value */
  temp = ((int16_t) rawData[0] << 8) | rawData[1];
  gyro[0] = (float) temp * gyroRes;
  temp = ((int16_t) rawData[2] << 8) | rawData[3];
  gyro[1] = (float) temp * gyroRes;
  temp = ((int16_t) rawData[4] << 8) | rawData[5];
  gyro[2] = (float) temp * gyroRes;
}


/**
 * @brief - Enable/Disable FIFO bit in USER_CTRL
 */
void ICM20648_enable_FIFO( bool enable)
{
  uint8_t reg[2];

  /* Enable the FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_USER_CTRL, reg, 2);
  if(enable)
  {
    reg[1] = reg[1] | ICM20648_BIT_FIFO_EN;  // Set FIFO enable bit
  }
  else
  {
    reg[1] = reg[1] & ~ICM20648_BIT_FIFO_EN;  // Clear FIFO enable bit
  }
  ICM20648_write( ICM20648_REG_USER_CTRL, reg[1]);
}


void ICM20648_reset_FIFO()
{
  /* Reset the FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_RST, 0x0F);

  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_RST, 0x00);
}


void ICM20648_FIFO_mode( bool snapshot)
{
  if(snapshot) /* Snapshot mode */
  {
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_write( ICM20648_REG_FIFO_MODE, 0x0F);
  }
  else /* Stream Mode */
  {
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_write( ICM20648_REG_FIFO_MODE, 0x00);
  }
}




/**
 * @brief - Enable/Disable FIFO Overflow Interrupt INT2
 */
void ICM20648_enable_FIFO_overflow_interrupt( bool enable)
{
  if(enable)
  {
    /* Enable the FIFO Watermark Interrupt */
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_write( ICM20648_REG_INT_ENABLE_2, 0x01);
  }
  else
  {
    /* Disable the FIFO Watermark Interrupt */
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_write( ICM20648_REG_INT_ENABLE_2, 0x00);
  }
}


/**
 * @brief - Enable/Disable FIFO Watermark Interrupt INT3
 */
void ICM20648_enable_FIFO_watermark_interrupt( bool enable)
{
  if(enable)
  {
    /* Enable the FIFO Watermark Interrupt */
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_write( ICM20648_REG_INT_ENABLE_3, 0x0F);
  }
  else
  {
    /* Disable the FIFO Watermark Interrupt */
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_write( ICM20648_REG_INT_ENABLE_3, 0x00);
  }
}


/**
 * @brief - Check FIFO Watermark Interrupt INT3 status
 */
uint8_t ICM20648_check_FIFO_watermark_interrupt()
{
  uint8_t rx_data[2];

  /* Enable the FIFO Watermark Interrupt */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_INT_STATUS_3, rx_data, 2);

  return rx_data[1];
}


uint16_t ICM20648_check_FIFO_count()
{
  uint8_t data[3];
  /* Read FIFO sample count */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_FIFO_COUNT_H, &data[0], 3);
  /* Convert to a 16 bit value */
  return ((uint16_t) (data[1] << 8) | data[2]);
}


/*
 * @brief - Set FIFO Enable Register 2
 */
void ICM20648_set_FIFO_sensor_enable( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_EN_2, config );
}


/*
 * @brief - Read FIFO Contents
 */
void ICM20648_read_FIFO( icm20648_all_imu_sensors_t * imu )
{
  uint8_t data[255];
  uint8_t index = 1;

  ICM20648_select_bank(ICM20648_USER_BANK_0);

  ICM20648_read(ICM20648_REG_FIFO_R_W, data, index+12);
  imu->accel.x_axis = (int16_t) data[index] << 8 | data[index+1];
  imu->accel.y_axis = (int16_t) data[index+2] << 8 | data[index+3];
  imu->accel.z_axis = (int16_t) data[index+4] << 8 | data[index+5];
  imu->gyro.x_axis  = (int16_t) data[index+6] << 8 | data[index+7];
  imu->gyro.y_axis  = (int16_t) data[index+8] << 8 | data[index+9];
  imu->gyro.z_axis  = (int16_t) data[index+10] << 8 | data[index+11];
}


/*
 * @brief - Read FIFO Contents in Burst.  Max Read size is 256bytes
 */
uint32_t ICM20648_read_FIFO_burst( uint8_t * data, uint16_t length )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

  if(length > 255) return NRF_ERROR_INVALID_LENGTH;

  ICM20648_read(ICM20648_REG_FIFO_R_W, data, length);

  return NRF_SUCCESS;
}


/**
 * @brief - Enable/Disable DMP bit in USER_CTRL
 */
void ICM20648_enable_DMP( bool enable)
{
  uint8_t reg[2];

  /* Enable the FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_USER_CTRL, reg, 2);
  if(enable)
  {
    reg[1] = reg[1] | ICM20648_BIT_DMP_EN;  // Set DMP enable bit
  }
  else
  {
    reg[1] = reg[1] & ~ICM20648_BIT_DMP_EN;  // Clear DMP enable bit
  }
  ICM20648_write( ICM20648_REG_USER_CTRL, reg[1]);
}





/**
 * @brief - Set DMP Watermark level
 */
void ICM20648_config_DMP_watermark( uint16_t watermark )
{
  uint8_t reg[2];

  /* Enable the FIFO */
  //ICM20648_select_bank(ICM20648_USER_BANK_0);

  reg[1] = 0x01;  // Hi byte of WATERMARK Register
  ICM20648_write( ICM20648_REG_MEM_BANK_SELECT, reg[1] );

  reg[1] = 0xFE; // Lo byte of WATERMARK Register
  ICM20648_write( ICM20648_REG_MEM_START_ADDR, reg[1] );

  ICM20648_write_two_bytes( ICM20648_REG_MEM_R_W, watermark );
}


/**
 * @brief - Enable/Disable Sleep bit in PWR_MGMT_1
 */
void ICM20648_enable_sleep( bool enable)
{
  uint8_t reg[2];

  /* Enable the FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_PWR_MGMT_1, reg, 2);
  if(enable)
  {
    reg[1] = reg[1] | ICM20648_BIT_SLEEP;  // Set FIFO enable bit
  }
  else
  {
    reg[1] = reg[1] & ~ICM20648_BIT_SLEEP;  // Clear FIFO enable bit
  }
  ICM20648_write( ICM20648_REG_PWR_MGMT_1, reg[1]);
}



/**@brief Config PWR_MGMT_1 Register
 *
 */
void	ICM20648_set_pwr_mgmt_1( uint8_t config )
{

  ICM20648_select_bank(ICM20648_USER_BANK_0);
	ICM20648_write( ICM20648_REG_PWR_MGMT_1, config);

	NRFX_DELAY_US(30000); // Add delay to allow clock sources to activate

	// If Device Reset bit set, expect the Reset Value 0x41
	if( IS_SET(config, 7) )
	{
		NRFX_DELAY_US(10000); // Add delay to allow device to reset
		UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_H_RESET );
	}
	else
	{
		UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_PWR_MGMT_1, config );
	}

}


/**@brief Config PWR_MGMT_1 Register
 *
 */
void	ICM20648_set_pwr_mgmt_2( uint8_t config )
{
	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_write(ICM20648_REG_PWR_MGMT_2, config);

	//******************************************
	// Section 9.8 of ICM20648 Datasheet - requires a 22usec delay after disabling GYRO before any
	// subsequent SPI writes
	if( (config & 0x07) == 0x07 ) NRFX_DELAY_US(22);
	//******************************************

	UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_PWR_MGMT_2, config );
}


/**
 * @brief - Writes contents of LP_CONFIG
 */
void ICM20648_set_lp_config( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

  ICM20648_write(ICM20648_REG_LP_CONFIG, config);
}


/**
 * @brief - Set Interrupt Configuration Register
 */
void	ICM20648_set_int_pin_cgf( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_write(ICM20648_REG_INT_PIN_CFG, config);

	UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_INT_PIN_CFG, config );
}


void	ICM20648_set_int_enable( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_write(ICM20648_REG_INT_ENABLE, config);

	UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_INT_ENABLE, config );
}


void	ICM20648_set_int_enable_1( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_write(ICM20648_REG_INT_ENABLE_1, config);

	UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_INT_ENABLE_1, config );
}


void	ICM20648_set_int_enable_2( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_write(ICM20648_REG_INT_ENABLE_2, config);

	UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_INT_ENABLE_2, config );
}



void	ICM20648_set_int_enable_3( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

  ICM20648_write(ICM20648_REG_INT_ENABLE_3, config);

	UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_INT_ENABLE_3, config );
}


void ICM20648_set_fifo_2( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_write(ICM20648_REG_FIFO_EN_2, config);

	UNIT_TEST( ICM20648_USER_BANK_0, ICM20648_REG_FIFO_EN_2, config );
}



void ICM20648_set_gyroscope_config_1( uint8_t config )
{
  ICM20648_select_bank(ICM20648_USER_BANK_2);

  ICM20648_write(ICM20648_REG_GYRO_CONFIG_1, config);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_GYRO_CONFIG_1, config );
}



uint8_t ICM20648_get_gyroscope_config_1()
{
  uint8_t rx_data[2];
  memset(rx_data, 0, sizeof(rx_data));

  ICM20648_select_bank(ICM20648_USER_BANK_2);

  ICM20648_read(ICM20648_REG_GYRO_CONFIG_1, rx_data, 2);

  return rx_data[1];
}



void ICM20648_set_gyroscope_config_2( uint8_t config )
{
  uint8_t tx_data;
	uint8_t rx_data[2];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_2);

	ICM20648_read(ICM20648_REG_GYRO_CONFIG_2, rx_data, 2);

	tx_data = config | ( rx_data[1] & (BIT_7 | BIT_6) ); //	Ignore bits 6 and 7

  ICM20648_write(ICM20648_REG_GYRO_CONFIG_2, tx_data);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_GYRO_CONFIG_2, tx_data );
}




void ICM20648_set_accelerometer_intel_ctrl( uint8_t config )
{
	ICM20648_select_bank(ICM20648_USER_BANK_2);

  ICM20648_write(ICM20648_REG_ACCEL_INTEL_CTRL, config);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_ACCEL_INTEL_CTRL, config );
}


void ICM20648_set_accelerometer_wake_threshold( uint8_t config )
{
	ICM20648_select_bank(ICM20648_USER_BANK_2);

	ICM20648_write(ICM20648_REG_ACCEL_WOM_THR, config);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_ACCEL_WOM_THR, config );
}


uint8_t ICM20648_get_accelerometer_config()
{
  uint8_t rx_data[2];
  memset(rx_data, 0, sizeof(rx_data));

  ICM20648_select_bank(ICM20648_USER_BANK_2);

  ICM20648_read(ICM20648_REG_ACCEL_CONFIG, rx_data, 2);

  return rx_data[1];
}

void ICM20648_set_accelerometer_config( uint8_t config )
{
	ICM20648_select_bank(ICM20648_USER_BANK_2);

  ICM20648_write(ICM20648_REG_ACCEL_CONFIG, config);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_ACCEL_CONFIG, config );
}


void ICM20648_set_accelerometer_config_2( uint8_t config )
{
	uint8_t tx_data;
	uint8_t rx_data[2];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_2);

	ICM20648_read(ICM20648_REG_ACCEL_CONFIG, rx_data, 2);

	tx_data = config | ( rx_data[1] & (BIT_7 | BIT_6 | BIT_5) ); //	Ignore bits 5,6 and 7

  ICM20648_write(ICM20648_REG_ACCEL_CONFIG, tx_data);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_ACCEL_CONFIG_2, tx_data );
}


void ICM20648_set_temp_config( uint8_t config )
{
	ICM20648_select_bank(ICM20648_USER_BANK_2);

  ICM20648_write(ICM20648_REG_TEMP_CONFIG, config);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_TEMP_CONFIG, config );
}



void ICM20648_set_mod_ctrl_user( uint8_t config )
{
	ICM20648_select_bank(ICM20648_USER_BANK_2);

  ICM20648_write(ICM20648_REG_MOD_CTRL_USR, config);

	UNIT_TEST( ICM20648_USER_BANK_2, ICM20648_REG_MOD_CTRL_USR, config );
}

//*********************************************************************************************
//*                            SELF TESTS AND OFFSETS
//*********************************************************************************************

icm20648_all_imu_sensors_t	ICM20648_read_self_test_imu( uint8_t config )
{
	icm20648_all_imu_sensors_t self_test;
	uint8_t rx_data[7];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_1);

  ICM20648_read(ICM20648_REG_SELF_TEST_X_GYRO, rx_data, 7);

	self_test.gyro.x_axis = rx_data[1];
	self_test.gyro.y_axis = rx_data[2];
	self_test.gyro.z_axis = rx_data[3];
	self_test.accel.x_axis = rx_data[4];
	self_test.accel.y_axis = rx_data[5];
	self_test.accel.z_axis = rx_data[6];
	return self_test;
}


icm20648_three_axis_t	ICM20648_get_accelerometer_offset_values()
{
	icm20648_three_axis_t accel_offsets;
	uint8_t rx_data[7];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_1);

	ICM20648_read(ICM20648_REG_XA_OFFSET_H, rx_data, sizeof(rx_data));

	// Remove last bit in lower reg and first bit in upper reg
	accel_offsets.x_axis = (uint16_t)((rx_data[1] & 0x7F) << 8) | rx_data[2] >> 1;
	accel_offsets.y_axis = (uint16_t)((rx_data[3] & 0x7F) << 8) | rx_data[4] >> 1;
	accel_offsets.z_axis = (uint16_t)((rx_data[5] & 0x7F) << 8)  | rx_data[6] >> 1;

	return accel_offsets;
}


//*********************************************************************************************
//*                            INTERRUPT STATUS READS
//*********************************************************************************************
/**
 * @brief - Read Interrupt Status Register
 */
uint8_t	ICM20648_read_int_status(void)
{
	uint8_t rx_data[2];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_0);

  ICM20648_read(ICM20648_REG_INT_STATUS, rx_data, 2);

	return rx_data[1];
}


/**
 * @brief - Read Interrupt 1 Status
 */
uint8_t	ICM20648_read_interrupt_status_1(void)
{
	uint8_t rx_data[2];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_0);

  ICM20648_read(ICM20648_REG_INT_STATUS_1, rx_data, 2);

	return rx_data[1];
}


/**
 * @brief - Read Interrupt 2 Status
 */
uint8_t	ICM20648_read_interrupt_status_2(void)
{
	uint8_t rx_data[2];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_0);

  ICM20648_read(ICM20648_REG_INT_STATUS_2, rx_data, 2);

	return rx_data[1];
}


/**
 * @brief - Read Interrupt 3 Status
 */
uint8_t	ICM20648_read_interrupt_status_3(void)
{
	uint8_t rx_data[2];
	memset(rx_data, 0, sizeof(rx_data));

	ICM20648_select_bank(ICM20648_USER_BANK_0);

  ICM20648_read(ICM20648_REG_INT_ENABLE_3, rx_data, 2);

	return rx_data[1];
}


//*********************************************************************************************
//*                            INTERRUPT SENSOR READS
//*********************************************************************************************
/**@brief read both accelerometer and gyroscope sensors
 *
 */
void	ICM20648_read_all_imu_sensors( icm20648_all_imu_sensors_t * imu )
{
	uint8_t axis[13];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_ACCEL_XOUT_H_SH, axis, 7);
	imu->accel.x_axis = (int16_t) axis[1] << 8 | axis[2];
	imu->accel.y_axis = (int16_t) axis[3] << 8 | axis[4];
	imu->accel.z_axis = (int16_t) axis[5] << 8 | axis[6];

  ICM20648_read(ICM20648_REG_GYRO_XOUT_H_SH, axis, 7);
  imu->gyro.x_axis = (int16_t) axis[1] << 8 | axis[2];
  imu->gyro.y_axis = (int16_t) axis[3] << 8 | axis[4];
  imu->gyro.z_axis = (int16_t) axis[5] << 8 | axis[6];

//	imu->gyro.x_axis	= axis[7] << 8 | axis[8];
//	imu->gyro.y_axis 	= axis[9] << 8 | axis[10];
//	imu->gyro.z_axis 	= axis[11] << 8 | axis[12];

	IMU_READ_TEST(*imu);
}

/**@brief read accelerometer sensor
 *
 */
icm20648_three_axis_t	ICM20648_read_accelerometer_all_axis(void)
{
	icm20648_three_axis_t accel_reading;
	uint8_t axis[7];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_ACCEL_XOUT_H_SH, axis, 7);

	accel_reading.x_axis = axis[1] << 8 | axis[2];
	accel_reading.y_axis = axis[3] << 8 | axis[4];
	accel_reading.z_axis = axis[5] << 8 | axis[6];

	return accel_reading;
}


/**@brief read accelerometer x-axis
 *
 */
uint16_t	ICM20648_read_accelerometer_x_axis(void)
{
	uint8_t axis[3];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_ACCEL_XOUT_H_SH, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read accelerometer y-axis
 *
 */
uint16_t	ICM20648_read_accelerometer_y_axis(void)
{
	uint8_t axis[3];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_ACCEL_YOUT_H_SH, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read accelerometer z-axis
 *
 */
uint16_t	ICM20648_read_accelerometer_z_axis(void)
{
	uint8_t axis[3];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_ACCEL_ZOUT_H_SH, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read gyroscope sensor
 *
 */
icm20648_three_axis_t	ICM20648_read_gyroscope_all_axis(void)
{
	icm20648_three_axis_t gyro_reading;
	uint8_t axis[7];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_GYRO_XOUT_H_SH, axis, 7);

	gyro_reading.x_axis = axis[1] << 8 | axis[2];
	gyro_reading.y_axis = axis[3] << 8 | axis[4];
	gyro_reading.z_axis = axis[5] << 8 | axis[6];

	return gyro_reading;
}


/**@brief read gyroscope x-axis
 *
 */
uint16_t	ICM20648_read_gyroscope_x_axis(void)
{
	uint8_t axis[3];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_GYRO_XOUT_H_SH, axis, 3);

	return axis[1] << 8 | axis[2];
}


/**@brief read gyroscope y-axis
 *
 */
uint16_t	ICM20648_read_gyroscope_y_axis(void)
{
	uint8_t axis[3];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_GYRO_YOUT_H_SH, axis, 3);

	return axis[1] << 8 | axis[2];
}

/**@brief read gyroscope z-axis
 *
 */
uint16_t	ICM20648_read_gyroscope_z_axis(void)
{
	uint8_t axis[3];

	ICM20648_select_bank(ICM20648_USER_BANK_0);

	ICM20648_read(ICM20648_REG_GYRO_ZOUT_H_SH, axis, 3);

	return axis[1] << 8 | axis[2];
}

//*********************************************************************************************
//*********************************************************************************************

/**
 * @brief Reads the device Temperature
 */
void ICM20648_read_temperature( float *temperature )
{
  uint8_t data[3];
  int16_t raw_temp;

  /* Read temperature registers */
  ICM20648_read( ICM20648_REG_TEMPERATURE_H, data, 3);

  /* Convert to int16 */
  raw_temp = (int16_t) ((data[1] << 8) + data[2]);

  /* Calculate the Celsius value from the raw reading */
  *temperature = ((float) ( raw_temp - ICM20648_TEMPERATURE_OFFSET) / ICM20648_TEMPERATURE_SCALE_VALUE) + ICM20648_TEMPERATURE_OFFSET;
}


/**
 * @brief Reads the device ID
 */
uint8_t ICM20648_read_device_ID(void)
{
  uint8_t device_id[2];
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_WHO_AM_I, device_id, 2);
  return device_id[1];
}







/**
 * @brief Runs the Self Test routine and returns a boolean for PASS/FAIL
 */
//@self
bool ICM20648_run_self_test()
{
  bool test_result = true;;
  float self_test_result;
  icm20648_all_imu_sensors_self_test_t self_test_ON, self_test_OFF;
  icm20648_all_imu_sensors_t  temp;
  memset( &self_test_ON, 0, sizeof(self_test_ON));
  memset( &temp, 0, sizeof(temp) );

  /**  Self-Test Setup */
  ICM20648_sensor_enable(true, true, true);  // Enable all sensors

  ICM20648_set_accelerometer_sample_rate(1000);  // Set sample rate to 1kHz

  ICM20648_set_gyroscope_sample_rate(1000);  // Set sample rate to 1kHz

  ICM20648_set_gyroscope_lpf(ICM20648_GYRO_BW_24HZ);

  ICM20648_set_accelerometer_lpf(ICM20648_ACCEL_BW_24HZ);

  ICM20648_set_gyroscope_config_1( 0x11); // Set Range to +/-250dps, with DLPF value of 2

  ICM20648_set_accelerometer_config( 0x11 ); // Set Range to +/-2G, with DLPF value of 2

  ICM20648_set_gyroscope_config_2(ICM20648_BIT_GYRO_SELF_TEST_EN | 0x03);  // enable XYZ Gyro Self Test  w/ 8x averaging
  NRFX_DELAY_US(50000);
  ICM20648_set_accelerometer_config_2(ICM20648_BIT_ACCEL_SELF_TEST_EN | 0x02);  // enable XYZ Accel Self Test w/ 16x averaging
  NRFX_DELAY_US(50000);

  //********************************************************************************************
  /**  Average 200 readings */
  for( uint8_t i=1; i<200; i++)
  {
    ICM20648_read_all_imu_sensors(&temp); // Read Sensors

    self_test_ON.accel.x_axis += temp.accel.x_axis;
    self_test_ON.accel.y_axis += temp.accel.y_axis;
    self_test_ON.accel.z_axis += temp.accel.z_axis;
    self_test_ON.gyro.x_axis += temp.gyro.x_axis;
    self_test_ON.gyro.y_axis += temp.gyro.y_axis;
    self_test_ON.gyro.z_axis += temp.gyro.z_axis;
    NRFX_DELAY_US(1000);
  }

  self_test_ON.accel.x_axis /= 200;
  self_test_ON.accel.y_axis /= 200;
  self_test_ON.accel.z_axis /= 200;
  self_test_ON.gyro.x_axis /= 200;
  self_test_ON.gyro.y_axis /= 200;
  self_test_ON.gyro.z_axis /= 200;

  //********************************************************************************************
  /**  Self-Test Without Internal Sensor Stimulus */
  ICM20648_set_gyroscope_config_2(0x03);  // disable XYZ Gyro Self Test w/ 8x averaging
  NRFX_DELAY_US(50000);
  ICM20648_set_accelerometer_config_2(0x02);  // disable XYZ Accel Self Test  w/ 16x averaging
  NRFX_DELAY_US(50000);

  memset( &self_test_OFF, 0, sizeof(self_test_OFF));
  memset( &temp, 0, sizeof(temp) );

  /**  Average 200 readings */
  for( uint8_t i=1; i<200; i++)
  {
    ICM20648_read_all_imu_sensors(&temp); // Read Sensors

    self_test_OFF.accel.x_axis += temp.accel.x_axis;
    self_test_OFF.accel.y_axis += temp.accel.y_axis;
    self_test_OFF.accel.z_axis += temp.accel.z_axis;
    self_test_OFF.gyro.x_axis += temp.gyro.x_axis;
    self_test_OFF.gyro.y_axis += temp.gyro.y_axis;
    self_test_OFF.gyro.z_axis += temp.gyro.z_axis;
    NRFX_DELAY_US(1000);
  }

  self_test_OFF.accel.x_axis /= 200;
  self_test_OFF.accel.y_axis /= 200;
  self_test_OFF.accel.z_axis /= 200;
  self_test_OFF.gyro.x_axis /= 200;
  self_test_OFF.gyro.y_axis /= 200;
  self_test_OFF.gyro.z_axis /= 200;

  // Subtract the LSBs, mask MSBs
  self_test_ON.accel.x_axis = (self_test_ON.accel.x_axis & 0xFF) - (self_test_OFF.accel.x_axis & 0xFF);
  self_test_ON.accel.y_axis = (self_test_ON.accel.y_axis & 0xFF) - (self_test_OFF.accel.y_axis & 0xFF);
  self_test_ON.accel.z_axis = (self_test_ON.accel.z_axis & 0xFF) - (self_test_OFF.accel.z_axis & 0xFF);
  self_test_ON.gyro.x_axis  = (self_test_ON.gyro.x_axis & 0xFF) - (self_test_OFF.gyro.x_axis & 0xFF);
  self_test_ON.gyro.y_axis  = (self_test_ON.gyro.y_axis & 0xFF) - (self_test_OFF.gyro.y_axis & 0xFF);
  self_test_ON.gyro.z_axis  = (self_test_ON.gyro.z_axis & 0xFF) - (self_test_OFF.gyro.z_axis & 0xFF);

  //********************************************************************************************
  ICM20648_get_factory_self_test(&temp);
  /** Calculate Selt Test Stored Values ST_OTP */
  if( temp.accel.x_axis != 0 )
  {
    temp.accel.x_axis = inv_Self_Test_Equation[ temp.accel.x_axis - 1 ];
  }

  if( temp.accel.y_axis != 0 )
  {
    temp.accel.y_axis = inv_Self_Test_Equation[ temp.accel.y_axis - 1 ];
  }

  if( temp.accel.z_axis != 0 )
  {
    temp.accel.z_axis = inv_Self_Test_Equation[ temp.accel.z_axis - 1 ];
  }

  if( temp.gyro.x_axis != 0 )
  {
    temp.gyro.x_axis = inv_Self_Test_Equation[ temp.gyro.x_axis - 1 ];
  }
  if( temp.gyro.y_axis != 0 )
  {
    temp.gyro.y_axis = inv_Self_Test_Equation[ temp.gyro.y_axis - 1 ];
  }
  if( temp.gyro.z_axis != 0 )
  {
    temp.gyro.z_axis = inv_Self_Test_Equation[ temp.gyro.z_axis - 1 ];
  }
  //********************************************************************************************

  self_test_result = (float) self_test_ON.accel.x_axis / temp.accel.x_axis;
  if( self_test_result > 0.5 ) test_result = false;

  self_test_result = (float) self_test_ON.accel.y_axis / temp.accel.y_axis;
  if( self_test_result > 0.5 ) test_result = false;

  self_test_result = (float) self_test_ON.accel.z_axis / temp.accel.z_axis;
  if( self_test_result > 0.5 ) test_result = false;

  self_test_result  = (float) self_test_ON.gyro.x_axis / temp.gyro.x_axis;
  if( self_test_result > 1.5 ) test_result = false;

  self_test_result  = (float) self_test_ON.gyro.y_axis / temp.gyro.y_axis;
  if( self_test_result > 1.5 ) test_result = false;

  self_test_result  = (float) self_test_ON.gyro.z_axis / temp.gyro.z_axis;
  if( self_test_result > 1.5 ) test_result = false;

  return test_result;
}



/**
 * @brief Runs a Calibration routine and returns the float bias values in an parameter array [3] pointers
 */
//@calibration
void ICM20648_calibrate( float *accel_bias_scaled, float *gyro_bias_scaled )
{
  uint8_t data[13];
  uint16_t i, packet_count, fifo_count;
  int32_t gyro_bias[3] = { 0, 0, 0 };
  int32_t accel_bias[3] = { 0, 0, 0 };
  int32_t accel_temp[3];
  int32_t gyro_temp[3];
  int32_t accel_bias_factory[3];
  int32_t gyro_bias_stored[3];
  float gyro_res = 0 , accel_res = 0;

  /* Set 1kHz sample rate */
  ICM20648_set_accelerometer_sample_rate(1100.0);
  ICM20648_set_gyroscope_sample_rate(1100.0);
  //ICM20648_sampleRateSet(1100.0);

  /* 246Hz BW for the accelerometer and 200Hz for the gyroscope */
  ICM20648_set_accelerometer_lpf( ICM20648_ACCEL_BW_246HZ );
  ICM20648_set_gyroscope_lpf( ICM20648_GYRO_BW_12HZ );

  /* Set the most sensitive range: 2G full scale and 250dps full scale */
  ICM20648_set_accelerometer_scale( ICM20648_ACCEL_FULLSCALE_2G );
  ICM20648_set_gyroscope_scale( ICM20648_GYRO_FULLSCALE_250DPS );


  /* Retrieve the resolution per bit */
  ICM20648_get_accelerometer_resolution(&accel_res);
  ICM20648_get_gyroscope_resolution(&gyro_res);

  /* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
  /* Experiments show that the gyro needs more time to get reliable results */
  nrf_delay_ms(50);

  /* Disable the FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);

  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_MODE, 0x0F);

  /* Enable accelerometer and gyro to store the data in FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_EN_2, ICM20648_BIT_ACCEL_FIFO_EN | ICM20648_BITS_GYRO_FIFO_EN);

  /* Reset the FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_RST, 0x0F);

  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_RST, 0x00);

  /* Enable the FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_USER_CTRL, ICM20648_BIT_FIFO_EN);

  /* Enable the accelerometer and the gyro */
  ICM20648_sensor_enable( true, true, false);

  /* The max FIFO size is 512 bytes, one set of measurements takes 12 bytes */
  /* (3 axes, 2 sensors, 2 bytes each value ) 340 samples use 512 bytes of FIFO */
  /* Loop until at least 490 samples gathered */
  fifo_count = 0;
  while ( fifo_count < 490 )
  {
    nrf_delay_ms(5);
    /* Read FIFO sample count */
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_read( ICM20648_REG_FIFO_COUNT_H, &data[0], 3);
    /* Convert to a 16 bit value */
    fifo_count = ((uint16_t) (data[1] << 8) | data[2]);
  }

  /* Disable accelerometer and gyro to store the data in FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_FIFO_EN_2, 0x00);

  /* Read FIFO sample count */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_FIFO_COUNT_H, &data[0], 3);

  /* Convert to a 16 bit value */
  fifo_count = ((uint16_t) (data[1] << 8) | data[2]);

  /* Calculate the number of data sets (3 axis of accel an gyro, two bytes each = 12 bytes) */
  packet_count = fifo_count / 12;

  /* Retrieve the data from the FIFO */
  for ( i = 0; i < packet_count; i++ )
  {
    ICM20648_select_bank(ICM20648_USER_BANK_0);
    ICM20648_read( ICM20648_REG_FIFO_R_W, &data[0], 13);
    /* Convert to 16 bit signed accel and gyro x,y and z values */
    accel_temp[0] = ((int16_t) (data[1] << 8) | data[2]);
    accel_temp[1] = ((int16_t) (data[3] << 8) | data[4]);
    accel_temp[2] = ((int16_t) (data[5] << 8) | data[6]);
    gyro_temp[0]  = ((int16_t) (data[7] << 8) | data[8]);
    gyro_temp[1]  = ((int16_t) (data[9] << 8) | data[10]);
    gyro_temp[2]  = ((int16_t) (data[11] << 8)| data[12]);

    /* Sum the values */
    accel_bias[0] += accel_temp[0];
    accel_bias[1] += accel_temp[1];
    accel_bias[2] += accel_temp[2];
    gyro_bias[0]  += gyro_temp[0];
    gyro_bias[1]  += gyro_temp[1];
    gyro_bias[2]  += gyro_temp[2];
  }

  /* Divide by packet count to get the average */
  accel_bias[0] /= packet_count;
  accel_bias[1] /= packet_count;
  accel_bias[2] /= packet_count;
  gyro_bias[0]  /= packet_count;
  gyro_bias[1]  /= packet_count;
  gyro_bias[2]  /= packet_count;

  /* Acceleormeter: add or remove (depending on the orientation of the chip) 1G (gravity) from the Z axis value */
  if ( accel_bias[2] > 0L )
  {
    accel_bias[2] -= (int32_t) (1.0 / accel_res);
  }
  else
  {
    accel_bias[2] += (int32_t) (1.0 / accel_res);
  }

  /* Convert the values to degrees per sec for displaying */
  gyro_bias_scaled[0] = (float) gyro_bias[0] * gyro_res;
  gyro_bias_scaled[1] = (float) gyro_bias[1] * gyro_res;
  gyro_bias_scaled[2] = (float) gyro_bias[2] * gyro_res;

  /* Read stored gyro trim values. After reset these values are all 0 */
  ICM20648_select_bank(ICM20648_USER_BANK_2);
  ICM20648_read( ICM20648_REG_XG_OFFS_USRH, &data[0], 3);
  gyro_bias_stored[0] = ((int16_t) (data[1] << 8) | data[2]);
  ICM20648_read( ICM20648_REG_YG_OFFS_USRH, &data[0], 3);
  gyro_bias_stored[1] = ((int16_t) (data[1] << 8) | data[2]);
  ICM20648_read( ICM20648_REG_ZG_OFFS_USRH, &data[0], 3);
  gyro_bias_stored[2] = ((int16_t) (data[1] << 8) | data[2]);

  /* The gyro bias should be stored in 1000dps full scaled format. We measured in 250dps to get */
  /* the best sensitivity, so need to divide by 4 */
  /* Substract from the stored calibration value */
  gyro_bias_stored[0] -= gyro_bias[0] / 4;
  gyro_bias_stored[1] -= gyro_bias[1] / 4;
  gyro_bias_stored[2] -= gyro_bias[2] / 4;

  /* Split the values into two bytes */
  data[0] = (gyro_bias_stored[0] >> 8) & 0xFF;
  data[1] = (gyro_bias_stored[0]) & 0xFF;
  data[2] = (gyro_bias_stored[1] >> 8) & 0xFF;
  data[3] = (gyro_bias_stored[1]) & 0xFF;
  data[4] = (gyro_bias_stored[2] >> 8) & 0xFF;
  data[5] = (gyro_bias_stored[2]) & 0xFF;

  /* Write the  gyro bias values to the chip */
  ICM20648_select_bank(ICM20648_USER_BANK_2);
  ICM20648_write( ICM20648_REG_XG_OFFS_USRH, data[0]);
  ICM20648_write( ICM20648_REG_XG_OFFS_USRL, data[1]);
  ICM20648_write( ICM20648_REG_YG_OFFS_USRH, data[2]);
  ICM20648_write( ICM20648_REG_YG_OFFS_USRL, data[3]);
  ICM20648_write( ICM20648_REG_ZG_OFFS_USRH, data[4]);
  ICM20648_write( ICM20648_REG_ZG_OFFS_USRL, data[5]);

  /* Read stored gyro trim values. After reset these values are all 0 */
  ICM20648_select_bank(ICM20648_USER_BANK_2);
  ICM20648_read( ICM20648_REG_XG_OFFS_USRH, &data[0], 3);
  gyro_bias_stored[0] = ((int16_t) (data[1] << 8) | data[2]);
  ICM20648_read( ICM20648_REG_YG_OFFS_USRH, &data[0], 3);
  gyro_bias_stored[1] = ((int16_t) (data[1] << 8) | data[2]);
  ICM20648_read( ICM20648_REG_ZG_OFFS_USRH, &data[0], 3);
  gyro_bias_stored[2] = ((int16_t) (data[1] << 8) | data[2]);

  /* Calculate the accelerometer bias values to store in the hardware accelerometer bias registers. These registers contain */
  /* factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold */
  /* non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature */
  /* compensation calculations(? the datasheet is not clear). Accelerometer bias registers expect bias input */
  /* as 2048 LSB per g, so that the accelerometer biases calculated above must be divided by 8. */

  /* Read factory accelerometer trim values */
  ICM20648_select_bank(ICM20648_USER_BANK_1);
  ICM20648_read( ICM20648_REG_XA_OFFSET_H, &data[0], 3 );
  accel_bias_factory[0] = ((int16_t) (data[1] << 8) | ( data[2] >> 1) );
  ICM20648_read( ICM20648_REG_YA_OFFSET_H, &data[0], 3 );
  accel_bias_factory[1] = ((int16_t) (data[1] << 8) | ( data[2] >> 1) );
  ICM20648_read( ICM20648_REG_ZA_OFFSET_H, &data[0], 3 );
  accel_bias_factory[2] = ((int16_t) (data[1] << 8) | ( data[2] >> 1) );

  /* Construct total accelerometer bias, including calculated average accelerometer bias from above */
  /* Scale the 2g full scale (most sensitive range) results to 16g full scale - divide by 8 */
  /* Clear the last bit (temperature compensation? - the datasheet is not clear) */
  /* Substract from the factory calibration value */
  accel_bias_factory[0] -= ((accel_bias[0] / 8) & ~1);
  accel_bias_factory[1] -= ((accel_bias[1] / 8) & ~1);
  accel_bias_factory[2] -= ((accel_bias[2] / 8) & ~1);

  /* Split the values into two bytes */
  data[0] = (accel_bias_factory[0] >> 8) & 0xFF;
  data[1] = (accel_bias_factory[0]) & 0xFF;
  data[2] = (accel_bias_factory[1] >> 8) & 0xFF;
  data[3] = (accel_bias_factory[1]) & 0xFF;
  data[4] = (accel_bias_factory[2] >> 8) & 0xFF;
  data[5] = (accel_bias_factory[2]) & 0xFF;

  // Edit do not store bias, use float value accel_bias_scaled
  // in the calling application code
  /* Store them in the accelerometer offset registers */
//  ICM20648_select_bank(ICM20648_USER_BANK_1);
//  ICM20648_write( ICM20648_REG_XA_OFFSET_H, data[0]);
//  ICM20648_write( ICM20648_REG_XA_OFFSET_L, data[1]);
//  ICM20648_write( ICM20648_REG_YA_OFFSET_H, data[2]);
//  ICM20648_write( ICM20648_REG_YA_OFFSET_L, data[3]);
//  ICM20648_write( ICM20648_REG_ZA_OFFSET_H, data[4]);
//  ICM20648_write( ICM20648_REG_ZA_OFFSET_L, data[5]);

  /* Convert the values to G for displaying */
  accel_bias_scaled[0] = (float) accel_bias[0] * accel_res;
  accel_bias_scaled[1] = (float) accel_bias[1] * accel_res;
  accel_bias_scaled[2] = (float) accel_bias[2] * accel_res;

  /* Turn off FIFO */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_USER_CTRL, 0x00);

  /* Disable all sensors */
  ICM20648_sensor_enable( false, false, false);
}

uint8_t ICM20648_get_accelerometer_scale()
{
  uint8_t current_scale = ICM20648_get_accelerometer_config();

  current_scale &= ICM20648_MASK_ACCEL_FULLSCALE;
  return current_scale;
}

void ICM20648_set_accelerometer_scale( uint8_t scale )
{
  uint8_t current_scale = ICM20648_get_accelerometer_config();

  current_scale &= ~ICM20648_MASK_ACCEL_FULLSCALE;
  current_scale |= scale;
  ICM20648_set_accelerometer_config(current_scale);
}



void ICM20648_set_gyroscope_scale( uint8_t scale )
{
  uint8_t current_scale = ICM20648_get_gyroscope_config_1();

  current_scale &= ~ICM20648_MASK_GYRO_FULLSCALE;
  current_scale |= scale;
  ICM20648_set_gyroscope_config_1(current_scale);
}


uint8_t ICM20648_get_gyroscope_scale()
{
  uint8_t current_scale = ICM20648_get_gyroscope_config_1();

  current_scale &= ICM20648_MASK_GYRO_FULLSCALE;
  return current_scale;
}




uint8_t ICM20648_get_accelerometer_lpf()
{
  uint8_t current_bw = ICM20648_get_accelerometer_config();

  current_bw &= ICM20648_MASK_GYRO_BW;
  return current_bw;
}

void ICM20648_set_accelerometer_lpf( uint8_t bandwidth )
{
  uint8_t current_bw = ICM20648_get_accelerometer_config();

  current_bw &= ~ICM20648_MASK_GYRO_BW;
  current_bw |= bandwidth;
  ICM20648_set_accelerometer_config(current_bw);
}



void ICM20648_set_gyroscope_lpf( uint8_t bandwidth )
{
  uint8_t current_bw = ICM20648_get_gyroscope_config_1();

  current_bw &= ~ICM20648_MASK_GYRO_BW;
  current_bw |= bandwidth;
  ICM20648_set_gyroscope_config_1(current_bw);
}


uint8_t ICM20648_get_gyroscope_lpf()
{
  uint8_t current_bw = ICM20648_get_gyroscope_config_1();

  current_bw &= ICM20648_MASK_GYRO_BW;
  return current_bw;
}


/***************************************************************************/
void ICM20648_set_gyroscope_sample_rate( float sample_rate )
{
  uint8_t gyro_div;

  /* Calculate the sample rate divider */
  sample_rate = (1125.0 / sample_rate) - 1.0;

  /* Check if it fits in the divider register */
  if ( sample_rate > 255.0 )
  {
    sample_rate = 255.0;
  }

  if ( sample_rate < 0 )
  {
    sample_rate = 0.0;
  }

  /* Write the value to the register */
  gyro_div = (uint8_t) sample_rate;

  ICM20648_select_bank(ICM20648_USER_BANK_2);
  ICM20648_write( ICM20648_REG_GYRO_SMPLRT_DIV, gyro_div);
}


uint8_t ICM20648_get_gyroscope_sample_rate()
{
  uint8_t reg_value[2];
  ICM20648_select_bank(ICM20648_USER_BANK_2);
  ICM20648_read( ICM20648_REG_GYRO_SMPLRT_DIV, reg_value, 2 );
  return reg_value[1];
}

/***************************************************************************/
void ICM20648_set_accelerometer_sample_rate( float sample_rate )
{
  uint16_t accel_div;

  /* Calculate the sample rate divider */
  sample_rate = (1125.0 / sample_rate) - 1.0;

  /* Check if it fits in the divider registers */
  if ( sample_rate > 4095.0 )
  {
    sample_rate = 4095.0;
  }

  if ( sample_rate < 0 )
  {
    sample_rate = 0.0;
  }

  /* Write the value to the registers */
  accel_div = (uint16_t) sample_rate;

  ICM20648_select_bank(ICM20648_USER_BANK_2);
  ICM20648_write( ICM20648_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accel_div >> 8));
  ICM20648_write( ICM20648_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accel_div & 0xFF));
}


uint16_t ICM20648_get_accelerometer_sample_rate()
{
  uint8_t reg_value[3];
  ICM20648_read( ICM20648_REG_ACCEL_SMPLRT_DIV_1, reg_value, 3 );
  return reg_value[1] << 8 | reg_value[2];
}

/***************************************************************************/



void  ICM20648_sensor_enable( bool accel, bool gyro, bool temp )
{
  uint8_t pwg_mgmt_1[2];
  uint8_t pwg_mgmt_2;

  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_read( ICM20648_REG_PWR_MGMT_1, pwg_mgmt_1, 2 );

  pwg_mgmt_2 = 0;

  /* To enable the accelerometer clear the DISABLE_ACCEL bits in PWR_MGMT_2 */
  if ( accel )
  {
    pwg_mgmt_2 &= ~( ICM20648_BIT_PWR_ACCEL_STBY);
  }
  else
  {
    pwg_mgmt_2 |= ICM20648_BIT_PWR_ACCEL_STBY;
  }
  /* To enable gyro clear the DISABLE_GYRO bits in PWR_MGMT_2 */
  if ( gyro )
  {
    pwg_mgmt_2 &= ~( ICM20648_BIT_PWR_GYRO_STBY);
  }
  else
  {
    pwg_mgmt_2 |= ICM20648_BIT_PWR_GYRO_STBY;
  }

  /* To enable the temperature sensor clear the TEMP_DIS bit in PWR_MGMT_1 */
  if ( temp )
  {
    pwg_mgmt_1[1] &= ~( ICM20648_BIT_TEMP_DIS);
  }
  else
  {
    pwg_mgmt_1[1] |= ICM20648_BIT_TEMP_DIS;
  }

  /* Write back the modified values */
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_PWR_MGMT_1, pwg_mgmt_1[1]);
  ICM20648_write( ICM20648_REG_PWR_MGMT_2, pwg_mgmt_2);
}


void ICM20648_get_factory_self_test( icm20648_all_imu_sensors_t * self_test )
{
  uint8_t rx_data[7];
  memset(rx_data, 0, sizeof(rx_data));

  ICM20648_select_bank(ICM20648_USER_BANK_1);

  ICM20648_read(ICM20648_REG_SELF_TEST_X_GYRO, rx_data, 4);
  self_test->gyro.x_axis = rx_data[1];
  self_test->gyro.y_axis = rx_data[2];
  self_test->gyro.z_axis = rx_data[3];

  ICM20648_read(ICM20648_REG_SELF_TEST_X_ACCEL, rx_data, 4);
  self_test->accel.x_axis = rx_data[1];
  self_test->accel.y_axis = rx_data[2];
  self_test->accel.z_axis = rx_data[3];
}


void ICM20648_reset( void )
{
  ICM20648_select_bank(ICM20648_USER_BANK_0);
  ICM20648_write( ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_H_RESET );
  NRFX_DELAY_US(100000);
}



//*********************************************************************************************
//*********************************************************************************************
//@read
void ICM20648_read( uint8_t address, uint8_t * rx_data, uint16_t length )
{
  uint8_t tx_data[2];

  memset(rx_data, 0, length);
  memset(tx_data, 0, sizeof(tx_data));

  tx_data[0] = address;
  mw_spi_master_transfer(m_ICM20648_SPI_ID, SPI_READ, tx_data, length, rx_data, length);
}


//@write
void ICM20648_write( uint8_t address, uint8_t value )
{
  uint8_t tx_data[2];
  uint8_t rx_data[2];
  memset(tx_data, 0, sizeof(tx_data));
  memset(rx_data, 0, sizeof(rx_data));


  tx_data[0] = address;
  tx_data[1] = value;
  mw_spi_master_transfer(m_ICM20648_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));
}

//@write
void ICM20648_write_two_bytes( uint8_t address, uint16_t value )
{
  uint8_t tx_data[3];
  uint8_t rx_data[2];
  memset(tx_data, 0, sizeof(tx_data));
  memset(rx_data, 0, sizeof(rx_data));

  tx_data[0] = address;
  tx_data[1] = value >> 8;
  tx_data[2] = value & 0xFF;
  mw_spi_master_transfer(m_ICM20648_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));
}


//@write
void ICM20648_write_bytes( uint8_t address, uint8_t * data, uint16_t length )
{
  uint8_t rx_data[2];
  memset(rx_data, 0, sizeof(rx_data));

  mw_spi_master_transfer(m_ICM20648_SPI_ID, SPI_WRITE, data, length, rx_data, sizeof(rx_data));
}


//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************


void ICM20648_initialize( external_device_config_t device_config )
{
	if( device_config.communication == SPI_COMMUNICATION )
	{
		spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

		if(return_value.err_code != NRF_SUCCESS) APP_ERROR_CHECK(return_value.err_code);

		m_ICM20648_SPI_ID = return_value.device_id;

		MW_LOG_INFO("ICM20648 device SPI Initialized");
	}

	if ( device_config.communication == TWI_COMMUNICATION )
	{
		mw_twi_master_init(device_config.twi_config);

		MW_LOG_INFO("ICM20648 device TWI Initialized");
	}

	m_icm20648_initialized = true;
}
