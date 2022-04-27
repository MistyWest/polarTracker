/*
 * MPU9250.c
 *
 *  Created on: Feb 14, 2019
 *      Author: klockwood
 */

#include "MPU9250.h"
#include "../CLI_Logging/mw_logging.h"
#include "_mw_external_device.h"

#define TWI0_SENSORSoff
#define TWI1_SENSORS
#define TEST_MODE

#define GPIOTE_CONFIGoff

static external_device_communication_t m_driver_interface;
//static uint32_t m_sensor_interrupt_pin;

static bool			m_mpu9250_initialized = false;

static external_driver_spi_config_t 	m_MPU9250_SPI_ID;
static external_driver_twi_config_t		m_MPU9250_TWI;

/* Indicates if reading operation from accelerometer has ended. */
//static uint8_t 							twi_buffer[15];

//static void MPU9250_write( uint8_t address, uint8_t value );
static void MPU9250_read( uint8_t address, uint8_t * rx_data, uint8_t length );


//static void check_accel_value();

//static uint32_t read_sensor_data_proskida();
//static uint32_t read_accel_data_proskida();
/**
 * @brief TWI events handler.
 */
/*
static void m_twim_callback( nrfx_twim_evt_t const * twi_event )
{
    switch(twi_event->type)
    {
        case NRFX_TWI_EVT_DONE:
            break;

        case NRFX_TWI_EVT_ADDRESS_NACK:
        	//MW_LOG("TWI: Address NACK\r\n", NULL, SENSOR_EVENT);
        	break;

        case NRFX_TWI_EVT_DATA_NACK:
        	//MW_LOG("TWI: Data NACK\r\n", NULL, SENSOR_EVENT);
        	break;

       default:
    	  // MW_LOG("TWI: Unknown Error\r\n", NULL, SENSOR_EVENT);
            break;
    }
}*/


/*
uint32_t sensor_go_to_sleep()
{
	twi_buffer[0] = PWR_MGMT_1;
	twi_buffer[1] = 0x40;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}


uint32_t reset_accelerometer()
{
	twi_buffer[0] = PWR_MGMT_1;
	twi_buffer[1] = ACCEL_RESET;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t start_up_1_accelerometer()
{
	twi_buffer[0] = PWR_MGMT_1;
	twi_buffer[1] = PWR_MGMT_1_LP;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t enable_cycle_mode_accelerometer()
{
	twi_buffer[0] = PWR_MGMT_1;
	twi_buffer[1] = ENABLE_CYCLE_MODE;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t disable_cycle_mode_accelerometer()
{
	twi_buffer[0] = PWR_MGMT_1;
	twi_buffer[1] = ENABLE_CYCLE_MODE;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}


uint32_t start_up_2_accelerometer()
{
	twi_buffer[0] = PWR_MGMT_2;
	twi_buffer[1] = PWR_MGMT_2_DEFAULT;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t start_up_2_lp_accelerometer()
{
	twi_buffer[0] = PWR_MGMT_2;
	twi_buffer[1] = PWR_MGMT_2_LP;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}


uint32_t default_config_1_sensors_proskida()
{
	twi_buffer[0] = SMPLRT_DIV;
	twi_buffer[1] = 0x04;									//Register 25	Sample Rate Divider - 100Hz
	twi_buffer[2] = 0x00;//0x07;							//Register 26	Configuration
	twi_buffer[3] = SENSOR_GYRO_RANGE | USER_LPF_FILTER;	//Register 27	Gyro Configuration
	twi_buffer[4] = SENSOR_ACCEL_RANGE;						//Register 28	Accel Configuration 1
	twi_buffer[5] = ACCEL_FCHOICE_CLEAR | ACCEL_DLPFCFG_218HZ;		//Register 29	Accel Configuration 2
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 6, TWI_ISSUE_STOP );

}

uint32_t default_config_2_sensors_proskida()
{
	twi_buffer[0] = LP_ACCEL_ODR;
	twi_buffer[1] = ACCEL_LP_ODR_250HZ;//ACCEL_LP_ODR_125HZ;						//Register 30	Accel ODR - Supply Current in uA = 8 uA + ODR*0.376
	twi_buffer[2] = 0x00;									//Register 31	Wake On Motion Threshold
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 3, TWI_ISSUE_STOP );
}

uint32_t default_config_3_sensors_proskida()
{
	twi_buffer[0] = SIGNAL_PATH_RESET;	//Register 104 - Reset digital paths
	twi_buffer[1] = 0x07;				//Register 104 - Reset Accel, Gyro and Temp
	twi_buffer[2] = 0x00;				//Register 105 - Disable Accel Interrupts
	twi_buffer[3] = 0x00;				//Register 106 - Not Applicable
	twi_buffer[4] = 0x01;				//Register 107 - Power Management 1 - Don't touch
	twi_buffer[5] = 0x00;				//Register 108 - Power Management 2 - Enable Accel,  Enable Gyro
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 6, TWI_ISSUE_STOP );
}

uint32_t default_set_accel_range()
{
	twi_buffer[0] = ACCEL_CONFIG;							//Register 28	Accel Configuration 1
	twi_buffer[1] = SENSOR_ACCEL_RANGE;						//Register 28	Accel Configuration 1
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}


uint32_t enable_sensors_proskida()
{
	twi_buffer[0] = SIGNAL_PATH_RESET;
	twi_buffer[1] = 0x07;		//Register 104 - Reset Signal Path
	twi_buffer[2] = 0x00;		//Register 104 - Disable Interrupts
	twi_buffer[3] = 0x00;		//Register 105 - Not Applicable
	twi_buffer[4] = 0x00;		//Register 106 - Power Management 1 - Don't touch
	twi_buffer[5] = 0x00;		//Register 107 - Power Management 2 - Enable Gyro and Accel
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 6, TWI_ISSUE_STOP );
}


uint32_t lp_setup_sensors_proskida()
{
	twi_buffer[0] = SIGNAL_PATH_RESET;	//Register 104 - Reset digital paths
	twi_buffer[1] = 0x07;				//Register 104 - Reset Accel, Gyro and Temp
	twi_buffer[2] = 0x00;				//Register 105 - Disable Accel Interrupts
	twi_buffer[3] = 0x00;				//Register 106 - Not Applicable
	twi_buffer[4] = 0x00;				//Register 107 - Power Management 1 - Don't touch
	twi_buffer[5] = DISABLE_GYRO_XYZ;	//Register 108 - Power Management 2 - Enable Accel,  Disable Gyro
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 6, TWI_ISSUE_STOP );
}



uint32_t lp_config_1_sensors_proskida()
{
	twi_buffer[0] = SMPLRT_DIV;
	twi_buffer[1] = 0x5A;									//Register 25	Sample Rate Divider - 100Hz
	twi_buffer[2] = 0x00;									//Register 26	Configuration
	twi_buffer[3] = SENSOR_GYRO_RANGE | USER_LPF_FILTER;	//Register 27	Gyro Configuration
	twi_buffer[4] = SENSOR_ACCEL_RANGE;						//Register 28	Accel Configuration 1
	twi_buffer[5] = ACCEL_FCHOICE_SET | ACCEL_DLPFCFG_10HZ;	//Register 29	Accel Configuration 2
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 6, TWI_ISSUE_STOP );
}


uint32_t lp_config_2_sensors_proskida()
{
	twi_buffer[0] = LP_ACCEL_ODR;
	twi_buffer[1] = ACCEL_LP_ODR_1HZ;						//Register 30	Accel ODR - Supply Current in uA = 8 uA + ODR*0.376
	twi_buffer[2] = 0x00;									//Register 31	Wake On Motion Threshold
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 3, TWI_ISSUE_STOP );
}

uint32_t lp_config_3_sensors_proskida()
{
	twi_buffer[0] = SIGNAL_PATH_RESET;	//Register 104 - Reset digital paths
	twi_buffer[1] = 0x07;				//Register 104 - Reset Accel, Gyro and Temp
	twi_buffer[2] = 0x00;				//Register 105 - Disable Accel Interrupts
	twi_buffer[3] = 0x00;				//Register 106 - Not Applicable
	twi_buffer[4] = 0x01;				//Register 107 - Power Management 1 - Don't touch
	twi_buffer[5] = 0x07;				//Register 108 - Power Management 2 - Enable Accel,  Enable Gyro
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 6, TWI_ISSUE_STOP );
}


uint32_t wake_up_config_1_sensors_proskida()
{
	twi_buffer[0] = SMPLRT_DIV;
	twi_buffer[1] = 0x5A;									//Register 25	Sample Rate Divider - 100Hz
	twi_buffer[2] = 0x00;									//Register 26	Configuration
	twi_buffer[3] = SENSOR_GYRO_RANGE | USER_LPF_FILTER;	//Register 27	Gyro Configuration
	twi_buffer[4] = SENSOR_ACCEL_RANGE;						//Register 28	Accel Configuration 1
	twi_buffer[5] = ACCEL_FCHOICE_SET | ACCEL_DLPFCFG_218HZ;		//Register 29	Accel Configuration 2
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 6, TWI_ISSUE_STOP );
}

uint32_t wake_up_config_2_sensors_proskida()
{
	twi_buffer[0] = LP_ACCEL_ODR;
	twi_buffer[1] = ACCEL_LP_ODR_125HZ;						//Register 30	Accel ODR - Supply Current in uA = 8 uA + ODR*0.376
	twi_buffer[2] = 0x00;									//Register 31	Wake On Motion Threshold
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 3, TWI_ISSUE_STOP );
}

uint32_t low_power_odr_accelerometer_proskida()
{
	twi_buffer[0] = LP_ACCEL_ODR;
	twi_buffer[1] = ACCEL_LP_ODR_125HZ;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}


uint32_t set_lpf_and_bw_accelerometer_proskida()
{
	twi_buffer[0] = ACCEL_CONFIG_2;
	//twi_buffer[1] = ACCEL_LPF_AND_BW_99HZ;
	twi_buffer[1] = ACCEL_LPF_AND_BW_10HZ;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}


uint32_t wake_on_motion_accelerometer_proskida()
{
	twi_buffer[0] = WOM_THR;
	twi_buffer[1] = WAKE_THESHOLD_D; // WAKE_THESHOLD_A WAKE_THESHOLD_B WAKE_THESHOLD_D
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}


uint32_t config_interrupts_proskida()
{
	twi_buffer[0] = INT_PIN_CFG;
	twi_buffer[1] = 0x08; // 0x30 0x00
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t unconfig_interrupts_proskida()
{
	twi_buffer[0] = INT_PIN_CFG;
	twi_buffer[1] = 0xC0; // 0x30 0x00
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t enable_interrupts_proskida()
{
	twi_buffer[0] = INT_ENABLE;
	twi_buffer[1] = WAKE_ON_MOTION_INT;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t disable_interrupts_proskida()
{
	twi_buffer[0] = INT_ENABLE;
	twi_buffer[1] = DISABLE_INTERRUPTS;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t read_interrupt_status_proskida()
{
	twi_buffer[0] = INT_STATUS;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}

uint32_t read_power_managment_2()
{
	twi_buffer[0] = PWR_MGMT_2;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}

uint32_t enable_intelligence_accelerometer()
{
	twi_buffer[0] = MOT_DETECT_CTRL;
	twi_buffer[1] = ENABLE_INTELLIGENCE_MODE;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t disable_intelligence_accelerometer()
{
	twi_buffer[0] = MOT_DETECT_CTRL;
	twi_buffer[1] = DISABLE_INTELLIGENCE_MODE;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP );
}

uint32_t get_accel_bias_values()
{
	twi_buffer[0] = XA_OFFSET_H;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_ISSUE_STOP );
}

//@sensor @data
uint32_t request_sensor_data()
{
	return read_sensor_data_proskida();
}

static uint32_t read_sensor_data_proskida()
{
	twi_buffer[0] = ACCEL_XOUT_H;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, 0); //TWI_DONT_STOP );

	// 6-6-2018 #PHIL - debug statements for reading back other config register data if necessary
//	twi_buffer[0] = SMPLRT_DIV;
//	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, 0);
}

//@accel
uint32_t request_accel_data()
{
	return read_accel_data_proskida();
}

static uint32_t read_accel_data_proskida()
{
	twi_buffer[0] = ACCEL_XOUT_H;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, 0); //TWI_DONT_STOP );
}


static uint32_t read_accel_x_axis_l_status_proskida()
{
	twi_buffer[0] = ACCEL_XOUT_L;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}


static uint32_t read_accel_y_axis_h_status_proskida()
{
	twi_buffer[0] = ACCEL_YOUT_H;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}


static uint32_t read_accel_y_axis_l_status_proskida()
{
	twi_buffer[0] = ACCEL_YOUT_L;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}

static uint32_t read_accel_z_axis_h_status_proskida()
{
	twi_buffer[0] = ACCEL_ZOUT_H;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}

static uint32_t read_accel_z_axis_l_status_proskida()
{
	twi_buffer[0] = ACCEL_ZOUT_L;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}
*/


//@gyro
/*
uint32_t request_gyro_data()
{
	twi_buffer[0] = GYRO_XOUT_H;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, 0); //TWI_DONT_STOP );
}


//@temp
uint32_t request_temperature_data()
{
	twi_buffer[0] = TEMP_OUT_H;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, 0 ); //TWI_DONT_STOP );
}



uint32_t start_accel_self_test()
{
	twi_buffer[0] = SELF_TEST_X_ACCEL;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 1, TWI_DONT_STOP );
}

uint32_t end_accel_self_test()
{
	twi_buffer[0] = ACCEL_CONFIG;
	twi_buffer[1] = SENSOR_ACCEL_RANGE;
	return mw_write_twi( MPU9250_ADDRESS, twi_buffer, 2, TWI_ISSUE_STOP ); //Restore Config
}*/

//Super Hack for catching when the MPU9250 randomly seems to switch Range/Scaling
//this catches the and sets this.
/*static void check_accel_value( mpu9250_sensor_t accel_data )
{
	float resultant = 	( accel_data.x_axis * accel_data.x_axis ) +
						( accel_data.y_axis * accel_data.y_axis ) +
						( accel_data.z_axis * accel_data.z_axis );

	resultant = resultant * get_accel_scale_factor(SENSOR_ACCEL_RANGE) * get_accel_scale_factor(SENSOR_ACCEL_RANGE);

	if( resultant < RESULTANT_THRESHOLD )
	{
		accel_data.x_axis = accel_data.x_axis * 4;
		accel_data.x_axis = accel_data.x_axis * 4;
		accel_data.x_axis = accel_data.x_axis * 4;
	}
}*/


float get_accel_scale_factor( uint8_t accel_scale )
{
	switch(accel_scale)
	{
	case ACCEL_SCALE_4G:
		return  ( 4.0 / SENSOR_RES );//GRAVITY;
		break;

	case ACCEL_SCALE_8G:
		return  ( 8.0 / SENSOR_RES );//GRAVITY;
		break;

	case ACCEL_SCALE_16G:
		return  ( 16.0 / SENSOR_RES );//GRAVITY;
		break;

	default:
	case ACCEL_SCALE_2G:
		return  ( 2.0 / SENSOR_RES );//GRAVITY;
		break;
	}
}

float get_gyro_scale_factor( uint8_t gyro_scale )
{
	switch(gyro_scale)
	{
	case GYRO_SCALE_500dps:
		return  500.0 / SENSOR_RES / DEG2RAD;
		break;

	case GYRO_SCALE_1000dps:
		return  1000.0 / SENSOR_RES / DEG2RAD;
		break;

	case GYRO_SCALE_2000dps:
		return  2000.0 / SENSOR_RES / DEG2RAD;
		break;

	default:
	case GYRO_SCALE_250dps:
		return  250.0 / SENSOR_RES / DEG2RAD;
		break;
	}
}


uint32_t MPU9250_who_am_i()
{
	uint8_t chip_id[1];
	MPU9250_read( WHO_AM_I, chip_id, sizeof(chip_id) );

	MW_LOG_INFO("MPU9250 Chip ID: 0x%X", chip_id[0])
	return 0;
}

//*********************************************************************************************
//*********************************************************************************************

static void MPU9250_read( uint8_t address, uint8_t * rx_data, uint8_t length )
{
	static uint8_t tx_data[28];

	memset(rx_data, 0, length);
	memset(tx_data, 0, sizeof(tx_data));

	tx_data[0] = address;

  if ( m_driver_interface == TWI_COMMUNICATION )
  {
    mw_twi_master_enable(m_MPU9250_TWI.twi_instance);
    mw_twi_master_tx( m_MPU9250_TWI.twi_instance,
                      m_MPU9250_TWI.slave_address,
                      tx_data,
                      sizeof(uint8_t),
                      NO_STOP );

    mw_twi_master_rx( m_MPU9250_TWI.twi_instance,
                      m_MPU9250_TWI.slave_address,
                      rx_data,
                      length );

    mw_twi_master_disable(m_MPU9250_TWI.twi_instance);
  }
}

/*static void MPU9250_write( uint8_t address, uint8_t value )
{
  static uint8_t tx_data[2];
  memset(rx_data, 0, length);
  memset(tx_data, 0, sizeof(tx_data));

  tx_data[0] = address;
  tx_data[1] = value;

  if ( m_driver_interface == TWI_COMMUNICATION )
  {
    mw_twi_master_enable(m_MPU9250_TWI.twi_instance);
    mw_twi_master_tx( m_MPU9250_TWI.twi_instance,
                      m_MPU9250_TWI.slave_address,
                      tx_data,
                      sizeof(uint8_t),
                      STOP );

    mw_twi_master_disable(m_MPU9250_TWI.twi_instance);
  }
}*/

//*********************************************************************************************
//*********************************************************************************************



void MPU9250_initialize( external_device_config_t device_config )
{
  m_driver_interface = device_config.communication;

	if( device_config.communication == SPI_COMMUNICATION )
	{
		spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

		if(return_value.err_code != NRF_SUCCESS) APP_ERROR_CHECK(return_value.err_code);

		m_MPU9250_SPI_ID = return_value.device_id;

		MW_LOG_INFO("ICM20649_device SPI Initialized");
	}

	if ( device_config.communication == TWI_COMMUNICATION )
	{
		mw_twi_master_init(device_config.twi_config);

		/*mw_twi_config_t twi_config;

		twi_config.frequency = device_config.twi_config.frequency;
		twi_config.hold_bus_uninit = device_config.twi_config.hold_bus_uninit;
		twi_config.interrupt_priority = device_config.twi_config.interrupt_priority;
		twi_config.scl = device_config.twi_config.scl;
		twi_config.sda = device_config.twi_config.sda;

		mw_twi_init(device_config.twi_config.instance,
								device_config.twi_config.slave_addr,
								&twi_config,
		            NULL,
		            NULL);*/



		m_MPU9250_TWI.twi_instance 	= device_config.twi_config.instance;
		m_MPU9250_TWI.slave_address = device_config.twi_config.slave_addr;

		MW_LOG_INFO("ICM20649_device TWI Initialized");
	}

	m_mpu9250_initialized = true;
}
