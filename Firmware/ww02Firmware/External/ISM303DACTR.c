
#include "nrfx_gpiote.h"

#include "mw_twi_master.h"
#include "project_settings.h"
#include "_mw_external_device.h"
#include "mw_logging.h"

#include "ISM303DACTR.h"

#define REGISTER_WRITE_VERIFY_ENABLE      1

static external_device_communication_t    m_ISM303DACTR_interface;
static external_driver_spi_config_t       m_ISM303DACTR_SPI;
static external_driver_twi_config_t       m_ISM303DACTR_XL_TWI;
static external_driver_twi_config_t       m_ISM303DACTR_MAG_TWI;

static external_driver_twi_config_t    *  m_current_TWI;

static bool m_ISM303DACTR_initialized = false;
static void ISM303DACTR_REGISTER_VERIFY( uint8_t address, uint8_t compare_value )
{
#if REGISTER_WRITE_VERIFY_ENABLE
  uint8_t rx_data;
  ism303dactr_register_read(address, &rx_data, 1);

  if( rx_data != compare_value )
  {
    while(1){}  // Error Caught
  }
#endif
}


/**
 * @brief - Convert a 2's compliment number to integer
 */
static int8_t convert_twos_compliment_to_int( uint8_t value )
{
  if( !( value & 0x80 ) ) return value;
  return (~(value - 1)) * -1;
}


/**
 * @brief - Reconfigure TWI for a different Device ID
 *
 * 1) Accelerometer: 0x1D
 * 2) Magnetometer: 0x1E
 */
static void ism303dacrt_switch_twi_devices( external_driver_twi_config_t * twi_config )
{
  if ( m_current_TWI->slave_address != twi_config->slave_address )
  {
    mw_twi_master_disable(m_current_TWI->twi_instance);
    mw_twi_master_change_slave_address(m_current_TWI->twi_instance, twi_config->slave_address);
    m_current_TWI = twi_config;
  }
}

void ism303dactr_mag_event_handler(mw_twi_evt_t const * p_event,
                           void *                 p_context)
{
    MW_LOG_INFO("Transfer completed on TWI0 ism303dac MAG.");
}


void ism303dactr_acc_event_handler(mw_twi_evt_t const * p_event,
                           void *                 p_context)
{
    MW_LOG_INFO("Transfer completed on TWI0 ism303dac ACC.");
}



/**
 * @brief - Converts raw accelerometer axis values to decimal G's
 */
ism303dac_accel_float_out_t ism303dactr_convert_raw_values_acc( ism303dac_accel_out_t * accel_values )
{
  uint8_t reg;
  ism303dac_accel_float_out_t accel_out;

  ism303dactr_register_read(ISM303DAC_CTRL1_A, &reg, 1);
  reg &= 0x0C;
  reg = reg >> 2;

  switch(reg)
  {
  case ACC_RANGE_2G:
    accel_out.x_axis = ACC_2G_CONVERSION(accel_values->x_axis);
    accel_out.y_axis = ACC_2G_CONVERSION(accel_values->y_axis);
    accel_out.z_axis = ACC_2G_CONVERSION(accel_values->z_axis);
    break;
  case ACC_RANGE_4G:
    accel_out.x_axis = ACC_4G_CONVERSION(accel_values->x_axis);
    accel_out.y_axis = ACC_4G_CONVERSION(accel_values->y_axis);
    accel_out.z_axis = ACC_4G_CONVERSION(accel_values->z_axis);
    break;
  case ACC_RANGE_8G:
    accel_out.x_axis = ACC_8G_CONVERSION(accel_values->x_axis);
    accel_out.y_axis = ACC_8G_CONVERSION(accel_values->y_axis);
    accel_out.z_axis = ACC_8G_CONVERSION(accel_values->z_axis);
    break;
  case ACC_RANGE_16G:
  default:
    accel_out.x_axis = ACC_16G_CONVERSION(accel_values->x_axis);
    accel_out.y_axis = ACC_16G_CONVERSION(accel_values->y_axis);
    accel_out.z_axis = ACC_16G_CONVERSION(accel_values->z_axis);
    break;
  }

  return accel_out;
}



/**
 * @brief - Returns the Acceleromter Range setting
 */
uint8_t ism303dactr_get_acc_range()
{
  ism303dac_ctrl1_a_t ctrl1_config;

  /* Get Current Range Setting */
  ism303dactr_register_read( ISM303DAC_CTRL1_A, (uint8_t*)&ctrl1_config, 1 );

  return ctrl1_config.fs;
}

ism303dac_accel_out_t ism303dactr_read_acc()
{
  uint8_t rx_data[6];
  ism303dac_accel_out_t accel_data;

  ism303dactr_register_read( ISM303DAC_OUT_X_L_A, rx_data, 6 );

  accel_data.x_axis = rx_data[1] << 8 | rx_data[0];
  accel_data.y_axis = rx_data[3] << 8 | rx_data[2];
  accel_data.z_axis = rx_data[5] << 8 | rx_data[4];
  return accel_data;
}


float ism303dactr_reading_temperature()
{
  uint8_t temperature= 0;
  ism303dactr_register_read( ISM303DAC_OUT_T_A, &temperature, 1 );
  return (float)( convert_twos_compliment_to_int(temperature)) + 25.0f;
}

/**
 * @brief - Set Sleep Bit
 */
void ism303dactr_set_sleep_bit( bool enable )
{
  uint8_t reg_value;

  ism303dactr_register_read( ISM303DAC_WAKE_UP_THS_A, &reg_value, 1);

  if ( enable )
  {
    /* Enable Sleep bit */
    reg_value |= 0x40;
  }
  else
  {
    /* Disable Sleep bit */
    reg_value &= 0xBF;
  }

  ism303dactr_register_write(ISM303DAC_WAKE_UP_THS_A, reg_value);
}


/**
 * @brief - Soft Reset of Device
 */
void ism303dactr_reset()
{
  ism303dactr_register_write(ISM303DAC_CTRL2_A, 0x44);
}

/**
 * @brief - Soft Reset of Device in progress?
 */
bool ism303dactr_reset_in_progress()
{
  uint8_t reg_value;
  ism303dactr_register_read(ISM303DAC_CTRL2_A, &reg_value, 1);
  return (reg_value >> 6);
}


void ism303dactr_who_am_i_test_mag()
{
  uint8_t who_am_i;
  ism303dactr_register_read( ISM303DAC_WHO_AM_I_M, &who_am_i, 1);
  ISM303DACTR_REGISTER_VERIFY( ISM303DAC_WHO_AM_I_M, ISM303DAC_ID_MG);
}



void ism303dactr_who_am_i_test_acc()
{
  uint8_t who_am_i;
  ism303dactr_register_read( ISM303DAC_WHO_AM_I_A, &who_am_i, 1 ) ;
  ISM303DACTR_REGISTER_VERIFY( ISM303DAC_WHO_AM_I_A, ISM303DAC_ID_XL );
}



/**
 * @brief - Register Read
 */
void ism303dactr_register_read( uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes )
{
  uint8_t tx_data[1];

  if ( m_ISM303DACTR_interface == TWI_COMMUNICATION )
  {
    if( register_address >= ISM303DAC_OFFSET_X_REG_L_M )
    {
      ism303dacrt_switch_twi_devices(&m_ISM303DACTR_MAG_TWI);
    }
    else
    {
      ism303dacrt_switch_twi_devices(&m_ISM303DACTR_XL_TWI);
    }
    tx_data[0] = register_address;
    mw_twi_master_enable(m_current_TWI->twi_instance);
    mw_twi_master_tx ( m_current_TWI->twi_instance,
                       m_current_TWI->slave_address,
                       tx_data,
                       sizeof(tx_data),
                       NO_STOP );

    mw_twi_master_rx( m_current_TWI->twi_instance,
                      m_current_TWI->slave_address,
                      destination,
                      number_of_bytes );
    mw_twi_master_disable(m_current_TWI->twi_instance);
  }
}


/**
 * @brief - Register Write
 */
void ism303dactr_register_write( uint8_t register_address, uint8_t value )
{
  uint8_t tx_data[2];
  tx_data[0] = register_address;
  tx_data[1] = value;

  if ( m_ISM303DACTR_interface == TWI_COMMUNICATION )
  {
    if ( register_address >= ISM303DAC_OFFSET_X_REG_L_M )
    {
      ism303dacrt_switch_twi_devices(&m_ISM303DACTR_MAG_TWI);
    }
    else
    {
      ism303dacrt_switch_twi_devices(&m_ISM303DACTR_XL_TWI);
    }

    mw_twi_master_enable(m_current_TWI->twi_instance);
    mw_twi_master_tx(m_current_TWI->twi_instance, m_current_TWI->slave_address, tx_data, sizeof(tx_data),
    STOP);
    mw_twi_master_disable(m_current_TWI->twi_instance);
  }
}




/**
 * @brief - Driver Initialization
 */
void ism303dactr_init( external_device_config_t device_config,  external_device_config_t device_config_mag )
{
  m_ISM303DACTR_interface = device_config.communication;
  if ( device_config.communication == SPI_COMMUNICATION )
  {
    spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

    if ( return_value.err_code != NRF_SUCCESS ) APP_ERROR_CHECK(return_value.err_code);

    m_ISM303DACTR_SPI = return_value.device_id;

    MW_LOG_INFO("TMP75C device SPI Initialized");
  }

  if ( device_config.communication == TWI_COMMUNICATION )
  {

    mw_twi_master_init(device_config_mag.twi_config);
    m_ISM303DACTR_MAG_TWI.twi_instance = device_config_mag.twi_config.instance;
    m_ISM303DACTR_MAG_TWI.slave_address = device_config_mag.twi_config.slave_addr;

    mw_twi_master_init(device_config.twi_config);
    m_ISM303DACTR_XL_TWI.twi_instance = device_config.twi_config.instance;
    m_ISM303DACTR_XL_TWI.slave_address = device_config.twi_config.slave_addr;
    m_current_TWI = &m_ISM303DACTR_XL_TWI;

    MW_LOG_INFO("ISM303DACTR device TWI Initialized");
  }

  m_ISM303DACTR_initialized = true;
}
