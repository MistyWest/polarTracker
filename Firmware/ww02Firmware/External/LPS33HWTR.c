
#include "nrfx_gpiote.h"
#include "mw_twi_master.h"

#include "project_settings.h"
#include "_mw_external_device.h"
#include "mw_logging.h"
#include "LPS33HWTR.h"

#define REGISTER_WRITE_VERIFY_ENABLE      1

static external_device_communication_t    m_LPS33HWTR_interface;
static external_driver_spi_config_t       m_LPS33HWTR_SPI;
static external_driver_twi_config_t       m_LPS33HWTR_TWI;



static void LPS33HWTR_REGISTER_VERIFY( uint8_t address, uint8_t compare_value )
{
#if REGISTER_WRITE_VERIFY_ENABLE
  uint8_t rx_data;
  lps33hwtr_register_read(address, &rx_data, 1);

  if( rx_data != compare_value )
  {
    while(1){}  // Error Caught
  }
#endif
}



void lps33hw_event_handler(mw_twi_evt_t const * p_event,
                           void *                 p_context)
{
    MW_LOG_INFO("Transfer completed on TWI0 lps33hw.");


}


float mw_lps33hw_from_lsb_to_hpa(int32_t lsb)
{
  return ( (float)lsb / 4096.0f );

}


/**
 * @brief - Enable/Disable Low Power Mode
 */
void lps33hwtr_enable_low_power_mode( bool enable )
{
  uint8_t reg_value;
  lps33hwtr_register_read(LPS33HW_RES_CONF, &reg_value, 1);

  reg_value &= 0xFE; //Mask relevant bits

  reg_value |= enable;

  lps33hwtr_register_write( LPS33HW_RES_CONF, reg_value);
}


/**
 * @brief - Enable/Disable I2C Interface
 */
void lps33hwtr_enable_i2c( bool i2c )
{
  uint8_t reg_value;
  lps33hwtr_register_read(LPS33HW_CTRL_REG2, &reg_value, 1);

  reg_value &= 0xF7; //Mask relevant bits

  reg_value |= (i2c << 3);

  lps33hwtr_register_write( LPS33HW_CTRL_REG2, reg_value);
}




/**
 * @brief - Set ODR
 */
void lps33hwtr_set_odr( uint8_t odr )
{
  uint8_t reg_value;
  lps33hwtr_register_read(LPS33HW_CTRL_REG1, &reg_value, 1);

  reg_value &= 0x0F; //Mask relevant bits
  odr &= 0x07;  //Mask relevant bits

  reg_value |= odr;

  lps33hwtr_register_write( LPS33HW_CTRL_REG1, reg_value);
}



/**
 * @brief - Device ID verification
 */
bool lps33hwtr_who_am_i_test()
{
  uint8_t whoamI = 0;
  lps33hwtr_register_read(LPS33HW_WHO_AM_I, &whoamI, 1);
  if ( whoamI != LPS33HW_ID )
  {
    LPS33HWTR_REGISTER_VERIFY(LPS33HW_WHO_AM_I, LPS33HW_ID);
    return false; /*manage here device not found */
  }
  return true;
}




/**
 * @brief - Register Read
 */
void lps33hwtr_register_read( uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes )
{
  uint8_t tx_data[1];

  if ( m_LPS33HWTR_interface == TWI_COMMUNICATION )
  {
    tx_data[0] = register_address;
    mw_twi_master_enable(m_LPS33HWTR_TWI.twi_instance);
    mw_twi_master_tx ( m_LPS33HWTR_TWI.twi_instance,
                       m_LPS33HWTR_TWI.slave_address,
                       tx_data,
                       sizeof(tx_data),
                       NO_STOP );

    mw_twi_master_rx( m_LPS33HWTR_TWI.twi_instance,
                      m_LPS33HWTR_TWI.slave_address,
                      destination,
                      number_of_bytes );
    mw_twi_master_disable(m_LPS33HWTR_TWI.twi_instance);
  }
}


/**
 * @brief - Register Write
 */
void lps33hwtr_register_write( uint8_t register_address, uint8_t value )
{
  uint8_t tx_data[2];
  tx_data[0] = register_address;
  tx_data[1] = value;

  if ( m_LPS33HWTR_interface == TWI_COMMUNICATION )
  {
    mw_twi_master_enable(m_LPS33HWTR_TWI.twi_instance);
    mw_twi_master_tx(m_LPS33HWTR_TWI.twi_instance, m_LPS33HWTR_TWI.slave_address, tx_data, sizeof(tx_data),
    STOP);
    mw_twi_master_disable(m_LPS33HWTR_TWI.twi_instance);
  }
}




/**
 * @brief - Driver Initialization
 */
void lps33hwtr_init( external_device_config_t device_config )
{
  m_LPS33HWTR_interface = device_config.communication;
  if ( device_config.communication == SPI_COMMUNICATION )
  {
    spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

    if ( return_value.err_code != NRF_SUCCESS ) APP_ERROR_CHECK(return_value.err_code);

    m_LPS33HWTR_SPI = return_value.device_id;

    MW_LOG_INFO("TMP75C device SPI Initialized");
  }

  if ( device_config.communication == TWI_COMMUNICATION )
  {
    mw_twi_master_init(device_config.twi_config);
    m_LPS33HWTR_TWI.twi_instance = device_config.twi_config.instance;
    m_LPS33HWTR_TWI.slave_address = device_config.twi_config.slave_addr;
    MW_LOG_INFO("lps33hwtr device TWI Initialized");
  }
}






