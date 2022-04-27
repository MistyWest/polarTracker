/*
 * TMP75C.c
 *
 *  Created on: Feb 28, 2019
 *      Author: klockwood
 */

#include <nrf_delay.h>

#include "TMP75C.h"
#include "../CLI_Logging/mw_logging.h"
#include "_mw_external_device.h"

static external_device_communication_t m_TMP75C_interface;

static bool                               m_TMP75C_initialized = false;

static external_driver_spi_config_t       m_TMP75C_SPI_ID;
static external_driver_twi_config_t       m_TMP75C_TWI;


void TMP75C_set_lower_limit( float limit, float scale_factor )
{
  uint8_t tx_data[3];
  uint16_t temp;
  temp = limit / scale_factor;

  tx_data[0] = TMP75C_LOWER_LIMIT_REG;
  tx_data[1] = temp >> 8;
  tx_data[2] = (temp << 4) & 0xF0;
  mw_twi_master_enable(m_TMP75C_TWI.twi_instance);
  mw_twi_master_tx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    STOP );

  mw_twi_master_disable(m_TMP75C_TWI.twi_instance);
}



void TMP75C_set_upper_limit( float limit, float scale_factor )
{
  uint8_t tx_data[3];
  uint16_t temp;
  temp = limit / scale_factor;

  tx_data[0] = TMP75C_UPPER_LIMIT_REG;
  tx_data[1] = temp >> 8;
  tx_data[2] = (temp << 4) & 0xF0;
  mw_twi_master_enable(m_TMP75C_TWI.twi_instance);
  mw_twi_master_tx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    STOP );

  mw_twi_master_disable(m_TMP75C_TWI.twi_instance);
}


void TMP75C_set_configure( uint8_t config  )
{
  uint8_t tx_data[3];

  tx_data[0] = TMP75C_CONFIGURATION_REG;
  tx_data[1] = config;
  tx_data[2] = 0x00;    // blank byte
  mw_twi_master_enable(m_TMP75C_TWI.twi_instance);
  mw_twi_master_tx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    STOP );

  mw_twi_master_disable(m_TMP75C_TWI.twi_instance);
}


uint8_t TMP75C_get_configure()
{
  uint8_t rx_data[2];
  uint8_t tx_data[1];
  memset(rx_data, 0 , sizeof(rx_data));
  memset(tx_data, 0 , sizeof(tx_data));

  //TMP75C_read( TMP75C_CONFIGURATION_REG, rx_data, 2);

  tx_data[0] = TMP75C_CONFIGURATION_REG;
  mw_twi_master_enable(m_TMP75C_TWI.twi_instance);
  mw_twi_master_tx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    NO_STOP );

  mw_twi_master_rx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    rx_data,
                    sizeof(rx_data) );

  mw_twi_master_disable(m_TMP75C_TWI.twi_instance);

  return rx_data[0];
}


void TMP75C_start_one_shot_measurement()
{
  uint8_t tx_data[3];

  tx_data[0] = TMP75C_ONE_SHOT_REG;
  tx_data[1] = 0x00; // random value to trigger measurement
  tx_data[2] = 0x04; // random value to trigger measurement
  mw_twi_master_enable(m_TMP75C_TWI.twi_instance);
  mw_twi_master_tx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    STOP );

  mw_twi_master_disable(m_TMP75C_TWI.twi_instance);
}


float TMP75C_read_temperature( float conversion_factor )
{
  int16_t temp;
  uint8_t tx_data[1];
  uint8_t rx_data[2];

  tx_data[0] = TMP75C_TEMPERATURE_REG;
  mw_twi_master_enable(m_TMP75C_TWI.twi_instance);
  mw_twi_master_tx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    NO_STOP );

  mw_twi_master_rx( m_TMP75C_TWI.twi_instance,
                    m_TMP75C_TWI.slave_address,
                    rx_data,
                    sizeof(rx_data) );

  mw_twi_master_disable(m_TMP75C_TWI.twi_instance);

  /** Test values for -55C
  rx_data[0] = 0xC9;
  rx_data[1] = 0x00;*/

  temp = (int8_t)rx_data[0];
  return (float)( temp + ( ( (rx_data[1]) >> 4) * conversion_factor ) );
}


//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************
//*********************************************************************************************

/**
 * @brief TMP75C Driver Initialization
 *
 * Initialzation parameter contains all interface configurations
 */
void TMP75C_initialize( external_device_config_t device_config )
{
  m_TMP75C_interface = device_config.communication;
  if ( device_config.communication == SPI_COMMUNICATION )
  {
    spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

    if ( return_value.err_code != NRF_SUCCESS )
      APP_ERROR_CHECK(return_value.err_code);

    m_TMP75C_SPI_ID = return_value.device_id;

    MW_LOG_INFO("TMP75C device SPI Initialized");
  }

  if ( device_config.communication == TWI_COMMUNICATION )
  {
    mw_twi_master_init(device_config.twi_config);

    m_TMP75C_TWI.twi_instance  = device_config.twi_config.instance;
    m_TMP75C_TWI.slave_address = device_config.twi_config.slave_addr;

    MW_LOG_INFO("TMP75C device TWI Initialized");
  }

  m_TMP75C_initialized = true;
}
