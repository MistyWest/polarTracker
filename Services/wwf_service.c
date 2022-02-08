/*
 * wwrf_service.c
 *
 *  Created on: Nov 5, 2019
 *      Author: klockwood
 *
 *
 *      This is a wrapper class for the Nordic ble_nus service
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrfx_gpiote.h"
#include "nrfx_timer.h"
#include "nrfx_clock.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "boards.h"
#include "ble_nus.h"

#include "mw_thread.h"
#include "mw_logging.h"
#include "project_settings.h"
#include "wwf_service.h"

#include "mw_rtc_thread.h"
#include "mw_data_manager_thread.h"

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                   /**< BLE NUS service instance. */

static uint16_t * m_conn_handle = NULL;

static wwf_external_command_handler_t m_external_handler = NULL;


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
//@nus
static void wwf_data_handler(ble_nus_evt_t * p_evt)
{
  if (p_evt->type == BLE_NUS_EVT_RX_DATA)
  {
    MW_LOG_INFO("Received data from BLE NUS");

    /* Pass to external handler if exists */
    if( m_external_handler != NULL )
    {
      m_external_handler( p_evt->params.rx_data.p_data,
                          p_evt->params.rx_data.length );
    }

    for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
    {
      MW_LOG_INFO( "0x%X", p_evt->params.rx_data.p_data[i]);
    }

    if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
    {
      MW_LOG_INFO("\r\n");
    }
  }
}


/**
 * @brief - Set external handler function
 */
void wwf_set_external_command_handler_t( wwf_external_command_handler_t handler )
{
  m_external_handler = handler;
}


/**
 * @brief - Send data over WWF Service
 */
void wwf_send_data( uint8_t * data, uint16_t length )
{
  uint32_t err_code;
  do
  {
    err_code = ble_nus_data_send(&m_nus, data, &length, *m_conn_handle);
    if ( err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
        err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING &&
        err_code != NRF_ERROR_INVALID_STATE &&
        err_code != NRF_ERROR_NOT_FOUND &&
        err_code != NRF_ERROR_RESOURCES &&
        err_code != NRF_SUCCESS )
    {
      APP_ERROR_CHECK(err_code);
    }

    if ( err_code == NRF_ERROR_RESOURCES )
    {
      MW_LOG_DEBUG( "Error = NRF_ERROR_RESOURCES");
      MW_LOG_DEBUG( "***** Send Buffer FULL!!!! ******");
      while ( 1 )
      {
      };
    }
  }
  while ( err_code == NRF_ERROR_RESOURCES );
}


void wwf_service_init( uint16_t * conn_handle )
{
  uint32_t err_code;
  ble_nus_init_t     nus_init;

  m_conn_handle = conn_handle;

  /**************************************************/
  // Initialize NUS.
  memset(&nus_init, 0, sizeof(nus_init));
  nus_init.data_handler = wwf_data_handler;

  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);
  /**************************************************/
}
