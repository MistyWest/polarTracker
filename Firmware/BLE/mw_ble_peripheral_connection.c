/*
 * mw_ble_peripheral_connection.c
 *
 *  Created on: April 19, 2017
 *      Author: KLockwood
 */

/** @file
 *
 * @defgroup -
 * @{
 * @ingroup -
 * @brief MW Connected State Machine
 *
 */

#include "sdk_common.h"
#include "sdk_config.h"
#include "nrf.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_soc.h"
#include "app_util.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

// NRF Logging Includes
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "mw_ble_settings.h"
#include "mw_ble_peripheral_connection.h"

MW_BLE_CONNECTED_DEF(m_connected);
NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */

mw_ble_peripheral_connection_t * mw_ble_peripheral_connection_obj = &m_connected; /**< Point to Module Object, only used because I like the look of mw_ble_connected_obj->xxx.yy  VS mw_ble_connected_obj.xxx.yyy */

mw_peripheral_connection_update_handler_t m_evt_handler;

#define CONVERT_INTERVAL_UNITS_TO_MSEC( x )  x = x * 125 / 100

static void mw_connection_parameter_manager( const ble_gap_conn_params_t param_update );
static void mw_update_parameter_obj( ble_gap_conn_params_t conn_params, mw_connection_parameter_mode_t mode );
static void mw_set_gatt_data_length( uint8_t value );
static void mw_set_gatt_mtu( uint16_t att_mtu );
static void mw_set_connection_event_length_extention( bool status );
static void auto_update_ble_phy();

/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
//@ble
void mw_ble_connected_on_ble_evt( ble_evt_t const * p_ble_evt,  void * p_context )
{

  switch ( p_ble_evt->header.evt_id )
  {
  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO( "PERIPHERAL CONNECTION MODULE - Connection Interval %d", p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval*5/4);
    mw_ble_peripheral_connection_obj->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    mw_connection_parameter_manager(p_ble_evt->evt.gap_evt.params.connected.conn_params);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    mw_ble_peripheral_connection_obj->conn_handle = BLE_CONN_HANDLE_INVALID;
    mw_ble_peripheral_connection_obj->conn_params.mode = DEFAULT_CONN_PARAMS;
    break;

#if !S112
  case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    break;
#endif

  case BLE_GAP_EVT_CONN_PARAM_UPDATE:
    mw_connection_parameter_manager(p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params);
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
#if !S112
    NRF_LOG_INFO("PERIPHERAL CONNECTION MODULE - Conn Param Update request, Max Interval: %d", p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.max_conn_interval);
    NRF_LOG_INFO("PERIPHERAL CONNECTION MODULE - Conn Param Update request, Min Interval: %d", p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.min_conn_interval);
#endif
    break;


  case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    auto_update_ble_phy();
    break;

  case BLE_GAP_EVT_PHY_UPDATE:
    mw_ble_peripheral_connection_obj->ble_data_params.phys.tx_phys = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
    mw_ble_peripheral_connection_obj->ble_data_params.phys.rx_phys = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;
    break;

  //******************************************
  //Handled in GATT module callback function
#if !S112
  case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
    break;
  case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    break;
#endif
  //******************************************

  default:
    // No implementation needed.
    break;
  }
}



/**@brief Function for handling events from the GATT library. */
//@gatt
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  switch (p_evt->evt_id)
  {
    case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
      mw_ble_peripheral_connection_obj->ble_data_params.att_mtu =  p_evt->params.att_mtu_effective;
      NRF_LOG_INFO("PERIPHERAL CONNECTION MODULE - ATT MTU exchange completed. MTU set to %u bytes.", p_evt->params.att_mtu_effective);
      m_evt_handler(CONNECTION_MTU_UPDATE, mw_ble_peripheral_connection_obj);
      break;

    case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
#if !S112
      mw_ble_peripheral_connection_obj->ble_data_params.data_len = p_evt->params.data_length;
      NRF_LOG_INFO("PERIPHERAL CONNECTION MODULE - Data length updated to %u bytes.", p_evt->params.data_length);
      m_evt_handler(CONNECTION_DATA_LENGTH_UPDATE, mw_ble_peripheral_connection_obj);
#endif
      break;
  }
}



/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the module.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
void mw_on_conn_params_evt( ble_conn_params_evt_t * p_evt )
{
  uint32_t err_code;

  if ( p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED )
  {
    // err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    err_code = NRF_SUCCESS; //send_fast_iphone_mode_request(p_evt);
    if ( (err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY) )
    {
      APP_ERROR_HANDLER(err_code);
    }
  }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void mw_conn_params_error_handler( uint32_t nrf_error )
{
  if ( (nrf_error != NRF_SUCCESS) && (nrf_error != NRF_ERROR_BUSY) )
  {
    APP_ERROR_HANDLER(nrf_error);
  }
}


/**@brief Function for updating the module with the current connection parameters and mode
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void mw_update_parameter_obj( ble_gap_conn_params_t conn_params, mw_connection_parameter_mode_t mode )
{
  mw_ble_peripheral_connection_obj->conn_params.mode = mode;
  mw_ble_peripheral_connection_obj->conn_params.params.min_conn_interval  = conn_params.min_conn_interval;
  mw_ble_peripheral_connection_obj->conn_params.params.max_conn_interval  = conn_params.max_conn_interval;
  mw_ble_peripheral_connection_obj->conn_params.params.slave_latency      = conn_params.slave_latency;
  mw_ble_peripheral_connection_obj->conn_params.params.conn_sup_timeout   = conn_params.conn_sup_timeout;
}



/**@brief Function for requesting a Phy Updates */
static void auto_update_ble_phy()
{
  uint32_t err_code;
  NRF_LOG_DEBUG("PHY update request.");
  ble_gap_phys_t const phys =
  {
      .rx_phys = BLE_GAP_PHY_AUTO,
      .tx_phys = BLE_GAP_PHY_AUTO,
    };
  err_code = sd_ble_gap_phy_update(mw_ble_peripheral_connection_obj->conn_handle, &phys);
  APP_ERROR_CHECK(err_code);
}


/**@brief   Function to get current Connection Parameters
 *
 */
uint16_t mw_get_connection_interval()
{
  return mw_ble_peripheral_connection_obj->conn_params.params.min_conn_interval;
}


void mw_set_connected_power( int8_t tx_power )
{
  APP_ERROR_CHECK(sd_ble_gap_tx_power_set(  BLE_GAP_TX_POWER_ROLE_CONN,
                                            mw_ble_peripheral_connection_obj->conn_handle,
                                            mw_ble_peripheral_connection_obj->connected_power));
  NRF_LOG_DEBUG("PERIPHERAL CONNECTION MODULE - Radio Power Updated to: %d", tx_power);
}


static void mw_connection_parameter_manager( const ble_gap_conn_params_t param_update )
{
  if( check_conn_param_update( param_update, mw_ble_peripheral_connection_obj->conn_params.mode ) )
  {
    switch( mw_ble_peripheral_connection_obj->conn_params.mode )
    {
    case FAST_IPHONE_CONN_PARAMS_REQUEST:
      mw_ble_peripheral_connection_obj->conn_params.mode = FAST_IPHONE_CONN_PARAMS;
      break;

    case FAST_CONN_PARAMS_REQUEST:
      mw_ble_peripheral_connection_obj->conn_params.mode = FAST_CONN_PARAMS;
      break;

    case SLOW_CONN_PARAMS_REQUEST:
      mw_ble_peripheral_connection_obj->conn_params.mode = SLOW_CONN_PARAMS;
      break;

    default:
      break;
    }
  }

  mw_update_parameter_obj( param_update, mw_ble_peripheral_connection_obj->conn_params.mode); //  Update Module Obj storing connection parameters
  m_evt_handler(CONNECTION_INTERVAL_UPDATE, mw_ble_peripheral_connection_obj);  //  Update callback function
}


/**@brief Update Connection Parameter Request
 *
 * @details This function will be called to send an Updated Connection Parameter Request
 *
 * @param[in]   p_ble_evt   BLE event variable struct
 * @param[in]   update    Update Type
 */
void mw_update_connection_parameters( uint16_t conn_handle, mw_connection_parameter_mode_t update )
{
  if ( mw_ble_peripheral_connection_obj->conn_params.mode == FAST_CONN_PARAMS_REQUEST )
  {
    send_fast_mode_request(conn_handle);
  } else if ( mw_ble_peripheral_connection_obj->conn_params.mode == FAST_IPHONE_CONN_PARAMS_REQUEST )
  {
    send_fast_iphone_mode_request(conn_handle);
  }
  else if( mw_ble_peripheral_connection_obj->conn_params.mode == SLOW_CONN_PARAMS_REQUEST )
  {
    send_slow_mode_request(conn_handle);
  }
}

/**@brief Send iPhone Fast Mode Connection Parameter Request
 *
 * @details This function will be called to send the Fast Mode Connection Parameter Request
 *
 * @param[in]   conn_handle  Connection Handler to send the request to
 */
uint32_t send_fast_iphone_mode_request( const uint16_t conn_handle )
{
  uint32_t err_code = NRF_SUCCESS;
  ble_gap_conn_params_t conn_params;

  //First check current connection parameterm avoid redundant requests
  if ( !check_conn_param_update( mw_ble_peripheral_connection_obj->conn_params.params, FAST_IPHONE_CONN_PARAMS_REQUEST ) )
  {
    mw_ble_peripheral_connection_obj->conn_params.mode = FAST_IPHONE_CONN_PARAMS_REQUEST;
    conn_params.min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS);
    conn_params.max_conn_interval = MSEC_TO_UNITS(35, UNIT_1_25_MS);
    conn_params.slave_latency = 5;
    conn_params.conn_sup_timeout = MSEC_TO_UNITS(6000, UNIT_10_MS);
    err_code = sd_ble_gap_conn_param_update( conn_handle, &conn_params );
  }
  else
  {
    ble_conn_params_stop();
  }

  return err_code;
}

/**@brief Send Fast Mode Connection Parameter Request
 *
 * @details This function will be called to send the Fast Mode Connection Parameter Request
 *
 * @param[in]   conn_handle  Connection Handler to send the request to
 */
uint32_t send_fast_mode_request( const uint16_t conn_handle )
{
  uint32_t err_code = NRF_SUCCESS;
  ble_gap_conn_params_t conn_params;

  //First check current connection parameterm avoid redundant requests
  if ( !check_conn_param_update( mw_ble_peripheral_connection_obj->conn_params.params, FAST_CONN_PARAMS_REQUEST) )
  {
    mw_ble_peripheral_connection_obj->conn_params.mode = FAST_CONN_PARAMS_REQUEST;
    conn_params.min_conn_interval = mw_ble_peripheral_connection_obj->fast_mode_params.min_conn_interval;
    conn_params.max_conn_interval = mw_ble_peripheral_connection_obj->fast_mode_params.max_conn_interval;
    conn_params.slave_latency = mw_ble_peripheral_connection_obj->fast_mode_params.slave_latency;
    conn_params.conn_sup_timeout = mw_ble_peripheral_connection_obj->fast_mode_params.conn_sup_timeout;
    err_code = sd_ble_gap_conn_param_update( conn_handle, &conn_params );
  }
  else
  {
    ble_conn_params_stop();
  }

  return err_code;
}

/**@brief Send Slow Mode Connection Parameter Request
 *
 * @details This function will be called to send the Slow Mode Connection Parameter Request
 *
 * @param[in]   conn_handle  Connection Handler to send the request to
 */
uint32_t send_slow_mode_request( const uint16_t conn_handle )
{
  uint32_t err_code = NRF_SUCCESS;
  ble_gap_conn_params_t conn_params;

  //First check current connection parameterm avoid redundant requests
  if ( !check_conn_param_update( mw_ble_peripheral_connection_obj->conn_params.params, SLOW_CONN_PARAMS_REQUEST) )
  {
    mw_ble_peripheral_connection_obj->conn_params.mode = SLOW_CONN_PARAMS_REQUEST;
    conn_params.min_conn_interval = mw_ble_peripheral_connection_obj->slow_mode_params.min_conn_interval;
    conn_params.max_conn_interval = mw_ble_peripheral_connection_obj->slow_mode_params.max_conn_interval;
    conn_params.slave_latency = mw_ble_peripheral_connection_obj->slow_mode_params.slave_latency;
    conn_params.conn_sup_timeout = mw_ble_peripheral_connection_obj->slow_mode_params.conn_sup_timeout;
    err_code = sd_ble_gap_conn_param_update( conn_handle, &conn_params );
  }
  else
  {
    //MW_LOG("Updated to Slow Connection Parameter Mode...\r\n", NULL, BLE_EVENT);
    ble_conn_params_stop();
  }

  return err_code;
}

/**@brief Checks Update for Connection Parameters
 *
 * @details This function will be called to verify Slow Mode was enabled
 *
 * @param[in]   p_ble_evt   BLE Event struct
 * @param[in]   type      Type of Connection Parameter Update
 */
bool check_conn_param_update( const ble_gap_conn_params_t update, mw_connection_parameter_mode_t type )
{
  if ( type == SLOW_CONN_PARAMS_REQUEST )
  {
    if ( update.min_conn_interval < mw_ble_peripheral_connection_obj->slow_mode_params.min_conn_interval )
      return false;

    if ( update.slave_latency < mw_ble_peripheral_connection_obj->slow_mode_params.slave_latency )
      return false;

    if ( update.conn_sup_timeout < mw_ble_peripheral_connection_obj->slow_mode_params.conn_sup_timeout )
      return false;
  }

  if ( type == FAST_CONN_PARAMS_REQUEST )
  {
    if (update.max_conn_interval > mw_ble_peripheral_connection_obj->fast_mode_params.max_conn_interval )
      return false;

    if ( update.slave_latency > mw_ble_peripheral_connection_obj->fast_mode_params.slave_latency )
      return false;
  }

  if ( type == FAST_IPHONE_CONN_PARAMS_REQUEST )
  {
    if ( update.max_conn_interval > MSEC_TO_UNITS(30, UNIT_1_25_MS) )
      return false;

    if ( update.slave_latency > 5 )
      return false;
  }

  return true;
}

/**@brief Update for Connection Mode for Parameter Handling
 *
 * @details This function to update the modules method of handling
 *          Connection Parameter Updating
 *
 * @param[in]   mode   Mode update value
 */
void update_mw_connection_mode( mw_update_mode_t mode )
{
  mw_ble_peripheral_connection_obj->module_update_mode = mode;
}

static void mw_set_gatt_mtu( uint16_t att_mtu )
{
  APP_ERROR_CHECK(nrf_ble_gatt_att_mtu_periph_set(&m_gatt, att_mtu));
}


/**@brief Function for enabling Data Length Extention */
static void mw_set_connection_event_length_extention( bool status )
{
  ret_code_t err_code;
  ble_opt_t opt;

  memset(&opt, 0x00, sizeof(opt));
  opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;

  err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting GATT Data Length */
static void mw_set_gatt_data_length( uint8_t value )
{
#if !S112
  APP_ERROR_CHECK(nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, value));
#endif
}


/**@brief Function for initializing the GATT library. */
static void gatt_init( void )
{
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init( void )
{
  uint32_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));
  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID; //Can be used to trigger on Notification subscription
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = mw_on_conn_params_evt;
  cp_init.error_handler = mw_conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for setup of preferred connection parameters.
 */
static void conn_params_setup( void )
{
  uint32_t err_code = NRF_SUCCESS;

  memset(&mw_ble_peripheral_connection_obj->fast_mode_params, 0, sizeof(mw_ble_peripheral_connection_obj->fast_mode_params));
  memset(&mw_ble_peripheral_connection_obj->slow_mode_params, 0, sizeof(mw_ble_peripheral_connection_obj->slow_mode_params));

  mw_ble_peripheral_connection_obj->module_update_mode = DEFAULT_UPDATE_MODE;
  mw_ble_peripheral_connection_obj->default_mode_params.min_conn_interval   = DEFAULT_MIN_CONN_INTERVAL;
  mw_ble_peripheral_connection_obj->default_mode_params.max_conn_interval   = DEFAULT_MAX_CONN_INTERVAL;
  mw_ble_peripheral_connection_obj->default_mode_params.slave_latency       = DEFAULT_SLAVE_LATENCY;
  mw_ble_peripheral_connection_obj->default_mode_params.conn_sup_timeout    = DEFAULT_CONN_SUP_TIMEOUT;

  mw_ble_peripheral_connection_obj->fast_mode_params.min_conn_interval = FAST_MIN_CONN_INTERVAL;
  mw_ble_peripheral_connection_obj->fast_mode_params.max_conn_interval = FAST_MAX_CONN_INTERVAL;
  mw_ble_peripheral_connection_obj->fast_mode_params.slave_latency    = FAST_SLAVE_LATENCY;
  mw_ble_peripheral_connection_obj->fast_mode_params.conn_sup_timeout = FAST_CONN_SUP_TIMEOUT;

  mw_ble_peripheral_connection_obj->slow_mode_params.min_conn_interval = SLOW_MIN_CONN_INTERVAL;
  mw_ble_peripheral_connection_obj->slow_mode_params.max_conn_interval = SLOW_MAX_CONN_INTERVAL;
  mw_ble_peripheral_connection_obj->slow_mode_params.slave_latency    = SLOW_SLAVE_LATENCY;
  mw_ble_peripheral_connection_obj->slow_mode_params.conn_sup_timeout   = SLOW_CONN_SUP_TIMEOUT;

  // err_code = sd_ble_gap_ppcp_set( &mw_ble_connected_obj->slow_mode_params );
  err_code = sd_ble_gap_ppcp_set(&mw_ble_peripheral_connection_obj->default_mode_params);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize MW Connection Module
 *
 * @details Initializes module and sets default configuration values
 *
 * @param[in]   slow_conn_params   Slow connection Parameters
 * @param[in]   fast_conn_params   Fast connection Parameters
 */
void mw_ble_peripheral_connection_init( mw_peripheral_connection_update_handler_t evt_handler )
{
  m_evt_handler = evt_handler;

  //Default values
  mw_ble_peripheral_connection_obj->ble_data_params.att_mtu                   = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  mw_ble_peripheral_connection_obj->ble_data_params.conn_evt_len_ext_enabled  = true;
  mw_ble_peripheral_connection_obj->ble_data_params.data_len                  = NRF_SDH_BLE_GAP_DATA_LENGTH;

  //Setup Device Name
  gap_params_init();

  //Setup Connection Parameters
  conn_params_setup();

  //Initialize Connection Parameters
  conn_params_init();

  //Initialize GATT Parameters
  gatt_init();

  //Set BLE 4.2 Data Parameters
  mw_set_gatt_mtu(NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  mw_set_gatt_data_length(NRF_SDH_BLE_GAP_DATA_LENGTH);
  mw_set_connection_event_length_extention(true);

  NRF_LOG_INFO( "PERIPHERAL CONNECTION MODULE - Initialized" );
}

/**
 * @}
 */

