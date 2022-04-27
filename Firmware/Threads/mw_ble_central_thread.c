/*
 * mw_ble_central_thread.c
 *
 *  Created on: Nov 30, 2018
 *      Author: KLockwood
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "app_uart.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "ble_db_discovery.h"
#include "nrf_ble_scan.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

// Services
#include "ble_nus_c.h"


//FreeRTOS Includes
#include <FreeRTOS/FreeRTOS_includes.h>

//Project Includes
#include <project_settings.h>
#include <mw_ble_settings.h>
#include "mw_thread.h"
#include "../CLI_Logging/mw_logging.h"

// Threads - which interact with BLE thread
#include "mw_ble_central_thread.h"
#include "mw_leds_thread.h"
#include "mw_cli_thread.h"

#if BLE_CENTRAL_THREAD_ENABLED

#if USE_BSP_MODULE
#include "bsp_btn_ble.h"
#endif

#if !CENTRAL_LINK_COUNT
#warning Central Link Count should be set to greater than 0 in mw_ble_settings.h
#endif

#if !NRF_BLE_SCAN_ENABLED || !BLE_DB_DISCOVERY_ENABLED
#warning NRF_BLE_SCAN_ENABLED and BLE_DB_DISCOVERY_ENABLED must be enabled in sdk_config.h
#endif


#define BLE_LOG_TAG                 "BLE Central Thread: "

#define NUS_SERVICE_UUID_TYPE       BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

static volatile mw_thread_mode_t    m_mw_ble_thread_mode = THREAD_NULL;


NRF_BLE_GATT_DEF(m_gatt);                                                   /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                            /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                                   /**< Scanning Module instance. */


static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;            /**< Handle of the current connection. */

static char const m_target_periph_name[] = TARGET_PERIPHERAL_DEVICE_NAME;

/* Connection parameters requested for connection. */
static ble_gap_conn_params_t m_conn_param = {
    .min_conn_interval  = DEFAULT_MIN_CONN_INTERVAL,   // Minimum connection interval.
    .max_conn_interval  = DEFAULT_MAX_CONN_INTERVAL,   // Maximum connection interval.
    .slave_latency      = DEFAULT_SLAVE_LATENCY,       // Slave latency.
    .conn_sup_timeout   = DEFAULT_CONN_SUP_TIMEOUT     // Supervisory timeout.
};

/* Set Central Scanning Parameters */
static ble_gap_scan_params_t m_scan_params = MW_BLE_SCAN_DEFAULT_PARAMETERS;

static uint16_t m_ble_nus_mtu_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT;

//******************************************************
//******************************************************

/**@snippet [Handling events from the ble_nus_c module] */

/**@brief Function for handling Scanning Module events.
 */
//@scan
static void scan_evt_handler( scan_evt_t const * p_scan_evt )
{
  ret_code_t err_code;
//  ble_gap_evt_adv_report_t const * p_scanned_peer = p_scan_evt->params.filter_match.p_adv_report;
//  ble_gap_scan_params_t const * p_scan_param = p_scan_evt->p_scan_params;

  switch ( p_scan_evt->scan_evt_id )
  {
  case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
    err_code = p_scan_evt->params.connecting_err.err_code;
    APP_ERROR_CHECK(err_code);
    break;

  case NRF_BLE_SCAN_EVT_CONNECTED:
    // Scan is automatically stopped by the connection.
    MW_LOG_RAW( BLE_LOG_TAG "Connected to device: %02x%02x%02x%02x%02x%02x",
        p_scan_evt->params.connected.p_connected->peer_addr.addr[0],
        p_scan_evt->params.connected.p_connected->peer_addr.addr[1],
        p_scan_evt->params.connected.p_connected->peer_addr.addr[2],
        p_scan_evt->params.connected.p_connected->peer_addr.addr[3],
        p_scan_evt->params.connected.p_connected->peer_addr.addr[4],
        p_scan_evt->params.connected.p_connected->peer_addr.addr[5] );
    break;

  case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
    CLI_SEND_STRING( "Scan complete\r\n");
    CLI_SEND_STRING( "*****************************************\r\n" );
    break;

  case NRF_BLE_SCAN_EVT_FILTER_MATCH:
    MW_LOG_INFO( BLE_LOG_TAG "Device Found matching SCAN FILTER");
    /** Added any action function */
    break;

  default:
    break;
  }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
//@ble
static void ble_evt_handler( ble_evt_t const * p_ble_evt, void * p_context )
{
  uint32_t err_code;
  ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

  switch ( p_ble_evt->header.evt_id )
  {
  case BLE_GAP_EVT_CONNECTED:
    MW_LOG_RAW( "*****************************************\r\n" );
    MW_LOG_RAW(BLE_LOG_TAG "Connected");
    CLI_SEND_STRING( "*****************************************\r\n" );
    CLI_SEND_STRING("Connected");
#if USE_BSP_MODULE
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
#endif
#if LED_THREAD_ENABLED
    mw_leds_ble_event(CENTRAL_CONNECTED_LEDS, NORMAL_CONTEXT);
#endif
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
    APP_ERROR_CHECK(err_code);

    // Ping peer for Data Length Updates
    err_code = sd_ble_gap_data_length_update(p_gap_evt->conn_handle, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    MW_LOG_RAW(BLE_LOG_TAG "Disconnected");
    CLI_SEND_STRING("Disconnected");
    mw_leds_ble_event(SCANNING_LEDS, NORMAL_CONTEXT);
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    //scan_start();
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    MW_LOG_DEBUG(BLE_LOG_TAG "Connection Parameter Request");
    MW_LOG_DEBUG(BLE_LOG_TAG "Interval Min: %d \r\n", p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval * 5 / 4);
    MW_LOG_DEBUG(BLE_LOG_TAG "Interval Max: %d \r\n", p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval * 5 / 4);
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    MW_LOG_DEBUG(BLE_LOG_TAG "PHY update Request");
    ble_gap_phys_t const phys = { .rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO, };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    MW_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    MW_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler( ble_db_discovery_evt_t * p_evt )
{
  /*Add handler function for Database Discovery */
}



/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler( nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt )
{
  switch ( p_evt->evt_id )
  {
  case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
    NRF_LOG_INFO("ATT MTU exchange completed");
    m_ble_nus_mtu_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Ble NUS MTU set to %dbytes", m_ble_nus_mtu_len);

    /*Add Application Functions to propagate updates */
    break;

  case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
    NRF_LOG_INFO("Data Length Update");
    m_ble_nus_max_data_len = p_evt->params.data_length;;
    NRF_LOG_INFO("Ble NUS max data length set to %dbytes", m_ble_nus_max_data_len );

    /*Add Application Functions to propagate updates */
    break;
  }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
#if USE_BSP_MODULE
static void bsp_event_handler(bsp_event_t event)
{
  ret_code_t err_code;
  ble_gap_conn_params_t conn_params;

  switch (event)
  {
    case BSP_EVENT_SLEEP:
    sleep_mode_enter();
    break;

    case BSP_EVENT_KEY_0:
    conn_params.min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS);
    conn_params.max_conn_interval = MSEC_TO_UNITS(35, UNIT_1_25_MS);
    conn_params.slave_latency = 5;
    conn_params.conn_sup_timeout = MSEC_TO_UNITS(500, UNIT_10_MS);
    err_code = sd_ble_gap_conn_param_update( m_conn_handle, &conn_params );
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
      APP_ERROR_CHECK(err_code);
    }
    break;

    case BSP_EVENT_KEY_1:

    break;

    case BSP_EVENT_KEY_2:
    MW_LOG_DEBUG( BLE_LOG_TAG "Stopped BLE Throughput Test");
    stop_data_throughput_test();
    break;

    case BSP_EVENT_KEY_3:
    MW_LOG_DEBUG( BLE_LOG_TAG "Disconnecting.....");
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
      APP_ERROR_CHECK(err_code);
    }
    break;

    default:
    break;
  }
}
#endif


/**
 * @brief - Used to disconnect from a specific Peer
 */
void mw_ble_central_disconnect_from_peer()
{
#if !BLE_CENTRAL_THREAD_ENABLED
  return;
#endif

  uint32_t err_code;
  err_code =  sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  if ( err_code != NRF_SUCCESS )
  {
    MW_LOG_DEBUG(BLE_LOG_TAG "sd_ble_gap_disconnect() failed: 0x%x.", err_code);
  }
}


/**
 * @brief - Used to connect to a specific Peer
 */
void mw_ble_central_connect_to_peer( ble_gap_addr_t peer_addr )
{
#if !BLE_CENTRAL_THREAD_ENABLED
  return;
#endif
  uint32_t err_code;

  // Initiate connection.
  m_conn_param.min_conn_interval = DEFAULT_MIN_CONN_INTERVAL;
  m_conn_param.max_conn_interval = DEFAULT_MAX_CONN_INTERVAL;

  err_code = sd_ble_gap_connect(&peer_addr, &m_scan_params, &m_conn_param, APP_BLE_CONN_CFG_TAG);
  if ( err_code != NRF_SUCCESS )
  {
    MW_LOG_DEBUG(BLE_LOG_TAG "sd_ble_gap_connect() failed: 0x%x.", err_code);
  }
}




/**@brief Function for starting scanning. */
void scan_start()
{
#if !BLE_CENTRAL_THREAD_ENABLED
#if FREERTOS_CLI_THREAD_ENABLED
  CLI_SEND_STRING("*****************************************\r\n");
  CLI_SEND_STRING("Device ");
  CLI_SEND_STRING("  MAC: 00:11:22:33:44:55");
  CLI_SEND_STRING(" ,RSSI: -65");
  CLI_SEND_STRING("dB\r\n");
  CLI_SEND_STRING( "Scan complete\r\n");
  CLI_SEND_STRING( "*****************************************\r\n" );
  return;
#endif
#endif
  ret_code_t ret;

  if(m_mw_ble_thread_mode != THREAD_INITIALIZED) return;

  ret = nrf_ble_scan_start(&m_scan);
  APP_ERROR_CHECK(ret);

  MW_LOG_DEBUG( BLE_LOG_TAG "Starting Scanning");
  mw_leds_ble_event(SCANNING_LEDS, NORMAL_CONTEXT);

#if USE_BSP_MODULE
  ret = bsp_indication_set(BSP_INDICATE_SCANNING);
  APP_ERROR_CHECK(ret);
#endif

}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init( void )
{
  // Initialize timer module.
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init( void )
{
  //TODO - If necessary
}

/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start( void )
{
  // Start application timers.
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init( bool * p_erase_bonds )
{
#if USE_BSP_MODULE
  ret_code_t err_code;
  bsp_event_t startup_event;

  err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
#endif
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init( void )
{
  ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init( void )
{
  ret_code_t err_code;
  nrf_ble_scan_init_t init_scan;

  memset(&init_scan, 0, sizeof(init_scan));

  init_scan.p_scan_param = &m_scan_params;
  init_scan.connect_if_match = false;
  init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

  err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
  APP_ERROR_CHECK(err_code);

  /* Add UUID based Filter */
//  err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
//  APP_ERROR_CHECK(err_code);
//
//  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
//  APP_ERROR_CHECK(err_code);
//  MW_LOG_DEBUG( BLE_LOG_TAG "Starting Scanning for UUID %d ", m_nus_uuid.uuid);

  /* Add Device Name based Filter */
  err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, &m_target_periph_name);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
  APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the GATT library. */
void gatt_init( void )
{
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init( void )
{
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  //Increase TX Buffer Size
  ble_cfg_t ble_cfg;
  memset(&ble_cfg, 0, sizeof ble_cfg);
  ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
  ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 16;
  err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for starting scanning. */
void mw_ble_central_thread_start()
{
  m_mw_ble_thread_mode = THREAD_INITIALIZED;
  MW_LOG_DEBUG(BLE_LOG_TAG "BLE Central Thread Starting....");

  scan_start();
}

/**@brief Function for application main entry.
 */
void mw_ble_central_thread_init(void)
{
	bool erase_bonds = false;

	// Configure and initialize the BLE stack
	ble_stack_init();

	// Initialize modules.
	timers_init();
	buttons_leds_init(&erase_bonds);
	gatt_init();
  db_discovery_init();
	scan_init();
	services_init();
	application_timers_start();


	// Create a FreeRTOS task for the BLE stack.
	nrf_sdh_freertos_init(mw_ble_central_thread_start, NULL);


}

#endif


