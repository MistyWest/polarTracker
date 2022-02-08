/*
 * mw_ble_peripheral_thread.c
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
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "app_uart.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

// Services
#include "ble_bas.h"
#include "ble_nus.h"
#include "ble_dis.h"

//FreeRTOS Includes
#include <FreeRTOS/FreeRTOS_includes.h>

//Project Includes
#include <project_settings.h>
#include <mw_ble_settings.h>
#include "mw_thread.h"
#include "../CLI_Logging/mw_logging.h"
#include "throughput_test.h"
#include "mw_ble_advertising.h"
#include "mw_ble_peripheral_connection.h"
#include "device_info_service.h"

// Threads - which interact with BLE thread
#include "mw_leds_thread.h"
#include "mw_data_manager_thread.h"
#include "mw_adc_thread.h"
#include "mw_sensor_thread.h"
#include "mw_cli_thread.h"
#include "mw_temperature_thread.h"
#include "wwf_service.h"

#if !PERIPHERAL_LINK_COUNT
#warning Peripheral Link Count should be set to greater than 0 in mw_ble_settings.h
#endif

#if !BLE_ADVERTISING_ENABLED
#warning BLE_ADVERTISING_ENABLED must be enabled in sdk_config.h
#endif

#if USE_BSP_MODULE
#include "bsp_btn_ble.h"
#endif

#define BLE_LOG_TAG                 "BLE Thread: "

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/

static void advertising_start(void * p_erase_bonds);

/* Declare all UUID to be included in Advertising Payload*/
static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    //{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    //{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN},
};


//******************************************************
/**
 * @brief - Returns current connection handle
 */
uint16_t mw_ble_get_connection_handle()
{
  return m_conn_handle;
}


//******************************************************


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler( pm_evt_t const * p_evt )
{
  pm_handler_on_pm_evt(p_evt);
  pm_handler_flash_clean(p_evt);

  switch ( p_evt->evt_id )
  {
  case PM_EVT_PEERS_DELETE_SUCCEEDED:
    mw_leds_ble_event(DISCONNECTED_LEDS, NORMAL_CONTEXT);
    ret_code_t err_code = NRF_ERROR_BUSY;
    while ( err_code == NRF_ERROR_BUSY )
    {
      err_code = mw_ble_advertising_start(MW_BLE_ADV_MODE_FAST);
      MW_LOG_DEBUG("Waiting for Flash to complete");
      vTaskDelay(20);
    }
    break;

  default:
    break;
  }
}




/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	// Initialize timer module.
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    //***************************************************
    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    //***************************************************
    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HARDWARE_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)FIRMWARE_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER);


    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);


    //***************************************************
    // Initialize WWF Service
    wwf_service_init(&m_conn_handle);
    //***************************************************
}


/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
	// Start application timers.
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	  ret_code_t err_code;
#if USE_BSP_MODULE
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
#endif
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
//@adv
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
#if USE_BSP_MODULE
	uint32_t err_code;
#endif

	switch ( ble_adv_evt )
	{
  case MW_BLE_ADV_EVT_ULTRA_FAST:
    MW_LOG_INFO( BLE_LOG_TAG "Ultra Fast advertising.");
#if USE_BSP_MODULE
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
#endif
    break;

	case MW_BLE_ADV_EVT_FAST:
		MW_LOG_INFO( BLE_LOG_TAG "Fast advertising.");
#if USE_BSP_MODULE
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		APP_ERROR_CHECK(err_code);
#endif
		break;

	case MW_BLE_ADV_EVT_MED:
		MW_LOG_INFO( BLE_LOG_TAG "Medium advertising.");
#if USE_BSP_MODULE
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		APP_ERROR_CHECK(err_code);
#endif
		break;

	case MW_BLE_ADV_EVT_SLOW:
		MW_LOG_INFO( BLE_LOG_TAG "Slow advertising.");
#if LED_THREAD_ENABLED
    mw_leds_ble_event(LOW_POWER_DISCONNECTED_LEDS, NORMAL_CONTEXT);
#endif
#if USE_BSP_MODULE
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
		APP_ERROR_CHECK(err_code);
#endif
		break;

	case MW_BLE_ADV_EVT_IDLE:
		sleep_mode_enter();
		break;

	default:
		break;
	}
}

/**@brief Function for handling events from the Peripheral Connection Module */
static void on_connection_evt( mw_peripheral_connection_update_t event, mw_ble_peripheral_connection_t * update )
{
	switch ( event )
	{
	case CONNECTION_INTERVAL_UPDATE:
#if PROJECT_APPLICATION_THREAD_ENABLED
    //TODO - pass MTU Change
#endif
		break;

	case CONNECTION_MTU_UPDATE:
#if PROJECT_APPLICATION_THREAD_ENABLED
    //TODO - pass MTU Change
#endif
		break;

	case CONNECTION_DATA_LENGTH_UPDATE:
#if PROJECT_APPLICATION_THREAD_ENABLED
    //TODO - pass Data Length  Change
#endif
		break;

	case CONNECTION_PHY_UPDATE:
#if PROJECT_APPLICATION_THREAD_ENABLED
		//TODO - pass Phy Change
#endif
		break;
	}
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
//@ble
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
	uint32_t err_code;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		MW_LOG_INFO( BLE_LOG_TAG "\r\n\r\n***Connected***");
		/*MW_LOG_INFO( BLE_LOG_TAG "Central Address: %x %x %x %x %x %x \r\n", p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[0],
																														p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[1],
																														p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[2],
																														p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[3],
																														p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[4],
																														p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[5]);*/

#if USE_BSP_MODULE
		err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		APP_ERROR_CHECK(err_code);
#endif
#if LED_THREAD_ENABLED
		mw_leds_ble_event(CONNECTED_LEDS, NORMAL_CONTEXT);
#endif
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		MW_LOG_INFO( BLE_LOG_TAG "\r\n\r\n***Disconnected***");
		mw_leds_ble_event(DISCONNECTED_LEDS, NORMAL_CONTEXT);
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		break;

	case BLE_GAP_EVT_CONN_PARAM_UPDATE:
		MW_LOG_INFO( BLE_LOG_TAG "Connection Parameter Updated");
		MW_LOG_INFO( BLE_LOG_TAG "Interval Min: %d \r\n" ,p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval*5/4 );
		MW_LOG_INFO( BLE_LOG_TAG "Interval Max: %d \r\n" ,p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval*5/4 );
		break;

	case BLE_GATTS_EVT_HVN_TX_COMPLETE:
		//MW_LOG_INFO( BLE_LOG_TAG "Notification Send Complete");
		break;

	case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		NRF_LOG_DEBUG("GATT Client Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
																		 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GAP_EVT_PHY_UPDATE:
		MW_LOG_INFO( BLE_LOG_TAG "PHY update Status: %d \r\n", p_ble_evt->evt.gap_evt.params.phy_update.status);
		MW_LOG_INFO( BLE_LOG_TAG "PHY update recieved Tx Phy: %d \r\n", p_ble_evt->evt.gap_evt.params.phy_update.tx_phy);
		MW_LOG_INFO( BLE_LOG_TAG "PHY update recieved Rx Phy: %d \r\n", p_ble_evt->evt.gap_evt.params.phy_update.rx_phy);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		NRF_LOG_DEBUG("GATT Server Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		APP_ERROR_CHECK(err_code);
		break;

	default:
		// No implementation needed.
		break;
	}
}


/**
 * @brief - Function to trigger BLE disconnect
 */
uint32_t mw_ble_peripheral_disconnect()
{
  uint32_t err_code;

  if( m_conn_handle != BLE_CONN_HANDLE_INVALID )
  {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
  }
  return err_code;
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void mw_ble_peripheral_stack_init(void)
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
		conn_params.slave_latency     = 5;
		conn_params.conn_sup_timeout  = MSEC_TO_UNITS(500, UNIT_10_MS);
		err_code = sd_ble_gap_conn_param_update( m_conn_handle, &conn_params );
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
		break;

	case BSP_EVENT_KEY_1:

		break;

	case BSP_EVENT_KEY_2:
		MW_LOG_INFO( BLE_LOG_TAG "Stopped BLE Throughput Test");
		stop_data_throughput_test();
		break;

	case BSP_EVENT_KEY_3:
		MW_LOG_INFO( BLE_LOG_TAG "Disconnecting.....");
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


/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
  return;
	ble_gap_sec_params_t sec_param;
	ret_code_t           err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond           = SEC_PARAM_BOND;
	sec_param.mitm           = SEC_PARAM_MITM;
	sec_param.lesc           = SEC_PARAM_LESC;
	sec_param.keypress       = SEC_PARAM_KEYPRESS;
	sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
	sec_param.oob            = SEC_PARAM_OOB;
	sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
	sec_param.kdist_own.id   = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id  = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
//	ret_code_t err_code;
//MW_LOG_INFO( BLE_LOG_TAG "Erase bonds!");
//
//
//	err_code = pm_peers_delete();
//	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void mw_advertising_init( void )
{
	ret_code_t err_code;
	mw_ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	//Applies default advertising configurations
	mw_ble_advertising_default_config( &init );

	init.advdata.uuids_complete.uuid_cnt 	= sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.advdata.uuids_complete.p_uuids 	= m_adv_uuids;
	init.evt_handler = on_adv_evt;

	err_code = mw_ble_advertising_init( &init );
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection functionality. */
static void mw_connection_init( void )
{
	mw_ble_peripheral_connection_init( on_connection_evt );
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
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


/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
  bool erase_bonds = *(bool*)p_erase_bonds;

  while( ( !mw_temperature_thread_ready() || !mw_sensor_thread_ready() || !mw_adc_thread_ready() || !mw_data_manager_thread_ready() ) )
  {
    vTaskDelay(MW_BLE_THREAD_START_UP_DELAY);
  }

  if (erase_bonds)
  {
  	delete_bonds();
  	// Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
  }
  else
  {
    mw_leds_ble_event(DISCONNECTED_LEDS, NORMAL_CONTEXT);

    MW_LOG_INFO( BLE_LOG_TAG "Starting Advertising....");
  	ret_code_t err_code = mw_ble_advertising_start(MW_BLE_ADV_MODE_FAST);
  	APP_ERROR_CHECK(err_code);
  }
}



/**
 * @brief - Thread Initialization
 */
void mw_ble_peripheral_thread_init(void)
{
#if !BLE_PERIPHERAL_THREAD_ENABLED
  return;
#endif

	bool erase_bonds = false;

	// Configure and initialize the BLE stack
	mw_ble_peripheral_stack_init();

	// Initialize modules.
	timers_init();
	buttons_leds_init(&erase_bonds);
	mw_connection_init();
	mw_advertising_init();
	services_init();
	peer_manager_init();
	application_timers_start();

	// Create a FreeRTOS task for the BLE stack.
	// The task will run advertising_start() before entering its loop
	nrf_sdh_freertos_init(advertising_start, &erase_bonds);
}




