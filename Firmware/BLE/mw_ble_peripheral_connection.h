/*
 * mw_ble_peripheral_connection.h
 *
 *  Created on: April 19, 2017
 *      Author: KLockwood
 */

#ifndef MW_BLE_PERIPHERAL_CONNECTION_H_
#define MW_BLE_PERIPHERAL_CONNECTION_H_

#include <stdint.h>
#include <string.h>
#include "ble_gap.h"
#include "ble_conn_params.h"

typedef enum
{
	DEFAULT_UPDATE_MODE = 0,
	DYNAMIC_UPDATE_MODE
} mw_update_mode_t;

typedef enum
{
	DEFAULT_CONN_PARAMS = 0,
	SLOW_CONN_PARAMS_REQUEST,
	FAST_CONN_PARAMS_REQUEST,
	FAST_IPHONE_CONN_PARAMS_REQUEST,
	SLOW_CONN_PARAMS,
	FAST_CONN_PARAMS,
	FAST_IPHONE_CONN_PARAMS
} mw_connection_parameter_mode_t;


typedef struct
{
	mw_connection_parameter_mode_t	mode;
	ble_gap_conn_params_t 					params;
} mw_connection_parameters_t;

typedef enum
{
	CONNECTION_INTERVAL_UPDATE,
	CONNECTION_DATA_LENGTH_UPDATE,
	CONNECTION_MTU_UPDATE,
	CONNECTION_PHY_UPDATE
}mw_peripheral_connection_update_t;


/* Structure for storing the BLE Connection Data Parameters */
typedef struct
{
    uint16_t        		att_mtu;                    /**< GATT ATT MTU, in bytes. */
    ble_gap_phys_t  		phys;                       /**< Preferred PHYs. */
    uint8_t         		data_len;                   /**< Data length. */
    bool            		conn_evt_len_ext_enabled;   /**< Connection event length extension status. */
} ble_data_params_t;


/* Default BLE Connection Data Parameters */
#define DEFAULT_BLE_SETTINGS                      \
{                                                 \
  .att_mtu                  = 20,                 \
  .data_len                 = 23,                 \
  .conn_evt_len_ext_enabled = 0,                  \
  .phys.tx_phys             = BLE_GAP_PHY_1MBPS,  \
  .phys.rx_phys             = BLE_GAP_PHY_1MBPS,  \
}                                                 \


typedef struct
{
	uint16_t												conn_handle;
	int8_t													connected_power;
	mw_update_mode_t 								module_update_mode; 	/**< Contains the Connection Modules Mode of Operation */
	ble_gap_conn_params_t 					default_mode_params; 	/**< Stores the Modules Default Connection Parameters for acceptable Central Parameters */
	ble_gap_conn_params_t 					slow_mode_params; 		/**< Stores the Modules Slow Connection Parameters set during Initialization */
	ble_gap_conn_params_t 					fast_mode_params; 		/**< Stores the Modules Fast Connection Parameters set during Initialization */
	mw_connection_parameters_t	 		conn_params; 					/**< Stores curent conenction parameters */
	ble_data_params_t								ble_data_params;
} mw_ble_peripheral_connection_t;

/**@brief   Macro for defining a mw_ble_connected instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */

#define MW_BLE_CONNECTED_DEF(_name)                                                               \
static mw_ble_peripheral_connection_t _name; /**< Module Object */                               	\
NRF_SDH_BLE_OBSERVER(_name ## _ble_obs,                                                           \
                     BLE_ADV_BLE_OBSERVER_PRIO,                                                   \
										 mw_ble_connected_on_ble_evt, &_name);


/**@brief   BLE Peripheral Connection event handler type. */
typedef void (*mw_peripheral_connection_update_handler_t)( mw_peripheral_connection_update_t event, mw_ble_peripheral_connection_t * update );

/**@brief   Function for handling BLE events.
 *
 * @details This function must be called from the BLE stack event dispatcher for
 *          the module to handle BLE events that are relevant for the Connected Module.
 *
 * @param[in] p_ble_evt     BLE stack event.
 * @param[in] p_adv         Advertising module instance.
 */
void mw_ble_connected_on_ble_evt( ble_evt_t const * p_ble_evt,  void * p_context );

/**@brief   Function for handling system events.
 *
 * @details This function must be called to handle system events that are relevant
 *          for the Connected Module.
 *
 * @param[in] sys_evt       System event.
 * @param[in] p_adv         Advertising module instance.
 */
void mw_ble_connected_on_sys_evt( uint32_t sys_evt,  void * p_context );

/**@brief Update Connection Parameter Request
 *
 * @details This function will be called to send an Updated Connection Parameter Request
 *
 * @param[in]   conn_handle   Connection Handler to send the request to
 * @param[in]   update   			Update Type
 */
void mw_update_connection_parameters( uint16_t conn_handle, mw_connection_parameter_mode_t update );

/**@brief Send iPhone Fast Mode Connection Parameter Request
 *
 * @details This function will be called to send the Fast Mode Connection Parameter Request
 *
 * @param[in]   conn_handle   Connection Handler to send the request to
 */
uint32_t send_fast_iphone_mode_request( const uint16_t conn_handle );

/**@brief Send Fast Mode Connection Parameter Request
 *
 * @details This function will be called to send the Fast Mode Connection Parameter Request
 *
 * @param[in]   conn_handle   Connection Handler to send the request to
 */
uint32_t send_fast_mode_request( const uint16_t conn_handle );

/**@brief Send Slow Mode Connection Parameter Request
 *
 * @details This function will be called to send the Slow Mode Connection Parameter Request
 *
 * @param[in]   conn_handle   Connection Handler to send the request to
 */
uint32_t send_slow_mode_request( const uint16_t conn_handle );

/**@brief Checks Update for Connection Parameters
 *
 * @details This function will be called to verify Slow Mode was enabled
 *
 * @param[in]   update    BLE Parameter update
 * @param[in]   type   		Type of Connection Parameter Update
 */
bool check_conn_param_update( const ble_gap_conn_params_t update, mw_connection_parameter_mode_t type );

/**@brief Update for Connection Mode for Parameter Handling
 *
 * @details This function to update the modules method of handling
 *          Connection Parameter Updating
 *
 * @param[in]   mode   Mode update value
 */
void update_mw_connection_mode( mw_update_mode_t mode );

/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
//@ble
void on_ble_evt_mw_connected( ble_evt_t const * p_ble_evt, void * p_context );

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the module.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
void mw_on_conn_params_evt( ble_conn_params_evt_t * p_evt );

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void mw_conn_params_error_handler( uint32_t nrf_error );

/**@brief   Function to set radio Tx power during a connection
 *
 * @param[in] tx_power         tx_power
 *
 * * @note Supported tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm
 */
void mw_set_connected_power( int8_t tx_power );


/**@brief   Function to get current Connection Parameters
 *
 */
uint16_t mw_get_connection_interval();


/**@brief Initialize MW Connection Module
 *
 * @details Initializes module and sets default configuration values
 *
 */
void mw_ble_peripheral_connection_init( mw_peripheral_connection_update_handler_t evt_handler );

#endif /* MW_BLE_PERIPHERAL_CONNECTION_H_ */
