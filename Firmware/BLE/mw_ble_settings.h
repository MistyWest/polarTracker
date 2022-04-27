/*
 * mw_ble_settings.h
 *
 *  Created on: Sep 29, 2017
 *      Author: KLockwood
 */

#ifndef BLE_MW_BLE_SETTINGS
#define BLE_MW_BLE_SETTINGS

#include "project_settings.h"

/**********************************************************
 *
 * 				DEVICE SETTINGS
 *
 **********************************************************/
#ifndef DEVICE_NAME
#if BLE_PERIPHERAL_THREAD_ENABLED
#define DEVICE_NAME                         "WWF Tracker"                   		 /**< Name of device. Will be included in the advertising data. */
#elif BLE_CENTRAL_THREAD_ENABLED
#define DEVICE_NAME                         "MW_Central_device"                          /**< Name of device. Will be included in the advertising data. */
#else
#define DEVICE_NAME                         "MW_device"                                  /**< Name of device. Will be included in the advertising data. */
#endif
#endif

/**********************************************************
 *
 * 				BLE SETTINGS - PERIPHERAL
 *
 **********************************************************/
#define MW_GET_ADV_TIMEOUT(ADV_TIMEOUT) ADV_TIMEOUT * 100

#define IS_SRVC_CHANGED_CHARACT_PRESENT 		1 																					/**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define CENTRAL_LINK_COUNT               		1                                          	/**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            		1                                          	/**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

//Radio Power Settings
#define MW_ADVERTISING_RADIO_TX							TX_PWR_ZERO_DB
#define MW_CONNECTED_RADIO_TX								TX_PWR_ZERO_DB
#define TX_PWR_ZERO_DB											0
#define TX_PWR_FOUR_DB											4

// ULTRA fast Advertising Parameters
#define MW_ULTRA_FAST_ADV_INTERVAL          32                                          /**< 20msec in units of 0.625msec */
#define MW_ULTRA_FAST_ADV_TIMEOUT           MW_GET_ADV_TIMEOUT(2)                       /**< 8 second timeout - units 10msec*/

// Fast Advertising Parameters
#define MW_FAST_ADV_INTERVAL								80																					/**< 20msec in units of 0.625msec */
#define MW_FAST_ADV_TIMEOUT									MW_GET_ADV_TIMEOUT(8)												/**< 8 second timeout - units 10msec*/

// Slow Advertising Parameters
#define MW_MED_ADV_INTERVAL				    			668																					/**< 417.5msec in units of 0.625msec */
#define MW_MED_ADV_TIMEOUT									MW_GET_ADV_TIMEOUT(30 /*285*/)											/**< 285 second timeout - units 10msec */

// Ultra Slow Advertising Parameters
#define MW_SLOW_ADV_INTERVAL			    			2510																				/**< 1.285sec in units of 0.625msec */
#define MW_SLOW_ADV_TIMEOUT			        		MW_GET_ADV_TIMEOUT(0) 											/**< No timeout - units 10msec */

// Default Connection Parameters
#define DEFAULT_MIN_CONN_INTERVAL           MSEC_TO_UNITS(30, UNIT_1_25_MS)           	/**< Minimum acceptable connection interval (8 msec). */
#define DEFAULT_MAX_CONN_INTERVAL           MSEC_TO_UNITS(50, UNIT_1_25_MS)          	  /**< Maximum acceptable connection interval (28 msec). */
#define DEFAULT_SLAVE_LATENCY               30                                         	/**< Slave latency. */
#define DEFAULT_CONN_SUP_TIMEOUT            MSEC_TO_UNITS(4000, UNIT_10_MS)            	/**< Connection supervisory time-out (4 seconds). */

// Slow Connection Parameters
#define SLOW_MIN_CONN_INTERVAL              MSEC_TO_UNITS(450, UNIT_1_25_MS)           	/**< Minimum acceptable connection interval (0.4 seconds). */
#define SLOW_MAX_CONN_INTERVAL              MSEC_TO_UNITS(495, UNIT_1_25_MS)           	/**< Maximum acceptable connection interval (0.65 second). */
#define SLOW_SLAVE_LATENCY                  3                                          	/**< Slave latency. */
#define SLOW_CONN_SUP_TIMEOUT               MSEC_TO_UNITS(6000, UNIT_10_MS)            	/**< Connection supervisory time-out (4 seconds). */

// Fast Connection Parameters
#define FAST_MIN_CONN_INTERVAL              MSEC_TO_UNITS(20, UNIT_1_25_MS)           	/**< Minimum acceptable connection interval (8 msec). */
#define FAST_MAX_CONN_INTERVAL              MSEC_TO_UNITS(40, UNIT_1_25_MS)          		/**< Maximum acceptable connection interval (28 msec). */
#define FAST_SLAVE_LATENCY                  5                                          	/**< Slave latency. */
#define FAST_CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS)            	/**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      1000                                      	/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (2 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       3000                                        /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        2                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                          /**< Maximum encryption key size. */

#define APP_BLE_OBSERVER_PRIO               3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                           /**< A tag identifying the SoftDevice BLE configuration. */

/**********************************************************
 *
 *        BLE SETTINGS - CENTRAL
 *
 **********************************************************/
#define TARGET_PERIPHERAL_DEVICE_NAME       "Tile"

#define MW_BLE_SCAN_EXT_ADV_PACKETS         0                           /**< Accept extended advertising packets - NOT SUPPORTED */

#define MW_BLE_SCAN_REPORT_INCOMPLETE_EVTS  0                           /**< Report Incomplete Events when Extended Adv Enabled - NOT SUPPORTED */

#define MW_BLE_SCAN_ACTIVE_SCANNING         1                           /**< Perform active scanning by sending scan requests */

#define MW_BLE_SCAN_FILTER_POLICY           BLE_GAP_SCAN_FP_ACCEPT_ALL  /**< Scanning Filter Policy*/

#define MW_BLE_SCAN_BUFFER                  31                          /**< Data length for an advertising set. */

#define MW_BLE_SCAN_NAME_MAX_LEN            32                          /**< Maximum size for the name to search in the advertisement report. */

#define MW_BLE_SCAN_SHORT_NAME_MAX_LEN      50                          /**< Maximum size of the short name to search for in the advertisement report. */


#define MW_BLE_SCAN_SCAN_WINDOW             80                          /**< Scanning window. Determines the scanning window in units of 0.625 millisecond. */

#define MW_BLE_SCAN_SCAN_INTERVAL           160                         /**< Scanning interval. Determines the scan interval in units of 0.625 millisecond. */

#define MW_BLE_SCAN_SCAN_DURATION           1000                        /**< Duration of a scanning session in units of 10 ms. Range: 0x0001 - 0xFFFF (10 ms to 10.9225 ms).
                                                                           * If set to 0x0000, the scanning continues until it is explicitly disabled. */

#define MW_BLE_SCAN_ALL_CHANNELS            {0, 0, 0, 0, 0 }            /**< Scan all advertising channels,  see @ble_gap_ch_mask_t */

#define MW_BLE_SCAN_MIN_CONNECTION_INTERVAL 7.5                         /**< Determines minimum connection interval in milliseconds. */

#define MW_BLE_SCAN_MAX_CONNECTION_INTERVAL 30                          /**< Determines maximum connection interval in milliseconds. */

#define MW_BLE_SCAN_SLAVE_LATENCY           0                           /**< Determines the slave latency in counts of connection events. */

#define MW_BLE_SCAN_SUPERVISION_TIMEOUT     4000                        /**< Determines the supervision time-out in units of 10 millisecond. */

/**< PHY to scan on.
* <0=> BLE_GAP_PHY_AUTO
* <1=> BLE_GAP_PHY_1MBPS
* <2=> BLE_GAP_PHY_2MBPS
* <4=> BLE_GAP_PHY_CODED
* <255=> BLE_GAP_PHY_NOT_SET
 */
#define MW_BLE_SCAN_SCAN_PHY                  BLE_GAP_PHY_AUTO

#define MW_BLE_SCAN_DEFAULT_PARAMETERS                                \
{                                                                     \
    .extended               = MW_BLE_SCAN_EXT_ADV_PACKETS,            \
    .report_incomplete_evts = MW_BLE_SCAN_REPORT_INCOMPLETE_EVTS,     \
    .active                 = MW_BLE_SCAN_ACTIVE_SCANNING,            \
    .filter_policy          = MW_BLE_SCAN_FILTER_POLICY,              \
    .scan_phys              = MW_BLE_SCAN_SCAN_PHY,                   \
    .interval               = MW_BLE_SCAN_SCAN_INTERVAL,              \
    .window                 = MW_BLE_SCAN_SCAN_WINDOW,                \
    .timeout                = MW_BLE_SCAN_SCAN_DURATION,              \
    .channel_mask           = MW_BLE_SCAN_ALL_CHANNELS                \
}                                                                     \

//==========================================================

// <h> Check LF Crystal Settings

//==========================================================
/* Check SDK LFCLK settings.  If using External Xtal
 * Set RC Settings to 0
 */
#if ( (NRF_SDH_CLOCK_LF_SRC == 1) && (NRF_SDH_CLOCK_LF_RC_CTIV || NRF_SDH_CLOCK_LF_RC_CTIV ) )
#warning Incorrect LFCLK Settings.  NRF_SDH_CLOCK_LF_RC_CTIV and NRF_SDH_CLOCK_LF_RC_CTIV should be set to zero
#endif


/**********************************************************
 *
 *        BLE SETTINGS - Misc
 *
 **********************************************************/

#ifndef OPCODE_LENGTH
#define OPCODE_LENGTH        1
#endif
#ifndef HANDLE_LENGTH
#define HANDLE_LENGTH        2
#endif

#endif /* BLE_MW_BLE_SETTINGS */
