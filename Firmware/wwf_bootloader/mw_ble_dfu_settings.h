/*
 * mw_ble_dfu_settings.h
 *
 *  Created on: May 21, 2019
 *      Author: klockwood
 */

#ifndef MW_BLE_DFU_SETTINGS_H_
#define MW_BLE_DFU_SETTINGS_H_

#ifdef NRF_DFU_BLE_ADV_NAME
#undef NRF_DFU_BLE_ADV_NAME
#endif
#define NRF_DFU_BLE_ADV_NAME              "WWF_DFU"


//==========================================================
// <e> NRF_DFU_TRANSPORT_BLE - BLE transport settings
//==========================================================

#ifdef NRF_DFU_BLE_ADV_INTERVAL
#undef NRF_DFU_BLE_ADV_INTERVAL
#endif
#define NRF_DFU_BLE_ADV_INTERVAL          40                /* Advertising interval (in units of 0.625 ms) */


#ifdef NRF_BL_DFU_INACTIVITY_TIMEOUT_MS
#undef NRF_BL_DFU_INACTIVITY_TIMEOUT_MS
#endif
#define NRF_BL_DFU_INACTIVITY_TIMEOUT_MS  0   /* Inactivity Timeout - If 0, no inactivity timer will be used. Values 1-99 are invalid. */


//==========================================================

// <h> BLE DFU connection

//==========================================================
// <o> NRF_DFU_BLE_MIN_CONN_INTERVAL - Minimum connection interval (units).
// <i> Minimum GAP connection interval, in 1.25 ms units.

#ifdef NRF_DFU_BLE_MIN_CONN_INTERVAL
#undef NRF_DFU_BLE_MIN_CONN_INTERVAL
#endif
#define NRF_DFU_BLE_MIN_CONN_INTERVAL 12


// <o> NRF_DFU_BLE_MAX_CONN_INTERVAL - Maximum connection interval (units).
// <i> Maximum GAP connection interval, in 1.25 ms units.

#ifdef NRF_DFU_BLE_MAX_CONN_INTERVAL
#undef NRF_DFU_BLE_MAX_CONN_INTERVAL
#endif
#define NRF_DFU_BLE_MAX_CONN_INTERVAL 12

// <o> NRF_DFU_BLE_CONN_SUP_TIMEOUT_MS - Supervision timeout (ms).
// <i> GAP connection supervision timeout, in milliseconds.

#ifdef NRF_DFU_BLE_CONN_SUP_TIMEOUT_MS
#undef NRF_DFU_BLE_CONN_SUP_TIMEOUT_MS
#endif
#define NRF_DFU_BLE_CONN_SUP_TIMEOUT_MS 6000


// </h>
//==========================================================

#endif /* MW_BLE_DFU_SETTINGS_H_ */
