/*
 * mw_ble_peripheral_thread.h
 *
 *  Created on: Nov 30, 2018
 *      Author: KLockwood
 */

#ifndef MW_BLE_PERIPHERAL_THREAD_H_
#define MW_BLE_PERIPHERAL_THREAD_H_

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief - Returns current connection handle
 */
uint16_t mw_ble_get_connection_handle();


/**
 * @brief - Function to trigger BLE disconnect
 */
uint32_t mw_ble_peripheral_disconnect();


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 *
 * Note: Exposed in case we want to initialise the stack but not use BLE
 *
 */
void mw_ble_peripheral_stack_init(void);


/**
 * @brief - Thread Initialization
 */
void mw_ble_peripheral_thread_init();

#endif /* MW_BLE_PERIPHERAL_THREAD_H_ */
