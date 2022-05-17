/*
 * mw_ble_central_thread.h
 *
 *  Created on: Nov 30, 2018
 *      Author: KLockwood
 */

#ifndef THREADS_MW_BLE_CENTRAL_THREAD_H_
#define THREADS_MW_BLE_CENTRAL_THREAD_H_

#include "ble_gap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Function for starting scanning. */
void scan_start();


/**
 * @brief - Used to disconnect from a specific Peer
 */
void mw_ble_central_disconnect_from_peer();


/**
 * @brief - Used to connect to a specific Peer
 */
void mw_ble_central_connect_to_peer( ble_gap_addr_t peer_addr );

/**
 * @brief - Thread Initialization
 */
void mw_ble_central_thread_init();

#endif /* THREADS_MW_BLE_CENTRAL_THREAD_H_ */
