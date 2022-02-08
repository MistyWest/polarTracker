/*
 * mw_radio_notification.h
 *
 *  Created on: Oct 25, 2017
 *      Author: KLockwood
 */

#ifndef BLE_MW_RADIO_NOTIFICATION_H_
#define BLE_MW_RADIO_NOTIFICATION_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**@brief Application radio notification event handler type. */
typedef void (*mw_radio_notification_evt_handler_t)( bool radio_active );

/**@brief Function for initializing the Radio Notification module.
 *
 * @param[in]  irq_priority   Interrupt priority for the Radio Notification interrupt handler.
 * @param[in]  distance       The time from an Active event until the radio is activated.
 * @param[in]  evt_handler    Handler to be executed when a radio notification event has been
 *                            received.
 *
 * @return     NRF_SUCCESS on successful initialization, otherwise an error code.
 */
uint32_t mw_radio_notification_init( uint32_t irq_priority, uint8_t type, uint8_t distance, mw_radio_notification_evt_handler_t evt_handler );

#endif /* BLE_MW_RADIO_NOTIFICATION_H_ */
