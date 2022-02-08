/*
 * mw_radio_notification.c
 *
 *  Created on: Oct 25, 2017
 *      Author: KLockwood
 */

#include  "sdk_common.h"

#include "nrf_soc.h"
#include "nrf_log.h"
#include "nrf_nvic.h"
#include "sdk_errors.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "mw_radio_notification.h"

static bool m_radio_active = false; /**< Current radio state. */
static mw_radio_notification_evt_handler_t m_evt_handler = NULL; /**< Application event handler for handling Radio Notification events. */

void SWI1_IRQHandler( void )
{
	m_radio_active = !m_radio_active;
	if ( m_evt_handler != NULL )
	{
		m_evt_handler(m_radio_active);
	}
}

uint32_t mw_radio_notification_init( uint32_t irq_priority, uint8_t type, uint8_t distance, mw_radio_notification_evt_handler_t evt_handler )
{
	uint32_t err_code;

	m_evt_handler = evt_handler;

	// Initialize Radio Notification software interrupt
	err_code = sd_nvic_ClearPendingIRQ(SWI1_IRQn);
	if ( err_code != NRF_SUCCESS )
	{
		return err_code;
	}

	err_code = sd_nvic_SetPriority(SWI1_IRQn, irq_priority);
	if ( err_code != NRF_SUCCESS )
	{
		return err_code;
	}

	err_code = sd_nvic_EnableIRQ(SWI1_IRQn);
	if ( err_code != NRF_SUCCESS )
	{
		return err_code;
	}

	// Configure the event
	return sd_radio_notification_cfg_set(type, distance);
}
