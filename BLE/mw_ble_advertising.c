/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include  "sdk_common.h"
#include  "nordic_common.h"

#include "ble_advdata.h"
#include "nrf_soc.h"
#include "nrf_log.h"
#include "nrf_fstorage.h"
#include "sdk_errors.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "mw_ble_settings.h"
#include "mw_ble_advertising.h"

#define BLE_ADV_MODES MW_BLE_ADV_MODE_COUNT /**< Total number of possible advertising modes. */

static void on_connected( mw_ble_advertising_t * const p_advertising, ble_evt_t const * p_ble_evt );
static void on_disconnected( mw_ble_advertising_t * const p_advertising, ble_evt_t const * p_ble_evt );
static void on_terminated( mw_ble_advertising_t * const p_advertising, ble_evt_t const * p_ble_evt );

static bool whitelist_has_entries( mw_ble_advertising_t * const p_advertising );
static bool addr_is_valid( uint8_t const * const p_addr );
static mw_ble_adv_mode_t adv_mode_next_get( mw_ble_adv_mode_t adv_mode );

MW_BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */

/**@brief   Function for handling BLE events.
 *
 * @details This function must be called from the BLE stack event dispatcher for
 *          the module to handle BLE events that are relevant for the Advertising Module.
 *
 * @param[in] p_ble_evt     BLE stack event.
 * @param[in] p_adv         Advertising module instance.
 */
//@ble
void mw_ble_advertising_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context )
{
	mw_ble_advertising_t * p_advertising = (mw_ble_advertising_t *) p_context;

	switch ( p_ble_evt->header.evt_id )
	{
	case BLE_GAP_EVT_CONNECTED:
		on_connected(p_advertising, p_ble_evt);
		break;

		// Upon disconnection, whitelist will be activated and direct advertising is started.
	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnected(p_advertising, p_ble_evt);
		break;

		// Upon terminated advertising (time-out), the next advertising mode is started.
	case BLE_GAP_EVT_ADV_SET_TERMINATED:
		on_terminated(p_advertising, p_ble_evt);
		break;

	default:
		break;
	}
}

/**@brief   Function for handling system events.
 *
 * @details This function must be called to handle system events that are relevant
 *          for the Advertising Module. Specifically, the advertising module can not use the
 *          softdevice as long as there are pending writes to the flash memory. This
 *          event handler is designed to delay advertising until there is no flash operation.
 *
 * @param[in] sys_evt       System event.
 * @param[in] p_adv         Advertising module instance.
 */
void mw_ble_advertising_on_sys_evt( uint32_t evt_id, void * p_context )
{
	switch ( evt_id )
	{
	default:
		// No implementation needed.
		break;
	}
}

/**@brief Function for handling the Connected event.
 *
 * @param[in] p_ble_evt Event received from the BLE stack.
 */
static void on_connected( mw_ble_advertising_t * const p_advertising, ble_evt_t const * p_ble_evt )
{
	if ( p_ble_evt->evt.gap_evt.params.connected.role == BLE_GAP_ROLE_PERIPH )
	{
		p_advertising->current_slave_link_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	}
}

/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_advertising Advertising module instance.
 * @param[in] p_ble_evt Event received from the BLE stack.
 */
static void on_disconnected( mw_ble_advertising_t * const p_advertising, ble_evt_t const * p_ble_evt )
{
	uint32_t ret;

	p_advertising->whitelist_temporarily_disabled = false;

	if ( p_ble_evt->evt.gap_evt.conn_handle == p_advertising->current_slave_link_conn_handle&&
	p_advertising->adv_modes_config.ble_adv_on_disconnect_disabled == false )
	{
		ret = mw_ble_advertising_start(MW_BLE_ADV_MODE_DIRECTED_HIGH_DUTY);
		if ( (ret != NRF_SUCCESS) && (p_advertising->error_handler != NULL) )
		{
			p_advertising->error_handler(ret);
		}
	}
}

/**@brief Function for handling the Timeout event.
 *
 * @param[in] p_advertising Advertising module instance.
 * @param[in] p_ble_evt Event received from the BLE stack.
 */
static void on_terminated( mw_ble_advertising_t * const p_advertising, ble_evt_t const * p_ble_evt )
{
	ret_code_t ret;

	if ( p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_SET_TERMINATED )
	{
		// Nothing to do.
		return;
	}

	if ( p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT
			|| p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_LIMIT_REACHED )
	{
		// Start advertising in the next mode.
		ret = mw_ble_advertising_start(adv_mode_next_get(p_advertising->adv_mode_current));

		if ( (ret != NRF_SUCCESS) && (p_advertising->error_handler != NULL) )
		{
			p_advertising->error_handler(ret);
		}
	}
}

/** @brief Function to determine if a flash write operation in in progress.
 *
 * @return true if a flash operation is in progress, false if not.
 */
//static bool flash_access_in_progress()
//{
//	return nrf_fstorage_is_busy(NULL);
//}

/**@brief Get the next available advertising mode.
 *
 * @param[in] p_advertising Advertising module instance.
 * @param[in] adv_mode Requested advertising mode.
 *
 * @returns adv_mode if possible, or the best available mode if not.
 */
static mw_ble_adv_mode_t adv_mode_next_avail_get( mw_ble_advertising_t * const p_advertising, mw_ble_adv_mode_t adv_mode )
{
	bool peer_addr_is_valid = addr_is_valid(p_advertising->peer_address.addr);

	// If a mode is disabled, continue to the next mode.

	switch ( adv_mode )
	{
	case MW_BLE_ADV_MODE_DIRECTED_HIGH_DUTY:
		if ( (p_advertising->adv_modes_config.ble_adv_directed_high_duty_enabled) && (!p_advertising->adv_modes_config.ble_adv_extended_enabled)
				&& (peer_addr_is_valid) )
		{
			return MW_BLE_ADV_MODE_DIRECTED_HIGH_DUTY;
		}
		// Fallthrough.

	case MW_BLE_ADV_MODE_DIRECTED:
		if ( (p_advertising->adv_modes_config.ble_adv_directed_enabled) && peer_addr_is_valid )
		{
			return MW_BLE_ADV_MODE_DIRECTED;
		}
		// Fallthrough.

  case MW_BLE_ADV_MODE_ULTRA_FAST:
    if ( p_advertising->adv_modes_config.ble_adv_ultra_fast_enabled )
    {
      //NRF_LOG_INFO("MW-Advertising: BLE_ADV_MODE_FAST \r\n");
      return MW_BLE_ADV_MODE_ULTRA_FAST;
    }
    // Fallthrough.

	case MW_BLE_ADV_MODE_FAST:
		if ( p_advertising->adv_modes_config.ble_adv_fast_enabled )
		{
			//NRF_LOG_INFO("MW-Advertising: BLE_ADV_MODE_FAST \r\n");
			return MW_BLE_ADV_MODE_FAST;
		}
		// Fallthrough.

	case MW_BLE_ADV_MODE_MED:
		if ( p_advertising->adv_modes_config.ble_adv_med_enabled )
		{
			//NRF_LOG_INFO("MW-Advertising: BLE_ADV_MODE_MED \r\n");
			return MW_BLE_ADV_MODE_MED;
		}
		// Fallthrough.
	case MW_BLE_ADV_MODE_SLOW:
		if ( p_advertising->adv_modes_config.ble_adv_slow_enabled )
		{
			//NRF_LOG_INFO("MW-Advertising: BLE_ADV_MODE_SLOW \r\n");
			return MW_BLE_ADV_MODE_SLOW;
		}
		// Fallthrough.

	default:
		NRF_LOG_INFO("MW-Advertising: BLE_ADV_MODE_IDLE \r\n");
		return MW_BLE_ADV_MODE_IDLE;
	}
}

/**@brief Function for starting high duty directed advertising.
 *
 * @param[in]  p_advertising Advertising instance.
 * @param[out] p_adv_params Advertising parameters.
 *
 * @return NRF_SUCCESS
 */
static ret_code_t set_adv_mode_directed_high_duty( mw_ble_advertising_t * const p_advertising, ble_gap_adv_params_t * p_adv_params )
{
	p_advertising->adv_evt = MW_BLE_ADV_EVT_DIRECTED_HIGH_DUTY;
	p_advertising->p_adv_data = NULL;

	p_adv_params->p_peer_addr = &(p_advertising->peer_address);
	p_adv_params->interval = 0;
	p_adv_params->properties.type =
	BLE_GAP_ADV_TYPE_CONNECTABLE_NONSCANNABLE_DIRECTED_HIGH_DUTY_CYCLE;
	p_adv_params->duration = BLE_GAP_ADV_TIMEOUT_HIGH_DUTY_MAX;

	return NRF_SUCCESS;
}

/**@brief Function for starting directed slow advertising.
 *
 * @param[in]  p_advertising Advertising module instance.
 * @param[out] p_adv_params Advertising parameters.
 *
 * @return NRF_SUCCESS
 */
static ret_code_t set_adv_mode_directed( mw_ble_advertising_t * const p_advertising, ble_gap_adv_params_t * p_adv_params )
{
	p_advertising->adv_evt = MW_BLE_ADV_EVT_DIRECTED;
#if !defined (S112)
	if ( p_advertising->adv_modes_config.ble_adv_extended_enabled )
	{
		p_adv_params->properties.type =
		BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_DIRECTED;
	} else
	{
#endif // !defined (S112)
		p_adv_params->properties.type =
		BLE_GAP_ADV_TYPE_CONNECTABLE_NONSCANNABLE_DIRECTED;
#if !defined (S112)
	}
#endif // !defined (S112)
	p_adv_params->duration = p_advertising->adv_modes_config.ble_adv_directed_timeout;

	p_advertising->p_adv_data = NULL;

	p_adv_params->p_peer_addr = &p_advertising->peer_address;
	p_adv_params->interval = p_advertising->adv_modes_config.ble_adv_directed_interval;

	return NRF_SUCCESS;
}

/**@brief Function for indicating whether to use whitelist for advertising.
 *
 * @param[in]  p_advertising Advertising module instance.
 *
 * @return Whether to use whitelist.
 */
static bool use_whitelist( mw_ble_advertising_t * const p_advertising )
{
	return ((p_advertising->adv_modes_config.ble_adv_whitelist_enabled) && (!p_advertising->whitelist_temporarily_disabled)
			&& (whitelist_has_entries(p_advertising)));
}

/**@brief Function for setting new advertising flags in the advertising parameters.
 *
 * @param[in]  p_advertising Advertising module instance.
 * @param[in]  flags         New flags.
 *
 * @return Any error from @ref sd_ble_gap_adv_set_configure.
 */
static ret_code_t flags_set( mw_ble_advertising_t * const p_advertising, uint8_t flags )
{
	uint8_t * p_flags = ble_advdata_parse(p_advertising->adv_data.adv_data.p_data, p_advertising->adv_data.adv_data.len,
	BLE_GAP_AD_TYPE_FLAGS);

	if ( p_flags != NULL )
	{
		*p_flags = flags;
	}

	return sd_ble_gap_adv_set_configure(&p_advertising->adv_handle, &p_advertising->adv_data, &p_advertising->adv_params);
}



/**@brief Function for starting ULTRA fast advertising.
 *
 * @param[in]  p_advertising Advertising module instance.
 * @param[out] p_adv_params Advertising parameters.
 *
 * @return NRF_SUCCESS or an error from @ref flags_set().
 */
static ret_code_t set_adv_mode_ultra_fast( mw_ble_advertising_t * const p_advertising, ble_gap_adv_params_t * p_adv_params )
{
  ret_code_t ret;

  p_adv_params->interval = p_advertising->adv_modes_config.ble_adv_ultra_fast_interval;
  p_adv_params->duration = p_advertising->adv_modes_config.ble_adv_ultra_fast_timeout;

#if !defined (S112)
  if ( p_advertising->adv_modes_config.ble_adv_extended_enabled )
  {
    p_advertising->adv_params.properties.type =
    BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
  } else
  {
#endif // !defined (S112)
    p_advertising->adv_params.properties.type =
    BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
#if !defined (S112)
  }
#endif // !defined (S112)

  if ( use_whitelist(p_advertising) )
  {
    p_adv_params->filter_policy = BLE_GAP_ADV_FP_FILTER_CONNREQ;

    // Set correct flags.
    ret = flags_set(p_advertising, BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
    VERIFY_SUCCESS(ret);

    p_advertising->adv_evt = MW_BLE_ADV_EVT_ULTRA_FAST_WHITELIST;
  } else
  {
    p_advertising->adv_evt = MW_BLE_ADV_EVT_ULTRA_FAST;
  }
  p_advertising->p_adv_data = &(p_advertising->adv_data);
  return NRF_SUCCESS;
}


/**@brief Function for starting fast advertising.
 *
 * @param[in]  p_advertising Advertising module instance.
 * @param[out] p_adv_params Advertising parameters.
 *
 * @return NRF_SUCCESS or an error from @ref flags_set().
 */
static ret_code_t set_adv_mode_fast( mw_ble_advertising_t * const p_advertising, ble_gap_adv_params_t * p_adv_params )
{
	ret_code_t ret;

	p_adv_params->interval = p_advertising->adv_modes_config.ble_adv_fast_interval;
	p_adv_params->duration = p_advertising->adv_modes_config.ble_adv_fast_timeout;

#if !defined (S112)
	if ( p_advertising->adv_modes_config.ble_adv_extended_enabled )
	{
		p_advertising->adv_params.properties.type =
		BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
	} else
	{
#endif // !defined (S112)
		p_advertising->adv_params.properties.type =
		BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
#if !defined (S112)
	}
#endif // !defined (S112)

	if ( use_whitelist(p_advertising) )
	{
		p_adv_params->filter_policy = BLE_GAP_ADV_FP_FILTER_CONNREQ;

		// Set correct flags.
		ret = flags_set(p_advertising, BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
		VERIFY_SUCCESS(ret);

		p_advertising->adv_evt = MW_BLE_ADV_EVT_FAST_WHITELIST;
	} else
	{
		p_advertising->adv_evt = MW_BLE_ADV_EVT_FAST;
	}
	p_advertising->p_adv_data = &(p_advertising->adv_data);
	return NRF_SUCCESS;
}


/**@brief Function for starting medium advertising.
 *
 * @param[in]  p_advertising Advertising module instance.
 * @param[out] p_adv_params Advertising parameters.
 *
 * @return NRF_SUCCESS or an error from @ref flags_set().
 */
static ret_code_t set_adv_mode_med( mw_ble_advertising_t * const p_advertising, ble_gap_adv_params_t * p_adv_params )
{
	ret_code_t ret;

	p_adv_params->interval = p_advertising->adv_modes_config.ble_adv_med_interval;
	p_adv_params->duration = p_advertising->adv_modes_config.ble_adv_med_timeout;

#if !defined (S112)
	if ( p_advertising->adv_modes_config.ble_adv_extended_enabled )
	{
		p_advertising->adv_params.properties.type =
		BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
	} else
	{
#endif // !defined (S112)
		p_advertising->adv_params.properties.type =
		BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
#if !defined (S112)
	}
#endif // !defined (S112)

	if ( use_whitelist(p_advertising) )
	{
		p_adv_params->filter_policy = BLE_GAP_ADV_FP_FILTER_CONNREQ;

		// Set correct flags.
		ret = flags_set(p_advertising, BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
		VERIFY_SUCCESS(ret);

		p_advertising->adv_evt = MW_BLE_ADV_EVT_MED_WHITELIST;
	} else
	{
		p_advertising->adv_evt = MW_BLE_ADV_EVT_MED;
	}
	p_advertising->p_adv_data = &(p_advertising->adv_data);
	return NRF_SUCCESS;
}

/**@brief Function for starting slow advertising.
 *
 * @param[in]  p_advertising Advertising module instance.
 * @param[out] p_adv_params Advertising parameters.
 *
 * @return NRF_SUCCESS or an error from @ref flags_set().
 */
static ret_code_t set_adv_mode_slow( mw_ble_advertising_t * const p_advertising, ble_gap_adv_params_t * p_adv_params )
{
	ret_code_t ret;

	p_adv_params->interval = p_advertising->adv_modes_config.ble_adv_slow_interval;
	p_adv_params->duration = p_advertising->adv_modes_config.ble_adv_slow_timeout;

#if !defined (S112)
	if ( p_advertising->adv_modes_config.ble_adv_extended_enabled )
	{
		p_advertising->adv_params.properties.type =
		BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
	} else
	{
#endif // !defined (S112)
		p_advertising->adv_params.properties.type =
		BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
#if !defined (S112)
	}
#endif // !defined (S112)

	if ( use_whitelist(p_advertising) )
	{
		p_adv_params->filter_policy = BLE_GAP_ADV_FP_FILTER_CONNREQ;

		// Set correct flags.
		ret = flags_set(p_advertising, BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
		VERIFY_SUCCESS(ret);

		p_advertising->adv_evt = MW_BLE_ADV_EVT_SLOW_WHITELIST;
	} else
	{
		p_advertising->adv_evt = MW_BLE_ADV_EVT_SLOW;
	}
	p_advertising->p_adv_data = &(p_advertising->adv_data);
	return NRF_SUCCESS;
}

/**@brief Function for checking if the whitelist is in use.
 *
 * @param[in] p_advertising Advertising module instance.
 */
static bool whitelist_has_entries( mw_ble_advertising_t * const p_advertising )
{
	return p_advertising->whitelist_in_use;
}

/**@brief Function for checking if an address is valid.
 *
 * @param[in] p_addr Pointer to a bluetooth address.
 */
static bool addr_is_valid( uint8_t const * const p_addr )
{
	for ( uint32_t i = 0; i < BLE_GAP_ADDR_LEN; i++ )
	{
		if ( p_addr[i] != 0 )
		{
			return true;
		}
	}
	return false;
}

/**@brief Function for checking the next advertising mode.
 *
 * @param[in] adv_mode Current advertising mode.
 */
static mw_ble_adv_mode_t adv_mode_next_get( mw_ble_adv_mode_t adv_mode )
{
	return (mw_ble_adv_mode_t) ((adv_mode + 1) % BLE_ADV_MODES);
}

void ble_advertising_conn_cfg_tag_set( mw_ble_advertising_t * const p_advertising, uint8_t ble_cfg_tag )
{
	p_advertising->conn_cfg_tag = ble_cfg_tag;
}

/**@brief Function for checking that a phy define value matches one of the valid phys from the SD.
 *
 * @param[in]  PHY to be validated.
 *
 * @retval true  If the PHY value is valid (1mbit, 2mbit, coded).
 * @retval false If the PHY value is invalid.
 */
static bool phy_is_valid( uint32_t const * const p_phy )
{
	if ( (*p_phy) == BLE_GAP_PHY_1MBPS || (*p_phy) == BLE_GAP_PHY_2MBPS
#if defined (S140)
	|| (*p_phy) == BLE_GAP_PHY_CODED
#endif // !defined (S140)
	)
	{
		return true;
	} else
	{
		return false;
	}
}

void mw_set_advertising_power( int8_t tx_power )
{
	APP_ERROR_CHECK(sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, tx_power));
}

uint32_t mw_ble_advertising_start( mw_ble_adv_mode_t advertising_mode )
{
	uint32_t ret;

	if ( m_advertising.initialized == false )
	{
		return NRF_ERROR_INVALID_STATE;
	}

	m_advertising.adv_mode_current = advertising_mode;

	// Delay starting advertising until the flash operations are complete.
//	if ( flash_access_in_progress() )
//	{
//		m_advertising.advertising_start_pending = true;
//		return NRF_ERROR_BUSY;
//	}

	memset(&m_advertising.peer_address, 0, sizeof(m_advertising.peer_address));

	if ( ((m_advertising.adv_modes_config.ble_adv_directed_high_duty_enabled)
			&& (m_advertising.adv_mode_current == MW_BLE_ADV_MODE_DIRECTED_HIGH_DUTY))
			|| ((m_advertising.adv_modes_config.ble_adv_directed_enabled) && (m_advertising.adv_mode_current == MW_BLE_ADV_MODE_DIRECTED_HIGH_DUTY))
			|| ((m_advertising.adv_modes_config.ble_adv_directed_enabled) && (m_advertising.adv_mode_current == MW_BLE_ADV_MODE_DIRECTED)) )
	{
		if ( m_advertising.evt_handler != NULL )
		{
			m_advertising.peer_addr_reply_expected = true;
			m_advertising.evt_handler(MW_BLE_ADV_EVT_PEER_ADDR_REQUEST);
		} else
		{
			m_advertising.peer_addr_reply_expected = false;
		}
	}

	m_advertising.adv_mode_current = adv_mode_next_avail_get(&m_advertising, advertising_mode);

	// Fetch the whitelist.
	if ( (m_advertising.evt_handler != NULL)
			&& (m_advertising.adv_mode_current == MW_BLE_ADV_MODE_ULTRA_FAST || m_advertising.adv_mode_current == MW_BLE_ADV_MODE_FAST || m_advertising.adv_mode_current == MW_BLE_ADV_MODE_MED
					|| m_advertising.adv_mode_current == MW_BLE_ADV_MODE_SLOW) && (m_advertising.adv_modes_config.ble_adv_whitelist_enabled)
			&& (!m_advertising.whitelist_temporarily_disabled) )
	{
		m_advertising.whitelist_in_use = false;
		m_advertising.whitelist_reply_expected = true;
		m_advertising.evt_handler(MW_BLE_ADV_EVT_WHITELIST_REQUEST);
	} else
	{
		m_advertising.whitelist_reply_expected = false;
	}

	// Initialize advertising parameters with default values.
	memset(&m_advertising.adv_params, 0, sizeof(m_advertising.adv_params));

	m_advertising.adv_params.properties.type =
	BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;

	// Use 1MBIT as primary phy if no phy was selected.
	if ( phy_is_valid(&m_advertising.adv_modes_config.ble_adv_primary_phy) )
	{
		m_advertising.adv_params.primary_phy = m_advertising.adv_modes_config.ble_adv_primary_phy;
	} else
	{
		m_advertising.adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
	}

	if ( m_advertising.adv_modes_config.ble_adv_extended_enabled )
	{
		// Use 1MBIT as secondary phy if no phy was selected.
		if ( phy_is_valid(&m_advertising.adv_modes_config.ble_adv_primary_phy) )
		{
			m_advertising.adv_params.secondary_phy = m_advertising.adv_modes_config.ble_adv_secondary_phy;
		} else
		{
			m_advertising.adv_params.secondary_phy = BLE_GAP_PHY_1MBPS;
		}
	}
	m_advertising.adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;

	// Set advertising parameters and events according to selected advertising mode.
	switch ( m_advertising.adv_mode_current )
	{
	case MW_BLE_ADV_MODE_DIRECTED_HIGH_DUTY:
		ret = set_adv_mode_directed_high_duty(&m_advertising, &m_advertising.adv_params);
		break;

	case MW_BLE_ADV_MODE_DIRECTED:
		ret = set_adv_mode_directed(&m_advertising, &m_advertising.adv_params);
		break;

  case MW_BLE_ADV_MODE_ULTRA_FAST:
    ret = set_adv_mode_ultra_fast(&m_advertising, &m_advertising.adv_params);
    break;

	case MW_BLE_ADV_MODE_FAST:
		ret = set_adv_mode_fast(&m_advertising, &m_advertising.adv_params);
		break;

	case MW_BLE_ADV_MODE_MED:
		ret = set_adv_mode_med(&m_advertising, &m_advertising.adv_params);
		break;

	case MW_BLE_ADV_MODE_SLOW:
		ret = set_adv_mode_slow(&m_advertising, &m_advertising.adv_params);
		break;

	case MW_BLE_ADV_MODE_IDLE:
		m_advertising.adv_evt = MW_BLE_ADV_EVT_IDLE;
		break;

	default:
		break;
	}

	if ( m_advertising.adv_mode_current != MW_BLE_ADV_MODE_IDLE )
	{

		ret = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, m_advertising.p_adv_data, &m_advertising.adv_params);
		if ( ret != NRF_SUCCESS )
		{
			return ret;
		}
		ret = sd_ble_gap_adv_start(m_advertising.adv_handle, m_advertising.conn_cfg_tag);

		if ( ret != NRF_SUCCESS )
		{
			return ret;
		}
	}

	if ( m_advertising.evt_handler != NULL )
	{
		m_advertising.evt_handler(m_advertising.adv_evt);
	}

	return NRF_SUCCESS;
}

uint32_t ble_advertising_peer_addr_reply( mw_ble_advertising_t * const p_advertising, ble_gap_addr_t * p_peer_address )
{
	if ( !p_advertising->peer_addr_reply_expected )
	{
		return NRF_ERROR_INVALID_STATE;
	}

	p_advertising->peer_addr_reply_expected = false;

	memcpy(&p_advertising->peer_address, p_peer_address, sizeof(p_advertising->peer_address));

	return NRF_SUCCESS;
}

uint32_t ble_advertising_whitelist_reply( mw_ble_advertising_t * const p_advertising, ble_gap_addr_t const * p_gap_addrs, uint32_t addr_cnt,
		ble_gap_irk_t const * p_gap_irks, uint32_t irk_cnt )
{
	if ( !p_advertising->whitelist_reply_expected )
	{
		return NRF_ERROR_INVALID_STATE;
	}

	p_advertising->whitelist_reply_expected = false;
	p_advertising->whitelist_in_use = ((addr_cnt > 0) || (irk_cnt > 0));

	return NRF_SUCCESS;
}

uint32_t ble_advertising_restart_without_whitelist( mw_ble_advertising_t * const p_advertising )
{
	ret_code_t ret;

	(void) sd_ble_gap_adv_stop(p_advertising->adv_handle);

	p_advertising->whitelist_temporarily_disabled = true;
	p_advertising->whitelist_in_use = false;
	p_advertising->adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
	// Set correct flags.
	ret = flags_set(p_advertising, BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
	VERIFY_SUCCESS(ret);

	ret = mw_ble_advertising_start(m_advertising.adv_mode_current);
	if ( (ret != NRF_SUCCESS) && (p_advertising->error_handler != NULL) )
	{
		p_advertising->error_handler(ret);
	}

	return NRF_SUCCESS;
}

void ble_advertising_modes_config_set( mw_ble_advertising_t * const p_advertising, mw_ble_adv_modes_config_t const * const p_adv_modes_config )
{
	p_advertising->adv_modes_config = *p_adv_modes_config;
}

/**@brief Function for checking if an advertising module configuration is legal.
 *
 * @details Advertising module can not be initialized if high duty directed advertising is used
 *          together with extended advertising.
 *
 * @param[in] p_config Pointer to the configuration.
 *
 * @return True  If the configuration is valid.
 * @return False If the configuration is invalid.
 */
static bool config_is_valid( mw_ble_adv_modes_config_t const * const p_config )
{
	if ( (p_config->ble_adv_directed_high_duty_enabled == true) && (p_config->ble_adv_extended_enabled == true) )
	{
		return false;
	}
#if !defined (S140)
	else if ( p_config->ble_adv_primary_phy == BLE_GAP_PHY_CODED || p_config->ble_adv_secondary_phy == BLE_GAP_PHY_CODED )
	{
		return false;
	}
#endif // !defined (S140)
	else
	{
		return true;
	}
}


void mw_ble_advertising_default_config( mw_ble_advertising_init_t * adv_config )
{
	adv_config->advdata.name_type 						= BLE_ADVDATA_FULL_NAME;
	adv_config->advdata.include_appearance 		= true;
	adv_config->advdata.flags 								= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

  adv_config->config.ble_adv_ultra_fast_enabled   = true;
  adv_config->config.ble_adv_ultra_fast_interval  = MW_ULTRA_FAST_ADV_INTERVAL;
  adv_config->config.ble_adv_ultra_fast_timeout   = MW_ULTRA_FAST_ADV_TIMEOUT;

	adv_config->config.ble_adv_fast_enabled  	= true;
	adv_config->config.ble_adv_fast_interval 	= MW_FAST_ADV_INTERVAL;
	adv_config->config.ble_adv_fast_timeout  	= MW_FAST_ADV_TIMEOUT;

	adv_config->config.ble_adv_med_enabled  	= true;
	adv_config->config.ble_adv_med_interval 	= MW_MED_ADV_INTERVAL;
	adv_config->config.ble_adv_med_timeout 		= MW_MED_ADV_TIMEOUT;

	adv_config->config.ble_adv_slow_enabled 	= true;
	adv_config->config.ble_adv_slow_interval 	= MW_SLOW_ADV_INTERVAL;
	adv_config->config.ble_adv_slow_timeout 	= MW_SLOW_ADV_TIMEOUT;

	adv_config->config.ble_adv_primary_phy 		= BLE_GAP_PHY_1MBPS; // Must be changed to connect in long range. (BLE_GAP_PHY_CODED)
	adv_config->config.ble_adv_primary_phy		= BLE_GAP_PHY_1MBPS;
}


/**
 * @brief - Initialize Advertising Module
 */
uint32_t mw_ble_advertising_init( mw_ble_advertising_init_t const * const p_init )
{
	uint32_t ret;
	if ( p_init == NULL )
	{
		return NRF_ERROR_NULL;
	}
	if ( !config_is_valid(&p_init->config) )
	{
		return NRF_ERROR_INVALID_PARAM;
	}

	m_advertising.adv_mode_current = MW_BLE_ADV_MODE_IDLE;
	m_advertising.adv_modes_config = p_init->config;
	m_advertising.conn_cfg_tag = BLE_CONN_CFG_TAG_DEFAULT;
	m_advertising.evt_handler = p_init->evt_handler;
	m_advertising.error_handler = p_init->error_handler;
	m_advertising.current_slave_link_conn_handle = BLE_CONN_HANDLE_INVALID;
	m_advertising.p_adv_data = &m_advertising.adv_data;

	memset(&m_advertising.peer_address, 0, sizeof(m_advertising.peer_address));

	// Copy advertising data.
	if ( !m_advertising.initialized )
	{
		m_advertising.adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
	}
	m_advertising.adv_data.adv_data.p_data = m_advertising.enc_advdata;
	m_advertising.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

	ret = ble_advdata_encode(&p_init->advdata, m_advertising.enc_advdata, &m_advertising.adv_data.adv_data.len);
	VERIFY_SUCCESS(ret);

	if ( p_init->srdata_used )
	{
		m_advertising.adv_data.scan_rsp_data.p_data = m_advertising.enc_scan_rsp_data;
		m_advertising.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

		ret = ble_advdata_encode(&p_init->srdata, m_advertising.adv_data.scan_rsp_data.p_data, &m_advertising.adv_data.scan_rsp_data.len);
		VERIFY_SUCCESS(ret);
	} else
	{
		m_advertising.adv_data.scan_rsp_data.p_data = NULL;
		m_advertising.adv_data.scan_rsp_data.len = 0;
	}

	// Configure a initial advertising configuration. The advertising data and and advertising
	// parameters will be changed later when we call @ref ble_advertising_start, but must be set
	// to legal values here to define an advertising handle.
	m_advertising.adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
	m_advertising.adv_params.duration = m_advertising.adv_modes_config.ble_adv_fast_timeout;
	m_advertising.adv_params.properties.type =
	BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
	m_advertising.adv_params.p_peer_addr = NULL;
	m_advertising.adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
	m_advertising.adv_params.interval = m_advertising.adv_modes_config.ble_adv_fast_interval;

	ret = sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, NULL, &m_advertising.adv_params);
	VERIFY_SUCCESS(ret);

	m_advertising.initialized = true;


	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
	return ret;
}

