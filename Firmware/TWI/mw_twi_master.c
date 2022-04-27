/*
 * mw_twim_master.c
 *
 *  TWIM Master wrapper for Nordic SDK.  Note, This is a wrapper for nfrx_twimm implements (DMA)
 *
 *
 *  Created on: Feb 13, 2019
 *      Author: Sean Edmond
 */

#include "string.h"
#include "nrf_error.h"
#include "boards.h"

#include "nrfx_twim.h"

#include "mw_twi_master.h"
#include "../CLI_Logging/mw_logging.h"

#define ENABLE_TWI_LOGGING 0

#if NRFX_CHECK(NRFX_TWIM_ENABLED)

bool twim_init[NRFX_TWIM_ENABLED_COUNT] = { false };
volatile bool twi_xfer_done[NRFX_TWIM_ENABLED_COUNT] = { false };


static mw_twim_device_config_t  m_twi_device_config[MAX_TWI_INSTANCES];

static uint8_t                  m_current_twi_instance = TWI_MASTER_ERROR;

//TODO decide if we should find a way to do this with a lower memory footprint
mw_twim_evt_handler_t mw_twim_callback[NRFX_TWIM_ENABLED_COUNT][MAX_NUM_SLAVES];



#if NRFX_CHECK(NRFX_TWIM0_ENABLED)
void nrfx_twim0_event_handler( nrfx_twim_evt_t const * p_event,
															 void * p_context )
{
	//MW_LOG_INFO("Transfer completed on TWIM0.");

	twi_xfer_done[NRFX_TWIM0_INST_IDX] = true;

	/*TODO any common logging*/

	mw_twim_evt_t mw_event;
	mw_event.type = p_event->type;
	mw_event.p_primary_buf = p_event->xfer_desc.p_primary_buf;
	mw_event.p_secondary_buf = p_event->xfer_desc.p_secondary_buf;
	mw_event.primary_length = p_event->xfer_desc.primary_length;
	mw_event.secondary_length = p_event->xfer_desc.secondary_length;

	//call callback registered for this slave
	if( mw_twim_callback[NRFX_TWIM0_INST_IDX][p_event->xfer_desc.address] != NULL )
	{
		mw_twim_callback[NRFX_TWIM0_INST_IDX][p_event->xfer_desc.address](&mw_event);
	}
}

#endif

#if NRFX_CHECK(NRFX_TWIM1_ENABLED)
void nrfx_twim1_event_handler( nrfx_twim_evt_t const * p_event,
															 void * p_context )
{
#if ENABLE_TWI_LOGGING
	MW_LOG_INFO("Transfer completed on TWIM1.");
#endif

	twi_xfer_done[NRFX_TWIM1_INST_IDX] = true;

	/*TODO any common logging */

	mw_twim_evt_t mw_event;
	mw_event.type = p_event->type;
	mw_event.p_primary_buf = p_event->xfer_desc.p_primary_buf;
	mw_event.p_secondary_buf = p_event->xfer_desc.p_secondary_buf;
	mw_event.primary_length = p_event->xfer_desc.primary_length;
	mw_event.secondary_length = p_event->xfer_desc.secondary_length;

  //call callback registered for this slave
  if( mw_twim_callback[NRFX_TWIM1_INST_IDX][p_event->xfer_desc.address] != NULL )
  {
    mw_twim_callback[NRFX_TWIM1_INST_IDX][p_event->xfer_desc.address](&mw_event);
  }
}

#endif

nrfx_twim_evt_handler_t mw_nrfx_twim_event_handler_get( int instance )
{
	nrfx_twim_evt_handler_t nrfx_twim_evt_handle = NULL;

	switch ( instance )
	{
#if NRFX_CHECK(NRFX_TWIM0_ENABLED)
	case 0:
		nrfx_twim_evt_handle = nrfx_twim0_event_handler;
		break;
#endif
#if NRFX_CHECK(NRFX_TWIM1_ENABLED)
		case 1 :
		nrfx_twim_evt_handle = nrfx_twim1_event_handler;
		break;
#endif
	}

	if ( nrfx_twim_evt_handle == NULL )
	{
		MW_LOG_INFO("TWI Master Error: nrfx_twim_evt_handle is NULL, check sdk_config.h");
	}

	return (nrfx_twim_evt_handle);
}

const int mw_twim_idx_get( int instance )
{
	int twim_idx = TWI_MASTER_ERROR;

	switch ( instance )
	{
#if NRFX_CHECK(NRFX_TWIM0_ENABLED)
	case 0:
		twim_idx = NRFX_TWIM0_INST_IDX;
		break;
#endif
#if NRFX_CHECK(NRFX_TWIM1_ENABLED)
		case 1 :
		twim_idx = NRFX_TWIM1_INST_IDX;
		break;
#endif
	}

	if ( twim_idx == TWI_MASTER_ERROR )
	{
		MW_LOG_INFO("TWI Master Error: twim_idx is not found, check sdk_config.h");
	}

	return (twim_idx);
}

const nrfx_twim_t * mw_twim_inst_get( int instance )
{
	const nrfx_twim_t * nrfx_twim = NULL;
	int twim_idx = mw_twim_idx_get(instance);

	nrfx_twim = &twim_insts[twim_idx];

	if ( nrfx_twim == NULL )
	{
		MW_LOG_INFO("TWI Master Error: nrfx_twim is NULL, check sdk_config.h");
	}

	return (nrfx_twim);
}


void mw_twi_master_rx( int twim_inst_num,
											 uint8_t address,
											 uint8_t * p_data,
											 size_t length )
{
	const nrfx_twim_t * twim_inst = mw_twim_inst_get(twim_inst_num);
	int twim_idx								  = mw_twim_idx_get(twim_inst_num);

	//wait for bus to free up
	while ( nrfx_twim_is_busy(twim_inst) )
	{
		__WFE();
	}

	twi_xfer_done[twim_idx] = false;

	APP_ERROR_CHECK(nrfx_twim_rx(twim_inst, address, p_data, length));

	//TODO... decide if it's appropriate to block here
	while ( !twi_xfer_done[twim_idx] )
	{
		__WFE();
	}
}

void mw_twi_master_tx( int twim_inst_num,
											 uint8_t address,
											 uint8_t const * p_data,
											 size_t length,
											 bool no_stop )
{

	const nrfx_twim_t * twim_inst = mw_twim_inst_get(twim_inst_num);
	int twim_idx								  = mw_twim_idx_get(twim_inst_num);

	//int twim_idx = mw_twim_idx_get(twim_inst_num);

	//wait for bus to free up
	while ( nrfx_twim_is_busy(twim_inst) )
	{
		__WFE();
	}

	twi_xfer_done[twim_idx] = false;

	//Don't block for TX
	APP_ERROR_CHECK(nrfx_twim_tx(twim_inst, address, p_data, length, no_stop));

	while( !twi_xfer_done[twim_idx] )
	{
		__WFE();
	}
}


void mw_twi_master_change_slave_address( int instance, uint8_t slave_address )
{
  int twim_idx = mw_twim_idx_get(instance);

  m_twi_device_config[twim_idx].slave_addr = slave_address;
}


void mw_twi_master_enable( int instance )
{
  const nrfx_twim_t *twim_inst = mw_twim_inst_get(instance);

  mw_twi_master_init( m_twi_device_config[instance] );
  nrfx_twim_enable(twim_inst);
}

void mw_twi_master_disable( int instance )
{
	const nrfx_twim_t *twim_inst = mw_twim_inst_get(instance);

  /* This Instance is  initialized */
  if( twim_init[instance] )
  {
    nrfx_twim_disable(twim_inst);
    mw_twi_master_uninit(instance);
  }
}


/**
 * @brief Function for uninitializing the TWIM instance.
 *
 * @param[in] twim_inst_num    Integer for the TWIM instance you want to use
 */
void mw_twi_master_uninit( int twim_inst_num )
{
	const nrfx_twim_t *twim_inst = mw_twim_inst_get(twim_inst_num);
	int twim_idx = mw_twim_idx_get(twim_inst_num);
  nrfx_twim_uninit(twim_inst);
	twim_init[twim_idx] = false;
	m_current_twi_instance = MAX_NUM_SLAVES;
}



/**
 * @brief Function for initializing the TWIM driver instance.
 *
 * @param[in] device_config    Full Device TWI Configuration
 *
 */
void mw_twi_master_init( mw_twim_device_config_t device_config )
{
	const nrfx_twim_t *twim_inst = mw_twim_inst_get(device_config.instance);
	int twim_idx 								 = mw_twim_idx_get(device_config.instance);

	/* This Instance and Device is already initialized */
	if( twim_init[twim_idx] && device_config.slave_addr == m_twi_device_config[twim_idx].slave_addr ) return;

	/*TODO add logging*/

	nrfx_twim_config_t nrfx_twim 			= NRFX_TWIM_DEFAULT_CONFIG;
	nrfx_twim.frequency								= device_config.frequency;
	nrfx_twim.hold_bus_uninit 				= device_config.hold_bus_uninit;
	nrfx_twim.interrupt_priority 			= device_config.interrupt_priority;
	nrfx_twim.scl 										= device_config.scl;
	nrfx_twim.sda 										= device_config.sda;

	//set the callback to Driver
	mw_twim_callback[twim_idx][device_config.slave_addr] = device_config.mw_twim_handler;

  memcpy( &m_twi_device_config[device_config.instance], &device_config, sizeof(device_config) );

	//only call nrfx_spi_init() on first initialization
	if ( twim_init[twim_idx] == false )
	{
		nrfx_twim_evt_handler_t twim_evt_handler = mw_nrfx_twim_event_handler_get(device_config.instance);

		APP_ERROR_CHECK(nrfx_twim_init(twim_inst, &nrfx_twim, twim_evt_handler, device_config.context));

		twim_init[twim_idx] = true;

		m_current_twi_instance = device_config.instance;
	}

	twi_xfer_done[twim_idx] = true;
}

#endif

