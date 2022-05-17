/*
 * mw_twi.c
 *
 *  TWI wrapper for Nordic SDK.  Note, This is a wrapper for the none
 *  DMA version version of TWI/I2C.  Note, nfrx_twim implements the DMA
 *  version
 *
 *  Created on: January 3, 2019
 *      Author: Sean Edmond
 */

#include "mw_twi.h"

#include "../CLI_Logging/mw_logging.h"

#define TWI_ERROR				99

#if NRFX_CHECK(NRFX_TWI_ENABLED)

bool twi_init[NRFX_TWI_ENABLED_COUNT] = { false };
bool xfer_done[NRFX_TWI_ENABLED_COUNT] = { false };

//TODO decide if twe should find a way to do this with a lower memory footprint
mw_twi_evt_handler_t mw_twi_callback[NRFX_TWI_ENABLED_COUNT][MAX_NUM_SLAVES];




#if NRFX_CHECK(NRFX_TWI0_ENABLED)
void nrfx_twi0_event_handler(nrfx_twi_evt_t const * p_event,
                            void *                 p_context)
{
    MW_LOG_INFO("Transfer completed on TWI0.");

    xfer_done[NRFX_TWI0_INST_IDX] = true;

    /*TODO any common logging*/

    mw_twi_evt_t mw_event;
    mw_event.type = p_event->type;
    mw_event.p_primary_buf = p_event->xfer_desc.p_primary_buf;
    mw_event.p_secondary_buf = p_event->xfer_desc.p_secondary_buf;
    mw_event.primary_length = p_event->xfer_desc.primary_length;
    mw_event.secondary_length = p_event->xfer_desc.secondary_length;

    //call callback registered for this slave
    mw_twi_callback[NRFX_TWI0_INST_IDX][p_event->xfer_desc.address](&mw_event);
}

#endif

#if NRFX_CHECK(NRFX_TWI1_ENABLED)
void nrfx_twi1_event_handler(nrfx_twi_evt_t const * p_event,
                                                void *                 p_context)
{
    NRF_LOG_INFO("Transfer completed on TWI1.");

    xfer_done[NRFX_TWI1_INST_IDX] = true;

    /*TODO any common logging */

    mw_twi_evt_t mw_event;
    mw_event.type = p_event->type;
    mw_event.p_primary_buf = p_event->xfer_desc.p_primary_buf;
    mw_event.p_secondary_buf = p_event->xfer_desc.p_secondary_buf;
    mw_event.primary_length = p_event->xfer_desc.primary_length;
    mw_event.secondary_length = p_event->xfer_desc.secondary_length;

    //call callback registered for this slave
    mw_twi_callback[NRFX_TWI1_INST_IDX][p_event->xfer_desc.address](&mw_event);
}

#endif


nrfx_twi_evt_handler_t mw_nrfx_twi_event_handler_get( int instance )
{
	nrfx_twi_evt_handler_t nrfx_twi_evt_handle = NULL;

	switch ( instance )
	{
#if NRFX_CHECK(NRFX_TWI0_ENABLED)
	case 0:
		nrfx_twi_evt_handle = nrfx_twi0_event_handler;
		break;
#endif
#if NRFX_CHECK(NRFX_TWI1_ENABLED)
		case 1 :
		nrfx_twi_evt_handle = nrfx_twi1_event_handler;
		break;
#endif
	}

	if ( nrfx_twi_evt_handle == NULL )
	{
		MW_LOG_INFO("TWI Error: TWI Event Null Error");
	}
	return (nrfx_twi_evt_handle);
}

const int mw_twi_idx_get( int instance )
{
	int twi_idx = TWI_ERROR;

	switch ( instance )
	{
#if NRFX_CHECK(NRFX_TWI0_ENABLED)
	case 0:
		twi_idx = NRFX_TWI0_INST_IDX;
		break;
#endif
#if NRFX_CHECK(NRFX_TWI1_ENABLED)
		case 1 :
		twi_idx = NRFX_TWI1_INST_IDX;
		break;
#endif
	}

	if ( twi_idx == TWI_ERROR )
	{
		MW_LOG_INFO("TWI Error: TWI Index Get Error");
	}

	return (twi_idx);
}


const nrfx_twi_t* mw_twi_inst_get( int instance )
{
	const nrfx_twi_t* nrfx_twi = NULL;
	int twi_idx = mw_twi_idx_get(instance);
	nrfx_twi = &twi_insts[twi_idx];

	if ( nrfx_twi == NULL )
	{
		MW_LOG_INFO("TWI Error: TWI Instance Null Error");
	}

	return (nrfx_twi);
}





void mw_twi_enable( int instance )
{
	const nrfx_twi_t *twi_inst = mw_twi_inst_get(instance);
	nrfx_twi_enable(twi_inst);
}

void mw_twi_disable( int instance )
{
	const nrfx_twi_t *twi_inst = mw_twi_inst_get(instance);
	nrfx_twi_disable(twi_inst);
}



void mw_twi_init(int twi_inst_num,
                 int slave_addr,
                 mw_twi_config_t const * mw_twi_config,
                 mw_twi_evt_handler_t    mw_twi_handler,
                 void *                  p_context)
{

	const nrfx_twi_t *twi_inst = mw_twi_inst_get(twi_inst_num);
	int twi_idx = mw_twi_idx_get(twi_inst_num);

	/*TODO add logging*/

	nrfx_twi_config_t nrfx_twi = NRFX_TWI_DEFAULT_CONFIG;

	nrfx_twi.frequency = mw_twi_config->frequency;
	nrfx_twi.hold_bus_uninit = mw_twi_config->hold_bus_uninit;
	nrfx_twi.interrupt_priority = mw_twi_config->interrupt_priority;
	nrfx_twi.scl = mw_twi_config->scl;
	nrfx_twi.sda = mw_twi_config->sda;

	//set the callback for the slave
	mw_twi_callback[twi_idx][slave_addr] = mw_twi_handler;

	//only call nrfx_spi_init() on first initialization
	if ( twi_init[twi_idx] == false )
	{
		nrfx_twi_evt_handler_t nrfx_handler = mw_nrfx_twi_event_handler_get(twi_inst_num);
		APP_ERROR_CHECK(nrfx_twi_init(twi_inst, &nrfx_twi, nrfx_handler, p_context));

		twi_init[twi_idx] = true;
	}
}




void mw_twi_uninit(int twi_inst_num)
{

    const nrfx_twi_t *twi_inst = mw_twi_inst_get(twi_inst_num);
    int twi_idx = mw_twi_idx_get(twi_inst_num);

    nrfx_twi_uninit(twi_inst);

    twi_init[twi_idx] = false;

}


void mw_twi_rx(int twi_inst_num,
               uint8_t            address,
               uint8_t *          p_data,
               size_t             length)
{

	const nrfx_twi_t *twi_inst = mw_twi_inst_get(twi_inst_num);
	int twi_idx = mw_twi_idx_get(twi_inst_num);

	//wait for bus to free up
	do
	{
		__WFE();
	}
	while ( nrfx_twi_is_busy(twi_inst) );

	xfer_done[twi_idx] = false;

	APP_ERROR_CHECK(nrfx_twi_rx(twi_inst, address, p_data, length));

	//TODO... decide if it's appropriate to block here
	do
	{
		__WFE();
	}
	while ( xfer_done[twi_idx] == false );

}



void mw_twi_tx(int twi_inst_num,
               uint8_t            address,
               uint8_t const *    p_data,
               size_t             length,
               bool               no_stop)
{
    const nrfx_twi_t *twi_inst = mw_twi_inst_get(twi_inst_num);
    //int twi_idx = mw_twi_idx_get(twi_inst_num);

    //wait for bus to free up
    do
    {
        __WFE();
    }while(nrfx_twi_is_busy(twi_inst));

    //Don't block for TX
    APP_ERROR_CHECK(nrfx_twi_tx(twi_inst,
                    address,
                    p_data,
                    length,
                    no_stop));

}


#endif




