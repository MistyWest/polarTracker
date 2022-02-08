/*
 * mw_spi_mmaster.c
 *
 *  Created on: Jan 21, 2019
 *      Author: klockwood
 *
 *
 *
 *      SPI Master Module - configured for multi chip and multi configuration usage
 *
 *      A device which wishes to use this module my called the function
 *
 *      spi_device_init_return_t mw_spi_master_device_init( mw_spim_device_config_t spi_device_config )
 *
 *      this returns a Device ID which must be stored in the Device Driver for future use of any SPI
 *      APIs
 *
 *
 *      *****Noted Errata - PAN89********
 *
 *      This anomaly applies to IC Rev. Rev 2, build codes QFAA-E00, CIAA-E00, QFAB-E00.
 *
 *						It was inherited from the previous IC revision Rev 1.
 *						Symptoms
 *						Static current consumption between 400 �A and 450 �A when using SPIM or TWIM in combination with GPIOTE.
 *
 *						Conditions
 *						GPIOTE is configured in event mode
 *						TWIM/SPIM utilizes EasyDMA
 *						Consequences
 *						Current consumption higher than specified.
 *
 *						Workaround
 *						Turn the TWIM/SPIM off and back on after it has been disabled. To do so, write 0 followed by 1 to the POWER register (address 0xFFC) of the TWIM/SPIM that must be disabled:
 *						If TWIM0 or SPIM0 is used:
 *
 *					*(volatile uint32_t *)0x40003FFC = 0;
 *					*(volatile uint32_t *)0x40003FFC;
 *					*(volatile uint32_t *)0x40003FFC = 1;
 *					If TWIM1 or SPIM1 is used:
 *
 *					*(volatile uint32_t *)0x40004FFC = 0;
 *					*(volatile uint32_t *)0x40004FFC;
 *					*(volatile uint32_t *)0x40004FFC = 1;
 *					If SPIM2 is used:
 *
 *	 					*(volatile uint32_t *)0x40023FFC = 0;
 *						*(volatile uint32_t *)0x40023FFC;
 *					*(volatile uint32_t *)0x40023FFC = 1;
 *					Reconfiguration of TWIM/SPIM is required before next usage.
 */

#include "nrf_error.h"
#include "nrfx_spim.h"
#include "boards.h"

#include "mw_spi_master.h"
#include "mw_logging.h"


#define SPI_TRANSFER_TIMEOUT	200

#define SET_SPI_READ_BIT(x)  (x | 0x80)
#define SET_SPI_WRITE_BIT(x) (x & 0x7F)

#if NRFX_SPIM_ENABLED
static uint8_t 						      m_current_spi_device = 0;
static spi_instance_status_t 		m_spi_drivers_initialized[3] = { { false, 0} ,
                                                                 { false, 0} ,
                                                                 { false, 0} };

static volatile bool 			m_spi_transfer_in_progress = false;
#endif

mw_spim_device_config_t	 	m_spi_devices[MAX_SPI_DEVICES];

#if NRFX_SPIM_ENABLED
static uint32_t mw_spi_master_driver_init( uint8_t device_id );
#endif

//static void mw_spi_master_driver_uninit( uint8_t device_id );


// To use, need to have NRFX_SPI_ENABLED in sdk_config

const int USE_SPI_PERIPHERAL(int instance)
{
	int spi_idx = 99;

	switch(instance)
	{
#if NRFX_CHECK(NRFX_SPIM0_ENABLED)
    case 0 :
    	spi_idx = NRFX_SPIM0_INST_IDX;
    	break;
#endif
#if NRFX_CHECK(NRFX_SPIM1_ENABLED)
    case 1 :
    	spi_idx = NRFX_SPIM1_INST_IDX;
    	break;
#endif
#if NRFX_CHECK(NRFX_SPIM2_ENABLED)
    case 2 :
    	spi_idx = NRFX_SPIM2_INST_IDX;
    	break;
#endif
	}

	if(spi_idx == 99)
	{
		MW_LOG_INFO("SPI Master Instance Error: Not properly enabled in sdk_config.h");
		return 99;
  }
 	else
	{
 		return(spi_idx);
	}
}


#if NRFX_CHECK(NRFX_SPIM0_ENABLED)
void nrfx_spim0_event_handler( nrfx_spim_evt_t const * p_event,
                               void *                  p_context )
{
	if(p_event->type == NRFX_SPIM_EVENT_DONE)
	{
    /*NRF_LOG_INFO("Transfer completed on SPI 0.");
    if (p_event->xfer_desc.rx_length != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(p_event->xfer_desc.p_rx_buffer,
                              strlen((const char *)p_event->xfer_desc.p_rx_buffer));
    }*/
    m_spi_transfer_in_progress = false;
	}
	else
	{
		NRF_LOG_INFO("SPI 0 Error");
	}

  //call generic callback function
  //TODO sendcallback to device specific handler

	if( m_spi_devices[m_current_spi_device].handler != NULL )
	{
		m_spi_devices[m_current_spi_device].handler( p_event->xfer_desc.p_rx_buffer, p_event->xfer_desc.rx_length );
	}
}
#endif


#if NRFX_CHECK(NRFX_SPIM1_ENABLED)
void nrfx_spim1_event_handler( nrfx_spim_evt_t const * p_event,
                               void *                  p_context )
{
	if(p_event->type == NRFX_SPIM_EVENT_DONE)
	{
    /*NRF_LOG_INFO("Transfer completed on SPI 1.");
    if (p_event->xfer_desc.rx_length != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(p_event->xfer_desc.p_rx_buffer,
                              strlen((const char *)p_event->xfer_desc.p_rx_buffer));
    }*/
    m_spi_transfer_in_progress = false;
	}
	else
	{
		NRF_LOG_INFO("SPI 1 Error");
	}

  //call generic callback function
  //TODO sendcallback to device specific handler

	if( m_spi_devices[m_current_spi_device].handler != NULL )
	{
		m_spi_devices[m_current_spi_device].handler( p_event->xfer_desc.p_rx_buffer, p_event->xfer_desc.rx_length );
	}
}
#endif


#if NRFX_CHECK(NRFX_SPIM2_ENABLED)
void nrfx_spim2_event_handler( nrfx_spim_evt_t const * p_event,
                               void *                  p_context )
{
	if(p_event->type == NRFX_SPIM_EVENT_DONE)
	{
    /*NRF_LOG_INFO("Transfer completed on SPI 2.");
    if (p_event->xfer_desc.rx_length != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(p_event->xfer_desc.p_rx_buffer,
                              strlen((const char *)p_event->xfer_desc.p_rx_buffer));
    }*/
    m_spi_transfer_in_progress = false;
	}
	else
	{
		NRF_LOG_INFO("SPI Error");
	}

  //call generic callback function
  //TODO sendcallback to device specific handler

	if( m_spi_devices[m_current_spi_device].handler != NULL )
	{
		m_spi_devices[m_current_spi_device].handler( p_event->xfer_desc.p_rx_buffer, p_event->xfer_desc.rx_length );
	}
}
#endif


/*
 * @brief Return the function handler of the passed SPI Instance
 */
#if NRFX_SPIM_ENABLED
nrfx_spim_evt_handler_t mw_nrfx_event_handler_get(int instance)
{
  nrfx_spim_evt_handler_t nrfx_spim_evt_handle = NULL;

  switch(instance)
  {
#if NRFX_CHECK(NRFX_SPIM0_ENABLED)
  case 0 :
  nrfx_spim_evt_handle = nrfx_spim0_event_handler;
  break;
#endif
#if NRFX_CHECK(NRFX_SPIM1_ENABLED)
  case 1 :
  nrfx_spim_evt_handle = nrfx_spim1_event_handler;
  break;
#endif
#if NRFX_CHECK(NRFX_SPIM2_ENABLED)
  case 2 :
  nrfx_spim_evt_handle = nrfx_spim2_event_handler;
  break;
#endif
  }

  if(nrfx_spim_evt_handle == NULL)
  {
    MW_LOG_INFO("SPI Master Instance Error: Not properly enabled in sdk_config.h");
    return NULL;
  }
  else
  {
    return(nrfx_spim_evt_handle);
  }
}
#endif


/*
 * @brief Return the index value of the passed SPI Instance
 */
const int mw_spim_idx_get( int instance )
{
  int spi_idx = SPI_MASTER_ERROR;

  switch ( instance )
  {
#if NRFX_CHECK(NRFX_SPIM0_ENABLED)
  case 0:
  spi_idx = NRFX_SPIM0_INST_IDX;
  break;
#endif
#if NRFX_CHECK(NRFX_SPIM1_ENABLED)
  case 1 :
  spi_idx = NRFX_SPIM1_INST_IDX;
  break;
#endif
#if NRFX_CHECK(NRFX_SPIM2_ENABLED)
  case 2 :
  spi_idx = NRFX_SPIM2_INST_IDX;
  break;
#endif
  }

  if ( spi_idx == SPI_MASTER_ERROR )
  {
    MW_LOG_INFO("SPI Master Instance Error: Not properly enabled in sdk_config.h");
  }
  return (spi_idx);
}


/**
 * @brief Return the Instance value of the passed SPI Instance
 */
const nrfx_spim_t * mw_spim_inst_get( int instance )
{
  const nrfx_spim_t * nrfx_spim = NULL;

#if NRFX_SPIM_ENABLED

  int spim_idx = mw_spim_idx_get(instance);

  if ( spim_idx == SPI_MASTER_ERROR )
  {
    return NULL;
  }

  nrfx_spim = &spim_insts[spim_idx];

  if ( nrfx_spim == NULL )
  {
    return NULL;
  }

#endif

  return (nrfx_spim);
}



/**
 * @brief Function for perfomring a SPI Transfer
 *
 * This function configures and enables the specified peripheral.
 *
 * @param[in] device_id    		Interger of the SPI device id
 * @param[in] p_tx_buffer     Pointer to the transmit buffer
 * @param[in] tx_length       Transmit buffer length
 * @param[in] p_rx_buffer     Pointer to the receive buffer
 * @param[in] rx_length       Receive buffer length
 *
 */
void mw_spi_master_transfer( uint8_t 								device_id,		// Device Id call for control of SPI bus
														 spi_transfer_type_t 		rw,						// Read or Write transfer
														 uint8_t  		* 				p_tx_buffer,  // Pointer to TX buffer.
														 size_t          				tx_length,    // TX buffer length.
														 uint8_t      * 				p_rx_buffer,  // Pointer to RX buffer.
														 size_t          				rx_length )
{
#if NRFX_SPIM_ENABLED

	uint32_t err_code;
	volatile uint16_t 	m_spi_transfer_count = 0;

	if( rw == SPI_READ ) 				p_tx_buffer[0] 	= SET_SPI_READ_BIT( p_tx_buffer[0] );
	else if( rw == SPI_WRITE ) 	p_tx_buffer[0] 	= SET_SPI_WRITE_BIT( p_tx_buffer[0] );
	else if( rw == SPI_NO_RW_BIT )	__asm__("nop");
	else 												MW_LOG_INFO("mw SPI Master: Invalid read/write parameter passsed");

  // Initialize SPI if necessary
	mw_spi_master_driver_init( device_id );

	const nrfx_spim_t * spim_inst = mw_spim_inst_get( m_spi_devices[device_id].spi_instance );

	nrfx_spim_xfer_desc_t xfer_desc;
	xfer_desc.p_tx_buffer = p_tx_buffer;
	xfer_desc.tx_length 	= tx_length;
	xfer_desc.p_rx_buffer = p_rx_buffer;
	xfer_desc.rx_length 	= rx_length;

	m_spi_transfer_in_progress 	= true;
	m_spi_transfer_count				= 0;

	uint16_t size_remaining = tx_length;
	uint16_t transfer_size = tx_length;

	/**********************************************/
	/* Check for Large Transfer Condition */
	bool large_spi_transfer = false;
	if( tx_length > 255 )
	{
	  large_spi_transfer = true;
	  transfer_size = 255;
	  nrf_gpio_pin_clear( m_spi_devices[device_id].ss_pin );
	  nrfx_spim_change_cs(spim_inst, NRFX_SPIM_PIN_NOT_USED);
	}
	/**********************************************/

  while ( size_remaining > 0 )
  {
    /**********************************************/
    if ( large_spi_transfer )
    {
      xfer_desc.tx_length = transfer_size;
    }
    /**********************************************/

    err_code = nrfx_spim_xfer(spim_inst, &xfer_desc, 0);

    if ( err_code != NRF_SUCCESS )
    {
      APP_ERROR_CHECK(err_code);
    }

    // Wait for SPI Transfer to Complete
    if ( m_spi_devices[device_id].handler == NULL )
    {
      while ( (m_spi_transfer_in_progress) && (m_spi_transfer_count < SPI_TRANSFER_TIMEOUT) )
      {
        __WFE();
        m_spi_transfer_count++;
      }
    }

    if ( m_spi_transfer_count >= SPI_TRANSFER_TIMEOUT )
    {
      MW_LOG_INFO("SPI Master Timeout: Couldn't communicate with device");
    }

    /**********************************************/
    /* Get next transfer chunk */
    size_remaining -= transfer_size;
    if ( large_spi_transfer )
    {
      transfer_size = 255;
      if ( size_remaining <= 255 )
      {
        transfer_size = size_remaining;
      }
    }
    /**********************************************/
  } //end while()

	/* Return control to CS pin to SPI Module*/
	if ( large_spi_transfer )
	{
	  nrf_gpio_pin_clear( m_spi_devices[device_id].ss_pin );
	  nrfx_spim_change_cs(spim_inst, m_spi_devices[device_id].ss_pin);
	}

#endif
}


/**
 * @brief Function for uninstalling a SPI instance
 */
void mw_spi_master_driver_uninit(	uint8_t device_id )
{
#if NRFX_SPIM_ENABLED
	mw_spim_device_config_t spi_device_config = m_spi_devices[device_id];
	const nrfx_spim_t * spim_inst = mw_spim_inst_get(spi_device_config.spi_instance);
  nrfx_spim_uninit(spim_inst);

  m_spi_drivers_initialized[spi_device_config.spi_instance].initialized = false;
#endif
}


/**
 * @brief Function for initializing the SPI master driver instance.
 *
 * This function configures and enables the specified peripheral.
 *
 * @param[in] spi_device_config    SPI Device configuration
 *
 */
#if NRFX_SPIM_ENABLED
static uint32_t mw_spi_master_driver_init( uint8_t device_id )
{
	mw_spim_device_config_t spi_device_config = m_spi_devices[device_id];

  const nrfx_spim_t * spim_inst = mw_spim_inst_get(spi_device_config.spi_instance);
  int spim_idx = mw_spim_idx_get(spi_device_config.spi_instance);

  // SPI Module not set in sdk_config.h
  if(spim_idx == SPI_MASTER_ERROR) return NRF_ERROR_RESOURCES;

	// If this device is already initialized
	if ( m_spi_drivers_initialized[spi_device_config.spi_instance].initialized && (m_spi_drivers_initialized[spi_device_config.spi_instance].device_id == device_id) ) return 0;

	// If another device is already initialized, uninit
	if ( m_spi_drivers_initialized[spi_device_config.spi_instance].initialized && (m_spi_drivers_initialized[spi_device_config.spi_instance].device_id != device_id) )
	  mw_spi_master_driver_uninit(m_spi_drivers_initialized[spi_device_config.spi_instance].device_id);

	nrfx_spim_config_t nrfx_spi = NRFX_SPIM_DEFAULT_CONFIG;
	nrfx_spi.sck_pin 						= spi_device_config.sck_pin;
	nrfx_spi.mosi_pin 					= spi_device_config.mosi_pin;
	nrfx_spi.miso_pin 					= spi_device_config.miso_pin;
	nrfx_spi.ss_pin 						= spi_device_config.ss_pin;
	nrfx_spi.frequency 					= spi_device_config.frequency;
	nrfx_spi.mode 							= spi_device_config.mode;
	nrfx_spi.bit_order 					= spi_device_config.bit_order;

	//defaults
	nrfx_spi.orc 								= 0xFF;
	nrfx_spi.irq_priority				= spi_device_config.irq_priority;


	nrfx_spim_evt_handler_t nrfx_handler = mw_nrfx_event_handler_get(spi_device_config.spi_instance);

	m_spi_drivers_initialized[spi_device_config.spi_instance].device_id     = device_id;
	m_spi_drivers_initialized[spi_device_config.spi_instance].initialized 	= true;

	return nrfx_spim_init(	spim_inst,
													&nrfx_spi,
													nrfx_handler,
													spi_device_config.context);

	m_current_spi_device = m_spi_drivers_initialized[spi_device_config.spi_instance].device_id;
}
#endif


/**
 * @brief Function to registe and initialze a SPI Device driver with the SPI Master Module
 *        The Device must store the return_value.device_id for use with other API calls
 *
 *        Initialization automatically assigns SPI instances.  Assure they are enabled in sdK_config.h
 *
 * @param[in] spi_device_config    SPI Device configuration
 */
spi_device_init_return_t mw_spi_master_device_init( mw_spim_device_config_t spi_device_config )
{
  spi_device_init_return_t return_value;
  return_value.err_code = NRF_SUCCESS;
  return_value.device_id = INVALID_DEVICE_ID;

#if NRFX_SPIM_ENABLED
	static uint8_t spi_device_count = 0;


	return_value.device_id = 0;
	return_value.err_code = NRF_SUCCESS;

	// If we've reached the maximum number of SPI devices
	if( spi_device_count >= MAX_SPI_DEVICES ) return_value.err_code = NRF_ERROR_NO_MEM;


	memcpy( &m_spi_devices[spi_device_count], &spi_device_config, sizeof(spi_device_config) );


	// First device initialized.
	if( spi_device_count == 0 )
	{
		return_value.err_code = mw_spi_master_driver_init( spi_device_count );
	}


	// Set device id
	return_value.device_id = spi_device_count;

	spi_device_count++;

#endif

	return return_value;
}





