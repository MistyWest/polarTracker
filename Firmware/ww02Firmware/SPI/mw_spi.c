/*
 * mw_spi.c
 *
 *  SPI wrapper for Nordic SDK.  Note, This is a wrapper for the none
 *  DMA version version of SPI.  Note, nfrx_spim implements the DMA
 *  version .
 *
 *  Created on: January 2, 2019
 *      Author: Sean Edmond
 */

// To use, need to have NRFX_SPI_ENABLED in sdk_config
#include "nrf_error.h"
#include "app_error.h"
#include "mw_logging.h"
#include "mw_spi.h"

mw_spi_evt_handler_t mw_spi_callback[NRFX_SPI_ENABLED_COUNT];
bool spi_busy[NRFX_SPI_ENABLED_COUNT];

#if NRFX_CHECK(NRFX_SPI_ENABLED)

#if NRFX_CHECK(NRFX_SPI0_ENABLED)
void nrfx_spi0_event_handler( nrfx_spi_evt_t const * p_event, void * p_context )
{
  NRF_LOG_INFO("Transfer completed on SPI0.");
  if (p_event->xfer_desc.rx_length != 0)
  {
    NRF_LOG_INFO(" Received:");
    NRF_LOG_HEXDUMP_INFO(p_event->xfer_desc.p_rx_buffer,
        strlen((const char *)p_event->xfer_desc.p_rx_buffer));
  }

  mw_spi_evt_t mw_event;
  mw_event.rx_buffer = p_event->xfer_desc.p_rx_buffer;
  mw_event.rx_length = p_event->xfer_desc.rx_length;
  //call generic callback function
  if(mw_spi_callback[NRFX_SPI0_INST_IDX] != NULL)
  {
    mw_spi_callback[NRFX_SPI0_INST_IDX](&mw_event);
  }

  spi_busy[NRFX_SPI0_INST_IDX] = false;
}

#endif

#if NRFX_CHECK(NRFX_SPI1_ENABLED)
void nrfx_spi1_event_handler( nrfx_spi_evt_t const * p_event, void * p_context )
{
  NRF_LOG_INFO("Transfer completed on SPI1.");
  if (p_event->xfer_desc.rx_length != 0)
  {
    NRF_LOG_INFO(" Received:");
    NRF_LOG_HEXDUMP_INFO(p_event->xfer_desc.p_rx_buffer,
        strlen((const char *)p_event->xfer_desc.p_rx_buffer));
  }

  mw_spi_evt_t mw_event;
  mw_event.rx_buffer = p_event->xfer_desc.p_rx_buffer;
  mw_event.rx_length = p_event->xfer_desc.rx_length;
  //call generic callback function
  if(mw_spi_callback[NRFX_SPI1_INST_IDX] != NULL)
  {
    mw_spi_callback[NRFX_SPI1_INST_IDX](&mw_event);
  }

  spi_busy[NRFX_SPI1_INST_IDX] = false;
}

#endif

#if NRFX_CHECK(NRFX_SPI2_ENABLED)
void nrfx_spi2_event_handler( nrfx_spi_evt_t const * p_event, void * p_context )
{
  NRF_LOG_INFO("Transfer completed on SPI2.");
  if (p_event->xfer_desc.rx_length != 0)
  {
    NRF_LOG_INFO(" Received:");
    NRF_LOG_HEXDUMP_INFO(p_event->xfer_desc.p_rx_buffer,
        strlen((const char *)p_event->xfer_desc.p_rx_buffer));
  }

  mw_spi_evt_t mw_event;
  mw_event.rx_buffer = p_event->xfer_desc.p_rx_buffer;
  mw_event.rx_length = p_event->xfer_desc.rx_length;
  //call generic callback function
  if(mw_spi_callback[NRFX_SPI2_INST_IDX] != NULL)
  {
    mw_spi_callback[NRFX_SPI2_INST_IDX](&mw_event);
  }

  spi_busy[NRFX_SPI2_INST_IDX] = false;
}
#endif


nrfx_spi_evt_handler_t mw_nrfx_event_handler_get(int instance)
{
  nrfx_spi_evt_handler_t nrfx_spi_evt_handle = NULL;

  switch(instance)
  {
#if NRFX_CHECK(NRFX_SPI0_ENABLED)
    case 0 :
    nrfx_spi_evt_handle = nrfx_spi0_event_handler;
    break;
#endif
#if NRFX_CHECK(NRFX_SPI1_ENABLED)
    case 1 :
    nrfx_spi_evt_handle = nrfx_spi1_event_handler;
    break;
#endif
#if NRFX_CHECK(NRFX_SPI2_ENABLED)
    case 2 :
    nrfx_spi_evt_handle = nrfx_spi2_event_handler;
    break;
#endif
  }

  if(nrfx_spi_evt_handle == NULL)
  {
    MW_LOG_INFO("SPI Instance Error: Not properly enabled in sdk_config.h");
    return NULL;
  }
  else
  {
    return(nrfx_spi_evt_handle);
  }
}


const int mw_spi_idx_get(int instance)
{
  int spi_idx = SPI_ERROR;

  switch(instance)
  {
#if NRFX_CHECK(NRFX_SPI0_ENABLED)
    case 0 :
    spi_idx = NRFX_SPI0_INST_IDX;
    break;
#endif
#if NRFX_CHECK(NRFX_SPI1_ENABLED)
    case 1 :
    spi_idx = NRFX_SPI1_INST_IDX;
    break;
#endif
#if NRFX_CHECK(NRFX_SPI2_ENABLED)
    case 2 :
    spi_idx = NRFX_SPI2_INST_IDX;
    break;
#endif
  }

  if(spi_idx == SPI_ERROR)
  {
    MW_LOG_INFO("SPI Instance Error: Not properly enabled in sdk_config.h");
    return SPI_ERROR;
  }
  else
  {
    return(spi_idx);
  }
}

const nrfx_spi_t* mw_spi_inst_get(int instance)
{
  const nrfx_spi_t * nrfx_spi = NULL;
  int spi_idx = mw_spi_idx_get(instance);

  if( spi_idx == SPI_ERROR )
  {
    return NULL;
  }

  nrfx_spi = &spi_insts[spi_idx];

  if(nrfx_spi == NULL)
  {
    return NULL;
  }

  return(nrfx_spi);
}


/**
 * @brief - SPI Write
 */
void mw_spi_write( int spi_inst_num, uint8_t const * p_tx_buffer, ///< Pointer to TX buffer.
    size_t tx_length,   ///< TX buffer length.
    uint8_t * p_rx_buffer, ///< Pointer to RX buffer.
    size_t rx_length )
{

  const nrfx_spi_t *spi_inst = mw_spi_inst_get(spi_inst_num);
  int spi_idx = mw_spi_idx_get(spi_inst_num);

  nrfx_spi_xfer_desc_t xfer_desc;
  xfer_desc.p_tx_buffer = p_tx_buffer;
  xfer_desc.tx_length = tx_length;
  xfer_desc.p_rx_buffer = p_rx_buffer;
  xfer_desc.rx_length = rx_length;

  while ( spi_busy[spi_idx] )
  {
    __WFE();
  }

  spi_busy[spi_idx] = true;

  APP_ERROR_CHECK(nrfx_spi_xfer(spi_inst, &xfer_desc, 0));

}


void mw_spi_uninit( int spi_inst_num )
{
  const nrfx_spi_t *spi_inst = mw_spi_inst_get(spi_inst_num);

  nrfx_spi_uninit(spi_inst);
}



void mw_spi_init( int spi_inst_num,
                  mw_spi_config_t const * mw_spi_config,
                  mw_spi_evt_handler_t mw_handler,
                  void * p_context)
{

  const nrfx_spi_t *spi_inst = mw_spi_inst_get(spi_inst_num);
  int spi_idx = mw_spi_idx_get(spi_inst_num);

  /*TODO add logging*/

  nrfx_spi_config_t nrfx_spi = NRFX_SPI_DEFAULT_CONFIG;
  nrfx_spi.sck_pin = mw_spi_config->sck_pin;
  nrfx_spi.mosi_pin = mw_spi_config->mosi_pin;
  nrfx_spi.miso_pin = mw_spi_config->miso_pin;
  nrfx_spi.ss_pin = mw_spi_config->ss_pin;
  nrfx_spi.irq_priority = mw_spi_config->irq_priority;
  nrfx_spi.orc = mw_spi_config->orc;
  nrfx_spi.frequency = mw_spi_config->frequency;
  nrfx_spi.mode = mw_spi_config->mode;
  nrfx_spi.bit_order = mw_spi_config->bit_order;

  if( mw_handler != NULL )  mw_spi_callback[spi_idx] = mw_handler;

  nrfx_spi_evt_handler_t nrfx_handler = mw_nrfx_event_handler_get(spi_inst_num);

  APP_ERROR_CHECK(nrfx_spi_init(spi_inst, &nrfx_spi, nrfx_handler, p_context));

  spi_busy[spi_idx] = false;
}


#endif //***end of #if NRFX_CHECK(NRFX_SPI_ENABLED) ***


