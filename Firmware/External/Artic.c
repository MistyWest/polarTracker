/*
 * artic.c
 *
 *  Created on: Oct 10, 2019
 *      Author: klockwood
 */

#include "nrf_delay.h"
#include "mw_logging.h"
#include "mw_spi_master.h"
#include "mw_spi.h"
#include "_mw_external_device.h"

#include "Artic.h"

#define USE_MW_SPIM                       0    /**<Use SPI Master, if not use SPI (non-DMA Module)*/

#define ARTIC_LOG_TAG                     "ARTIC Driver:"

#if !USE_MW_SPIM
#define INST_SPI_ARTIC                    2
static volatile bool read_in_progress = false;
static volatile bool write_in_progress = false;
#endif

static bool m_spi_initialized = false;

static external_driver_spi_config_t       m_ARTIC_SPI;

/**
 * @brief - SPI Module Callback
 */
#if USE_MW_SPIM
void artic_driver_spi_callback( uint8_t * rx_data, size_t rx_length  )
{

}
#else
void artic_driver_spi_callback( mw_spi_evt_t const * spi_event )
{
  if(spi_event->rx_length > 0)
  {
      read_in_progress = false;
  }
  else
  {
      write_in_progress = false;
  }
}
#endif



/**
 * @brief - Artic SPI Write operation
 */
void artic_spi_write( uint8_t * data,
                      int length)
{
#if USE_MW_SPIM
  mw_spi_master_transfer(m_ARTIC_SPI, SPI_WRITE, data, length, NULL, 0);
#else
  write_in_progress = true;
  mw_spi_write(m_ARTIC_SPI, data, length, NULL, 0);
#endif
}




/**
 * @brief - Artic Burst Write operation
 */
void artic_burst_spi_write( short int address,
                            artic_mem_t mem,
                            uint8_t * data,
                            int length)
{
  uint8_t burst_mem_sel = 0;
  uint8_t burst_data[4];

  switch(mem)
  {
    case ARTIC_P_MEM:
      //full_addr = address | ARTIC_P_ADDR;
      burst_mem_sel = 0;
      break;
    case ARTIC_X_MEM:
      //full_addr address | ARTIC_X_ADDR;
      burst_mem_sel = 1;
      break;
    case ARTIC_Y_MEM:
      //full_addr = address | ARTIC_Y_ADDR;
       burst_mem_sel = 2;
      break;
    case ARTIC_IO_MEM:
      //full_addr = address | ARTIC_IO_ADDR;
       burst_mem_sel = 3;
      break;
  }

#if !USE_MW_SPIM
  while(read_in_progress || write_in_progress)
  {
      __WFE();
  }
#endif

  /*write to burst register in preparation */
  burst_data[0] = 0; //every burst write starts with 0 (BURSTMODE_REG)
  burst_data[1] = (1 << 3) | (burst_mem_sel << 1);
  burst_data[2] = address >> 8;
  burst_data[3] = (address & 0xFF);

#if USE_MW_SPIM
  mw_spi_master_transfer(m_ARTIC_SPI, SPI_WRITE, burst_data, 4, NULL, 0);
#else
  write_in_progress = true;
  mw_spi_write(m_ARTIC_SPI, burst_data, 4, NULL, 0);
#endif

  nrf_delay_us(30);

  /*now that burst register is set-up, burst the data */
#if USE_MW_SPIM
  mw_spi_master_transfer(m_ARTIC_SPI, SPI_WRITE, data, length, NULL, 0);
#else
  mw_spi_write(m_ARTIC_SPI, data, length, NULL, 0);
  while(write_in_progress)
  {
      __WFE();
  }
#endif
}


/**
 * @brief - Artic Burst Read operation
 */
void artic_burst_spi_read( short int address,
                           artic_mem_t mem,
                           uint8_t * data,
                           int length )
{
  uint8_t burst_mem_sel = 0;
  uint8_t burst_data[4];

  switch(mem)
  {
    case ARTIC_P_MEM:
      //full_addr = address | ARTIC_P_ADDR;
      burst_mem_sel = 0;
      break;
    case ARTIC_X_MEM:
      //full_addr address | ARTIC_X_ADDR;
      burst_mem_sel = 1;
      break;
    case ARTIC_Y_MEM:
      //full_addr = address | ARTIC_Y_ADDR;
       burst_mem_sel = 2;
      break;
    case ARTIC_IO_MEM:
      //full_addr = address | ARTIC_IO_ADDR;
       burst_mem_sel = 3;
      break;
  }

  MW_LOG_INFO(ARTIC_LOG_TAG "waiting for any current progress to complete");

#if !USE_MW_SPIM
  while(read_in_progress || write_in_progress)
  {
      __WFE();
  }
#endif

  MW_LOG_INFO(ARTIC_LOG_TAG "set Burst Mode");

  /*write to burst register in preparation */
  burst_data[0] = 0; //every burst write starts with 0 (BURSTMODE_REG)
  burst_data[1] = (1 << 3) | (burst_mem_sel << 1) | 1; //1 for read
  burst_data[2] = address >> 8;
  burst_data[3] = (address & 0xFF);

#if USE_MW_SPIM
  mw_spi_master_transfer(m_ARTIC_SPI, SPI_WRITE, burst_data, 4, NULL, 0);
#else
  read_in_progress = true;
  mw_spi_write(m_ARTIC_SPI, burst_data, 4, NULL, 0);
#endif

  nrf_delay_us(100);

  MW_LOG_INFO(ARTIC_LOG_TAG "Reading data..");

  read_in_progress = true;
#if USE_MW_SPIM
  mw_spi_master_transfer(m_ARTIC_SPI, SPI_WRITE, data, length, data, length);
#else
  mw_spi_write(m_ARTIC_SPI, data, length, data, length);

  while(read_in_progress)
  {
      __WFE();
  }
#endif

  MW_LOG_INFO(ARTIC_LOG_TAG "artic_burst_spi_read COMPLETE");
}


/**
 * @brief - Artic Command Write
 */
void artic_command_write(uint8_t command)
{
  /*now that burst register is set-up, burst the data */
#if USE_MW_SPIM
  mw_spi_master_transfer(m_ARTIC_SPI,SPI_WRITE, &command, 1, NULL, 0);
#else

  while(read_in_progress || write_in_progress)
  {
      __WFE();
  }

  /*now that burst register is set-up, burst the data */
  write_in_progress = true;
  mw_spi_write(m_ARTIC_SPI, &command, 1, NULL, 0);

  while(write_in_progress)
  {
      __WFE();
  }
#endif
}

void artic_int1_clear()
{
  MW_LOG_DEBUG(ARTIC_LOG_TAG "artic_int1_clear");

  artic_command_write(ARTIC_CMD_CLR_INT_1);
}


void artic_int2_clear()
{
  MW_LOG_DEBUG(ARTIC_LOG_TAG "artic_int2_clear");

  artic_command_write(ARTIC_CMD_CLR_INT_2);
}



//*********************************************************************************************
//*********************************************************************************************
/**
 * @brief - Driver Interface Initialization
 */
#if USE_MW_SPIM
void artic_init( external_device_config_t device_config )
{
  if ( device_config.communication == SPI_COMMUNICATION )
  {
    spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

    if ( return_value.err_code != NRF_SUCCESS ) APP_ERROR_CHECK(return_value.err_code);

    m_ARTIC_SPI = return_value.device_id;

    MW_LOG_INFO(ARTIC_LOG_TAG "ARTIC SPI Initialized");
  }
}


#else

void artic_init( mw_spi_config_t spi_config )
{
  mw_spi_config_t spi_cfg;

  if(m_spi_initialized) return;

  //Through testing, thiis is the Maximum frequency
  //this doesn't align with the theoretical frequency /* min clock period is 40ns = 25Mbps */
  m_ARTIC_SPI = spi_config.spi_instance;
  spi_cfg.bit_order = spi_config.bit_order;
  spi_cfg.frequency = spi_config.frequency;
  spi_cfg.irq_priority = spi_config.irq_priority;
  spi_cfg.miso_pin = spi_config.miso_pin;
  spi_cfg.mosi_pin = spi_config.mosi_pin;
  spi_cfg.sck_pin = spi_config.sck_pin;
  spi_cfg.ss_pin = spi_config.ss_pin;
  spi_cfg.orc = 0xFF;
  spi_cfg.mode = spi_config.mode;

  mw_spi_init(m_ARTIC_SPI, &spi_cfg, artic_driver_spi_callback, NULL);

  m_spi_initialized = true;

  MW_LOG_INFO(ARTIC_LOG_TAG "ARTIC SPI Initialized");
}
#endif


