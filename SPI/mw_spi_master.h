/*
 * mw_spi_mmaster.h
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

#ifndef SPI_MW_SPI_MASTER_H_
#define SPI_MW_SPI_MASTER_H_

#include "nrf_spi.h"
#include "nrfx_spim.h"

#define MAX_SPI_DEVICES			8
#define SPI_MASTER_ERROR		99
#define INVALID_DEVICE_ID   -99

static nrfx_spim_t const spim_insts[NRFX_SPIM_ENABLED_COUNT] = {
    #if NRFX_CHECK(NRFX_SPIM0_ENABLED)
        #define SPIM_INST0 0
        NRFX_SPIM_INSTANCE(SPIM_INST0),
    #endif
    #if NRFX_CHECK(NRFX_SPIM1_ENABLED)
        #define SPIM_INST1 1
        NRFX_SPIM_INSTANCE(SPIM_INST1),
    #endif
    #if NRFX_CHECK(NRFX_SPIM2_ENABLED)
        #define SPIM_INST2 2
        NRFX_SPIM_INSTANCE(SPIM_INST2),
    #endif
};


const nrfx_spim_t * mw_spi_m_inst_get(int instance);

typedef struct
{
  uint8_t device_no;
  uint8_t timeout_errors;
}mw_spi_master_error_tracking_t;


//typedef void ( * mw_spim_device_evt_handler_t ) ( uint8_t * rx_data, size_t rx_length  );
typedef void ( * mw_spim_device_evt_handler_t ) ( uint8_t * rx_data, size_t rx_length  );


/**
 * @brief SPMaster Driverce configuration structure.
 */
typedef struct
{
	int														spi_instance;			// SPI Peripheral Instance
	mw_spim_device_evt_handler_t  handler;					// Device specific handler
  uint8_t 											irq_priority;			// SPI IRQ Priority
  void 												* context;					// SPI Context
  uint8_t 						 					sck_pin;      		// SCK pin number.
  uint8_t 											mosi_pin;     		// MOSI pin number (optional).
																									/**< Set to @ref NRFX_SPIM_PIN_NOT_USED
																							 			*   if this signal is not needed. */
  uint8_t 											miso_pin;     		// MISO pin number (optional).
                        													/**< Set to @ref NRFX_SPIM_PIN_NOT_USED
                        							 	 	 	 			*   if this signal is not needed. */
  uint8_t 											ss_pin;       		// Slave Select pin number (optional).
                        													/**< Set to @ref NRFX_SPIM_PIN_NOT_USED
                        							 	 	 	 	 	 	 *   if this signal is not needed. */
  bool 													ss_active_high;  	// Polarity of the Slave Select pin during transmission.
  nrf_spim_frequency_t 					frequency; 				// SPI frequency.
  nrf_spim_mode_t      					mode;      				// SPI mode.
  nrf_spim_bit_order_t 					bit_order; 				// SPI bit order
} mw_spim_device_config_t;


typedef struct
{
  bool    initialized;
  uint8_t device_id;
}spi_instance_status_t;


typedef struct
{
	uint32_t err_code;
	uint8_t  device_id;
}spi_device_init_return_t;

typedef enum
{
    SPI_READ,
    SPI_WRITE,
    SPI_NO_RW_BIT
}spi_transfer_type_t;


const int USE_SPI_PERIPHERAL(int instance);


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
														 uint8_t 			* 				p_tx_buffer,  // Pointer to TX buffer.
														 size_t          				tx_length,    // TX buffer length.
														 uint8_t      * 				p_rx_buffer,  // Pointer to RX buffer.
														 size_t          				rx_length );


/**
 * @brief Function for uninstalling a SPI instance
 */

void mw_spi_master_driver_uninit( uint8_t device_id );


/**
 * @brief Function for initializing/registering devices to the SPI module
 *
 * @param[in] spi_device_config    SPI device configuration
 */
spi_device_init_return_t mw_spi_master_device_init( mw_spim_device_config_t spi_device_config );



#endif /* SPI_MW_SPI_MASTER_H_ */
