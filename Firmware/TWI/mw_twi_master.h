/*
 * mw_twi_master.h
 *
 *
 *
 *  Created on: February 13, 2019
 *      Author: Kevin Lockwood
 */

#ifndef MW_TWI_MASTER_H_
#define MW_TWI_MASTER_H_

#include "nrfx_twim.h"

#define TWI_MASTER_ERROR        99
#define MAX_TWI_INSTANCES       2

/*this could be reduced. Could select from a pool of addresses*/
#define MAX_NUM_SLAVES        127

#define NO_STOP               true
#define STOP                  false


#if NRFX_CHECK(NRFX_TWIM_ENABLED)


/*array of callback functions for each slave*/


static nrfx_twim_t const twim_insts[NRFX_TWIM_ENABLED_COUNT] = {
    #if NRFX_CHECK(NRFX_TWIM0_ENABLED)
        #define TWIM_INST0 0
        NRFX_TWIM_INSTANCE(TWIM_INST0),
    #endif
    #if NRFX_CHECK(NRFX_TWIM1_ENABLED)
        #define TWIM_INST1 1
        NRFX_TWIM_INSTANCE(TWIM_INST1),
    #endif
};

const nrfx_twim_t* mw_twim_inst_get(int instance);


#endif /*could extend this to support other TWIM flavours */


typedef struct
{
	nrfx_twim_xfer_type_t type;             ///< Type of transfer.
	uint8_t 							address;          ///< Slave address.
	size_t 								primary_length;   ///< Number of bytes transferred.
	size_t 								secondary_length; ///< Number of bytes transferred.
	uint8_t 						* p_primary_buf;    ///< Pointer to transferred data.
	uint8_t 						* p_secondary_buf;  ///< Pointer to transferred data.
} mw_twim_evt_t;


typedef void (* mw_twim_evt_handler_t) ( mw_twim_evt_t const * twim_event );


/*MW layer*/

/**
 * @brief Structure for the TWIM master driver instance configuration.
 */
typedef struct
{
	int										instance;						// TWI Instance - verify with settings in sdk_config.h
	mw_twim_evt_handler_t mw_twim_handler;		// TWI Handler Function
  void 							*	  context;						// SPI Context
	int 									slave_addr;					// Slave Address
	uint32_t 							scl;                ///< SCL pin number.
	uint32_t 							sda;								///< SDA pin number.
	nrf_twim_frequency_t 	frequency;					///< TWIM frequency.
	uint8_t 							interrupt_priority;	///< Interrupt priority.
	bool 									hold_bus_uninit;		///< Hold pull up state on gpio pins after uninit.
}mw_twim_device_config_t;



void mw_twi_master_change_slave_address( int instance, uint8_t slave_address );

void mw_twi_master_enable( int instance );

void mw_twi_master_disable( int instance );

void mw_twi_master_rx( int 							twim_inst_num,
											 uint8_t            address,
											 uint8_t *          p_data,
											 size_t             length );



/**
 * @brief Function for sending data to a TWIM slave.
 *
 * The transmission will be stopped when an error occurs. If a transfer is ongoing,
 * the function returns the error code @ref NRFX_ERROR_BUSY.
 *
 * @param[in] twim_inst_num Integer for the TWIM instance you want to use
 * @param[in] address      Address of a specific slave device (only 7 LSB).
 * @param[in] p_data       Pointer to a transmit buffer.
 * @param[in] length       Number of bytes to send.
 * @param[in] no_stop      If set, the stop condition is not generated on the bus
 *                         after the transfer has completed successfully (allowing
 *                         for a repeated start in the next transfer).
 *
 */
void mw_twi_master_tx( int twim_inst_num,
											 uint8_t            address,
											 uint8_t const *    p_data,
											 size_t             length,
											 bool               no_stop );


/**
 * @brief Function for uninitializing the TWIM instance.
 *
 * @param[in] twim_inst_num    Integer for the TWIM instance you want to use
 */
void mw_twi_master_uninit(int twim_inst_num);

/**
 * @brief Function for initializing the TWIM driver instance.
 *
 * @param[in] device_config    Full Device TWI Configuration
 *
 */
void mw_twi_master_init( mw_twim_device_config_t device_config );



#endif /* MW_TWI_MASTER_H_ */
