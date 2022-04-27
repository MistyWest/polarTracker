/*
 * mw_twi.h
 *
 *
 *
 *  Created on: January 3, 2019
 *      Author: Sean Edmond
 */

#ifndef MW_TWI_H_
#define MW_TWI_H_

#include "nrfx_twi.h"


/*this could be reduced. Could select from a pool of addresses*/
#define MAX_NUM_SLAVES 127


#if NRFX_CHECK(NRFX_TWI_ENABLED)


/*array of callback functions for each slave*/


static nrfx_twi_t const twi_insts[NRFX_TWI_ENABLED_COUNT] = {
    #if NRFX_CHECK(NRFX_TWI0_ENABLED)
        #define TWI_INST0 0
        NRFX_TWI_INSTANCE(TWI_INST0),
    #endif
    #if NRFX_CHECK(NRFX_TWI1_ENABLED)
        #define TWI_INST1 1
        NRFX_TWI_INSTANCE(TWI_INST1),
    #endif
};

const nrfx_twi_t* mw_twi_inst_get(int instance);


#endif /*could extend this to support other TWI flavours */



/*MW layer*/

/**
 * @brief Structure for the TWI master driver instance configuration.
 */
typedef struct
{
    uint32_t            scl;                 ///< SCL pin number.
    uint32_t            sda;                 ///< SDA pin number.
    nrf_twi_frequency_t frequency;           ///< TWI frequency.
    uint8_t             interrupt_priority;  ///< Interrupt priority.
    bool                hold_bus_uninit;     ///< Hold pull up state on gpio pins after uninit.
} mw_twi_config_t;


typedef struct
{
    nrfx_twi_xfer_type_t    type;             ///< Type of transfer.
    uint8_t                 address;          ///< Slave address.
    size_t                  primary_length;   ///< Number of bytes transferred.
    size_t                  secondary_length; ///< Number of bytes transferred.
    uint8_t *               p_primary_buf;    ///< Pointer to transferred data.
    uint8_t *               p_secondary_buf;  ///< Pointer to transferred data.
} mw_twi_evt_t;


typedef void (* mw_twi_evt_handler_t) ( mw_twi_evt_t const * twi_event );



void mw_twi_enable( int instance );

void mw_twi_disable( int instance );


void mw_twi_rx(int twi_inst_num,
               uint8_t            address,
               uint8_t *          p_data,
               size_t             length);



/**
 * @brief Function for sending data to a TWI slave.
 *
 * The transmission will be stopped when an error occurs. If a transfer is ongoing,
 * the function returns the error code @ref NRFX_ERROR_BUSY.
 *
 * @param[in] twi_inst_num Integer for the TWI instance you want to use
 * @param[in] address      Address of a specific slave device (only 7 LSB).
 * @param[in] p_data       Pointer to a transmit buffer.
 * @param[in] length       Number of bytes to send.
 * @param[in] no_stop      If set, the stop condition is not generated on the bus
 *                         after the transfer has completed successfully (allowing
 *                         for a repeated start in the next transfer).
 *
 */
void mw_twi_tx(int twi_inst_num,
               uint8_t            address,
               uint8_t const *    p_data,
               size_t             length,
               bool               no_stop);



/**
 * @brief Function for uninitializing the TWI instance.
 *
 * @param[in] twi_inst_num    Integer for the TWI instance you want to use
 */
void mw_twi_uninit(int twi_inst_num);



/**
 * @brief Function for initializing the TWI driver instance.
 *
 * @param[in] twi_inst_num    Integer for the TWI instance you want to use
 * @param[in] slave_addr      The slave address to associate this callback for
 * @param[in] mw_twi_config   Pointer to the structure with initial configuration.
 * @param[in] mw_twi_handler  Event handler provided by the user. If NULL, blocking mode is enabled.
 * @param[in] p_context       Context passed to event handler.
 *
 */
void mw_twi_init(int                     twi_inst_num,
                 int                     slave_addr,
                 mw_twi_config_t const * mw_twi_config,
                 mw_twi_evt_handler_t    mw_twi_handler,
                 void *                  p_context);





#endif /* MW_TWI_H_ */
