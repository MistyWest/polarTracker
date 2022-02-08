/*
 * mw_nrfx_spim.h
 *
 *  SPIM wrapper for Nordic SDK
 *
 *  Created on: January 2, 2019
 *      Author: Sean Edmond
 */

#ifndef MW_SPI_H_
#define MW_SPI_H_


#include "nrfx_spi.h"

#define SPI_ERROR    99


typedef struct
{
    uint8_t * rx_buffer;
    size_t    rx_length;
} mw_spi_evt_t;

typedef void (* mw_spi_evt_handler_t) ( mw_spi_evt_t const * spi_event );


#if NRFX_CHECK(NRFX_SPI_ENABLED)

static nrfx_spi_t const spi_insts[NRFX_SPI_ENABLED_COUNT] = {
    #if NRFX_CHECK(NRFX_SPI0_ENABLED)
        #define SPI_INST0 0
        NRFX_SPI_INSTANCE(SPI_INST0),
    #endif
    #if NRFX_CHECK(NRFX_SPI1_ENABLED)
        #define SPI_INST1 1
        NRFX_SPI_INSTANCE(SPI_INST1),
    #endif
    #if NRFX_CHECK(NRFX_SPI2_ENABLED)
        #define SPI_INST2 2
        NRFX_SPI_INSTANCE(SPI_INST2),
    #endif
};

#endif


/**
 * @brief SPIM master driver instance configuration structure.
 */
typedef struct
{
    int     spi_instance; ///< SPI Peripheral Instance
    uint8_t sck_pin;      ///< SCK pin number.
    uint8_t mosi_pin;     ///< MOSI pin number (optional).
                          /**< Set to @ref NRFX_SPIM_PIN_NOT_USED
                           *   if this signal is not needed. */
    uint8_t miso_pin;     ///< MISO pin number (optional).
                          /**< Set to @ref NRFX_SPIM_PIN_NOT_USED
                           *   if this signal is not needed. */
    uint8_t ss_pin;       ///< Slave Select pin number (optional).
                          /**< Set to @ref NRFX_SPIM_PIN_NOT_USED
                           *   if this signal is not needed. */
    uint8_t irq_priority; ///< Interrupt priority.
    uint8_t orc;          ///< Over-run character.
                          /**< This character is used when all bytes from the TX buffer are sent,
                               but the transfer continues due to RX. */
    nrf_spi_frequency_t frequency; ///< SPI frequency.
    nrf_spi_mode_t      mode;      ///< SPI mode.
    nrf_spi_bit_order_t bit_order; ///< SPI bit order.
    mw_spi_evt_handler_t handler;  ///< SPI callback handler.
} mw_spi_config_t;




const nrfx_spi_t* mw_spi_inst_get( int instance );


/**
 * @brief Function for perfomring a SPI transfer
 *
 * This function configures and enables the specified peripheral.
 *
 * @param[in] spi_inst_num    Interger of the SPI interface
 * @param[in] p_tx_buffer     Pointer to the transmit buffer
 * @param[in] tx_length       Transmit buffer length
 * @param[in] p_rx_buffer     Pointer to the receive buffer
 * @param[in] rx_length       Receive buffer length
 *
 */
void mw_spi_write(int spi_inst_num,
                   uint8_t const * p_tx_buffer, ///< Pointer to TX buffer.
                   size_t          tx_length,   ///< TX buffer length.
                   uint8_t       * p_rx_buffer, ///< Pointer to RX buffer.
                   size_t          rx_length);


/**
 * @brief Function for uninitializing the SPI master driver instance.
 *
 * @param[in] spi_inst_num    Interger of the SPI interface
 */
void mw_spi_uninit(int spi_inst_num);



/**
 * @brief Function for initializing the SPI master driver instance.
 *
 * This function configures and enables the specified peripheral.
 *
 * @param[in] spi_inst_num    Interger of the SPI interface
 * @param[in] mw_spi_config   Pointer to the structure with initial configuration.
 *
 * @param     handler    Event handler provided by the user. If NULL, transfers
 *                       will be performed in blocking mode.
 * @param     p_context  Context passed to event handler.
 *
 */
void mw_spi_init(int spi_inst_num,
                  mw_spi_config_t const * mw_spi_config,
                  mw_spi_evt_handler_t handler,
                  void * p_context);

#endif /* MW_SPI_H_ */
