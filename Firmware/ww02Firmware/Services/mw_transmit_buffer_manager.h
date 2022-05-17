/*
 * tx_buffer_manager.h
 *
 *  Created on: Sep 19, 2017
 *      Author: klockwood
 */

/**@file
 *
 * @defgroup mw_transmit_buffer_manager Transmit Buffer Manager
 * @{
 * @ingroup  mw_services
 * @brief    Transit buffer manager
 */

#ifndef SERVICES_MW_TRANSMIT_BUFFER_MANAGER_H_
#define SERVICES_MW_TRANSMIT_BUFFER_MANAGER_H_

// Include service which requires this extension
#include "proskida_service.h"



//*************************************
// Create Data Buffer Size/Type Based on
// the specific characteristic you wish
// to apply the buffer manager to

#define BUFFER_LENGTH				300
#define BUFFER_HALF_LENGTH			BUFFER_LENGTH / 2
#define BUFFER_QUARTER_LENGTH		BUFFER_HALF_LENGTH / 2
#define BUFFER_DATA_SIZE			PROSKIDA_SENSOR_CHAR_SIZE


typedef struct
{
	uint8_t 		data[ BUFFER_DATA_SIZE ];
}data_record_t;

typedef struct
{
	data_record_t	buffer[ BUFFER_LENGTH ];
	uint16_t		length;
}data_buffer_t;

void on_tx_complete_evt( proskida_service_t * p_myService, ble_evt_t * p_ble_evt );

void buffer_tx_data( uint16_t char_handle, uint8_t * data, uint16_t length );

void mw_transmit_buffer_clean();

#endif /* SERVICES_MW_TRANSMIT_BUFFER_MANAGER_H_ */
