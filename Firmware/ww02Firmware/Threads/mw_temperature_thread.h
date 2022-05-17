/*
 * mw_flash_thread.h
 *
 *  Created on: Mar 8, 2019
 *      Author: sedmond
 */

#ifndef THREADS_MW_TEMPERATURE_THREAD_H_
#define THREADS_MW_TEMPERATURE_THREAD_H_

#include "stdbool.h"

typedef enum
{
  TEMPERATURE_UNKNOWN,
  TEMPERATURE_OK,
  TEMPERATURE_COLD,
}mw_temp_operating_mode_t;



typedef void (* mw_temperature_external_handler_t) ( mw_temp_operating_mode_t const temperature_state, bool isr );


/**
 * @brief Set External Handler for Temperature Events
 */
void mw_temperature_set_external_handler( mw_temperature_external_handler_t external_handler );


/**
 * @brief Returns ture when Thread initialized and running
 */
bool mw_temperature_thread_ready();


/**
 * @brief Function for application main entry.
 */
void mw_temperature_thread_init( void );


#endif /* THREADS_MW_FLASH_THREAD_H_ */
