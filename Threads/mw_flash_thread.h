/*
 * mw_flash_thread.h
 *
 *  Created on: Oct 15, 2019
 *      Author: klockwood
 */

#ifndef THREADS_MW_FLASH_THREAD_H_
#define THREADS_MW_FLASH_THREAD_H_


/**
 * @brief - Save Bear ID
 */
void mw_flash_thread_save_bear_id_and_profile( uint16_t bear_id, bool isr );


/**
 * @brief - Load Bear ID
 */
void mw_flash_thread_load_bear_id_and_profile( uint16_t * buffer );


/**
 * @brief Returns ture when Thread initialized and running
 */
bool  mw_flash_thread_ready( void );


/**
 * @brief Function for application main entry.
 */
void mw_flash_thread_init( void );

#endif /* THREADS_MW_FLASH_THREAD_H_ */
