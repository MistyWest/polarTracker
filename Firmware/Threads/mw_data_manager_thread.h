/*
 * mw_data_manager_thread.h
 *
 *  Created on: Oct 8, 2019
 *      Author: klockwood
 */

#ifndef THREADS_MW_DATA_MANAGER_THREAD_H_
#define THREADS_MW_DATA_MANAGER_THREAD_H_

/**
 * @brief - Set PA gain
 */
void mw_data_manager_update_pa_gain( uint8_t gain );

/**
 * @brief - Set Profile Type
 */
void mw_data_manager_set_profile( uint8_t profile );

/**
 * @brief - Set Bear ID
 */
void mw_data_manager_set_bear_id( uint8_t id );

/**
 * @brief Returns ture when Thread initialized and running
 */
bool  mw_data_manager_thread_ready( void );


/**
 * @brief Function for application main entry.
 */
void mw_data_manager_thread_init( void );


#endif /* THREADS_MW_DATA_MANAGER_THREAD_H_ */
