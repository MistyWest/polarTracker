/*
 * mw_thread.h
 *
 *  Created on: Sep 27, 2017
 *      Author: klockwood
 */

/**@file
 * @defgroup mw_thread Thread
 * @ingroup  mw_thread
 * @brief    Thread
 */

#ifndef THREADS_MW_THREAD_H_
#define THREADS_MW_THREAD_H_

#define ISR_CONTEXT           true
#define NORMAL_CONTEXT        false


#define MW_SUCCESS            0
#define MW_INVALID_PARAM      7

typedef enum
{
	THREAD_NULL,
	THREAD_INITIALIZED,
	THREAD_ACTIVE,
	THREAD_SUSPENDED,
	THREAD_PAUSED,
	THREAD_LOW_POWER_PAUSED
}mw_thread_mode_t;

#endif /* THREADS_MW_THREAD_H_ */
