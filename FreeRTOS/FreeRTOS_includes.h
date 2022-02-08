/*
 * FreeRTOS_includes.h
 *
 *  Created on: Nov 28, 2018
 *      Author: KLockwood
 */

#ifndef FREERTOS_INCLUDES_H_
#define FREERTOS_INCLUDES_H_

#include <nrf_error.h>

//FreeRTOS Includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

#define OSTIMER_WAIT_FOR_QUEUE_LONG         20                                      /**< Number of ticks to wait for the timer queue to be ready */

#endif /* FREERTOS_INCLUDES_H_ */
