/*
 * mw_adc_thread.h
 *
 *  Created on: Jan 22, 2019
 *      Author: klockwood
 */

#ifndef MW_SENSOR_THREAD_H_
#define MW_SENSOR_THREAD_H_

typedef enum
{
	ADC_IDLE = 0,
	ADC_START,
	ADC_RUNNING,
	ADC_STOP
}mw_adc_states_t;


/**
 * @brief - Get Battery Level
 */
uint8_t adc_get_battery_level( float current_temperature );


/**
 * @brief Thread is ready
*/
bool mw_adc_thread_ready(void);

void mw_adc_thread_init(void);

#endif /* MW_ADC_THREAD_H_ */
