/*
 * mw_pwm_peripheral.h
 *
 *  Created on: Aug 11, 2017
 *      Author: klockwood
 */

#ifndef MW_PWM_PERIPHERAL_H_
#define MW_PWM_PERIPHERAL_H_

#include "nrf_pwm.h"

#define MW_PWM_0
#define MW_PWM_1
#define MW_PWM_2


//logic inverted for PCA10010
#ifdef BOARD_PCA10040s
#define SOFT_OFF_TRANSITION				1
#define SOFT_ON_TRANSITION				0
#else
#define SOFT_OFF_TRANSITION				0
#define SOFT_ON_TRANSITION				1
#endif

typedef struct
{
	uint16_t	top_limit;
	uint16_t	mid_limit;
	uint16_t	hi_step;
	uint16_t	lo_step;
}pwm_params_t;

typedef enum
{
	PWM_SOFT_UP_FINISHED,
	PWM_SOFT_DOWN_FINISHED
}mw_pwm_event_t;

/**@brief PWM callback type. */
typedef void (* mw_pwm_callback_t) ( mw_pwm_event_t event );


typedef struct
{
	uint32_t pin;
	mw_pwm_callback_t callback;
}pwm_module_params_t;


void mw_pwm_off();
void mw_pwm_stop();
void mw_pwm_start( uint32_t pin, bool direction );
void mw_pwm_peripheral_init( pwm_module_params_t params );

#endif /* MW_PWM_PERIPHERAL_H_ */
