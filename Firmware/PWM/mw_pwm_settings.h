/*
 * mw_pwm_settings.h
 *
 *  Created on: Feb 12, 2019
 *      Author: klockwood
 */

#ifndef PWM_MW_PWM_SETTINGS_H_
#define PWM_MW_PWM_SETTINGS_H_

#include <nrf_pwm.h>
#include <app_util_platform.h>

#define MW_PWM_INSTANCE											0					// Must match instance enabled in sdk_config.h file

#define MW_PWM_IRQ_PRIORITY									APP_IRQ_PRIORITY_LOWEST

#define MW_PWM_FREQUENCY										NRF_PWM_CLK_4MHz
#define MW_PWM_COUNT_MODE										NRF_PWM_MODE_UP
#define MW_PWM_LOAD_MODE										NRF_PWM_LOAD_INDIVIDUAL
#define MW_PWM_STEP_MODE										NRF_PWM_STEP_TRIGGERED

#define MW_PWM_COUNT_DOWN_TOP_LIMIT_VALUE		32000
#define MW_PWM_COUNT_DOWN_MID_LIMIT_VALUE		18000
#define MW_PWM_COUNT_DOWN_HI_STEP_VALUE			180
#define MW_PWM_COUNT_DOWN_LO_STEP_VALUE			80

#define MW_PWM_COUNT_UP_TOP_LIMIT_VALUE			32000
#define MW_PWM_COUNT_UP_MID_LIMIT_VALUE			18000
#define MW_PWM_COUNT_UP_HI_STEP_VALUE				180
#define MW_PWM_COUNT_UP_LO_STEP_VALUE				80


#endif /* PWM_MW_PWM_SETTINGS_H_ */
