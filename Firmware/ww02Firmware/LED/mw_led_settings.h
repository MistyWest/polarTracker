/*
 * mw_led_settings.h
 *
 *  Created on: Mar 28, 2019
 *      Author: klockwood
 */

#ifndef LED_MW_LED_SETTINGS_H_
#define LED_MW_LED_SETTINGS_H_


/**********************************************************
 *
 *        LED xTimer Intervals
 *
 **********************************************************/
#define LED_FLASH_INTERVAL          5000
#define LED_BLINK_INTERVAL_SLOW     1000
#define LED_BLINK_INTERVAL_MED      500
#define LED_BLINK_INTERVAL_FAST     100

#define LED_PWM_INTERVAL_SLOW       15
#define LED_PWM_INTERVAL_MED        15
#define LED_PWM_INTERVAL_FAST       5

#define LED_PWM_DUTY_SLOW           2 //Treated as Percentage
#define LED_PWM_DUTY_MED            40  //Treated as Percentage
#define LED_PWM_DUTY_FAST           2 //Treated as Percentage

#define LED_SOFT_PWM_INTERVAL       18
#define LED_SOFT_STEP_INTERVAL      75 //95
#define LED_SOFT_LAST_PHASE         200
#define LED_SOFT_MINIMUM_VALUE      1

#define LED_SOLID_ON_INTERVAL       5000

/**********************************************************
**********************************************************/
#define START_UP_FAST_TIME          50
#define START_UP_SLOW_TIME          100
#define START_UP_CYCLES             14

#define DFU_SEQUENCE_ON_TIME        800
#define DFU_SEQUENCE_END_TIME       100

#define ADVERTISING_SEQUENCE_ON_TIME  20
#define ADVERTISING_SEQUENCE_OFF_TIME 4000 - ADVERTISING_SEQUENCE_ON_TIME
#define LP_ADVERTISING_SEQUENCE_OFF_TIME 38000 - ADVERTISING_SEQUENCE_ON_TIME

#define CONNECTED_SEQUENCE_ON_TIME    50
#define CONNECTED_SEQUENCE_OFF_TIME   200
#define CONNECTED_SEQUENCE_BLINKS     5

#define CONNECTING_SEQUENCE_TIME      5000

#define ASSIGN_SEQUENCE_ON_TIME       2000
#define ASSIGN_SEQUENCE_OFF_TIME      2
#define ASSIGN_SEQUENCE_BLINKS        1

#define BATTERY_CHARGING_ON_DURATION  100
#define BATTERY_CHARGING_OFF_DURATION 800
#define BATTERY_CHARGING_COMPLETE     3000

#define BATTERY_LOW_SEQUENCE_ON_TIME  100
#define BATTERY_LOW_SEQUENCE_OFF_TIME 200
#define BATTERY_LOW_SEQUENCE_BLINKS   3
#define BATTERY_LOW_SEQUENCE_TIME     5000


#define START_DATA_SEQUENCE_ON_TIME   80
#define START_DATA_SEQUENCE_OFF_TIME  100
#define START_DATA_SEQUENCE_BLINKED   3

#define DATA_SEQUENCE_ON_TIME         100
#define DATA_SEQUENCE_OFF_TIME        2900

#define END_DATA_SEQUENCE_TIME        800

#define LOW_POWER_ON_TIME             50
#define LOW_POWER_OFF_TIME            8950

#define END_DATA_SEQUENCE_ON_TIME_CHARING   20
#define END_DATA_SEQUENCE_OFF_TIME_CHARING  50
#define END_DATA_SEQUENCE_BLINKED_CHARING   9

#define STANDARD_BLINKS_ON_DURATION   100
#define STANDARD_BLINKS_OFF_DURATION  200

#define HEARTBEAT_ON_DURATION         50
#define HEARTBEAT_OFF_DURATION        200

#define HEARTBEAT_2_ON_DURATION       50
#define HEARTBEAT_2_OFF_DURATION      800


#endif /* LED_MW_LED_SETTINGS_H_ */
