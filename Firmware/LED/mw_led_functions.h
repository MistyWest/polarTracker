/*
 * mw_led_functions.h
 *
 *  Created on: Aug 11, 2017
 *      Author: klockwood
 */

#ifndef MW_LED_FUNCTIONS_H_
#define MW_LED_FUNCTIONS_H_

#define SOFT_ON             1
#define SOFT_OFF            0

#define OFF_AT_THE_END      true
#define KEEP_ON_AT_END      false

#define LED_ON              true
#define LED_OFF             false

typedef enum
{
	LED_SOFT_ON_FINISHED,
	LED_SOFT_OFF_FINISHED,
	LED_BLINK_FINISHED,
	LED_SOLID_ON_FINISHED
}mw_led_event_t;

typedef struct
{
	uint32_t led;
	uint32_t duration_on;
	uint32_t duration_off;
	uint8_t  count;
	bool	 state;
}mw_led_blink_t;

typedef struct
{
	uint32_t led;
	uint32_t duration_on;
	uint32_t duration_off;
	bool	 state;
}mw_led_flash_t;

/**@brief LED callback type. */
typedef void (* mw_led_callback_t) ( mw_led_event_t event );

void mw_led_pwm_off();
void mw_led_soft_transition( uint32_t led, uint8_t direction );
void mw_led_stop_soft_transition();

void mw_led_start_blinks( uint32_t led, uint32_t duration_on, uint32_t duration_off, uint8_t blinks );
void mw_led_stop_blinks();

/**
 * @brief - Start Flash 0 Timer
 */
void mw_led_start_flash_0( uint32_t led, uint32_t duration_on, uint32_t duration_off );

/**
 * @brief - Stop Flash 0 Timer
 */
void mw_led_stop_flash_0();

/**
 * @brief - Start Flash 1 Timer
 */
void mw_led_start_flash_1( uint32_t led, uint32_t duration_on, uint32_t duration_off );

/**
 * @brief - Stop Flash 1 Timer
 */
void mw_led_stop_flash_1();


/**
 * @brief - Start Flash Secondary Timer
 */
void mw_led_start_secondary_flash( uint32_t led, uint32_t duration_on, uint32_t duration_off );

/**
 * @brief - Stop Flash Secondary Timer
 */
void mw_led_stop_secondary_flash();


/**
 * @brief - Start Turn ON Timer
 */
void mw_led_solid_start( uint32_t led, bool start_state, uint32_t duration_msec, bool switch_at_the_end );

/**
 * @brief - Start Turn ON Timer
 */
void mw_led_on_stop();


void mw_turn_on_led( uint32_t led );
void mw_turn_off_led( uint32_t led );


/**
 * @brief - Stops all potential sequences
 */
void mw_led_all_functions_off();


/**
 * @brief - Initialize all sequence functions for module
 */

void mw_led_functions_init( mw_led_callback_t callback );

#endif /* MW_LED_FUNCTIONS_H_ */
