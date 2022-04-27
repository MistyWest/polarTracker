/*
 * mw_led_functions.c
 *
 *  Created on: Aug 11, 2017
 *      Author: klockwood
 */

#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"

#include <FreeRTOS_includes.h>

#include "boards.h"
#include "mw_pwm_peripheral.h"
#include <LED/mw_led_settings.h>
#include <LED/mw_led_functions.h>

static TimerHandle_t m_led_flash_interval_timer_0;
static TimerHandle_t m_led_flash_interval_timer_1;
static TimerHandle_t m_led_flash_interval_secondary_timer;

static TimerHandle_t m_led_blink_interval_timer;

static TimerHandle_t m_led_on_timer;

static uint32_t led_pwm;

static mw_led_blink_t m_led_blink;
static mw_led_flash_t m_led_flash_0;
static mw_led_flash_t m_led_flash_1;
static mw_led_flash_t m_led_secondary_flash;
static mw_led_flash_t m_led_on;

static mw_led_callback_t m_led_callback;

static bool  m_switch_after_solid_on = false;
/**
 * @brief - PWM Module callback function
 */
static void led_pwm_handler( mw_pwm_event_t event )
{
	if( event == PWM_SOFT_UP_FINISHED)
	{
		//mw_turn_on_led(led_pwm);
		m_led_callback(LED_SOFT_OFF_FINISHED);
	}
	if( event == PWM_SOFT_DOWN_FINISHED)
	{
		//mw_turn_off_led(led_pwm);
		m_led_callback(LED_SOFT_ON_FINISHED);
	}
}


/**************************************************************************************/
/********************************* LED Flash 0 ****************************************/

/**
 * @brief - Flash 0 Timer Timeout Handler
 */
static void led_flash_0_timeout_handler(TimerHandle_t xTimer)
{
	uint32_t next_phase_duration;

	if(m_led_flash_0.state)
	{
		next_phase_duration = m_led_flash_0.duration_off;
		mw_turn_off_led(m_led_flash_0.led);
	}
	else
	{
		next_phase_duration = m_led_flash_0.duration_on;
		mw_turn_on_led(m_led_flash_0.led);
	}

	m_led_flash_0.state = !m_led_flash_0.state;
	xTimerChangePeriod(m_led_flash_interval_timer_0, next_phase_duration, OSTIMER_WAIT_FOR_QUEUE);
}


/**
 * @brief - Start Flash 0 Timer
 */
void mw_led_start_flash_0( uint32_t led, uint32_t duration_on, uint32_t duration_off )
{
  m_led_flash_0.led           = led;
  m_led_flash_0.duration_on   = duration_on;
  m_led_flash_0.duration_off  = duration_off;
  m_led_flash_0.state         = true;

  xTimerStop(m_led_flash_interval_timer_0, OSTIMER_WAIT_FOR_QUEUE);
  xTimerChangePeriod(m_led_flash_interval_timer_0, duration_on, OSTIMER_WAIT_FOR_QUEUE);

  if (pdPASS != xTimerStart(m_led_flash_interval_timer_0, OSTIMER_WAIT_FOR_QUEUE))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  mw_turn_on_led(led);
}


/**
 * @brief - Stop Flash 0 Timer
 */
void mw_led_stop_flash_0()
{
  mw_turn_off_led(m_led_flash_0.led);
  xTimerStop(m_led_flash_interval_timer_0, OSTIMER_WAIT_FOR_QUEUE);
}


/**************************************************************************************/
/******************************** LED Flash 1 *****************************************/

/**
 * @brief - Flash 1 Timer Timeout Handler
 */
static void led_flash_1_timeout_handler(TimerHandle_t xTimer)
{
  uint32_t next_phase_duration;

  if(m_led_flash_1.state)
  {
    next_phase_duration = m_led_flash_1.duration_off;
    mw_turn_off_led(m_led_flash_1.led);
  }
  else
  {
    next_phase_duration = m_led_flash_1.duration_on;
    mw_turn_on_led(m_led_flash_1.led);
  }

  m_led_flash_1.state = !m_led_flash_1.state;
  xTimerChangePeriod(m_led_flash_interval_timer_1, next_phase_duration, OSTIMER_WAIT_FOR_QUEUE);
}


/**
 * @brief - Start Flash 1 Timer
 */
void mw_led_start_flash_1( uint32_t led, uint32_t duration_on, uint32_t duration_off )
{
  m_led_flash_1.led           = led;
  m_led_flash_1.duration_on   = duration_on;
  m_led_flash_1.duration_off  = duration_off;
  m_led_flash_1.state         = true;

  xTimerStop(m_led_flash_interval_timer_1, OSTIMER_WAIT_FOR_QUEUE);
  xTimerChangePeriod(m_led_flash_interval_timer_1, duration_on, OSTIMER_WAIT_FOR_QUEUE);

  if (pdPASS != xTimerStart(m_led_flash_interval_timer_1, OSTIMER_WAIT_FOR_QUEUE))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  mw_turn_on_led(led);
}


/**
 * @brief - Stop Flash 1 Timer
 */
void mw_led_stop_flash_1()
{
  mw_turn_off_led(m_led_flash_1.led);
  xTimerStop(m_led_flash_interval_timer_1, OSTIMER_WAIT_FOR_QUEUE);
}


/**************************************************************************************/
/****************************** LED Flash Secondary ***********************************/


static void led_flash_secondary_timeout_handler(TimerHandle_t xTimer)
{
	uint32_t next_phase_duration;

	if(m_led_secondary_flash.state)
	{
		next_phase_duration = m_led_secondary_flash.duration_off;
		mw_turn_off_led(m_led_secondary_flash.led);
	}
	else
	{
		next_phase_duration = m_led_secondary_flash.duration_on;
		mw_turn_on_led(m_led_secondary_flash.led);
	}

	m_led_secondary_flash.state = !m_led_secondary_flash.state;
	xTimerChangePeriod(m_led_flash_interval_secondary_timer, next_phase_duration, OSTIMER_WAIT_FOR_QUEUE);
}



void mw_led_start_secondary_flash( uint32_t led, uint32_t duration_on, uint32_t duration_off )
{
	m_led_secondary_flash.led 			= led;
	m_led_secondary_flash.duration_on 	= duration_on;
	m_led_secondary_flash.duration_off 	= duration_off;
	m_led_secondary_flash.state			= true;

	xTimerStop(m_led_flash_interval_secondary_timer, OSTIMER_WAIT_FOR_QUEUE);
	xTimerChangePeriod(m_led_flash_interval_secondary_timer, duration_on, OSTIMER_WAIT_FOR_QUEUE);

	if (pdPASS != xTimerStart(m_led_flash_interval_secondary_timer, OSTIMER_WAIT_FOR_QUEUE))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}

	mw_turn_on_led(led);
}

void mw_led_stop_secondary_flash()
{
	mw_turn_off_led(m_led_secondary_flash.led);
	xTimerStop(m_led_flash_interval_secondary_timer, OSTIMER_WAIT_FOR_QUEUE);
}

/**********************************************************************************************/
/*********************************** LED Blinking *********************************************/
static void led_blink_timeout_handler(TimerHandle_t xTimer)
{
	uint32_t next_phase_duration;

	if(m_led_blink.count == 0)
	{
		m_led_blink.state = false;
		mw_turn_off_led(m_led_blink.led);
		m_led_callback(LED_BLINK_FINISHED);
		return;
	}

	if(m_led_blink.state)
	{
		m_led_blink.count--;
		mw_turn_off_led(m_led_blink.led);
		m_led_blink.state = false;
		next_phase_duration = m_led_blink.duration_off;
	}
	else
	{
		mw_turn_on_led(m_led_blink.led);
		m_led_blink.state = true;
		next_phase_duration = m_led_blink.duration_on;
	}

	xTimerChangePeriod(m_led_blink_interval_timer, next_phase_duration, OSTIMER_WAIT_FOR_QUEUE);

	//Restart Timer
	if (pdPASS != xTimerStart(m_led_blink_interval_timer, OSTIMER_WAIT_FOR_QUEUE))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
}


void mw_led_start_blinks( uint32_t led, uint32_t duration_on, uint32_t duration_off, uint8_t blinks)
{
	m_led_blink.led 					= led;
	m_led_blink.duration_on 	= duration_on;
	m_led_blink.duration_off 	= duration_off;
	m_led_blink.count 				= blinks;
	m_led_blink.state					= true;

	xTimerStop(m_led_blink_interval_timer, OSTIMER_WAIT_FOR_QUEUE);
	xTimerChangePeriod(m_led_blink_interval_timer, duration_on, OSTIMER_WAIT_FOR_QUEUE);

	if (pdPASS != xTimerStart(m_led_blink_interval_timer, OSTIMER_WAIT_FOR_QUEUE))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}

	mw_turn_on_led(led);
}

void mw_led_stop_blinks()
{
	mw_turn_off_led(m_led_blink.led);
	xTimerStop(m_led_blink_interval_timer, OSTIMER_WAIT_FOR_QUEUE);
}


/**********************************************************************************************/
/*********************************** LED Solid ON *********************************************/
/**
 * @brief - Turn OFF Led which is currently solid ON
 */
static void led_on_timeout_handler(TimerHandle_t xTimer)
{
  if(m_switch_after_solid_on)
  {
    if(m_led_on.state)
    {
      mw_turn_off_led(m_led_on.led);
      m_led_on.state = false;
    }
    else
    {
      mw_turn_on_led(m_led_on.led);
      m_led_on.state = true;
    }
  }
  m_led_callback(LED_SOLID_ON_FINISHED);
}


/**
 * @brief - Start Turn ON Timer
 */
void mw_led_solid_start( uint32_t led, bool start_state, uint32_t duration_msec, bool switch_at_the_end )
{
  if(start_state)
  {
    mw_turn_on_led(led);
    m_led_on.state = true;
  }
  else
  {
    mw_turn_off_led(led);
    m_led_on.state = false;
  }
  m_led_on.led = led;

  m_switch_after_solid_on = switch_at_the_end;

  xTimerStop(m_led_on_timer, OSTIMER_WAIT_FOR_QUEUE);
  xTimerChangePeriod(m_led_on_timer, duration_msec, OSTIMER_WAIT_FOR_QUEUE);

  if (pdPASS != xTimerStart(m_led_on_timer, OSTIMER_WAIT_FOR_QUEUE))
  {
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

/**
 * @brief - Start Turn ON Timer
 */
void mw_led_on_stop()
{
  xTimerStop(m_led_on_timer, OSTIMER_WAIT_FOR_QUEUE);
  mw_turn_off_led(m_led_on.led);
}


/**
 * @brief - Manual Turn ON of an LED
 */
void mw_turn_on_led( uint32_t led )
{
#ifdef BOARD_PCA10040
	nrf_gpio_pin_clear(led);
#else
	nrf_gpio_pin_set(led);
#endif
}


/**
 * @brief - Manual Turn OFF of an LED
 */
void mw_turn_off_led( uint32_t led )
{
#ifdef BOARD_PCA10040
	nrf_gpio_pin_set(led);
#else
	nrf_gpio_pin_clear(led);
#endif
}


void mw_led_soft_transition( uint32_t led, uint8_t direction )
{
	mw_pwm_start(led, direction);
	led_pwm = led;
}

void mw_led_stop_soft_transition()
{
	mw_pwm_stop();
}

void mw_led_pwm_off()
{
	mw_pwm_off();
}


/**
 * @brief - Stops all potential sequences
 */
void mw_led_all_functions_off()
{
  mw_pwm_off();
  mw_led_stop_blinks();
  mw_led_stop_secondary_flash();
  mw_led_stop_flash_0();
  mw_led_stop_flash_1();
}


/**
 * @brief - Initialize all sequence functions for module
 */
void mw_led_functions_init( mw_led_callback_t callback )
{
	// Create timers.
	m_led_flash_interval_timer_0 = xTimerCreate("FLASH_LED0", LED_FLASH_INTERVAL, pdTRUE, NULL, led_flash_0_timeout_handler);
	if ((NULL == m_led_flash_interval_timer_0))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}

  m_led_flash_interval_timer_1 = xTimerCreate("FLASH_LED1", LED_FLASH_INTERVAL, pdTRUE, NULL, led_flash_1_timeout_handler);
  if ((NULL == m_led_flash_interval_timer_1))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

	// Create timers.
	m_led_flash_interval_secondary_timer = xTimerCreate("FLASH_LED_SEC", LED_FLASH_INTERVAL, pdTRUE, NULL, led_flash_secondary_timeout_handler);
	if ((NULL == m_led_flash_interval_secondary_timer))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}


	m_led_blink_interval_timer = xTimerCreate("BLINK_LED", LED_BLINK_INTERVAL_FAST, pdFALSE, NULL, led_blink_timeout_handler);
	if ((NULL == m_led_blink_interval_timer))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}


	m_led_on_timer = xTimerCreate("ON_LED", LED_SOLID_ON_INTERVAL, pdFALSE, NULL, led_on_timeout_handler);
  if ((NULL == m_led_on_timer))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }


	m_led_callback = callback;

	pwm_module_params_t pwm_params;
	pwm_params.pin 										= LED_1;
	pwm_params.callback 							= led_pwm_handler;

	mw_pwm_peripheral_init(pwm_params);
}
