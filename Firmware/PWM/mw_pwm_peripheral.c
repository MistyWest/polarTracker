/*
 * mw_pwm_peripheral.c
 *
 *  Created on: Aug 11, 2017
 *      Author: klockwood
 */

#include "stdio.h"
#include "string.h"
#include "nrfx_common.h"
#include "nrfx_pwm.h"
#include "app_util_platform.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "mw_pwm_settings.h"
#include "mw_pwm_peripheral.h"

#if NRFX_PWM_ENABLED

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)

static uint8_t m_used = 0;

static pwm_module_params_t pwm_params;

static uint8_t m_soft_transition_phase;
static nrf_pwm_values_individual_t m_soft_transition_seq_values;
static nrf_pwm_sequence_t const m_soft_seq = {
		.values.p_individual = &m_soft_transition_seq_values,
		.length = NRF_PWM_VALUES_LENGTH(m_soft_transition_seq_values),
		.repeats = 0,
		.end_delay = 0 };

static bool pwm_module_active = false;

static bool m_transition_direction;

static nrfx_pwm_t m_pwm_1 = NRFX_PWM_INSTANCE( MW_PWM_INSTANCE );			// Ignore any compile errors only in "Problems" tab

mw_pwm_callback_t m_pwm_callback;

static void mw_pwm_pin_high( uint32_t led );
static void mw_pwm_pin_low( uint32_t led );

static void init_pwm_1( uint32_t pwm_pin );

static void pwm_1_handler( nrfx_pwm_evt_type_t event_type )
{
	uint8_t channel = m_soft_transition_phase >> 1;
	bool next_phase = false;
	uint16_t * p_channels;
	uint16_t value;
	if ( event_type == NRFX_PWM_EVT_FINISHED )
	{
		if ( m_transition_direction == SOFT_ON_TRANSITION )
		{
			p_channels = (uint16_t *) &m_soft_transition_seq_values;
			value = p_channels[channel];

			if ( value > MW_PWM_COUNT_DOWN_MID_LIMIT_VALUE )
			{
				value -= MW_PWM_COUNT_DOWN_LO_STEP_VALUE;
			}
			else
			{
				value -= MW_PWM_COUNT_DOWN_HI_STEP_VALUE;
			}

			if ( value < MW_PWM_COUNT_DOWN_HI_STEP_VALUE )
			{
				next_phase = true;
			}
			p_channels[channel] = value;

      if ( next_phase )
      {
        mw_pwm_stop();

        mw_pwm_pin_high(pwm_params.pin);

        //Send Callback
        pwm_params.callback(PWM_SOFT_DOWN_FINISHED);
      }
		}
		else
		{
			p_channels = (uint16_t *) &m_soft_transition_seq_values;
			value = p_channels[channel];

			if ( value > MW_PWM_COUNT_UP_MID_LIMIT_VALUE )
			{
				value += MW_PWM_COUNT_UP_LO_STEP_VALUE;
			}
			else
			{
				value += MW_PWM_COUNT_UP_HI_STEP_VALUE;
			}

			if ( value >= MW_PWM_COUNT_UP_TOP_LIMIT_VALUE )
			{
				next_phase = true;
			}

			p_channels[channel] = value;

      if ( next_phase )
      {
        nrfx_pwm_stop(&m_pwm_1, NULL);

        mw_pwm_pin_low(pwm_params.pin);

        //Send Callback
        pwm_params.callback(PWM_SOFT_UP_FINISHED);
      }
		}
	}
}

static void mw_pwm_pin_high( uint32_t led )
{
#ifdef BOARD_PCA10040
	nrf_gpio_pin_clear(led);
#else
	nrf_gpio_pin_set(led);
#endif
}

static void mw_pwm_pin_low( uint32_t led )
{
#ifdef BOARD_PCA10040
	nrf_gpio_pin_set(led);
#else
	nrf_gpio_pin_clear(led);
#endif
}

void mw_pwm_stop()
{
	bool ready_flag = false;
	while ( !ready_flag )
	{
		ready_flag = nrfx_pwm_stop(&m_pwm_1, true);
	}
	pwm_module_active = false;
}

void mw_pwm_off()
{
	mw_pwm_stop();
	mw_pwm_pin_low(pwm_params.pin);
}

void mw_pwm_start( uint32_t pin, bool direction )
{
	m_transition_direction = direction;

	//New pin to PWM
	if ( pwm_params.pin != pin )
	{
		pwm_params.pin = pin;
		nrfx_pwm_uninit(&m_pwm_1);
		m_used = 0;
		init_pwm_1(pin);
	}

	mw_pwm_stop();

	if ( m_transition_direction == SOFT_OFF_TRANSITION )
	{
    mw_pwm_pin_high(pin);
	  //mw_pwm_pin_low(pin);
    m_soft_transition_seq_values.channel_0 = 0;
		//m_soft_transition_seq_values.channel_0 = MW_PWM_COUNT_UP_TOP_LIMIT_VALUE;
	}
	else
	{
	  mw_pwm_pin_low(pin);
	  //mw_pwm_pin_high(pin);
	 // m_soft_transition_seq_values.channel_0 = 0;
	  m_soft_transition_seq_values.channel_0 = MW_PWM_COUNT_UP_TOP_LIMIT_VALUE;
	}

	m_soft_transition_seq_values.channel_1 = 0;
	m_soft_transition_seq_values.channel_2 = 0;
	m_soft_transition_seq_values.channel_3 = 0;
	m_soft_transition_phase = 1;

	nrfx_pwm_simple_playback(&m_pwm_1, &m_soft_seq, 1, NRFX_PWM_FLAG_LOOP);

	pwm_module_active = true;
}



static void init_pwm_1( uint32_t pwm_pin )
{
  uint32_t err_code;
  nrfx_pwm_config_t  pwm_config = { .output_pins = {
      pwm_pin | NRFX_PWM_PIN_INVERTED
#ifdef BOARD_PCA10040
      | NRFX_PWM_PIN_INVERTED
#endif
      , // channel 0
      NRFX_PWM_PIN_NOT_USED,             // channel 1
      NRFX_PWM_PIN_NOT_USED,             // channel 2
      NRFX_PWM_PIN_NOT_USED,             // channel 3
      },
      .irq_priority = APP_IRQ_PRIORITY_LOWEST,
      .base_clock   = MW_PWM_FREQUENCY,
      .count_mode   = MW_PWM_COUNT_MODE,
      .top_value    = MW_PWM_COUNT_DOWN_TOP_LIMIT_VALUE,
      .load_mode    = MW_PWM_LOAD_MODE,
      .step_mode    = MW_PWM_STEP_MODE };

  pwm_params.pin = pwm_pin;

  err_code = nrfx_pwm_init(&m_pwm_1, &pwm_config, pwm_1_handler);
  APP_ERROR_CHECK(err_code);
  m_used |= USED_PWM(1);
}


void mw_pwm_peripheral_init( pwm_module_params_t params )
{
	memcpy(&pwm_params, &params, sizeof(params));

	//init_pwm_0( pin_config.pins[0] );
	init_pwm_1(pwm_params.pin);
	//init_pwm_2( pin_config.pins[2] );
}

#endif
