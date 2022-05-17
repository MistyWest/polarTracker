/*
 * mw_saadc.c
 *
 *  Created on: Jan 25, 2019
 *      Author: klockwood
 */



#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nordic_common.h"
#include "nrf_log.h"
#include "nrf_error.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "sdk_errors.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "app_timer.h"
#include "nrfx_ppi.h"
#include "nrfx_saadc.h"
#include "nrf_delay.h"

#include "project_settings.h"
#include "mw_saadc.h"

#include "../CLI_Logging/mw_logging.h"
#include "mw_saadc_settings.h"

#if NRFX_CHECK(NRFX_SAADC_ENABLED)

#define SAADC_LOGGING_ENABLED         0
#define SAADC_LOG_TAG                 "MW SAADC Driver: "

#define SAADC_TIMER_ENABLED           NRFX_TIMER_ENABLED_COUNT

#define NULL_CHANNEL									99

#if SAADC_TIMER_ENABLED
#define MW_SAADC_TIMER_COMPARE_FLAG  	NRFX_CONCAT_3(NRF_TIMER_SHORT_COMPARE, MW_SAADC_TIMER_INSTANCE, _CLEAR_MASK)
#define MW_SAADC_TIMER_CC_CHANNEL			NRFX_CONCAT_2(NRF_TIMER_CC_CHANNEL, MW_SAADC_TIMER_INSTANCE)
#endif

static volatile bool									saadc_initialized = false;
static volatile bool                  timer_initialized = false;
static volatile bool									saadc_measurement_in_progress = false;
static volatile bool									saadc_manual_measurement_mode = false;
static volatile bool									saadc_tear_down = false;
static volatile mw_saadc_calibrate_t	saadc_calibration = MW_SAADC_UNCALIBRATED;

static mw_saadc_configuration_t 		m_saadc_config;

static nrf_saadc_value_t						m_buffer_pool[2][MW_SAADC_SAMPLES_IN_BUFFER];

static nrf_saadc_value_t            m_manual_buffer_pool[MW_SAADC_SAMPLES_IN_BUFFER];

// HFCLK Variables
#if SAADC_TIMER_ENABLED
static nrfx_timer_t 								mw_saadc_timer = NRFX_TIMER_INSTANCE(MW_SAADC_TIMER_INSTANCE);
static nrf_ppi_channel_t 						m_ppi_channel;
#endif

static void mw_saadc_setup_hfclk();
static void mw_saadc_driver_uninit();
static void mw_saadc_driver_init();
static bool mw_saadc_configured_channel( mw_saadc_channel_config_t * channel );

static void mw_saadc_error_check( uint32_t err_code, uint8_t index )
{
	if(err_code != NRF_SUCCESS)
	{
		APP_ERROR_CHECK(err_code);
	}
}

//callback
static volatile bool manual_measure_test = false;
static void mw_saadc_callback( nrfx_saadc_evt_t const * p_event )
{
	uint32_t err_code;

	if (p_event->type == NRFX_SAADC_EVT_DONE)
	{
	  manual_measure_test  = true;
    if( saadc_tear_down )
    {
      return;
    }

		saadc_measurement_in_progress = false;				//ADC Sampling Complete

    if(!saadc_manual_measurement_mode)
    {
    	//Set buffer so the SAADC can write to it again.
			err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, MW_SAADC_SAMPLES_IN_BUFFER);
			if(err_code != NRF_SUCCESS)
			{
	      APP_ERROR_CHECK(err_code);
			}

			// Trigger calling application callback
			m_saadc_config.callback( p_event, NEW_SAADC_MEASUREMENT_EVENT );
    }
    else
    {
      // Trigger calling application callback - This was a single measurement
      m_saadc_config.callback( p_event, NEW_SAADC_SINGLE_MEASUREMENT_EVENT );

      saadc_manual_measurement_mode = false;
      saadc_measurement_in_progress = false;

      mw_saadc_driver_uninit();
    }
	}
	else if (p_event->type == NRFX_SAADC_EVT_LIMIT)
	{

		//Limit Callback
  }
	else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
	{
		m_saadc_config.callback( p_event, NEW_SAADC_CALIBRATION_EVENT );
		saadc_calibration = MW_SAADC_CALIBRATED;
  }
	else
	{
#if SAADC_LOGGING_ENABLED
		MW_LOG_INFO( SAADC_LOG_TAG "ADC OTHER Event Occurs\r\n");
#endif
	}
}

/**@brief TIMER module callback handler
 *
 */
void timer_handler(nrf_timer_event_t event_type,
    											void            * p_context)
{
}



/**@brief Calibrate MW ADC Module - Blocking call
 *
 * @details This function will trigger the ADC Calibration
 *
 */
/* SAADC calibration has a bug!!!!!!!!!!!!!!!!!!!!!!!!*/
void mw_saadc_calibrate()
{
  mw_saadc_driver_init();

  nrfx_saadc_abort();

	saadc_calibration = MW_SAADC_CALIBRATING;

	//Trigger calibration task
	while(nrfx_saadc_calibrate_offset() != NRF_SUCCESS)
	{
#ifdef FREERTOS
	  vTaskDelay(50);
#else
	  nrf_delay_ms(50);
#endif
	}

	while( saadc_calibration != MW_SAADC_CALIBRATED )
	{
		__WFE();
		__SEV();
		__WFE();
	}

	nrfx_saadc_abort();
	mw_saadc_driver_uninit();
}


/**@brief Convert SAADC Measurement to Voltage
 *
 */
float mw_saadc_convert_to_volts( int16_t adc_value)
{
	return (float)((adc_value) * MW_SAADC_REF_VOLTAGE) / ( MW_SAADC_RESOLUTION_VALUE * MW_SAADC_GAIN_VALUE);
}


/**@brief Update MW ADC Module Sampling Interval
 *
 * @param must be in units of microseconds
 *
 */
void mw_saadc_update_interval( uint32_t update_usec )
{
#if SAADC_TIMER_ENABLED

  timer_initialized = false;

  nrfx_timer_disable(&mw_saadc_timer);

  /* setup m_timer for compare event every interval */
  uint32_t ticks = nrfx_timer_us_to_ticks(&mw_saadc_timer, update_usec);
  nrfx_timer_extended_compare(&mw_saadc_timer,
                                  MW_SAADC_TIMER_CC_CHANNEL,
                                  ticks,
                                  MW_SAADC_TIMER_COMPARE_FLAG,
                                  false);

  nrfx_timer_enable(&mw_saadc_timer);
  timer_initialized = true;

#endif
}


/**@brief Update MW ADC Module Sampling Interval in ticks
 *
 * @param must be in ticks
 *
 */
void mw_saadc_update_interval_ticks( uint32_t update_ticks )
{
#if SAADC_TIMER_ENABLED

  timer_initialized = false;
  nrfx_timer_disable(&mw_saadc_timer);
  mw_saadc_set_interval_ticks( update_ticks );
  nrfx_timer_enable(&mw_saadc_timer);
  timer_initialized = true;
#endif
}


/**@brief Set MW ADC Module Sampling Interval in ticks
 *
 * @param must be in ticks
 *
 */
void mw_saadc_set_interval_ticks( uint32_t update_ticks )
{
#if SAADC_TIMER_ENABLED
  nrfx_timer_extended_compare(&mw_saadc_timer,
                              MW_SAADC_TIMER_CC_CHANNEL,
                              update_ticks,
                              MW_SAADC_TIMER_COMPARE_FLAG,
                              false);

  m_saadc_config.measurement_config.interval_ticks = update_ticks;
#endif
}


/**@brief Pause Automatic ADC Sampling
 *
 * @details This function will Pause automated ADC measurements
 *
 */
void mw_saadc_pause()
{
#if SAADC_TIMER_ENABLED
  if(timer_initialized) nrfx_timer_pause(&mw_saadc_timer);
#endif
}


/**@brief Resume Automatic ADC Sampling
 *
 * @details This function will Resume automated ADC measurements
 *
 */
void mw_saadc_resume()
{
  if(timer_initialized)
  {
#if SAADC_TIMER_ENABLED
    nrfx_timer_resume(&mw_saadc_timer);
    nrfx_ppi_channel_enable(m_ppi_channel);
#endif
  }
}


/**@brief Start ADC Measurement Timer
 *
 * @details This function will start Measurement Timer
 *
 */
void mw_saadc_start_timer()
{
  saadc_manual_measurement_mode = false;
  if(! timer_initialized) mw_saadc_setup_hfclk();
  mw_saadc_resume();
}


/**@brief Single ADC Measurement
 *
 * @details This function will start a trigger Measurement
 *
 * @return - ADC Single Measument uint16_t
 */
int16_t mw_saadc_single_measurement( uint8_t channel )
{
  uint32_t err_code;
  uint8_t i=0;

  mw_saadc_driver_init();

  /* Iterate through all SAADC Channels previously configured */
  while ( i < MW_SAADC_MAXIMUM_CHANNELS )
  {
    if ( mw_saadc_configured_channel(&m_saadc_config.channel_configuration[i]) )
    {
      err_code = nrfx_saadc_sample_convert(i, &m_manual_buffer_pool[i]);
      if ( err_code != NRF_SUCCESS )
      {
        APP_ERROR_CHECK(err_code);
      }
    }
    i++;
  }

  saadc_manual_measurement_mode = false;
  saadc_measurement_in_progress = false;
  mw_saadc_driver_uninit();

  if( channel > i )
  {
    return ADC_NULL_RESULT;
  }

  return m_manual_buffer_pool[channel];
}



/**@brief Stop ADC Measurement Timer
 *
 * @details This function will stop Measurement Timer
 *
 */
void mw_saadc_stop_timer()
{
#if SAADC_TIMER_ENABLED
  nrfx_timer_clear(&mw_saadc_timer);
  nrfx_timer_disable(&mw_saadc_timer);
  nrfx_ppi_channel_disable(m_ppi_channel);
#endif
}

/**
 * @brief Manual Measurement complete? *
 */
bool mw_saadc_manual_measurement_in_progress()
{
  return saadc_measurement_in_progress && saadc_manual_measurement_mode;
}


/**@brief Measure ADC
 *
 * @details This function will trigger an ADC measurement
 *
 */
void mw_saadc_start()
{
  uint32_t err_code;

  mw_saadc_driver_init();

  /* On the off chance this was trigger in the middle of a Manual Measurement */
  while ( saadc_measurement_in_progress )
  {
    nrf_delay_ms(2);
  }

  saadc_manual_measurement_mode = false;

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], MW_SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again.
  if( err_code != NRF_SUCCESS )
  {
    APP_ERROR_CHECK(err_code);
  }
  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], MW_SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again.
  if( err_code != NRF_SUCCESS )
  {
    APP_ERROR_CHECK(err_code);
  }

  if ( m_saadc_config.measurement_config.mode == AUTO_MEASUREMENT_MODE )
  {
    mw_saadc_setup_hfclk();
  }

}


/**@brief Stop MW ADC Module
 *
 * @details This function will be called to Stop ADC State Machine
 *
 */
//@stop
void mw_saadc_stop()
{
	if( saadc_measurement_in_progress ) //If no measurement in progress
	{
#if SAADC_LOGGING_ENABLED
	  MW_LOG_INFO(SAADC_LOG_TAG "Busy, trying to trigger another sample");
#endif
    saadc_tear_down = true;//adc measurement in progress, set tear down flag
    while ( nrfx_saadc_is_busy() )
    {
      __WFE();
      __SEV();
      __WFE();
    }
	}

  MW_LOG_INFO( SAADC_LOG_TAG "Stopped");
  nrfx_saadc_abort();
  mw_saadc_driver_uninit();

	//irq_count = 0; //use for debugging
	saadc_tear_down = false; //adc measurement in progress, reset tear down flag

#if SAADC_LOGGING_ENABLED
	MW_LOG_INFO( SAADC_LOG_TAG "*** ADC Stopped ***\r\n"); //Clear the SAADC interrupt if set
#endif
}


static void mw_saadc_setup_hfclk()
{
#if SAADC_TIMER_ENABLED
  ret_code_t err_code;

	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
	timer_cfg.mode 					      = NRF_TIMER_MODE_TIMER;
	timer_cfg.bit_width 		      = NRF_TIMER_BIT_WIDTH_24; // NRF_TIMER_BIT_WIDTH_16  NRF_TIMER_BIT_WIDTH_32
	timer_cfg.frequency 		      = (nrf_timer_frequency_t)NRF_TIMER_FREQ_16MHz;

	if(timer_initialized == true) return;

	err_code = nrfx_timer_init(&mw_saadc_timer, &timer_cfg, timer_handler);
	APP_ERROR_CHECK(err_code);

	/* setup m_timer for compare event every interval */
	uint32_t ticks = m_saadc_config.measurement_config.interval_ticks;

	/* If ticks wasn't define and calling module used interval_usec instead */
	if( m_saadc_config.measurement_config.interval_usec )
	{
	  ticks = nrfx_timer_us_to_ticks(&mw_saadc_timer, (uint32_t)m_saadc_config.measurement_config.interval_usec);
	}

	nrfx_timer_extended_compare(&mw_saadc_timer,
															MW_SAADC_TIMER_CC_CHANNEL,
															ticks,
															MW_SAADC_TIMER_COMPARE_FLAG,
															false);

	/* setup ppi channel so that timer compare event is triggering sample task in SAADC */
	err_code = nrfx_ppi_channel_alloc(&m_ppi_channel);
	APP_ERROR_CHECK(err_code);

	uint32_t timer_compare_event_addr = nrfx_timer_compare_event_address_get(&mw_saadc_timer, MW_SAADC_TIMER_CC_CHANNEL);
	uint32_t saadc_sample_task_addr   = nrfx_saadc_sample_task_get();


	err_code = nrfx_ppi_channel_assign( m_ppi_channel,
																		  timer_compare_event_addr,
																			saadc_sample_task_addr);
  APP_ERROR_CHECK(err_code);

	err_code = nrfx_ppi_channel_enable(m_ppi_channel);
	APP_ERROR_CHECK(err_code);

	timer_initialized = true;
#endif
}

/** @brief  Checks if this SAADC channel has been configured or not
 *
 */
static bool mw_saadc_configured_channel( mw_saadc_channel_config_t * channel )
{
	return (( channel->channel_config.pin_n != NRF_SAADC_INPUT_DISABLED ) ||
				  ( channel->channel_config.pin_p != NRF_SAADC_INPUT_DISABLED ) );
}


/**@brief Uninitialize SAADC Module
 *
 * @details This function will be called to initialize ADC
 *
 */
static void mw_saadc_driver_uninit()
{
  uint32_t err_code;
  uint8_t i = 0;
  if(!saadc_initialized) return;

  //while ( m_saadc_config.channel_configuration[i].channel != NULL_CHANNEL )
  while( i < MW_SAADC_MAXIMUM_CHANNELS )
  {
    err_code = nrfx_saadc_channel_uninit(m_saadc_config.channel_configuration[i].channel);
    if ( err_code != NRF_SUCCESS )
    {
      MW_LOG_INFO( SAADC_LOG_TAG "Uninitialize Error: %d \r\n", err_code);
    }
    i++;
  }

  nrfx_saadc_uninit();

#if SAADC_LOGGING_ENABLED
  MW_LOG_INFO( SAADC_LOG_TAG "*** ADC Uninitialized ***\r\n");
#endif

  saadc_initialized = false;
}



/**@brief Initialize SAADC Module
 *
 * @details This function will be called to initialize ADC
 *
 */
static void mw_saadc_driver_init()
{
  uint32_t err_code;

  if(saadc_initialized)
  {
    return; // Already initialized
  }

  //************************************************************************
  // Initialize the SAADC Driver with saadc_config configuration
  if(m_saadc_config.saadc_config.interrupt_priority == 0)
  {
    err_code = 7;
  }
  err_code = nrfx_saadc_init(&m_saadc_config.saadc_config, mw_saadc_callback);

  if( err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE )
  {
    mw_saadc_error_check(err_code, 1);
  }
  //************************************************************************

  //************************************************************************
  // Initialize all channels passed in config
  uint8_t i=0;
  while( mw_saadc_configured_channel( &m_saadc_config.channel_configuration[i] ) )
  {
    err_code = nrfx_saadc_channel_init(m_saadc_config.channel_configuration[i].channel, &m_saadc_config.channel_configuration[i].channel_config);
    mw_saadc_error_check(err_code, 2);
    i++;
  }
  //************************************************************************

#if SAADC_LOGGING_ENABLED
  MW_LOG_INFO( SAADC_LOG_TAG "*** ADC Initialized ***");
#endif
  saadc_initialized = true;
  saadc_tear_down       = false;
  saadc_measurement_in_progress = false;
}



/**@brief Initialize MW ADC Module
 *
 * @details This function will be called to initialize ADC
 *
 */
void mw_saadc_init( mw_saadc_configuration_t config )
{
	memcpy( &m_saadc_config, &config, sizeof(config) );
	memset( m_buffer_pool, 0, sizeof(m_buffer_pool) );

	mw_saadc_driver_init();
}

#else
//#warning Enable NRFX_SAADC_ENABLED in sdk_config.h to use this module Or remove from makefile
#endif
