/*
 * mw_sensor_thread.c
 *
 *  Created on: Jan 22, 2019
 *      Author: klockwood
 */

/**@file
 * @defgroup mw_thread_adc ADC Thread
 * @{
 * @ingroup  mw_thread
 * @brief    ADC thread
 */

#include <stdint.h>
#include <string.h>
#include "nrf_error.h"
#include "nrfx_clock.h"
#include "nrf_gpio.h"
#include "FreeRTOS_includes.h"
#include "mw_thread.h"
#include "project_settings.h"
#include "mw_power_management.h"
#include "mw_adc_thread.h"
#include "mw_battery.h"

#include "../CLI_Logging/mw_logging.h"
#include "mw_saadc.h"

#define ADC_LOG_TAG								"Battery Thread: "
#define ADC_LOGGING_ENABLED       1

/* ADC Input */
#define ADC_INPUT_PIN             NRF_SAADC_INPUT_AIN4
#define ADC_INPUT_CHANNEL         0

#define MW_ADC_SAMPLE_DELAY       800

TaskHandle_t 											m_mw_adc_thread;        /**< Definition of Thread. */

SemaphoreHandle_t                 adc_semph;

static volatile mw_thread_mode_t  m_mw_adc_thread_mode = THREAD_NULL;

static volatile mw_adc_states_t 	m_adc_thread_states;

static int16_t 									  m_adc_result = 0;
static float                      m_adc_result_float = 0;

static void mw_enable_battery_sampling();
static void mw_disable_battery_sampling();

static void MW_RESUME_ADC_THREAD( bool isr);
static void MW_SUSPEND_ADC_THREAD();


static void adc_handler( nrfx_saadc_evt_t const * saadc_result, mw_saadc_event_t event )
{
	if( event == NEW_SAADC_MEASUREMENT_EVENT )
	{
		m_adc_result = saadc_result->data.done.p_buffer[0];
		m_adc_result_float = mw_saadc_convert_to_volts(m_adc_result);
	}
  if( event == NEW_SAADC_CALIBRATION_EVENT )
  {
  }
}


/**
 * @brief - Get Battery Level
 */
uint8_t adc_get_battery_level( float current_temperature )
{
  return mw_battery_calculate_percentage( m_adc_result_float, current_temperature );
}



/**@brief Function checking/logging ADC measurements
*/
void adc_measurement_logging()
{
  static float min_value = 100;
  static float max_value = 0;

  if ( min_value > m_adc_result_float )
  {
    min_value = m_adc_result_float;
  }

  if ( max_value < m_adc_result_float )
  {
    max_value = m_adc_result_float;
  }

#if ADC_LOGGING_ENABLED
  MW_LOG_INFO("******************************");
  MW_LOG_INFO(ADC_LOG_TAG "Current ADC Value is: " MW_FLOAT_VALUE "V", MW_LOG_FLOAT(m_adc_result_float));
  MW_LOG_INFO(ADC_LOG_TAG "Max ADC Value is: " MW_FLOAT_VALUE " V", MW_LOG_FLOAT(max_value));
  MW_LOG_INFO(ADC_LOG_TAG "Min ADC Value is: " MW_FLOAT_VALUE " V", MW_LOG_FLOAT(min_value));
  MW_LOG_INFO(ADC_LOG_TAG "ADC Value Range is: " MW_FLOAT_VALUE " V", MW_LOG_FLOAT(max_value - min_value));
  MW_LOG_INFO("******************************");
#endif
}



/**
 * @brief- Perform Single ADC SAADC measurement
 */
void adc_measure()
{
  mw_enable_battery_sampling();

  //mw_saadc_calibrate();
  m_adc_result = mw_saadc_single_measurement(ADC_INPUT_CHANNEL);
  m_adc_result_float = mw_saadc_convert_to_volts(m_adc_result);
  m_adc_result_float *= 2.007;  // Account for the resistor divider

#if ADC_LOGGING_ENABLED
  MW_LOG_INFO(ADC_LOG_TAG "Current ADC Value is: " MW_FLOAT_VALUE "V", MW_LOG_FLOAT(m_adc_result_float));
#endif
  mw_disable_battery_sampling();
}


/**
 * @brief - perform SAADC calibration
 */
void adc_calibrate()
{
  mw_enable_battery_sampling();

  mw_saadc_calibrate();

  mw_disable_battery_sampling();
}


static void mw_disable_battery_sampling()
{
  nrf_gpio_pin_clear(PIN_BATTERY_SAMPLE_EN);
  vTaskDelay(MW_ADC_SAMPLE_DELAY);
}


static void mw_enable_battery_sampling()
{
  nrf_gpio_pin_set(PIN_BATTERY_SAMPLE_EN);
  vTaskDelay(MW_ADC_SAMPLE_DELAY);
}


/**
 * @brief - Initializes the SAADC Driver for use in the this thread
 */
static void mw_adc_init()
{
	mw_saadc_configuration_t adc_config;
	memset(&adc_config, 0, sizeof(adc_config));

	adc_config.callback 																					= adc_handler;
	adc_config.channel_configuration[0].channel	 									= 0;
	adc_config.channel_configuration[0].channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	adc_config.channel_configuration[0].channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	adc_config.channel_configuration[0].channel_config.gain 			= MW_SAADC_GAIN;
	adc_config.channel_configuration[0].channel_config.reference 	= MW_SAADC_REFERENCE;
	adc_config.channel_configuration[0].channel_config.acq_time 	= MW_SAADC_ACQ_TIME
	adc_config.channel_configuration[0].channel_config.acq_time 	= MW_SAADC_ACQ_TIME
	;
	adc_config.channel_configuration[0].channel_config.mode 			= NRF_SAADC_MODE_SINGLE_ENDED;
	adc_config.channel_configuration[0].channel_config.burst 			= NRF_SAADC_BURST_ENABLED;
	adc_config.channel_configuration[0].channel_config.pin_p 			= ADC_INPUT_PIN;               // ADC Sampling pin
	adc_config.channel_configuration[0].channel_config.pin_n 			= NRF_SAADC_INPUT_DISABLED;

	adc_config.measurement_config.mode 														= MANUAL_MEASUREMENT_MODE;
	adc_config.measurement_config.interval_usec 									= MW_SAADC_SAMPLE_INTERVAL;  // 5000 msec interval

	adc_config.saadc_config.resolution 														= (nrf_saadc_resolution_t) MW_SAADC_CONFIG_RESOLUTION;
	adc_config.saadc_config.oversample 														= (nrf_saadc_oversample_t) MW_SAADC_CONFIG_OVERSAMPLE;
	adc_config.saadc_config.interrupt_priority 										= MW_SAADC_CONFIG_IRQ_PRIORITY;
	adc_config.saadc_config.low_power_mode 												= MW_SAADC_CONFIG_LP_MODE;

	mw_saadc_init( adc_config );

	mw_disable_battery_sampling();
}



//@resume
static void MW_RESUME_ADC_THREAD( bool isr)
{
  if ( isr )
  {
    if ( m_mw_adc_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_adc_thread_mode = THREAD_ACTIVE;
      xTaskResumeFromISR(m_mw_adc_thread); // Resume myself
    }
  }
  else
  {
    if ( m_mw_adc_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_adc_thread_mode = THREAD_ACTIVE;
      vTaskResume(m_mw_adc_thread); // Resume myself
    }
  }
}


//@suspend
static void MW_SUSPEND_ADC_THREAD()
{
  m_mw_adc_thread_mode = THREAD_SUSPENDED;
  vTaskSuspend(m_mw_adc_thread); // Suspend myself
}


/**
 * @brief Thread is ready
*/
bool mw_adc_thread_ready(void)
{
#if !ADC_THREAD_ENABLED
  return true;
#endif
  return m_mw_adc_thread_mode != THREAD_NULL;
}


/**@brief ADC Thread Task
*/
static void mw_adc_task(void * arg)
{
  vTaskDelay(MW_ADC_THREAD_START_UP_DELAY);

  /* Set ADC sampling OFF */
  nrf_gpio_cfg_output(PIN_BATTERY_SAMPLE_EN);
  mw_disable_battery_sampling();

	mw_adc_init();

#if ADC_LOGGING_ENABLED
	MW_LOG_INFO(ADC_LOG_TAG "Ready");
#endif

	m_adc_thread_states = ADC_IDLE;
	m_mw_adc_thread_mode = THREAD_INITIALIZED;

  while ( true )
  {
    adc_measure();
    adc_measurement_logging();
    vTaskDelay(ADC_THREAD_POLL_INTERVAL);
  }

  /* Keep compiler happy */
  MW_RESUME_ADC_THREAD(NORMAL_CONTEXT);
  MW_SUSPEND_ADC_THREAD();
}



/**@brief Function for application main entry.
*/
void mw_adc_thread_init(void)
{
	adc_semph = xSemaphoreCreateBinary();
	if (NULL == adc_semph)
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
	xSemaphoreGive(adc_semph);

	if(xTaskCreate( mw_adc_task, "ADC", ADC_THREAD_STACK_SIZE, NULL, ADC_THREAD_PRIORITY, &m_mw_adc_thread ) != pdPASS)
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}

	nrf_gpio_cfg_output(PIN_BATTERY_SAMPLE_EN);
}
