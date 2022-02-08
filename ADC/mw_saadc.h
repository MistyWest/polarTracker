/*
 * mw_saadc.h
 *
 *  Created on: Jan 25, 2019
 *      Author: klockwood
 */

#ifndef MW_SAADC_H_
#define MW_SAADC_H_


#include "nrfx_saadc.h"
#include "mw_saadc_settings.h"

typedef struct
{
	bool saadc_started;
	bool saadc_initialized;
	bool saadc_calibrated;
	bool saadc_referenced;
}mw_saadc_states_t;

typedef enum
{
	MW_SAADC_UNCALIBRATED,
	MW_SAADC_CALIBRATING,
	MW_SAADC_CALIBRATED
}mw_saadc_calibrate_t;

typedef enum
{
	NEW_SAADC_MEASUREMENT_EVENT,
	NEW_SAADC_SINGLE_MEASUREMENT_EVENT,
	NEW_SAADC_CALIBRATION_EVENT,
}mw_saadc_event_t;

/**@brief SAADC callback type. */
typedef void (* mw_saadc_callback_t) ( nrfx_saadc_evt_t const * saadc_result, mw_saadc_event_t event );

typedef struct
{
	uint8_t											channel;
	nrf_saadc_channel_config_t	channel_config;
}mw_saadc_channel_config_t;


// Module mode of measurement
typedef enum
{
	MANUAL_MEASUREMENT_MODE,
	AUTO_MEASUREMENT_MODE
}mw_saadc_measurement_mode_t;


typedef uint32_t mw_saadc_measurement_interval_t;

#define ADC_NULL_RESULT   -9999

// Module Measurement Configuration
typedef struct
{
	mw_saadc_measurement_mode_t			mode;
	mw_saadc_measurement_interval_t	interval_usec;
	mw_saadc_measurement_interval_t interval_ticks;
}mw_saadc_measurement_config_t;


// Module Initialization Settings
typedef struct
{
	nrfx_saadc_config_t 						saadc_config;
	mw_saadc_callback_t 						callback;
	mw_saadc_channel_config_t				channel_configuration[MW_SAADC_MAXIMUM_CHANNELS];
	mw_saadc_measurement_config_t	  measurement_config;
}mw_saadc_configuration_t;


/**@brief Convert SAADC Measurement to Voltage
 *
 */
float mw_saadc_convert_to_volts( int16_t adc_value);


/**@brief Update MW ADC Module Sampling Interval
 *
 * @param must be in units of milliseconds
 *
 */
void mw_saadc_update_interval( uint32_t update_msec );


/**@brief Update MW ADC Module Sampling Interval in ticks
 *
 * @param must be in ticks
 *
 */
void mw_saadc_update_interval_ticks( uint32_t update_ticks );


/**@brief Set MW ADC Module Sampling Interval in ticks
 *
 * @param must be in ticks
 *
 */
void mw_saadc_set_interval_ticks( uint32_t update_ticks );


/**@brief Start ADC Measurement Timer
 *
 * @details This function will start Measurement Timer
 *
 */
void mw_saadc_start_timer();


/**@brief Stop ADC Measurement Timer
 *
 * @details This function will stop Measurement Timer
 *
 */
void mw_saadc_stop_timer();


/**@brief Single ADC Measurement
 *
 * @details This function will start a trigger Measurement
 *
 * @return - ADC Single Measument uint16_t
 */
int16_t mw_saadc_single_measurement( uint8_t channel );


/**@brief Pause Automatic ADC Sampling
 *
 * @details This function will Pause automated ADC measurements
 *
 */
void mw_saadc_pause();

/**@brief Resume Automatic ADC Sampling
 *
 * @details This function will Resume automated ADC measurements
 *
 */
void mw_saadc_resume();


/**
 * @brief Manual Measurement complete? *
 */
bool mw_saadc_manual_measurement_in_progress();


/**@brief Measure ADC
 *
 * @details This function will trigger an ADC measurement
 *
 * @param:  mode - Timer based measurements or single manual measurement
 *
 */
void mw_saadc_start();


/**@brief Stop SAADC Measurement
 *
 * @details This function will start an SAADC measurement
 *
 */
void mw_saadc_stop();


/**@brief Blocking call to calculate Calibration offsets
 * *
 */
void mw_saadc_calibrate();


/**@brief Converts the SAADC ADC value to float
 * *
 */
float mw_saadc_convert_adc_to_volts( uint16_t adc );


/**@brief Flushes MW ADC Module conversion buffers
 *
 * @details This function will flush all SAADC buffers
 *
 */
//@flush
void mw_saadc_flush_buffers();

/**@brief Uninitialize MW SAADC Module
 *
 * @details This function will uninitialize all channels and SAADC
 *
 */
void mw_saadc_uninit();

/**@brief Re-Initialize MW ADC Module
 *
 * @details This function will be called to re-initialize ADC
 *
 */
void mw_saadc_re_init();

/**@brief Initialize MW SAADC Module
 *
 * @details This function will be called to initialize SAADC
 *
 */
void mw_saadc_init( mw_saadc_configuration_t adc_config );


#endif /* MW_SAADC_H_ */
