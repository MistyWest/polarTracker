/*
 * mw_saadc_settings.h
 *
 *  Created on: Jan 28, 2019
 *      Author: klockwood
 */

#ifndef MW_SAADC_SETTINGS_H_
#define MW_SAADC_SETTINGS_H_

//#include "nrf_saadc.h"


/**********************************************************
 *
 * 				ADC SETTINGS
 *
 **********************************************************/
#define MW_SAADC_TIMER_INSTANCE                 1     // Use NRFX_TIMER1

#ifndef MW_SAADC_SAMPLE_INTERVAL
#define MW_SAADC_SAMPLE_INTERVAL                5000  // 5msec interval
#endif

#define MW_SAADC_MAXIMUM_CHANNELS		  					3		// Maximum number of channels for SAADC
#define MW_SAADC_SAMPLES_IN_BUFFER							MW_SAADC_MAXIMUM_CHANNELS //(MW_SAADC_MAXIMUM_CHANNELS * MW_SAADC_CONFIG_OVERSAMPLE)

#define MW_SAADC_ACQUIRE_MODE							      NRF_SAADC_BURST_DISABLED // NRF_SAADC_BURST_DISABLED NRF_SAADC_BURST_ENABLED

#define MW_SAADC_RESOLUTION											NRF_SAADC_RESOLUTION_12BIT
#define MW_SAADC_RESOLUTION_VALUE								(1U << (8 + ( 2 * MW_SAADC_RESOLUTION )))

#define MW_SAADC_REFERENCE											NRF_SAADC_REFERENCE_INTERNAL 	// NRF_SAADC_REFERENCE_INTERNAL NRF_SAADC_REFERENCE_VDD4
#define MW_SAADC_ACQ_TIME	 											NRF_SAADC_ACQTIME_40US; 			// NRF_SAADC_ACQTIME_3US NRF_SAADC_ACQTIME_10US NRF_SAADC_ACQTIME_15US NRF_SAADC_ACQTIME_20US NRF_SAADC_ACQTIME_40US
#define MW_SAADC_GAIN		 												NRF_SAADC_GAIN1_6 						// NRF_SAADC_GAIN1_4  NRF_SAADC_GAIN1_6

#if (MW_SAADC_GAIN == NRF_SAADC_GAIN1_6)
#define MW_SAADC_GAIN_VALUE											(0.166666666667f)
#elif (MW_SAADC_GAIN == NRF_SAADC_GAIN1_5)
#define MW_SAADC_GAIN_VALUE											(0.2f)
#elif (MW_SAADC_GAIN == NRF_SAADC_GAIN1_4)
#define MW_SAADC_GAIN_VALUE											(0.25f)
#elif (MW_SAADC_GAIN == NRF_SAADC_GAIN1_3)
#define MW_SAADC_GAIN_VALUE											(0.333333333333f)
#elif (MW_SAADC_GAIN == NRF_SAADC_GAIN1_2)
#define MW_SAADC_GAIN_VALUE											(0.5f)
#elif (MW_SAADC_GAIN == NRF_SAADC_GAIN1)
#define MW_SAADC_GAIN_VALUE											(1f)
#elif (MW_SAADC_GAIN == NRF_SAADC_GAIN2)
#define MW_SAADC_GAIN_VALUE											(2f)
#elif (MW_SAADC_GAIN == NRF_SAADC_GAIN4)
#define MW_SAADC_GAIN_VALUE											(4f)
#endif

#if (MW_SAADC_REFERENCE == NRF_SAADC_REFERENCE_INTERNAL)
#define MW_SAADC_REF_VOLTAGE										(0.6f) 												// Voltage value of MW_SAADC_REFERENCE setting
#else
#define MW_SAADC_REF_VOLTAGE										(0.825f) 											// Voltage value of MW_SAADC_REFERENCE setting
#warning Warning: verify this value equals VDD/4
#endif

// <o> MW_SAADC_CONFIG_RESOLUTION  - Resolution
// <0=> 8 bit
// <1=> 10 bit
// <2=> 12 bit
// <3=> 14 bit
#define MW_SAADC_CONFIG_RESOLUTION  2

// <o> MW_SAADC_CONFIG_OVERSAMPLE  - Sample period
// <0=> Disabled
// <1=> 2x
// <2=> 4x
// <3=> 8x
// <4=> 16x
// <5=> 32x
// <6=> 64x
// <7=> 128x
// <8=> 256x
#define MW_SAADC_CONFIG_OVERSAMPLE  0

// <q> MW_SAADC_CONFIG_LP_MODE  - Enabling low power mode
#define MW_SAADC_CONFIG_LP_MODE     0


// <o> MW_SAADC_CONFIG_IRQ_PRIORITY  - Interrupt priority
// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
// <4=> 4
// <5=> 5
// <6=> 6
// <7=> 7
#define MW_SAADC_CONFIG_IRQ_PRIORITY 3


/**
 * @brief Macro for setting ADC Pin Settings
 *        in single ended mode.
 *
 * @param PIN_P Analog input.
 */
#define MW_SAADC_DEFAULT_CHANNEL_CONFIG_SE (ADC_PIN) 	\
{																											\
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      	\
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      	\
    .gain       = NRF_SAADC_GAIN1_6,                	\
    .reference  = NRF_SAADC_REFERENCE_VDD4,     			\
    .acq_time   = NRF_SAADC_ACQTIME_10US,           	\
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      	\
    .burst      = NRF_SAADC_BURST_DISABLED,         	\
    .pin_p      = (nrf_saadc_input_t)(ADC_PIN),     	\
    .pin_n      = NRF_SAADC_INPUT_DISABLED          	\
}




/**
 * @brief Macro for setting ADC pin in Differential Mode
 *
 * @param PIN_P Positive analog input.
 * @param PIN_N Negative analog input.
 */
#define MW_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL (PIN_P, PIN_N)   \
{                                                                    \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,                       \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,                       \
    .gain       = NRF_SAADC_GAIN1_6,                                 \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,                      \
    .acq_time   = NRF_SAADC_ACQTIME_10US,                            \
    .mode       = NRF_SAADC_MODE_DIFFERENTIAL,                       \
    .pin_p      = (nrf_saadc_input_t)(PIN_P),                        \
    .pin_n      = (nrf_saadc_input_t)(PIN_N)                         \
}

#endif /* MW_SAADC_SETTINGS_H_ */
