/*
 * mw_sensor_thread.h
 *
 *  Created on: Jan 22, 2019
 *      Author: klockwood
 */

#ifndef THREADS_MW_SENSOR_THREAD_H_
#define THREADS_MW_SENSOR_THREAD_H_

typedef enum
{
  UNKNOWN,
  CHECKING_ORIENTATION,
  GOOD_ORIENTATION,
  BAD_ORIENTATION
}device_orientation_t;


typedef enum
{
  THRESHOLD_0_90    = 0,
  THRESHOLD_0_95,
  THRESHOLD_1_00,
  THRESHOLD_1_05,
  THRESHOLD_1_15,
  THRESHOLD_1_25,
  THRESHOLD_1_35,
  THRESHOLD_1_45,
  THRESHOLD_1_55,
  THRESHOLD_1_65,
  THRESHOLD_1_75,
  THRESHOLD_1_85,
  THRESHOLD_1_95,
  THRESHOLD_2_05,
  THRESHOLD_2_15,
  THRESHOLD_2_25,
  THRESHOLD_2_50,
  THRESHOLD_2_75,
  THRESHOLD_2_95,
  THRESHOLD_3_15,
  THRESHOLD_3_50
}actigraphy_threshold_look_up_t;


typedef void (* mw_sensor_orientation_check_external_handler_t) ( device_orientation_t const orientation_status, bool isr );


/**
 * @brief - Set Handler signal Orientation Check complete
 */
void mw_sensor_set_external_handler( mw_sensor_orientation_check_external_handler_t external_handler);


/**
 * @brief - Threshold look-up table
 */
float mw_sensor_threshold_look_up_table ( uint8_t index );


/**
 * @brief - Check Orientation for viable transmission
 */
device_orientation_t mw_sensor_check_orientation( bool isr );


/**
 * @brief - Set orientation threshold
 */
void mw_sensor_set_orientation_threshold( int16_t threshold );



/**
 * @brief - Get Temperature High
 */
float mw_sensor_get_temperature_hi(void);


/**
 * @brief - Get Temperature lo
 */
float mw_sensor_get_temperature_lo(void);

/**
 * @brief - Get Temperature Average
 */
float mw_sensor_get_temperature_avg(void);

/**
 * @brief - Get Current Temperature
 */
float mw_sensor_get_temperature_current(void);

/**
 * @brief - Reset Temperature Variables
 */
void mw_sensor_reset_temperature(void);

/**
 * @brief - Update actigraphy threshhold setting - Float
 *
 * @param - threshold - float value in G's
 */
void mw_sensor_update_activity_threshold( float threshold, bool isr );


/**
 * @brief - Reset Activity Counter
 */
float mw_sensor_get_actigraphy_threshold();


/**
 * @brief - Reset Activity Counter
 */
uint16_t mw_sensor_get_activity_counter(void);


/**
 * @brief - Reset Activity Counter
 */
void mw_sensor_reset_activity_counter(void);


/**
 * @brief Thread is ready
*/
bool mw_sensor_thread_ready(void);


/**
 * @ Thread Initialization
 */
void mw_sensor_thread_init(void);

#endif /* THREADS_MW_SENSOR_THREAD_H_ */
