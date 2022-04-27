/*
 * mw_flash_thread.h
 *
 *  Created on: Mar 8, 2019
 *      Author: sedmond
 */

#ifndef THREADS_MW_RTC_THREAD_H_
#define THREADS_MW_RTC_THREAD_H_

#include "AM0805AQ.h"


typedef enum
{
  RTC_SECONDS = 1,
  RTC_MINUTES = 60,
  RTC_HOURS = 3600,
  RTC_DAYS = 86400
}rtc_time_unit_t;

typedef enum
{
  RTC_TIME_NULL = 0,
  RTC_TIME_INIT,
  RTC_TIME_SET
}mw_rtc_time_mode_t;

typedef enum
{
  RTC_ALARM_NULL = 0,
  RTC_ALARM_SET,
  RTC_ALARM_DISABLED,
  RTC_ALARM_ALARMING
}mw_rtc_alarm_mode_t;

typedef struct
{
  mw_rtc_time_mode_t  time_mode;
  mw_rtc_alarm_mode_t alarm_mode;
}mw_rtc_mode_t;


typedef void (* mw_rtc_alarm_external_handler_t) ( mw_rtc_mode_t const rtc_status, bool isr );


/**
 * @brief - Set External Handler RTC Alarm
 */
void mw_rtc_set_external_handler( mw_rtc_alarm_external_handler_t external_handler );


/**
 * @brief - Accessor function to set RTC Time
 *
 */
uint8_t mw_rtc_set_time_raw( const uint8_t * data, uint8_t length, bool context );


/**
 * @brief - Accessor function to set RTC Time
 *
 */
void mw_rtc_set_time( time_reg_struct_t time_update, bool context );


/**
 * @brief - Accessor function to set RTC Date/Time
 *
* Note: Time data is bit scaled according to AM0805 datasheet
 */
void mw_rtc_auto_set_alarm( uint8_t days_from_now, rtc_time_unit_t units, bool context );


/**
 * @brief - Accessor function to get RTC Time
 *
 * MUST BE CALLED FROM main/normal Context NOT from ISR
 */
void mw_rtc_get_time( time_reg_struct_t * time_update );


/**
 * @brief - Accessor function to update RTC Alarm
 */
void mw_rtc_set_alarm( time_reg_struct_t alarm_update, alarm_repeat_t alarm_repeats );


/**
 * @brief - Accessor function to get RTC Time
 *
 * MUST BE CALLED FROM main/normal Context NOT from ISR
 */
void mw_rtc_get_alarm( time_reg_struct_t * time_update );


/**
 * @brief - Returns TRUE if the Thread is currently active
 */
bool mw_rtc_thread_ready( void );

/**
 * @brief Function for application main entry.
 */
void mw_rtc_thread_init( void );



#endif /* THREADS_MW_FLASH_THREAD_H_ */
