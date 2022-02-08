/*
 * wwf_device_settings.h
 *
 *  Created on: Feb 18, 2020
 *      Author: klockwood
 */

#ifndef WWF_DEVICE_SETTINGS_H_
#define WWF_DEVICE_SETTINGS_H_

#include "project_settings.h"
#include "mw_rtc_thread.h"

/* Number of Argos Transmission Repeats per Transmission Cycle */
#define DEFAULT_TRANSMISSION_REPITIIONS       60


/* Interval time (milliseconds) between repeated Argos Transmissions */
#define DEFAULT_TRANSMISSION_INTERVAL        (30 * 1000)//(90 * 1000)  /* 90 seconds */


/* Time Duration of a Data Gather Time Slot */
#define PROFILE_DATA_GATHER_INTERVAL         (5)
#define PROFILE_DATA_GATHER_INTERVAL_UNITS   RTC_MINUTES
/*  RTC_SECONDS
    RTC_MINUTES
    RTC_HOURS
    RTC_DAYS   */



/* Number of Data Time Slots Gathers before Transmitting */
#define PROFILE_DATA_GATHER_INDEX_MAX        2 //(14)   /*Sensor Capture Windows*/


/* Default Bear ID value */
#define DEFAULT_BEAR_ID_AND_PROFILE          0xACE // 0xACE  0xDAD


/******************************************
 * ARGOS ID associated with MW account
 * 173498  ->  0x2B978AD
 * 173499  ->  0x2B978BE
 * 173500  ->  0x2B978C7
*******************************************/
#define DEFAULT_ARGOS_28BIT_ID_FIELD         0x2B978AD

/*
 * Project Timer Intervals *
 */
#ifdef BATTERY_MEASUREMENT_INTERVAL
#undef BATTERY_MEASUREMENT_INTERVAL
#define BATTERY_MEASUREMENT_INTERVAL         60000         /**< Battery level measurement interval (ms). */
#endif


#ifdef TEMPERATURE_TIME_INTERVAL
#undef TEMPERATURE_TIME_INTERVAL
#define TEMPERATURE_TIME_INTERVAL            300000
#endif

#endif /* WWF_DEVICE_SETTINGS_H_ */
