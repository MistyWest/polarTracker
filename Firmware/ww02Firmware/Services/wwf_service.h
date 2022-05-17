/*
 * wwf_service.h
 *
 *  Created on: Nov 5, 2019
 *      Author: klockwood
 */

#ifndef WWF_SERVICE_H_
#define WWF_SERVICE_H_

/***************************************************/
/* Service Command IDs */
#define BEAR_ID_AND_PROFILE_UUID           0x01
#define ACTIGRAPHY_THRESHOLD_UUID          0x02
#define SET_TIME_UUID                      0x03
#define SET_LOCATION_UUID                  0x04
#define START_TRACKING                     0x05
#define GET_DEVICE_SUMMARY                 0x06
#define SET_ARGOS_USER_ID                  0x07
#define RUN_TEST_UUID                      0x08

#define SET_PA_GAIN                        0xAA


#define UPDATE_DEVICE_FW                   0xFA

/***************************************************/

#define WWF_DATA_TRACKING_UNIT_SECONDS     0
#define WWF_DATA_TRACKING_UNIT_MINUTES     1
#define WWF_DATA_TRACKING_UNIT_HOURS       2
#define WWF_DATA_TRACKING_UNIT_DAYS        3


#define BEAR_ID_AND_PROFILE_CHAR_SIZE      3
#define ACTIGRAPHY_THRESHOLD_CHAR_SIZE     2
#define SET_TIME_CHAR_SIZE                 9
#define SET_ARGOS_USER_ID_CHAR_SIZE        5

#define TEST_CMD_CHAR_SIZE                 3

/****************************************************/

#define ARTIC_POWERING_ON_AND_PROGRAMMING           0xB0
#define ARTIC_POWERING_ON_AND_PROGRAMMING_SUCCESS   0xB1
#define ARTIC_FIRMWARE_CONFIRMED                    0xB2

/****************************************************/

typedef void (* wwf_external_command_handler_t)( const uint8_t * data, uint8_t length );


/**
 * @brief - Set external handler function
 */
void wwf_set_external_command_handler_t( wwf_external_command_handler_t handler );


/**
 * @brief - Send data over WWF Service
 */
void wwf_send_data( uint8_t * data, uint16_t length );


/**
 * @brief - Initialize WWF Service
 */
void wwf_service_init( uint16_t * conn_handle );


#endif /* WWF_SERVICE_H_ */




