/*
 * TMP75C.h
 *
 *  Created on: Feb 28, 2019
 *      Author: klockwood
 */

#ifndef EXTERNAL_TMP75C_H_
#define EXTERNAL_TMP75C_H_

#include "_mw_external_device.h"

#define TMP75C_TWI_DEVICE_ADDRESS     0x48

/* TMP75C Common Regiser Values */
#define TMP75C_ONE_SHOT_CONFIG_9BIT     0x85
///#define TMP75C_ONE_SHOT_CONFIG        0xB7
#define TMP75C_ACTIVE                 0x00
#define TMP75C_SHUTDOWN               0x01

/* TMP75C register banks */
#define TMP75C_POINTER_REG            0x00
#define TMP75C_TEMPERATURE_REG        0x00
#define TMP75C_CONFIGURATION_REG      0x01
#define TMP75C_LOWER_LIMIT_REG        0x02
#define TMP75C_UPPER_LIMIT_REG        0x03
#define TMP75C_ONE_SHOT_REG           0x04

#define TMP75C_SCALE_FACTOR_12BIT    (0.0625f)
#define TMP75C_SCALE_FACTOR_11BIT    (0.125f)
#define TMP75C_SCALE_FACTOR_10BIT    (0.25f)
#define TMP75C_SCALE_FACTOR_9BIT     (0.5f)

/**
 * @brief - set Lower Temperature Limit
 */
void TMP75C_set_lower_limit( float limit, float scale_factor );

/**
 * @brief - set Upper Temperature Limit
 */
void TMP75C_set_upper_limit( float limit, float scale_factor );

/**
 * @brief - write to Configuration Register
 */
void TMP75C_set_configure( uint8_t config );

/**
 * @brief - read from Configuration Register
 */
uint8_t TMP75C_get_configure();


/**
 * @brief - start/trigger a One Shot Measurement
 */
void TMP75C_start_one_shot_measurement();


/**
 * @brief - read last Temperature Measurement
 *
 * @return - Returns a Float value in deg C
 */
float TMP75C_read_temperature( float conversion_factor );


/**
 * @brief TMP75C Driver Initialization
 *
 * Initialzation parameter contains all interface configurations
 */
void TMP75C_initialize( external_device_config_t device_config );

#endif /* EXTERNAL_TMP75C_H_ */
