/*
 * RFPA0133.h
 *
 *  Created on: Oct 10, 2019
 *      Author: klockwood
 */

#ifndef EXTERNAL_RFPA0133_H_
#define EXTERNAL_RFPA0133_H_

typedef enum
{
  FIVE_DB_MODE = 5,
  SIXTEEN_DB_MODE = 16,
  TWENTY_THREE_DB_MODE = 23,
  TWENTY_NINE_DB_MODE = 29,
}rfpa0133_gain_modes;


/**
 * @brief - Got to Low Power Mode
 */
void rfpa0133_enable_set_gain( rfpa0133_gain_modes gain );


/**
 * @brief - Got to Low Power Mode
 */
void rfpa0133_low_power_mode(void);


/**
 * @brief - Driver Initialization
 */
void rfpa0133_init(void);

#endif /* EXTERNAL_RFPA0133_H_ */
