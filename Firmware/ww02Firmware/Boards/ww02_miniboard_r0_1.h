/*
 * ww02_miniboard_r0_1.h
 *
 *  Created on: Sep 30, 2019
 *      Author: klockwood
 */

#ifndef BOARDS_WW02_MINIBOARD_R0_1_H_
#define BOARDS_WW02_MINIBOARD_R0_1_H_

/* Power Managment Control/Mode Pins */
#define PIN_1V8_EN            30    // P0.30
#define PIN_1V8_MODE          31    // P0.31
#define PIN_ARTIC_ON_3V3      20    // P0.20
#define PIN_3V3_MODE          29    // P0.29

#define PIN_5V_SHDN           26    // P0.26
#define PIN_5V_MODE           27    // P0.27

#define PIN_SENS_3V3_EN       11    // P0.11

#define PIN_POWER_LINE_CNTRL  17    // P0.17

#define PIN_BATTERY_SAMPLE_EN 19    // P0.19
#define PIN_BATTERY_VOLTAGE   28    // P0.28


/* Temperature Flag Pin */
#define PIN_nCOLD             12    // P0.12


/* LED */
#define PIN_LED               10    // P0.10

/* ARTIC Data Interface */
#define SPIM2_MISO_PIN        2
#define SPIM2_MOSI_PIN        3
#define SPIM2_SCK_PIN         4
#define SPIM2_SS_PIN          5

#define PIN_ARTIC_INT1        22    // P0.22
#define PIN_ARTIC_INT2        23    // P0.23

/* PA control */
#define PIN_PA_G8             6
#define PIN_PA_G16            8



/* RTC Interface */
#define TWI0_SCL_PIN          15
#define TWI0_SDA_PIN          14

#define PIN_RTC_nTIRQ         7
#define PIN_RTC_nIRQ2         9


/* Sensor Interface */
#define TWI1_SCL_PIN          24
#define TWI1_SDA_PIN          25

#define PIN_ACC_INT_XL        13


/* CLI Interface */
#define RX_PIN_NUMBER         18
#define TX_PIN_NUMBER         16
#define CTS_PIN_NUMBER        7
#define RTS_PIN_NUMBER        5
#define HWFC                  false

#endif /* BOARDS_WW02_MINIBOARD_R0_1_H_ */
