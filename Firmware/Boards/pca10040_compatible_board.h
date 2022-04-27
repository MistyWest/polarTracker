/*
 * ww02_miniboard_r0_1.h
 *
 *  Created on: Sep 30, 2019
 *      Author: klockwood
 */

#ifndef BOARDS_WW02_MINIBOARD_R0_1_H_
#define BOARDS_WW02_MINIBOARD_R0_1_H_

/* Power Managment Control/Mode Pins */
#define PIN_1V8_EN            17    // P0.30
#define PIN_1V8_MODE          18    // P0.31
#define PIN_ARTIC_ON_3V3      19    // P0.20
#define PIN_3V3_MODE          20    // P0.29

#define PIN_5V_SHDN           13    // P0.26
#define PIN_5V_MODE           14    // P0.27

#define PIN_SENS_3V3_EN       15    // P0.11

#define PIN_POWER_LINE_CNTRL  29    // P0.29

#define PIN_BATTERY_SAMPLE_EN 19    // P0.19
#define PIN_BATTERY_VOLTAGE   28    // P0.28


/* Temperature Flag Pin */
#define PIN_nCOLD             12    // P0.12


/* LED */
#define PIN_LED               10    // P0.10

#define PIN_ARTIC_INT1        22    // P0.22
#define PIN_ARTIC_INT2        23    // P0.23

/* PA control */
#define PIN_PA_G8             6
#define PIN_PA_G16            8


//SPI pin out
#define SPIS_MISO_PIN   28  // SPI MISO signal.
#define SPIS_CSN_PIN    12  // SPI CSN signal.
#define SPIS_MOSI_PIN   25  // SPI MOSI signal.
#define SPIS_SCK_PIN    29  // SPI SCK signal.

#define SPIM0_SCK_PIN   29  // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN  25  // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN  28  // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS_PIN    12  // SPI Slave Select GPIO pin number.

#define SPIM1_SCK_PIN   2   // SPI clock GPIO pin number.
#define SPIM1_MOSI_PIN  3   // SPI Master Out Slave In GPIO pin number.
#define SPIM1_MISO_PIN  4   // SPI Master In Slave Out GPIO pin number.
#define SPIM1_SS_PIN    5   // SPI Slave Select GPIO pin number.

#define SPIM2_SCK_PIN   15  // SPI clock GPIO pin number.
#define SPIM2_MOSI_PIN  12  // SPI Master Out Slave In GPIO pin number.
#define SPIM2_MISO_PIN  11  // SPI Master In Slave Out GPIO pin number.
#define SPIM2_SS_PIN    16  // SPI Slave Select GPIO pin number.

//TWI or I2C Pin-out
#define TWI0_SDA_PIN   26  // I2C 1 Data Line.
#define TWI0_SCL_PIN   27  // I2C 1 Clock Line.

//TWI or I2C Pin-out
#define TWI1_SCL_PIN   24  // I2C 1 Data Line.
#define TWI1_SDA_PIN   25  // I2C 1 Clock Line.

#define PIN_ACC_INT_XL  28
#define PIN_RTC_nTIRQ   28
#define PIN_RTC_nIRQ2   28


#endif /* BOARDS_WW02_MINIBOARD_R0_1_H_ */
