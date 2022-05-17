/*
 * mw_external_device.h
 *
 *  Created on: Feb 13, 2019
 *      Author: klockwood
 */

#ifndef EXTERNAL__MW_EXTERNAL_DEVICE_H_
#define EXTERNAL__MW_EXTERNAL_DEVICE_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <nrfx.h>
#include "nrf_error.h"

#include "mw_spi.h"
#include "mw_spi_master.h"

#include "mw_twi.h"
#include "mw_twi_master.h"


typedef enum
{
	SPI_COMMUNICATION = 55,  // Keep values non-zero
	TWI_COMMUNICATION
}external_device_communication_t;


// Struct pass to all External Device Drivers for initialization
typedef struct
{
	external_device_communication_t	    communication;
	mw_spim_device_config_t							spi_config;
	mw_twim_device_config_t							twi_config;
} external_device_config_t;


// Struct for drivers which is used SPI to store credentials
typedef uint8_t external_driver_spi_config_t;

// Struct for drivers which use TWI to store credentials
typedef struct
{
	int		twi_instance;
	int		slave_address;
}external_driver_twi_config_t;

#endif /* EXTERNAL__MW_EXTERNAL_DEVICE_H_ */


