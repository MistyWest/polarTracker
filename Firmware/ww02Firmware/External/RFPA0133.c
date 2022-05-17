/*
 * RFPA0133.c
 *
 *  Created on: Oct 10, 2019
 *      Author: klockwood
 */


#include "nrf_gpio.h"
#include "boards.h"
#include "project_board.h"
#include "RFPA0133.h"



/**
 * @brief - Got to Low Power Mode
 */
void rfpa0133_enable_set_gain( rfpa0133_gain_modes gain )
{
  switch(gain)
  {
  case FIVE_DB_MODE:
    nrf_gpio_pin_clear(PIN_PA_G8);
    nrf_gpio_pin_clear(PIN_PA_G16);
    break;

  case SIXTEEN_DB_MODE:
    nrf_gpio_pin_set(PIN_PA_G8);
    nrf_gpio_pin_clear(PIN_PA_G16);
    break;

  case TWENTY_THREE_DB_MODE:
    nrf_gpio_pin_clear(PIN_PA_G8);
    nrf_gpio_pin_set(PIN_PA_G16);
    break;

  case TWENTY_NINE_DB_MODE:
    nrf_gpio_pin_set(PIN_PA_G8);
    nrf_gpio_pin_set(PIN_PA_G16);
    break;
  }
}


/**
 * @brief - Got to Low Power Mode
 */
void rfpa0133_low_power_mode(void)
{
  nrf_gpio_pin_clear(PIN_PA_G8);
  nrf_gpio_pin_clear(PIN_PA_G16);
}


/**
 * @brief - Driver Initialization
 */
void rfpa0133_init(void)
{
  nrf_gpio_cfg_output(PIN_PA_G8);
  nrf_gpio_cfg_output(PIN_PA_G16);
}
