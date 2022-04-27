/*
 * mw_power_management.c
 *
 *  Created on: Oct 1, 2019
 *      Author: klockwood
 */


#include "nrf_gpio.h"
#include "boards.h"
#include "project_board.h"
#include "mw_power_management.h"


bool mw_set_5V_rail( rail_mode_t mode )
{
  bool return_value = true;

  switch(mode)
  {
  case OFF:
    nrf_gpio_pin_clear(PIN_5V_SHDN);
    nrf_gpio_pin_clear(PIN_5V_MODE);
    break;

  case ON_BURST_MODE:
    nrf_gpio_pin_set(PIN_5V_SHDN);
    nrf_gpio_pin_set(PIN_5V_MODE);
    break;

  case ON_FIXED_PWM:
    nrf_gpio_pin_set(PIN_5V_SHDN);
    nrf_gpio_pin_clear(PIN_5V_MODE);
    break;

  default:
    return_value = false;
    break;
  }

  return return_value;
}



bool mw_set_artic_3V3_rail( rail_mode_t mode )
{
  bool return_value = true;

  switch(mode)
  {
  case OFF:
    nrf_gpio_pin_set(PIN_ARTIC_ON_3V3);
    break;

  case ON:
    nrf_gpio_pin_clear(PIN_ARTIC_ON_3V3);
    break;

  default:
    return_value = false;
    break;
  }

  return return_value;
}


bool mw_set_sensors_3V3_rail( rail_mode_t mode )
{
  bool return_value = true;

  switch(mode)
  {
  case OFF:
    nrf_gpio_pin_set(PIN_SENS_3V3_EN);
    break;

  case ON:
    nrf_gpio_pin_clear(PIN_SENS_3V3_EN);
    break;

  default:
    return_value = false;
    break;
  }

  return return_value;
}


bool mw_set_master_3V3_rail( rail_mode_t mode )
{
  bool return_value = true;

  switch(mode)
  {
  case OFF:
    //nrf_gpio_pin_set(PIN_POWER_LINE_CNTRL);
    nrf_gpio_pin_clear(PIN_POWER_LINE_CNTRL);
    break;

  case ON:
    //nrf_gpio_pin_clear(PIN_POWER_LINE_CNTRL);
    nrf_gpio_pin_set(PIN_POWER_LINE_CNTRL);
    break;

  default:
    return_value = false;
    break;
  }

  return return_value;
}



bool mw_set_1V8_rail( rail_mode_t mode )
{
  bool return_value = true;

  switch(mode)
  {
  case OFF:
    nrf_gpio_pin_clear(PIN_1V8_EN);
    nrf_gpio_pin_clear(PIN_1V8_MODE);
    break;

  case ON_PWM_MODE:
    nrf_gpio_pin_set(PIN_1V8_EN);
    nrf_gpio_pin_set(PIN_1V8_MODE);
    break;

  case ON_HYSTERESIS_MODE:
    nrf_gpio_pin_set(PIN_1V8_EN);
    nrf_gpio_pin_clear(PIN_1V8_MODE);
    break;

  default:
    return_value = false;
    break;
  }

  return return_value;
}

void mw_power_management_init()
{
  /* Initial Power Management Pins */
  nrf_gpio_cfg_output(PIN_1V8_EN);
  nrf_gpio_cfg_output(PIN_ARTIC_ON_3V3);
  nrf_gpio_cfg_output(PIN_5V_SHDN);
  nrf_gpio_cfg_output(PIN_5V_MODE);
  nrf_gpio_cfg_output(PIN_SENS_3V3_EN);
  nrf_gpio_cfg_output(PIN_POWER_LINE_CNTRL);

  /* Default Power Management Settings */
  mw_set_master_3V3_rail(ON);
  mw_set_1V8_rail(OFF);
  mw_set_artic_3V3_rail(OFF);
  mw_set_5V_rail(OFF);
  mw_set_sensors_3V3_rail(OFF);

  /* Disconnect ARTIC pins */
  nrf_gpio_input_disconnect(SPIM2_MISO_PIN);
  nrf_gpio_input_disconnect(SPIM2_MOSI_PIN);
  nrf_gpio_input_disconnect(SPIM2_SCK_PIN);
  nrf_gpio_input_disconnect(SPIM2_SS_PIN);
  nrf_gpio_input_disconnect(PIN_ARTIC_INT1);
  nrf_gpio_input_disconnect(PIN_ARTIC_INT2);

  /* Disconnect PA pins */
  nrf_gpio_input_disconnect(PIN_PA_G8);
  nrf_gpio_input_disconnect(PIN_PA_G16);

  /* RTC and Sensor pins */
  nrf_gpio_input_disconnect(TWI0_SCL_PIN);
  nrf_gpio_input_disconnect(TWI0_SDA_PIN);
  nrf_gpio_input_disconnect(PIN_RTC_nTIRQ);
  nrf_gpio_input_disconnect(PIN_RTC_nIRQ2);
  nrf_gpio_input_disconnect(TWI1_SCL_PIN);
  nrf_gpio_input_disconnect(TWI1_SDA_PIN);
  nrf_gpio_input_disconnect(PIN_ACC_INT_XL);

}
