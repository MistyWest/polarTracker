/*
 * mw_battery.c
 *
 *  Created on: Apr 1, 2019
 *      Author: klockwood
 */

#include <Battery/mw_battery.h>
#include "nrf_gpio.h"

#include "FreeRTOS_includes.h"


float m_compensation_ratio = 1;
uint32_t m_charge_control;
uint32_t m_charge_status;
uint32_t m_usb_power;

mw_battery_charger_states_t m_battery_charger_states = MW_BATTERY_CHARGER_IDLE;

/******************************************************************************************************/
/**********************************   Battery Measurement Models **************************************/

static uint8_t battery_percentage_30C_to_60C( float battery_voltage )
{
  uint8_t new_battery_percentage = 0;

  if ( battery_voltage >= 4.15 )
  {
    new_battery_percentage = 100;
  }
  else if ( ( battery_voltage >= 4.0 ) && ( battery_voltage < 4.15 ) )
  {
    new_battery_percentage = 100 - ((4.15 - battery_voltage) * 10) / (4.15 - 4.0);    // 91-99%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.9 ) && ( battery_voltage < 4.0 ) )
  {
    new_battery_percentage = 91 - ((4.0 - battery_voltage) * 8) / (4.0 - 3.9);  //82%-90%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.85 ) && ( battery_voltage < 3.9 ) )
  {
    new_battery_percentage = 82 - ((3.9 - (battery_voltage)) * 8) / (3.9 - 3.85); // 73%-81%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.7 ) && ( battery_voltage < 3.85 ) )
  {
    new_battery_percentage = 73 - ((3.85 - battery_voltage) * 42) / (3.85 - 3.7);    //30-72%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.6 ) && ( battery_voltage < 3.7 ) )
  {
    new_battery_percentage = 32 - ((3.7 - battery_voltage) * 14) / (3.7 - 3.6);    //17-31%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.5 ) && ( battery_voltage < 3.6 ) )
  {
    new_battery_percentage = 17 - ((3.6 - battery_voltage) * 6) / (3.6 - 3.5);   //10-16
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.4 ) && ( battery_voltage < 3.5 ) )
  {
    new_battery_percentage = 10 - ((3.5 - battery_voltage) * 4) / (3.5 - 3.4);   //5-9%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.3 ) && ( battery_voltage < 3.4 ) )
  {
    new_battery_percentage = 5 - ((3.4 - battery_voltage) * 3) / (3.4 - 3.3);   //1-4%
    battery_voltage = 0;//reset value
  }
  else if (battery_voltage < 3.3)
  {
    new_battery_percentage = 0;
  }

  return new_battery_percentage;
}


static uint8_t battery_percentage_10C_to_30C( float battery_voltage )
{
  uint8_t new_battery_percentage = 0;

  if ( battery_voltage >= BATT_LEVEL_97 )
  {
    new_battery_percentage = 100;
  }
  else if ( ( battery_voltage >= 4.0 ) && ( battery_voltage < 4.15 ) )
  {
    new_battery_percentage = 100 - ((4.15 - battery_voltage) * 10) / (4.15 - 4.0);    // 91-99%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.9 ) && ( battery_voltage < 4.0 ) )
  {
    new_battery_percentage = 91 - ((4.0 - battery_voltage) * 8) / (4.0 - 3.9);  //82%-90%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.85 ) && ( battery_voltage < 3.9 ) )
  {
    new_battery_percentage = 82 - ((3.9 - (battery_voltage)) * 8) / (3.9 - 3.85); // 73%-81%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.6 ) && ( battery_voltage < 3.85 ) )
  {
    new_battery_percentage = 73 - ((3.85 - battery_voltage) * 40) / (3.85 - 3.6);    //32-72%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.5 ) && ( battery_voltage < 3.6 ) )
  {
    new_battery_percentage = 32 - ((3.6 - battery_voltage) * 15) / (3.6 - 3.5);   //16-31
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.4 ) && ( battery_voltage < 3.5 ) )
  {
    new_battery_percentage = 16 - ((3.5 - battery_voltage) * 8) / (3.5 - 3.4);   //7-15%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.3 ) && ( battery_voltage < 3.4 ) )
  {
    new_battery_percentage = 7 - ((3.4 - battery_voltage) * 5) / (3.4 - 3.3);   //1-6%
    battery_voltage = 0;//reset value
  }
  else if (battery_voltage < 3.3)
  {
    new_battery_percentage = 0;
  }

  return new_battery_percentage;
}



static uint8_t battery_percentage_0C_to_10C_temp( float battery_voltage )
{
  uint8_t new_battery_percentage = 0;

  if ( battery_voltage >= 4.1 )
  {
    new_battery_percentage = 100;
  }

  else if ( ( battery_voltage >= 4.0 ) && ( battery_voltage < 4.1 ) )
  {
    new_battery_percentage = 99 - ((4.1 - (battery_voltage)) * 5) / (4.1 - 4.0); // 94%-98%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.85 ) && ( battery_voltage < 4.0 ) )
  {
    new_battery_percentage = 94 - ((4.0 - battery_voltage) * 15) / (4.0 - 3.85);    //78-93%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.6 ) && ( battery_voltage < 3.85 ) )
  {
    new_battery_percentage = 78 - ((3.85 - battery_voltage) * 45) / (3.85 - 3.6);    //32-77%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.5 ) && ( battery_voltage < 3.6 ) )
  {
    new_battery_percentage = 32 - ((3.6 - battery_voltage) * 15) / (3.6 - 3.5);    //16-31%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.4 ) && ( battery_voltage < 3.5 ) )
  {
    new_battery_percentage = 16 - ((3.5 - battery_voltage) * 10) / (3.5 - 3.4);    //5-15%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.3 ) && ( battery_voltage < 3.4 ) )
  {
    new_battery_percentage = 5 - ((3.4 - battery_voltage) * 4) / (3.4 - 3.3);    //1-4%
    battery_voltage = 0;//reset value
  }

  else if ( battery_voltage < 3.3 )
  {
    new_battery_percentage = 0;
  }

  return new_battery_percentage;
}


static uint8_t battery_percentage_minus10C_to_0C_temp( float battery_voltage )
{
  uint8_t new_battery_percentage = 0;

  if ( battery_voltage >= 4.0 )
  {
    new_battery_percentage = 100;
  }

  else if ( ( battery_voltage >= 3.8 ) && ( battery_voltage < 4.0 ) )
  {
    new_battery_percentage = 99 - ((4 - (battery_voltage)) * 20) / (4.0 - 3.8); // 78%-98%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.5 ) && ( battery_voltage < 3.8 ) )
  {
    new_battery_percentage = 78 - ((3.4 - battery_voltage) * 56) / (3.8 - 3.5);    //21-77%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.4 ) && ( battery_voltage < 3.5 ) )
  {
    new_battery_percentage = 21 - ((3.5 - battery_voltage) * 8) / (3.5 - 3.4);    //5-20%
    battery_voltage = 0;//reset value
  }

  else if ( battery_voltage < 3.3 )
  {
    new_battery_percentage = 0;
  }

  return new_battery_percentage;
}


static uint8_t battery_percentage_minus20C_to_minus10C_temp( float battery_voltage )
{
  uint8_t new_battery_percentage = 0;

  if ( battery_voltage >= 4.0 )
  {
    new_battery_percentage = 100;
  }

  else if ( ( battery_voltage >= 3.8 ) && ( battery_voltage < 4.0 ) )
  {
    new_battery_percentage = 99 - ((4 - (battery_voltage)) * 20) / (4.0 - 3.8); // 78%-98%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.4 ) && ( battery_voltage < 3.8 ) )
  {
    new_battery_percentage = 78 - ((3.4 - battery_voltage) * 63) / (3.8 - 3.4);    //14-77%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.3 ) && ( battery_voltage < 3.4 ) )
  {
    new_battery_percentage = 14 - ((BATT_LEVEL_3 - battery_voltage) * 8) / (BATT_LEVEL_3 - 3.3);    //5-13%
    battery_voltage = 0;//reset value
  }

  else if ( battery_voltage < 3.3 )
  {
    new_battery_percentage = 0;
  }

  return new_battery_percentage;
}



static uint8_t battery_percentage_lowest_temp( float battery_voltage )
{
  uint8_t new_battery_percentage = 0;

  if ( battery_voltage >= BATT_LEVEL_89 )
  {
    new_battery_percentage = 100;
  }

  else if ( ( battery_voltage >= BATT_LEVEL_74 ) && ( battery_voltage < BATT_LEVEL_89 ) )
  {
    new_battery_percentage = 99 - ((BATT_LEVEL_89 - (battery_voltage)) * 9) / (BATT_LEVEL_89 - BATT_LEVEL_74); // 90%-98%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= BATT_LEVEL_3 ) && ( battery_voltage < BATT_LEVEL_74 ) )
  {
    new_battery_percentage = 90 - ((BATT_LEVEL_74 - battery_voltage) * 65) / (BATT_LEVEL_74 - BATT_LEVEL_3);    //25-89%
    battery_voltage = 0;//reset value
  }

  else if ( ( battery_voltage >= 3.2 ) && ( battery_voltage < BATT_LEVEL_3 ) )
  {
    new_battery_percentage = 25 - ((BATT_LEVEL_3 - battery_voltage) * 16) / (BATT_LEVEL_3 - 3.3);    //8-24%
    battery_voltage = 0;//reset value
  }

  else if ( battery_voltage < 3.3 )
  {
    new_battery_percentage = 0;
  }

  return new_battery_percentage;
}



uint8_t mw_battery_calculate_percentage( float battery_level, float temperature_level )
{
  uint8_t reported_level = 100;

  battery_level *= m_compensation_ratio;

  if( temperature_level >= 60 )
  {
    reported_level = battery_percentage_30C_to_60C(battery_level);
  }

  if( ( temperature_level < 60 ) &&
      ( temperature_level >= 30 ) )
  {
    reported_level = battery_percentage_30C_to_60C(battery_level);
  }

  if( ( temperature_level < 30) &&
      ( temperature_level >= 10 ) )
  {
    reported_level = battery_percentage_10C_to_30C(battery_level);
  }

  if( ( temperature_level < 10 ) &&
      ( temperature_level >= 0 ) )
  {
    reported_level = battery_percentage_0C_to_10C_temp(battery_level);
  }

  if( ( temperature_level < 0 ) &&
      ( temperature_level >= -10 ) )
  {
    reported_level = battery_percentage_minus10C_to_0C_temp(battery_level);
  }

  if( ( temperature_level < -10 ) &&
      ( temperature_level > -20 ) )
  {
    reported_level = battery_percentage_minus20C_to_minus10C_temp(battery_level);
  }

  if( temperature_level <= -20 )
  {
    reported_level = battery_percentage_lowest_temp(battery_level);
  }
  return reported_level;
}


/******************************************************************************************************/
/**************************************   Battery Charger  ********************************************/
/**
 * @brief - Disable Charger IC
 */
static void disable_charger()
{
  nrf_gpio_cfg_output(m_charge_control);
  nrf_gpio_pin_set(m_charge_control);
}

/**
 * @brief - Enable Charger IC
 */
static void enable_charger()
{
  nrf_gpio_cfg_input(m_charge_control, NRF_GPIO_PIN_NOPULL);
}


/**
 * @brief - Check Charge Status
 */
mw_battery_charger_states_t mw_battery_check_charge_status()
{
  mw_battery_charger_states_t current_state = MW_BATTERY_CHARGER_IDLE;

  uint32_t pin_state = nrf_gpio_pin_read(m_usb_power);

  /* If USB Power Connected */
  if( pin_state )
  {
    enable_charger();

    vTaskDelay(500);

    pin_state = nrf_gpio_pin_read(m_charge_status);

    /* If starting charging */
    if( !pin_state && m_battery_charger_states == MW_BATTERY_CHARGER_IDLE )
    {
      //
      m_battery_charger_states = MW_BATTERY_CHARGING;
      current_state = MW_BATTERY_CHARGE_STARTING;
    }

    /* If still charging */
    else if( !pin_state && m_battery_charger_states == MW_BATTERY_CHARGING )
    {
      current_state = MW_BATTERY_CHARGING;
    }

    /* If charging complete */
    else if( pin_state && m_battery_charger_states == MW_BATTERY_CHARGING )
    {
      //
      m_battery_charger_states = MW_BATTERY_CHARGER_IDLE;
      current_state = MW_BATTERY_CHARGE_COMPLETE;
    }

    /* If still charging */
    else if( pin_state && m_battery_charger_states == MW_BATTERY_CHARGER_IDLE )
    {
      current_state = MW_BATTERY_CHARGER_IDLE;
    }

  }
  else
  {
    if ( m_battery_charger_states != MW_BATTERY_CHARGER_IDLE )
    {
      m_battery_charger_states = MW_BATTERY_CHARGER_IDLE;
      current_state = MW_BATTERY_USB_UNPLUGGED;
    }
  }

  return current_state;
}


/******************************************************************************************************/
/******************************************************************************************************/

/**
 *
 * @brief initialize battery module with voltage dividor ratio.  If none pass a value of 1
 */
void mw_battery_initialize( float compensation_resistors, uint32_t charge_control, uint32_t charge_status, uint32_t usb_power )
{
  m_compensation_ratio = 1 / compensation_resistors;
  m_charge_control = charge_control;
  m_charge_status = charge_status;
  m_usb_power = usb_power;
  disable_charger();

  nrf_gpio_cfg( m_charge_control,
          NRF_GPIO_PIN_DIR_INPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_D0S1,
          NRF_GPIO_PIN_NOSENSE);  /* Init Battery Charger */

  nrf_gpio_cfg( m_charge_status,
          NRF_GPIO_PIN_DIR_INPUT,
          NRF_GPIO_PIN_INPUT_CONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_D0S1,
          NRF_GPIO_PIN_SENSE_HIGH);  /* Init Batt Charger Status */

  nrf_gpio_cfg( m_usb_power,
          NRF_GPIO_PIN_DIR_INPUT,
          NRF_GPIO_PIN_INPUT_CONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_D0S1,
          NRF_GPIO_PIN_SENSE_HIGH);  /* Init USB Power */

}


