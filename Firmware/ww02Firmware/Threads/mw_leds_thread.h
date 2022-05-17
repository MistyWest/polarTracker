/*
 * mw_leds_thread.h
 *
 *  Created on: Feb 11, 2019
 *      Author: klockwood
 */

#ifndef THREADS_MW_LEDS_THREAD_H_
#define THREADS_MW_LEDS_THREAD_H_

typedef enum
{
  BLE_EVENT_IDLE_LEDS = 0x55,
  DISCONNECTED_LEDS,
  SCANNING_LEDS,
  LOW_POWER_DISCONNECTED_LEDS,
  CONNECTED_LEDS,
  CENTRAL_CONNECTED_LEDS
}mw_led_ble_event_t;


typedef enum
{
  COMMAND_IDLE_LEDS = 0x55,
  HEARTBEAT_LED,
  HEARTBEAT_2_LED,
  BLINKS_LED
/* Added more as needed */
}mw_led_command_event_t;


typedef enum
{
  BATTERY_CHARGING_IDLE = 0x55,
  BATTERY_CHARGING_LEDS,
  BATTERY_CHARGING_COMPLETE_LEDS,
  BATTERY_CHARGING_USB_UNPLUG
}mw_led_battery_charge_event_t;

/**
 * @brief - Accessor function triggering DFU Leds
 */
void mw_leds_go_to_dfu();

/**
 * @brief - Accessor function for sending Connect/Disconnect events
 */
void mw_leds_ble_event( mw_led_ble_event_t event, bool isr );


/**
 * @brief - Accessor function for sending Command events
 */
void mw_leds_command_event( mw_led_command_event_t event, bool isr );


/**
 * @brief - Accessor function for sending Battery Charge events
 */
void mw_leds_battery_charge_event( mw_led_battery_charge_event_t event, bool isr );


/**
 * @brief - Accessor function for sending Low Battery events
 */
void mw_leds_low_battery( bool enable );


/**@brief Function for application main entry.
 */
void mw_leds_thread_init( void );


#endif /* THREADS_MW_LEDS_THREAD_H_ */
