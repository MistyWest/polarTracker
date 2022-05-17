/*
 * mw_leds_thead.c
 *
 *  Created on: Feb 11, 2019
 *      Author: klockwood
 */

#include <stdint.h>
#include <string.h>
#include <nrf_error.h>
#include "boards.h"

#include "project_settings.h"
#include "mw_thread.h"
#include "mw_pwm_peripheral.h"
#include "mw_led_settings.h"
#include "mw_led_functions.h"
#include "mw_leds_thread.h"
#include "../CLI_Logging/mw_logging.h"

#define LED_LOG_TAG								"LEDs Thread: "

#define LED_RED                   PIN_LED
#define LED_BLUE                  PIN_LED
#define LED_GREEN                 PIN_LED

SemaphoreHandle_t leds_semph;

TaskHandle_t m_mw_leds_thread; /**< Definition of Thread. */

static volatile mw_thread_mode_t       m_mw_led_thread_mode;

static mw_led_ble_event_t              m_ble_status      = DISCONNECTED_LEDS;      /* Stores BLE Connectivity State */
static mw_led_command_event_t          m_cmd_status      = COMMAND_IDLE_LEDS;      /* Stores Command State */
static mw_led_battery_charge_event_t   m_battery_charger = BATTERY_CHARGING_IDLE;  /* Stores Battery Charge State */

static volatile bool m_new_ble_event_flag          = false;
static volatile bool m_new_cmd_event_flag          = false;
static volatile bool m_new_batt_charger_event_flag = false;

static volatile bool m_blinking_finished     = true;
static volatile bool m_solid_on_finished     = true;
static volatile bool m_pwm_off_finished      = true;
static volatile bool m_pwm_on_finished       = true;

static void mw_leds_connected();
static void mw_leds_disconnected();
static void mw_leds_low_power_disconnected();

static void mw_leds_central_connected();
static void mw_leds_scanning();

static void mw_leds_battery_charging();
static void mw_leds_battery_charging_complete();

static void mw_led_stop_ble_leds();

static void MW_RESUME_LED_THREAD( bool isr );
static void MW_SUSPEND_LED_THREAD();


/**
 * @brief - mw_led_functions.c module callback handler
 */
static void led_function_handler( mw_led_event_t event )
{
  switch(event)
  {
    case LED_BLINK_FINISHED:
      m_blinking_finished = true;
      break;

    case LED_SOLID_ON_FINISHED:
      m_solid_on_finished = true;
      break;

    case LED_SOFT_ON_FINISHED:
      m_pwm_on_finished = true;
      break;

    case LED_SOFT_OFF_FINISHED:
      m_pwm_off_finished = true;
      break;

    default:
      break;
  }
}



static void stop_all_led_sequences()
{
  /* Stop all current LED sequences */
  m_battery_charger = BATTERY_CHARGING_IDLE;
  mw_led_stop_ble_leds();
}



static void restore_ble_event_leds( bool isr )
{
  mw_led_stop_flash_1();
  m_new_ble_event_flag = true;
  MW_RESUME_LED_THREAD(isr);
}


/**************************************************************************************************/
/*******************************  BLE EVENT Sequence Functions ************************************/
/**
 * @brief - Thread handler function for BLE LED events
 */
static void mw_led_new_ble_event_handler()
{
  switch(m_ble_status)
  {
  case CONNECTED_LEDS:
    mw_leds_connected();
    break;

  case CENTRAL_CONNECTED_LEDS:
    mw_leds_central_connected();
    break;

  case SCANNING_LEDS:
    mw_leds_scanning();
    break;

  case DISCONNECTED_LEDS:
    mw_leds_disconnected();
    break;

  case LOW_POWER_DISCONNECTED_LEDS:
    mw_leds_low_power_disconnected();
    break;

  default:
    break;
  }

  m_new_ble_event_flag = false;
}


/**
 * @brief - Starts BLE Connected LED Sequence
 */
static void mw_leds_connected()
{
  /* Stop Disconnected LEDs */
  mw_led_stop_flash_0();

  // 2 green blinks
  mw_led_start_blinks(LED_GREEN, CONNECTED_SEQUENCE_ON_TIME, CONNECTED_SEQUENCE_OFF_TIME, CONNECTED_SEQUENCE_BLINKS);

  m_blinking_finished = false;

  while(!m_blinking_finished)
  {
    vTaskDelay(50);  //Wait for complete flag
  }
}

/**
 * @brief - Starts BLE Central Role Connected LED Sequence
 */
static void mw_leds_central_connected()
{
  /* Stop Disconnected LEDs */
  mw_led_stop_flash_0();

  // 2 green blinks
  mw_led_start_blinks(LED_GREEN, CONNECTED_SEQUENCE_ON_TIME, CONNECTED_SEQUENCE_OFF_TIME, CONNECTED_SEQUENCE_BLINKS);

  m_blinking_finished = false;

  while(!m_blinking_finished)
  {
    vTaskDelay(50);  //Wait for complete flag
  }
  mw_turn_on_led(LED_GREEN);
}


/**
 * @brief - Starts BLE Disconnected LED Sequence
 */
static void mw_leds_disconnected()
{
  /* Stop Disconnected LEDs */
  mw_led_stop_flash_0();

  // flashing green
  mw_led_start_flash_0(LED_GREEN, ADVERTISING_SEQUENCE_ON_TIME, ADVERTISING_SEQUENCE_OFF_TIME);
}


/**
 * @brief - Starts BLE Disconnected LED Sequence
 */
static void mw_leds_low_power_disconnected()
{
  /* Stop Disconnected LEDs */
  mw_led_stop_flash_0();

  // flashing green
  mw_led_start_flash_0(LED_GREEN, ADVERTISING_SEQUENCE_ON_TIME, LP_ADVERTISING_SEQUENCE_OFF_TIME);
}



/**
 * @brief - Starts BLE Scanning LED Sequence
 */
static void mw_leds_scanning()
{
  /* Stop Disconnected LEDs */
  mw_led_stop_flash_0();

  // flashing green
  mw_led_start_flash_0(LED_GREEN, ADVERTISING_SEQUENCE_ON_TIME, ADVERTISING_SEQUENCE_OFF_TIME);
}


/**
 * @brief - Function for stopping current BLE LED sequences
 */
static void mw_led_stop_ble_leds()
{
  switch ( m_ble_status )
  {
  case CONNECTED_LEDS:
    mw_led_stop_blinks();
    break;

  case CENTRAL_CONNECTED_LEDS:
    mw_turn_off_led(LED_GREEN);
    break;

  case SCANNING_LEDS:
  case DISCONNECTED_LEDS:
    mw_led_stop_flash_0();
    break;

  case LOW_POWER_DISCONNECTED_LEDS:
    mw_led_stop_flash_0();
    break;

  default:
    break;
  }
}

/**************************************************************************************************/
/*******************************  BLE Command Sequence Functions *********************************/

/**
 * @brief - Starts a Heartbeat LED
 */
static void mw_leds_heartbeat()
{
  /* Stop Disconnected LEDs */
  mw_led_stop_flash_0();

  // flashing green
  mw_led_start_flash_0(LED_RED, HEARTBEAT_ON_DURATION, HEARTBEAT_OFF_DURATION);
}


/**
 * @brief - Starts a Heartbeat 2 LED
 */
static void mw_leds_heartbeat_2()
{
  /* Stop Disconnected LEDs */
  mw_led_stop_flash_0();

  // flashing green
  mw_led_start_flash_0(LED_RED, HEARTBEAT_2_ON_DURATION, HEARTBEAT_2_OFF_DURATION);
}




/**
 * @brief - Thread handler function for Command LED events
 */
static void mw_led_new_cmd_event_handler()
{
  if( m_ble_status != BLE_EVENT_IDLE_LEDS ) mw_led_stop_ble_leds();

  switch(m_cmd_status)
  {
/** Added Cases as needed */
  case HEARTBEAT_LED:
    mw_leds_heartbeat();
    break;

  case HEARTBEAT_2_LED:
    mw_leds_heartbeat_2();
    break;

  case BLINKS_LED:
    mw_led_start_blinks(LED_GREEN, STANDARD_BLINKS_ON_DURATION, STANDARD_BLINKS_OFF_DURATION, 2);
    break;

  default:
    mw_led_stop_flash_0();
    break;
  }

  m_new_cmd_event_flag = false;
}



/**************************************************************************************************/
/******************************  Battery Charging Sequence Functions ******************************/
/**
 * @brief - Thread handler function for Battery Charging LED events
 */
static void mw_led_new_batt_charger_event_handler()
{
  mw_led_stop_ble_leds();

  switch(m_battery_charger)
  {
  case BATTERY_CHARGING_LEDS:
    mw_leds_battery_charging();
    break;

  case BATTERY_CHARGING_COMPLETE_LEDS:
    mw_leds_battery_charging_complete();
    break;

  case BATTERY_CHARGING_USB_UNPLUG:
    m_battery_charger = BATTERY_CHARGING_IDLE;
    break;

  default:
    break;
  }

  m_battery_charger = BATTERY_CHARGING_IDLE;
  restore_ble_event_leds(NORMAL_CONTEXT);
  m_new_batt_charger_event_flag = false;
}


static void mw_leds_battery_charging()
{
  mw_turn_off_led(LED_BLUE);
  while(m_battery_charger == BATTERY_CHARGING_LEDS)
  {
    mw_led_soft_transition(LED_BLUE, SOFT_ON);
    m_pwm_on_finished = false;
    while(!m_pwm_on_finished)
    {
      vTaskDelay(50);  //Wait for complete flag
    }

    // slow PWM on/off blue led
    mw_led_solid_start(LED_BLUE, LED_ON, BATTERY_CHARGING_ON_DURATION, KEEP_ON_AT_END);
    m_solid_on_finished = false;
    while(!m_solid_on_finished)
    {
      vTaskDelay(50);  //Wait for complete flag
    }

    mw_led_soft_transition(LED_BLUE, SOFT_OFF);
    m_pwm_off_finished = false;
    while(!m_pwm_off_finished)
    {
      vTaskDelay(50);  //Wait for complete flag
    }

    // slow PWM on/off blue led
    mw_led_solid_start(LED_BLUE, LED_OFF, BATTERY_CHARGING_OFF_DURATION, KEEP_ON_AT_END);
    m_solid_on_finished = false;
    while(!m_solid_on_finished)
    {
      vTaskDelay(50);  //Wait for complete flag
    }
  }

  /* Transition to Charge Complete LEDs */
  if(m_battery_charger == BATTERY_CHARGING_COMPLETE_LEDS) mw_leds_battery_charging_complete();
}

static void mw_leds_battery_charging_complete()
{
  mw_led_soft_transition(LED_BLUE, SOFT_ON);
  m_pwm_on_finished = false;
  while ( !m_pwm_on_finished )
  {
    vTaskDelay(50);  //Wait for complete flag
  }

  // slow PWM on/off blue led
  mw_led_solid_start(LED_BLUE, LED_ON, BATTERY_CHARGING_COMPLETE, KEEP_ON_AT_END);
  m_solid_on_finished = false;
  while ( !m_solid_on_finished )
  {
    vTaskDelay(50);  //Wait for complete flag
  }

  mw_led_soft_transition(LED_BLUE, SOFT_OFF);
  m_pwm_off_finished = false;
  while ( !m_pwm_off_finished )
  {
    vTaskDelay(50);  //Wait for complete flag
  }

  // Solid blue then slow fade
  m_battery_charger = BATTERY_CHARGING_IDLE;
}

/**************************************************************************************************/
/********************************  Low Battery Sequence Functions *********************************/

/**
 * @brief - Accessor function for sending Low Battery events
 */
void mw_leds_low_battery( bool enable )
{
#if !LED_THREAD_ENABLED
  return;
#endif
  if(enable)
  {
    // flashing red led
    mw_led_start_secondary_flash(LED_RED, LOW_POWER_ON_TIME, LOW_POWER_OFF_TIME);
  }
  else
  {
    mw_led_stop_secondary_flash();
  }
}



/**************************************************************************************************/
/********************************** Thread Accessor Functions *************************************/

/**
 * @brief - Accessor function for sending Connect/Disconnect events
 */
void mw_leds_ble_event( mw_led_ble_event_t event, bool isr )
{
#if !LED_THREAD_ENABLED
  return;
#endif

  m_ble_status = event;

  //If charging - block
  if( m_battery_charger != BATTERY_CHARGING_IDLE) return;

  m_new_ble_event_flag = true;
  MW_RESUME_LED_THREAD(isr);
}


/**
 * @brief - Accessor function for sending Command events
 */
void mw_leds_command_event( mw_led_command_event_t event, bool isr )
{
#if !LED_THREAD_ENABLED
  return;
#endif

  m_cmd_status = event;
  m_new_cmd_event_flag = true;
  MW_RESUME_LED_THREAD(isr);
}


/**
 * @brief - Accessor function for sending Battery Charge events
 */
void mw_leds_battery_charge_event( mw_led_battery_charge_event_t event, bool isr )
{
#if !LED_THREAD_ENABLED
  return;
#endif

  /* If this is a new event */
  if( m_battery_charger != event )
  {
    m_battery_charger = event;
    m_new_batt_charger_event_flag = true;
    MW_RESUME_LED_THREAD(isr);
  }
}

/**************************************************************************************************/
/********************************    DFU LED Sequence   *******************************************/
/**
 * @brief - Accessor function triggering DFU Leds
 */
void mw_leds_go_to_dfu()
{
  /* Stop all current LED sequences */
  stop_all_led_sequences();

  mw_turn_off_led(LED_RED);
  mw_turn_off_led(LED_GREEN);
  mw_turn_off_led(LED_BLUE);

  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_on_led(LED_RED);
  mw_turn_on_led(LED_GREEN);
  mw_turn_on_led(LED_BLUE);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_off_led(LED_RED);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_off_led(LED_GREEN);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_off_led(LED_BLUE);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_on_led(LED_RED);
  mw_turn_on_led(LED_GREEN);
  mw_turn_on_led(LED_BLUE);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_off_led(LED_RED);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_off_led(LED_GREEN);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);
  mw_turn_off_led(LED_BLUE);
  vTaskDelay(DFU_SEQUENCE_ON_TIME);

}

/**************************************************************************************************/
/**********************************  Start Up LED Sequence ****************************************/
static void mw_leds_start_up()
{
  mw_turn_off_led(LED_RED);
  mw_turn_off_led(LED_GREEN);
  mw_turn_off_led(LED_BLUE);

  for( uint8_t i=0; i<START_UP_CYCLES; i++)
  {
    mw_turn_on_led(LED_RED);
    vTaskDelay(START_UP_FAST_TIME);

    mw_turn_off_led(LED_RED);
    vTaskDelay(START_UP_FAST_TIME);
  }

  mw_turn_on_led(LED_RED);
  vTaskDelay(START_UP_FAST_TIME*5);
  mw_turn_off_led(LED_RED);
}




/**************************************************************************************************/
/**************************************************************************************************/

/**
 * @brief - Resume Thread
 *
 * @param - If calling function was from Interrupt Context
 */
//@resume
static void MW_RESUME_LED_THREAD( bool isr )
{
#if !LED_THREAD_ENABLED
  return;
#endif

  m_mw_led_thread_mode = THREAD_ACTIVE;
  if(isr)
  {
    xTaskResumeFromISR(m_mw_leds_thread); // Resume myself
  }
  else
  {
    vTaskResume(m_mw_leds_thread); // Resume myself
  }
}

/**
 * @brief - Suspend Thread
 */
//@suspend
static void MW_SUSPEND_LED_THREAD()
{
#if !LED_THREAD_ENABLED
  return;
#endif

  m_mw_led_thread_mode = THREAD_SUSPENDED;
  vTaskSuspend(m_mw_leds_thread); // Suspend myself
}



static void mw_leds_task( void * arg )
{
  m_mw_led_thread_mode = THREAD_INITIALIZED;
	mw_led_functions_init( led_function_handler );
	m_mw_led_thread_mode = THREAD_ACTIVE;

	mw_leds_start_up();

	//mw_leds_command_event(HEARTBEAT_LED,NORMAL_CONTEXT);

	while ( true )
	{
    if(m_new_batt_charger_event_flag) mw_led_new_batt_charger_event_handler();

    if(m_new_cmd_event_flag) mw_led_new_cmd_event_handler();

    if(m_new_ble_event_flag) mw_led_new_ble_event_handler();

    MW_SUSPEND_LED_THREAD();
	}  // end while()

}


/**@brief Function for application main entry.
 */
void mw_leds_thread_init( void )
{

	leds_semph = xSemaphoreCreateBinary();
	if ( NULL == leds_semph )
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
	xSemaphoreGive(leds_semph);

	if ( xTaskCreate(mw_leds_task, "LEDS", LED_THREAD_STACK_SIZE, NULL, LED_THREAD_PRIORITY, &m_mw_leds_thread) != pdPASS )
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}

	nrf_gpio_cfg_output(LED_RED);
	nrf_gpio_cfg_output(LED_BLUE);
	nrf_gpio_cfg_output(LED_GREEN);

	mw_turn_off_led(LED_RED);
	mw_turn_off_led(LED_BLUE);
	mw_turn_off_led(LED_GREEN);
}
