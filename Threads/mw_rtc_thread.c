/*
 * mw_rtc_thread.c
 *
 *  Created on: May 30, 2019
 *      Author: sedmond
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrfx_gpiote.h"
#include "boards.h"
#include "nrf_delay.h"

#include "FreeRTOS_includes.h"
#include <mw_logging.h>
#include "mw_thread.h"

#include "project_settings.h"

#include "mw_data_manager_thread.h"
#include "mw_rtc_thread.h"
#include "mw_data_manager_thread.h"

#include "AM0805AQ.h"

#ifndef RTC_TWI_INSTANCE
#define RTC_TWI_INSTANCE            1
#endif

#define RTC_LOG_TAG                 "RTC Thread: "
#define RTC_LOGGING_ENABLED         0

#define PIN_RTC_SCL                 TWI0_SCL_PIN
#define PIN_RTC_SDA                 TWI0_SDA_PIN

SemaphoreHandle_t                   rtc_semph;

TaskHandle_t                        m_mw_rtc_thread;        /**< Definition of Thread. */

static mw_rtc_mode_t                m_rtc;

static volatile mw_thread_mode_t    m_mw_rtc_thread_mode;

static mw_rtc_alarm_external_handler_t  m_rtc_external_handler = NULL;


static time_reg_struct_t            m_time_update;

static uint32_t                     m_time_from_now_alarm = 0;

//**********************************************************
/* Thread Control Flags */

//**********************************************************
static bool m_update_time = false;
static bool m_update_alarm = false;
static bool m_calc_set_alarm = false;
//**********************************************************

static void MW_SUSPEND_RTC_THREAD();
static void MW_RESUME_RTC_THREAD( bool isr );


/**************************************************************************************/
/**************************************************************************************/
/* Handler Functions */

static void am0805aq_irq(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if(m_rtc.alarm_mode == RTC_ALARM_NULL) return;  /*Ignore if Alarm not set */

//  uint8_t reason = am0805_get_status();
//  reason++;
  if( pin == PIN_RTC_nTIRQ )
  {
#if RTC_LOGGING_ENABLED
    MW_LOG_INFO(RTC_LOG_TAG "PIN_RTC_nTIRQ hit");
#endif
    m_rtc.alarm_mode = RTC_ALARM_ALARMING;
  }

  if( pin == PIN_RTC_nIRQ2 )
  {

    MW_LOG_INFO(RTC_LOG_TAG "PIN_RTC_nIRQ2 hit - ALARM CONDITION");

    m_rtc.alarm_mode = RTC_ALARM_ALARMING;
    if(m_rtc_external_handler != NULL)
    {
      m_rtc_external_handler(m_rtc, ISR_CONTEXT);
    }
  }
}

static void am0805aq_event_handler( mw_twim_evt_t const * twim_event )
{
  //Do stuff
}



/**************************************************************************************/
/**************************************************************************************/
/* Accessor Functions to Thread */
/**
 * @brief - Accessor function to update RTC Alarm
 */
void mw_rtc_set_external_handler( mw_rtc_alarm_external_handler_t external_handler )
{
#if RTC_THREAD_ENABLED
  if( external_handler != NULL )
  {
    m_rtc_external_handler = external_handler;
  }
#else
  external_handler = NULL;
#endif
}



/**
 * @brief - Accessor function to get RTC Time
 *
 * MUST BE CALLED FROM main/normal Context NOT from ISR
 */
void mw_rtc_get_time( time_reg_struct_t * time_update )
{
#if !RTC_THREAD_ENABLED
  return;
#endif

  am0805_get_time(time_update);
}


/**
 * @brief - Accessor function to set RTC Time
 *
 */
uint8_t mw_rtc_set_time_raw( const uint8_t * data, uint8_t length, bool context )
{
#if !RTC_THREAD_ENABLED
  return MW_SUCCESS;
#endif

  if(length != 8) return MW_INVALID_PARAM;

  m_time_update.year    = data[0];
  m_time_update.month   = data[1];
  m_time_update.weekday = data[2];
  m_time_update.date    = data[3];
  m_time_update.hour    = data[4];
  m_time_update.minute  = data[5];
  m_time_update.second  = data[6];
  m_time_update.hundredth  = data[7];

  m_update_time = true;
  MW_RESUME_RTC_THREAD(context);

  return MW_SUCCESS;
}


/**
 * @brief - Accessor function to set RTC Date/Time
 *
* Note: Time data is bit scaled according to AM0805 datasheet
 */
void mw_rtc_auto_set_alarm( uint8_t time_from_now, rtc_time_unit_t units, bool context )
{
#if !RTC_THREAD_ENABLED
  return;
#endif

  m_time_from_now_alarm = time_from_now * units;
  m_calc_set_alarm = true;
  MW_RESUME_RTC_THREAD(context);
}



/**
 * @brief - Accessor function to set RTC Date/Time
 *
* Note: Time data is bit scaled according to AM0805 datasheet
 */
void mw_rtc_set_time( time_reg_struct_t time_update, bool context )
{
#if !RTC_THREAD_ENABLED
  return;
#endif

  m_time_update = time_update;
  m_update_time = true;
  MW_RESUME_RTC_THREAD(context);
}


/**
 * @brief - Accessor function to get RTC Time
 *
 * MUST BE CALLED FROM main/normal Context NOT from ISR
 */
void mw_rtc_get_alarm( time_reg_struct_t * time_update )
{
#if !RTC_THREAD_ENABLED
  return;
#endif

  am0805_get_time(time_update);
}


/**
 * @brief - Accessor function to update RTC Alarm
 *
 * Note: Time data is bit scaled according to AM0805 datasheet
 */
void mw_rtc_set_alarm( time_reg_struct_t alarm_update, alarm_repeat_t alarm_repeats )
{
#if !RTC_THREAD_ENABLED
  return;
#endif

  am0805_config_alarm( alarm_update, alarm_repeats, PULSE_1_4_SEC, PIN_FOUT_nIRQ );
  m_rtc.alarm_mode = RTC_ALARM_SET;
}



/**************************************************************************************/
/**************************************************************************************/
static void calculate_and_set_alarm( uint32_t period )
{
  am0805_config_countdown_timer( PERIOD_SEC, period, REPEAT_PLUSE_1_64_SEC, PIN_FOUT_nIRQ );
}


static void rtc_uninit()
{



}

static void enable_rtc_interrupt()
{
  nrfx_gpiote_in_event_enable(PIN_RTC_nTIRQ, true);
  nrfx_gpiote_in_event_enable(PIN_RTC_nIRQ2, true);
}


static void disable_rtc_interrupt()
{
  nrfx_gpiote_in_event_disable(PIN_RTC_nTIRQ);
  nrfx_gpiote_in_event_disable(PIN_RTC_nIRQ2);
}



/**
 * @brief - Setup pin interrupt via GPIOTE.  Module assumed to be initialized in main.c
 */
static void rtc_configure_interrupts()
{
  uint32_t err_code;

  /* Verify GPIOTE was already initialized */
  if(!nrfx_gpiote_is_init())  nrfx_gpiote_init();

  nrf_gpio_cfg_input(PIN_RTC_nTIRQ, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(PIN_RTC_nIRQ2, NRF_GPIO_PIN_PULLUP);

  nrfx_gpiote_in_config_t int_pin_cfg = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(false); //NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_TOGGLE(false); NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(false);

  err_code = nrfx_gpiote_in_init( PIN_RTC_nTIRQ,
                                  &int_pin_cfg,
                                  am0805aq_irq );

  err_code = nrfx_gpiote_in_init( PIN_RTC_nIRQ2,
                                  &int_pin_cfg,
                                  am0805aq_irq );
  APP_ERROR_CHECK(err_code);
}



static void rtc_init()
{
  external_device_config_t device_config;

  device_config.communication                 = TWI_COMMUNICATION;
  device_config.twi_config.instance           = RTC_TWI_INSTANCE;
  device_config.twi_config.slave_addr         = AM0805AQ_I2C_ADDR;
  device_config.twi_config.hold_bus_uninit    = false;
  device_config.twi_config.frequency          = NRF_TWI_FREQ_100K;
  device_config.twi_config.scl                = PIN_RTC_SCL;
  device_config.twi_config.sda                = PIN_RTC_SDA;
  device_config.twi_config.mw_twim_handler    = am0805aq_event_handler;
  device_config.twi_config.context            = NULL;
  device_config.twi_config.interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY;

  am0805_init( device_config );

  am0805_verify_product_id();

  am0805_clock_cfg();
}


/**
 * @brief - Returns TRUE if the Thread is currently active
 */
bool mw_rtc_thread_ready()
{
#if RTC_THREAD_ENABLED
  return ( m_mw_rtc_thread_mode != THREAD_NULL );
#else
  return true;
#endif
}


//@resume
static void MW_RESUME_RTC_THREAD( bool isr )
{
  if ( isr == ISR_CONTEXT )
  {
    if ( m_mw_rtc_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_rtc_thread_mode = THREAD_ACTIVE;
      xTaskResumeFromISR(m_mw_rtc_thread); // Resume myself
    }
  }
  else
  {
    if ( m_mw_rtc_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_rtc_thread_mode = THREAD_ACTIVE;
      vTaskResume(m_mw_rtc_thread); // Resume myself
    }
  }
}




//@suspend
static void MW_SUSPEND_RTC_THREAD()
{
    m_mw_rtc_thread_mode = THREAD_SUSPENDED;
    vTaskSuspend(m_mw_rtc_thread); // Suspend myself
}



static void mw_rtc_task( void * arg )
{
  m_rtc.time_mode = RTC_TIME_INIT;

  while( ! mw_data_manager_thread_ready() )
  {
    vTaskDelay(500);
  }

  /* Enable pin interrupts */
  enable_rtc_interrupt();



  /* Enable pin interrupts */
  enable_rtc_interrupt();

  //initialize state variables
  /* CONFIGURE INITIAL TIME */
  /*
   * hundredth : 0 ~ 99
   * second : 0 ~ 59
   * minute : 0 ~ 59
   * weekday : 0 ~ 6
   * month : 1 ~ 12
   * year : 0 ~ 99
   * mode : 0 ~ 2
   */
  time_reg_struct_t time_regs;
  time_regs.month = 10;
  time_regs.date = 3;
  time_regs.weekday = 3;
  time_regs.year = 19;
  time_regs.hour = 12;
  time_regs.minute = 00;
  time_regs.second = 00;
  time_regs.hundredth = 0;
  time_regs.mode = TWENTY_FOUR_HR_FORMAT;
  am0805_set_time(time_regs);

  am0805_disable_interrupts();

  mw_rtc_set_alarm(time_regs, ONCE_PER_WEEK); //   ONCE_PER_MINUTE ONCE_PER_HOUR ONCE_PER_WEEK

//  uint8_t test_time = 15;
//  mw_rtc_auto_set_alarm( test_time, RTC_MINUTES, NORMAL_CONTEXT);
#if RTC_LOGGING_ENABLED
  MW_LOG_INFO(RTC_LOG_TAG "*****************************");
  MW_LOG_INFO(RTC_LOG_TAG " Starting Animal Tracking");
  MW_LOG_INFO(RTC_LOG_TAG "Reset Tracking Variables");
  MW_LOG_INFO(RTC_LOG_TAG "Setting alarm for %d secs from now..", test_time);
  MW_LOG_INFO(RTC_LOG_TAG "*****************************");
#endif

  m_mw_rtc_thread_mode = THREAD_INITIALIZED;


  //initialize state variables
  m_mw_rtc_thread_mode = THREAD_INITIALIZED;


  while (1)
  {
    /**************************************************/
    /* Update RTC Current Time */
    if( m_update_time )
    {
      am0805_set_time(m_time_update);
      m_rtc.time_mode = RTC_TIME_SET;
      m_update_time = false;
    }

    /**************************************************/
    /* Calculates and sets Alarm */
    if(m_calc_set_alarm)
    {
      calculate_and_set_alarm(m_time_from_now_alarm);
    }

    /**************************************************/
    /* Update Alarm */
    if( m_update_alarm )
    {
    }

    MW_SUSPEND_RTC_THREAD();
  }

  rtc_uninit();

  /* Keep Compiler Happy */
  MW_RESUME_RTC_THREAD(NORMAL_CONTEXT);
  MW_SUSPEND_RTC_THREAD();
}




/**
 * @brief Function for application main entry.
 */
void mw_rtc_thread_init( void )
{

  rtc_configure_interrupts();
  disable_rtc_interrupt();

  /* required by the AM0805AQ part to stabilize */
  nrf_delay_ms(1000);

  rtc_init();

  m_rtc.time_mode =  RTC_TIME_NULL;
  m_rtc.alarm_mode = RTC_ALARM_NULL;

  rtc_semph = xSemaphoreCreateBinary();
  if ( NULL == rtc_semph )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  xSemaphoreGive(rtc_semph);

  if ( xTaskCreate(mw_rtc_task, "RTC", RTC_THREAD_STACK_SIZE, NULL, RTC_THREAD_PRIORITY, &m_mw_rtc_thread) != pdPASS )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}
