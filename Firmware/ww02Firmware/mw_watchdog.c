/*
 * mw_watchdog.c
 *
 *  Created on: Oct 13, 2020
 *      Author: klockwood
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "app_error.h"
#include "nrfx_wdt.h"

#include "project_settings.h"
#include <FreeRTOS/FreeRTOS_includes.h>
#include "mw_logging.h"
#include "mw_watchdog.h"

#ifndef WATCHDOG_TIMEOUT
#define WATCHDOG_TIMEOUT              20000
#endif

#ifndef WATCHDOG_FEED_INTERVAL
#define WATCHDOG_FEED_INTERVAL        8000
#endif


static TimerHandle_t                m_wdt_timer;
nrfx_wdt_channel_id                 m_channel_id;


/****************************************************************************************************/
/****************************************************************************************************/
/**
 * @brief WDT Feed timer
 */
/*
 * @brief - IMU inactivity Timer Timeout Handler
 */
static void wdt_timer_timeout_handler(TimerHandle_t xTimer)
{
  //MW_LOG_DEBUG("Reloading WatchDog");
  nrfx_wdt_channel_feed(m_channel_id);
}


/**
 * @brief WDT events handler.
 */
void wdt_event_handler( void )
{
  /* Reset Device */
  sd_nvic_SystemReset();

  //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

/****************************************************************************************************/
/****************************************************************************************************/

void mw_watchdog_enable()
{
  nrfx_wdt_enable();
  xTimerStart(m_wdt_timer, 500);
}

/****************************************************************************************************/
/****************************************************************************************************/

/**
 * @brief - Initializes WatchDog - Autostarts
 */
void mw_watchdog_init()
{

//Configure WDT.
  nrfx_wdt_config_t config;
  config.behaviour = (nrf_wdt_behaviour_t) NRFX_WDT_CONFIG_BEHAVIOUR;
  config.reload_value = WATCHDOG_TIMEOUT; /* 20 seconds timeout */
  config.interrupt_priority = NRFX_WDT_CONFIG_IRQ_PRIORITY;

  uint32_t err_code = nrfx_wdt_init(&config, wdt_event_handler);
  APP_ERROR_CHECK(err_code);
  err_code = nrfx_wdt_channel_alloc(&m_channel_id);
  APP_ERROR_CHECK(err_code);

  m_wdt_timer = xTimerCreate("WDT", WATCHDOG_FEED_INTERVAL, pdTRUE, NULL, wdt_timer_timeout_handler);
  if ( (NULL == m_wdt_timer) )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  mw_watchdog_enable();
}
