/*
 * mw_temp_thread.c
 *
 *  Created on: May 30, 2019
 *      Author: sedmond
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrfx_gpiote.h"
#include "boards.h"

#include "FreeRTOS_includes.h"
#include <mw_logging.h>
#include "mw_temperature_thread.h"
#include "mw_thread.h"

#include "project_settings.h"

#define PIN_LO                      0
#define PIN_HI                      1

SemaphoreHandle_t                   temp_semph;

TaskHandle_t                        m_mw_temp_thread;        /**< Definition of Thread. */

static volatile mw_thread_mode_t    m_mw_temp_thread_mode = THREAD_NULL;

static mw_temp_operating_mode_t     m_temp_mode = TEMPERATURE_UNKNOWN;

static mw_temperature_external_handler_t  m_temp_external_handler = NULL;

static void pin_change_handler();

static void MW_SUSPEND_TEMPERATURE_THREAD();
static void MW_RESUME_TEMPERATURE_THREAD( bool isr);



static void temperature_pin_irq( nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action )
{
  if(pin != PIN_nCOLD) return;

  switch(action)
  {
  case NRF_GPIOTE_POLARITY_TOGGLE:
    MW_LOG_INFO("NRF_GPIOTE_POLARITY_TOGGLE");
    pin_change_handler();
    break;

  default:
    break;
  }

  MW_RESUME_TEMPERATURE_THREAD(ISR_CONTEXT);  // keep compiler happy

}



static void pin_change_handler()
{
  uint8_t pin_state = nrf_gpio_pin_read(PIN_nCOLD);

  if(pin_state == PIN_HI)
  {
    MW_LOG_INFO("Pin is high!");
  }
  else
  {
    MW_LOG_INFO("Pin is low!");
  }

  if( m_temp_external_handler != NULL )
  {
    m_temp_external_handler(m_temp_mode, ISR_CONTEXT);
  }
}


/**
 * @brief Returns ture when Thread initialized and running
 */
void mw_temperature_set_external_handler( mw_temperature_external_handler_t external_handler )
{
#if TEMPERAURE_THREAD_ENABLED
  if( external_handler != NULL )
  {
    m_temp_external_handler = external_handler;
  }
#else
  external_handler = NULL;
#endif

}



void enable_temperature_interrupt()
{
  nrfx_gpiote_in_event_enable(PIN_nCOLD, true);
}


void disable_temperature_interrupt()
{
  nrfx_gpiote_in_event_disable(PIN_nCOLD);
}


//@resume
static void MW_RESUME_TEMPERATURE_THREAD( bool isr)
{
  if ( isr )
  {
    if ( m_mw_temp_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_temp_thread_mode = THREAD_ACTIVE;
      xTaskResumeFromISR(m_mw_temp_thread); // Resume myself
    }
  }
  else
  {
    if ( m_mw_temp_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_temp_thread_mode = THREAD_ACTIVE;
      vTaskResume(m_mw_temp_thread); // Resume myself
    }
  }
}



//@suspend
static void MW_SUSPEND_TEMPERATURE_THREAD()
{
    m_mw_temp_thread_mode = THREAD_SUSPENDED;
    vTaskSuspend(m_mw_temp_thread); // Suspend myself
}


/**
 * @brief - Setup pin interrupt via GPIOTE.  Module assumed to be initialized in main.c
 */
static void temperature_init()
{
  ret_code_t err_code;

  /* Verify GPIOTE was already initialized */
  if(!nrfx_gpiote_is_init())  nrfx_gpiote_init();


  nrf_gpio_cfg_input(PIN_nCOLD, NRF_GPIO_PIN_NOPULL);

  nrfx_gpiote_in_config_t cold_pin_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

  err_code = nrfx_gpiote_in_init( PIN_nCOLD,
                                  &cold_pin_cfg,
                                  temperature_pin_irq );
  APP_ERROR_CHECK(err_code);
}


static void mw_temperature_task( void * arg )
{
  vTaskDelay(MW_TEMPERATURE_THREAD_START_UP_DELAY);

  //initialize state variables
  m_mw_temp_thread_mode = THREAD_INITIALIZED;
  m_temp_mode = TEMPERATURE_OK;

  enable_temperature_interrupt();
  MW_LOG_INFO("MW Temperature Thread Ready");

  while(1)
  {
    MW_SUSPEND_TEMPERATURE_THREAD();
  }
}


/**
 * @brief Returns ture when Thread initialized and running
 */
bool  mw_temperature_thread_ready( void )
{
#if TEMPERAURE_THREAD_ENABLED
  return (m_mw_temp_thread_mode != THREAD_NULL);
#else
  return true;
#endif
}

/**
 * @brief Function for application main entry.
 */
void mw_temperature_thread_init( void )
{
  temperature_init();

  temp_semph = xSemaphoreCreateBinary();
  if ( NULL == temp_semph )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  xSemaphoreGive(temp_semph);

  if ( xTaskCreate(mw_temperature_task, "TEMP", TEMPERATURE_THREAD_STACK_SIZE, NULL, TEMPERATURE_THREAD_PRIORITY, &m_mw_temp_thread) != pdPASS )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

}
