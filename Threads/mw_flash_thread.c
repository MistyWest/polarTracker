/*
 * mw_flash_thread.c
 *
 *  Created on: Oct 15, 2019
 *      Author: klockwood
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nrf.h"
#include "nrf_soc.h"
#include "nordic_common.h"
#include "boards.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf_fstorage.h"

#include "project_settings.h"



#if (BLE_PERIPHERAL_THREAD_ENABLED || BLE_CENTRAL_THREAD_ENABLED || 1)
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#else
#include "nrf_drv_clock.h"
#endif

#include "nrf_fstorage_nvmc.h"

#include "FreeRTOS_includes.h"
#include <mw_logging.h>
#include "mw_thread.h"
#include "mw_flash_thread.h"

/******************************************/
/* Flash Region for storage */
#define FLASH_STORAGE_START_ADDRESS  0x71000
#define FLASH_STORAGE_END_ADDRESS    0x73000
/******************************************/

#define FLASH_LOG_TAG               "Flash Manager Thread: "

SemaphoreHandle_t                   flash_semph;

TaskHandle_t                        m_mw_flash_thread;        /**< Definition of Thread. */

static volatile mw_thread_mode_t    m_mw_flash_thread_mode = THREAD_NULL;

static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

static volatile bool wait_for_read = false;


NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = FLASH_STORAGE_START_ADDRESS,
    .end_addr   = FLASH_STORAGE_END_ADDRESS,
};


/******************************************/
/* Global Variables */
static uint16_t          m_bear_id;
/******************************************/
/* State Control Flags */
static volatile bool     m_update_bear_id  = false;
/******************************************/

static void MW_SUSPEND_FLASH_THREAD();
static void MW_RESUME_FLASH_THREAD( bool isr);


/********************************************************************************************
 *
 *  Handler Function(s) *
 *
 *******************************************************************************************/
static void fstorage_evt_handler( nrf_fstorage_evt_t * p_evt )
{
  if ( p_evt->result != NRF_SUCCESS )
  {
    MW_LOG_INFO(FLASH_LOG_TAG"--> Event received: ERROR while executing an fstorage operation.");
    return;
  }

  switch ( p_evt->id )
  {
  case NRF_FSTORAGE_EVT_WRITE_RESULT:
    MW_LOG_INFO(FLASH_LOG_TAG"--> Event received: wrote %d bytes at address 0x%x.", p_evt->len, p_evt->addr);
    break;

  case NRF_FSTORAGE_EVT_ERASE_RESULT:
    MW_LOG_INFO(FLASH_LOG_TAG"--> Event received: erased %d page from address 0x%x.", p_evt->len, p_evt->addr);
    break;

  case NRF_FSTORAGE_EVT_READ_RESULT:
    MW_LOG_INFO(FLASH_LOG_TAG"--> Event received: Read %d page from address 0x%x.", p_evt->len, p_evt->addr);
    wait_for_read = false;
    break;

  default:
    break;
  }
}


/********************************************************************************************
 *
 *  External Access Functions *
 *
 *******************************************************************************************/
/**
 * @brief - Save Bear ID
 */
void mw_flash_thread_save_bear_id_and_profile( uint16_t bear_id, bool isr )
{
#if !FLASH_THREAD_ENABLED
  return;
#endif
  m_bear_id = bear_id;

  m_update_bear_id = true;

  MW_RESUME_FLASH_THREAD(isr);
}


/**
 * @brief - Load Bear ID
 */
void mw_flash_thread_load_bear_id_and_profile( uint16_t * buffer )
{
#if !FLASH_THREAD_ENABLED
  return;
#endif
  uint32_t return_code;

  uint32_t read_value;

  MW_LOG_INFO(FLASH_LOG_TAG"Read from flash.....");
  MW_LOG_INFO(FLASH_LOG_TAG"Read from flash.....");
  return_code = nrf_fstorage_read(&fstorage, FLASH_STORAGE_START_ADDRESS, &read_value, sizeof(read_value));
  APP_ERROR_CHECK(return_code);

  vTaskDelay(50);
  wait_for_flash_ready(&fstorage);

  MW_LOG_INFO(FLASH_LOG_TAG"Done.");
  MW_LOG_INFO(FLASH_LOG_TAG"Read \"0x%lx\" from flash.", read_value);

  m_bear_id = (uint16_t)read_value;
  *buffer = (uint16_t)read_value;
}



/********************************************************************************************
 *
 *  Internal Function *
 *
 *******************************************************************************************/

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
//static uint32_t nrf5_flash_end_addr_get()
//{
//    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
//    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
//    uint32_t const code_sz         = NRF_FICR->CODESIZE;
//
//    return (bootloader_addr != 0xFFFFFFFF ?
//            bootloader_addr : (code_sz * page_sz));
//}


//static void wait_for_flash_read()
//{
//  /* While fstorage is busy, sleep and wait for an event. */
//  while ( wait_for_read )
//  {
//    vTaskDelay(10);
//  }
//}
//@wait
static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
  /* While fstorage is busy, sleep and wait for an event. */
  while ( nrf_fstorage_is_busy(p_fstorage) )
  {
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
    vTaskDelay(100);
    MW_LOG_INFO(FLASH_LOG_TAG"waiting for flash operation to complete");
  }
}


static void write_bear_id_to_flash()
{
  uint32_t return_code;

  bool check_value = ((FLASH_STORAGE_START_ADDRESS & (fstorage.p_flash_info->erase_unit - 1)) == 0);
  MW_LOG_INFO(FLASH_LOG_TAG"Check: \"%b\" .", check_value);

  check_value = ((FLASH_STORAGE_START_ADDRESS >= fstorage.start_addr) && (FLASH_STORAGE_START_ADDRESS + 4096 - 1 <= fstorage.end_addr));

  return_code = nrf_fstorage_erase(&fstorage, FLASH_STORAGE_START_ADDRESS, 1, NULL);
  APP_ERROR_CHECK(return_code);

  wait_for_flash_ready(&fstorage);

  check_value = !(4096 % fstorage.p_flash_info->program_unit);

  MW_LOG_INFO(FLASH_LOG_TAG"Writing \"%s\" to flash.", m_bear_id);
  return_code = nrf_fstorage_write(&fstorage, FLASH_STORAGE_START_ADDRESS, &m_bear_id, sizeof(m_bear_id), NULL);
  APP_ERROR_CHECK(return_code);

  wait_for_flash_ready(&fstorage);
  MW_LOG_INFO(FLASH_LOG_TAG"Done.")
}


static void print_flash_info( nrf_fstorage_t * p_fstorage )
{
  MW_LOG_INFO(FLASH_LOG_TAG"========| flash info |========");
  MW_LOG_INFO(FLASH_LOG_TAG"erase unit: \t%d bytes", p_fstorage->p_flash_info->erase_unit);
  MW_LOG_INFO(FLASH_LOG_TAG"program unit: \t%d bytes", p_fstorage->p_flash_info->program_unit);
  MW_LOG_INFO(FLASH_LOG_TAG"==============================");
}


//@init
static void mw_flash_driver_init(void)
{
  uint32_t return_code;

  nrf_fstorage_api_t * p_fs_api;

#ifdef SOFTDEVICE_PRESENT
  MW_LOG_INFO(FLASH_LOG_TAG"SoftDevice is present.");
  MW_LOG_INFO(FLASH_LOG_TAG"Initializing nrf_fstorage_sd implementation...");
  /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
   * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
   * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
  p_fs_api = &nrf_fstorage_sd;
#else
  MW_LOG_INFO(FLASH_LOG_TAG"SoftDevice not present.");
  MW_LOG_INFO(FLASH_LOG_TAG"Initializing nrf_fstorage_nvmc implementation...");
  /* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
   * nrf_fstorage_nvmc uses the NVMC peripheral. This implementation can be used when the
   * SoftDevice is disabled or not present.
   *
   * Using this implementation when the SoftDevice is enabled results in a hardfault. */
  p_fs_api = &nrf_fstorage_nvmc;
#endif

  return_code = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
  APP_ERROR_CHECK(return_code);

  print_flash_info(&fstorage);
}



//@resume
static void MW_RESUME_FLASH_THREAD( bool isr )
{
  if ( isr )
  {
    if ( m_mw_flash_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_flash_thread_mode = THREAD_ACTIVE;
      xTaskResumeFromISR(m_mw_flash_thread); // Resume myself
    }
  }
  else
  {
    if ( m_mw_flash_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_flash_thread_mode = THREAD_ACTIVE;
      vTaskResume(m_mw_flash_thread); // Resume myself
    }
  }
}

//@suspend
static void MW_SUSPEND_FLASH_THREAD()
{
  m_mw_flash_thread_mode = THREAD_SUSPENDED;
  vTaskSuspend(m_mw_flash_thread); // Suspend myself
}



/**
 * @brief - Thread
 */
static void mw_flash_task( void * arg )
{
  mw_flash_driver_init();

  mw_flash_thread_load_bear_id_and_profile(&m_bear_id);

  /*****************************/
  /* Test Flash */
//  m_bear_id = 0xFEED;
//  write_bear_id_to_flash();
//  m_bear_id = mw_flash_thread_load_bear_id();
  /*****************************/

  m_mw_flash_thread_mode = THREAD_INITIALIZED;

  while ( 1 )
  {

    if(m_update_bear_id)
    {
      write_bear_id_to_flash();

      m_update_bear_id = false;
    }


    MW_SUSPEND_FLASH_THREAD();
  }

  MW_RESUME_FLASH_THREAD( NORMAL_CONTEXT);
  MW_SUSPEND_FLASH_THREAD();
}


/**
 * @brief Thread is ready
*/
bool mw_flash_thread_ready(void)
{
#if !FLASH_THREAD_ENABLED
  return true;
#endif
  return m_mw_flash_thread_mode != THREAD_NULL;
}


/**
 * @brief Function for application main entry.
 */
void mw_flash_thread_init( void )
{
#if !FLASH_THREAD_ENABLED
  return;
#endif

  flash_semph = xSemaphoreCreateBinary();
  if ( NULL == flash_semph )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  xSemaphoreGive(flash_semph);

  if ( xTaskCreate(mw_flash_task, "DATA", FLASH_THREAD_STACK_SIZE, NULL, FLASH_THREAD_PRIORITY, &m_mw_flash_thread) != pdPASS )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

}


