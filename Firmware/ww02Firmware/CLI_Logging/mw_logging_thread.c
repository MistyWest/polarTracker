/*
 * mw_logging.c
 *
 *  Created on: Jul 24, 2019
 *      Author: klockwood
 */

#include "FreeRTOS_includes.h"
#include "mw_logging.h"
#include "project_settings.h"
#include "mw_ble_settings.h"


static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */


static void mw_log_welcome_message()
{
  //** Welcome Message
  MW_LOG_MISTYWEST_WELCOME
  NRF_LOG_PROCESS();
  MW_LOG_INFO(MW_LOG_BORDER);
  NRF_LOG_PROCESS();
  MW_LOG_INFO("%s starting", DEVICE_NAME);
  NRF_LOG_PROCESS();
  MW_LOG_INFO(MW_LOG_BORDER);
  NRF_LOG_PROCESS();
}


/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED && LOGGING_THREAD_ENABLED
  vTaskResume(m_logger_thread);
#endif
}


/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
  UNUSED_PARAMETER(arg);

  mw_log_welcome_message();

  while (1)
  {
    NRF_LOG_PROCESS();

#if configUSE_IDLE_HOOK
    vTaskSuspend(NULL); // Suspend myself
#else
    vTaskDelay(20);
#endif
  }
}


/**@brief Function for initializing the nrf log module.
 */
void mw_logging_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

#if NRF_LOG_ENABLED
  NRF_LOG_DEFAULT_BACKENDS_INIT();
#else
  return;
#endif


  // Start execution.
  if (pdPASS != xTaskCreate(logger_thread, "LOGGER", LOGGING_THREAD_STACK_SIZE, NULL, LOGGING_THREAD_PRIORITY, &m_logger_thread))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}



