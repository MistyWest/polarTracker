/*
 * main.c
 *
 *  Created on: Jan 2, 2019
 *      Author: KLockwood
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "app_error.h"
#include "ble.h"
#include "sdk_config.h"
#include "nrf_drv_clock.h"

//Project Includes
#include "project_settings.h"
#include <FreeRTOS/FreeRTOS_includes.h>
#include "mw_power_management.h"
#include "mw_logging.h"
#include "mw_cli_thread.h"

#include <mw_ble_peripheral_thread.h>
#include "mw_ble_central_thread.h"

#include "mw_sensor_thread.h"
#include "mw_adc_thread.h"
#include "mw_leds_thread.h"
#include "mw_temperature_thread.h"
#include "mw_rtc_thread.h"
#include "mw_data_manager_thread.h"
#include "mw_flash_thread.h"
#include "artic_thread.h"
#include "wwf_service.h"
#include "mw_watchdog.h"

#if BLE_PERIPHERAL_THREAD_ENABLED && BLE_CENTRAL_THREAD_ENABLED
#warning Multi-mode Central and Peripheral roles not currently supported
#endif

#define DEAD_BEEF                   0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/****************************************************************************************************/
/****************************************************************************************************/

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/****************************************************************************************************/
/****************************************************************************************************/

/**@brief Function for initializing the clock.
 *
 * Using legacy driver, nrfx driver seems to cause issues
 */
static void clock_init(void)
{
	ret_code_t err_code = nrf_drv_clock_init(/*nrf_clock_handler*/);
	APP_ERROR_CHECK(err_code);
}



/****************************************************************************************************/
/****************************************************************************************************/

/**
 * @brief Function for initializing the board
 */
static void board_init(void)
{
  /* Initial Power Management */
  mw_power_management_init();
  mw_set_master_3V3_rail(ON);
  mw_set_1V8_rail(OFF);
  mw_set_artic_3V3_rail(OFF);
  mw_set_5V_rail(OFF);

  nrf_gpio_cfg_output(SPIM2_MISO_PIN);
  nrf_gpio_cfg_output(SPIM2_MOSI_PIN);
  nrf_gpio_cfg_output(SPIM2_SCK_PIN);
  nrf_gpio_cfg_output(SPIM2_SS_PIN);

  nrf_gpio_pin_clear(SPIM2_MISO_PIN);
  nrf_gpio_pin_clear(SPIM2_MOSI_PIN);
  nrf_gpio_pin_clear(SPIM2_SCK_PIN);
  nrf_gpio_pin_clear(SPIM2_SS_PIN);
}


/**
 * @brief - Initialize Watchdog
 */
static void wdt_init()
{
  mw_watchdog_init();
}


/****************************************************************************************************/
// This module is used to assure FPU events are handled so the system can return to proper
// idle sleep current as per Nordic recommendation:
// https://devzone.nordicsemi.com/question/70989/fpu-divide-by-0-and-high-current-consumption/
#define FPU_EXCEPTION_MASK 0x0000009F

//@FPU
void SETUP_FPU_HANDLER()
{
  NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOW);
  NVIC_EnableIRQ(FPU_IRQn);
}

void FPU_IRQHandler(void)
{
    uint32_t *fpscr = (uint32_t *)(FPU->FPCAR+0x40);
    (void)__get_FPSCR();

    *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}
/****************************************************************************************************/
/****************************************************************************************************/


/**@brief Function for application main entry.
 */
int main(void)
{
  /* Initialize clock module */
 	clock_init();


 	/* Setup FPU Handler */
 	SETUP_FPU_HANDLER();


  /* Initialize board */
	board_init();


  /* Initialize watchdog module */
  wdt_init();


#if TEMPERAURE_THREAD_ENABLED || SENSOR_THREAD_ENABLED || RTC_THREAD_ENABLED || ARTIC_THREAD_ENABLED
	nrfx_gpiote_init();
#endif

#if LOGGING_THREAD_ENABLED
	mw_logging_init();
#endif


#if CLI_THREAD_ENABLED
  mw_cli_thread_init();
#endif


#if RTC_THREAD_ENABLED
  mw_rtc_thread_init();
#endif


#if BLE_PERIPHERAL_THREAD_ENABLED
  mw_ble_peripheral_thread_init();
#elif !BLE_PERIPHERAL_THREAD_ENABLED && FLASH_THREAD_ENABLED
  mw_ble_peripheral_stack_init();
#endif

#if BLE_CENTRAL_THREAD_ENABLED
  mw_ble_central_thread_init();
#endif


#if ADC_THREAD_ENABLED
  mw_adc_thread_init();
#endif


#if SENSOR_THREAD_ENABLED
	mw_sensor_thread_init();
#endif


#if LED_THREAD_ENABLED
	mw_leds_thread_init();
#endif


#if TEMPERAURE_THREAD_ENABLED
	mw_temperature_thread_init();
#endif


#if ARTIC_THREAD_ENABLED
  artic_thread_init();
#endif



#if FLASH_THREAD_ENABLED
  mw_flash_thread_init();
#endif


#if DATA_MANAGER_THREAD_ENABLED
  mw_data_manager_thread_init();
#endif


	// Activate deep sleep mode
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	// Start FreeRTOS scheduler.
	vTaskStartScheduler();

	for (;;)
	{
		APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
	}
}

/****************************************************************************************************/
/****************************************************************************************************/
