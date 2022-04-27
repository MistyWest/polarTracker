/*
 * project_settings.h
 *
 *  Created on: Nov 28, 2018
 *      Author: KLockwood
 */

#ifndef PROJECT_SETTINGS_H_
#define PROJECT_SETTINGS_H_

#include "FreeRTOS_includes.h"
#include "sdk_config.h"
#include "project_board.h"

//#ifdef NRF_LOG_ENABLED
//#undef NRF_LOG_ENABLED
//#define NRF_LOG_ENABLED 0
//#endif


/*Replace if custom board file if one is created */
#if BOARD_MW_PROJECT_CUSTOM
#include "ww02_miniboard_r0_1.h"
#endif
#if BOARD_PCA10040_COMPATIBLE
#include "pca10040_compatible_board.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

//***************************************************
//  FREERTOS Thread defines
//***************************************************
/*
 * Enable/Disable Project Threads
 */
#define BLE_PERIPHERAL_THREAD_ENABLED              1
#define BLE_CENTRAL_THREAD_ENABLED                 0

#define CLI_THREAD_ENABLED                         0
#define LOGGING_THREAD_ENABLED                     0
#define SENSOR_THREAD_ENABLED                      1
#define ADC_THREAD_ENABLED                         1
#define LED_THREAD_ENABLED                         1
#define TEMPERAURE_THREAD_ENABLED                  1
#define RTC_THREAD_ENABLED                         1
#define DATA_MANAGER_THREAD_ENABLED                1
#define ARTIC_THREAD_ENABLED                       1
#define FLASH_THREAD_ENABLED                       0


/*
 * Thread Priorities - note BLE Thread is defined in FreeRTOSConfig.h
 */
#define TEMPERATURE_THREAD_PRIORITY                1
#define LOGGING_THREAD_PRIORITY                    1
#define LED_THREAD_PRIORITY                        2
#define CLI_THREAD_PRIORITY                        3
#define DATA_MANAGER_THREAD_PRIORITY               3
#define SENSOR_THREAD_PRIORITY                     4
#define RTC_THREAD_PRIORITY                        4
#define ADC_THREAD_PRIORITY                        5
#define ARTIC_THREAD_PRIORITY                      6
#define FLASH_THREAD_PRIORITY                      2


/*
 * Thread Stack Allocations - note BLE Stack size is defined in FreeRTOSConfig.h
 */
#define CLI_THREAD_STACK_SIZE                      512
#define LOGGING_THREAD_STACK_SIZE                  512
#define SENSOR_THREAD_STACK_SIZE                   512
#define ADC_THREAD_STACK_SIZE                      512
#define RTC_THREAD_STACK_SIZE                      512
#define DATA_MANAGER_THREAD_STACK_SIZE             512
#define LED_THREAD_STACK_SIZE                      256
#define TEMPERATURE_THREAD_STACK_SIZE              256
#define ARTIC_THREAD_STACK_SIZE                    512
#define FLASH_THREAD_STACK_SIZE                    512

/*
 * Thread Start-up delays
 */
#define MW_SENSOR_THREAD_START_UP_DELAY            500
#define MW_ADC_THREAD_START_UP_DELAY               200
#define MW_BLE_THREAD_START_UP_DELAY               500
#define MW_TEMPERATURE_THREAD_START_UP_DELAY       2000
#define MW_RTC_THREAD_START_UP_DELAY               1000

/*
 * Thread polling interval
 */
#define SENSOR_THREAD_POLL_INTERVAL					       2000
#define ADC_THREAD_POLL_INTERVAL						       60000
#define TEMPERATURE_THREAD_POLL_INTERVAL           8000


//***************************************************
//             ** CLI Includes **
// Include header files here which will use/connect-to the CLI
//***************************************************
#include "mw_ble_peripheral_thread.h"
#include "mw_ble_central_thread.h"


//***************************************************
// Peripheral Instances
//***************************************************
#define RTC_TWI_INSTANCE                            0   /**< TWIM0 */
#define SENSOR_TWI_INSTANCE                         1   /**< TWIM1 */
#define ARTIC_SPI_INSTANCE                          2   /**< SPI2 */

//***************************************************
//  Application Timer Intervals
//***************************************************
/*
 * Project Timer Intervals *
 */
#define BATTERY_MEASUREMENT_INTERVAL               2000         /**< Battery level measurement interval (ms). */

#define MW_SAADC_SAMPLE_INTERVAL                   500000       // 500msec interval

#define TEMPERATURE_TIME_INTERVAL                  50000       //300000


//***************************************************
//  Service defines
//***************************************************
#define UART_SERVICE                               1


//***************************************************
//  Unit Test defines
//***************************************************
/*
 * Defining which tests are enabled in the system
 */
#define THROUGHPUT_TEST                             0


#define BYPASS_ARGOS_TRANSMISSIONS                  0

#define PA_TEST_MODE_FOR_JOHN                       0
#define TEST_MODE_FOR_JOHN                          0


#endif /* PROJECT_SETTINGS_H_ */
