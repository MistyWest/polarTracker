#ifndef FREERTOS_CLI_TASK_H
#define FREERTOS_CLI_TASK_H

#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "SEGGER_RTT.h"
#include "nrf_drv_uart.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"

#include "mw_cli.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
  TERMINAL_MODE,
  CONNECTED_MODE
}cli_mode_t;

/**
 * @brief - Update CLI mode
 */
void cli_change_mode( cli_mode_t mode );

void CLI_SEND_STRING( char str[] );
void CLI_SEND_STRING_RAW(uint8_t * p_data, uint16_t data_len);
void CLI_RESTORE_TERMINAL();
void CLI_send_str_with_reset( char * str, uint8_t reset_timeout );
void CLI_initialize();

/**
 * @brief - returns True with Thread is initialized
 */
bool mw_cli_thread_ready();


/**
 * @brief - Thread Initialization
 */
void mw_cli_thread_init(void);


#endif /* COMMAND_INTERPRETER_H */
