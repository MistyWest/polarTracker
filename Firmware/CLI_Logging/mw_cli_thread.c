#include "mw_cli_thread.h"

#include <project_settings.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS_includes.h"

/* FreeRTOS+CLI includes. */
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"

#include "mw_thread.h"
#include "mw_logging.h"
#include "mw_ble_central_thread.h"
#include "mw_cli_settings.h"
#include "mw_cli.h"

#if (!APP_UART_ENABLED)||(!APP_FIFO_ENABLED)
#warning Must enable App Uart and App Fifo in sdk_config.h to use CLI module
#endif


#if defined(BOARD_PCA10059)
#define RX_PIN_NUMBER  15
#define TX_PIN_NUMBER  17
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           false
#endif

#define TAKE_CNTRL() xSemaphoreTake( semph_CLI, 500)
#define RELEASE_CNTRL() xSemaphoreGive(semph_CLI)

SemaphoreHandle_t semph_CLI;
TaskHandle_t      m_mw_cli_thread; /**< Definition of Thread. */
static volatile mw_thread_mode_t  m_mw_cli_thread_mode = THREAD_NULL;


app_uart_buffers_t buffers;
static char        cli_logging_buffer[CLI_BUFFER_SIZE];
static uint8_t     cli_logging_buffer_index = 0;

//static uint8_t     tx_buf[TX_BUF_SIZE];

static cli_mode_t m_cli_mode = TERMINAL_MODE;

static bool m_cli_new_character_received  = false;
static bool m_cli_new_data_to_send        = false;

static uint8_t initialized = 0;
void uart_error_handle(app_uart_evt_t * p_event);

static TickType_t last_contact = 0;
static volatile uint8_t sleep = 2; // if 0 the CLI is in sleep mode, if 1 CLI will transition into wake mode, if 2 CLI is in wake mode


//static void CLI_set_pin_interrupt(uint8_t * p_err); // used to convert RX pin to interrupt to wake the CLI
//static void CLI_wake_haldler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void CLI_reinitalize(void);
void CLI_uart_disable(void);
void CLI_PUSH_STRING(char * str);
static void uart_initialize(void);

//static uint8_t stay_flag = 0;


static void cli_welcome_message()
{
  //** Welcome Message
  CLI_SEND_STRING( MW_LOG_BORDER "\r\n");
  CLI_SEND_STRING( MW_LOG_BORDER "\r\n");
  CLI_SEND_STRING( "Command Line Interface Online.. \r\n");
  CLI_SEND_STRING( MW_LOG_BORDER "\r\n");
}


/**
 * @brief - Update CLI mode
 */
void cli_change_mode( cli_mode_t mode )
{
  m_cli_mode = mode;
}




/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handler( app_uart_evt_t * p_event )
{
  switch ( p_event->evt_type )
  {
  /**@snippet [Handling data from UART] */
  case APP_UART_DATA_READY:
    //NRF_LOG_ERROR("Received Data");
    m_cli_new_character_received = true;
    break;

    /**@snippet [Handling data from UART] */
  case APP_UART_COMMUNICATION_ERROR:
    NRF_LOG_ERROR("Communication error occurred while handling UART.");
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;

  case APP_UART_FIFO_ERROR:
    NRF_LOG_ERROR("Error occurred in FIFO module used by UART.")
    ;
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;

  default:
    break;
  }
}

static void uart_initialize()
{
	uint32_t err_code;

  app_uart_comm_params_t const comm_params = {
      .rx_pin_no = RX_PIN_NUMBER,
      .tx_pin_no = TX_PIN_NUMBER,
      .rts_pin_no = RTS_PIN_NUMBER,
      .cts_pin_no = CTS_PIN_NUMBER,
      .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
      .use_parity = false,
      .baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200 };

  APP_UART_FIFO_INIT(&comm_params,
      RX_BUF_SIZE,
      TX_BUF_SIZE,
      uart_event_handler,
      APP_IRQ_PRIORITY_LOWEST,
      err_code);

  APP_ERROR_CHECK(err_code);

	initialized = 1;
}

void CLI_uart_disable(void)
{
  app_uart_close();

  NRF_UARTE0->EVENTS_ENDRX = 0;

  NRF_UARTE0->EVENTS_ENDTX = 0;

  NRF_UARTE0->EVENTS_ERROR = 0;

  NRF_UARTE0->EVENTS_RXTO = 0;

}

static void CLI_reinitalize(void)
{
	uint32_t err_code;

  app_uart_comm_params_t const comm_params = {
      .rx_pin_no = RX_PIN_NUMBER,
      .tx_pin_no = TX_PIN_NUMBER,
      .rts_pin_no = RTS_PIN_NUMBER,
      .cts_pin_no = CTS_PIN_NUMBER,
      .flow_control =       APP_UART_FLOW_CONTROL_DISABLED,
      .use_parity = false,
      .baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200 };

  APP_UART_FIFO_INIT(&comm_params,
      RX_BUF_SIZE,
      TX_BUF_SIZE,
      uart_event_handler,
      APP_IRQ_PRIORITY_LOWEST,
      err_code);

  APP_ERROR_CHECK(err_code);

	initialized = 1;	// allows access to UART peripheral
}

void CLI_SEND_STRING( char * str )
{
  //uint16_t length = strlen(str);
	//uint16_t i = 0;

	if(!initialized)
		return;

	TAKE_CNTRL();

	if( cli_logging_buffer_index + strlen(str) < CLI_BUFFER_SIZE )
	{
    memcpy( &cli_logging_buffer[cli_logging_buffer_index], str, strlen(str) );
    cli_logging_buffer_index += strlen(str);
	}

	m_cli_new_data_to_send = true;

//	while(str[i] != 0)
//	{
//		app_uart_put(str[i]);
//		i++;
//	}

//	if( str[length-1] == '\n' )
//	{
//	  CLI_send_str_with_reset("\r\n" CLI_TAG, 1);
//	}
	RELEASE_CNTRL();
}


void CLI_SEND_STRING_RAW(uint8_t * p_data, uint16_t data_len)
{
  uint32_t ret_val;

  if(!initialized)
    return;

  TAKE_CNTRL();

  for (uint32_t i = 0; i < data_len; i++)
  {
      do
      {
          ret_val = app_uart_put(p_data[i]);
          if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
          {
              APP_ERROR_CHECK(ret_val);
          }
      } while (ret_val == NRF_ERROR_BUSY);
  }
//  if (p_data[data_len-1] == '\r')
//  {
//      while (app_uart_put('\n') == NRF_ERROR_BUSY);
//  }

  RELEASE_CNTRL();
}


void CLI_RESTORE_TERMINAL()
{
  CLI_send_str_with_reset("\r\n" CLI_TAG, 1);
}


void CLI_send_str_with_reset(char * str, uint8_t reset_timeout)
{
  uint16_t i = 0;

  if(!initialized)
    return;

  TAKE_CNTRL();
  if(reset_timeout)
    last_contact = xTaskGetTickCount(); // reset timer for CLI

  while(str[i] != 0)
  {
    app_uart_put(str[i]);
    i++;
  }

  RELEASE_CNTRL();
}

void CLI_PUSH_STRING(char * str)
{
  uint16_t i = 0;

  if(!initialized)
    return;

  TAKE_CNTRL();
  while(str[i] != 0)
  {
    app_uart_put(str[i]);
    i++;
  }

  RELEASE_CNTRL();
}

//static void CLI_set_pin_interrupt(uint8_t * p_err)
//{
//	*p_err = 0;
//
//	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
//	in_config.pull = NRF_GPIO_PIN_NOPULL;
//
//	nrf_drv_gpiote_in_init( RX_PIN_NUMBER, &in_config, CLI_wake_haldler);
//
//
//	nrf_drv_gpiote_in_event_enable( RX_PIN_NUMBER, true);
//
//}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

//static void CLI_wake_haldler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
//{
//	if(pin == RX_PIN_NUMBER) {
//		sleep = 1;
//		nrf_drv_gpiote_in_event_disable(RX_PIN_NUMBER);
//	}
//
//}

void m_mw_cli_task( void *pvParameters )
{
  uint16_t lBytes, lByte;
  signed char cInChar, cInputIndex = 0;
  static char cInputString[ cmdMAX_INPUT_SIZE ];
  static char cOutputString[ cmdMAX_OUTPUT_SIZE ];
  static char cLocalBuffer[ cmdSOCKET_INPUT_BUFFER_SIZE ];
  BaseType_t xMoreDataToFollow;

  lBytes = 0;

  uint32_t err_code;
  uint8_t byte = 0;
//  uint8_t err = 0;


  uart_initialize();

  cli_welcome_message();

  CLI_send_str_with_reset("\r\n" CLI_TAG, 1);
  lBytes = 0;

  /* Register commands with the FreeRTOS+CLI command interpreter. */
  vRegisterCLICommands();

  m_mw_cli_thread_mode = THREAD_INITIALIZED;

  for(;;)
  {

//    if( (xTaskGetTickCount() - last_contact > TIME_OUT) &&
//        sleep != 0 && stay_flag == 0)
//    { // timeout and not already sleeping
//      last_contact = xTaskGetTickCount();
//      sleep = 0;
//      CLI_PUSH_STRING("CLI: Going into sleep mode\r\n");
//      vTaskDelay(250);
//      //nrf_uart_disable(NRF_UART0);
//      CLI_uart_disable();
//      CLI_set_pin_interrupt(&err);
//      initialized = 0;
//    }

    if(sleep == 2)
    {

      if ( m_cli_new_data_to_send )
      {
        app_uart_put('\r');
        for( int i=0; cli_logging_buffer[i] != 0; i++)
        {
          app_uart_put( cli_logging_buffer[i] );
        }

        memset((uint8_t*)cli_logging_buffer, '\0', CLI_BUFFER_SIZE);
        cli_logging_buffer_index = 0;
        m_cli_new_data_to_send = false;
        CLI_send_str_with_reset("\r\n" CLI_TAG, 1);
      }

      if ( m_cli_new_character_received )
      {
        err_code = app_uart_get(&byte);

        if ( err_code == NRF_SUCCESS )
        {
          app_uart_put(byte); // echo out
          cLocalBuffer[lBytes] = byte;
          lBytes++;
          last_contact = xTaskGetTickCount(); // reset timeout
          m_cli_new_character_received = false;

          /* If Esc character received performance action */
          if ( byte == '\e' )
          {
            //TODO
          }


        }

        if ( cLocalBuffer[lBytes - 1] == '\r' )
        {
          /* Process each received byte in turn. */
          lByte = 0;
          while ( lByte < lBytes )
          {
            /* The next character in the input buffer. */
            cInChar = cLocalBuffer[lByte];
            lByte++;

            /* Newline characters are taken as the end of the command
             string. */
            if ( cInChar == '\r' )
            {
              /* Process the input string received prior to the
               newline. */
              do
              {
                if ( m_cli_mode == TERMINAL_MODE )
                {
                  /* Pass the string to FreeRTOS CLI. */
                  xMoreDataToFollow = FreeRTOS_CLIProcessCommand(cInputString, cOutputString, cmdMAX_OUTPUT_SIZE);

                  /* Send the output generated by the command's
                   implementation. */
                  CLI_send_str_with_reset(cOutputString, 1);
                }

                vTaskDelay(5);
              }
              while ( xMoreDataToFollow != pdFALSE ); /* Until the command does not generate any more output. */

              /* All the strings generated by the command processing
               have been sent.  Clear the input string ready to receive
               the next command. */
              cInputIndex = 0;
              memset(cInputString, 0x00, cmdMAX_INPUT_SIZE);

              /* Transmit a spacer, just to make the command console
               easier to read. */
              CLI_send_str_with_reset("\r\n" CLI_TAG, 1);
              lBytes = 0;
            }
            else
            {
              if ( cInChar == '\r' )
              {
                /* Ignore the character.  Newlines are used to
                 detect the end of the input string. */
              }
              else if ( cInChar == '\b' )
              {
                /* Backspace was pressed.  Erase the last character
                 in the string - if any. */
                if ( cInputIndex > 0 )
                {
                  cInputIndex--;
                  cInputString[cInputIndex] = '\0';
                }
              }
              else
              {
                /* A character was entered.  Add it to the string
                 entered so far.  When a \n is entered the complete
                 string will be passed to the command interpreter. */
                if ( cInputIndex < cmdMAX_INPUT_SIZE )
                {
                  cInputString[cInputIndex] = cInChar;
                  cInputIndex++;
                }
              }
            }
          }
          memset(cLocalBuffer, 0x00, lBytes); // clear out local buffer
        }

        vTaskDelay(CLI_THREAD_LOOP_DELAY);
      }
    }
    else if(sleep == 1) { // wake up CLI and reset sleep timer
      sleep = 2; // wakey wakey
      last_contact = xTaskGetTickCount();
      CLI_reinitalize();
      CLI_send_str_with_reset("CLI: Awake\r\n", 1);
    }
    else if(sleep == 0) { // in sleep mode
      vTaskDelay(500);
    }
  }

  CLI_send_str_with_reset("CLI Task ended\r\n", 1);
  vTaskDelete(NULL);
}


/**
 * @brief - returns True with Thread is initialized
 */
bool mw_cli_thread_ready()
{
#if !CLI_THREAD_ENABLED
  return true;
#endif
  return m_mw_cli_thread_mode != THREAD_NULL;
}


/**
 * @brief - Initialize Thread
 */
void mw_cli_thread_init(void)
{
   semph_CLI = xSemaphoreCreateBinary();
  if ( NULL == semph_CLI )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  xSemaphoreGive(semph_CLI);

  if ( xTaskCreate(m_mw_cli_task, "freertos_CLI", CLI_THREAD_STACK_SIZE, NULL, CLI_THREAD_PRIORITY, &m_mw_cli_thread) != pdPASS )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}
