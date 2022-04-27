
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS_includes.h"

/* FreeRTOS+CLI includes. */
#include "project_settings.h"

#include "mw_cli.h"
/*
 * Defines a command that starts/stops events being recorded for offline viewing
 * in FreeRTOS+Trace.
 */
static BaseType_t scan_cmd_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t disconnect_cmd_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Structure that defines the "scan" command line command. */
static const CLI_Command_Definition_t xScan =
{
	"scan", /* The command string to type. */
	"\r\nscan:\r\n Scan for nearby devices\r\n\r\n",
	scan_cmd_handler, /* The function to run. */
	0 /* No parameters are expected. */
};


/* Structure that defines the "connect" command line command. */
static const CLI_Command_Definition_t xDisconnect =
{
  "disconnect", /* The command string to type. */
  "\r\ndisconnect:\r\n Disconnect from current device\r\n\r\n",
  disconnect_cmd_handler, /* The function to run. */
  0 /* No parameter is expected. */
};



/*-----------------------------------------------------------*/

void vRegisterCLICommands( void )
{
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand( &xScan );
  FreeRTOS_CLIRegisterCommand( &xDisconnect );
}


static BaseType_t scan_cmd_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

#if BLE_CENTRAL_THREAD_ENABLED
	sprintf( pcWriteBuffer, "Starting Scan for nearby devices \r\n\r\n");
  scan_start();
#else
  sprintf( pcWriteBuffer, "BLE Central Mode not enabled in code \r\n\r\n");
#endif

	return pdFALSE;
}


static BaseType_t disconnect_cmd_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

#if BLE_CENTRAL_THREAD_ENABLED
  sprintf( pcWriteBuffer, "Disconnecting from current device \r\n\r\n");
  mw_ble_central_disconnect_from_peer();
#endif

#if BLE_PERIPHERAL_THREAD_ENABLED
  sprintf( pcWriteBuffer, "Disconnecting from current device \r\n\r\n");
  mw_ble_peripheral_disconnect();
#endif


  return pdFALSE;
}

/*-----------------------------------------------------------*/



