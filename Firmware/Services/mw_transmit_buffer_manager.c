/*
 * tx_buffer_manager.c
 *
 *  Created on: Sep 19, 2017
 *      Author: klockwood
 */

#include <mw_transmit_buffer_manager.h>

static volatile bool 		data_in_buffer = false;

static volatile bool 		full_buffer = false;

static data_buffer_t		data_buffer;

static uint16_t 			char_value_buffered;

static inline void pop_transmit_buffer();

/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
//@ble
inline void on_tx_complete_evt(proskida_service_t * p_myService, ble_evt_t * p_ble_evt)
{
	uint32_t err_code = NRF_SUCCESS;
	uint16_t length = 255;

	if(data_in_buffer)
	{
		if( char_value_buffered == p_myService->loadcell_handle.value_handle )
		{
			length = PROSKIDA_LOADCELL_CHAR_SIZE;
		}

		if( char_value_buffered == p_myService->sensor_handle.value_handle )
		{
			length = PROSKIDA_SENSOR_CHAR_SIZE;
		}

		if( char_value_buffered == p_myService->sensor_no_gyro_handle.value_handle )
		{
			length = PROSKIDA_SENSOR_NO_GYRO_CHAR_SIZE;
		}

		if( length != 255)
		{
			//Insert Service Function call to send buffered data
			err_code = send_data_update_proskida_service( p_myService, char_value_buffered, data_buffer.buffer[0].data, length );
			pop_transmit_buffer();
		}

		if( err_code == BLE_ERROR_NO_TX_PACKETS)
		{
			return;
		}
	}
}

static inline void pop_transmit_buffer()
{
	if( data_buffer.length == 1 ) //If only one element
	{
		memset( &data_buffer, 0, sizeof(data_buffer) );
		data_buffer.length = 0;
		data_in_buffer = false;
		MW_LOG("   ################## TX Buffer is EMPTY ##################\r\n", NULL, GENERAL_TEXT);
		return;
	}
	else
	{
		for( int i=0; i<data_buffer.length; i++ )
		{
			data_buffer.buffer[i] = data_buffer.buffer[i+1];
		}
	}

	data_buffer.length = data_buffer.length - 1;

	/*char temp[2];
	sprintf( temp, "%i", data_buffer.length);
	//MW_LOG("   ################## Buffer Data Sent, size now: ", NULL, BLE_EVENT);
	//MW_LOG(temp, NULL, BLE_EVENT);
	//MW_LOG(" ##################\r\n", NULL, BLE_EVENT);*/
}


inline void buffer_tx_data( uint16_t char_handle, uint8_t * data, uint16_t length )
{
	if( data_buffer.length >= BUFFER_LENGTH )
	{
		if(!full_buffer) MW_LOG("   ################## TX Buffer is 100% FULL ##################\r\n", NULL, GENERAL_TEXT);
		full_buffer = true;
		return;   //Buffer full
	}

	for(int i=0; i < length; i++)
	{
		data_buffer.buffer[ data_buffer.length ].data[i] = data[i];
	}

	char_value_buffered = char_handle;
	data_buffer.length++;
	data_in_buffer = true;

	if( data_buffer.length == BUFFER_QUARTER_LENGTH + BUFFER_HALF_LENGTH )
	{
		MW_LOG("   ################## TX Buffer is 75% FULL ##################\r\n", NULL, GENERAL_TEXT);
	}
	if( data_buffer.length == BUFFER_HALF_LENGTH )
	{
		MW_LOG("   ################## TX Buffer is 50% FULL ##################\r\n", NULL, GENERAL_TEXT);
	}
	if( data_buffer.length == BUFFER_QUARTER_LENGTH )
	{
		MW_LOG("   ################## TX Buffer is a 25% FULL ##################\r\n", NULL, GENERAL_TEXT);
	}
}

void mw_transmit_buffer_clean()
{
	memset( &data_buffer, 0, sizeof(data_buffer) );
	data_buffer.length = 0;
}


