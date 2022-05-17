/*
 * unit_testing.c
 *
 *  Created on: Nov 28, 2018
 *      Author: KLockwood
 */

/**@file
 * @defgroup mw_test Throughput Test
 * @ingroup  mw_test
 * @brief    Throughput test
 */

//Nordic Radio Includes
#include <FreeRTOS/FreeRTOS_includes.h>
#include "ble_gap.h"
#include "ble_gatts.h"

//Logging Includes
#include "ble_nus.h"

//FreeRTOS Includes
#include "throughput_test.h"

#include "../CLI_Logging/mw_logging.h"
#include "throughput_test_settings.h"
//******************************************************

static TimerHandle_t m_test_data_timer;              /**< Definition of data timer. */

static uint32_t timeout_counts = 0;
static uint32_t tx_send_error = 0;
static uint32_t data_sent = 0;


static uint16_t * m_conn_handle;
static void * 		m_svc_obj;

static uint16_t m_mtu_value = 23;
static uint16_t m_data_length_value = 27;
static uint8_t 	m_phy_value;
static uint16_t m_conn_interval = DATA_SEND_INTERVAL + DATA_SEND_INTERVAL_BUFFER;

static uint16_t m_data_interval = DATA_SEND_INTERVAL + DATA_SEND_INTERVAL_BUFFER;
//******************************************************

static void increment_data_array( uint8_t * array, uint32_t length );


//@start
void start_data_throughput_test()
{
	timeout_counts = 0;
	tx_send_error = 0;
	data_sent = 0;
	if (pdPASS != xTimerStart(m_test_data_timer, OSTIMER_WAIT_FOR_QUEUE))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
}


//@stop
void stop_data_throughput_test()
{
	if (pdPASS != xTimerStop(m_test_data_timer, OSTIMER_WAIT_FOR_QUEUE))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
}


static void increment_data_array( uint8_t * array, uint32_t length )
{
	bool propagate = false;

	for(int i=0; i<length; i++)
	{
		if( array[i] < 99 )
		{
			if(!propagate) array[i]++;
			return;
		}
		else
		{
			array[i] = 0;
			array[i+1] += 1;
			propagate = true;
		}
	}
}


/**@brief Function for handling the Data timer time-out.
 *
 * @details This function will be called each time the Data timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
//@data
static void data_timeout_handler(TimerHandle_t xTimer)
{
	uint32_t err_code = NRF_SUCCESS;
	static uint8_t data_array[NRF_SDH_BLE_GATT_MAX_MTU_SIZE];
	uint16_t length = m_mtu_value - 3;

	timeout_counts++;

	increment_data_array(data_array, length);
	UNUSED_PARAMETER(xTimer);

  for( int i=0; i<1; i++)
  {
		err_code = ble_nus_data_send(m_svc_obj, data_array, &length, *m_conn_handle );
		if( ( err_code != NRF_ERROR_NOT_FOUND ) &&
				( err_code != NRF_SUCCESS ) &&
				( err_code != NRF_ERROR_RESOURCES ) &&
				( err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING ) &&
				( err_code != NRF_ERROR_INVALID_STATE ))
		{
			APP_ERROR_HANDLER(err_code);
		}

		if( ( err_code == NRF_ERROR_RESOURCES ) ||
				( err_code == NRF_ERROR_NO_MEM ) )
		{
			MW_LOG_INFO("BLE Data Rate Error Count:%d ,  This Error was:%d \r\n", tx_send_error++, err_code);
		}

		if( err_code == 0 )
		{
			data_sent++;
		}
  }

  //************** Calculate throughput over the past 100 samples ***************************
	float live_data_rate;
	if(data_sent >= 99)
	{
		live_data_rate = 1000*(length*100) / (m_data_interval*timeout_counts);
		MW_LOG_INFO( "BLE Data Rate: " NRF_LOG_FLOAT_MARKER "Bytes/sec\r\n", NRF_LOG_FLOAT(live_data_rate));
		data_sent = 0;
		timeout_counts = 0;
	}
	//****************************************************************************************
}


static void unit_test_update_timer_period( bool irq_context, uint16_t interval_update )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	bool timer_restart = xTimerIsTimerActive( m_test_data_timer );	//Is the current time active?

	if(irq_context)
	{
		if( xTimerChangePeriodFromISR( m_test_data_timer, m_data_interval, &xHigherPriorityTaskWoken ) != pdPASS )
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}

		if( xHigherPriorityTaskWoken != pdFALSE )
		{
		}
	}
	else
	{
		if ( pdPASS != xTimerChangePeriod( m_test_data_timer, m_data_interval, OSTIMER_WAIT_FOR_QUEUE ))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
	}

	if(timer_restart)
	{
		if (pdPASS != xTimerStart(m_test_data_timer, OSTIMER_WAIT_FOR_QUEUE))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
	}
	else
	{
		if (pdPASS != xTimerStop(m_test_data_timer, OSTIMER_WAIT_FOR_QUEUE))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
	}

	MW_LOG_INFO( "BLE Throughput Test Interval Changed to: %d",  m_data_interval);
}

void throughput_test_check_throughput_test( uint8_t value )
{
	uint32_t err_code;

	if( value == START_THROUGHPUT_TEST )
	{
		MW_LOG_INFO("Starting BLE Throughput Test");
		start_data_throughput_test();
	}

	if( value == STOP_THROUGHPUT_TEST )
	{
		MW_LOG_INFO("Stopped BLE Throughput Test");
		stop_data_throughput_test();
	}

	if( value == ENABLE_2MBPS_TEST )
	{
		MW_LOG_INFO("2Mbps PHY update request.");

		ble_gap_phys_t const phys =
		{
				.rx_phys = BLE_GAP_PHY_2MBPS,
				.tx_phys = BLE_GAP_PHY_2MBPS,
		};
		err_code = sd_ble_gap_phy_update(*m_conn_handle, &phys);
		APP_ERROR_CHECK(err_code);

		MW_LOG_INFO("Request Error Code: %d", err_code);
	}
}

void throughput_test_update_ble_phy_setting( uint8_t phy )
{
	m_phy_value						= phy;
	throughput_test_update_ble_connection_params(false, m_conn_interval);
}


void throughput_test_update_ble_throughput_settings( uint16_t mtu_size_update, uint16_t gap_length_update )
{
	m_mtu_value						= mtu_size_update;
	m_data_length_value		= gap_length_update;
	throughput_test_update_ble_connection_params( false, m_conn_interval );

	MW_LOG_INFO("Data Thru-put Test: Updated MTU Setting: %d", m_mtu_value);
	MW_LOG_INFO("Data Thru-put Test: Updated Data Length Setting: %d", m_data_length_value);
}


void throughput_test_update_ble_connection_params( bool irq_context, uint16_t interval_update )
{
	m_conn_interval = interval_update;

	if(m_phy_value == BLE_GAP_PHY_2MBPS)
	{
		m_data_interval = interval_update / DATA_SEND_INTERVAL_DIVISOR_2MBPS + DATA_SEND_INTERVAL_BUFFER;
	}
	if( (m_phy_value == BLE_GAP_PHY_1MBPS) || (m_phy_value == BLE_GAP_PHY_AUTO ) )
	{
		if( 30  > m_data_length_value )
			m_data_interval = interval_update + DATA_SEND_INTERVAL_BUFFER_LOW_DL;
		else if( m_data_length_value > 80 )
		  m_data_interval = interval_update + DATA_SEND_INTERVAL_BUFFER * 10;
		else
			m_data_interval = interval_update / DATA_SEND_INTERVAL_DIVISOR_1MBPS + DATA_SEND_INTERVAL_BUFFER * 3;
	}


#if DATA_SEND_INTERVAL_NON_ADAPTIVE
  m_data_interval = DATA_SEND_INTERVAL;  //Use hardcoded value
#endif

	unit_test_update_timer_period( irq_context, m_data_interval );

	MW_LOG_INFO("Data Thru-put Test: Updated Connection Param: %d", m_conn_interval);
	MW_LOG_INFO("Data Thru-put Test: Updated Data Interval: %d", m_data_interval);
}



void throughput_test_init( uint16_t * conn_handle, ble_nus_t * p_nus )
{
	m_conn_handle = conn_handle;
	m_svc_obj			= p_nus;
  m_test_data_timer = xTimerCreate("DATA",
  																 m_data_interval,
																	 pdTRUE,
                               	 	 NULL,
																	 data_timeout_handler);

  /* Error checking */
  if ( (NULL == m_test_data_timer) )
  {
  	APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}
