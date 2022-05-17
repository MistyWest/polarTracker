/*
 * freertos_debug_support.c
 *
 *  Created on: Oct 6, 2017
 *      Author: klockwood
 */

//***************************
//Nordic Includes
#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_pwr_mgmt.h"

//***************************
//FreeRTOS Includes
#include "FreeRTOS/FreeRTOS_includes.h"
#include "FreeRTOS/FreeRTOSConfig.h"

//A function used by FreeRTOS to handle Stack Overflow events
//Very useful! Requires configCHECK_FOR_STACK_OVERFLOW to be set to 1 or 2
//See: http://www.freertos.org/Stacks-and-stack-overflow-checking.html
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName )
{
	char temp[configMAX_TASK_NAME_LEN];
	for( int i=0; i<configMAX_TASK_NAME_LEN; i++ )
	{
		temp[i] = pcTaskName[i];
 	}

	temp[configMAX_TASK_NAME_LEN - 1] = temp[0];

	UBaseType_t current_task_stack_level = uxTaskGetStackHighWaterMark(NULL);
	while(1);

	current_task_stack_level++; // Keep compiler happy
}

void vApplicationMallocFailedHook( void )
{
	while(1);
}


void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );

    r0 = r0 + r1 + r2 + r3 + r12 + lr + pc + psr;
}

/*
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}*/

void MemoryManagement_Handler(void)
{
	while(1);
}
void BusFault_Handler(void)
{
	while(1);
}
void UsageFault_Handler(void)
{
	while(1);
}

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
	/*volatile unsigned long ul = 0;

	taskENTER_CRITICAL();
	{
		while( ul == 0 )
		{
			//portNOP();
			__asm__("nop");*/
	    NRF_BREAKPOINT_COND;
	/*	}
	}
	taskEXIT_CRITICAL();*/
}

#ifdef DEBUG_FIRMWARE
#if (configGENERATE_RUN_TIME_STATS == 1)
#define RUNTIME_TIMER_FREQUENCY			NRF_TIMER_FREQ_1MHz
const nrf_drv_timer_t 					FREERTOS_RUNTIME_STAT_TIMER = NRF_DRV_TIMER_INSTANCE(3);
uint32_t 								FREERTOS_RUNTIME_VALUE = 0;
nrf_ppi_channel_t 						ppi_channel_3;
#endif

/**
 * @brief Handler for timer events.
 */
void freertos_runtime_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
       case NRF_TIMER_EVENT_COMPARE1:
            FREERTOS_RUNTIME_VALUE++;
            break;

        default:
            //Do nothing.
            break;
    }
}
#endif

void vConfigureTimerForRunTimeStats( void )
{
#ifdef DEBUG_FIRMWARE
	static int i = 0;
	i++;
	if( i == 1) return;  //Used to ignore first call from freeRTOS system.  2nd call comes from BLE thread when Timer perihperals are ready

	uint32_t err_code = NRF_SUCCESS;

	nrf_drv_timer_config_t freertos_runtime_timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	freertos_runtime_timer_cfg.mode = NRF_TIMER_MODE_TIMER;
	freertos_runtime_timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
	freertos_runtime_timer_cfg.frequency = (nrf_timer_frequency_t)RUNTIME_TIMER_FREQUENCY;
	err_code = nrf_drv_timer_init( &FREERTOS_RUNTIME_STAT_TIMER, &freertos_runtime_timer_cfg, freertos_runtime_timer_event_handler );
	if (NRF_SUCCESS == err_code)
	{
		//APP_ERROR_CHECK(err_code);
	}

	err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_3);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_assign(ppi_channel_3,
	                                          nrf_drv_timer_event_address_get(&FREERTOS_RUNTIME_STAT_TIMER, NRF_TIMER_EVENT_COMPARE3),
	                                          nrf_drv_timer_task_address_get(&FREERTOS_RUNTIME_STAT_TIMER, NRF_TIMER_TASK_CLEAR));
	APP_ERROR_CHECK(err_code);

	// Enable both configured PPI channels
	err_code = nrf_drv_ppi_channel_enable(ppi_channel_3);
	APP_ERROR_CHECK(err_code);

	uint32_t timeout_ticks = nrf_drv_timer_us_to_ticks(&FREERTOS_RUNTIME_STAT_TIMER, 100);
	nrf_drv_timer_extended_compare( &FREERTOS_RUNTIME_STAT_TIMER, NRF_TIMER_CC_CHANNEL3, timeout_ticks, NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK, true);

	nrf_drv_timer_clear(&FREERTOS_RUNTIME_STAT_TIMER);
	nrf_drv_timer_enable(&FREERTOS_RUNTIME_STAT_TIMER);
#endif
}


typedef struct
{
	char task_name[8];
}task_name_t;
#ifdef DEBUG_FIRMWARE
static task_name_t task_names_array[6];
static uint16_t usage_max[6];
static uint8_t overflow_counter = 0;
void get_run_time_stats()
{

	static TaskStatus_t *pxTaskStatusArray;
	static volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime; //, ulStatsAsPercentage;
	static uint32_t run_time_counter_array[6];

	//MW_LOG("\r\n****************************** Getting Run Time Stats ************************************\r\n", NULL, SENSOR_EVENT);
	/* Make sure the write buffer does not contain a string. */
	//*pcWriteBuffer = 0x00;

	/* Take a snapshot of the number of tasks in case it changes while this
	function is executing. */
	uxArraySize = uxTaskGetNumberOfTasks();

	/* Allocate a TaskStatus_t structure for each task.  An array could be
		allocated statically at compile time. */
	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL )
	{
		/* Generate raw status information about each task. */
	    uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
	    									uxArraySize,
											&ulTotalRunTime );

		for( x = 0; x < uxArraySize; x++ )
		{
			run_time_counter_array[x]	= pxTaskStatusArray[x].ulRunTimeCounter;
		   	usage_max[x] 				= pxTaskStatusArray[x].usStackHighWaterMark;
			sprintf( task_names_array[x].task_name, "%s", pxTaskStatusArray[x].pcTaskName );
		}

		int index = 0;
		while( index < uxArraySize)
		{
			if( task_names_array[index].task_name[0] == 'S' )
			{
				break;
			}
			index++;
		}

		if( usage_max[index] > SENSOR_THREAD_STACK_SIZE - 100 )
		{
			overflow_counter++;
		}
		if( usage_max[index] > SENSOR_THREAD_STACK_SIZE - 50 )
		{
			overflow_counter++;
		}


		//MW_LOG(task_names_array[0].task_name, NULL, GENERAL_TEXT);
		//MW_LOG("\r\n", NULL, GENERAL_TEXT);

		run_time_counter_array[0] = run_time_counter_array[0] * 1;
		usage_max[0] = usage_max[0] * 1;
	}
	//while(1){}
}
#else
void get_run_time_stats(){ return; }
#endif
