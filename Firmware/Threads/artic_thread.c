/*
 * mw_artic.c
 *
 *  Code for configuring the ARTIC (aka ARGOS) chip.
 *  Used for creating Argos-2 compatible messages.
 *
 *  Created on: January 7, 2019
 *      Author: Sean Edmond
 */

#include "artic_thread.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrfx_gpiote.h"
#include "boards.h"
#include "nrfx_timer.h"
#include "nrfx_clock.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "FreeRTOS_includes.h"

#include "math.h"

#include "../Artic/ARTIC_PMEM.h"
#include "../Artic/ARTIC_XMEM.h"
#include "../Artic/ARTIC_YMEM.h"
#include "mw_thread.h"
#include "mw_data_manager_thread.h"

#include "project_settings.h"
#include "project_board.h"
#include "mw_logging.h"
#include "mw_power_management.h"
#include "_mw_external_device.h"
#include "Artic.h"
#include "RFPA0133.h"

#define ARTIC_LOG_TAG                   "Artic Thread: "

#ifndef ARTIC_SPI_INSTANCE
#define ARTIC_SPI_INSTANCE              2
#endif

#define ARTIC_SPI_IRQ_PRIORITY          5
#define ARTIC_SPI_MODE                  NRF_SPI_MODE_1  //sample on the falling edge

#define PIN_ARTIC_SPI_MISO              SPIM2_MISO_PIN
#define PIN_ARTIC_SPI_MOSI              SPIM2_MOSI_PIN
#define PIN_ARTIC_SPI_SCK               SPIM2_SCK_PIN
#define PIN_ARTIC_SPI_NSS               SPIM2_SS_PIN


#define DEFAULT_TXCO_WARMUP_TIME        13

SemaphoreHandle_t                       artic_semph;

TaskHandle_t                            m_mw_artic_thread;        /**< Definition of Thread. */

static volatile mw_thread_mode_t        m_artic_thread_mode = THREAD_NULL;


static m_artic_device_t                 m_artic_device;

static volatile artic_int1_t            m_artic_int1 = {0};
static volatile artic_int2_t            m_artic_int2 = {0};
static volatile artic_fw_state_t        m_artic_fw_state = {0};

static uint8_t                          m_argos_user_id[4];


bool read_in_progress;
bool write_in_progress;

bool service_int1_irq;
bool service_int2_irq;



//**********************************************************
/* Thread Control Flags */

//**********************************************************
static volatile bool m_artic_int1_wait = false;
static volatile bool m_artic_int2_wait = false;

static bool          m_artic_in_use = false;
//**********************************************************
/* Static Function Declarations */

//**********************************************************
static void artic_power_on(void);
static void artic_power_off(void);
static void enable_artic_interrupt();
static void disable_artic_interrupt();
static uint8_t artic_get_tcxo_warmup_time(void);
static void artic_program(void);
static void artic_setup_interface(void);

static void artic_interrupt_1_wait_and_clear(void);
static void artic_interrupt_2_wait_and_clear(void);

static void MW_RESUME_ARTIC_THREAD( bool isr );
static void MW_SUSPEND_ARTIC_THREAD();


//**********************************************************
/* Semaphore */

//**********************************************************
static inline void TAKE_CNTRL()
{
  if( xSemaphoreTake( artic_semph, 50) == pdFALSE)
  {
    MW_LOG_INFO(ARTIC_LOG_TAG "***************************Semaphore Timeout");
  }
}


static inline void RELEASE_CNTRL()
{
  xSemaphoreGive(artic_semph);
}




/**************************************************************************************/
/* Handler Functions */
/**************************************************************************************/
/**
 * @brief - Handler for INT1 and INT2 from ARTIC device
 */
static void artic_pin_irq(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if( pin == PIN_ARTIC_INT1 )
  {
    m_artic_int1_wait = false;
    MW_LOG_INFO(ARTIC_LOG_TAG "PIN_ARTIC_INT1 hit");
    MW_RESUME_ARTIC_THREAD(ISR_CONTEXT);
  }

  if( pin == PIN_ARTIC_INT2 )
  {
    m_artic_int2_wait = false;
    MW_LOG_INFO(ARTIC_LOG_TAG "PIN_ARTIC_INT2 hit");
    MW_RESUME_ARTIC_THREAD(ISR_CONTEXT);
  }
}


/**
 * @brief - Handler for SPI Driver Callback
 */
void artic_spi_callback( mw_spi_evt_t const * spi_event )
{
  MW_LOG_INFO(ARTIC_LOG_TAG "ARTIC SPI Callback hit");
}


/**************************************************************************************/
/* External Functions */
/**************************************************************************************/
//@init
uint32_t artic_power_on_and_program(void)
{
#if !ARTIC_THREAD_ENABLED
  return 0;
#endif

  m_artic_in_use = true;

  MW_LOG_INFO(ARTIC_LOG_TAG "*** Setup ARTIC interface");
  artic_setup_interface();

  MW_LOG_INFO(ARTIC_LOG_TAG "*** Enable ARTIC interrupts");
  enable_artic_interrupt();

  MW_LOG_INFO(ARTIC_LOG_TAG "*** Power Up ARTIC");
  artic_power_on();

  MW_LOG_INFO(ARTIC_LOG_TAG "*** Waiting for INT2 and clearing...");
  artic_interrupt_2_wait_and_clear();

  MW_LOG_INFO(ARTIC_LOG_TAG "*** Programming ARTIC");
  artic_program();

  uint32_t fw_version = artic_read_firmware_version();

  if ( fw_version == 0 ) return 99;

  m_artic_device.tcxo_warm_up_time = artic_get_tcxo_warmup_time();

  return 0;
}


void artic_power_down(void)
{
#if !ARTIC_THREAD_ENABLED
  return;
#endif
  disable_artic_interrupt();
  rfpa0133_low_power_mode();
  artic_power_off();

  nrf_gpio_pin_clear(SPIM2_MISO_PIN);
  nrf_gpio_pin_clear(SPIM2_MOSI_PIN);
  nrf_gpio_pin_clear(SPIM2_SCK_PIN);
  nrf_gpio_pin_clear(SPIM2_SS_PIN);

  m_artic_in_use = false;

  vTaskDelay(255);
}


/* Enable External PA */
void artic_power_on_pa(void)
{
  //mw_set_5V_rail(ON_BURST_MODE);
  mw_set_5V_rail(ON_FIXED_PWM);
}

/* Disable External PA */
void artic_power_off_pa(void)
{
  mw_set_5V_rail(OFF);
}


bool artic_is_use()
{
  return m_artic_in_use;
}


/**
 * @brief - Update ARGOS 28bit User ID field
 */
uint8_t update_argos_user_id ( const uint8_t * data, uint8_t length )
{
  if( length != 4 ) return ARTIC_ERROR_INVALID_PARAM;

  xSemaphoreTake(artic_semph, portMAX_DELAY);
  memcpy(m_argos_user_id, data, 4);
  xSemaphoreGive(artic_semph);
  return ARTIC_SUCCESS;
}


bool test_artic_power_up_program_power_down()
{
  artic_power_on_and_program();
  artic_power_down();
  return true;
}


/**************************************************************************************/
/* Internal Functions */
/**************************************************************************************/
/**
 * @brief - Handle Interrupt 1 Received from ARTIC chpi
 */
//@int1
static bool mcu_cmd_flag = false;  //TODO remove
static void artic_interrupt_1_wait_and_clear(void)
{
  static uint8_t wait_count = 0;
  uint8_t fw_status[3];

  /* Wait for Interrupt */
  MW_LOG_DEBUG(ARTIC_LOG_TAG "Waiting for INT1");
  while(m_artic_int1_wait)
  {
    MW_LOG_DEBUG(ARTIC_LOG_TAG "...wait for INT1 to clear");
    vTaskDelay(255);
    wait_count++;

    if(wait_count>=50)
    {
      MW_LOG_DEBUG(ARTIC_LOG_TAG "...ARTIC unresponsive");
    }
  }
  wait_count = 0;

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** artic_int1_callback ENTER ***");

  //read the status and update the context vars
  artic_burst_spi_read(ARTIC_FIRMWARE_STATUS, ARTIC_IO_MEM, fw_status, 3);

  //clear the interrupt
  artic_int1_clear();

  //current FW state...
  m_artic_fw_state.idle = fw_status[2] & 1;
  m_artic_fw_state.rx_in_progress = fw_status[2] >> 1;
  m_artic_fw_state.tx_in_progress = fw_status[2] >> 2;
  m_artic_fw_state.dsp2mcu_int1 = fw_status[0] >> 6;
  m_artic_fw_state.dsp2mcu_int2 = fw_status[0] >> 7;

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** FW STATUS ***");
  MW_LOG_DEBUG(ARTIC_LOG_TAG "idle: %d ", m_artic_fw_state.idle);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_in_progress: %d ", m_artic_fw_state.rx_in_progress);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "tx_in_progress: %d ", m_artic_fw_state.tx_in_progress);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "dsp2mcu_int1: %d ", m_artic_fw_state.dsp2mcu_int1);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "dsp2mcu_int2: %d ", m_artic_fw_state.dsp2mcu_int2);

  //SE_NOTE... not sure how reliable these are
  m_artic_int1.rx_valid_message = fw_status[2] >> 4;
  m_artic_int1.rx_satellite_detected = fw_status[2] >> 5;
  m_artic_int1.tx_finished = fw_status[2] >> 6;
  m_artic_int1.mcu_command_accepted = fw_status[2] >> 7;

  m_artic_int1.crc_calculated = fw_status[1] & 1;
  m_artic_int1.idle_state = fw_status[1] >> 1;
  m_artic_int1.rx_calibration = fw_status[1] >> 2;

  //Print out the status
  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** INT1 STATUS ***");
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_valid_message: %d ", m_artic_int1.rx_valid_message);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_satellite_detected: %d ", m_artic_int1.rx_satellite_detected);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "tx_finished: %d ", m_artic_int1.tx_finished);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "mcu_command_accepted: %d ", m_artic_int1.mcu_command_accepted);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "crc_calculated: %d ", m_artic_int1.crc_calculated);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "idle_state: %d ", m_artic_int1.idle_state);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_calibration: %d ", m_artic_int1.rx_calibration);

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** artic_int1_callback EXIT ***");


  //TODO remove
  if(mcu_cmd_flag)
  {
    MW_LOG_DEBUG(ARTIC_LOG_TAG "*** CHECK INT1 STATUS ***");
  }

}


/**
 * @brief - Handle Interrupt 2 Received from ARTIC chpi
 */
//@int2
static void artic_interrupt_2_wait_and_clear(void)
{
  static uint8_t wait_count = 0;
  uint8_t fw_status[3];

  /* Wait for Interrupt */
  MW_LOG_DEBUG(ARTIC_LOG_TAG "Waiting for INT2");
  while(m_artic_int2_wait)
  {
    MW_LOG_DEBUG(ARTIC_LOG_TAG "...wait for INT2 to clear");
    vTaskDelay(255);

    wait_count++;
    if(wait_count>=50)
    {
      MW_LOG_DEBUG(ARTIC_LOG_TAG "...ARTIC unresponsive");
    }
  }
  wait_count = 0;

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** artic_int2_callback ENTER ***");

  //read the status and update the context vars
  artic_burst_spi_read(ARTIC_FIRMWARE_STATUS, ARTIC_IO_MEM, fw_status, 3);

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** artic_burst_spi_read(ARTIC_FIRMWARE_STATUS, ARTIC_IO_MEM, fw_status, 3) ***");

  //clear interrupt 2
  artic_int2_clear();

  //current FW state...
  m_artic_fw_state.idle = fw_status[2] & 1;
  m_artic_fw_state.rx_in_progress = fw_status[2] >> 1;
  m_artic_fw_state.tx_in_progress = fw_status[2] >> 2;
  m_artic_fw_state.dsp2mcu_int1 = fw_status[0] >> 6;
  m_artic_fw_state.dsp2mcu_int1 = fw_status[0] >> 7;

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** FW STATUS ***");
  MW_LOG_DEBUG(ARTIC_LOG_TAG "idle: %d ", m_artic_fw_state.idle);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_in_progress: %d ", m_artic_fw_state.rx_in_progress);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "tx_in_progress: %d ", m_artic_fw_state.tx_in_progress);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "dsp2mcu_int1: %d ", m_artic_fw_state.dsp2mcu_int1);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "dsp2mcu_int2: %d ", m_artic_fw_state.dsp2mcu_int2);

  m_artic_int2.rx_timeout = fw_status[1] >> 5;
  m_artic_int2.satellite_timeout = fw_status[1] >> 6;
  m_artic_int2.rx_buffer_overflow = fw_status[1] >> 7;

  m_artic_int2.tx_invalid_message = fw_status[0] & 1;
  m_artic_int2.mcu_command_rejected = fw_status[0] >> 1;
  m_artic_int2.mcu_command_overflow = fw_status[0] >> 2;
  m_artic_int2.internal_error = fw_status[0] >> 5;

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** INT2 STATUS ***");
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_timeout: %d ", m_artic_int2.rx_timeout);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "satellite_timeout: %d ", m_artic_int2.satellite_timeout);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_buffer_overflow: %d ", m_artic_int2.rx_buffer_overflow);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "tx_invalid_message: %d ", m_artic_int2.tx_invalid_message);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "mcu_command_rejected: %d ", m_artic_int2.mcu_command_rejected);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "mcu_command_overflow: %d ", m_artic_int2.mcu_command_overflow);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "internal_error: %d ", m_artic_int2.internal_error);

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** artic_int2_callback EXIT ***");

  vTaskDelay(5);
}



/**
 * @brief - Check FW status / Interupt Status
 */
bool check_artic_status()
{
  uint32_t full_fw_status = 0;
  uint8_t fw_status[3];
  //read the status and update the context vars
  artic_burst_spi_read(ARTIC_FIRMWARE_STATUS, ARTIC_IO_MEM, fw_status, 3);

  //clear the interrupt
  artic_int1_clear();

  //current FW state...
  m_artic_fw_state.idle = fw_status[2] & 1;
  m_artic_fw_state.rx_in_progress = fw_status[2] >> 1;
  m_artic_fw_state.tx_in_progress = fw_status[2] >> 2;
  m_artic_fw_state.dsp2mcu_int1 = fw_status[0] >> 6;
  m_artic_fw_state.dsp2mcu_int2 = fw_status[0] >> 7;

  MW_LOG_DEBUG(ARTIC_LOG_TAG "*** FW STATUS ***");
  MW_LOG_DEBUG(ARTIC_LOG_TAG "idle: %d ", m_artic_fw_state.idle);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "rx_in_progress: %d ", m_artic_fw_state.rx_in_progress);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "tx_in_progress: %d ", m_artic_fw_state.tx_in_progress);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "dsp2mcu_int1: %d ", m_artic_fw_state.dsp2mcu_int1);
  MW_LOG_DEBUG(ARTIC_LOG_TAG "dsp2mcu_int2: %d ", m_artic_fw_state.dsp2mcu_int2);

  //SE_NOTE... not sure how reliable these are
  m_artic_int1.rx_valid_message = fw_status[2] >> 4;
  m_artic_int1.rx_satellite_detected = fw_status[2] >> 5;
  m_artic_int1.tx_finished = fw_status[2] >> 6;
  m_artic_int1.mcu_command_accepted = fw_status[2] >> 7;

  m_artic_int1.crc_calculated = fw_status[1] & 1;
  m_artic_int1.idle_state = fw_status[1] >> 1;
  m_artic_int1.rx_calibration = fw_status[1] >> 2;


  full_fw_status = ( fw_status[0] << 16 ) | ( fw_status[1] << 8 ) | ( fw_status[2] );

  return (full_fw_status == 0x01);
}

/**
 * @brief - Clear all flags
 */
void clear_status_flags()
{
    m_artic_int1.rx_valid_message      = 0;
    m_artic_int1.rx_satellite_detected = 0;
    m_artic_int1.tx_finished           = 0;
    m_artic_int1.mcu_command_accepted  = 0;

    m_artic_int1.crc_calculated        = 0;
    m_artic_int1.idle_state            = 0;
    m_artic_int1.rx_calibration        = 0;

    m_artic_int2.rx_timeout            = 0;
    m_artic_int2.satellite_timeout     = 0;
    m_artic_int2.rx_buffer_overflow    = 0;

    m_artic_int2.tx_invalid_message    = 0;
    m_artic_int2.mcu_command_rejected  = 0;
    m_artic_int2.mcu_command_overflow  = 0;
    m_artic_int2.internal_error        = 0;

    m_artic_fw_state.idle              = 0;
    m_artic_fw_state.rx_in_progress    = 0;
    m_artic_fw_state.tx_in_progress    = 0;
    m_artic_fw_state.dsp2mcu_int1      = 0;
    m_artic_fw_state.dsp2mcu_int1 = 0;

    m_artic_int2_wait = 0;
    m_artic_int1_wait = 0;
}


/**
 * @brief - power OFF Artic chip
 */
//@off
static void artic_power_off(void)
{
  if ( m_artic_device.power_state != POWER_OFF )
  {
    TAKE_CNTRL();

    //reset interrupt status
    clear_status_flags();

    mw_set_artic_3V3_rail(OFF);

    mw_set_1V8_rail(OFF);

    mw_set_5V_rail(OFF);


    RELEASE_CNTRL();

    m_artic_device.power_state = POWER_OFF;
  }

  MW_LOG_INFO(ARTIC_LOG_TAG "Powered Off Artic ");
  vTaskDelay(1000);
}


/**
 * @brief - power ON Artic chip
 */
//@on
static void artic_power_on(void)
{
  if ( m_artic_device.power_state != POWER_ON )
  {
    TAKE_CNTRL();

    //reset interrupt status
    clear_status_flags();

    mw_set_1V8_rail(ON_PWM_MODE);
    mw_set_artic_3V3_rail(ON);

    /* Set PA Gain */
    rfpa0133_enable_set_gain(TWENTY_NINE_DB_MODE);

    m_artic_int2_wait = true;

    RELEASE_CNTRL();

    m_artic_device.power_state = POWER_ON;
  }

  vTaskDelay(500);
  MW_LOG_INFO(ARTIC_LOG_TAG "Powered On Artic ");
}





/**
 * @brief - Get TXCO configured warm-up time
 */
static uint8_t artic_get_tcxo_warmup_time(void)
{

  uint8_t tcxo_warm_up[3];
  //Check the CRC values to ensure that the FW got programmed correctly
  artic_burst_spi_read( ARTIC_REG_TCXO_WARMUP, ARTIC_X_MEM, tcxo_warm_up, 3);
  MW_LOG_INFO("TCXO Warm up: %x", tcxo_warm_up[2]);

  return tcxo_warm_up[2];
}


/**
 * @brief - Program Artic chip
 */
//@program
static void artic_program(void)
{
  uint8_t dsp_ctrl_write[4];
  uint8_t crc_result[9];

  uint8_t test_data[50];

  //reset interrupt status
  clear_status_flags();

  //Burst the ARTIC FW after supplying power
  //P MEM FW
  artic_burst_spi_write(0,
                        ARTIC_P_MEM,
                        (uint8_t *)artic_pmem,
                        ARTIC_FW_P_SIZE);

  memcpy( test_data, (uint8_t *)artic_pmem, 50);

  //X MEM FW
  artic_burst_spi_write(0,
                        ARTIC_X_MEM,
                        (uint8_t *)artic_xmem,
                        ARTIC_FW_X_SIZE);

  memcpy( test_data, (uint8_t *)artic_xmem, 50);

  //Y MEM FW
  artic_burst_spi_write(0,
                        ARTIC_Y_MEM,
                        (uint8_t *)artic_ymem,
                        ARTIC_FW_Y_SIZE);

  memcpy( test_data, (uint8_t *)artic_ymem, 50);

  test_data[0] *= 1;

  //Perform one 24 bit write access to the top level register dsp_ctrl_reg to activate the DSP
  //i. The 32b sequence written via SPI will be 0x02000000 - Pg15 of ARTIC datasheet
  dsp_ctrl_write[0] = 0x02;
  dsp_ctrl_write[1] = 0;
  dsp_ctrl_write[2] = 0;
  dsp_ctrl_write[3] = 0;

  artic_spi_write(dsp_ctrl_write, 4);

  m_artic_int1_wait = true;
  artic_interrupt_1_wait_and_clear();


  //Check the CRC values to ensure that the FW got programmed correctly
  artic_burst_spi_read(ARTIC_REG_CRC_RESULT,
                       ARTIC_X_MEM,
                       crc_result,
                       9);

  //this needs to be here to not crash....
  //nrf_delay_ms(1);
  vTaskDelay(2);

  uint32_t p_mem_crc = (crc_result[0] << 16) |  (crc_result[1] << 8) |  (crc_result[2]);
  uint32_t x_mem_crc = (crc_result[3] << 16) |  (crc_result[4] << 8) |  (crc_result[5]);
  uint32_t y_mem_crc = (crc_result[6] << 16) |  (crc_result[7] << 8) |  (crc_result[8]);


  if( p_mem_crc != ARTIC_PMEM_FW_CRC)
  {
    MW_LOG_ERROR(ARTIC_LOG_TAG "PMEM CRC MISMATCH!  EXPECTED : %d vs ACTUAL : %d ", ARTIC_PMEM_FW_CRC, p_mem_crc);
  }

  if( x_mem_crc != ARTIC_XMEM_FW_CRC)
  {
    MW_LOG_ERROR(ARTIC_LOG_TAG "XMEM CRC MISMATCH!  EXPECTED : %d vs ACTUAL : %d ", ARTIC_XMEM_FW_CRC, x_mem_crc);
  }

  if( y_mem_crc != ARTIC_YMEM_FW_CRC)
  {
    MW_LOG_ERROR(ARTIC_LOG_TAG "YMEM CRC MISMATCH!  EXPECTED : %d vs ACTUAL : %d ", ARTIC_YMEM_FW_CRC, y_mem_crc);
  }
}



/**
 * @brief - Transmit payload, assumtion is that the ID field is 28-bits
 */
//@send
uint8_t artic_transmit_argos2a_1_pkt(uint8_t *payload, argos2_message_size_t argos2_message_size)
{
#if !ARTIC_THREAD_ENABLED
  return ARTIC_SUCCESS;
#endif

  uint8_t tx_buffer[ARGOS_TX_BUFFER_MAX_SIZE];
  uint8_t msg_length = 0;
  int payload_size;
  int i,j;

  if(m_artic_device.power_state != POWER_ON) return ARTIC_ERROR_INVALID_STATE;

  //following sequence from 3.12.1 in ARTIC datasheet

  //clear status
  clear_status_flags();
  vTaskDelay(200);
  artic_int1_clear();
  vTaskDelay(200);
  artic_int2_clear();
  vTaskDelay(200);


  while(!check_artic_status())
  {
    MW_LOG_ERROR(ARTIC_LOG_TAG "Waiting for the ARTIC to be idle");
    vTaskDelay(200);
    artic_int2_clear();
  }

  MW_LOG_ERROR(ARTIC_LOG_TAG "**** Setting ARTIC to PTT - ARGOS 2 mode");

  vTaskDelay(1200);
  //set the mode
  artic_command_write(ARTIC_CMD_SET_PTT_A2_TX_MODE);

  m_artic_int1_wait = true;
  artic_interrupt_1_wait_and_clear();

//  MW_LOG_ERROR(ARTIC_LOG_TAG "**** Wait for INT 1");
//  mcu_cmd_flag = true; //TODO remove
//  while(!check_artic_status())
//  {
//    MW_LOG_ERROR(ARTIC_LOG_TAG "Waiting for the ARTIC to be idle");
//    vTaskDelay(200);
//    artic_int2_clear();
//  }
  check_artic_status();

  MW_LOG_ERROR(ARTIC_LOG_TAG "**** INT 1 recevied");

  //look up the encoded length for the message header
  //from PLATFORM TRANSMITTER TERMINAL (PTT-A2) specification
  //TODO move to another API
  switch(argos2_message_size)
  {
      case ARGOS2_MESSAGE_56_BITS:
        msg_length = 0b0000;
        break;
      case ARGOS2_MESSAGE_120_BITS:
        msg_length = 0b0011;
        break;
      case ARGOS2_MESSAGE_152_BITS:
        msg_length = 0b0101;
        break;
      case ARGOS2_MESSAGE_184_BITS:
        msg_length = 0b1001;
        break;
      case ARGOS2_MESSAGE_216_BITS:
        msg_length = 0b1010;
        break;
      case ARGOS2_MESSAGE_248_BITS:
        msg_length = 0b1100;
        break;
      case ARGOS2_MESSAGE_280_BITS:
        msg_length = 0b1111;
        break;
      default:
        break;
  }

  //*********************************************************
  //Prepare the header....

  //first 3 bytes are for the size
  tx_buffer[0] = argos2_message_size >> 16;
  tx_buffer[1] = argos2_message_size >> 8;
  tx_buffer[2] = argos2_message_size;


  //*********************************************************
  xSemaphoreTake(artic_semph, portMAX_DELAY);
  /* Set ARGOS User ID Field */
  MW_LOG_ERROR( ARTIC_LOG_TAG "Argos User ID: %X:%X:%X:%X", m_argos_user_id[0]
                                                          , m_argos_user_id[1]
                                                          , m_argos_user_id[2]
                                                          , m_argos_user_id[3] );
  tx_buffer[3] = (msg_length << 4) | m_argos_user_id[0];
  tx_buffer[4] = m_argos_user_id[1];
  tx_buffer[5] = m_argos_user_id[2];
  tx_buffer[6] = m_argos_user_id[3];
  xSemaphoreGive(artic_semph);

  //*********************************************************

  //32 is for 4-bit message size field plus 28-bit ID field
  payload_size = (argos2_message_size-32)/8;

  for(i=0;i<payload_size;i++)
  {
    tx_buffer[7+i] = payload[i];
  }
  //Add zeros to the end
  for(j=i+7;j<ARGOS_TX_BUFFER_MAX_SIZE;j++)
  {
    tx_buffer[j] = 0;
  }

  artic_burst_spi_write(ARTIC_REG_TX_PAYLOAD,
                        ARTIC_X_MEM,
                        tx_buffer,
                        ARGOS_TX_BUFFER_MAX_SIZE);
  //*********************************************************

  //Wait for TCXO warm-up time
  vTaskDelay(m_artic_device.tcxo_warm_up_time);

  //*********************************************************
  //Transmit command
  artic_command_write(ARTIC_CMD_TX_1PKG);

  //*********************************************************

  mcu_cmd_flag = true; //TODO remove
  m_artic_int1_wait = true;
  artic_interrupt_1_wait_and_clear();

  //Wait for tx_finished status
  while (!m_artic_int1.tx_finished)
  {
    MW_LOG_DEBUG(ARTIC_LOG_TAG "Waiting for tx_finished")
    vTaskDelay(50);
  }

  return ARTIC_SUCCESS;
}



/**
 * @brief - Read Artic chip firmware version
 */
//@fw
uint32_t artic_read_firmware_version(void)
{
  uint8_t fw_version[9];
  //Check the CRC values to ensure that the FW got programmed correctly
  artic_burst_spi_read(ARTIC_FIRMWARE_VERSION,
                         ARTIC_P_MEM,
                         fw_version,
                         9);
  MW_LOG_INFO( "ARTIC FW Version Read: %x %x %x", fw_version[5], fw_version[6], fw_version[7] );

  check_artic_status(); //TODO remove

  return fw_version[5] << 16 | fw_version[6] << 8 | fw_version[7];
}



/**
 * @brief - Initialize SPI Interface
 */
//@setup
static void artic_setup_interface()
{
  mw_spi_config_t spi_config;
  spi_config.spi_instance    = ARTIC_SPI_INSTANCE;
  spi_config.frequency       = NRF_SPI_FREQ_500K; //NRF_SPI_FREQ_1M;
  spi_config.irq_priority    = ARTIC_SPI_IRQ_PRIORITY;
  spi_config.miso_pin        = PIN_ARTIC_SPI_MISO;
  spi_config.mosi_pin        = PIN_ARTIC_SPI_MOSI;
  spi_config.sck_pin         = PIN_ARTIC_SPI_SCK;
  spi_config.ss_pin          = PIN_ARTIC_SPI_NSS;
  spi_config.mode            = ARTIC_SPI_MODE;
  spi_config.bit_order       = NRF_SPI_BIT_ORDER_MSB_FIRST;
  spi_config.handler         = NULL;
  artic_init( spi_config );

  m_artic_device.interface_initialized = true;
}


/**
 * @brief - Enable Artic Interrupts
 */
static void enable_artic_interrupt()
{
  nrfx_gpiote_in_event_enable(PIN_ARTIC_INT1, true);
  nrfx_gpiote_in_event_enable(PIN_ARTIC_INT2, true);

  read_in_progress = false;
  write_in_progress = false;
}

/**
 * @brief - Disable Artic Interrupts
 */
static void disable_artic_interrupt()
{
  nrfx_gpiote_in_event_disable(PIN_ARTIC_INT1);
  nrfx_gpiote_in_event_disable(PIN_ARTIC_INT2);
}


/**
 * @brief - Setup pin interrupt via GPIOTE.  Module assumed to be initialized in main.c
 */
static void artic_configure_interrupts()
{
  uint32_t err_code;

  /* Verify GPIOTE was already initialized */
  if(!nrfx_gpiote_is_init())  nrfx_gpiote_init();

  nrf_gpio_cfg_input(PIN_ARTIC_INT1, NRF_GPIO_PIN_PULLDOWN); //NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(PIN_ARTIC_INT2, NRF_GPIO_PIN_PULLDOWN); //NRF_GPIO_PIN_NOPULL);

  nrfx_gpiote_in_config_t int_pin_cfg = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_TOGGLE(false); //NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_TOGGLE(false); NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(false);

  err_code = nrfx_gpiote_in_init( PIN_ARTIC_INT1, &int_pin_cfg, artic_pin_irq);
  APP_ERROR_CHECK(err_code);
  err_code = nrfx_gpiote_in_init( PIN_ARTIC_INT2, &int_pin_cfg, artic_pin_irq);
  APP_ERROR_CHECK(err_code);

  read_in_progress = false;
  write_in_progress = false;
}


/**
 * @brief - Resume Thread
 *
 * @param - If calling function was from Interrupt Context
 */
//@resume
static void MW_RESUME_ARTIC_THREAD( bool isr )
{
  if(m_artic_thread_mode == THREAD_NULL ) return;

  m_artic_thread_mode = THREAD_ACTIVE;
  if(isr)
  {
    xTaskResumeFromISR(m_mw_artic_thread); // Resume myself
  }
  else
  {
    vTaskResume(m_mw_artic_thread); // Resume myself
  }
}

/**
 * @brief - Suspend Thread
 */
//@suspend
static void MW_SUSPEND_ARTIC_THREAD()
{
  if(m_artic_thread_mode == THREAD_NULL ) return;
  m_artic_thread_mode = THREAD_SUSPENDED;
  vTaskSuspend(m_mw_artic_thread); // Suspend myself
}


/**
 * @brief Thread is ready
*/
bool mw_artic_thread_ready(void)
{
#if !ARTIC_THREAD_ENABLED
  return true;
#endif
  return m_artic_thread_mode != THREAD_NULL;
}



/**
 * @brief - Sensor Thread Task
 */
static void mw_artic_task(void * arg)
{
  /* Initialize PA */
  rfpa0133_init();
  rfpa0133_low_power_mode();

  artic_power_down();
  test_artic_power_up_program_power_down();

  m_artic_thread_mode = THREAD_ACTIVE;

  while(1)
  {
    //TODO add states
    MW_SUSPEND_ARTIC_THREAD();

  }
}


/**
 * @brief Thread is ready
*/
bool artic_thread_ready(void)
{
#if !ARTIC_THREAD_ENABLED
  return true;
#endif
  return m_artic_thread_mode != THREAD_NULL;
}


/********************************************************************************************
 *
 *  Initialization Function
 *
 *******************************************************************************************/

/**
 * @brief Function for application main entry.
*/
void artic_thread_init(void)
{
  artic_configure_interrupts();
  disable_artic_interrupt();

  m_artic_device.interface_initialized = false;
  m_artic_device.power_state = UNKNOWN_STATE;
  m_artic_device.tcxo_warm_up_time = DEFAULT_TXCO_WARMUP_TIME;

  /*Default ARGOS ID*/
  //TODO check Flash
  m_argos_user_id[0] = (uint8_t) (DEFAULT_ARGOS_28BIT_ID_FIELD >> 24);
  m_argos_user_id[1] = (uint8_t) (DEFAULT_ARGOS_28BIT_ID_FIELD >> 16);
  m_argos_user_id[2] = (uint8_t) (DEFAULT_ARGOS_28BIT_ID_FIELD >> 8);
  m_argos_user_id[3] = (uint8_t) (DEFAULT_ARGOS_28BIT_ID_FIELD);

  artic_semph = xSemaphoreCreateBinary();
  if (NULL == artic_semph)
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  xSemaphoreGive(artic_semph);

  if(xTaskCreate( mw_artic_task, "ARTIC", ARTIC_THREAD_STACK_SIZE, NULL, ARTIC_THREAD_PRIORITY, &m_mw_artic_thread ) != pdPASS)
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  nrf_gpio_cfg_output(SPIM2_MISO_PIN);
  nrf_gpio_cfg_output(SPIM2_MOSI_PIN);
  nrf_gpio_cfg_output(SPIM2_SCK_PIN);
  nrf_gpio_cfg_output(SPIM2_SS_PIN);
}









