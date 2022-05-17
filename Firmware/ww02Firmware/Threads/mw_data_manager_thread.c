/*
n * mw_data_manager_thread.c
 *
 *  Created on: Oct 8, 2019
 *      Author: klockwood
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf52.h"
#include "nrf_svc.h"
#include "nrf_soc.h"
#include "nrfx_gpiote.h"
#include "boards.h"
#include "nrf_power.h"
#include "FreeRTOS_includes.h"
#include <mw_logging.h>
#include "mw_thread.h"

#include "mw_data_manager_thread.h"
#include "project_settings.h"

#include "mw_sensor_thread.h"
#include "mw_temperature_thread.h"
#include "mw_rtc_thread.h"
#include "mw_adc_thread.h"
#include "mw_flash_thread.h"
#include "mw_leds_thread.h"
#include "artic_thread.h"

#include "mw_power_management.h"
#include "argos_pass_prediction.h"
#include "wwf_artic_payload.h"
#include "wwf_device_settings.h"
#include "wwf_service.h"

#if (PROFILE_INDEX_MAX > 31)
#warning "Maximum Argos Payload size is 31bytes"
#endif

#if (DATA_MANAGER_THREAD_ENABLED && !ARTIC_THREAD_ENABLED)
#warning "Data Manager won't run without the ARTIC Thread Enabled in project_settings.h"
#endif

#define DATA_LOG_TAG                "Data Manager Thread: "

#define BLE_MSG_TX_DONE             "Tx Phase Done"

SemaphoreHandle_t                   data_semph;

TaskHandle_t                        m_mw_data_thread;        /**< Definition of Thread. */

static volatile mw_thread_mode_t    m_mw_data_thread_mode = THREAD_NULL;


/******************************************/
/* Global Variables */
static time_reg_struct_t                m_system_time;
//static uint16_t                         m_activity_count = 0;
static uint16_t                         m_system_profile = 0x01;
static wwf_profile_type_1_data_struct_t m_payload;
static device_orientation_t             m_device_orientation = UNKNOWN;

static uint32_t                         m_tracking_cycle_time =  PROFILE_DATA_GATHER_INTERVAL;         /*data gathering cycles units */
static rtc_time_unit_t                  m_tracking_cycle_units = PROFILE_DATA_GATHER_INTERVAL_UNITS;   /*data gathering cycles units */

static uint8_t                          m_artic_send_buffer[31];
static uint8_t                          m_artic_send_repeats = 0;

#if ARTIC_THREAD_ENABLED
static uint8_t                          m_artic_send_repeats_limit = DEFAULT_TRANSMISSION_REPITIIONS;
static uint32_t                         m_artic_send_interval = DEFAULT_TRANSMISSION_INTERVAL;
#endif
/******************************************/
/* State Control Flags */
static bool m_rtc_alarming = false;
static bool m_orientation_get = false;
static bool m_temperature_alarm = false;
static bool m_sending_payload = false;
static bool m_send_device_summary = false;
static bool m_start_tracking = false;
static bool m_dfu_mode = false;

#if ARTIC_THREAD_ENABLED
static bool m_rtc_time_get = false;
#endif
/******************************************/

static void update_bear_id_and_profile(  const uint16_t data, bool isr );
static void trigger_device_summary_send( bool isr );
void send_device_summary();
static void device_start_tracking( uint8_t time, uint8_t units );

void decompress_payload( wwf_profile_type_1_data_struct_t * data_buffer, uint8_t * buffer );
void compress_payload( wwf_profile_type_1_data_struct_t * data_buffer, uint8_t * buffer );
void reset_system_variables();

static void check_dfu_trigger();

static void MW_SUSPEND_DATA_MANAGER_THREAD();
static void MW_RESUME_DATA_MANAGER_THREAD( bool isr );


/******************************************/
/* Test Mode Variables */
#define LP_MODE_ENABLE               0x0000
#define NORMAL_MODE_ENABLE           0x0001
#define STOP_TEST_SEQUENCE           0xA000
#define START_TEST_SEQUENCE          0xA001
#define DISABLE_1V8                  0x1800
#define ENABLE_1V8_PWM               0x1801
#define ENABLE_1V8_HYST              0x1802
#define DISABLE_4V2                  0x4200
#define ENABLE_4V2_PWM               0x4201
#define ENABLE_4V2_BURST             0x4202

static bool m_test_mode = false;
static void mw_test_mode_cmd( const uint8_t * data, uint8_t length );

/******************************************/
/*********************************************************/


#if PA_TEST_MODE_FOR_JOHN

#include "RFPA0133.h"
#include "mw_power_management.h"
static volatile bool       m_update_pa_gain  = false;
static rfpa0133_gain_modes m_pa_gain_update_value = TWENTY_NINE_DB_MODE;
#endif

/********************************************************************************************
 *
 *  Test Function(s) *
 *
 *******************************************************************************************/
#if ARTIC_THREAD_ENABLED
static uint8_t test_buffer[31];

static void test_data_compression_decompression()
{
  wwf_profile_type_1_data_struct_t decompress_data;

// Data Pack Test
  m_payload.bear_id_and_profile = DEFAULT_BEAR_ID_AND_PROFILE;
  mw_flash_thread_save_bear_id_and_profile(m_payload.bear_id_and_profile, NORMAL_CONTEXT);
  mw_flash_thread_load_bear_id_and_profile(&m_payload.bear_id_and_profile);
  m_payload.high_temperature = -20;
  m_payload.low_temperature = -44;
  m_payload.average_temperature = -28;
  m_payload.pressure_counts = 55;
  m_payload.battery_level = 94;
  bool flag = true;
  for ( uint8_t i = 0; i < PROFILE_DATA_GATHER_INDEX_MAX; i++ )
  {
    m_payload.wwf_activity_payload[i] = (i + 16380) & 0x3FFF;  /*simulate a 14bit value*/
//    if(flag) m_payload.wwf_activity_payload[i] = 0x3FFF;
//    if(!flag) m_payload.wwf_activity_payload[i] = 0x3555;
    flag = !flag;
  }

  compress_payload(&m_payload, test_buffer);

  test_buffer[0] *= 1;

  decompress_payload(&decompress_data, test_buffer);
 // m_payload.bear_id = 0x122;
  if( memcmp(&decompress_data, &m_payload, sizeof(m_payload)) )
  {
    MW_LOG_INFO(DATA_LOG_TAG"Decompressed Payload DOES NOT MATCH Original");
  }
  else
  {
    MW_LOG_INFO(DATA_LOG_TAG"Decompressed Payload MATCHES Original!!!!!!!!!");
  }
}


static void test_pass_prediction()
{
  llist * test_list = malloc(sizeof(llist));
  *test_list = NULL;

  pp_t new_value = {
  /*sat*/ {'A', 'B'},
  /*tpp*/ 123456,
  /*year_pp*/ 2019,
  /*month_pp*/ 10,
  /*day_pp*/ 31,
  /*hour_pp*/ 13,
  /*min_pp*/ 22,
  /*sec_pp*/ 59,
  /*duration*/ 300,
  /*site_max*/ 2
  };


  add_element_to_end( test_list, new_value );

  new_value.min_pp = 31;

  add_element_to_end( test_list, new_value );

  new_value.sat[0] = 'C';
  new_value.sat[1] = 'D';

  add_element_to_end( test_list, new_value );
}


static void set_rtc_default_time_and_alarm()
{
#if BYPASS_ARGOS_TRANSMISSIONS
  return;
#endif
#if RTC_THREAD_ENABLED
  /*
   * hundredth : 0 ~ 99
   * second : 0 ~ 59
   * minute : 0 ~ 59
   * weekday : 0 ~ 6
   * month : 1 ~ 12
   * year : 0 ~ 99
   * mode : 0 ~ 2
   */
  time_reg_struct_t time_regs;
  time_regs.month = 10;
  time_regs.date = 3;
  time_regs.weekday = 3;
  time_regs.year = 19;
  time_regs.hour = 12;
  time_regs.minute = 00;
  time_regs.second = 00;
  time_regs.hundredth = 0;
  time_regs.mode = TWENTY_FOUR_HR_FORMAT;
  am0805_set_time(time_regs);


  /* Set Alarm */
  MW_LOG_INFO(DATA_LOG_TAG "*****************************");
  MW_LOG_INFO(DATA_LOG_TAG " Starting Animal Tracking");
  MW_LOG_INFO(DATA_LOG_TAG " Reset Tracking Variables");

  mw_rtc_auto_set_alarm( m_tracking_cycle_time, m_tracking_cycle_units, NORMAL_CONTEXT);
#endif
}
#endif

/********************************************************************************************
 *
 *  Handler Function(s) *
 *
 *@handlers
 *******************************************************************************************/
static void m_rtc_alarm_handler( mw_rtc_mode_t const rtc_status, bool isr )
{
  m_rtc_alarming = true;
  MW_RESUME_DATA_MANAGER_THREAD(isr);
}


/**
 * brief - Orientation handler
 */
//@orient
static void m_orientation_check_handler( device_orientation_t const orientation_status, bool isr )
{
  MW_LOG_INFO(DATA_LOG_TAG"Orientation Handler");

  if( orientation_status == GOOD_ORIENTATION )
  {
    MW_LOG_INFO(DATA_LOG_TAG"GOOD orientation");
    m_device_orientation = orientation_status;
    m_sending_payload = true;
    m_artic_send_repeats = 0;
    MW_RESUME_DATA_MANAGER_THREAD(isr);
  }
  if( orientation_status == BAD_ORIENTATION )
  {
    MW_LOG_INFO(DATA_LOG_TAG"BAD orientation");
  }

  m_orientation_get = false;
}


static void m_temperature_handler( mw_temp_operating_mode_t const temperature_state, bool isr )
{
  m_temperature_alarm = true;
  MW_RESUME_DATA_MANAGER_THREAD(isr);
}


/**
 * @brief - Internal Command Handler to distribute date to its intended destination
 */
//@cmd
static void wwf_service_handler( const uint8_t * data, uint8_t length )
{

#if PA_TEST_MODE_FOR_JOHN
  if( length==1) mw_data_manager_update_pa_gain(data[0]);

  if(data[0] == UPDATE_DEVICE_FW)
  {
    m_dfu_mode = true;
    check_dfu_trigger();
  }
  return;
#endif

  switch( data[0] )
  {
  case BEAR_ID_AND_PROFILE_UUID:
    if( length==BEAR_ID_AND_PROFILE_CHAR_SIZE) update_bear_id_and_profile((uint16_t)(data[1] <<8 | data[2]), ISR_CONTEXT);
    break;

  case ACTIGRAPHY_THRESHOLD_UUID:
    if( length==ACTIGRAPHY_THRESHOLD_CHAR_SIZE)
    {
      mw_sensor_update_activity_threshold( mw_sensor_threshold_look_up_table(data[1]), ISR_CONTEXT);
    }
    break;

  case SET_TIME_UUID:
    if( length==SET_TIME_CHAR_SIZE) mw_rtc_set_time_raw(&data[1],length, ISR_CONTEXT);
    break;

  case SET_LOCATION_UUID:
    break;

  case START_TRACKING:
    if( m_test_mode ) return;
    if( length == 3 ) device_start_tracking(data[1], data[2]);
    break;

  case GET_DEVICE_SUMMARY:
    trigger_device_summary_send(ISR_CONTEXT);
    break;

  case SET_ARGOS_USER_ID:
    if( length==SET_ARGOS_USER_ID_CHAR_SIZE) update_argos_user_id(&data[1], length);
    break;

  case UPDATE_DEVICE_FW:
    m_dfu_mode = true;
    MW_RESUME_DATA_MANAGER_THREAD(ISR_CONTEXT);
    break;

  case SET_PA_GAIN:
    if( length==1) mw_data_manager_update_pa_gain(data[0]);
    break;


  case RUN_TEST_UUID:
    if( m_start_tracking ) return;
    if( length == TEST_CMD_CHAR_SIZE ) mw_test_mode_cmd( &data[1], TEST_CMD_CHAR_SIZE );
    MW_RESUME_DATA_MANAGER_THREAD(ISR_CONTEXT);
    break;

  }
}
/********************************************************************************************
 *
 *  Internal Function *
 *
 *******************************************************************************************/
/*
 *
 * Test Mode Command Arbiter
 *
 */
static void mw_test_mode_cmd( const uint8_t * data, uint8_t length )
{
  m_test_mode = true;

  switch( data[0] << 8 | data[1] )
  {
  case LP_MODE_ENABLE:
    // if disconnected
    //TODO go to slow advertising
    //mw_ble_advertising_start( mw_ble_adv_mode_t advertising_mode ) MW_BLE_ADV_MODE_SLOW
    break;
  case NORMAL_MODE_ENABLE:
    break;
  case STOP_TEST_SEQUENCE:
    break;
  case START_TEST_SEQUENCE:
    break;
  case DISABLE_1V8:
    mw_set_1V8_rail( OFF );
    break;
  case ENABLE_1V8_PWM:
    mw_set_1V8_rail( ON_PWM_MODE );
    break;
  case ENABLE_1V8_HYST:
    mw_set_1V8_rail( ON_HYSTERESIS_MODE );
    break;
  case DISABLE_4V2:
    mw_set_5V_rail( OFF );
    break;
  case ENABLE_4V2_PWM:
    mw_set_5V_rail( ON_FIXED_PWM );
    break;
  case ENABLE_4V2_BURST:
    mw_set_5V_rail( ON_BURST_MODE );
    break;
  }
}



/*
 * Update/Save Bear ID
 *
 */
static void update_bear_id_and_profile(  const uint16_t data, bool isr )
{
  /*Set new Bear ID and Profile */
  m_payload.bear_id_and_profile = data & 0x3FF;
  mw_flash_thread_save_bear_id_and_profile(m_payload.bear_id_and_profile, isr);

  /* 2 MSB are the Profile setting */
  m_system_profile = m_payload.bear_id_and_profile >> 10;
}


/**
 * @brief - Reset Tracking variables and set alarm
 */
void reset_system_variables()
{
  mw_sensor_reset_temperature();
  mw_sensor_reset_activity_counter();
}


/**
 * @brief - Sends Device summary
 */
static void trigger_device_summary_send( bool isr )
{
  m_send_device_summary = true;
  MW_RESUME_DATA_MANAGER_THREAD(isr);
}


/**
 * @brief - Start tracking
 */
static void device_start_tracking( uint8_t time, uint8_t units )
{
    m_start_tracking = true;

    m_tracking_cycle_time = time;

    switch(units)
    {
    case WWF_DATA_TRACKING_UNIT_SECONDS:
      m_tracking_cycle_units = RTC_SECONDS;
      break;

    case WWF_DATA_TRACKING_UNIT_MINUTES:
      m_tracking_cycle_units = RTC_MINUTES;
      break;

    case WWF_DATA_TRACKING_UNIT_HOURS:
      m_tracking_cycle_units = RTC_HOURS;
      break;

    case WWF_DATA_TRACKING_UNIT_DAYS:
      m_tracking_cycle_units = RTC_DAYS;
      break;

    default:
      return;
      break;
    }

    MW_RESUME_DATA_MANAGER_THREAD(ISR_CONTEXT);
}


/**
 * @brief - Sends Device summary
 */
void send_device_summary()
{
  /* Get Bear ID + Profile */
  mw_flash_thread_load_bear_id_and_profile( &m_payload.bear_id_and_profile );

  /* Get Current Time */
  time_reg_struct_t current_time;
  mw_rtc_get_time( &current_time );


  /* Send Actigraphy Threshold */
  float threshold = mw_sensor_get_actigraphy_threshold();


  /* Fill Payload and Send */
  uint8_t send_payload[17];

  send_payload[0] = (uint8_t)'I';
  send_payload[1] = (uint8_t)'D';
  send_payload[2] = m_payload.bear_id_and_profile >> 8;
  send_payload[3] = (uint8_t) m_payload.bear_id_and_profile;

  send_payload[4] = (uint8_t)'T';
  send_payload[5] = (uint8_t)'I';
  send_payload[6] = (uint8_t)'M';
  send_payload[7] = (uint8_t)'E';
  send_payload[8] = current_time.year;
  send_payload[9] = current_time.month;
  send_payload[10] = current_time.date;
  send_payload[11] = current_time.weekday;
  send_payload[12] = current_time.hour;
  send_payload[13] = current_time.minute;


  send_payload[14] = (uint8_t)'A';
  send_payload[15] = (uint8_t)'T';
  send_payload[16] = (uint8_t)(threshold);

  wwf_send_data(send_payload, 17);
}


/**
 * @brief -  Deompress data into WWF Payload Format
 */
void decompress_payload( wwf_profile_type_1_data_struct_t * data_buffer, uint8_t * buffer )
{
  //uint8_t temp_value;

  data_buffer->bear_id_and_profile = (buffer[0] << 4) | ((buffer[1] & 0xF0) >> 4);

  data_buffer->pressure_counts = ((buffer[1] & 0x0F) << 4) | ((buffer[2] & 0xF0) >> 4);

  data_buffer->average_temperature = ((buffer[2] & 0x0F) << 4) | ((buffer[3] & 0xF0) >> 4);
  data_buffer->high_temperature = ((buffer[3] & 0x0F) << 4) | ((buffer[4] & 0xF0) >> 4);
  data_buffer->low_temperature = ((buffer[4] & 0x0F) << 4) | ((buffer[5] & 0xF0) >> 4);

  data_buffer->battery_level = ((buffer[5] & 0x0F) << 4) | ((buffer[6] & 0xF0) >> 4);


  data_buffer->wwf_activity_payload[0] = ((buffer[6] & 0x0F) << 10) |  ( buffer[7] << 2 ) | ((buffer[8] >> 6) & 0x03);

  data_buffer->wwf_activity_payload[1] = ((buffer[8] & 0x3F) << 8) |  ( buffer[9]  );

  data_buffer->wwf_activity_payload[2] = ((buffer[10] << 6) |  ( (buffer[11] & 0xFC ) >> 2 ));

  data_buffer->wwf_activity_payload[3] = ((buffer[11] & 0x03) << 12) |  ( buffer[12] << 4 ) | (buffer[13] >> 4);


  data_buffer->wwf_activity_payload[4] = ((buffer[13] & 0x0F) << 10) |  ( buffer[14] << 2 ) | ((buffer[15] >> 6) &0x03);

  data_buffer->wwf_activity_payload[5] = ((buffer[15] & 0x3F) << 8) |  ( buffer[16]  );

  data_buffer->wwf_activity_payload[6] = ((buffer[17] << 6) |  ( (buffer[18] & 0xFC ) >> 2 ));

  data_buffer->wwf_activity_payload[7] = ((buffer[18] & 0x03) << 12) |  ( buffer[19] << 4 ) | (buffer[20] >> 4);


  data_buffer->wwf_activity_payload[8] = ((buffer[20] & 0x0F) << 10) |  ( buffer[21] << 2 ) | ((buffer[22] >> 6) &0x03);

  data_buffer->wwf_activity_payload[9] = ((buffer[22] & 0x3F) << 8) |  ( buffer[23]  );

  data_buffer->wwf_activity_payload[10] = ((buffer[24] << 6) |  ( (buffer[25] & 0xFC ) >> 2 ));

  data_buffer->wwf_activity_payload[11] = ((buffer[25] & 0x03) << 12) |  ( buffer[26] << 4 ) | (buffer[27] >> 4);


  data_buffer->wwf_activity_payload[12] = ((buffer[27] & 0x0F) << 10) |  ( buffer[28] << 2 ) | ((buffer[29] >> 6) &0x03);

  data_buffer->wwf_activity_payload[13] = ((buffer[29] & 0x3F) << 8) |  ( buffer[30]  );

}


/**
 * @brief -  Compress data into WWF Payload Format
 */
void compress_payload( wwf_profile_type_1_data_struct_t * data_buffer, uint8_t * buffer )
{
  uint8_t temp_value;

  memset( buffer, 0, sizeof(buffer) );

  /* Pack Bear ID */
  buffer[0] = (( data_buffer->bear_id_and_profile >> 8 ) & 0x0F) << 4;   // Grab upper byte LSB 4bits - shift into byte
  temp_value = ( data_buffer->bear_id_and_profile & 0xF0 ) >> 4;          // Grab lower byte MSB 4bits - shift into byte;
  buffer[0] = buffer[0] | temp_value;
  buffer[1] = ( data_buffer->bear_id_and_profile & 0x0F ) << 4;          // Grab lower byte LSB 4bits - shift into byte

  /* Pack Pressure Counts */
  temp_value =  (data_buffer->pressure_counts & 0xF0) >> 4;    // Grab byte MSB 4bits - shift into byte
  buffer[1] = buffer[1] | temp_value;
  buffer[2] = (data_buffer->pressure_counts & 0x0F) << 4;   // Grab byte LSB 4bits - shift into byte

  /* Pack Temperature Data */
  temp_value = (data_buffer->average_temperature & 0xF0) >> 4; // Grab byte MSB 4bits - shift into byte
  buffer[2] = buffer[2] | temp_value;
  buffer[3] = (data_buffer->average_temperature & 0x0F) << 4; // Grab byte LSB 4bits - shift into byte
  temp_value = (data_buffer->high_temperature & 0xF0) >> 4;    // Grab byte MSB 4bits - shift into byte
  buffer[3] = buffer[3] | temp_value;
  buffer[4] = (data_buffer->high_temperature & 0x0F) << 4;    // Grab byte LSB 4bits - shift into byte
  temp_value = (data_buffer->low_temperature & 0xF0) >> 4;     // Grab byte MSB 4bits - shift into byte
  buffer[4] = buffer[4] | temp_value;
  buffer[5] = (data_buffer->low_temperature & 0x0F) << 4;     // Grab byte LSB 4bits - shift into byte


  /* Pack Pressure Counts */
  temp_value = (data_buffer->battery_level & 0xF0) >> 4;    // Grab byte MSB 4bits - shift into byte
  buffer[5] = buffer[5] | temp_value;
  buffer[6] = (data_buffer->battery_level & 0x0F) << 4;   // Grab byte LSB 4bits - shift into byte


  /* Pack  Actigraphy */
  uint8_t buffer_index = 6;
  uint8_t activity_index = 0;
  while( buffer_index < (PAYLOAD_PAYLOAD_SIZE_BITS/8) )
  {
    /*** Next Entry ***/
    temp_value =  ( ( data_buffer->wwf_activity_payload[activity_index] >> 10 ) &  0x0F);  // Grab upper byte MSB 4bits - shift into byte
    buffer[buffer_index] = buffer[buffer_index] | temp_value;
    /* byte 6,13,20,27 */
    buffer[buffer_index + 1] = ((uint8_t) ( data_buffer->wwf_activity_payload[activity_index] >> 2 ) & 0xFF );   // Grab next 8bits - shift into byte
    /* byte 7,14,21,28 */
    buffer[buffer_index + 2] = ( ( data_buffer->wwf_activity_payload[activity_index] ) & 0x03 ) << 6;   // Grab LSB 2bits - shift into byte
    /* byte 8,15,22,29 */
    /*****************/

    activity_index++;

    /*** Next Entry ***/
    temp_value =  ( ( data_buffer->wwf_activity_payload[activity_index] >> 8 ) &  0x3F);  // Grab MSB 6bits - shift into byte
    buffer[buffer_index + 2] = buffer[buffer_index + 2] | temp_value;
    /* byte 8,15,29 */


    buffer[buffer_index + 3] = ( ( data_buffer->wwf_activity_payload[activity_index]) & 0xFF );   // Grab shift into byte
    /* byte 9,16,23,30 */


    activity_index++;
    if(activity_index == 14) /* @byte 30 */
    {
      return;
    }

    /*****************/


    /*** Next Entry ***/
    //byte 10,17,24
    buffer[buffer_index + 4] =  ( ( data_buffer->wwf_activity_payload[activity_index] >> 6 ) &  0xFF);  // Grab MSB 8bits - shift into byte


    buffer[buffer_index + 5] = ( ( data_buffer->wwf_activity_payload[activity_index] & 0x3F ) << 2 );   // Grab LSB 6bits of lower byte - shift into byte
    //byte 11,18,25
    /*****************/

    activity_index++;

    /*** Next Entry ***/

    temp_value = ( ( data_buffer->wwf_activity_payload[activity_index] >> 12) & 0x03 );  // Grab upper byte MSB 2bits - shift into byte

    buffer[buffer_index + 5] = buffer[buffer_index + 5] | temp_value;
    //byte 11,18,25

    buffer[buffer_index + 6] = ( ( data_buffer->wwf_activity_payload[activity_index] >> 4) & 0xFF );  // Grab next MSB 8bits - shift into byte
    //byte 12,19,26


    buffer[buffer_index + 7] = ( ( data_buffer->wwf_activity_payload[activity_index] ) &  0x0F) << 4;  // Grab next LSB 4bits - shift into byte
    //byte 13,20,27


    buffer_index += 7;
    activity_index++;
  }
}


void log_payload()
{
  MW_LOG_INFO(DATA_LOG_TAG"*********************************");
  MW_LOG_INFO(DATA_LOG_TAG"Sending Payload..");
  MW_LOG_INFO(DATA_LOG_TAG"Bear ID: %d", m_payload.bear_id_and_profile);
  MW_LOG_INFO(DATA_LOG_TAG"Battery Level: %d", m_payload.battery_level);
  MW_LOG_INFO(DATA_LOG_TAG"Average Temperature: %d", m_payload.average_temperature);
  MW_LOG_INFO(DATA_LOG_TAG"Low Temperature: %d", m_payload.low_temperature);
  MW_LOG_INFO(DATA_LOG_TAG"High Temperature: %d", m_payload.high_temperature);
  MW_LOG_INFO(DATA_LOG_TAG"Pressure Counts: %d", m_payload.pressure_counts);
  MW_LOG_INFO(DATA_LOG_TAG"*********************************");
}


/**
 * @brief - Prep Argos Passload to send
 */
static void mw_data_check_device_orientation()
{
  //TODO check pass Prediction first

  /* Check device oritentation */
  m_orientation_get = true;
  mw_sensor_check_orientation(NORMAL_CONTEXT);
}


/**
 * @brief - Assemble ARGOS Payload for transmission
 */
//@payload @send @update
void update_payload()
{
  static uint8_t activity_index = 0;

  /* Skip while transmitting data */
  if(m_sending_payload) return;

  mw_rtc_get_time(&m_system_time);
  MW_LOG_INFO(DATA_LOG_TAG"******RTC Alarmed******");
  MW_LOG_INFO(DATA_LOG_TAG"Current Time is: %d : %d : %d", m_system_time.hour, m_system_time.minute, m_system_time.second);
  MW_LOG_INFO(DATA_LOG_TAG"Updating Payload with measure event: %d out of %d", activity_index, PROFILE_DATA_GATHER_INDEX_MAX );
  /**********************************************************/
  /* Grab Activity counts */
  m_payload.wwf_activity_payload[activity_index] = mw_sensor_get_activity_counter();
  activity_index++;

  /* Reset Actigraphy Count */
  mw_sensor_reset_activity_counter();
  /**********************************************************/

  /* If this concludes the data gathering window.  Send Payload*/
  if(activity_index == PROFILE_DATA_GATHER_INDEX_MAX)
  {
    /**********************************************************/
    /* Grab Temperature Measurements */
    m_payload.high_temperature = mw_sensor_get_temperature_hi();
    m_payload.low_temperature = mw_sensor_get_temperature_lo();
    m_payload.average_temperature = mw_sensor_get_temperature_avg();

    /* Reset Temperature Measurements */
    mw_sensor_reset_temperature();
    /**********************************************************/

    /**********************************************************/
    /* Get Battery Level */
    float current_temp = mw_sensor_get_temperature_current();

    m_payload.battery_level = adc_get_battery_level(current_temp);
    /**********************************************************/

    MW_LOG_INFO(DATA_LOG_TAG"Checking Device Orientation...");
    mw_data_check_device_orientation();
    activity_index = 0;
  }

  m_rtc_alarming = false;
}


void clear_payload_struct()
{
  m_payload.battery_level = 0;
  m_payload.average_temperature = 0;
  m_payload.high_temperature = 0;
  m_payload.low_temperature = 0;
  m_payload.pressure_counts = 0;

  memset( m_payload.wwf_activity_payload, 0, sizeof(m_payload.wwf_activity_payload));
}


static void check_dfu_trigger()
{
  if ( m_dfu_mode )
  {
    /* Sets Flags for DFU Triggering */
    uint32_t reg_val;
    sd_power_gpregret_get(0, &reg_val);
    reg_val = reg_val | UPDATE_DEVICE_FW; /* Set bit 1 to triger DFU */

    vTaskDelay(10);
    sd_power_gpregret_set(0, reg_val);

    /* Reset Device */
    sd_nvic_SystemReset();
  }
}

void configure_system()
{
  memset( &m_payload, 0, sizeof(m_payload));
  memset(m_artic_send_buffer, 0, sizeof(m_artic_send_buffer));

  /* Set Orientation */
  mw_sensor_set_orientation_threshold(1200);

  /* Set Accel Threshold */
  //TODO

  /* Get Bear ID + Profile*/
  m_payload.bear_id_and_profile = DEFAULT_BEAR_ID_AND_PROFILE;
  mw_flash_thread_load_bear_id_and_profile( &m_payload.bear_id_and_profile );

  /* Set Time and Alarm */
  //TODO
}


/**
 * @brief - Set all Function Handlers for external modules
 */
void initialize_system_handlers()
{
  /* Configure Temperature Handler */
  mw_temperature_set_external_handler( m_temperature_handler );

  /* Configure Orientation Check Handler */
  mw_sensor_set_external_handler( m_orientation_check_handler );

  /* Configure RTC Alarm Handler */
  mw_rtc_set_external_handler( m_rtc_alarm_handler );

  /* Configure WWF Service Handler */
  wwf_set_external_command_handler_t( wwf_service_handler );
}


/**
 * @brief - Set PA gain
 */
void mw_data_manager_update_pa_gain( uint8_t gain )
{
  switch(gain)
  {
    case 5:
    case 16:
    case 23:
    case 29:
    break;

    /* Skip non rfpa0133_gain_modes values */
    default:
    return;
    break;
  }

  mw_leds_command_event( BLINKS_LED, ISR_CONTEXT );
  MW_LOG_INFO(DATA_LOG_TAG "Updating PA Gain to: %ddBm", gain);


#if PA_TEST_MODE_FOR_JOHN
  m_update_pa_gain = true;
  m_pa_gain_update_value = (rfpa0133_gain_modes) gain;
#endif
}



//@resume
static void MW_RESUME_DATA_MANAGER_THREAD( bool isr)
{
  if ( isr )
  {
    if ( m_mw_data_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_data_thread_mode = THREAD_ACTIVE;
      xTaskResumeFromISR(m_mw_data_thread); // Resume myself
    }
  }
  else
  {
    if ( m_mw_data_thread_mode == THREAD_SUSPENDED )
    {
      m_mw_data_thread_mode = THREAD_ACTIVE;
      vTaskResume(m_mw_data_thread); // Resume myself
    }
  }
}



//@suspend
static void MW_SUSPEND_DATA_MANAGER_THREAD()
{
    m_mw_data_thread_mode = THREAD_SUSPENDED;
    vTaskSuspend(m_mw_data_thread); // Suspend myself
}




/**
 * @brief - Thread
 */
static void mw_data_manager_task( void * arg )
{
#if ARTIC_THREAD_ENABLED
  uint8_t  ble_reply;
#endif

  while( !( mw_sensor_thread_ready() && mw_flash_thread_ready() && artic_thread_ready()  ) )
  {
    vTaskDelay(2000);
  }

  memset( &m_payload, 0, sizeof(m_payload) );

  initialize_system_handlers();

#if ARTIC_THREAD_ENABLED

  configure_system();

  MW_LOG_INFO(DATA_LOG_TAG "MW Data Manager Thread Ready");



#if PA_TEST_MODE_FOR_JOHN
  m_mw_data_thread_mode = THREAD_INITIALIZED;
  mw_set_artic_3V3_rail( OFF );
  mw_set_1V8_rail( OFF );
  artic_power_on_pa();
  rfpa0133_enable_set_gain(TWENTY_NINE_DB_MODE);  /**< defaults to 29dB */

  while(1)
  {
    if(m_update_pa_gain)
    {
      rfpa0133_enable_set_gain(m_pa_gain_update_value);
      m_update_pa_gain = false;
    }

    MW_LOG_INFO(DATA_LOG_TAG "Device in Test mode.  PA enabled at gain: %ddBm", m_pa_gain_update_value);
    vTaskDelay(1000);
  }
#endif


  /********************************************/
  /* Test Subsystems */
  test_data_compression_decompression();
  test_pass_prediction();
  test_artic_power_up_program_power_down();
  artic_power_down();
  vTaskDelay(2000);
  /********************************************/

  /* Clear Global buffers */
  configure_system();
#endif

  m_mw_data_thread_mode = THREAD_INITIALIZED;

  while( ! mw_rtc_thread_ready() )
  {
    vTaskDelay(500);
  }

  set_rtc_default_time_and_alarm();

  while(1)
  {
    /**************************************************/
    /* Update Device FW - Go to DFU mode */
    check_dfu_trigger();

    /**************************************************/

#if ARTIC_THREAD_ENABLED
    /* Handler RTC Alarm */
    if(m_rtc_alarming)
    {
      update_payload();
    }

    /**************************************************/
    /* Get current time */
    if(m_rtc_time_get)
    {
      mw_rtc_get_time(&m_system_time);
      MW_LOG_INFO(DATA_LOG_TAG "RTC Time");
      MW_LOG_INFO(DATA_LOG_TAG "Current Time is: %d : %d : %d", m_system_time.hour, m_system_time.minute, m_system_time.second);
      m_rtc_alarming = false;
    }

    /**************************************************/
    /* Send Device summary */
    if(m_send_device_summary)
    {
      send_device_summary();
      m_send_device_summary = false;
    }

    /**************************************************/
    /* Temperature Alarm */
    if(m_temperature_alarm)
    {
      //TODO
      //Go to low power state
    }


    /**************************************************/
    /* Sending Payload */
    if(m_sending_payload)
    {
#if BYPASS_ARGOS_TRANSMISSIONS
      //TODO remove
      m_sending_payload = false;
      m_artic_send_repeats = m_artic_send_repeats_limit * 1;
      return;
#endif

      if(m_artic_send_repeats == 0 )
      {
        log_payload();

        //Compress payload
        compress_payload(&m_payload, m_artic_send_buffer);
      }

      if(m_artic_send_repeats < m_artic_send_repeats_limit)
      {
        if(m_artic_send_repeats == 0 )
        {
          /* Turn ON LED */
          mw_leds_command_event( HEARTBEAT_2_LED, NORMAL_CONTEXT );

          /* BLE Send */
          ble_reply = ARTIC_POWERING_ON_AND_PROGRAMMING;
          wwf_send_data(&ble_reply, sizeof(ble_reply));

          artic_power_on_and_program();

          /* BLE Send */
          ble_reply = ARTIC_POWERING_ON_AND_PROGRAMMING_SUCCESS;
          wwf_send_data(&ble_reply, sizeof(ble_reply));


          artic_read_firmware_version();
          /* BLE Send */
          ble_reply = ARTIC_FIRMWARE_CONFIRMED;
          wwf_send_data(&ble_reply, sizeof(ble_reply));

        }

        /* Turn ON LED */
        artic_power_on_pa();

        /* Wait for Voltage Rail to Stabilize */
        vTaskDelay(250);

        /* Turn ON LED */
        mw_leds_command_event( HEARTBEAT_LED, NORMAL_CONTEXT );

        MW_LOG_INFO(DATA_LOG_TAG "*****************************");
        MW_LOG_INFO(DATA_LOG_TAG "Sending Payload index: %d", m_artic_send_repeats);
        MW_LOG_INFO(DATA_LOG_TAG "*****************************");

        /* ARGOS Send*/
        artic_transmit_argos2a_1_pkt(m_artic_send_buffer, ARGOS2_MESSAGE_280_BITS);

        /* Increment Transmission Counter */
        m_artic_send_repeats++;

        MW_LOG_INFO(DATA_LOG_TAG "*****************************");
        MW_LOG_INFO(DATA_LOG_TAG "Payload Successfully Sent.  Next Transmission in %dseconds", m_artic_send_interval/1000);
        MW_LOG_INFO(DATA_LOG_TAG "*****************************");

        /* Turn OFF LEDs */
        artic_power_off_pa();

        /* Turn OFF LED */
        mw_leds_command_event( COMMAND_IDLE_LEDS, NORMAL_CONTEXT );

        /* BLE Send */
        wwf_send_data(m_artic_send_buffer, sizeof(m_artic_send_buffer));

        vTaskDelay(m_artic_send_interval);

        /* Check DFU */
        check_dfu_trigger();
      }

      if(m_artic_send_repeats >= m_artic_send_repeats_limit)
      {
        MW_LOG_INFO(DATA_LOG_TAG "*****************************");
        MW_LOG_INFO(DATA_LOG_TAG "*****************************");
        MW_LOG_INFO(DATA_LOG_TAG "Completed Sending %d Transmissions", m_artic_send_repeats);
        MW_LOG_INFO(DATA_LOG_TAG "*****************************");
        MW_LOG_INFO(DATA_LOG_TAG "*****************************");

        /* BLE Send Transmission Done Message */
        memset(m_artic_send_buffer, 0, sizeof(m_artic_send_buffer));
        memcpy( m_artic_send_buffer, (uint8_t*)BLE_MSG_TX_DONE, sizeof(BLE_MSG_TX_DONE));
        wwf_send_data(m_artic_send_buffer, sizeof(m_artic_send_buffer));

        /* Reset Variables */
        m_artic_send_repeats = 0;
        clear_payload_struct();
        memset(m_artic_send_buffer, 0, sizeof(m_artic_send_buffer));
        artic_power_down();
        artic_power_off_pa();
        m_sending_payload = false;
      }
    }

    /**************************************************/
    /* Start Device Tracking */
    if(m_start_tracking)
    {
      MW_LOG_INFO(DATA_LOG_TAG "*****************************");
      MW_LOG_INFO(DATA_LOG_TAG " Starting Animal Tracking");
      MW_LOG_INFO(DATA_LOG_TAG "Reset Tracking Variables");
      reset_system_variables();
      MW_LOG_INFO(DATA_LOG_TAG "Setting alarm for %d in units of %d from now..", m_tracking_cycle_time, m_tracking_cycle_units);
      mw_rtc_auto_set_alarm( m_tracking_cycle_time, m_tracking_cycle_units, NORMAL_CONTEXT);
      MW_LOG_INFO(DATA_LOG_TAG "*****************************");
      m_start_tracking = false;
    }
    /**************************************************/
#endif

     MW_SUSPEND_DATA_MANAGER_THREAD();
  }
}



/**
 * @brief Returns ture when Thread initialized and running
 */
bool  mw_data_manager_thread_ready( void )
{
#if DATA_MANAGER_THREAD_ENABLED
  return (m_mw_data_thread_mode != THREAD_NULL);
#else
  return true;
#endif
}

/**
 * @brief Function for application main entry.
 */
void mw_data_manager_thread_init( void )
{
  data_semph = xSemaphoreCreateBinary();
  if ( NULL == data_semph )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  xSemaphoreGive(data_semph);

  if ( xTaskCreate(mw_data_manager_task, "DATA", DATA_MANAGER_THREAD_STACK_SIZE, NULL, DATA_MANAGER_THREAD_PRIORITY, &m_mw_data_thread) != pdPASS )
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

}
