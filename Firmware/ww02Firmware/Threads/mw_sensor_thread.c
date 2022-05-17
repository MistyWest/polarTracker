/*
 * mw_sensor_thread.c
 *
 *  Created on: Jan 22, 2019
 *      Author: klockwood
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrfx_gpiote.h"
#include "boards.h"
#include "nrfx_timer.h"
#include "nrfx_clock.h"
#include "nrf_soc.h"
#include "boards.h"
#include "FreeRTOS_includes.h"

#include "mw_thread.h"
#include "mw_sensor_thread.h"

#include "project_settings.h"
#include "wwf_device_settings.h"
#include "mw_spi_master.h"
#include "mw_twi_master.h"
#include "mw_logging.h"
#include "_mw_external_device.h"
#include "mw_power_management.h"
#include "ISM303DACTR.h"
#include "LPS33HWTR.h"

#define SENSOR_LOG_TAG							    "Sensor Thread: "

#define SENSOR_POLLING_ENABLED          0

#define SENSOR_LOG_LEVEL_DEBUG          1

#define SENSOR_FIFO_THRESHOLD           101

#define SENSOR_Z_AXIS_THRESHOLD         (-1800)
#define SENSOR_Y_AXIS_THRESHOLD         101

#ifndef SENSOR_TWI_INSTANCE
#define SENSOR_TWI_INSTANCE             0
#endif

#define PIN_SENSOR_SCL                  TWI1_SCL_PIN
#define PIN_SENSOR_SDA                  TWI1_SDA_PIN

#define DEFAULT_SENSOR_ODR              ACC_ODR_12_5HZ //ACC_ODR_12_5HZ // ACC_ODR_OFF  ACC_ODR_1HZ   ACC_ODR_12_5HZ

#define DEFAULT_LOW_TEMP                (20)
#define DEFAULT_HIGH_TEMP               (-20)
#define DEFAULT_AVG_TEMP                0

#ifndef TEMPERATURE_TIME_INTERVAL
#define TEMPERATURE_TIME_INTERVAL       20000
#endif

SemaphoreHandle_t 									    sensor_semph;

TaskHandle_t 												    m_mw_sensor_thread;        /**< Definition of Thread. */

static volatile mw_thread_mode_t        m_mw_sensor_thread_mode;

static TimerHandle_t                    m_temperature_timer;

/******************************************/
/* Global Variables */
static uint16_t                         m_actigraphy_count = 0;
static float                            m_activity_threshold = 1.25;
static float                            m_temperature_avg = 0;
static float                            m_temperature_hi = -20;
static float                            m_temperature_lo = 20;
static float                            m_temperature_current = 0;
static uint32_t                         m_temperature_samples = 0;
static device_orientation_t             m_orientation = UNKNOWN;
static mw_sensor_orientation_check_external_handler_t m_sensor_external_handler = NULL;

static ism303dac_accel_out_t            m_sensor_orientation_buffer[SENSOR_FIFO_THRESHOLD];
static ism303dac_accel_out_t            m_sensor_orientation_average;
static int16_t                          m_sensor_orientation_threshold = SENSOR_Z_AXIS_THRESHOLD;
/******************************************/
/* State Control Flags */
static volatile bool m_check_interrupt_flag = false;
static volatile bool m_update_threshold_flag = false;
static volatile bool m_check_orientation_flag = false;
static volatile bool m_calculating_orientation_flag = false;
static volatile bool m_measure_temperature_flag = false;
/******************************************/

static void sensor_convert_and_log_acc_date( ism303dac_accel_out_t accel_raw);
static void check_ISM303DACTR_orientation_enable();
static void check_ISM303DACTR_orientation_disable();

static void MW_RESUME_SENSOR_THREAD( bool isr );
static void MW_SUSPEND_SENSOR_THREAD();


/********************************************************************************************
 *
 *  Handler Function(s) *
 *
 *******************************************************************************************/
/**
 * @brief - IRQ Handler for Sensor Events
 */
static void sensor_event_handler( mw_twim_evt_t const * twim_event )
{

}


/**
 * @brief - IRQ Handler for Interrupt pins
 */
static void ism303dactr_irq(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if( pin == PIN_ACC_INT_XL )
  {
    MW_LOG_INFO(SENSOR_LOG_TAG "PIN_ACC_INT_XL hit");
    m_check_interrupt_flag = true;
    m_actigraphy_count++;
    MW_RESUME_SENSOR_THREAD(ISR_CONTEXT);
  }
}


/**
 * @brief - Turn OFF Led which is currently solid ON
 */
static void temperature_timeout_handler(TimerHandle_t xTimer)
{
  m_measure_temperature_flag = true;
  MW_RESUME_SENSOR_THREAD(ISR_CONTEXT);
}

/********************************************************************************************
 *
 *  External Access Functions *
 *
 *******************************************************************************************/
/**
 * @brief - Check Orientation for viable transmission
 */
device_orientation_t mw_sensor_check_orientation( bool isr )
{
#if SENSOR_THREAD_ENABLED
  if( m_orientation != CHECKING_ORIENTATION )
  {
    m_orientation = CHECKING_ORIENTATION;
    m_check_orientation_flag = true;
    MW_RESUME_SENSOR_THREAD(isr);
  }
  return m_orientation;
#else
  return 1;
#endif
}


/**
 * @brief - Set orientation threshold
 */
void mw_sensor_set_orientation_threshold( int16_t threshold )
{
  m_sensor_orientation_threshold = threshold;
}


/**
 * @brief - Set Handler signal Orientation Check complete
 */
void mw_sensor_set_external_handler( mw_sensor_orientation_check_external_handler_t external_handler )
{
#if SENSOR_THREAD_ENABLED
  if ( external_handler != NULL )
  {
    m_sensor_external_handler = external_handler;
  }
#else
  external_handler = NULL;
#endif
}



/**
 * @brief - Get Temperature Average
 */
float mw_sensor_get_temperature_avg(void)
{
  return m_temperature_avg;
}

/**
 * @brief - Get Temperature High
 */
float mw_sensor_get_temperature_hi(void)
{
  return m_temperature_hi;
}

/**
 * @brief - Get Temperature Low
 */
float mw_sensor_get_temperature_lo(void)
{
  return m_temperature_lo;
}

/**
 * @brief - Get Current Temperature
 */
float mw_sensor_get_temperature_current(void)
{
  return m_temperature_current;
}


/**
 * @brief - Reset Temperature Variables
 */
void mw_sensor_reset_temperature(void)
{
  m_temperature_lo = DEFAULT_LOW_TEMP;
  m_temperature_hi = DEFAULT_HIGH_TEMP;
  m_temperature_avg = DEFAULT_AVG_TEMP;
  m_temperature_samples = 0;
}

/**
 * @brief - Reset Activity Counter
 */
uint16_t mw_sensor_get_activity_counter(void)
{
  return m_actigraphy_count;
}


/**
 * @brief - Reset Activity Counter
 */
void mw_sensor_reset_activity_counter(void)
{
  m_actigraphy_count = 0;
}




/**
 * @brief - Update actigraphy threshhold setting - Float
 *
 * @param - threshold - float value in G's
 */
void mw_sensor_update_activity_threshold( float threshold, bool isr )
{
#if SENSOR_THREAD_ENABLED
  m_update_threshold_flag = true;
  m_activity_threshold = threshold;
  MW_RESUME_SENSOR_THREAD(isr);
#endif
}



/**
 * @brief - Reset Activity Counter
 */
float mw_sensor_get_actigraphy_threshold()
{
  return m_activity_threshold;
}

/********************************************************************************************
 *
 *  Internal Function *
 *
 *******************************************************************************************/
/**
 * @brief - Handler Function for interrupts
 */
static void sensor_interrupt_handler()
{
  uint8_t interrupt_flag, fifo_size;
  int32_t summed_x_axis = 0;
  int32_t summed_y_axis = 0;
  int32_t summed_z_axis = 0;

  ism303dactr_register_read( ISM303DAC_STATUS_A, &interrupt_flag, 1);

  /* If FIFO Threshold Interrupt */
  if( !m_calculating_orientation_flag && ( interrupt_flag >> 7) )
  {
    m_calculating_orientation_flag = true;

    MW_LOG_INFO(SENSOR_LOG_TAG "Orientation Fifo Full Event");
    m_actigraphy_count--; //Decrement as this wasn't a Wake-Up Event

    /* Check FIFO Count */
    ism303dactr_register_read( ISM303DAC_FIFO_SAMPLES_A, &fifo_size, 1);


    for( uint8_t i=0; i < fifo_size; i++)
    {
      m_sensor_orientation_buffer[i] = ism303dactr_read_acc();
    }


    /* Skip first entry.  Iteratively sum remaining buffer */
    for( uint8_t i=1; i < fifo_size; i++ )
    {
      summed_x_axis +=  m_sensor_orientation_buffer[i].x_axis;
      summed_y_axis +=  m_sensor_orientation_buffer[i].y_axis;
      summed_z_axis +=  m_sensor_orientation_buffer[i].z_axis;
    }

    m_sensor_orientation_average.x_axis = summed_x_axis / (fifo_size - 1);
    m_sensor_orientation_average.y_axis = summed_y_axis / (fifo_size - 1);
    m_sensor_orientation_average.z_axis = summed_z_axis / (fifo_size - 1);

    if( m_sensor_orientation_average.z_axis <= SENSOR_Z_AXIS_THRESHOLD)
    {
      m_orientation = BAD_ORIENTATION;
    }
    else
    {
      m_orientation = GOOD_ORIENTATION;
    }


    sensor_convert_and_log_acc_date(m_sensor_orientation_average);

    /* Disable FIFO */
    check_ISM303DACTR_orientation_disable();

    m_sensor_external_handler(m_orientation, NORMAL_CONTEXT);

    m_calculating_orientation_flag = false;

    return;
  }

  /* If Wake Up Threshold Interrupt */
  if( interrupt_flag >> 6)
  {
    ism303dactr_register_read( ISM303DAC_WAKE_UP_SRC_A, &interrupt_flag, 1);
    MW_LOG_INFO(SENSOR_LOG_TAG "Activity Threshold Wake up event occured");
  }

}

/**
 * @brief - Polls and Averages values to establish orientation
 */
static void check_ISM303DACTR_orientation_enable()
{
  /* Set Accel ODR and Range */
  ism303dac_ctrl1_a_t ctrl1_config;
  ctrl1_config.fs = ACC_RANGE_16G;
  ctrl1_config.odr = ACC_ODR_25HZ; //ACC_ODR_1HZ;
  ctrl1_config.bdu = BDR_ENABLED;
  ism303dactr_register_write( ISM303DAC_CTRL1_A, *(uint8_t*)&ctrl1_config);

  /* Enable FIFO */
  ism303dac_fifo_ctrl_a_t fifo_ctrl_config;
  fifo_ctrl_config.fmode = FIFO_MODE_ENABLED;
  fifo_ctrl_config.module_to_fifo = DISABLED;
  fifo_ctrl_config.if_cs_pu_dis = DISABLED;
  ism303dactr_register_write( ISM303DAC_FIFO_CTRL_A, *(uint8_t*)&fifo_ctrl_config);

  /* FIFO Threshold - 100 samples */
  ism303dactr_register_write( ISM303DAC_FIFO_THS_A, SENSOR_FIFO_THRESHOLD);


  /* Disable Sleep bit */
  ism303dactr_set_sleep_bit(0);


  /* Enable FIFO Threshold Interrupt */
  ism303dactr_register_write( ISM303DAC_CTRL4_A, ENABLE_WAKE_UP_INT1_BIT | ENABLE_FIFO_INT1_BIT );
}



/**
 * @brief - Polls and Averages values to establish orientation
 */
static void check_ISM303DACTR_orientation_disable()
{
  /* Set Accel ODR and Range */
  ism303dac_ctrl1_a_t ctrl1_config;
  ctrl1_config.fs = ACC_RANGE_16G;
  ctrl1_config.odr = ACC_ODR_12_5HZ; //ACC_ODR_1HZ;
  ctrl1_config.bdu = BDR_ENABLED;
  ism303dactr_register_write( ISM303DAC_CTRL1_A, *(uint8_t*)&ctrl1_config);

  /* Disable FIFO */
  ism303dac_fifo_ctrl_a_t fifo_ctrl_config;
  fifo_ctrl_config.fmode = FIFO_MODE_DISABLED;
  fifo_ctrl_config.module_to_fifo = DISABLED;
  fifo_ctrl_config.if_cs_pu_dis = DISABLED;
  ism303dactr_register_write( ISM303DAC_FIFO_CTRL_A, *(uint8_t*)&fifo_ctrl_config);

  /* FIFO Threshold - 100 samples */
  ism303dactr_register_write( ISM303DAC_FIFO_THS_A, 100);

  /* Disable FIFO Threshold Interrupt */
  ism303dactr_register_write( ISM303DAC_CTRL4_A, ENABLE_WAKE_UP_INT1_BIT );

  /* Enable Sleep Bit */
  ism303dactr_set_sleep_bit(1);
}


/**
 * @brief - Threshold look-up table
 */
float mw_sensor_threshold_look_up_table ( uint8_t index )
{
  switch ( index )
  {
  case THRESHOLD_0_90:
    return 0.9;
    break;
  case THRESHOLD_0_95:
    return 0.95;
    break;
  case THRESHOLD_1_00:
    return 1.00;
    break;
  case THRESHOLD_1_05:
    return 1.05;
    break;
  case THRESHOLD_1_15:
    return 1.15;
    break;
  case THRESHOLD_1_25:
    return 1.25;
    break;
  case THRESHOLD_1_35:
    return 1.35;
    break;
  case THRESHOLD_1_45:
    return 1.45;
    break;
  case THRESHOLD_1_55:
    return 1.55;
    break;
  case THRESHOLD_1_65:
    return 1.65;
    break;
  case THRESHOLD_1_75:
    return 1.75;
    break;
  case THRESHOLD_1_85:
    return 1.85;
    break;
  case THRESHOLD_1_95:
    return 1.95;
    break;
  case THRESHOLD_2_05:
    return 2.05;
    break;
  case THRESHOLD_2_15:
    return 2.15;
    break;
  case THRESHOLD_2_25:
    return 2.25;
    break;
  case THRESHOLD_2_50:
    return 2.50;
    break;
  case THRESHOLD_2_75:
    return 2.75;
    break;
  case THRESHOLD_2_95:
    return 2.95;
    break;
  case THRESHOLD_3_15:
    return 3.15;
    break;
  case THRESHOLD_3_50:
    return 3.5;
    break;
  default:
    return 1.27;
    break;
  }
}


/**
 * @brief - Set Actigrapy threshold
 */
static void set_ISM303DACTR_wake_up_threshold()
{
  uint8_t reg_value;
  float scaling_value = 0;
  uint8_t scaled_threshold;

  switch( ism303dactr_get_acc_range() )
  {
  case ACC_RANGE_2G:
    scaling_value = 2.0f/64.0f;
    break;

  case ACC_RANGE_4G:
    scaling_value = 4.0f/64.0f;
    break;

  case ACC_RANGE_8G:
    scaling_value = 4.0f/64.0f;
    break;

  case ACC_RANGE_16G:
    scaling_value = 16.0f/64.0f;
    break;
  }

  scaled_threshold = ( m_activity_threshold / scaling_value );
  scaled_threshold &= 0x3F;  /*Assure to mask all but 6 LSB bits*/

  /* Get current register contents */
  ism303dactr_register_read( ISM303DAC_WAKE_UP_THS_A, &reg_value, 1 );
  reg_value &= 0xC0;

  /* Enable Sleep, Set Accel Wake up Threshold = 0x07 */
  ism303dactr_register_write( ISM303DAC_WAKE_UP_THS_A, reg_value | scaled_threshold );
}


/**
 * @brief - Measure Temperature
 */
static void mw_sensor_measure_temperature()
{
  m_temperature_current = ism303dactr_reading_temperature();

  /* Check if current sample exceeds limits */
  if( m_temperature_current < m_temperature_lo ) m_temperature_lo = m_temperature_current;
  if( m_temperature_current > m_temperature_hi ) m_temperature_hi = m_temperature_current;

  /* Added to running average */
//  m_temperature_avg =  (m_temperature_avg * m_temperature_samples) + m_temperature_current;
//
//  m_temperature_samples++;
//
//  m_temperature_avg /= m_temperature_samples;

  MW_LOG_INFO(SENSOR_LOG_TAG "Temperature Measured: " MW_FLOAT_VALUE "degC", MW_LOG_FLOAT(m_temperature_current));
}

/**
 * @brief - Main Intitial Configuration of the LPS33HWTR Registres
 */
//@config
static void configure_LPS33HWTR_registers()
{
  lps33hwtr_who_am_i_test();
  lps33hwtr_set_odr(LPS33HW_ODR_OFF);
  lps33hwtr_enable_low_power_mode(true);
  lps33hwtr_enable_i2c(true);
}


/**
 * @brief - Main Intitial Configuration of the ISM303 Registres
 */
//@config
static void configure_ISM303DACTR_registers()
{
  uint8_t read_buffer[2];

  ism303dactr_reset();
//  ism303dac_ctrl2_a_t ctrl2_reg;
//  ctrl2_reg.soft_reset = 1;
//  ism303dactr_register_write( ISM303DAC_CTRL2_A, *(uint8_t*)&ctrl2_reg);
  while(ism303dactr_reset_in_progress())
  {
    vTaskDelay(100);
  }

  /* Set Accel ODR and Range */
  ism303dac_ctrl1_a_t ctrl1_config;
  ctrl1_config.fs = ACC_RANGE_8G;
  ctrl1_config.odr = DEFAULT_SENSOR_ODR;
  ctrl1_config.bdu = ENABLED;
  ctrl1_config.hf_odr = DISABLED;
  ism303dactr_register_write( ISM303DAC_CTRL1_A, *(uint8_t*)&ctrl1_config);

  /* Read ISM303DAC_CTRL1_A settings */
  ism303dactr_register_read( ISM303DAC_CTRL1_A, read_buffer, 2);

  /* Disable FIFO */
  ism303dac_fifo_ctrl_a_t fifo_ctrl_config;
  fifo_ctrl_config.fmode = FIFO_MODE_DISABLED;
  fifo_ctrl_config.module_to_fifo = DISABLED;
  fifo_ctrl_config.if_cs_pu_dis = DISABLED;
  ism303dactr_register_write( ISM303DAC_FIFO_CTRL_A, *(uint8_t*)&fifo_ctrl_config);

  /* Set Latching Interrupts */
  ism303dactr_register_write( ISM303DAC_CTRL2_A, ENABLE_AUTO_ADDR_INC_BIT );

  /* Set Latching Interrupts */
  //ism303dactr_register_write( ISM303DAC_CTRL3_A, ENABLE_LATCH_INT_BIT );

  /* Enable Sleep bit */
  ism303dactr_register_write( ISM303DAC_WAKE_UP_THS_A, ENABLE_SLEEP_BIT );

  /* Set Accel Wake up Threshold = 0x07 */
  set_ISM303DACTR_wake_up_threshold();

  /* Set Threshold Duration */
  ism303dactr_register_write( ISM303DAC_WAKE_UP_DUR_A, WAKE_UP_DURATION(WAKE_UP_1x_ODR) | SLEEP_DURATION(0) );


  /* Disable Magetometer */
  ism303dactr_register_write( ISM303DAC_CFG_REG_A_M, MAG_LP_POWER_MODE | MAG_SYS_MODE_IDLE );

  /* Read ISM303DAC_CFG_REG_A_M settings */
  ism303dactr_register_read( ISM303DAC_CFG_REG_A_M, read_buffer, 2);


  /* Enable Wakeup Interrupt 1 */
  ism303dactr_register_write( ISM303DAC_CTRL4_A, ENABLE_WAKE_UP_INT1_BIT );
}


static void setup_LPS33HWTR_interface()
{
  external_device_config_t device_config;

  device_config.communication                 = TWI_COMMUNICATION;
  device_config.twi_config.instance           = SENSOR_TWI_INSTANCE;
  device_config.twi_config.slave_addr         = LPS33HW_I2C_ADDR;
  device_config.twi_config.hold_bus_uninit    = false;
  device_config.twi_config.frequency          = NRF_TWI_FREQ_400K;
  device_config.twi_config.scl                = PIN_SENSOR_SCL;
  device_config.twi_config.sda                = PIN_SENSOR_SDA;
  device_config.twi_config.mw_twim_handler    = sensor_event_handler;
  device_config.twi_config.context            = NULL;
  device_config.twi_config.interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY;

  lps33hwtr_init( device_config );
}


static void setup_ISM303DACTR_interface()
{
  external_device_config_t device_config_xl, device_config_mag;

  device_config_xl.communication                 = TWI_COMMUNICATION;
  device_config_xl.twi_config.instance           = SENSOR_TWI_INSTANCE;
  device_config_xl.twi_config.slave_addr         = ISM303DACTR_I2C_ACC_ADDR;
  device_config_xl.twi_config.hold_bus_uninit    = false;
  device_config_xl.twi_config.frequency          = NRF_TWI_FREQ_400K;
  device_config_xl.twi_config.scl                = PIN_SENSOR_SCL;
  device_config_xl.twi_config.sda                = PIN_SENSOR_SDA;
  device_config_xl.twi_config.mw_twim_handler    = sensor_event_handler;
  device_config_xl.twi_config.context            = NULL;
  device_config_xl.twi_config.interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY;

  device_config_mag.communication                 = TWI_COMMUNICATION;
  device_config_mag.twi_config.instance           = SENSOR_TWI_INSTANCE;
  device_config_mag.twi_config.slave_addr         = ISM303DACTR_I2C_MAG_ADDR;
  device_config_mag.twi_config.hold_bus_uninit    = false;
  device_config_mag.twi_config.frequency          = NRF_TWI_FREQ_400K;
  device_config_mag.twi_config.scl                = PIN_SENSOR_SCL;
  device_config_mag.twi_config.sda                = PIN_SENSOR_SDA;
  device_config_mag.twi_config.mw_twim_handler    = sensor_event_handler;
  device_config_mag.twi_config.context            = NULL;
  device_config_mag.twi_config.interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY;

  ism303dactr_init ( device_config_xl, device_config_mag );
}


static void start_temperature_measurement_timer()
{
  if (pdPASS != xTimerStart(m_temperature_timer, OSTIMER_WAIT_FOR_QUEUE))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

static void enable_sensor_interrupt()
{
  nrfx_gpiote_in_event_enable(PIN_ACC_INT_XL, true);
}


static void disable_sensor_interrupt()
{
  nrfx_gpiote_in_event_disable(PIN_ACC_INT_XL);
}


/**
 * @brief - Setup pin interrupt via GPIOTE.  Module assumed to be initialized in main.c
 */
static void sensor_configure_interrupts()
{
  uint32_t err_code;

  /* Verify GPIOTE was already initialized */
  if(!nrfx_gpiote_is_init())  nrfx_gpiote_init();

  nrf_gpio_cfg_input(PIN_ACC_INT_XL, NRF_GPIO_PIN_NOPULL);

  nrfx_gpiote_in_config_t int_pin_cfg = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(false);

  err_code = nrfx_gpiote_in_init( PIN_ACC_INT_XL,
                                  &int_pin_cfg,
                                  ism303dactr_irq );
  APP_ERROR_CHECK(err_code);
}


/**
 * @brief - Convert Accelerometer Data and Log
 */
static void sensor_convert_and_log_acc_date( ism303dac_accel_out_t accel_raw)
{
  ism303dac_accel_float_out_t accel_converted = ism303dactr_convert_raw_values_acc(&accel_raw);
  MW_LOG_INFO(SENSOR_LOG_TAG "Accel x-axis is: " MW_FLOAT_VALUE "g", MW_LOG_FLOAT(accel_converted.x_axis));
  MW_LOG_INFO(SENSOR_LOG_TAG "Accel y-axis is: " MW_FLOAT_VALUE "g", MW_LOG_FLOAT(accel_converted.y_axis));
  MW_LOG_INFO(SENSOR_LOG_TAG "Accel z-axis is: " MW_FLOAT_VALUE "g", MW_LOG_FLOAT(accel_converted.z_axis));
}


/**
 * @brief - Resume Thread
 *
 * @param - If calling function was from Interrupt Context
 */
//@resume
static void MW_RESUME_SENSOR_THREAD( bool isr )
{
  m_mw_sensor_thread_mode = THREAD_ACTIVE;
  if(isr)
  {
    xTaskResumeFromISR(m_mw_sensor_thread); // Resume myself
  }
  else
  {
    vTaskResume(m_mw_sensor_thread); // Resume myself
  }
}

/**
 * @brief - Suspend Thread
 */
//@suspend
static void MW_SUSPEND_SENSOR_THREAD()
{
  m_mw_sensor_thread_mode = THREAD_SUSPENDED;
  vTaskSuspend(m_mw_sensor_thread); // Suspend myself
}


/**
 * @brief Thread is ready
*/
bool mw_sensor_thread_ready(void)
{
#if !SENSOR_THREAD_ENABLED
  return true;
#endif
  return m_mw_sensor_thread_mode != THREAD_NULL;
}



/**
 * @brief - Sensor Thread Task
 */
static void mw_sensor_task(void * arg)
{
	vTaskDelay(MW_SENSOR_THREAD_START_UP_DELAY);

	m_mw_sensor_thread_mode = THREAD_INITIALIZED;

	/* Enabled voltage rail */
	mw_set_sensors_3V3_rail(ON);
	vTaskDelay(100);

	/* Setup GPIOTE Interrupt */
	enable_sensor_interrupt();

	/* Configure i2c interface */
	setup_ISM303DACTR_interface();
  setup_LPS33HWTR_interface();

  /* Verify Chip */
  ism303dactr_who_am_i_test_mag();
  ism303dactr_who_am_i_test_acc();

	/* Configure Registers */
	configure_ISM303DACTR_registers();
	configure_LPS33HWTR_registers();

	/* Start Temperature Measurement Timer */
  start_temperature_measurement_timer();


#if SENSOR_POLLING_ENABLED
  float temperature_read;
	ism303dac_accel_out_t accel_raw;
	ism303dac_accel_float_out_t accel_converted;
#endif

	while (true)
	{
    /* Check Interrupt Status */
    if(m_check_interrupt_flag)
    {
       sensor_interrupt_handler();
       m_check_interrupt_flag = false;
    }

    /* Check Interrupt Status */
    if(m_update_threshold_flag)
    {
      set_ISM303DACTR_wake_up_threshold();
      m_update_threshold_flag = false;
    }

    /* Check Device Orientation */
    if(m_check_orientation_flag)
    {
      check_ISM303DACTR_orientation_enable();
      m_check_orientation_flag = false;
    }

    /* Measure Temperature */
    if(m_measure_temperature_flag)
    {
      mw_sensor_measure_temperature();
      m_measure_temperature_flag = false;
    }


#if SENSOR_POLLING_ENABLED
    MW_LOG_INFO(SENSOR_LOG_TAG    MW_LOG_BORDER);
    temperature_read = ism303dactr_reading_temperature();
    MW_LOG_INFO(SENSOR_LOG_TAG "Temperature: " MW_FLOAT_VALUE "C", MW_LOG_FLOAT(temperature_read));


    accel_raw = ism303dactr_read_acc();
    sensor_convert_and_log_acc_date(accel_raw) ism303dac_accel_out_t
    accel_converted = ism303dactr_convert_raw_values_acc(&accel_raw);
    MW_LOG_INFO(SENSOR_LOG_TAG "Accel x-axis is: " MW_FLOAT_VALUE "g", MW_LOG_FLOAT(accel_converted.x_axis));
    MW_LOG_INFO(SENSOR_LOG_TAG "Accel y-axis is: " MW_FLOAT_VALUE "g", MW_LOG_FLOAT(accel_converted.y_axis));
    MW_LOG_INFO(SENSOR_LOG_TAG "Accel z-axis is: " MW_FLOAT_VALUE "g", MW_LOG_FLOAT(accel_converted.z_axis));
    vTaskDelay(SENSOR_THREAD_POLL_INTERVAL);
#else
    MW_SUSPEND_SENSOR_THREAD();
#endif
	}

	/* Keep Compiler Happy */
	MW_SUSPEND_SENSOR_THREAD();
}



/********************************************************************************************
 *
 *  Initialization Function
 *
 *******************************************************************************************/

/**
 * @brief Function for application main entry.
*/
void mw_sensor_thread_init(void)
{
  sensor_configure_interrupts();
  disable_sensor_interrupt();

	sensor_semph = xSemaphoreCreateBinary();
	if (NULL == sensor_semph)
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
	xSemaphoreGive(sensor_semph);


	m_temperature_timer = xTimerCreate("TEMP", TEMPERATURE_TIME_INTERVAL, pdTRUE /*pdFALSE*/, NULL, temperature_timeout_handler);
  if ((NULL == m_temperature_timer))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }


	if(xTaskCreate( mw_sensor_task, "SENSOR", SENSOR_THREAD_STACK_SIZE, NULL, SENSOR_THREAD_PRIORITY, &m_mw_sensor_thread ) != pdPASS)
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
}
