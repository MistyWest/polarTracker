/*
 * LSM6DS3.c
 *
 *  Created on: Mar 21, 2019
 *      Author: klockwood
 */


#include <nrf_delay.h>
#include "boards.h"
#include "LSM6DS3.h"
#include "../CLI_Logging/mw_logging.h"
#include "_mw_external_device.h"

#define LSM6DS3_LOG_TAG                  "LSM6DS3: "

#define LSM6DS3_TEST_UNITS_ENABLED       1

#ifndef IMU_INT_1_PIN
#define IMU_INT_1_PIN                    22
#endif

static bool m_lsm6ds3_initialized = false;

static external_driver_spi_config_t m_LSM6DS3_SPI_ID;

static external_device_config_t  LSM6DS3_driver_config;

static void LSM6DS3_get_accelerometer_resolution( float * accel_res )
{
  uint8_t reg;

  /* Read the actual gyroscope full scale setting */
  reg = LSM6DS3_get_accel_ctrl1();
  reg &= LSM6DS3_ACCEL_SCALE_MASK;

  /* Calculate the resolution */
  switch ( reg )
  {
  case LSM6DS2_ACCEL_SCALE(LSM6DS3_ACCEL_SCALE_2G):
  *accel_res = LSM6DS3_ACCEL_2G_SCALE_VALUE;
  break;

  case LSM6DS2_ACCEL_SCALE(LSM6DS3_ACCEL_SCALE_4G):
  *accel_res = LSM6DS3_ACCEL_4G_SCALE_VALUE;
  break;

  case LSM6DS2_ACCEL_SCALE(LSM6DS3_ACCEL_SCALE_8G):
  *accel_res = LSM6DS3_ACCEL_8G_SCALE_VALUE;
  break;

  case LSM6DS2_ACCEL_SCALE(LSM6DS3_ACCEL_SCALE_16G):
  *accel_res = LSM6DS3_ACCEL_16G_SCALE_VALUE;
  break;
  }
}

static void LSM6DS3_get_gyroscope_resolution( float * gyro_res )
{
  uint8_t reg;

  /* Read the actual gyroscope full scale setting */
  reg = LSM6DS3_get_gyro_ctrl2();
  reg &= LSM6DS3_GYRO_SCALE_MASK;

  /* Calculate the resolution */
  switch ( reg )
  {
  case LSM6DS2_GYRO_SCALE(LSM6DS3_GYRO_SCALE_250DPS):
  *gyro_res = (LSM6DS3_GYRO_250DPS_SCALE_VALUE);
  break;

  case LSM6DS2_GYRO_SCALE(LSM6DS3_GYRO_SCALE_500DPS):
  *gyro_res = (LSM6DS3_GYRO_500DPS_SCALE_VALUE);
  break;

  case LSM6DS2_GYRO_SCALE(LSM6DS3_GYRO_SCALE_1000DPS):
  *gyro_res = (LSM6DS3_GYRO_1000DPS_SCALE_VALUE);
  break;

  case LSM6DS2_GYRO_SCALE(LSM6DS3_GYRO_SCALE_2000DPS):
  *gyro_res = (LSM6DS3_GYRO_2000DPS_SCALE_VALUE);
  break;
  }
}



/**
 * @brief - Convert Raw Acceleormeter Data to G's
 */
void LSM6DS3_convert_accelerometer_to_g( int16_t axis, float * float_value )
{
  LSM6DS3_get_accelerometer_resolution(float_value); // get resolution value
  *float_value = (float)axis * *float_value;          // calculate float value
}


/**
 * @brief - Convert Raw Gyroscope Data to dps's
 */
void LSM6DS3_convert_gyroscope_to_angular_rate( int16_t axis, float * float_value )
{
  LSM6DS3_get_gyroscope_resolution(float_value); // get resolution value
  *float_value = (float) axis * *float_value;     // calculate float value
}


/**
 * @brief - Check FIFO Count
 *
 * @return - Value is multiplied by 2 because device returns value in
 *           as represented by 1 LSB = 2bytes
 */
uint16_t LSM6DS3_check_FIFO_count()
{
  uint8_t rx_data[3];
  LSM6DS3_read(LSM6DS3_FIFO_STATUS1, &rx_data[0], 3);

  return (uint16_t)(rx_data[1]  | ((rx_data[2] & 0x0F) << 8)) * 2;
}


/**
 * @brief - Read FIFO
 *
 */
//@read
void LSM6DS3_read_fifo( uint8_t * data, uint16_t length )
{
  if(length > 255)
  {
    MW_LOG_INFO(LSM6DS3_LOG_TAG "FIFO READ Length ERROR!");
    return;
  }
  LSM6DS3_read(LSM6DS3_FIFO_DATA_OUT_L, data, length);
}


/**
 * @brief - Read Gyroscope
 *
 */
//@read
void LSM6DS3_read_gyroscope( lsm6ds3_three_axis_t * gyro )
{
  uint8_t rx_data[7];

  LSM6DS3_read(LSM6DS3_OUTX_L_G, rx_data, 7);
  gyro->x_axis = rx_data[1] << 8 | rx_data[2];
  gyro->y_axis = rx_data[3] << 8 | rx_data[4];
  gyro->z_axis = rx_data[5] << 8 | rx_data[6];
}


/**
 * @brief - Read Accelerometer
 *
 */
//@read
void LSM6DS3_read_accelerometer( lsm6ds3_three_axis_t * accel )
{
  uint8_t rx_data[7];

  LSM6DS3_read(LSM6DS3_OUTX_L_XL, rx_data, 7);
  accel->x_axis = rx_data[1] << 8 | rx_data[2];
  accel->y_axis = rx_data[3] << 8 | rx_data[4];
  accel->z_axis = rx_data[5] << 8 | rx_data[6];
}


/**
 * @brief - Read Accelerometer and Gyroscope
 *
 */
//@read
void LSM6DS3_read_imu( lsm6ds3_all_imu_sensors_t * imu )
{
  uint8_t rx_data[13];

  LSM6DS3_read(LSM6DS3_OUTX_L_G, rx_data, 13);
  imu->gyro.x_axis  = rx_data[1] << 8 | rx_data[2];
  imu->gyro.y_axis  = rx_data[3] << 8 | rx_data[4];
  imu->gyro.z_axis  = rx_data[5] << 8 | rx_data[6];
  imu->accel.x_axis = rx_data[7] << 8 | rx_data[8];
  imu->accel.y_axis = rx_data[9] << 8 | rx_data[10];
  imu->accel.z_axis = rx_data[11] << 8 | rx_data[12];
}



/**
 * @brief - Enable/Disable Accelerometer
 *
 */
void LSM6DS3_enable_accel( bool accel )
{
  uint8_t rx_data[2];

  /* Accelerometer Enable */
  LSM6DS3_read(LSM6DS3_CTRL9_XL, rx_data, 2);
  rx_data[1] = rx_data[1] & 0x04; /* mask unused bit */
  if(accel)
  {
    rx_data[1] = rx_data[1] | 0x38; /* Enable all Axis */
    LSM6DS3_write(LSM6DS3_CTRL9_XL, rx_data[1]);
  }
  else
  {

  }

  nrf_delay_ms(200);
}


/**
 * @brief - Enable/Disable Gyroscope
 *
 */
void LSM6DS3_enable_gyro( bool gyro )
{
  uint8_t rx_data[2];

  /* Gyroscope Enable */
  LSM6DS3_read(LSM6DS3_CTRL10_C, rx_data, 2);
  rx_data[1] = rx_data[1] & 0x07; /* mask unused bit */
  if(gyro)
  {
    rx_data[1] = rx_data[1] | 0x38; /* Enable all Axis */
    LSM6DS3_write(LSM6DS3_CTRL10_C, rx_data[1]);

    LSM6DS3_read(LSM6DS3_CTRL4_C, rx_data, 2);
    LSM6DS3_write(LSM6DS3_CTRL4_C, rx_data[1] & 0xBF); /* Assure Sleep disabled */

    LSM6DS3_write(LSM6DS3_CTRL2_G, 0x50); /* set to 208Hz by default */
  }
  else
  {
    LSM6DS3_read(LSM6DS3_CTRL2_G, rx_data, 2);
    LSM6DS3_write(LSM6DS3_CTRL2_G, rx_data[1] & 0x0F); /* Disable ODR */
  }

  nrf_delay_ms(200);
}

/**
 * @brief - Set Gyro High Performance Mode
 *
 */
void LSM6DS3_set_gyro_high_performance_mode( bool enabled )
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_CTRL7_G, rx_data, 2);

  rx_data[1] = rx_data[1] & 0x7C; /* mask unused bits */
  if(!enabled)
  {
    rx_data[1] = rx_data[1] | 0xFC;
  }

  LSM6DS3_write(LSM6DS3_CTRL7_G, rx_data[1]);
}



/**
 * @brief - Set Accel High Performance Mode
 *
 */
void LSM6DS3_set_accel_high_performance_mode( bool enabled )
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_CTRL6_C, rx_data, 2);

  rx_data[1] = rx_data[1] & 0xE0; /* mask unused bits */
  if(!enabled)
  {
    rx_data[1] = rx_data[1] & 0xE0;
  }

  LSM6DS3_write(LSM6DS3_CTRL6_C, rx_data[1]);
}


/**
 * @brief - Set Self Test
 *
 */
void LSM6DS3_set_self_test( uint8_t value)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_CTRL5_C, rx_data, 2);

  rx_data[1] = rx_data[1] & 0xE0; /* mask unused bits */
  value = value & 0x0F; /* filter out unnecessary bits */

  rx_data[1] =  rx_data[1] | value;
  LSM6DS3_write(LSM6DS3_CTRL5_C, value);
}


/**
 * @brief - Get Ctrl 4 Register
 *
 */
uint8_t LSM6DS3_get_reg_ctrl4(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_CTRL4_C, rx_data, 2);
  return (rx_data[1]);
}

/**
 * @brief - Set Ctrl 4 Register
 *
 */
void LSM6DS3_set_reg_ctrl4( uint8_t ctrl )
{
  LSM6DS3_write(LSM6DS3_CTRL4_C, ctrl);
}

/**
 * @brief - Set Ctrl 3 Register
 *
 */
void LSM6DS3_set_reg_ctrl3( uint8_t ctrl )
{
  LSM6DS3_write(LSM6DS3_CTRL3_C, ctrl);
}



/**
 * @brief - Get Gyro Ctrl2 Register
 *
 */
uint8_t LSM6DS3_get_gyro_ctrl2(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_CTRL2_G, rx_data, 2);
  return (rx_data[1]);
}

/**
 * @brief - Set Gyro Ctrl2 Register
 *
 */
void LSM6DS3_set_gyro_ctrl2( uint8_t ctrl )
{
  LSM6DS3_write(LSM6DS3_CTRL2_G, ctrl);
}

/**
 * @brief - Get INT2 Ctrl Register
 *
 */
uint8_t LSM6DS3_get_accel_ctrl1(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_CTRL1_XL, rx_data, 2);
  return (rx_data[1]);
}

/**
 * @brief - Set INT2 Ctrl Register
 *
 */
void LSM6DS3_set_accel_ctrl1( uint8_t ctrl )
{
  LSM6DS3_write(LSM6DS3_CTRL1_XL, ctrl);
}


/**
 * @brief - Get INT2 Ctrl Register
 *
 */
uint8_t LSM6DS3_get_int2_ctrl(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_INT2_CTRL, rx_data, 2);
  return (rx_data[1]);
}

/**
 * @brief - Set INT2 Ctrl Register
 *
 */
void LSM6DS3_set_int2_ctrl( uint8_t ctrl )
{
  LSM6DS3_write(LSM6DS3_INT2_CTRL, ctrl);
}



/**
 * @brief - Get INT1 Ctrl Register
 *
 */
uint8_t LSM6DS3_get_int1_ctrl(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_INT1_CTRL, rx_data, 2);
  return (rx_data[1]);
}

/**
 * @brief - Set INT1 Ctrl Register
 *
 */
void LSM6DS3_set_int1_ctrl( uint8_t ctrl )
{
  LSM6DS3_write(LSM6DS3_INT1_CTRL, ctrl);
}


/**
 * @brief - Flush FIFO
 */
void LSM6DS3_flush_fifo()
{
  static uint32_t loops = 0;
  uint8_t rx_data[10];
  uint16_t fifo_size = LSM6DS3_check_FIFO_count();

  while(fifo_size > 0)
  {
    LSM6DS3_read_fifo(rx_data, 10);
    fifo_size = LSM6DS3_check_FIFO_count();
    loops++;
  }
  loops=0;
}

/**
 * @brief - Get FIFO Mode
 *
 */
uint8_t LSM6DS3_get_fifo_mode(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_FIFO_CTRL5, rx_data, 2);
  return (rx_data[1] & 0x07);
}

/**
 * @brief - Set FIFO Mode
 *
 */
void LSM6DS3_set_fifo_mode( uint8_t mode )
{
  mode = mode & 0x07;  /* Assure to strip out unnecessary bits */
  LSM6DS3_write(LSM6DS3_FIFO_CTRL5, mode);
}


/**
 * @brief - Get FIFO Status
 *
 */
uint8_t LSM6DS3_get_fifo_status( void )
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_FIFO_STATUS2, rx_data, 2);
  return (rx_data[1]);
}


/**
 * @brief - Get FIFO ODR
 *
 */
uint8_t LSM6DS3_get_fifo_odr(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_FIFO_CTRL5, rx_data, 2);
  return (rx_data[1] >> 3);
}

/**
 * @brief - Set FIFO ODR
 *
 */
void LSM6DS3_set_fifo_odr( uint8_t fifo_odr )
{
  fifo_odr = fifo_odr & 0x07;  /* Assure to strip out unnecessary bits */
  fifo_odr = fifo_odr << 3;

  LSM6DS3_write(LSM6DS3_FIFO_CTRL5, fifo_odr);
}



/**
 * @brief - Get FIFO Decimation
 *
 */
uint8_t LSM6DS3_get_fifo_decimation(void)
{
  uint8_t rx_data[2];
  LSM6DS3_read(LSM6DS3_FIFO_CTRL3, rx_data, 2);
  return rx_data[1] & 0x07;
}


/**
 * @brief - Set FIFO Decimation
 *
 * @param - Decimation Factor
 */
void LSM6DS3_set_fifo_decimation( uint8_t decimation )
{
  uint8_t tx_data;

  decimation = decimation & 0x07;  /* Assure to strip out unnecessary bits */
  tx_data = decimation << 3 | decimation;

  LSM6DS3_write(LSM6DS3_FIFO_CTRL3, tx_data);
}


/**
 * @brief - Set FIFO Write Mode
 *
 * @param - STEP_MODE: data written on step detection
 *          DATA_RDY_MODE: data written as its measureed
 */
void LSM6DS3_set_fifo_write_mode( uint8_t mode )
{
  uint8_t rx_data[2];
  uint8_t tx_data;

  LSM6DS3_read(LSM6DS3_FIFO_CTRL2, rx_data, 2);

  mode    = mode & 0x40; /* strip out any extra bits written */

  tx_data = rx_data[1] & 0x8F; /* mask exist bits */

  tx_data = tx_data & mode;
  LSM6DS3_write(LSM6DS3_FIFO_CTRL2, tx_data);
}

/**
 * @brief - Get FIFO Threshold
 *
 * @return - Value is multiplied by 2 because device returns value in
 *           as represented by 1 LSB = 2bytes
 */
uint16_t LSM6DS3_get_fifo_threshold(void)
{
  uint8_t rx_data[3];
  uint16_t level;

  LSM6DS3_read(LSM6DS3_FIFO_CTRL1, rx_data, 3);

  level = ((rx_data[2] & 0x0F) << 8 ) | rx_data[1];

  return level * 2;
}

/**
 * @brief - Set FIFO Threshold
 *
 * @param - Absolute threshold, divided by 2 as representation
 *          on the device is 1 LSB = 2 bytes
 */
void LSM6DS3_set_fifo_threshold( uint16_t level )
{
  uint8_t rx_data[3];
  uint8_t tx_data[3];
  uint8_t mask;

  level /= 2; /*see note above */

  LSM6DS3_read(LSM6DS3_FIFO_CTRL1, rx_data, 3);

  mask = (rx_data[2] & 0xF0); /* Do not change non-threshold bits */

  tx_data[0] = LSM6DS3_FIFO_CTRL1;
  tx_data[1] = level & 0xFF;
  tx_data[2] = mask | ( ( level >> 8) & 0x0F);

  LSM6DS3_write_bytes( tx_data, 3 );
}


/**
 * @brief - Reaset device
 */
void LSM6DS3_reset_device()
{
  LSM6DS3_write(LSM6DS3_CTRL3_C, 0x01);
  nrf_delay_ms(200);
  LSM6DS3_write(LSM6DS3_CTRL3_C, 0x00);
}


/**
 * @brief - Reads the device ID, used to verify communication
 */
uint8_t LSM6DS3_read_device_ID(void)
{
  uint8_t device_id[2];
  LSM6DS3_read( LSM6DS3_WHO_AM_I_REG, device_id, 2);
  return device_id[1];
}


//*********************************************************************************************
//*********************************************************************************************
/**
 * @brief - Wrapper Function for Reading from a Register
 */
//@read
void LSM6DS3_read( uint8_t address, uint8_t * rx_data, uint16_t length )
{
  uint8_t tx_data[2];

  memset(rx_data, 0, length);
  memset(tx_data, 0, sizeof(tx_data));

  tx_data[0] = address;
  mw_spi_master_transfer(m_LSM6DS3_SPI_ID, SPI_READ, tx_data, 1 /*length*/, rx_data, length);
}

/**
 * @brief - Wrapper Function for Writing to a Register
 */
//@write
void LSM6DS3_write( uint8_t address, uint8_t value )
{
  uint8_t tx_data[2];
  uint8_t rx_data[2];
  memset(tx_data, 0, sizeof(tx_data));
  memset(rx_data, 0, sizeof(rx_data));


  tx_data[0] = address;
  tx_data[1] = value;
  mw_spi_master_transfer(m_LSM6DS3_SPI_ID, SPI_WRITE, tx_data, sizeof(tx_data), rx_data, sizeof(rx_data));
}


/**
 * @brief - Wrapper Function for Writing to a Register, multiple bytes
 *
 * @param - Address must be passed in the first of the data[]
 */
//@write
//@write multiple bytes
void LSM6DS3_write_bytes( uint8_t * data, uint16_t length )
{
  uint8_t rx_data[2];
  memset(rx_data, 0, sizeof(rx_data));

  mw_spi_master_transfer(m_LSM6DS3_SPI_ID, SPI_WRITE, data, length, rx_data, sizeof(rx_data));
}


//*********************************************************************************************
//*********************************************************************************************
/**
 * @brief - LSM6DS3 Driver Place in Low Power Mode
 */
void LSM6DS3_suspend()
{
  if( LSM6DS3_driver_config.communication == SPI_COMMUNICATION )
  {
    mw_spi_master_driver_uninit(m_LSM6DS3_SPI_ID);

    nrf_gpio_cfg( LSM6DS3_driver_config.spi_config.ss_pin,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_D0S1,
            NRF_GPIO_PIN_NOSENSE);  //IMU CS
    nrf_delay_ms(40);

    nrf_gpio_cfg( LSM6DS3_driver_config.spi_config.sck_pin,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_D0S1,
            NRF_GPIO_PIN_NOSENSE);  //IMU CLK
    nrf_delay_ms(40);

    nrf_gpio_cfg( LSM6DS3_driver_config.spi_config.mosi_pin,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_D0S1,
            NRF_GPIO_PIN_NOSENSE);  //IMU MOSI
    nrf_delay_ms(40);

    nrf_gpio_cfg( LSM6DS3_driver_config.spi_config.miso_pin,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_D0S1,
            NRF_GPIO_PIN_NOSENSE);  //IMU MISO
    nrf_delay_ms(40);

    nrf_gpio_cfg( IMU_INT_1_PIN,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_D0S1,
            NRF_GPIO_PIN_NOSENSE);  //IMU INT
    nrf_delay_ms(40);
  }
}


/**
 * @brief - LSM6DS3 Driver Initialization
 *
 * @param - Driver Initialzation parameter contains all interface configurations
 */
void LSM6DS3_initialize( external_device_config_t device_config )
{
  if( device_config.communication == SPI_COMMUNICATION )
  {
    spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

    if(return_value.err_code != NRF_SUCCESS) APP_ERROR_CHECK(return_value.err_code);

    m_LSM6DS3_SPI_ID = return_value.device_id;

    MW_LOG_INFO("LSM6DS3 device SPI Initialized");
  }

  if ( device_config.communication == TWI_COMMUNICATION )
  {
    mw_twi_master_init(device_config.twi_config);

    MW_LOG_INFO("LSM6DS3 device TWI Initialized");
  }

  memcpy(&LSM6DS3_driver_config, &device_config, sizeof(device_config));
  m_lsm6ds3_initialized = true;
}
