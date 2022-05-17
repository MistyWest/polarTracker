/* AM0805AQ.h , AM0805 code: external RTC module is used by host MCU */

#ifndef AM0805AQ_H_
#define AM0805AQ_H_

/*MW*/
#include "stdbool.h"
#include "_mw_external_device.h"

#define AM0805AQ_I2C_ADDR 0x69

/* Register Map */
#define AM0805_REG_HUNDREDTHS   0x00
#define AM0805_REG_ALM_HUN      0x08
#define AM0805_REG_STATUS       0x0F
#define AM0805_REG_CTRL1        0x10
#define AM0805_REG_CTRL2        0x11
#define AM0805_REG_INTMASK      0x12
#define AM0805_REG_SQW          0x13
#define AM0805_REG_SLEEPCTRL    0x17
#define AM0805_REG_TIM_CTRL     0x18
#define AM0805_REG_CDTIM        0x19
#define AM0805_REG_TIMINIT      0x1A
#define AM0805_REG_WDT          0x1B
#define AM0805_REG_OSC_CTRL     0x1C
#define AM0805_REG_OSCSTATUS    0x1D
#define AM0805_REG_ID0          0x28
#define AM0805_REG_ID1          0x29
#define AM0805_REG_EXTADDR_REG  0x3F

/* Register Value */
#define AM0805_VALUE_ID0        0x08
#define AM0805_VALUE_ID1        0x05


typedef enum
{
  XT1_INTERRUPT = 0x01, /**< WDI input pin will generate XT1 interrupt  */
  XT2_INTERRUPT = 0x02 /**< EXTI input pin will generate XT2 interrupt */
} input_interrupt_type_t;


typedef enum
{
  TWENTY_FOUR_HR_FORMAT = 2,
  TWELVE_HR_PM_FORMAT = 1,
  TWELVE_HR_AM_FORMAT = 0
}time_mode_t;


typedef struct
{
  uint8_t hundredth;
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t date;
  uint8_t weekday;
  uint8_t month;
  uint8_t year;
  time_mode_t mode;
} time_reg_struct_t;
/*
 * hundredth : 0 ~ 99
 * second : 0 ~ 59
 * minute : 0 ~ 59
 * weekday : 0 ~ 6
 * month : 1 ~ 12
 * year : 0 ~ 99
 * mode : 0 ~ 2
 */


typedef enum
{
  DISABLE_ALARM = 0, /**< disable alarm */
  ONCE_PER_YEAR = 1, /**< once per year */
  ONCE_PER_MONTH = 2, /**< once per month */
  ONCE_PER_WEEK = 3, /**< once per week */
  ONCE_PER_DAY = 4, /**< once per day */
  ONCE_PER_HOUR = 5, /**< once per hour */
  ONCE_PER_MINUTE = 6, /**< once per minute */
  ONCE_PER_SECOND = 7, /**< once per second */
  ONCE_PER_10TH_SEC = 8, /**< once per 10th of a second */
  ONCE_PER_100TH_SEC = 9 /**< once per 100th of a second */
} alarm_repeat_t;


typedef enum
{
  PERIOD_US = 0, /**< period in us */
  PERIOD_SEC = 1 /**< period in seconds */
} count_down_range_t;

typedef enum
{
  SINGLE_LEVEL_INTERRUPT = 0, /**< single level interrupt */
  REPEAT_PULSE_1_4096_SEC = 1, /**< a repeated pulsed interrupt, 1/4096 s (XT mode), 1/128 s (RC mode) (range must be 0) */
  SINGLE_PULSE_1_4096_SEC = 2, /**< a single pulsed interrupt, 1/4096 s (XT mode), 1/128 s (RC mode) (range must be 0) */
  REPEAT_PLUSE_1_128_SEC = 3, /**< a repeated pulsed interrupt, 1/128 s (range must be 0) */
  SINGLE_PLUSE_1_128_SEC = 4, /**< a single pulsed interrupt, 1/128 s (range must be 0) */
  REPEAT_PLUSE_1_64_SEC = 5, /**< a repeated pulsed interrupt, 1/64 s (range must be 1) */
  SINGLE_PLUSE_1_64_SEC = 6 /**< a single pulsed interrupt, 1/64 s (range must be 1) */
} count_down_repeat_t;


typedef enum
{
  LEVEL_INTERRUPT = 0x00, /**< level interrupt */
  PULSE_1_8192_SEC = 0x01, /**< pulse of 1/8192s (XT) or 1/128 s (RC) */
  PULSE_1_64_SEC = 0x10, /**< pulse of 1/64 s  */
  PULSE_1_4_SEC = 0x11 /**< pulse of 1/4 s  */
} interrupt_mode_t;


typedef enum
{
  INTERNAL_FLAG = 0, /**< internal flag only */
  PIN_FOUT_nIRQ = 1, /**< generate the interrupt on FOUT/nIRQ */
  PIN_PSW_nIRQ2 = 2, /**< generate the interrupt on PSW/nIRQ2 */
  PIN_nTIRQ = 3 /**< generate the interrupt on nTIRQ (not apply to ALARM) */
} interrupt_pin_t;

/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_get_time_reg_format_shift( uint8_t time, uint8_t bit_shift );


/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_set_time_reg_format_shift( uint8_t time, uint8_t bit_shift );


void am0805_register_read( uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes );

void am0805_register_write( uint8_t register_address, uint8_t value );

void am0805_burst_write( uint8_t *value, uint8_t number_of_bytes );

uint8_t am0805_read_ram( uint8_t address );

/**
 * @brief 0 Disables all interrupts and put in Low Power Mode
 */
void am0805_disable_interrupts();

/**
 * @brief - Configure Interrupts
 */
void am0805_config_input_interrupt( input_interrupt_type_t index_Interrupt );


void am0805_write_ram( uint8_t address, uint8_t data );

void am0805_set_time( time_reg_struct_t time_regs );

void am0805_get_time( time_reg_struct_t *time_regs );

uint8_t am0805_get_status();

void am0805_config_alarm( time_reg_struct_t time_regs, alarm_repeat_t repeat, interrupt_mode_t intmode, interrupt_pin_t pin );

void am0805_config_countdown_timer( count_down_range_t range, uint32_t period, count_down_repeat_t repeat, interrupt_pin_t pin );

void am0805_set_sleep( uint8_t timeout, uint8_t mode );

bool am0805_verify_product_id(void);

void am0805_clock_cfg(void);

void am0805_init( external_device_config_t device_config );

#endif //AM0805AQ_H_
