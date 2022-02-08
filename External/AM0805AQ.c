/* AM0805 Sample code: external RTC module is used by host MCU */

#include "boards.h"
#include "project_board.h"
#include "project_settings.h"
#include "mw_logging.h"
#include "_mw_external_device.h"
#include "AM0805AQ.h"

#define AM0805_DEBUG            0

#define REGISTER_WRITE_VERIFY_ENABLE 1

static external_device_communication_t    m_AM0805AQ_interface;
static external_driver_spi_config_t       m_AM0805AQ_SPI_ID;
static external_driver_twi_config_t       m_AM0805AQ_TWI;

static bool m_AM0805AQ_initialized = false;

static void setreg( char address, uint8_t mask );
static void clrreg( char address, uint8_t mask );
static uint8_t readreg( uint8_t address );



//SE_TODO remove me eventually
/*I2C i2c(I2C_SDA, I2C_SCL);*/
#if AM0805_DEBUG
Serial pc(p13,p14);
#endif


static void AM0805AQ_REGISTER_WRITE_VERIFY( uint8_t address, uint8_t compare_value )
{
#if REGISTER_WRITE_VERIFY_ENABLE
  uint8_t rx_data;
  am0805_register_read(address, &rx_data, 1);

  if( rx_data != compare_value )
  {
    while(1){}  // Error Caught
  }
#endif
}




/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_set_year_reg_format( uint8_t year )
{
  /* Max value check */
  if( year > 99 ) return 0x99;

  return am0805_set_time_reg_format_shift(year, 4);
}


/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_set_month_reg_format( uint8_t month )
{
  /* Max value check */
  if( month > 12 ) return 0x12;

  return am0805_set_time_reg_format_shift(month, 4);
}



/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_set_date_reg_format( uint8_t date )
{
  /* Max value check */
  if( date > 31 ) return 0x31;

  return am0805_set_time_reg_format_shift(date, 2);
}


/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_set_hour_reg_format( uint8_t hours )
{
  /* Max value check */
  if( hours > 23 ) return 0x00;

  return am0805_set_time_reg_format_shift(hours, 2);
}

/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_set_mins_or_sec_reg_format( uint8_t time )
{
  if( time > 59 ) return 0x59;

  return am0805_set_time_reg_format_shift(time, 3);
}



/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_get_year_reg_format( uint8_t year )
{
  /* Max value check */
  if( year > 0x99 ) return 0x99;

  return am0805_get_time_reg_format_shift(year, 4);
}


/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_get_month_reg_format( uint8_t month )
{
  /* Max value check */
  if( month > 0x12 ) return 0x12;

  return am0805_get_time_reg_format_shift(month, 4);
}



/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_get_date_reg_format( uint8_t date )
{
  /* Max value check */
  if( date > 0x31 ) return 0x31;

  return am0805_get_time_reg_format_shift(date, 2);
}


/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_get_hour_reg_format( uint8_t hours )
{
  /* Max value check */
  if( hours > 0x24 ) return 0x24;

  return am0805_get_time_reg_format_shift(hours, 2);
}

/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_get_mins_or_sec_reg_format( uint8_t time )
{
  if( time > 0x60 ) return 0x60;

  return am0805_get_time_reg_format_shift(time, 3);
}


/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_set_time_reg_format_shift( uint8_t time, uint8_t bit_shift )
{
  uint8_t upper_four_bits;
  uint8_t lower_four_bits;
  uint8_t upper_byte_make = 0;

  switch(bit_shift)
  {
  case 1:
    upper_byte_make = 0x01;
    break;

  case 2:
    upper_byte_make = 0x03;
    break;

  case 3:
    upper_byte_make = 0x07;
    break;

  case 4:
    upper_byte_make = 0x0F;
    break;

  /* Error case*/
  default:
    break;
  }


  upper_four_bits = time / 10;
  upper_four_bits &= upper_byte_make;

  lower_four_bits = (time % 10);

  return (upper_four_bits << 4)|lower_four_bits;
}


/**
 * @brief - Format data into AM0805AQ time register bit format
 */
uint8_t am0805_get_time_reg_format_shift( uint8_t time, uint8_t bit_shift )
{
  uint8_t upper_byte_make = 0;
  switch(bit_shift)
  {
  case 1:
    upper_byte_make = 0x01;
    break;

  case 2:
    upper_byte_make = 0x03;
    break;

  case 3:
    upper_byte_make = 0x07;
    break;

  case 4:
    upper_byte_make = 0x0F;
    break;

  /* Error case*/
  default:
    break;
  }


  uint8_t upper_four_bits = (time >> 4) & upper_byte_make;
  uint8_t lower_four_bits = time & 0x0F;

  return (upper_four_bits * 10 ) + lower_four_bits;
}


/**
 * @brief - Formats and bit shifts time struct for registers
 */
void am0805_set_format_time(time_reg_struct_t * time_regs)
{
  time_regs->year = am0805_set_year_reg_format(time_regs->year);
  time_regs->date = am0805_set_date_reg_format(time_regs->date);
  time_regs->hour = am0805_set_hour_reg_format(time_regs->hour);
  time_regs->minute = am0805_set_mins_or_sec_reg_format(time_regs->minute);
  time_regs->second = am0805_set_mins_or_sec_reg_format(time_regs->second);
}


/**
 * @brief - Formats and bit shifts time struct for registers
 */
void am0805_get_format_time(time_reg_struct_t * time_regs)
{
  time_regs->year = am0805_get_year_reg_format(time_regs->year);
  time_regs->date = am0805_get_date_reg_format(time_regs->date);
  time_regs->hour = am0805_get_hour_reg_format(time_regs->hour);
  time_regs->minute = am0805_get_mins_or_sec_reg_format(time_regs->minute);
  time_regs->second = am0805_get_mins_or_sec_reg_format(time_regs->second);
}


uint8_t am0805_get_status()
{
  uint8_t data;

  am0805_register_read(AM0805_REG_STATUS, &data, 1);
  return data;
}



/*
 * hundredth : 0 ~ 99
 * second : 0 ~ 59
 * minute : 0 ~ 59
 * weekday : 0 ~ 6
 * month : 1 ~ 12
 * year : 0 ~ 99
 * mode : 0 ~ 2
 */
void am0805_set_time(time_reg_struct_t time_regs)
{
    uint8_t temp;
    uint8_t temp_buff[9];


    am0805_set_format_time(&time_regs);

    /*
     * Determine whether 12 or 24-hour timekeeping mode is being used
     * and set the 1224 bit appropriately
     */
    if (time_regs.mode == 2)        // 24-hour day
    {
        clrreg(AM0805_REG_CTRL1, 0x40);
    }
    else if (time_regs.mode == 1)   // 12-hour day PM
    {
        time_regs.hour |= 0x20;     // Set AM/PM
        setreg(AM0805_REG_CTRL1, 0x40);
    }
    else                            // 12-hour day AM
    {
        setreg(AM0805_REG_CTRL1, 0x40);
    }

    /* Set the WRTC bit to enable counter writes. */
    setreg(AM0805_REG_CTRL1, 0x01);

//    /* Set the correct century */
//    if (time_regs.century == 0)
//    {
//        clrreg(AM0805_REG_STATUS, 0x80);
//    }
//    else
//    {
//        setreg(AM0805_REG_STATUS, 0x80);
//    }

    /* Write all of the time counters */
    temp_buff[0] = AM0805_REG_HUNDREDTHS;
    temp_buff[1] = time_regs.hundredth;
    temp_buff[2] = time_regs.second;
    temp_buff[3] = time_regs.minute;
    temp_buff[4] = time_regs.hour;
    temp_buff[5] = time_regs.date;
    temp_buff[6] = time_regs.month;
    temp_buff[7] = time_regs.year;
    temp_buff[8] = time_regs.weekday;

    /* Write the values to the AM18XX */
    am0805_burst_write(temp_buff, sizeof(temp_buff));

    /* Load the final value of the WRTC bit based on the value of protect */
    am0805_register_read(AM0805_REG_CTRL1, &temp, 1);
    temp &= 0x7E;                   // Clear the WRTC bit and the STOP bit
    //temp_buff[1] |= (0x01 & (~protect));    // Invert the protect bit and update WRTC
    am0805_register_write(AM0805_REG_CTRL1, temp);

    return;
}



void am0805_get_time(time_reg_struct_t *time_regs)
{
    uint8_t temp_buff[8];
    uint8_t time_mode;

    /* Read the counters. */
    am0805_register_read(AM0805_REG_HUNDREDTHS, temp_buff, 8);

    time_regs->hundredth = temp_buff[0];
    time_regs->second = temp_buff[1];
    time_regs->minute = temp_buff[2];
    time_regs->hour = temp_buff[3];
    time_regs->date = temp_buff[4];
    time_regs->month = temp_buff[5];
    time_regs->year = temp_buff[6];
    time_regs->weekday = temp_buff[7];

    /* Get the current hours format mode 12:24 */
    am0805_register_read(AM0805_REG_CTRL1, &time_mode, 1);
    if ((time_mode & 0x40) == 0)
    {
        /* 24-hour mode. */
        time_regs->mode = 2;
        time_regs->hour = time_regs->hour & 0x3F;           // Get tens:ones
    }
    else
    {
        /* 12-hour mode.  Get PM:AM. */
        time_regs->mode = (time_regs->hour & 0x20) ? 1 : 0;  // PM : AM
        time_regs->hour &= 0x1F;                            // Get tens:ones
    }

    time_regs->hundredth = temp_buff[0];
    time_regs->second = temp_buff[1];
    time_regs->minute = temp_buff[2];
    time_regs->hour = temp_buff[3];
    time_regs->date = temp_buff[4];
    time_regs->month = temp_buff[5];
    time_regs->year = temp_buff[6];
    time_regs->weekday = temp_buff[7];


    am0805_get_format_time(time_regs);

#if AM0805_DEBUG
    pc.printf("hundredth:%x\r\n",time_regs->hundredth);
    pc.printf("second:%x\r\n",time_regs->second);
    pc.printf("minute:%x\r\n",time_regs->minute);
    pc.printf("hour:%x\r\n",time_regs->hour);
    pc.printf("date:%x\r\n",time_regs->date);
    pc.printf("month:%x\r\n",time_regs->month);
    pc.printf("year:%x\r\n",time_regs->year);
    pc.printf("weekday:%x\r\n",time_regs->weekday);
#endif
}



void am0805_config_alarm(time_reg_struct_t time_regs, alarm_repeat_t repeat, interrupt_mode_t intmode, interrupt_pin_t pin)
{
  uint8_t temp;
    uint8_t temp_buff[9];
    clrreg(AM0805_REG_STATUS,0x08);                     // Clear TIM
    am0805_set_format_time(&time_regs);

    /* Determine whether a 12-hour or a 24-hour time keeping mode is being used */
    if (time_regs.mode == 1)
    {
        /* A 12-hour day PM */
        time_regs.hour = time_regs.hour | 0x20;   // Set AM/PM
    }

    /* Write all of the time counters */
    temp_buff[0] = AM0805_REG_ALM_HUN;
    temp_buff[1] = time_regs.hundredth;
    temp_buff[2] = time_regs.second;
    temp_buff[3] = time_regs.minute;
    temp_buff[4] = time_regs.hour;
    temp_buff[5] = time_regs.date;
    temp_buff[6] = time_regs.month;
    temp_buff[7] = time_regs.weekday;

    clrreg(AM0805_REG_TIM_CTRL, 0x1C);      // Clear the RPT field
    clrreg(AM0805_REG_INTMASK, 0x64);       // Clear the AIE bit and IM field
    clrreg(AM0805_REG_STATUS, 0x04);       // Clear the ALM flag
    readreg(AM0805_REG_STATUS);             // Clear the ALM flag

    if (pin == PIN_FOUT_nIRQ)
    {
        /* Interrupt on FOUT/nIRQ */
        am0805_register_read(AM0805_REG_CTRL2, &temp, 1);   // Get the Control2 Register
        temp = (temp & 0x03);               // Extract the OUT1S field
        if (temp != 0)                      // Not already selecting nIRQ
        {
            setreg(AM0805_REG_CTRL2, 0x03);    // Set OUT1S to 3
        }
    }
    if (pin == PIN_PSW_nIRQ2)
    {
        /* Interrupt on PSW/nIRQ2 */
        am0805_register_read(AM0805_REG_CTRL2, &temp, 1);   // Get the Control2 Register
        temp &= 0x1C;                       // Extract the OUT2S field
        if (temp != 0)                      // Not already selecting nIRQ
        {
            clrreg(AM0805_REG_CTRL2, 0x1C);    // Clear OUT2S
            setreg(AM0805_REG_CTRL2, 0x0C);    // Set OUT2S to 3
        }
    }

    if (repeat == ONCE_PER_10TH_SEC)
    {
        /* 10ths interrupt */
        temp_buff[1] |= 0xF0;
        repeat = ONCE_PER_SECOND;                   // Select correct RPT value
    }
    if (repeat == ONCE_PER_100TH_SEC)
    {
        /* 100ths interrupt */
        temp_buff[1] = 0xFF;
        repeat = ONCE_PER_SECOND;                   // Select correct RPT value
    }
    if (repeat != 0)                                // Don't initiate if repeat = 0
    {
        temp = (repeat << 2);                       // Set the RPT field to the value of repeat
        setreg(AM0805_REG_TIM_CTRL, temp);          // Was previously cleared
        //setreg(AM0805_REG_INTMASK, (intmode << 5)|0x04); // Set the alarm interrupt mode
        am0805_register_write(AM0805_REG_INTMASK, (intmode << 5)|0x04); // Set the alarm interrupt mode
        //setreg(AM0805_REG_INTMASK, 0x60); // Set the alarm interrupt mode
        am0805_burst_write(temp_buff, 8);           // Execute the burst write
        //setreg(AM0805_REG_INTMASK, 0x04);           // Set the AIE bit
    }
    else
        setreg(AM0805_REG_INTMASK, 0x60);           // Set IM field to 0x3 (reset value) to minimize current draw

    return;
}



void am0805_config_countdown_timer(count_down_range_t range, uint32_t period, count_down_repeat_t repeat, interrupt_pin_t pin)
{
    uint8_t tm = 0;
    uint8_t trpt = 0;
    uint8_t tfs = 0;
    uint8_t te = 0;
    uint8_t temp = 0;
    uint8_t tctrl = 0;
    int32_t timer = 0;
    uint8_t oscmode = 0;

    /* 0 = XT, 1 = RC */
    am0805_register_read(AM0805_REG_OSCSTATUS, &oscmode, 1);
    oscmode = (oscmode & 0x10) ? 1 : 0;

    /* disable count down timer */
    if (pin == INTERNAL_FLAG)
    {
        te = 0;
    }
    else
    {
        te = 1;
        if (repeat == SINGLE_LEVEL_INTERRUPT)
        {
            /* Level interrupt */
            tm = 1;                                     // Level
            trpt = 0;                                   // No repeat
            if (range == PERIOD_US)
            {
                /* Microseconds */
                if (oscmode == 0)
                {
                    /* XT Mode */
                    if (period <= 62500)                // Use 4K Hz
                    {
                        tfs = 0;
                        timer = (period * 4096);
                        timer = timer / 1000000;
                        timer = timer - 1;
                    }
                    else if (period <= 16384000)        // Use 64 Hz
                    {
                        tfs = 1;
                        timer = (period * 64);
                        timer /= 1000000;
                        timer = timer - 1;
                    }
                    else                                // Use 1 Hz
                    {
                        tfs = 2;
                        timer = period / 1000000;
                        timer = timer - 1;
                    }
                }
                else
                {
                    /* RC Mode */
                    if (period <= 2000000) {            // Use 128 Hz
                        tfs = 0;
                        timer = (period * 128);
                        timer /= 1000000;
                        timer = timer - 1;
                    }
                    else if (period <= 4000000) {       // Use 64 Hz
                        tfs = 1;
                        timer = (period * 64);
                        timer /= 1000000;
                        timer = timer - 1;
                    }
                    else {                              // Use 1 Hz
                        tfs = 2;
                        timer = period / 1000000;
                        timer = timer - 1;
                    }
                }
            }
            else
            {
                /* Seconds */
                if (period <= 256)
                {
                    /* Use 1 Hz */
                    tfs = 2;
                    timer = period - 1;
                }
                else
                {
                    /* Use 1/60 Hz */
                    tfs = 3;
                    timer = period / 60;
                    timer = timer - 1;
                }
            }
        }
        else
        {
            /* Pulse interrupts */
            tm = 0;                 // Pulse
            trpt = repeat & 0x01;   // Set up repeat
            if (repeat < REPEAT_PLUSE_1_128_SEC)
            {
                tfs = 0;
                if (oscmode == 0)
                {
                        timer = (period * 4096);
                        timer /= 1000000;
                        timer = timer - 1;
                }
                else
                {
                        timer = (period * 128);
                        timer /= 1000000;
                        timer = timer - 1;
                }
            }
            else if (repeat < REPEAT_PLUSE_1_64_SEC)
            {
                tfs = 1;
                timer = (period * 128);
                timer /= 1000000;
                timer = timer - 1;
            }
            else if (period <= 256)
            {
                /* Use 1 Hz */
                tfs = 2;
                timer = period - 1;
            }
            else
            {
                /* Use 1/60 Hz */
                tfs = 3;
                timer = period / 60;
                timer = timer - 1;
            }
        }
    }

    am0805_register_read(AM0805_REG_TIM_CTRL, &tctrl, 1);               // Get TCTRL, keep RPT, clear TE
    tctrl = tctrl & 0x1C;
    am0805_register_write(AM0805_REG_TIM_CTRL, tctrl);

    tctrl = tctrl | (te * 0x80) | (tm * 0x40) | (trpt * 0x20) | tfs;    // Merge the fields

    if (pin == PIN_FOUT_nIRQ)                                           // generate nTIRQ interrupt on FOUT/nIRQ (asserted low)
    {
         clrreg(AM0805_REG_CTRL2, 0x3);                                 // Clear OUT1S
    }
    if (pin == PIN_PSW_nIRQ2)                                           // generate nTIRQ interrupt on PSW/nIRQ2 (asserted low)
    {
         am0805_register_read(AM0805_REG_CTRL2, &temp, 1);              // Get OUT2S
         if ((temp & 0x1C) != 0)
         {
             temp = (temp & 0xE3) | 0x14;                               // If OUT2S != 0, set OUT2S to 5
         }
         am0805_register_write(AM0805_REG_CTRL2, temp);                 // Write back
    }
    if (pin != 0)
    {
        clrreg(AM0805_REG_STATUS,0x08);                     // Clear TIM
        setreg(AM0805_REG_INTMASK,0x08);                    // Set TIE
        am0805_register_write(AM0805_REG_CDTIM, timer);     // Initialize the timer
        am0805_register_write(AM0805_REG_TIMINIT, timer);   // Initialize the timer repeat
        am0805_register_write(AM0805_REG_TIM_CTRL, tctrl);  // Start the timer
    }

    return ;
}



/** Parameter:
 *  timeout - minimum timeout period in 7.8 ms periods (0 to 7)
 *  mode - sleep mode (nRST modes not available in AM08xx)
 *      0 => nRST is pulled low in sleep mode
 *      1 => PSW/nIRQ2 is pulled high on a sleep
 *      2 => nRST pulled low and PSW/nIRQ2 pulled high on sleep
 *  error ?returned value of the attempted sleep command
 *      0 => sleep request accepted, sleep mode will be initiated in timeout seconds
 *      1 => illegal input values
 *      2 => sleep request declined, interrupt is currently pending
 *      3 => sleep request declined, no sleep trigger interrupt enabled
**/
void am0805_set_sleep(uint8_t timeout, uint8_t mode)
{
    uint8_t slres = 0;
    uint8_t temp = 0;

#if AM0805_DEBUG
    am0805_register_read(AM0805_REG_CTRL2, &temp, 1);       // Get SLST bit (temp & 0x08)

    if ( ( temp & 0x08 ) == 0)
    {
        pc.printf("Previous Sleep Failed\r\n");
    } else {
        pc.printf("Previous Sleep Successful\r\n");
    }
    clrreg(AM0805_REG_CTRL2,0x08);                     // Clear SLST

    am0805_register_read(AM0805_REG_CTRL2, &temp, 1);       // Get SLST bit (temp & 0x08)

    if ( ( temp & 0x08 ) == 0)
    {
        pc.printf("Clear Succ\r\n");
    } else {
        pc.printf("Clear Fail\r\n");
    }
    clrreg(AM0805_REG_CTRL2,0x08);                     // Clear SLST
#endif

    if (mode > 0)
    {
        /* Sleep to PSW/nIRQ2 */
        am0805_register_read(AM0805_REG_CTRL2, &temp, 1);   // Read OUT2S
        temp = (temp & 0xE3) | 0x18;                        // MUST NOT WRITE OUT2S WITH 000
        am0805_register_write(AM0805_REG_CTRL2, temp);      // Write value to OUT2S
        slres = 0;
    }

    temp = timeout | (slres << 6) | 0x80;                   // Assemble SLEEP register value
    am0805_register_write(AM0805_REG_SLEEPCTRL, temp);      // Write to the register

#if AM0805_DEBUG
    /* Determine if SLEEP was accepted */
    am0805_register_read(AM0805_REG_CTRL2, &temp, 1);       // Get SLP bit (temp & 0x80)

    if ( ( temp & 0x80 ) == 0)
    {
        char reg_wdi_value = 0;
        /* SLEEP did not happen - determine why and return reason. */
        am0805_register_read(AM0805_REG_INTMASK, &temp, 1);         // Get status register interrupt enables
        am0805_register_read(AM0805_REG_WDT, &reg_wdi_value, 1);    // Get WDT register
        if ((( temp & 0x0F ) == 0) & (((reg_wdi_value & 0x7C) == 0) || ((reg_wdi_value & 0x80) == 0x80)))
        {
            pc.printf("No trigger interrupts enabled\r\n");
        }
        else
        {
            pc.printf("Interrupt pending\r\n");
        }
    }
    else
    {
        pc.printf("SLEEP request successful\r\n");
    }
#endif
}



void am0805_set_oscillator_control()
{
  //********************************
  //am0805_register_write(AM0805_REG_OSC_CTRL, 0x01);
  //AM0805AQ_REGISTER_WRITE_VERIFY(AM0805_REG_OSC_CTRL, 0x01);
  //********************************
}

/**
 * @brief - function to configure the 32.768 kHz clock *
 */
void am0805_clock_cfg(void)
{
  //********************************
  //set OUT1S = 0b01 (select SQW (or OUT) to nIRQ)
  am0805_register_write(AM0805_REG_CTRL1, 0x01);
  AM0805AQ_REGISTER_WRITE_VERIFY(AM0805_REG_CTRL1, 0x01);
  //********************************
  //set OUT1S = 0b01 (select SQW (or OUT) to nIRQ)
  am0805_register_write(AM0805_REG_CTRL2, 0x01);
  AM0805AQ_REGISTER_WRITE_VERIFY(AM0805_REG_CTRL2, 0x01);
  //********************************
  //set SQWE = 0b1, SQFS = 0b00001 (enable SQW at 32.768 kHZ)
  am0805_register_write(AM0805_REG_SQW, 0xA1);
  AM0805AQ_REGISTER_WRITE_VERIFY(AM0805_REG_SQW, 0xA1);
  //********************************
  //set SQWE = 0b1, SQFS = 0b00001 (enable SQW at 32.768 kHZ)
  am0805_register_write(AM0805_REG_OSC_CTRL, 0x00);
  AM0805AQ_REGISTER_WRITE_VERIFY(AM0805_REG_OSC_CTRL, 0x00);
  //********************************
}



bool am0805_verify_product_id( void )
{
  uint8_t who_am_i[2];
  am0805_register_read(AM0805_REG_ID0, who_am_i, 2);
#if AM0805_DEBUG
  pc.printf("ID:%x\r\n",who_am_i[0]);
#endif
  if ( (who_am_i[0] != AM0805_VALUE_ID0) || (who_am_i[1] != AM0805_VALUE_ID1) )
    return false;
  else
    return true;
}


static uint8_t get_extension_address( uint8_t address )
{
  uint8_t xadd;
  uint8_t temp;

  am0805_register_read(AM0805_REG_EXTADDR_REG, &temp, 1);
  temp = temp & 0xC0;

  if ( address < 64 )
  {
    xadd = 0x8;
  }
  else if ( address < 128 )
  {
    xadd = 0x9;
  }
  else if ( address < 192 )
  {
    xadd = 0xA;
  }
  else
  {
    xadd = 0xB;
  }
  return (xadd | temp);
}

/* Set one or more bits in the selected register, selected by 1's in the mask */
static void setreg( char address, uint8_t mask )
{
  uint8_t temp;

  am0805_register_read(address, &temp, 1);
  temp |= mask;
  am0805_register_write(address, temp);
}

/* Clear one or more bits in the selected register, selected by 1's in the mask */
static void clrreg( char address, uint8_t mask )
{
  uint8_t temp;

  am0805_register_read(address, &temp, 1);
  temp &= ~mask;
  am0805_register_write(address, temp);
}

/* Clear one or more bits in the selected register, selected by 1's in the mask */
static uint8_t readreg( uint8_t address )
{
  uint8_t temp;

  am0805_register_read(address, &temp, 1);
  return temp;
}


/**
 * @brief 0 Disables all interrupts and put in Low Power Mode
 */
void am0805_disable_interrupts()
{
  setreg(AM0805_REG_INTMASK, 0x60);           // Set IM field to 0x3 (reset value) to minimize current draw
}


/**
 * @brief - Configure Interrupts
 */
void am0805_config_input_interrupt( input_interrupt_type_t index_Interrupt )
{
  switch ( index_Interrupt )
  {
  case XT1_INTERRUPT:
    /* Set input interrupt pin EX1T */
    clrreg(AM0805_REG_STATUS, 0x01);             // Clear EX1
    setreg(AM0805_REG_INTMASK, 0x01);            // Set EX1E
    break;
  case XT2_INTERRUPT:
    /* Set input interrupt pin WDI */
    clrreg(AM0805_REG_STATUS, 0x02);             // Clear EX2
    setreg(AM0805_REG_INTMASK, 0x02);            // Set EX2E
    break;
  default:
#if AM0805_DEBUG
    pc.printf("Wrong Input Interrupt Index\r\n");
#endif
    break;
  }
}


/**
 * @brief - Read a byte from local RAM
 */
uint8_t am0805_read_ram(uint8_t address)
{
    uint8_t xadd;
    uint8_t temp;
    uint8_t reg_ram = 0;

    xadd = get_extension_address(address);                  // Calc XADDR value from address
    am0805_register_write(AM0805_REG_EXTADDR_REG, xadd);    // Load the XADDR register
    reg_ram = (address & 0x3F) | 0x40;                      // Read the data
    am0805_register_read(reg_ram, &temp, 1);
#if AM0805_DEBUG
    pc.printf("Read from addr:%x Data:%x\r\n",address,temp);
#endif
    return (uint8_t)temp;
}


/**
 * @brief - Write a byte to local RAM
 */
void am0805_write_ram(uint8_t address, uint8_t data)
{
    uint8_t xadd;
    uint8_t reg_ram = 0;

    xadd = get_extension_address(address);                  // Calc XADDR value from address
    am0805_register_write(AM0805_REG_EXTADDR_REG, xadd);    // Load the XADDR register
    reg_ram = (address & 0x3F) | 0x40;
    am0805_register_write(reg_ram, data);                   // Write the data
}


/**
 * @brief - Register Read
 */
void am0805_register_read( uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes )
{
  uint8_t tx_data[1];

  tx_data[0] = register_address;
  mw_twi_master_enable(m_AM0805AQ_TWI.twi_instance);
  mw_twi_master_tx( m_AM0805AQ_TWI.twi_instance,
                    m_AM0805AQ_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    NO_STOP );

  mw_twi_master_rx( m_AM0805AQ_TWI.twi_instance,
                    m_AM0805AQ_TWI.slave_address,
                    destination,
                    number_of_bytes );
  mw_twi_master_disable(m_AM0805AQ_TWI.twi_instance);
}


/**
 * @brief - Register Write
 */
void am0805_register_write( uint8_t register_address, uint8_t value )
{
  uint8_t tx_data[2];
  tx_data[0] = register_address;
  tx_data[1] = value;

  mw_twi_master_enable(m_AM0805AQ_TWI.twi_instance);
  mw_twi_master_tx( m_AM0805AQ_TWI.twi_instance,
                    m_AM0805AQ_TWI.slave_address,
                    tx_data,
                    sizeof(tx_data),
                    STOP );
  mw_twi_master_disable(m_AM0805AQ_TWI.twi_instance);
}


/**
 * @brief - Burst Register Write
 */
void am0805_burst_write(uint8_t * tx_data, uint8_t number_of_bytes)
{
  mw_twi_master_enable(m_AM0805AQ_TWI.twi_instance);
  mw_twi_master_tx( m_AM0805AQ_TWI.twi_instance,
                    m_AM0805AQ_TWI.slave_address,
                    tx_data,
                    number_of_bytes,
                    STOP );
  mw_twi_master_disable(m_AM0805AQ_TWI.twi_instance);
}


/**
 * @brief - Driver Initialization
 */
void am0805_init( external_device_config_t device_config )
{
  m_AM0805AQ_interface = device_config.communication;
  if ( device_config.communication == SPI_COMMUNICATION )
  {
    spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);

    if ( return_value.err_code != NRF_SUCCESS )
      APP_ERROR_CHECK(return_value.err_code);

    m_AM0805AQ_SPI_ID = return_value.device_id;

    MW_LOG_INFO("TMP75C device SPI Initialized");
  }

  if ( device_config.communication == TWI_COMMUNICATION )
  {
    mw_twi_master_init(device_config.twi_config);

    m_AM0805AQ_TWI.twi_instance  = device_config.twi_config.instance;
    m_AM0805AQ_TWI.slave_address = device_config.twi_config.slave_addr;

    MW_LOG_INFO("AM0805AQ device TWI Initialized");
  }

  m_AM0805AQ_initialized = true;
}
