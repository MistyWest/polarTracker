/*
 * mw_battery.h
 *
 *  Created on: Apr 1, 2019
 *      Author: klockwood
 */

#ifndef BATTERY_MW_BATTERY_H_
#define BATTERY_MW_BATTERY_H_

#include "stdio.h"

#define BATT_LEVEL_97       (4.150f)
#define BATT_LEVEL_94       (4.050f)
#define BATT_LEVEL_89       (4.000f)
#define BATT_LEVEL_74       (3.900f)
#define BATT_LEVEL_50       (3.800f)
#define BATT_LEVEL_24       (3.700f)
#define BATT_LEVEL_9        (3.500f)
#define BATT_LEVEL_3        (3.400f)
#define BATT_LEVEL_1        (3.300f)
#define BATT_LEVEL_0        (3.000f)

#define OFFSET_HI_MID_TEMP  (0.05f)
#define OFFSET_MID_TEMP     (0.1f)
#define OFFSET_LO_MID_TEMP  (0.2f)
#define OFFSET_LO_TEMP      (0.4f)
#define OFFSET_LOWER_TEMP   (0.6f)
#define OFFSET_LOWEST_TEMP  (0.75f)


typedef enum
{
  MW_BATTERY_CHARGER_IDLE = 0xB0,
  MW_BATTERY_CHARGING = 0xB1,
  MW_BATTERY_CHARGE_STARTING = 0xB2,
  MW_BATTERY_CHARGE_COMPLETE = 0xB3,
  MW_BATTERY_USB_UNPLUGGED = 0xB4
}mw_battery_charger_states_t;


/**
 * @brief - Check Charge Status
 */
mw_battery_charger_states_t mw_battery_check_charge_status();

/**
 * @brief - Calculate Battery Percentage
 */
uint8_t mw_battery_calculate_percentage( float battery_level, float temperature_level );

/**
 *
 * @brief initialize battery module with voltage dividor ratio.  If none pass a value of 1
 */
void mw_battery_initialize( float compensation_resistors, uint32_t charge_control, uint32_t charge_status, uint32_t usb_power );

#endif /* BATTERY_MW_BATTERY_H_ */
