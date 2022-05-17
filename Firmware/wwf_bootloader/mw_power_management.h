/*
 * mw_power_management.h
 *
 *  Created on: Oct 1, 2019
 *      Author: klockwood
 */

#ifndef MW_POWER_MANAGEMENT_H_
#define MW_POWER_MANAGEMENT_H_

typedef enum
{
  DEFAULT =0,
  ON,              // for ARTIC 3V3 and Master 3V3
  OFF,             // for all rails
  ON_BURST_MODE,   // 5V Burst Mode
  ON_FIXED_PWM,    // 5V Fixed PWM Mode
  ON_PWM_MODE,     // 1V8 PWM Mode
  ON_HYSTERESIS_MODE // 1V8 Hysteresis Mode
}rail_mode_t;

bool mw_set_master_3V3_rail( rail_mode_t mode );
bool mw_set_5V_rail( rail_mode_t mode );
bool mw_set_artic_3V3_rail( rail_mode_t mode );
bool mw_set_sensors_3V3_rail( rail_mode_t mode );
bool mw_set_1V8_rail( rail_mode_t mode );


void mw_power_management_init();

#endif /* MW_POWER_MANAGEMENT_H_ */
