/*
 * wwf_artic_payload.h
 *
 *  Created on: Oct 16, 2019
 *      Author: klockwood
 */

#ifndef WWF_ARTIC_PAYLOAD_H_
#define WWF_ARTIC_PAYLOAD_H_

#define PAYLOAD_PAYLOAD_SIZE_BITS  248

#define ACTIVITY_PAYLOAD_MAX      56

typedef struct
{
  uint16_t bear_id_and_profile;
  uint8_t pressure_counts;
  int8_t average_temperature;
  int8_t high_temperature;
  int8_t low_temperature;
  uint8_t battery_level;
  uint16_t wwf_activity_payload[ACTIVITY_PAYLOAD_MAX];
}wwf_profile_type_1_data_struct_t;


#endif /* WWF_ARTIC_PAYLOAD_H_ */
