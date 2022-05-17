
/*
 * mw_artic_thread.h
 *
 *  Code for configuring the ARTIC (aka ARGOS) chip.
 *  Used for creating Argos-2 compatible messages.
 *
 *  Created on: January 7, 2019
 *      Author: Sean Edmond
 */
#ifndef MW_ARTIC_THREAD_H_
#define MW_ARTIC_THREAD_H_

#include "stdint.h"

#include "Artic.h"
#include "wwf_device_settings.h"

#define ARTIC_SUCCESS                  0
#define ARTIC_ERROR_INVALID_PARAM      7
#define ARTIC_ERROR_INVALID_STATE      8

#ifndef DEFAULT_TRANSMISSION_REPITIIONS
#define DEFAULT_TRANSMISSION_REPITIIONS 8//20
#endif

#ifndef DEFAULT_TRANSMISSION_INTERVAL
#define DEFAULT_TRANSMISSION_INTERVAL   (90 * 1000)  /* 90 seconds */
#endif

typedef enum
{
  UNKNOWN_STATE,
  POWER_ON,
  POWER_OFF
}artic_device_power_states_t;

typedef struct
{
  bool interface_initialized;
  artic_device_power_states_t power_state;
  uint8_t tcxo_warm_up_time;
}m_artic_device_t;

uint32_t artic_power_on_and_program();
//static void artic_power_on_and_program(void * arg);

void artic_power_down();

void artic_power_off_pa(void);

void artic_power_on_pa(void);

bool artic_is_use();

uint8_t update_argos_user_id ( const uint8_t * data, uint8_t length );

bool test_artic_power_up_program_power_down();

uint8_t artic_transmit_argos2a_1_pkt(uint8_t * payload, argos2_message_size_t argos2_message_size);

uint32_t artic_read_firmware_version(void);

bool artic_thread_ready();

void artic_thread_init();

#endif /* MW_ARTIC_THREAD_H_ */
