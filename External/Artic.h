/*
 * artic.h
 *
 *  Created on: Oct 10, 2019
 *      Author: klockwood
 */

#ifndef ARTIC_ARTIC_H_
#define ARTIC_ARTIC_H_

#include "_mw_external_device.h"
#include "mw_spi.h"

#define ARTIC_FW_P_SIZE 7966*4
#define ARTIC_FW_X_SIZE 3372*3
#define ARTIC_FW_Y_SIZE 772*3


extern const volatile uint8_t artic_pmem[ARTIC_FW_P_SIZE];
extern const volatile uint8_t artic_xmem[ARTIC_FW_X_SIZE];
extern const volatile uint8_t artic_ymem[ARTIC_FW_Y_SIZE];


/*Firmware CRC values (should be checked after boot)*/
#define ARTIC_PMEM_FW_CRC 0x493982
#define ARTIC_XMEM_FW_CRC 0x32DBBA
#define ARTIC_YMEM_FW_CRC 0x65CC81

#define ARTIC_BURST_P_SEL  0
#define ARTIC_BURST_x_SEL  1
#define ARTIC_BURST_Y_SEL  2
#define ARTIC_BURST_IO_SEL 3

#define ARGOS_TX_BUFFER_MAX_SIZE 39 //3 byte length + 280 bits = 38 bytes.  Need 39 bytes to align to 24-bit word size

typedef enum
{
  ARTIC_P_MEM,
  ARTIC_X_MEM,
  ARTIC_Y_MEM,
  ARTIC_IO_MEM
} artic_mem_t;


//enum for total message length (including the header)
typedef enum
{
  ARGOS2_MESSAGE_56_BITS = 56, //24-bits
  ARGOS2_MESSAGE_88_BITS = 88, //56-bits
  ARGOS2_MESSAGE_120_BITS = 120, //88-bits
  ARGOS2_MESSAGE_152_BITS = 152, //120-bits
  ARGOS2_MESSAGE_184_BITS = 184, //152-bits
  ARGOS2_MESSAGE_216_BITS = 216, //184-bits
  ARGOS2_MESSAGE_248_BITS = 248, //216-bits
  ARGOS2_MESSAGE_280_BITS = 280  //248-bits
} argos2_message_size_t;


//current FW state
typedef struct
{
  bool idle;  //bit  0
  bool rx_in_progress; //bit 1
  bool tx_in_progress; //bit 2
  bool busy; //bit 3
  bool dsp2mcu_int1; //bit 22
  bool dsp2mcu_int2; //bit 23
} artic_fw_state_t;


//status from int1
typedef struct
{
  bool rx_valid_message;  //bit  4
  bool rx_satellite_detected; //bit 5
  bool tx_finished; //bit 6
  bool mcu_command_accepted; //bit 7
  bool crc_calculated; //bit 8
  bool idle_state; //bit 9
  bool rx_calibration; //bit 10
} artic_int1_t;

//status from int2
typedef struct
{
  bool rx_timeout; //bit 13
  bool satellite_timeout; //bit 14
  bool rx_buffer_overflow; //bit 15
  bool tx_invalid_message; //bit 16
  bool mcu_command_rejected; //bit 17
  bool mcu_command_overflow; //bit 18
  bool internal_error; //bit 21
} artic_int2_t;


/* --------------------------------------------*/
/* Artic Registers*/

/*Argos Commands
 * --------------------------------------------*/
/*Housekeeping commands (see AnSem ATRIC Datasheet page 20)*/
#define ARTIC_CMD_CLR_INT_1            0x80
#define ARTIC_CMD_CLR_INT_2            0xC0

/*Instruction commands (see AnSem ATRIC Datasheet page 18)*/
#define ARTIC_CMD_TX_1PKG              0x48 //0b0100 1000


/*Configuration Commands (see AnSem ATRIC Datasheet page 17)*/
//we'll only be using Argos 2 mode
#define ARTIC_CMD_SET_PTT_A2_TX_MODE   0x04

/* --------------------------------------------*/


/*Argos Registers/
 * --------------------------------------------*/
/*Memory X register locations (see AnSem ATRIC Datasheet page 21)*/
#define ARTIC_REG_ARGOS_CONFIG        0x0384  //read only register
#define ARTIC_REG_TX_PAYLOAD          0x0273
#define ARTIC_REG_TX_FREQ_ARGOS2      0x034F
#define ARTIC_REG_TCXO_WARMUP         0x036F  //warm-up time (in seconds, default is 10)
#define ARTIC_REG_TCXO_CONTROL        0x0370  //should be set 0
#define ARTIC_REG_CRC_RESULT          0x0371


/*Memory P register locations (see AnSem ATRIC Datasheet)*/
#define ARTIC_FIRMWARE_VERSION        0x0010  //read only register


/*Memory IO register locations */
#define ARTIC_FIRMWARE_STATUS         0x8018


#define BURST_READ                    0x10000


/* Msg Length Encoding */
#define MSG_LENGTH_CODE_24BITS        0x0380
#define MSG_LENGTH_CODE_56BITS        0x0583
#define MSG_LENGTH_CODE_88BITS        0x0785
#define MSG_LENGTH_CODE_120BITS       0x0986
#define MSG_LENGTH_CODE_152BITS       0x0B89
#define MSG_LENGTH_CODE_184BITS       0x0D8A
#define MSG_LENGTH_CODE_216BITS       0x0F8C
#define MSG_LENGTH_CODE_248BITS       0x118F



/* Modem Id - for payload header */
#define ARGOS_ID_BYTE_0               0x02
#define ARGOS_ID_BYTE_1               0xB9
#define ARGOS_ID_BYTE_2               0x78
#define ARGOS_ID_BYTE_3               0xAD    // 0xAD 0xBE 0xC7

/* --------------------------------------------*/


void artic_int1_clear();
void artic_int2_clear();

/**
 * @brief - Artic Command Write
 */
void artic_command_write(uint8_t command);


/**
 * @brief - Artic Write operation
 */
void artic_spi_write( uint8_t * data,
                      int length);


/**
 * @brief - Artic Burst Write operation
 */
void artic_burst_spi_write( short int address,
                            artic_mem_t mem,
                            uint8_t * data,
                            int length);


/**
 * @brief - Artic Burst Read operation
 */
void artic_burst_spi_read( short int address,
                           artic_mem_t mem,
                           uint8_t * data,
                           int length );


/**
 * @brief - Driver Interface Initialization
 */
#if USE_MW_SPIM
void artic_init( external_device_config_t device_config );
#else
void artic_init( mw_spi_config_t spi_config );
#endif



#endif /* ARTIC_ARTIC_H_ */
