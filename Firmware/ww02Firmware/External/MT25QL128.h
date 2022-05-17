// --------------------------------------------------------------------------------------------------------------------------------
//
//	proskida_MT25QL128.h
//
//	Author:		Philip Tsao
//	Revision:	1	-	First revision (01/05/2018)
//
//	MT25QL128 driver header file.
//
// --------------------------------------------------------------------------------------------------------------------------------

#ifndef PROSKIDA_MT25QL128_H_
#define PROSKIDA_MT25QL128_H_

#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"

#include "mw_external_component.h"

// MT25QL128 custom constants
#define MT25QL128_HEADER_START_ADDRESS			0x000000	// Starting address for custom information header
#define MT25QL128_HEADER_END_ADDRESS				0x000FFF	// Last byte address for the custom info header
#define MT25QL128_DATA_START_ADDRESS				0x001000	// Starting address for application/sensor data (header gets 1 sector - 4096 bytes)
#define MT25QL128_DATA_END_ADDRESS					0xFFFFFF	// Last byte address for application/sensor data (~16MB)
#define MT25QL128_SECTOR_LENGTH							4096		// Number of bytes in a sector
#define MT25QL128_BLOCK_32_LENGTH						32768		// Number of bytes in a 32kB Block
#define MT25QL128_BLOCK_64_LENGTH						65536		// Number of bytes in a 64kB Block
#define MT25QL128_PAGE_LENGTH								256			// Number of bytes per page
#define MT25QL128_HEADER_BYTE_LENGTH				11			// Combined length of all members in the header struct (MT25QL128HeaderInfo_t)

// Enums/structs
typedef struct
{
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t notificationType;
	uint32_t endByteAddress;
} MT25QL128HeaderInfo_t;

typedef struct
{
	uint16_t number_of_64kB_blocks;
	uint16_t number_of_32kB_blocks;
	uint16_t number_of_sectors;
} MT25QL128_content_breakdown_t;

// Function declarations
volatile uint8_t MT25QL128_read_status();
void MT25QL128_wait_for_write();
volatile bool MT25QL128_is_busy();
uint32_t MT25QL128_read_device_id();
void MT25QL128_write_enable();
void MT25QL128_chip_erase();
void MT25QL128_chip_erase_non_blocking();
void MT25QL128_block_erase_64kb(uint32_t startAddress);
void MT25QL128_block_erase_32kb(uint32_t startAddress);
void MT25QL128_sector_erase_4kb(uint32_t startAddress);
void MT25QL128_clear_status_register();
void MT25QL128_page_write(uint32_t startAddress, uint8_t* dataBufferStart, uint8_t bufferLength);
void MT25QL128_read(uint32_t startAddress, uint8_t* receiveBufferStart, uint8_t readLength);
void MT25QL128_write_custom_header(MT25QL128HeaderInfo_t headerInfo);
MT25QL128HeaderInfo_t MT25QL128_read_custom_header();
uint32_t MT25QL128_get_current_contents();
MT25QL128_content_breakdown_t MT25QL128_MT25QL128_content_analysis( uint32_t endAddress );
bool MT25QL128_write_read_check(uint32_t startAddress, uint8_t* dataBufferStart, uint8_t bufferLength);
void MT25QL128_auto_erase_sector(uint32_t currentWriteAddress);
void MT25QL128_fill_fake_data(uint32_t numOfBlocks, uint8_t blockSize, uint8_t notificationType);
void MT25QL128_read_and_print_rtt(uint32_t numOfBlocks, uint8_t blockSize);

void MT25QL128_uninit_spi();
void MT25QL128_init_spi( external_device_config_t device_config );

#endif /* PROSKIDA_MT25QL128_H_ */
