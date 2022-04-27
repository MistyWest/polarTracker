// --------------------------------------------------------------------------------------------------------------------------------
//
//	proskida_MT25QL128.c
//
//	Author:		Philip Tsao
//	Revision:	1	-	First revision (01/05/2018)
//
//	MT25QL128 driver source file.
//
// --------------------------------------------------------------------------------------------------------------------------------

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <nrfx.h>
#include "nrf_error.h"

#include "mw_external_component.h"
#include "mw_spi_master.h"
#include "MT25QL128.h"
#include "../CLI_Logging/mw_logging.h"

// MT25QL128 command constants
#define MT25QL128_NORMAL_READ								0x03
#define MT25QL128_PAGE_PROGRAM							0x02
#define MT25QL128_SECTOR_ERASE_4KB					0xD7
#define MT25QL128_BLOCK_ERASE_32KB					0x52
#define MT25QL128_BLOCK_ERASE_64KB					0xD8
#define MT25QL128_CHIP_ERASE								0xC7
#define MT25QL128_WRITE_ENABLE							0x06
#define MT25QL128_READ_STATUS								0x05
#define MT25QL128_DEEP_POWER_DOWN						0xB9
#define MT25QL128_RELEASE_POWER_DOWN				0xAB
#define MT25QL128_READ_ID										0x9F
#define MT25QL128_WRITE_STATUS							0x01

// Global variables
static uint8_t g_spiTransmitBuffer[10] = {0};
static uint8_t g_spiReceiveBuffer[10] = {0};

static volatile bool spi_drv_initialized = false;

static volatile uint8_t read_retVal = 0;

static uint8_t MT25QL128_SPI_ID;


// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_uninit_spi
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Function to uninitialize the SPI peripheral for our MT25QL128 chip to minimize power consumption.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_uninit_spi()
{
	nrf_drv_spi_uninit(&m_spi_master_0);
	SEGGER_RTT_printf(0, "SPI uninitialized!\n");
	spi_drv_initialized = false;
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_read_status
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Function to read the MT25QL128 status register and return the byte.
//
// --------------------------------------------------------------------------------------------------------------------------------
volatile uint8_t MT25QL128_read_status()
{
	//static uint8_t retVal = 0;
	g_spiTransmitBuffer[0] = MT25QL128_READ_STATUS;
	g_spiTransmitBuffer[1] = 0x00;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 2, g_spiReceiveBuffer, 2);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	read_retVal = g_spiReceiveBuffer[1];

	//SEGGER_RTT_printf(0, "MT25QL128 status register: 0x%02x!\n", read_retVal);

	return read_retVal;
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_wait_for_write
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Function which waits until the status register write-in-progress bit is clear (typically called after a write command).
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_wait_for_write()
{
	while ( 0x01 & MT25QL128_read_status() )
	{
		nrf_delay_ms(3);
	}
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_is_busy
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Function which returns whether MT25QL128 is busy or not.
//
// --------------------------------------------------------------------------------------------------------------------------------
volatile bool MT25QL128_is_busy()
{
	return ( MT25QL128_read_status() );
}





// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_read_device_id
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Function to read the MT25QL128 device ID bytes and return them in a concatenated 32-bit number.
//
// --------------------------------------------------------------------------------------------------------------------------------
uint32_t MT25QL128_read_device_id()
{
	uint32_t retVal = 0;
	g_spiTransmitBuffer[0] = MT25QL128_READ_ID;
	g_spiTransmitBuffer[1] = 0x00;
	g_spiTransmitBuffer[2] = 0x00;
	g_spiTransmitBuffer[3] = 0x00;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, g_spiReceiveBuffer, 4);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	retVal = (g_spiReceiveBuffer[1] << 16) | (g_spiReceiveBuffer[2] << 8) | (g_spiReceiveBuffer[3]);
	SEGGER_RTT_printf(0, "MT25QL128 device IDs: 0x%08x!\n", retVal);
	return retVal;
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_write_enable
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Sets the write bit so that we can program and erase the MT25QL128 chip.  This needs to be called prior to any erase/program
//	operation.  The write enable byte cannot be combined with another command buffer because it requires the CS line to go high
//	after a transfer in order to set the bit properly.
//
//	Note that there is an errata where a second redundant byte will be clocked out if the RX buffer length = 1 (even if rx buffer
//	is set to NULL).  There is a PPI/GPOITE work-around specified in the errata, or we can just set the rx length to 0 and it works.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_write_enable()
{
	g_spiTransmitBuffer[0] = MT25QL128_WRITE_ENABLE;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 1, NULL, 0);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	MT25QL128_wait_for_write();
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_chip_erase
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Erases the entire MT25QL128 chip memory.  For our 128Mbit MT25QL128, this can take 30-90 seconds according to the datasheet.
//
//	Note that there is an errata where a second redundant byte will be clocked out if the RX buffer length = 1 (even if rx buffer
//	is set to NULL).  There is a PPI/GPOITE work-around specified in the errata, or we can just set the rx length to 0 and it works.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_chip_erase()
{
	SEGGER_RTT_printf(0, "Erasing MT25QL128 chip...\n");
	MT25QL128_write_enable();
	g_spiTransmitBuffer[0] = MT25QL128_CHIP_ERASE;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 1, NULL, 0);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	MT25QL128_wait_for_write();
	SEGGER_RTT_printf(0, "*******************************\n");
	SEGGER_RTT_printf(0, "*******************************\n");
	SEGGER_RTT_printf(0, "MT25QL128 Chip Erase COMPLETE\n");
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_chip_erase_non_blocking
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Non-blocking version of the above - requires 30-90s to finish chip erase operation.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_chip_erase_non_blocking()
{
	SEGGER_RTT_printf(0, "Erasing MT25QL128 chip, non-blocking...\n");
	MT25QL128_write_enable();
	g_spiTransmitBuffer[0] = MT25QL128_CHIP_ERASE;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 1, NULL, 0);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_block_erase_64kb
//	Author:		Philip Tsao
//	Arguments:
//
// 	Erases a 64kB (k-byte) block of memory (actually 65536 bytes).  Can take 0.15-1s according to datasheet.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_block_erase_64kb(uint32_t startAddress)
{
	MT25QL128_write_enable();
	g_spiTransmitBuffer[0] = MT25QL128_BLOCK_ERASE_64KB;
	g_spiTransmitBuffer[1] = 0x0000FF & (startAddress >> 16);
	g_spiTransmitBuffer[2] = 0x0000FF & (startAddress >> 8);
	g_spiTransmitBuffer[3] = 0x0000FF & startAddress;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, NULL, 0);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	//MT25QL128_wait_for_write();
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_block_erase_32kb
//	Author:		Philip Tsao
//	Arguments:
//
// 	Erases a 32kB (k-byte) block of memory (actually 32768 bytes).  Can take 0.1-0.5s according to datasheet.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_block_erase_32kb(uint32_t startAddress)
{
	MT25QL128_write_enable();
	g_spiTransmitBuffer[0] = MT25QL128_BLOCK_ERASE_32KB;
	g_spiTransmitBuffer[1] = 0x0000FF & (startAddress >> 16);
	g_spiTransmitBuffer[2] = 0x0000FF & (startAddress >> 8);
	g_spiTransmitBuffer[3] = 0x0000FF & startAddress;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, NULL, 0);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	//MT25QL128_wait_for_write();
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_sector_erase_4kb
//	Author:		Philip Tsao
//	Arguments:
//
// 	Erases a 4kB (k-byte) sector of memory (actually 4096 bytes).  Can take 70-300ms according to datasheet.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_sector_erase_4kb(uint32_t startAddress)
{
	MT25QL128_wait_for_write();
	MT25QL128_write_enable();
	g_spiTransmitBuffer[0] = MT25QL128_SECTOR_ERASE_4KB;
	g_spiTransmitBuffer[1] = 0x0000FF & (startAddress >> 16);
	g_spiTransmitBuffer[2] = 0x0000FF & (startAddress >> 8);
	g_spiTransmitBuffer[3] = 0x0000FF & startAddress;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, NULL, 0);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	//MT25QL128_wait_for_write();
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_header_erase
//	Author:		Kevin Lockwood
//	Arguments:
//
// 	Erases a 4kB (k-byte) sector of memory (actually 4096 bytes).  Can take 70-300ms according to datasheet.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_header_erase(uint32_t startAddress)
{
	MT25QL128_sector_erase_4kb(startAddress);
	SEGGER_RTT_printf(0, "Waiting for Erase to complete\n");
	MT25QL128_wait_for_write();
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_clear_status_register
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Clears the status register to all zeros so everything is write-able.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_clear_status_register()
{
	MT25QL128_write_enable();
	g_spiTransmitBuffer[0] = MT25QL128_WRITE_STATUS;
	g_spiTransmitBuffer[1] = 0x00;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 2, NULL, 0);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
	MT25QL128_wait_for_write();
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_page_write
//	Author:		Philip Tsao
//	Arguments:
//
// 	Writes data to a page in memory starting from the start address.  0.2-0.8ms according to datasheet.  Sector being written to
//	must be erased prior to write otherwise this is ignored (can use sector, block, or chip erase).
//
//	Note this function can only write up to 255 bytes since nrf_drv_spi_transfer() can only send 255 bytes.  Also, a page write
//	operation will overflow to the top of the page if > 256 bytes are written, or if the write started in the middle and too many
//	bytes are written.  This can be avoided by always writing in half page 128 byte blocks to accommodate nrf_drv_spi_transfer()
//	usage and avoid overflow issues while maintaining relatively easy address tracking.
//
//	April 30, 2018 - added functionality to handle page overflow so we don't always have to write in 128 byte chunks.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_page_write(uint32_t startAddress, uint8_t* dataBufferStart, uint8_t bufferLength)
{
	// Find out how many bytes are left on the current memory page
	uint32_t bytesLeftInPage = MT25QL128_PAGE_LENGTH - (startAddress % MT25QL128_PAGE_LENGTH);

	// If data to write is greater than whats left in the page, break up the writes so we don't encounter page overflow
	if (bufferLength > bytesLeftInPage)
	{
		// Perform the first write and write the rest of the page memory
		MT25QL128_write_enable();
		g_spiTransmitBuffer[0] = MT25QL128_PAGE_PROGRAM;
		g_spiTransmitBuffer[1] = 0x0000FF & (startAddress >> 16);
		g_spiTransmitBuffer[2] = 0x0000FF & (startAddress >> 8);
		g_spiTransmitBuffer[3] = 0x0000FF & startAddress;
		nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
		nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, NULL, 0);
		nrf_drv_spi_transfer(&m_spi_master_0, dataBufferStart, bytesLeftInPage, NULL, 0);
		nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
		MT25QL128_wait_for_write();
		// Perform the second write and write the remaining bytes
		MT25QL128_write_enable();
		g_spiTransmitBuffer[0] = MT25QL128_PAGE_PROGRAM;
		g_spiTransmitBuffer[1] = 0x0000FF & ((startAddress + bytesLeftInPage) >> 16);
		g_spiTransmitBuffer[2] = 0x0000FF & ((startAddress + bytesLeftInPage) >> 8);
		g_spiTransmitBuffer[3] = 0x0000FF & (startAddress + bytesLeftInPage);
		nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
		nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, NULL, 0);
		nrf_drv_spi_transfer(&m_spi_master_0, (dataBufferStart + bytesLeftInPage), (bufferLength - bytesLeftInPage), NULL, 0);
		nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
		MT25QL128_wait_for_write();
	}
	// If data to write will fit in the page, just do a single MT25QL128 write
	else
	{
		MT25QL128_write_enable();
		g_spiTransmitBuffer[0] = MT25QL128_PAGE_PROGRAM;
		g_spiTransmitBuffer[1] = 0x0000FF & (startAddress >> 16);
		g_spiTransmitBuffer[2] = 0x0000FF & (startAddress >> 8);
		g_spiTransmitBuffer[3] = 0x0000FF & startAddress;
		nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
		nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, NULL, 0);
		nrf_drv_spi_transfer(&m_spi_master_0, dataBufferStart, bufferLength, NULL, 0);
		nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
		MT25QL128_wait_for_write();
	}
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_read
//	Author:		Philip Tsao
//	Arguments:
//
// 	Reads MT25QL128 data from the start address up to the read length (max of 255 bytes).
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_read(uint32_t startAddress, uint8_t* receiveBufferStart, uint8_t readLength)
{
	MT25QL128_wait_for_write();
	g_spiTransmitBuffer[0] = MT25QL128_NORMAL_READ;
	g_spiTransmitBuffer[1] = 0x0000FF & (startAddress >> 16);
	g_spiTransmitBuffer[2] = 0x0000FF & (startAddress >> 8);
	g_spiTransmitBuffer[3] = 0x0000FF & startAddress;
	nrf_gpio_pin_clear(project_board_config->MT25QL128_cs);
	nrf_drv_spi_transfer(&m_spi_master_0, g_spiTransmitBuffer, 4, NULL, 0);
	nrf_drv_spi_transfer(&m_spi_master_0, NULL, 0, receiveBufferStart, readLength);
	nrf_gpio_pin_set(project_board_config->MT25QL128_cs);
}



// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_write_custom_header
//	Author:		Philip Tsao
//	Arguments:
//
// 	Writes the MT25QL128 header data in the very first sector of MT25QL128.  This function handles the header sector erase for you.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_write_custom_header(MT25QL128HeaderInfo_t headerInfo)
{
	SEGGER_RTT_printf(0, "Updating MT25QL128 header info...\n");
	uint32_t address = MT25QL128_HEADER_START_ADDRESS;

	MT25QL128_write_enable();

	// Delete the header sector
	SEGGER_RTT_printf(0, "Erasing old MT25QL128 Header\n");
	MT25QL128_header_erase(MT25QL128_HEADER_START_ADDRESS);

	// Write the data
	uint8_t headerInfoBuffer[MT25QL128_HEADER_BYTE_LENGTH];
	headerInfoBuffer[0] = headerInfo.day;
	headerInfoBuffer[1] = headerInfo.month;
	headerInfoBuffer[2] = headerInfo.year;
	headerInfoBuffer[3] = headerInfo.hour;
	headerInfoBuffer[4] = headerInfo.minute;
	headerInfoBuffer[5] = headerInfo.second;
	headerInfoBuffer[6] = headerInfo.notificationType;
	headerInfoBuffer[7] = 0x000000FF & (headerInfo.endByteAddress >> 24);
	headerInfoBuffer[8] = 0x000000FF & (headerInfo.endByteAddress >> 16);
	headerInfoBuffer[9] = 0x000000FF & (headerInfo.endByteAddress >> 8);
	headerInfoBuffer[10] = 0x000000FF & (headerInfo.endByteAddress);
	MT25QL128_page_write(address, headerInfoBuffer, MT25QL128_HEADER_BYTE_LENGTH);

	SEGGER_RTT_printf(0, "MT25QL128 Header End Address:  %d...\n", headerInfo.endByteAddress);
}


// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_MT25QL128_content_analysis
//	Author:		Kevin Lockwood
//	Arguments:
//
//	Determines the breakdown of blocks and sectors of the current MT25QL128 contents.  This infomation can be used to more efficient
//  erasing procedures.
//
// --------------------------------------------------------------------------------------------------------------------------------
MT25QL128_content_breakdown_t MT25QL128_MT25QL128_content_analysis( uint32_t endAddress )
{
	MT25QL128_content_breakdown_t MT25QL128_contents;
	memset(&MT25QL128_contents, 0, sizeof(MT25QL128_contents));

	// Count the number of 64kB blocks
	MT25QL128_contents.number_of_64kB_blocks = endAddress / MT25QL128_BLOCK_64_LENGTH;
	if( MT25QL128_contents.number_of_64kB_blocks > 0 )
	{
		endAddress = endAddress - (MT25QL128_contents.number_of_64kB_blocks * MT25QL128_BLOCK_64_LENGTH);
	}

	// Count the number of 32kB blocks
	MT25QL128_contents.number_of_32kB_blocks = endAddress / MT25QL128_BLOCK_32_LENGTH;
	if( MT25QL128_contents.number_of_32kB_blocks > 0 )
	{
		endAddress = endAddress - (MT25QL128_contents.number_of_32kB_blocks * MT25QL128_BLOCK_32_LENGTH);
	}

	// Count the number of 4096B sectors
	MT25QL128_contents.number_of_sectors = endAddress / MT25QL128_SECTOR_LENGTH;
	if( (endAddress != 0) && (endAddress % MT25QL128_SECTOR_LENGTH != 0) )
	{
		MT25QL128_contents.number_of_sectors++;
	}

	//Since a Block Erase becomes faster then Sector erases
	if( MT25QL128_contents.number_of_sectors > 2 )
	{
		MT25QL128_contents.number_of_sectors = 0;
		MT25QL128_contents.number_of_32kB_blocks++;
	}

	return MT25QL128_contents;
}


// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_get_current_contents
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Reads the MT25QL128 header data returns current contents
//
// --------------------------------------------------------------------------------------------------------------------------------
uint32_t MT25QL128_get_current_contents()
{
	SEGGER_RTT_printf(0, "Reading MT25QL128 header info...\n");
	uint32_t address = MT25QL128_read_custom_header().endByteAddress;

	SEGGER_RTT_printf(0, "MT25QL128 Start Address is %d...\n", address);
	return address;
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_read_custom_header
//	Author:		Philip Tsao
//	Arguments:	None
//
// 	Reads the MT25QL128 header data and returns a header info struct.
//
// --------------------------------------------------------------------------------------------------------------------------------
MT25QL128HeaderInfo_t MT25QL128_read_custom_header()
{
	SEGGER_RTT_printf(0, "Reading MT25QL128 header info...\n");
	uint32_t address = MT25QL128_HEADER_START_ADDRESS;
	MT25QL128HeaderInfo_t headerInfo;

	// Read the data
	uint8_t headerInfoBuffer[MT25QL128_HEADER_BYTE_LENGTH];
	memset(headerInfoBuffer, 0, sizeof(headerInfoBuffer));
	MT25QL128_read(address, headerInfoBuffer, MT25QL128_HEADER_BYTE_LENGTH);
	headerInfo.day = headerInfoBuffer[0];
	headerInfo.month = headerInfoBuffer[1];
	headerInfo.year = headerInfoBuffer[2];
	headerInfo.hour = headerInfoBuffer[3];
	headerInfo.minute = headerInfoBuffer[4];
	headerInfo.second = headerInfoBuffer[5];
	headerInfo.notificationType = headerInfoBuffer[6];
	headerInfo.endByteAddress = ((uint32_t)headerInfoBuffer[7] << 24) | ((uint32_t)headerInfoBuffer[8] << 16) | ((uint32_t)headerInfoBuffer[9] << 8) | ((uint32_t)headerInfoBuffer[10]);

	// Return struct
	return headerInfo;
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_read_section_of_memory
//	Author:		Kevin Lockwood
//	Arguments:	None
//
// 	Reads a section of MT25QL128
//
// --------------------------------------------------------------------------------------------------------------------------------
MT25QL128HeaderInfo_t MT25QL128_read_section_of_memory( uint32_t address )
{
	SEGGER_RTT_printf(0, "Reading MT25QL128 the follow address %x...\n", address);
	MT25QL128HeaderInfo_t headerInfo;

	// Read the data
	uint8_t headerInfoBuffer[MT25QL128_HEADER_BYTE_LENGTH];
	memset(headerInfoBuffer, 0, sizeof(headerInfoBuffer));
	MT25QL128_read(address, headerInfoBuffer, MT25QL128_HEADER_BYTE_LENGTH);
	headerInfo.day = headerInfoBuffer[0];
	headerInfo.month = headerInfoBuffer[1];
	headerInfo.year = headerInfoBuffer[2];
	headerInfo.hour = headerInfoBuffer[3];
	headerInfo.minute = headerInfoBuffer[4];
	headerInfo.second = headerInfoBuffer[5];
	headerInfo.notificationType = headerInfoBuffer[6];
	headerInfo.endByteAddress = ((uint32_t)headerInfoBuffer[7] << 24) | ((uint32_t)headerInfoBuffer[8] << 16) | ((uint32_t)headerInfoBuffer[9] << 8) | ((uint32_t)headerInfoBuffer[10]);

	// Return struct
	return headerInfo;
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_write_read_check
//	Author:		Philip Tsao
//	Arguments:
//
//	Test function to write to MT25QL128 and then read it out and compare to make sure data read back is correct - returns true if
//	the data read back is the same as what was given to write.  This is a sanity check used during development but could be used
//	to handle write issues at the cost of time.
//
//	Sector must be erased before this will work like in a normal write scenario.
//
// --------------------------------------------------------------------------------------------------------------------------------
bool MT25QL128_write_read_check(uint32_t startAddress, uint8_t* dataBufferStart, uint8_t bufferLength)
{
	bool returnVal = true;
	uint8_t errorNum = 0;
	uint8_t receiveDataBuffer[255];

	// Write the data to MT25QL128
	MT25QL128_page_write(startAddress, dataBufferStart, bufferLength);

	// Read the data out
	MT25QL128_read(startAddress, receiveDataBuffer, bufferLength);

	// Check between buffers for errors
	for (int i=0; i<bufferLength; i++)
	{
		if (dataBufferStart[i] != receiveDataBuffer[i])
		{
			errorNum++;
			returnVal = false;
		}
	}
	SEGGER_RTT_printf(0, "MT25QL128_write_read_check() complete!  Attempted to write %d bytes, number of inaccurate bytes read: %d\n", bufferLength, errorNum);
	return returnVal;
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_auto_erase_sector
//	Author:		Philip Tsao
//	Arguments:
//
//	Determines if the current write address is in a new sector and if so, it will erase the sector (so we can achieve dynamic
//	erase/writing without having to erase the whole chip before writing).  This looks for address which are multiples of sector
//	lengths (each sector is 4096 bytes).  If the current address is in a new sector (a multiple of 4096), it will erase that sector.
//	Sector erase was chosen as opposed to block erases to minimize erase time so this can be used for dynamic erase/write operations.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_auto_erase_sector(uint32_t currentWriteAddress)
{
	uint32_t modResult;
	modResult = currentWriteAddress % MT25QL128_SECTOR_LENGTH;
	if (modResult == 0)
	{
		SEGGER_RTT_printf(0, "Erasing sector: %d\n", currentWriteAddress);
		MT25QL128_sector_erase_4kb(currentWriteAddress);
	}
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_fill_fake_data
//	Author:		Philip Tsao
//	Arguments:
//
//	Test function to fill MT25QL128 with a certain number of fake data bytes.  For development/debugging.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_fill_fake_data(uint32_t numOfBlocks, uint8_t blockSize, uint8_t notificationType)
{
	uint8_t writeDataBuffer[255];														// Data buffer to write
	uint32_t address = MT25QL128_DATA_START_ADDRESS;
	uint32_t endByteAddress = MT25QL128_DATA_START_ADDRESS + (numOfBlocks*blockSize);		// Last address to write based on total # of bytes

	// If end byte address is greater than MT25QL128 capacity, return and throw an error
	if (endByteAddress > MT25QL128_DATA_END_ADDRESS)
	{
		SEGGER_RTT_printf(0, "Requested amount of data will exceed MT25QL128 capacity!  Aborting fill fake data...\n");
		return;
	}

	// Create some fake data
	for (int i=0; i<blockSize; i++)
	{
		writeDataBuffer[i] = i;
	}

	// Erase chip (could also do auto sector erase before writes instead)
	MT25QL128_chip_erase();

	// Write header sector data
	MT25QL128HeaderInfo_t headerInfo;
	headerInfo.day = 30;
	headerInfo.month = 6;
	headerInfo.year = 18;
	headerInfo.hour = 12;
	headerInfo.minute = 60;
	headerInfo.second = 0;
	headerInfo.notificationType = notificationType;
	headerInfo.endByteAddress = endByteAddress;
	MT25QL128_write_custom_header(headerInfo);

	// Fill MT25QL128 chip with fake data
	while (address < endByteAddress)
	{
		MT25QL128_write_read_check(address, writeDataBuffer, blockSize);
		address += blockSize;
	}
}

// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_read_and_print_rtt
//	Author:		Philip Tsao
//	Arguments:
//
//	Test function to read bytes from MT25QL128 and print them out to RTT for logging/debugging purposes.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_read_and_print_rtt(uint32_t numOfBlocks, uint8_t blockSize)
{
	uint8_t receiveDataBuffer[255];
	uint32_t address = MT25QL128_HEADER_START_ADDRESS;
	uint32_t endAddress = MT25QL128_DATA_START_ADDRESS + numOfBlocks*blockSize;

	// Read and print data
	while (address < endAddress)
	{
		SEGGER_RTT_printf(0, "Address: %d, ", address);
		MT25QL128_read(address, receiveDataBuffer, blockSize);
		address += blockSize;
		for (int i=0; i<blockSize; i++)
		{
			SEGGER_RTT_printf(0, "%d ", receiveDataBuffer[i]);				// Make sure RTT buffer is large enough in sdk_config.h
		}
		SEGGER_RTT_printf(0, "\n");
		vTaskDelay(100);													// Give the RTT buffer some time to finish
	}
}


// --------------------------------------------------------------------------------------------------------------------------------
//
//	Name:		MT25QL128_init_spi
//	Author:		Philip Tsao
//	Arguments:
//
// 	Function to initialize the SPI peripheral for our MT25QL128 chip.
//
//	Note we do not tie a CS pin to the spi driver.  This means we have to toggle the CS line manually and allows more flexibility.
//
// --------------------------------------------------------------------------------------------------------------------------------
void MT25QL128_init_spi( external_device_config_t device_config )
{
	if( device_config.communication == SPI_COMMUNICATION )
	{
		mw_spim_device_config_t MT25QL128_device;
		spi_device_init_return_t return_value;

		memcpy( &MT25QL128_device, &device_config.spi_config, sizeof(mw_spim_device_config_t));
		return_value = mw_spi_master_device_init(MT25QL128_device);

		if(return_value.err_code != NRF_SUCCESS) APP_ERROR_CHECK(return_value.err_code);

		MT25QL128_SPI_ID = return_value.device_id;

		MW_LOG_INFO("ICM20649_device SPI Initialized");
	}

	if ( device_config.communication == TWI_COMMUNICATION )
	{
		mw_twim_device_config_t MT25QL128_device;
		memcpy( &MT25QL128_device, &device_config.twi_config, sizeof(mw_twim_device_config_t));

		mw_twi_master_init(MT25QL128_device);

		MW_LOG_INFO("ICM20649_device TWI Initialized");
	}

	// Clear the MT25QL128 status register so write commands can work
	MT25QL128_clear_status_register();
}
