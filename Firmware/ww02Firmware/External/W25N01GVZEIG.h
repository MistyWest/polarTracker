

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "boards.h"

#include "../CLI_Logging/mw_logging.h"
#include "_mw_external_device.h"


//SE_NOTE... decision to not check for ECC (ignore the error and read anyway)
//uncomment this to add ECC checking
//#define CHECK_ECC

#define W25XXX_NUM_BLOCKS         1024
#define W25XXX_PAGES_PER_BLOCK    64

#define W25XXX_PAGE_SIZE          2048U
#define W25XXX_BLOCK_SIZE         (W25XXX_PAGES_PER_BLOCK * W25XXX_PAGE_SIZE)
#define W25XXX_DIE_SIZE           (W25XXX_NUM_BLOCKS * W25XXX_BLOCK_SIZE)

#define W25XXX_WRITE_BUFF_SIZE    128
#define W25XXX_READ_BUFF_SIZE     W25XXX_WRITE_BUFF_SIZE

//flash operations
enum w25xxx_op_code {
  W25XXX_OP_RST = 0xff,
  W25MXX_OP_DIE_SELECT = 0xc2,
  W25XXX_OP_READ_JEDEC_ID = 0x9f,
  W25XXX_OP_READ_REG = 0x05,
  W25XXX_OP_WRITE_REG = 0x01,
  W25XXX_OP_WRITE_ENABLE = 0x06,
  W25XXX_OP_WRITE_DISABLE = 0x04,
  W25XXX_OP_BBM_SWAP_BLOCKS = 0xa1,
  W25XXX_OP_BBM_READ_LUT = 0xa5,
  W25XXX_OP_BBM_READ_LAST_ECC_FAIL_ADDR = 0xa9,
  W25XXX_OP_PROG_DATA_LOAD = 0x02,
  W25XXX_OP_PROG_RAND_DATA_LOAD = 0x84,
  W25XXX_OP_PROG_EXECUTE = 0x10,
  W25XXX_OP_BLOCK_ERASE = 0xd8,
  W25XXX_OP_PAGE_DATA_READ = 0x13,
  W25XXX_OP_READ = 0x03,
};


//failure codes
enum w25xxx_return_code {
  W25XXX_SUCCESS,
  W25XXX_ADDRESS_OUT_RANGE,
  W25XXX_ADDRESS_ECC_HARD,
  W25XXX_PROG_EXEC_FAIL,
  W25XXX_INVALID_ARG,
  W25XXX_ERASE_FAIL,
  W25XXX_INIT_FAIL,
  W25XXX_BB_FULL,
};

typedef enum w25xxx_return_code w25xxx_return_code_t;

enum w25xxx_reg {
  W25XXX_REG_PROT = 0xa0, /* Protection register */
  W25XXX_REG_CONF = 0xb0, /* Configuration register */
  W25XXX_REG_STAT = 0xc0, /* Status register */
};


//register fields within registers
#define W25XXX_REG_CONF_BUF (1 << 3)
#define W25XXX_REG_CONF_ECCE (1 << 4)

#define W25XXX_REG_STAT_BUSY (1 << 0)
#define W25XXX_REG_STAT_WEL (1 << 1)
#define W25XXX_REG_STAT_EFAIL (1 << 2)
#define W25XXX_REG_STAT_PFAIL (1 << 3)
#define W25XXX_REG_STAT_ECC0 (1 << 4)
#define W25XXX_REG_STAT_ECC1 (1 << 5)
#define W25XXX_REG_STAT_LUTF (1 << 6)

//size ofthe bad block look-up table
#define W25XXX_BB_LUT_SIZE 20


struct w25xxx_op {
  enum w25xxx_op_code op_code;
  uint8_t dummy_tx_bytes;  //number of bytes before TX dat
  size_t tx_len;
  uint32_t tx_data; //TX data
  uint8_t dummy_rx_bytes; //number of bytes before RX data
  size_t rx_len;
  uint8_t *rx_data;
};

#define W25XXX_OP_DEFAULT_CONFIG \
{                                \
    .op_code        = 0,         \
    .dummy_tx_bytes = 0,         \
    .tx_len         = 0,         \
    .tx_data        = 0,         \
    .dummy_rx_bytes = 0,         \
    .rx_len         = 0,         \
    .rx_data        = NULL,      \
}

//struct used for bad block management
typedef struct w25xxx_bb_lut_entry {
  uint32_t invalid : 1;
  uint32_t enable : 1;
  uint16_t lba : 10;
  uint16_t pba;
} w25xxx_bb_lut_entry_t;


typedef struct w25xxx_bb_lut {
    w25xxx_bb_lut_entry_t e[W25XXX_BB_LUT_SIZE];
} w25xxx_bb_lut_t;


/**
 * @brief - Function to init the flash.  Searches for bad blocks
 *          and programs the bad block look-up table.
 */
void w25xxx_init();

/**
 * @brief - Function to write to the flash.  If a full page is written
 *          if will program into the flash.  If a partial page is written
 *          it will wait for the page to fill up before writting.
 *          w25xxx_trigger_prog_exec() is called to program a partial page.
 *          It's required to erase flash with vfs_dev_w25xxx_erase()
 *          before writting.
 *
 * @param off - byte address to start wrtting at
 * @param len - length of bytes to write
 * @param src - pointer to byte array to write
 *
 * @return - w25xxx_return_code_t value
 *
 **/
w25xxx_return_code_t vfs_dev_w25xxx_write(size_t off,
                                          size_t len,
                                          const void *src);

/**
 * @brief - Function to read from flash
 *
 * @param off - byte address to start reading at
 * @param len - length of bytes to read
 * @param src - pointer to array to read into
 *
 *  @return - w25xxx_return_code_t value
 */
w25xxx_return_code_t vfs_dev_w25xxx_read(size_t off,
                                         size_t len,
                                         void *dst);
/**
 * @brief - Function to erase a block (or many blocks) from flash
 *
 * @param off - byte address to start erasing at (must be aligned to a block address!)
 * @param len - length of bytes to erase (must be aligned to a block address!)
 *
 * @return - w25xxx_return_code_t value
 */
w25xxx_return_code_t vfs_dev_w25xxx_erase(size_t off,
                                          size_t len);

/**
 * @brief - Function to write a partially written page buffer into flash
 *
 * @param off - the address of the last byte to written into the page buffer
 *
 * @return - w25xxx_return_code_t value
 */
w25xxx_return_code_t w25xxx_trigger_prog_exec(size_t off);




