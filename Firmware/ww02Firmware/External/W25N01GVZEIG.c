#include "W25N01GVZEIG.h"
#include "FreeRTOS_includes.h"

//spi device ID assigned by template
uint8_t w25_xxx_spi_dev_id;

//Number of reserved block from bad block management
uint16_t num_reserved_blocks;


void w25xxx_read_bb_lut(w25xxx_bb_lut_entry_t bb_lut[W25XXX_BB_LUT_SIZE]);
w25xxx_return_code_t w25xxx_bb_program(w25xxx_bb_lut_entry_t bb_lut[W25XXX_BB_LUT_SIZE]);



/**
 * @brief - wrapper from SPI transaction
 *
 * @param tx_len - length of TX bytes
 * @param tx_data - buffer for TX bytes
 * @param rx_len - length of RX bytes
 * @param rx_data - buffer for RX bytes
 *
 */
void w25xxx_txn(size_t tx_len,
                uint8_t *tx_data,
                size_t rx_len,
                uint8_t *rx_data)
{

    mw_spi_master_transfer(w25_xxx_spi_dev_id,
                            SPI_NO_RW_BIT,
                            tx_data,
                            tx_len,
                            rx_data,
                            rx_len);

}

/**
 * @brief - wrapper to execute on of the flash operations
 *
 * @param op - Struct describing the flash operation
 *
 */
void w25xxx_exec_instruction(struct w25xxx_op op)
{
    //op code (1 byte), dummy bytes before TX data, TX data
    size_t tx_length = (1 + op.dummy_tx_bytes + op.tx_len + op.dummy_rx_bytes + op.rx_len );

    //TODO... consider removing malloc?
//    uint8_t *tx_buff = malloc( tx_length * sizeof(uint8_t));
//    uint8_t *rx_buff = malloc( tx_length * sizeof(uint8_t));
    static uint8_t tx_buff[W25XXX_WRITE_BUFF_SIZE + 4];
    static uint8_t rx_buff[W25XXX_READ_BUFF_SIZE + 4];

    tx_buff[0] = op.op_code;


    for(int i=0;i<op.tx_len;i++)
    {

      if(i+1+op.dummy_tx_bytes >= (W25XXX_WRITE_BUFF_SIZE+3) )
      {
        MW_LOG_ERROR("Error Error Fire Yo!!!!");
      }
        tx_buff[i+1+op.dummy_tx_bytes] = (op.tx_data >> (op.tx_len-i-1)*8) & 0xff;
    }

    w25xxx_txn(tx_length,
                tx_buff,
                tx_length,
                rx_buff);

    //set rx_data buffer
    memcpy(op.rx_data,
           &(rx_buff[1 + op.dummy_tx_bytes + op.tx_len + op.dummy_rx_bytes]),
           op.rx_len);


//    free(tx_buff);
//    free(rx_buff);

}

/**
 * @brief - Function to init the flash.  Searches for bad blocks
 *          and programs the bad block look-up table.
 */
void w25xxx_init()
{
    w25xxx_bb_lut_entry_t bb_lut[W25XXX_BB_LUT_SIZE];
    uint8_t jedec_id[3];
    external_device_config_t device_config;
    int i;
    uint16_t lowest_lut_pba;

    device_config.communication = SPI_COMMUNICATION;
    device_config.spi_config.spi_instance = 1;
    device_config.spi_config.mode = NRF_SPIM_MODE_3;
    device_config.spi_config.bit_order = NRF_SPI_BIT_ORDER_MSB_FIRST;
    device_config.spi_config.frequency = NRF_SPI_FREQ_8M;
    device_config.spi_config.miso_pin = SPIM1_MISO_PIN;
    device_config.spi_config.mosi_pin = SPIM1_MOSI_PIN;
    device_config.spi_config.ss_pin = SPIM1_SS_PIN;
    device_config.spi_config.sck_pin = SPIM1_SCK_PIN;
    device_config.spi_config.ss_active_high = false;
    device_config.spi_config.handler = NULL;
    device_config.spi_config.irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY;

    spi_device_init_return_t return_value = mw_spi_master_device_init(device_config.spi_config);


    if(return_value.err_code != NRF_SUCCESS)
    {
        MW_LOG_ERROR("Error during SPI initialization");
    }
    else
    {
        MW_LOG_INFO("Initializated");
        w25_xxx_spi_dev_id = return_value.device_id;
    }


    //read JEDEC to confirm that SPI communication is working
    struct w25xxx_op jid_op = W25XXX_OP_DEFAULT_CONFIG;
    jid_op.op_code = W25XXX_OP_READ_JEDEC_ID;
    jid_op.dummy_tx_bytes = 1;
    jid_op.rx_len = 3;
    jid_op.rx_data = jedec_id;
    w25xxx_exec_instruction(jid_op);

    if( (jedec_id[0] != 0xEF) || (jedec_id[1] != 0xAA) || (jedec_id[2] != 0x21) )
    {
        MW_LOG_ERROR("FLASH JEDEC not correct!!!");
        //TODO, consider throwing error code
        MW_LOG_ERROR("JEDEC : %x, %x, %x", jedec_id[0], jedec_id[1], jedec_id[2]);
    }
    else
    {
        MW_LOG_INFO("Flash found");
    }


    //reset the device
    struct w25xxx_op rst_op = W25XXX_OP_DEFAULT_CONFIG;
    rst_op.op_code = W25XXX_OP_RST;
    w25xxx_exec_instruction(rst_op);

    //delay 1 ms after reset (requirement 500us)
    vTaskDelay(1);

    //remove register write protection
    struct w25xxx_op prot_reg_op = W25XXX_OP_DEFAULT_CONFIG;
    prot_reg_op.op_code = W25XXX_OP_WRITE_REG;
    prot_reg_op.tx_len = 2;
    prot_reg_op.tx_data = (W25XXX_REG_PROT << 8) | (0 & 0xFF);

    w25xxx_exec_instruction(prot_reg_op);

    //search for bad block and program the look-up table
    w25xxx_bb_program(bb_lut);

    //change the available size based on the last bad block
    lowest_lut_pba = W25XXX_NUM_BLOCKS;
    for(i=0;i<W25XXX_BB_LUT_SIZE;i++)
    {
        if(bb_lut[i].enable == true &&
           bb_lut[i].invalid == false)
        {
            if(bb_lut[i].pba < lowest_lut_pba)
            {
                lowest_lut_pba = bb_lut[i].pba;
            }
        }
    }

    num_reserved_blocks = W25XXX_NUM_BLOCKS-lowest_lut_pba;

    if( num_reserved_blocks > 20 )
    {
      MW_LOG_INFO("Bad block error");
    }

}

/**
 * @brief - Function to uninit
 */
void w25xxx_uninit()
{
    mw_spi_master_driver_uninit(w25_xxx_spi_dev_id);
}


/**
 * @brief - Function map an offset to a page and page offset
 *
 * @param off - Byte address offset
 * @param page_num - return value of page number given "off"
 * @param page_off - return value of the page offset for the page_num given "off"
 */
static bool w25xxx_map_page(size_t off,
                            uint16_t *page_num,
                            uint16_t *page_off)
{

    bool res = false;
    uint16_t block_num;

    *page_num = off / W25XXX_PAGE_SIZE;
    block_num = off / W25XXX_BLOCK_SIZE;

    if(block_num > W25XXX_NUM_BLOCKS-num_reserved_blocks)
    {
        MW_LOG_ERROR("ERROR, trying to address outside of usable blocks");
        res = false;
    }
    else
    {
        *page_off = off % W25XXX_PAGE_SIZE;
        res = true;
    }

    return res;
}

/**
 * @brief - Function for reading a page into the page buffer
 *
 * @param page_num - page number
 */
w25xxx_return_code_t w25xxx_page_data_read(uint16_t page_num)
{

    uint8_t reg;

    //submit instruction to read page data into buffer
    struct w25xxx_op page_read_op = W25XXX_OP_DEFAULT_CONFIG;
    page_read_op.op_code = W25XXX_OP_PAGE_DATA_READ;
    page_read_op.dummy_tx_bytes = 1;
    page_read_op.tx_len = 2;
    page_read_op.tx_data = page_num;
    w25xxx_exec_instruction(page_read_op);

    //read the status register until BUSY status is cleared
    struct w25xxx_op status_op = W25XXX_OP_DEFAULT_CONFIG;
    status_op.op_code = W25XXX_OP_READ_REG;
    status_op.tx_len = 1;
    status_op.tx_data = W25XXX_REG_STAT;
    status_op.rx_len = 1;
    status_op.rx_data = &reg;

    do
    {
        w25xxx_exec_instruction(status_op);
    } while(reg & W25XXX_REG_STAT_BUSY);

    //check for ECC errors
    #ifdef CHECK_ECC
    //read status register again
    w25xxx_exec_instruction(status_op);

    if (reg & (W25XXX_REG_STAT_ECC1 | W25XXX_REG_STAT_ECC0)) {
        bool hard = (reg & W25XXX_REG_STAT_ECC1);
        MW_LOG_DEBUG("%s ECC error @ page %u", (hard ? "Hard" : "Soft"),
            page_num);
        if (hard)
        {
            MW_LOG_ERROR("HARD ECC ERROR on page : %u", page_num);
            return(W25XXX_ADDRESS_ECC_HARD);
        }
    }
    #endif

    return(W25XXX_SUCCESS);
}

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
                                         void * dst)
{

    w25xxx_return_code_t res;
    uint8_t *dp = (uint8_t *) dst;
    uint16_t page_num, page_off;
    size_t rd_len;
    struct w25xxx_op read_op = W25XXX_OP_DEFAULT_CONFIG;
    uint8_t rxn_buf[W25XXX_READ_BUFF_SIZE];

    while (len > 0)
    {

        if (!w25xxx_map_page(off, &page_num, &page_off))
        {
            return(W25XXX_ADDRESS_OUT_RANGE);
        }

        rd_len = MIN(len, W25XXX_PAGE_SIZE - page_off);

        //trigger the command to get the data into the buffer
        if ((res = w25xxx_page_data_read(page_num)))
        {
            return(res);
        }

        //read from the buffer
        read_op.op_code = W25XXX_OP_READ;
        read_op.tx_len = 2;
        read_op.dummy_rx_bytes = 1;
        read_op.rx_data = rxn_buf;

        //write into the page buffer in W25XXX_WRITE_BUFF_SIZE byte chunks
        for (size_t rxn_off = 0, rxn_len = 0; rxn_off < rd_len;
            rxn_off += rxn_len)
        {
            rxn_len = MIN(W25XXX_READ_BUFF_SIZE, rd_len - rxn_off);

            read_op.rx_len = rxn_len;
            read_op.tx_data = page_off + rxn_off;

            w25xxx_exec_instruction(read_op);

            //may want to not use exec instruction so this is more efficient
            memcpy(dp, rxn_buf, rxn_len);
            dp += rxn_len;
        }

        off += rd_len;
        len -= rd_len;
    }

    return(W25XXX_SUCCESS);
}

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
                                          const void *src)
{
    const uint8_t *dp = (const uint8_t *) src;
    uint16_t page_num, page_off;
    struct w25xxx_op prog_exec_op = W25XXX_OP_DEFAULT_CONFIG;
    struct w25xxx_op write_en_op = W25XXX_OP_DEFAULT_CONFIG;
    struct w25xxx_op status_op = W25XXX_OP_DEFAULT_CONFIG;
    uint8_t txn_buf[3 + W25XXX_WRITE_BUFF_SIZE];
    uint8_t reg;


    //while loop will loop over multiple pages to write the
    //entire payload
    while (len > 0)
    {
        //write enable must be set after every page write
        write_en_op.op_code = W25XXX_OP_WRITE_ENABLE;
        w25xxx_exec_instruction(write_en_op);

        if (!w25xxx_map_page(off, &page_num, &page_off))
        {
            return(W25XXX_ADDRESS_OUT_RANGE);
        }

        size_t wr_len = MIN(len, (W25XXX_PAGE_SIZE - page_off));

        if ( (wr_len != W25XXX_PAGE_SIZE) &&
            (off%W25XXX_PAGE_SIZE) )
        {
            txn_buf[0] = W25XXX_OP_PROG_RAND_DATA_LOAD;
        }
        else
        {
            txn_buf[0] = W25XXX_OP_PROG_DATA_LOAD;
        }

        //write into the page buffer in W25XXX_WRITE_BUFF_SIZE byte chunks
        for (size_t txn_off = 0, txn_len = 0; txn_off < wr_len;
            txn_off += txn_len)
        {
            txn_len = MIN(W25XXX_WRITE_BUFF_SIZE, wr_len - txn_off);

            //write the column addres into the buffer
            txn_buf[1] = (page_off + txn_off) >> 8;
            txn_buf[2] = (page_off + txn_off) & 0xff;

            memcpy(txn_buf + 3, dp, txn_len);

            w25xxx_txn(3 + txn_len,
                        txn_buf,
                        0,
                        0);

            txn_buf[0] = W25XXX_OP_PROG_RAND_DATA_LOAD;
            dp += txn_len;
        }

        //only execute if at the end of a page
        if( len >= (W25XXX_PAGE_SIZE - page_off))
        {
            prog_exec_op.op_code = W25XXX_OP_PROG_EXECUTE;
            prog_exec_op.dummy_tx_bytes = 1;
            prog_exec_op.tx_data = page_num;
            prog_exec_op.tx_len = 2;

            w25xxx_exec_instruction(prog_exec_op);

            //read the status register until BUSY status is cleared
            status_op.op_code = W25XXX_OP_READ_REG;
            status_op.tx_len = 1;
            status_op.tx_data = W25XXX_REG_STAT;
            status_op.rx_len = 1;
            status_op.rx_data = &reg;

            do
            {
                w25xxx_exec_instruction(status_op);
            } while(reg & W25XXX_REG_STAT_BUSY);


            if (reg & W25XXX_REG_STAT_PFAIL)
            {
                MW_LOG_ERROR("Prog failed, page %u", page_num);
                return(W25XXX_PROG_EXEC_FAIL);
                /* TODO. On-the-fly remapping of bad blocks? */
            }
        }

        off += wr_len;
        len -= wr_len;
    }

    return(W25XXX_SUCCESS);
}

/**
 * @brief - Function to write a partially written page buffer into flash
 *
 * @param off - the address of the last byte to written into the page buffer
 *
 * @return - w25xxx_return_code_t value
 */
w25xxx_return_code_t w25xxx_trigger_prog_exec(size_t off)
{
    uint16_t page_num, page_off;
    struct w25xxx_op prog_exec_op = W25XXX_OP_DEFAULT_CONFIG;
    struct w25xxx_op status_op = W25XXX_OP_DEFAULT_CONFIG;
    uint8_t reg;

    if (!w25xxx_map_page(off, &page_num, &page_off))
    {
        return(W25XXX_ADDRESS_OUT_RANGE);
    }

    prog_exec_op.op_code = W25XXX_OP_PROG_EXECUTE;
    prog_exec_op.dummy_tx_bytes = 1;
    prog_exec_op.tx_data = page_num;
    prog_exec_op.tx_len = 2;

    w25xxx_exec_instruction(prog_exec_op);

    //read the status register until BUSY status is cleared
    status_op.op_code = W25XXX_OP_READ_REG;
    status_op.tx_len = 1;
    status_op.tx_data = W25XXX_REG_STAT;
    status_op.rx_len = 1;
    status_op.rx_data = &reg;

    do
    {
        w25xxx_exec_instruction(status_op);
    } while(reg & W25XXX_REG_STAT_BUSY);


    if (reg & W25XXX_REG_STAT_PFAIL)
    {
        MW_LOG_ERROR("Prog failed, page %u", page_num);
        return(W25XXX_PROG_EXEC_FAIL);
        /* TODO. On-the-fly remapping of bad blocks? */
    }

    return(W25XXX_SUCCESS);

}

/**
 * @brief - Function to erase a block (or many blocks) from flash
 *
 * @param off - byte address to start erasing at (must be aligned to a block address!)
 * @param len - length of bytes to erase (must be aligned to a block address!)
 *
 * @return - w25xxx_return_code_t value
 */
w25xxx_return_code_t vfs_dev_w25xxx_erase(size_t off,
                                          size_t len)
{
    uint16_t page_num, page_off;
    struct w25xxx_op write_en_op = W25XXX_OP_DEFAULT_CONFIG;
    struct w25xxx_op erase_op = W25XXX_OP_DEFAULT_CONFIG;
    struct w25xxx_op status_op = W25XXX_OP_DEFAULT_CONFIG;
    uint8_t reg;

    if (off % W25XXX_BLOCK_SIZE != 0 || len % W25XXX_BLOCK_SIZE != 0) {
        return(W25XXX_INVALID_ARG);
    }

    //while loop itterates over many pages
    while (len > 0)
    {
        //write enable must be called after every erase
        write_en_op.op_code = W25XXX_OP_WRITE_ENABLE;
        w25xxx_exec_instruction(write_en_op);

        if (!w25xxx_map_page(off, &page_num, &page_off))
        {
            return(W25XXX_ADDRESS_OUT_RANGE);
        }

        //erase the page
        erase_op.op_code = W25XXX_OP_BLOCK_ERASE;
        erase_op.dummy_tx_bytes = 1;
        erase_op.tx_data = page_num;
        erase_op.tx_len = 2;

        w25xxx_exec_instruction(erase_op);

        //read the status register until BUSY status is cleared
        status_op.op_code = W25XXX_OP_READ_REG;
        status_op.tx_len = 1;
        status_op.tx_data = W25XXX_REG_STAT;
        status_op.rx_len = 1;
        status_op.rx_data = &reg;

        do
        {
            w25xxx_exec_instruction(status_op);
        } while(reg & W25XXX_REG_STAT_BUSY);


        if (reg & W25XXX_REG_STAT_EFAIL) {
            MW_LOG_ERROR("Erase failed, page %u", page_num);
            return(W25XXX_ERASE_FAIL);
        }
        off += W25XXX_BLOCK_SIZE;
        len -= W25XXX_BLOCK_SIZE;
    }

    return(W25XXX_SUCCESS);
}


/**
 * @brief - Function to read bad block marker for a block
 *
 * @param block_num - block number
 * @param bad_block - true -> block is good.  false-> block is bad
 */
w25xxx_return_code_t w25xxx_read_bb_marker(uint16_t block_num,
                                           bool *bad_block)
{
    w25xxx_return_code_t res;
    struct w25xxx_op read_op = W25XXX_OP_DEFAULT_CONFIG;
    uint8_t bb_marker[2];

    if ((res = w25xxx_page_data_read(block_num*W25XXX_PAGES_PER_BLOCK)))
    {
        return(res);
    }

    //read the bad block marker
    read_op.op_code = W25XXX_OP_READ;
    read_op.tx_len = 2;
    read_op.tx_data = W25XXX_PAGE_SIZE;
    read_op.dummy_rx_bytes = 1;
    read_op.rx_data = bb_marker;
    read_op.rx_len = 2;

    w25xxx_exec_instruction(read_op);

    if(bb_marker[0] != 0xFF || bb_marker[1] != 0xFF)
    {
        *bad_block = true;
        MW_LOG_ERROR("Bad block detected at block : %d", block_num);
    }
    else
    {
        *bad_block = false;
    }

    return(W25XXX_SUCCESS);
}

/**
 * @brief - Function to read the programmed bad-block look-up table.
 *
 * @param bb_lut - return bad block look-up table read from registers
 */
void w25xxx_read_bb_lut(w25xxx_bb_lut_entry_t bb_lut[W25XXX_BB_LUT_SIZE])
{
    uint8_t tmp[W25XXX_BB_LUT_SIZE * 4];

    //submit instruction to read page data into buffer
    struct w25xxx_op read_bbm_op = W25XXX_OP_DEFAULT_CONFIG;
    read_bbm_op.op_code = W25XXX_OP_BBM_READ_LUT;
    read_bbm_op.dummy_tx_bytes = 1;
    read_bbm_op.rx_len = W25XXX_BB_LUT_SIZE * 4;
    read_bbm_op.rx_data = tmp;
    w25xxx_exec_instruction(read_bbm_op);


    for (int i = 0, j = 0; j < W25XXX_BB_LUT_SIZE; i++, j += 4)
    {
        bb_lut[i].enable = !!(tmp[j] & 0x80);
        bb_lut[i].invalid = !!(tmp[j] & 0x40);
        bb_lut[i].lba = (((uint16_t)(tmp[j] & 3)) << 8) | tmp[j + 1];
        bb_lut[i].pba = (((uint16_t) tmp[j + 2]) << 8) | tmp[j + 3];

        if (bb_lut[i].enable)
        {
            MW_LOG_DEBUG("Bad Block LUT entry %d: %d %d %u => %u",
                         i,
                         bb_lut[i].enable,
                         bb_lut[i].invalid,
                         bb_lut[i].lba,
                         bb_lut[i].pba);
        }
    }

}

/**
 * @brief - Function to find a new physical block for a bad block
 *
 * @param bb_lut - current programmed bad block loo-up table
 * @param bad_blocks - array of bad blocks (found based on bad block markers)
 * @param new_block - return value for the new physical block to map to
 * @param bb_lut_full - true : bad block LUT ful.  false: bad block LUT not full
 */
void w25xxx_find_new_physical_block(w25xxx_bb_lut_entry_t bb_lut[W25XXX_BB_LUT_SIZE],
                                    uint16_t bad_blocks[W25XXX_BB_LUT_SIZE],
                                    uint16_t *new_block,
                                    bool *bb_lut_full)
{
    int i;
    bool block_found = false;

    //- select from last block
    //- make sure the block isn't already mapped as a physical address in the LUT


    *bb_lut_full = true;
    for(i=0;i<W25XXX_BB_LUT_SIZE;i++)
    {
        if(bb_lut[i].enable == false)
        {
            *bb_lut_full = false;
        }
    }

    if(*bb_lut_full)
    {
        return;
    }


    *new_block = W25XXX_NUM_BLOCKS;
    while(block_found == false)
    {
        *new_block = *new_block-1;
        block_found = true;

        //search through LUT to see if the the physical address
        //is already mapped
        for(i=0;i<W25XXX_BB_LUT_SIZE;i++)
        {
            if(bb_lut[i].enable == true &&
               bb_lut[i].invalid == false &&
               bb_lut[i].pba ==  *new_block)
            {
                block_found = false;
            }
        }

        //search through back block list to see if the new block is
        //bad
        for(i=0;i<W25XXX_BB_LUT_SIZE;i++)
        {
            if(bad_blocks[i] == *new_block)
            {
                block_found = false;
            }
        }
    }

}

/**
 * @brief - Function to program the bad block look-up table
 *
 * @param bb_lut - returned bad-block look-up table
 */
 w25xxx_return_code_t w25xxx_bb_program(w25xxx_bb_lut_entry_t bb_lut[W25XXX_BB_LUT_SIZE])
 {
    bool bad_block;
    uint16_t bad_blocks[W25XXX_BB_LUT_SIZE];
    uint8_t bb_cnt = 0;
    w25xxx_return_code_t res;
    int i, j;
    uint16_t new_block;
    struct w25xxx_op bb_op = W25XXX_OP_DEFAULT_CONFIG;
    struct w25xxx_op write_en_op = W25XXX_OP_DEFAULT_CONFIG;
    bool bb_mapped;
    bool bb_lut_full;


    //read LUT
    w25xxx_read_bb_lut(bb_lut);

    //locate any bad blocks
    for(i=0;i<W25XXX_NUM_BLOCKS;i++)
    {
        if ((res = w25xxx_read_bb_marker(i,
                                         &bad_block)))
        {
            return(res);
        }

        if(bad_block == true &&
           bb_cnt < W25XXX_BB_LUT_SIZE)
        {
            bad_blocks[bb_cnt] = i;
            bb_cnt++;
        }
    }

    //for each bad block that isn't already contained in the LUT,
    //program the bad block
    for(i=0;i<bb_cnt;i++)
    {
        //make sure the bad block isn't already programmed
        //in the LUT
        bb_mapped = false;
        for(j=0;j<W25XXX_BB_LUT_SIZE;j++)
        {
            if(bb_lut[j].enable == true &&
               bb_lut[j].invalid == false &&
               bb_lut[j].lba ==  bad_blocks[i])
            {
                //bad block already mapped
                bb_mapped = true;
            }
        }

        if(bb_mapped == false)
        {
            w25xxx_find_new_physical_block(bb_lut,
                                        bad_blocks,
                                        &new_block,
                                        &bb_lut_full);


            //program bad block if the LUT isn't full
            if(bb_lut_full == false)
            {
                //write enable must be set after every page write
                write_en_op.op_code = W25XXX_OP_WRITE_ENABLE;
                w25xxx_exec_instruction(write_en_op);

                bb_op.op_code = W25XXX_OP_BBM_SWAP_BLOCKS;
                bb_op.tx_data = (bad_blocks[i] << 16) | new_block;
                bb_op.tx_len = 4;
                w25xxx_exec_instruction(bb_op);
            }
            else
            {
                MW_LOG_ERROR("Bad block LUT FULL!!!!");
                return(W25XXX_BB_FULL);
            }

            vTaskDelay(3);
            //read the new LUT
            w25xxx_read_bb_lut(bb_lut);
        }
    }


    return(W25XXX_SUCCESS);
 }
