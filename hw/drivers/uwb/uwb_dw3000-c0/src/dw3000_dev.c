/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/**
 * @file dw3000_dev.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Device file
 *
 * @details This is the dev base class which utilises the functions to perform initialization and necessary configurations on device.
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <dpl/dpl.h>
#include <dpl/dpl_cputime.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <stats/stats.h>
#include <dw3000-c0/dw3000_dev.h>
#include <dw3000-c0/dw3000_regs.h>
#include <dw3000-c0/dw3000_hal.h>
#include <dw3000-c0/dw3000_phy.h>

#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static void dw3000_dev_xfer(dw3000_dev_instance_t * inst, const uint32_t regFileID,
                            const uint16_t indx, const uint16_t length, uint8_t *buffer,
                            const spi_modes_e mode);


/**
 * API to perform dw3000_read from given address.
 *
 * @param inst          Pointer to dw3000_dev_instance_t.
 * @param reg           Member of dw3000_cmd_t structure.
 * @param subaddress    Member of dw3000_cmd_t structure.
 * @param buffer        Result is stored in buffer.
 * @param length        Represents buffer length.
 * @return struct uwb_dev_status
 */

struct uwb_dev_status
dw3000_read(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length)
{
    dw3000_dev_xfer(inst, reg, subaddress, length, buffer, DW3000_SPI_RD_BIT);
    return inst->uwb_dev.status;
}

/**
 * API to performs dw3000_write into given address.
 *
 * @param inst          Pointer to dw3000_dev_instance_t.
 * @param reg           Member of dw3000_cmd_t structure.
 * @param subaddress    Member of dw3000_cmd_t structure.
 * @param buffer        Result is stored in buffer.
 * @param length        Represents buffer length.
 * @return struct uwb_dev_status
 */

struct uwb_dev_status
dw3000_write(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length)
{
    dw3000_dev_xfer(inst, reg, subaddress, length, buffer, DW3000_SPI_WR_BIT);
    return inst->uwb_dev.status;
}

/**
 * API to read data from dw3000 register based on given parameters.
 *
 * @param inst          Pointer to dw3000_dev_instance_t.
 * @param reg           Register from where data is read.
 * @param subaddress    Address where data is read.
 * @param nbytes        Length of data.
 * @return   buffer.value
 */
uint64_t
dw3000_read_reg(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, size_t nbytes)
{
    union _buffer{
        uint8_t array[sizeof(uint64_t)];
        uint64_t value;
    } __attribute__((__packed__, aligned (8))) buffer = {0};

    dw3000_dev_xfer(inst, reg, subaddress, nbytes, buffer.array, DW3000_SPI_RD_BIT);
    return buffer.value;
}

/**
 * API to write data into dw3000 register based on given parameters.
 *
 * @param inst          Pointer to dw3000_dev_instance_t.
 * @param reg           Register from where data is written into.
 * @param subaddress    Address where writing of data begins.
 * @param val           Value to be written.
 * @param nbytes        Length of data.
 * @return   buffer.value
 */
void
dw3000_write_reg(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint64_t val, size_t nbytes)
{
    union _buffer{
        uint8_t array[sizeof(uint64_t)];
        uint64_t value;
    } __attribute__((__packed__))  buffer;

    buffer.value = val;
    assert(nbytes <= sizeof(uint64_t));

#ifdef DW3000_API_ERROR_CHECK
    if (nbytes == 1) assert((0xFFFFFFFFFFFFFF00ULL&val)==0x0);
    if (nbytes == 2) assert((0xFFFFFFFFFFFF0000ULL&val)==0x0);
    if (nbytes == 3) assert((0xFFFFFFFFFF000000ULL&val)==0x0);
    if (nbytes == 4) assert((0xFFFFFFFF00000000ULL&val)==0x0);
    if (nbytes == 5) assert((0xFFFFFF0000000000ULL&val)==0x0);
    if (nbytes == 6) assert((0xFFFF000000000000ULL&val)==0x0);
    if (nbytes == 7) assert((0xFF00000000000000ULL&val)==0x0);
#endif
    dw3000_dev_xfer(inst, reg, subaddress, nbytes, buffer.array, DW3000_SPI_WR_BIT);
}


/**
 * API to modify data in dw3000 register based on given parameters.
 *
 * @param inst          Pointer to dw3000_dev_instance_t.
 * @param reg           Register from where data is written into.
 * @param subaddress    Address where modification of data begins.
 * @param _and          Value to AND to register.
 * @param _or           Value to OR to register.
 * @param nbytes        Length of data.
 * @return void
 */
void
dw3000_modify_reg(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint32_t _and, uint32_t _or, size_t nbytes)
{
    uint8_t buf[8] = {0};
    spi_modes_e mode = 0;
    switch (nbytes) {
    case (1): {
        mode = DW3000_SPI_AND_OR_8;
        buf[0] = (uint8_t)(_and&0xff);
        buf[1] = (uint8_t)(_or&0xff);
        break;
    }
    case (2): {
        mode = DW3000_SPI_AND_OR_16;
        buf[0] = (uint8_t)_and;
        buf[1] = (uint8_t)(_and>>8);
        buf[2] = (uint8_t)_or;
        buf[3] = (uint8_t)(_or>>8);
        break;
    }
    case (4): {
        mode = DW3000_SPI_AND_OR_32;
        buf[0] = (uint8_t)_and;
        buf[1] = (uint8_t)(_and>>8);
        buf[2] = (uint8_t)(_and>>16);
        buf[3] = (uint8_t)(_and>>24);
        buf[4] = (uint8_t)_or;
        buf[5] = (uint8_t)(_or>>8);
        buf[6] = (uint8_t)(_or>>16);
        buf[7] = (uint8_t)(_or>>24);
        break;
    }
    default: assert(0);
    }

    dw3000_dev_xfer(inst, reg, subaddress, 2*nbytes, buf, mode);
}

static uint8_t crcTable[256];
static void
_dwt_crc8init(void)
{
    uint8_t bit;
    uint8_t  remainder;
    int dividend;

    /* Compute the remainder of each possible dividend. */
    for (dividend = 0; dividend < 256; ++dividend) {
        /* Start with the dividend followed by zeros. */
        remainder = dividend;

        /* Perform modulo-2 division, a bit at a time. */
        for (bit = 8; bit > 0; --bit) {
            /* Try to divide the current data bit. */
            if (remainder & TOPBIT) {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            } else {
                remainder = (remainder << 1);
            }
        }
        /* Store the result into the table. */
        crcTable[dividend] = remainder;
    }

}   /* _dwt_crc8init() */

/*!
* @fn dwt_generatecrc8()
*
* @brief  this function is used to calculate 8-bit CRC, it uses 100000111 polynomial (i.e. P(x) = x^8+ x^2+ x^1+ x^0)
* this funfction has been optimised to use crcTable[] and calculate the CRC on byte by byte basis.
*
* input parameters:
* @param byteArray         - data to calculate CRC for
* @param len               - length of byteArray
* @param crcRemainderInit  - the remainder is the crc, also it is initially set to the initialisation value for CRC calculation
*
* output parameters
*
* returns 8-bit calculate CRC value
*/
static uint8_t
dwt_generatecrc8(const uint8_t* byteArray, int len, uint8_t crcRemainderInit)
{
    uint8_t data;
    int byte;

    /*
    * Divide the message by the polynomial, a byte at a time.
    */
    for (byte = 0; byte < len; ++byte)
    {
        data = byteArray[byte] ^ crcRemainderInit;
        crcRemainderInit = crcTable[data];// ^ (crcRemainderInit << 8);
    }

    /*
    * The final remainder is the CRC.
    */
    return(crcRemainderInit);
}


/*!
* @fn dw3000_dev_xfer()
*
* @brief  this function is used to read/write to the DW3000 device registers
*
* input parameters:
* @param recordNumber  - ID of register file or buffer being accessed
* @param index         - byte index into register file or buffer being accessed
* @param length        - number of bytes being written
* @param buffer        - pointer to buffer containing the 'length' bytes to be written
* @param rw            - DW3000_SPI_WR_BIT/DW3000_SPI_RD_BIT
*
* no return value
*/
static void
dw3000_dev_xfer
(
    dw3000_dev_instance_t * inst,
    const uint32_t    regFileID,  //0x0, 0x04-0x7F ; 0x10000, 0x10004, 0x10008-0x1007F; 0x20000 etc
    const uint16_t    indx,       //sub-index, calculated from regFileID 0..0x7F,
    const uint16_t    length,
    uint8_t           *buffer,
    const spi_modes_e mode
)
{
    uint16_t  addr;
    uint8_t  header[2];           // Buffer to compose header in
    uint16_t cnt = 0;             // Counter for length of a header

    uint16_t reg_file     = 0x1F & ((regFileID + indx) >> 16);
    uint16_t reg_offset   = 0x7F &  (regFileID + indx);

    /* Check if someone is trying to use an index / offset that is larger than 0x7F */
    assert((0x80&(regFileID + indx)) == 0);
    assert(length       < 0x3100);
    assert(mode == DW3000_SPI_WR_BIT ||\
           mode == DW3000_SPI_RD_BIT ||\
           mode == DW3000_SPI_AND_OR_8 ||\
           mode == DW3000_SPI_AND_OR_16 ||\
           mode == DW3000_SPI_AND_OR_32);

    // Write message header selecting WRITE operation and addresses as appropriate
    addr = (reg_file << 9) | (reg_offset << 2);

    header[0] = (uint8_t)((mode | addr) >> 8);//  & 0xFF; //bit7 + addr[4:0] + sub_addr[6:6]
    header[1] = (uint8_t)(addr | (mode & 0x03));// & 0xFF; //EAM: subaddr[5:0]+ R/W/AND_OR

    if (/*reg_offset == 0 && */length == 0)
    {   /* Fast Access Commands (FAC)
         * only write operation is possible for this mode
         * bit_7=one is W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=one: MODE of FastAccess
         */
        assert(mode == DW3000_SPI_WR_BIT);

        header[0] = (uint8_t)((DW3000_SPI_WR_BIT>>8) | (regFileID<<1) | DW3000_SPI_FAC);
        cnt = 1;
    }
    else if (reg_offset == 0 /*&& length > 0*/ && (mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT))
    {   /* Fast Access Commands with Read/Write support (FACRW)
         * bit_7 is R/W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=zero: MODE of FastAccess
         */
        header[0] |= DW3000_SPI_FARW;
        cnt = 1;
    } else {
        /* Extended Address Mode with Read/Write support (EAMRW)
         * b[0] = bit_7 is R/W operation, bit_6 one = ExtendedAddressMode;
         * b[1] = addr<<2 | (mode&0x3)
         */
        header[0] |= DW3000_SPI_EAMRW;
        cnt = 2;
    }

    switch (mode)
    {
    case    DW3000_SPI_AND_OR_8:
    case    DW3000_SPI_AND_OR_16:
    case    DW3000_SPI_AND_OR_32:
    case    DW3000_SPI_WR_BIT:
    {
        uint8_t crc8 = 0;
        if (inst->uwb_dev.config.spicrc_w_enable)
        {
            //generate 8 bit CRC
            crc8 = dwt_generatecrc8(header, cnt, 0);
            crc8 = dwt_generatecrc8(buffer, length, crc8);

            // Write it to the SPI
            // writetospiwithcrc(cnt, header, length, buffer, crc8);
            assert(0);
        }
        else
        {
            // Write it to the SPI
            if (cnt+length < MYNEWT_VAL(DW3000_DEVICE_SPI_RD_MAX_NOBLOCK) ||
                inst->uwb_dev.config.blocking_spi_transfers) {
                hal_dw3000_write(inst, header, cnt, buffer, length);
            } else {
                hal_dw3000_write_noblock(inst, header, cnt, buffer, length);
            }
        }
        break;
    }
    case DW3000_SPI_RD_BIT:
    {
        if (cnt+length < MYNEWT_VAL(DW3000_DEVICE_SPI_RD_MAX_NOBLOCK) ||
            inst->uwb_dev.config.blocking_spi_transfers) {
            hal_dw3000_read(inst, header, cnt, buffer, length);
        } else {
            hal_dw3000_read_noblock(inst, header, cnt, buffer, length);
        }

        //check that the SPI read has correct CRC-8 byte
        //also don't do for SPICRC_CFG_ID register itself to prevent infinite recursion
        if ((inst->uwb_dev.config.spicrc_r_enable) && (regFileID != SPICRC_CFG_ID))
        {
            uint8_t crc8, dwcrc8;
            //generate 8 bit CRC from the read data
            crc8 = dwt_generatecrc8(header, cnt, 0);
            crc8 = dwt_generatecrc8(buffer, length, crc8);

            //read the CRC that was generated in the DW3000 for the read transaction
            // dwcrc8 = dwt_read8bitoffsetreg(SPICRC_CFG_ID, 0);
            assert(0);

            //if the two CRC don't match report SPI read error

            //AI: FIXME: potential problem in callback if it will try to read/write SPI with CRC again.
            if (crc8 != dwcrc8) {
                inst->uwb_dev.status.spi_r_error = 1;
            }
        }
        break;
    }
    default:
        assert(0);
        break;
    }

} // end dw3000_dev_xfer()


/**
 * API to do softreset on dw3000 by writing data into PMSC_CTRL0_SOFTRESET_OFFSET.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
void
dw3000_softreset(dw3000_dev_instance_t * inst)
{
#if 0
    // Set system clock to XTI
    dw3000_phy_sysclk_XTAL(inst);
    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE, sizeof(uint16_t)); // Disable PMSC ctrl of RF and RX clk blocks
    dw3000_write_reg(inst, AON_ID, AON_WCFG_OFFSET, 0x0, sizeof(uint16_t)); // Clear any AON auto download bits (as reset will trigger AON download)
    dw3000_write_reg(inst, AON_ID, AON_CFG0_OFFSET, 0x0, sizeof(uint8_t));  // Clear the wake-up configuration
    // Uploads always-on (AON) data array and configuration
    dw3000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0x0, sizeof(uint8_t)); // Clear the register
    dw3000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE, sizeof(uint8_t));
    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_ALL, sizeof(uint8_t));// Reset HIF, TX, RX and PMSC

    // DW3000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
    dpl_cputime_delay_usecs(10);

    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR, sizeof(uint8_t)); // Clear reset
#endif
}

/**
 * API to configure dw3000.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @returns DPL_OK on success
 */
int
dw3000_dev_config(dw3000_dev_instance_t * inst)
{
    int i;
    int rc;
    int timeout = 10;

retry:
    /* Reset chip */
    hal_dw3000_reset(inst);
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    /* Do nothing */
    (void) rc;
#else
    /* Make sure spi is setup with the correct speed etc */
    rc = hal_spi_disable(inst->spi_num);
    assert(rc == 0);
    rc = hal_spi_config(inst->spi_num, &inst->spi_settings);
    assert(rc == 0);
    hal_spi_set_txrx_cb(inst->spi_num, hal_dw3000_spi_txrx_cb, (void*)inst);
    rc = hal_spi_enable(inst->spi_num);
    assert(rc == 0);
#endif

    /* Need to make sure DW IC is in IDLE_RC before proceeding */
    for (i=0;i<20;i++) {
        if (dw3000_dev_checkidlerc(inst)) {
            break;
        }
        dpl_cputime_delay_usecs(50);
    }

    inst->uwb_dev.device_id = dw3000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));
    inst->uwb_dev.status.initialized = ((inst->uwb_dev.device_id&DEV_ID_RIDTAG_BIT_MASK) == (DWT_DEVICE_ID&DEV_ID_RIDTAG_BIT_MASK));

    if (!inst->uwb_dev.status.initialized && --timeout) {
        /* In case dw3000 was sleeping */
        dw3000_dev_wakeup(inst);
        goto retry;
    }

    /* This driver does not support A0 or B0 */
    assert((inst->uwb_dev.device_id&DEV_ID_REV_BIT_MASK) > 1);

    /* Determine if we're PDOA Capable from the device id */
    if (inst->uwb_dev.device_id & DWT_XX_PDOA_DEV_ID_MASK) {
        inst->uwb_dev.capabilities.single_receiver_pdoa = 1;
    }

    if(!inst->uwb_dev.status.initialized) {
        return DPL_TIMEOUT;
    }
#if 0
    uint8_t buf[96] = {0};
    uint8_t buf2[sizeof(buf)] = {0};
    for (i=0;i<sizeof(buf);i++) {
        buf[i] = i;
    }
    dw3000_write(inst, SCRATCH_RAM_ID, 0,  buf, sizeof(buf));
    dw3000_read(inst, SCRATCH_RAM_ID, 0,  buf2, sizeof(buf2));
    for (i=0;i<sizeof(buf);i++) {
        buf[i] = i;
        printf("%d: %02X %02X\n", i, buf[i], buf2[i]);
    }
    dw3000_read(inst, SCRATCH_RAM_ID, 0,  buf2, sizeof(buf2));
    for (i=0;i<sizeof(buf);i++) {
        printf("%d: %02X %02X\n", i, buf[i], buf2[i]);
    }
#endif
    dw3000_phy_init(inst, NULL);

    inst->uwb_dev.pan_id = MYNEWT_VAL(PANID);
    inst->uwb_dev.uid = inst->part_id & 0xffff;

    if (inst == hal_dw3000_inst(0)) {
#if  MYNEWT_VAL(DW_DEVICE_ID_0)
        inst->uwb_dev.uid = MYNEWT_VAL(DW_DEVICE_ID_0);
#endif
    } else if (inst == hal_dw3000_inst(1)){
#if  MYNEWT_VAL(DW_DEVICE_ID_1)
        inst->uwb_dev.uid = MYNEWT_VAL(DW_DEVICE_ID_1);
#endif
    } else if (inst == hal_dw3000_inst(2)){
#if  MYNEWT_VAL(DW_DEVICE_ID_2)
        inst->uwb_dev.uid = MYNEWT_VAL(DW_DEVICE_ID_2);
#endif
    }
    inst->uwb_dev.euid = (((uint64_t)inst->lot_id) << 32) + inst->part_id;

    dw3000_set_panid(inst,inst->uwb_dev.pan_id);
    dw3000_mac_init(inst, NULL);

    /* Set the initial id configuration */
    if (!inst->uwb_dev.uid || inst->uwb_dev.uid == 0xFFFF) {
        /* Having an address of 0 or 0xFFFF isn't valid */
        inst->uwb_dev.uid = 0x1;
    }
    dw3000_set_panid(inst,inst->uwb_dev.pan_id);
    dw3000_set_eui(inst,inst->uwb_dev.euid);
    dw3000_set_address16(inst,inst->uwb_dev.uid);

    return DPL_OK;
}

/**
 * @fn dw3000_dev_force_clocks()
 *
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * input parameters
 * @param clocks - set of clocks to enable/disable
 *
 * output parameters none
 *
 * no return value
 */
void
dw3000_dev_force_clocks(struct _dw3000_dev_instance_t * inst, int clocks)
{
    uint16_t regvalue0 = 0;

    // A0 by default always enable TX and RX clk buffers
    regvalue0 = CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK | CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;

    if (clocks == FORCE_CLK_ALL) {
        //all clockmode
        //SYS_CLK_SEL = PLL
        regvalue0 |= (uint16_t)FORCE_SYSCLK_PLL << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

        //RX_CLK_SEL = ON
        regvalue0 |= (uint16_t)FORCE_CLK_PLL << CLK_CTRL_RX_CLK_SEL_BIT_OFFSET;

        //FORCE_CIA_CLKS = ON
        regvalue0 = regvalue0 | CLK_CTRL_FORCE_CIA_CLKS_ON_BIT_MASK;

        //NVM_CLK_EN = ON
        regvalue0 = regvalue0 | CLK_CTRL_FORCE_NVM_CLK_EN_BIT_MASK;

        //RX_BUF_CLK = ON
        regvalue0 = regvalue0 | CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;

        //RSD_CLK = ON
        regvalue0 = regvalue0 | CLK_CTRL_RSD_CLK_ON_BIT_MASK;

        // ACC_MEM_CLK = ON
        regvalue0 = regvalue0 | CLK_CTRL_ACC_MEM_CLK_ON_BIT_MASK;

        //TX_CLK_SEL = ON
        regvalue0 |= (uint16_t)FORCE_CLK_PLL << CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;

        //TX_BUF_CLK = ON
        regvalue0 = regvalue0 | CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK;
    } else if (clocks == FORCE_CLK_SYS_TX) {
        //system and tx only

        //SYS_CLK_SEL = PLL
        regvalue0 |= (uint16_t)FORCE_SYSCLK_PLL << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

        //RX_CLK_SEL = OFF
        regvalue0 &= ~((uint16_t)FORCE_CLK_PLL << CLK_CTRL_RX_CLK_SEL_BIT_OFFSET);

        // Don't clear RX_BUF_CLK - This is left enabled to keep the PLL loaded

        //TX_CLK_SEL = ON
        regvalue0 |= (uint16_t)FORCE_CLK_PLL << CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;

        //TX_BUF_CLK = ON
        regvalue0 = regvalue0 | CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK;

    } else if (clocks == FORCE_CLK_SYS_RX) {
        //system and rx only

        //SYS_CLK_SEL = PLL
        regvalue0 |= (uint16_t)FORCE_SYSCLK_PLL << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

        //RX_CLK_SEL = OFF
        regvalue0 |= (uint16_t)FORCE_CLK_PLL << CLK_CTRL_RX_CLK_SEL_BIT_OFFSET;

        //RX_BUF_CLK = ON
        regvalue0 |= CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;

        //FORCE_LDE_CLKS = ON
        regvalue0 |= CLK_CTRL_FORCE_CIA_CLKS_ON_BIT_MASK;

        //NVM_CLK_EN = ON
        regvalue0 |= CLK_CTRL_FORCE_NVM_CLK_EN_BIT_MASK;

        //RSD_CLK = ON
        regvalue0 |= CLK_CTRL_RSD_CLK_ON_BIT_MASK;

        //ACC_MEM_CLK = ON
        regvalue0 |= CLK_CTRL_ACC_MEM_CLK_ON_BIT_MASK;

        //TX_CLK_SEL = OFF
        regvalue0 &= ~((uint16_t)FORCE_CLK_PLL << CLK_CTRL_TX_CLK_SEL_BIT_OFFSET);

        // Don't clear TX_BUF_CLK - This is left enabled to keep the PLL loaded
    }
    else if (clocks == FORCE_CLK_SYS)
    {
        regvalue0 |= (uint16_t)FORCE_SYSCLK_PLL << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;
    }
    else if (clocks == FORCE_CLK_FOSC_DIV)
    {
        //Sytem = FOSC/4
        regvalue0 |= (uint16_t)FORCE_SYSCLK_FOSC_DIV << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

    }
    else if (clocks == FORCE_CLK_FOSC)
    {
        //Sytem = FOSC
        regvalue0 |= (uint16_t)FORCE_SYSCLK_FOSC << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;
    }
    else if (clocks == FORCE_CLK_AUTO)
    {
        //Full auto for sequencing
        //NVM CLOCK, RX buff and code mem are set by default
        regvalue0 = (uint16_t) (CLK_CTRL_FORCE_NVM_CLK_EN_BIT_MASK | CLK_CTRL_RX_BUFF_AUTO_CLK_BIT_MASK | CLK_CTRL_CODE_MEM_AUTO_CLK_BIT_MASK);
    }
    dw3000_write_reg(inst, CLK_CTRL_ID, 0x0, regvalue0, sizeof(uint16_t));

} // end dwt_force_clocks()


/**
 * API to set the sleep counter to new value, this function programs the high 16-bits of the 28-bit counter.
 *
 * NOTE: this function needs to be run before dw3000_dev_configure_sleep, also the SPI freq has to be < 3MHz
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @param count Value of the sleep counter to program.
 *
 * @return void
 */
void
dw3000_dev_set_sleep_timer(dw3000_dev_instance_t * inst, uint16_t count)
{
#if 0
    dw3000_phy_sysclk_XTAL(inst); // Force system clock to be the 19.2 MHz XTI clock.
    dw3000_write_reg(inst, AON_ID, AON_CFG1_OFFSET, 0x0, sizeof(uint8_t)); // Disable the sleep counter
    dw3000_write_reg(inst, AON_ID, AON_CFG0_SLEEP_TIM_OFFSET, count, sizeof(uint16_t));     // Write new sleep counter
    dw3000_write_reg(inst, AON_ID, AON_CFG1_OFFSET, AON_CFG1_SLEEP_CEN | AON_CFG1_LPOSC_CAL, sizeof(uint8_t));   // Enable the sleep counter
    dw3000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_UPL_CFG, sizeof(uint8_t));     // Upload array
    dw3000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0, sizeof(uint8_t));                    // Upload array
    dw3000_phy_sysclk_SEQ(inst); // The system clock will run off the 19.2 MHz XTI clock until the PLL is calibrated and locked
#else
    assert(0);
#endif
}

/**
 *  API to configure the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 *  i.e., before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
 *  will be preserved and the device can immediately perform the desired action TX/RX.
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 *
 * @return void
 */
void
dw3000_dev_configure_sleep(dw3000_dev_instance_t * inst)
{
    uint16_t reg = dw3000_read_reg(inst, AON_DIG_CFG_ID, 0, sizeof(uint16_t));
    reg |= inst->sleep_mode;
    if (inst->uwb_dev.config.wakeup_rx_enable) {
        reg |= DWT_GOTORX;
    } else {
        reg &= ~(DWT_GOTORX);
        reg |= DWT_GOTOIDLE;
    }

    if (inst->uwb_dev.status.LDO_enabled) {
        reg |= DWT_LOADLDO;
    } else {
        reg &= ~DWT_LOADLDO;
    }

    dw3000_write_reg(inst, AON_DIG_CFG_ID, 0, reg, sizeof(uint16_t));

    reg = dw3000_read_reg(inst, ANA_CFG_ID, 0, sizeof(uint16_t));
    reg |= DWT_WAKE_CS | DWT_SLEEP | DWT_SLP_EN;
    dw3000_write_reg(inst, ANA_CFG_ID, 0, reg, sizeof(uint16_t));
}

/**
 * API to enter device into sleep mode.
 *
 * @param inst   Pointer to dw3000_dev_instance_t.
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw3000_dev_enter_sleep(dw3000_dev_instance_t * inst)
{
    // Critical region, atomic lock with mutex
    dpl_error_t err = dpl_mutex_pend(&inst->mutex, DPL_WAIT_FOREVER);
    if (err != DPL_OK) {
        inst->uwb_dev.status.mtx_error = 1;
        goto mtx_error;
    }

    /* Upload always on array configuration and enter sleep */
    dw3000_write_reg(inst, AON_CTRL_ID, 0, 0x0, sizeof(uint8_t));
    dw3000_write_reg(inst, AON_CTRL_ID, 0, AON_CTRL_ARRAY_UPLOAD_BIT_MASK, sizeof(uint8_t));
    inst->uwb_dev.status.sleeping = 1;

    // Critical region, unlock mutex
    err = dpl_mutex_release(&inst->mutex);
    assert(err == DPL_OK);
mtx_error:
    return inst->uwb_dev.status;
}

/**
 * API to wakeup device from sleep to init.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw3000_dev_wakeup(dw3000_dev_instance_t * inst)
{
    int timeout=10, initialised;
    uint32_t devid;
    // Critical region, atomic lock with mutex
    dpl_error_t err = dpl_mutex_pend(&inst->mutex, DPL_WAIT_FOREVER);
    if (err != DPL_OK) {
        inst->uwb_dev.status.mtx_error = 1;
        goto mtx_error;
    }

    /* Set sleeping status bit to zero here to allow a wakeup irq
     * to be captured. */
    inst->uwb_dev.status.sleeping = 0;
    devid = dw3000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));
    initialised = ((devid&DEV_ID_RIDTAG_BIT_MASK) ==
                   (DWT_DEVICE_ID&DEV_ID_RIDTAG_BIT_MASK));
    while (!initialised && --timeout) {
        hal_dw3000_wakeup(inst);
        devid = dw3000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));
        initialised = ((devid&DEV_ID_RIDTAG_BIT_MASK) ==
                       (DWT_DEVICE_ID&DEV_ID_RIDTAG_BIT_MASK));

    }
    inst->uwb_dev.status.sleeping = !initialised;
    dw3000_write_reg(inst, SYS_STATUS_ID, 0,
                     SYS_STATUS_RCINIT_BIT_MASK|SYS_STATUS_SPIRDY_BIT_MASK,
                     sizeof(uint32_t));
    dw3000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR, sizeof(uint32_t));

    /* Antenna delays lost in deep sleep ? */
    dw3000_phy_set_rx_antennadelay(inst, inst->uwb_dev.rx_antenna_delay);
    dw3000_phy_set_tx_antennadelay(inst, inst->uwb_dev.tx_antenna_delay);

    // Critical region, unlock mutex
    err = dpl_mutex_release(&inst->mutex);
    assert(err == DPL_OK);

    /* In case dw3000 was instructed to sleep directly after tx
     * we may need to release the tx sem */
    if(dpl_sem_get_count(&inst->tx_sem) == 0) {
        dpl_sem_release(&inst->tx_sem);
    }
mtx_error:
    return inst->uwb_dev.status;
}


/**
 * API to set the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The dev_configure_sleep() function
 * needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * @param inst       Pointer to dw3000_dev_instance_t.
 * @param enable     1 to configure the device to enter deep sleep after TX, 0 to disables the configuration.
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw3000_dev_enter_sleep_after_tx(dw3000_dev_instance_t * inst, uint8_t enable)
{
    inst->control.sleep_after_tx = enable;
    if(inst->control.sleep_after_tx) {
        dw3000_modify_reg(inst, SEQ_CTRL_ID, 0, 0xffff,
                          SEQ_CTRL_AUTO_TX2SLP_BIT_MASK, sizeof(uint16_t));
    } else {
        dw3000_modify_reg(inst, SEQ_CTRL_ID, 0,
                          (uint16_t)~(SEQ_CTRL_AUTO_TX2SLP_BIT_MASK),
                          0xffff, sizeof(uint16_t));
    }
    return inst->uwb_dev.status;
}

/**
 *  Sets the auto RX to sleep bit. This means that after a frame
 *  received the device will enter deep sleep mode. The dev_configure_sleep() function
 *  needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events).
 * @param inst     Pointer to dw3000_dev_instance_t.
 * @param enable   1 to configure the device to enter deep sleep after TX, 0 to disables the configuration
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw3000_dev_enter_sleep_after_rx(dw3000_dev_instance_t * inst, uint8_t enable)
{
#if 0
    inst->control.sleep_after_rx = enable;
    uint32_t reg = dw3000_read_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, sizeof(uint32_t));
    if(inst->control.sleep_after_rx)
        reg |= PMSC_CTRL1_ARXSLP;
    else
        reg &= ~(PMSC_CTRL1_ARXSLP);

    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, reg, sizeof(uint32_t));

#else
    printf("%s not implemented\n", __func__);
    assert(0);
#endif
    return inst->uwb_dev.status;
}

/**
 * This function checks if the DW3000 is in IDLE_RC state
 *
 * @param inst   Pointer to dw3000_dev_instance_t.
 * @return int   value is 1 if the IDLE_RC bit is set and 0 otherwise
 */
int
dw3000_dev_checkidlerc(dw3000_dev_instance_t * inst)
{
    /* Upload always on array configuration and enter sleep */
    uint32_t reg = dw3000_read_reg(inst, SYS_STATUS_ID, 2, sizeof(uint32_t)) << 16;
    return ( (reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK));
}

/*******************************************************/

inline static struct uwb_dev_status
uwb_dw3000_mac_config(struct uwb_dev *dev, struct uwb_dev_config *config)
{
    return dw3000_mac_config((dw3000_dev_instance_t *)dev, config);
}

inline static void
uwb_dw3000_txrf_config(struct uwb_dev *dev, struct uwb_dev_txrf_config *config)
{
    dw3000_phy_config_txrf((dw3000_dev_instance_t *)dev, config);
}

inline static bool
uwb_dw3000_txrf_power_value(struct uwb_dev * dev, uint8_t *reg, dpl_float32_t coarse, dpl_float32_t fine)
{
    return dw3000_phy_txrf_power_value((dw3000_dev_instance_t *)dev, reg, coarse, fine);
}

inline static void
uwb_dw3000_sleep_config(struct uwb_dev *dev)
{
    dw3000_dev_configure_sleep((dw3000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw3000_enter_sleep(struct uwb_dev *dev)
{
    return dw3000_dev_enter_sleep((dw3000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw3000_enter_sleep_after_tx(struct uwb_dev *dev, uint8_t enable)
{
    return dw3000_dev_enter_sleep_after_tx((dw3000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw3000_enter_sleep_after_rx(struct uwb_dev *dev, uint8_t enable)
{
    return dw3000_dev_enter_sleep_after_rx((dw3000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw3000_wakeup(struct uwb_dev *dev)
{
    return dw3000_dev_wakeup((dw3000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw3000_set_dblrxbuf(struct uwb_dev *dev, bool enable)
{
    return dw3000_set_dblrxbuff((dw3000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw3000_set_rx_timeout(struct uwb_dev *dev, uint32_t timeout)
{
    return dw3000_set_rx_timeout((dw3000_dev_instance_t *)dev, timeout);
}

inline static struct uwb_dev_status
uwb_dw3000_adj_rx_timeout(struct uwb_dev *dev, uint32_t timeout)
{
    return dw3000_adj_rx_timeout((dw3000_dev_instance_t *)dev, timeout);
}

inline static struct uwb_dev_status
uwb_dw3000_set_rx_window(struct uwb_dev *dev, uint64_t rx_start, uint64_t rx_end)
{
    return dw3000_set_rx_window((dw3000_dev_instance_t *)dev, rx_start, rx_end);
}

inline static struct uwb_dev_status
uwb_dw3000_set_abs_timeout(struct uwb_dev *dev, uint64_t rx_end)
{
    return dw3000_set_abs_timeout((dw3000_dev_instance_t *)dev, rx_end);
}

inline static struct uwb_dev_status
uwb_dw3000_set_cca_tx(struct uwb_dev *dev, bool enable)
{
    return dw3000_set_cca_tx((dw3000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw3000_set_delay_start(struct uwb_dev *dev, uint64_t dx_time)
{
    return dw3000_set_delay_start((dw3000_dev_instance_t *)dev, dx_time);
}

inline static struct uwb_dev_status
uwb_dw3000_start_tx(struct uwb_dev *dev)
{
    return dw3000_start_tx((dw3000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw3000_start_rx(struct uwb_dev *dev)
{
    return dw3000_start_rx((dw3000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw3000_stop_rx(struct uwb_dev *dev)
{
    return dw3000_stop_rx((dw3000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw3000_write_tx(struct uwb_dev* dev, uint8_t *tx_frame_bytes,
                    uint16_t tx_buffer_offset, uint16_t tx_frame_length)
{
    return dw3000_write_tx((dw3000_dev_instance_t *)dev, tx_frame_bytes,
                           tx_buffer_offset, tx_frame_length);
}

inline static void
uwb_dw3000_write_tx_fctrl(struct uwb_dev* dev, uint16_t tx_frame_length,
                          uint16_t tx_buffer_offset, struct uwb_fctrl_ext *ext)
{
    dw3000_write_tx_fctrl((dw3000_dev_instance_t *)dev, tx_frame_length,
                          tx_buffer_offset, ext);
}

inline static int
uwb_dw3000_tx_wait(struct uwb_dev* dev, dpl_time_t timeout)
{
    return dw3000_tx_wait((dw3000_dev_instance_t *)dev, timeout);
}

inline static int
uwb_dw3000_hal_noblock_wait(struct uwb_dev* dev, dpl_time_t timeout)
{
    return hal_dw3000_rw_noblock_wait((dw3000_dev_instance_t *)dev, timeout);
}

inline static struct uwb_dev_status
uwb_dw3000_set_wait4resp(struct uwb_dev *dev, bool enable)
{
    return dw3000_set_wait4resp((dw3000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw3000_set_wait4resp_delay(struct uwb_dev *dev, uint32_t delay)
{
    return dw3000_set_wait4resp_delay((dw3000_dev_instance_t *)dev, delay);
}

inline static struct uwb_dev_status
uwb_dw3000_set_rxauto_disable(struct uwb_dev *dev, bool disable)
{
    return dw3000_set_rxauto_disable((dw3000_dev_instance_t *)dev, disable);
}

inline static uint64_t
uwb_dw3000_read_systime(struct uwb_dev* dev)
{
    return dw3000_read_systime((dw3000_dev_instance_t *)dev);
}

inline static uint32_t
uwb_dw3000_read_systime_lo32(struct uwb_dev* dev)
{
    return dw3000_read_systime_lo((dw3000_dev_instance_t *)dev);
}

inline static uint64_t
uwb_dw3000_read_rxtime(struct uwb_dev* dev)
{
    return dw3000_read_rxtime((dw3000_dev_instance_t *)dev);
}

inline static uint32_t
uwb_dw3000_read_rxtime_lo32(struct uwb_dev* dev)
{
    return dw3000_read_rxtime_lo((dw3000_dev_instance_t *)dev);
}

inline static uint64_t
uwb_dw3000_read_sts_rxtime(struct uwb_dev* dev)
{
    return dw3000_read_sts_rxtime((dw3000_dev_instance_t *)dev);
}

inline static uint64_t
uwb_dw3000_read_txtime(struct uwb_dev* dev)
{
    return dw3000_read_txtime((dw3000_dev_instance_t *)dev);
}

inline static uint32_t
uwb_dw3000_read_txtime_lo32(struct uwb_dev* dev)
{
    return dw3000_read_txtime_lo((dw3000_dev_instance_t *)dev);
}

inline static uint16_t
uwb_dw3000_phy_frame_duration(struct uwb_dev* dev, uint16_t nlen, struct uwb_phy_attributes * attrib)
{
    return dw3000_phy_frame_duration((attrib) ? attrib : &dev->attrib, nlen);
}

inline static uint16_t
uwb_dw3000_phy_SHR_duration(struct uwb_dev* dev, struct uwb_phy_attributes * attrib)
{
    return dw3000_phy_SHR_duration((attrib) ? attrib : &dev->attrib);
}

inline static uint16_t
uwb_dw3000_phy_data_duration(struct uwb_dev* dev, uint16_t nlen, struct uwb_phy_attributes * attrib)
{
    return dw3000_phy_data_duration((attrib) ? attrib : &dev->attrib, nlen);
}

inline static void
uwb_dw3000_phy_forcetrxoff(struct uwb_dev* dev)
{
    return dw3000_phy_forcetrxoff((dw3000_dev_instance_t *)dev);
}

inline static void
uwb_dw3000_phy_rx_reset(struct uwb_dev* dev)
{
    return dw3000_phy_rx_reset((dw3000_dev_instance_t *)dev);
}

inline static void
uwb_dw3000_phy_repeated_frames(struct uwb_dev* dev, uint64_t rate)
{
    return dw3000_phy_repeated_frames((dw3000_dev_instance_t *)dev, rate);
}

inline static struct uwb_dev_status
uwb_dw3000_set_on_error_continue(struct uwb_dev * dev, bool enable)
{
    return dw3000_set_on_error_continue((dw3000_dev_instance_t *)dev, enable);
}

inline static void
uwb_dw3000_set_panid(struct uwb_dev * dev, uint16_t pan_id)
{
    return dw3000_set_panid((dw3000_dev_instance_t *)dev, pan_id);
}

inline static void
uwb_dw3000_set_uid(struct uwb_dev * dev, uint16_t uid)
{
    return dw3000_set_address16((dw3000_dev_instance_t *)dev, uid);
}

inline static void
uwb_dw3000_set_euid(struct uwb_dev * dev, uint64_t euid)
{
    return dw3000_set_eui((dw3000_dev_instance_t *)dev, euid);
}

static dpl_float64_t
uwb_dw3000_calc_clock_offset_ratio(struct uwb_dev * dev, int32_t val, uwb_cr_types_t type)
{
    if (type == UWB_CR_CARRIER_INTEGRATOR) {
        return dw3000_calc_clock_offset_ratio((dw3000_dev_instance_t *)dev, val);
    } else if (type == UWB_CR_RXTTCKO) {
        return dw3000_calc_clock_offset_ratio_ttco((dw3000_dev_instance_t *)dev, val);
    }
    return DPL_FLOAT64_INIT(0.0f);
}

inline static dpl_float32_t
uwb_dw3000_get_rssi(struct uwb_dev * dev)
{
    return dw3000_get_rssi((dw3000_dev_instance_t *)dev);
}

inline static dpl_float32_t
uwb_dw3000_get_fppl(struct uwb_dev * dev)
{
    return dw3000_get_fppl((dw3000_dev_instance_t *)dev);
}

inline static dpl_float32_t
uwb_dw3000_calc_rssi(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag)
{
    struct _dw3000_dev_rxdiag_t* diag_3k = (struct _dw3000_dev_rxdiag_t*)diag;
    return dw3000_calc_rssi((dw3000_dev_instance_t *)dev, &diag_3k->ipatov);
}

inline static dpl_float32_t
uwb_dw3000_calc_seq_rssi(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag, uint16_t type)
{
    struct _dw3000_dev_rxdiag_t* diag_3k = (struct _dw3000_dev_rxdiag_t*)diag;
    if (type == UWB_RXDIAG_STS) {
        if ((diag_3k->rxd.valid & UWB_RXDIAG_STS) == 0) {
            return DPL_FLOAT32_NAN();
        }
        return dw3000_calc_rssi((dw3000_dev_instance_t *)dev, &diag_3k->sts);
    } else if (type == UWB_RXDIAG_STS2) {
        if ((diag_3k->rxd.valid & UWB_RXDIAG_STS2) == 0) {
            return DPL_FLOAT32_NAN();
        }
        return dw3000_calc_rssi((dw3000_dev_instance_t *)dev, &diag_3k->sts2);
    }
    if ((diag_3k->rxd.valid & UWB_RXDIAG_IPATOV) == 0) {
        return DPL_FLOAT32_NAN();
    }
    return dw3000_calc_rssi((dw3000_dev_instance_t *)dev, &diag_3k->ipatov);
}

inline static dpl_float32_t
uwb_dw3000_calc_fppl(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag)
{
    struct _dw3000_dev_rxdiag_t* diag_3k = (struct _dw3000_dev_rxdiag_t*)diag;
    return dw3000_calc_fppl((dw3000_dev_instance_t *)dev, &diag_3k->ipatov);
}

inline static dpl_float32_t
uwb_dw3000_estimate_los(struct uwb_dev * dev, dpl_float32_t rssi, dpl_float32_t fppl)
{
    return dw3000_estimate_los(rssi, fppl);
}

static dpl_float32_t uwb_dw3000_calc_pdoa(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag)
{
    struct _dw3000_dev_rxdiag_t *dw3k_diag = (struct _dw3000_dev_rxdiag_t*)diag;
    if (dev->config.rx.pdoaMode == DWT_PDOA_M0 ||
        dev->config.rx.stsMode == DWT_STS_MODE_OFF) {
        return DPL_FLOAT32_NAN();
    }
    /* Check sts quality */
    if (dw3k_diag->sts.accumCount < ((dw3000_dev_instance_t *)dev)->sts_threshold) {
        return DPL_FLOAT32_NAN();
    }
    return DPL_FLOAT32_DIV(DPL_FLOAT32_I32_TO_F32(dw3k_diag->pdoa),
                           DPL_FLOAT32_I32_TO_F32((int32_t)1<<11));
}

inline static struct uwb_dev_status
uwb_dw3000_mac_framefilter(struct uwb_dev * dev, uint16_t enable)
{
    /* The UWB_FF_* is the same as the DWT_FF_* for dw3000 */
    return dw3000_mac_framefilter((dw3000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw3000_set_autoack(struct uwb_dev * dev, bool enable)
{
    return dw3000_set_autoack((dw3000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw3000_set_autoack_delay(struct uwb_dev * dev, uint8_t delay)
{
    return dw3000_set_autoack_delay((dw3000_dev_instance_t *)dev, delay);
}

inline static struct uwb_dev_status
uwb_dw3000_event_cnt_ctrl(struct uwb_dev * dev, bool enable, bool reset)
{
    return dw3000_phy_event_cnt_ctrl((dw3000_dev_instance_t *)dev, enable, reset);
}

inline static struct uwb_dev_status
uwb_dw3000_event_cnt_read(struct uwb_dev * dev, struct uwb_dev_evcnt *res)
{
    return dw3000_phy_event_cnt_read((dw3000_dev_instance_t *)dev, res);
}


static const struct uwb_driver_funcs dw3000_uwb_funcs = {
    .uf_mac_config = uwb_dw3000_mac_config,
    .uf_txrf_config = uwb_dw3000_txrf_config,
    .uf_txrf_power_value = uwb_dw3000_txrf_power_value,
    .uf_sleep_config = uwb_dw3000_sleep_config,
    .uf_enter_sleep = uwb_dw3000_enter_sleep,
    .uf_enter_sleep_after_tx = uwb_dw3000_enter_sleep_after_tx,
    .uf_enter_sleep_after_rx = uwb_dw3000_enter_sleep_after_rx,
    .uf_wakeup = uwb_dw3000_wakeup,
    .uf_set_dblrxbuf = uwb_dw3000_set_dblrxbuf,
    .uf_set_rx_timeout = uwb_dw3000_set_rx_timeout,
    .uf_adj_rx_timeout = uwb_dw3000_adj_rx_timeout,
    .uf_set_rx_window = uwb_dw3000_set_rx_window,
    .uf_set_abs_timeout = uwb_dw3000_set_abs_timeout,
    .uf_set_delay_start = uwb_dw3000_set_delay_start,
    .uf_set_cca_tx = uwb_dw3000_set_cca_tx,
    .uf_start_tx = uwb_dw3000_start_tx,
    .uf_start_rx = uwb_dw3000_start_rx,
    .uf_stop_rx = uwb_dw3000_stop_rx,
    .uf_write_tx = uwb_dw3000_write_tx,
    .uf_write_tx_fctrl_ext = uwb_dw3000_write_tx_fctrl,
    .uf_hal_noblock_wait = uwb_dw3000_hal_noblock_wait,
    .uf_tx_wait = uwb_dw3000_tx_wait,
    .uf_set_wait4resp = uwb_dw3000_set_wait4resp,
    .uf_set_wait4resp_delay = uwb_dw3000_set_wait4resp_delay,
    .uf_set_rxauto_disable = uwb_dw3000_set_rxauto_disable,
    .uf_read_systime = uwb_dw3000_read_systime,
    .uf_read_systime_lo32 = uwb_dw3000_read_systime_lo32,
    .uf_read_rxtime = uwb_dw3000_read_rxtime,
    .uf_read_rxtime_lo32 = uwb_dw3000_read_rxtime_lo32,
    .uf_read_sts_rxtime = uwb_dw3000_read_sts_rxtime,
    .uf_read_txtime = uwb_dw3000_read_txtime,
    .uf_read_txtime_lo32 = uwb_dw3000_read_txtime_lo32,
    .uf_phy_frame_duration = uwb_dw3000_phy_frame_duration,
    .uf_phy_SHR_duration = uwb_dw3000_phy_SHR_duration,
    .uf_phy_data_duration = uwb_dw3000_phy_data_duration,
    .uf_phy_forcetrxoff = uwb_dw3000_phy_forcetrxoff,
    .uf_phy_rx_reset = uwb_dw3000_phy_rx_reset,
    .uf_phy_repeated_frames = uwb_dw3000_phy_repeated_frames,
    .uf_set_on_error_continue = uwb_dw3000_set_on_error_continue,
    .uf_set_panid = uwb_dw3000_set_panid,
    .uf_set_uid = uwb_dw3000_set_uid,
    .uf_set_euid = uwb_dw3000_set_euid,
    .uf_calc_clock_offset_ratio = uwb_dw3000_calc_clock_offset_ratio,
    .uf_get_rssi = uwb_dw3000_get_rssi,
    .uf_get_fppl = uwb_dw3000_get_fppl,
    .uf_calc_rssi = uwb_dw3000_calc_rssi,
    .uf_calc_seq_rssi = uwb_dw3000_calc_seq_rssi,
    .uf_calc_fppl = uwb_dw3000_calc_fppl,
    .uf_estimate_los = uwb_dw3000_estimate_los,
    .uf_calc_pdoa = uwb_dw3000_calc_pdoa,
    .uf_mac_framefilter = uwb_dw3000_mac_framefilter,
    .uf_set_autoack = uwb_dw3000_set_autoack,
    .uf_set_autoack_delay = uwb_dw3000_set_autoack_delay,
    .uf_event_cnt_ctrl = uwb_dw3000_event_cnt_ctrl,
    .uf_event_cnt_read = uwb_dw3000_event_cnt_read,
};

/**
 * API to initialize a dw3000_dev_instance_t structure from the os device initialization callback.
 *
 * @param odev  Pointer to struct os_dev.
 * @param arg   Argument to set as pointer to struct dw3000_dev_cfg.
 * @return OS_OK on success
 */
int
dw3000_dev_init(struct os_dev *odev, void *arg)
{
    dpl_error_t err;
    struct dw3000_dev_cfg *cfg = (struct dw3000_dev_cfg*)arg;
    struct uwb_dev *udev = (struct uwb_dev*)odev;
    dw3000_dev_instance_t *inst = (dw3000_dev_instance_t *)odev;
#if MYNEWT_VAL(UWB_PKG_INIT_LOG)
    DIAGMSG("{\"utime\": %"PRIu32",\"msg\": \"dw3000_dev_init\"}\n",
            dpl_cputime_ticks_to_usecs(dpl_cputime_get32()));
#endif

    if (inst == NULL) {
        inst = (dw3000_dev_instance_t *) calloc(1, sizeof(dw3000_dev_instance_t));
        assert(inst);
        inst->uwb_dev.status.selfmalloc = 1;
        udev = (struct uwb_dev*)inst;
    }

    udev->rxbuf_size = MYNEWT_VAL(UWB_RX_BUFFER_SIZE);
    udev->txbuf_size = MYNEWT_VAL(DW3000_HAL_SPI_BUFFER_SIZE);
    uwb_dev_init(udev);

    /* Setup common uwb interface */
    udev->uw_funcs = &dw3000_uwb_funcs;
    udev->rxdiag = (struct uwb_dev_rxdiag*)&inst->rxdiag;
    inst->rxdiag.rxd.rxd_len = sizeof(inst->rxdiag);
#if MYNEWT_VAL(CIR_ENABLED)
    udev->cir = (struct cir_instance*)inst->cir;
#endif
#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
    inst->sys_status_bt_idx = 0;
    inst->sys_status_bt_lock = 0;
#endif

    /* Set which capabilities we already know */
    udev->capabilities.sts = 1;

    /* Check size requirements */
    assert(sizeof(inst->rxdiag) <= MYNEWT_VAL(UWB_DEV_RXDIAG_MAXLEN));

    /* Capture dev_cfg parameters */
#if MYNEWT_VAL(BUS_DRIVER_PRESENT) == 0
    inst->spi_sem = cfg->spi_sem;
    inst->spi_num = cfg->spi_num;
    inst->spi_settings.baudrate = cfg->spi_baudrate;
#endif
    inst->irq_pin = cfg->irq_pin;
    inst->rst_pin = cfg->rst_pin;
    inst->ss_pin  = cfg->ss_pin;
    udev->rx_antenna_delay = cfg->rx_antenna_delay;
    udev->tx_antenna_delay = cfg->tx_antenna_delay;
    udev->ext_clock_delay = cfg->ext_clock_delay;

    err = dpl_mutex_init(&inst->mutex);
    assert(err == DPL_OK);
    err = dpl_sem_init(&inst->tx_sem, 0x1);
    assert(err == DPL_OK);
    err = dpl_sem_init(&inst->spi_nb_sem, 0x1);
    assert(err == DPL_OK);

    /* phy attritubes per the IEEE802.15.4-2011 standard, Table 99 and Table 101 */
    udev->attrib.Tpsym = DPL_FLOAT32_INIT(1.0176282f); //!< Preamble symbols duration (usec) for MPRF of 62.89Mhz
    udev->attrib.Tbsym = DPL_FLOAT32_INIT(1.0256410f); //!< Baserate symbols duration (usec) 850khz
    udev->attrib.Tdsym = DPL_FLOAT32_INIT(0.1282051f); //!< Datarate symbols duration (usec) 6.81Mhz

    SLIST_INIT(&inst->uwb_dev.interface_cbs);

#if MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN) || MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
    inst->bt_ticks2usec = 1000000/MYNEWT_VAL(OS_CPUTIME_FREQ);
#endif

    /* Precalculate CRC8 LUT */
    _dwt_crc8init();
    return DPL_OK;
}

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
static void
dw3000_init_node_cb(struct bus_node *bnode, void *arg)
{
    int err = dw3000_dev_init((struct os_dev *)bnode, arg);
    assert(!err);
}

static struct bus_node_callbacks dw3000_bus_node_cbs = {
        .init = dw3000_init_node_cb,
    };

int
dw3000_create_spi_dev(struct bus_spi_node *node, const char *name, struct dw3000_dev_cfg *cfg)
{
    bus_node_set_callbacks((struct os_dev *)node, &dw3000_bus_node_cbs);
    // this will call dw3000_dev_init()
    return bus_spi_node_create(name, node, &cfg->spi_node_cfg, cfg);
}
#endif

/**
 * API to free the acquired resources.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
void
dw3000_dev_deinit(dw3000_dev_instance_t * inst)
{
    assert(inst);
#if MYNEWT_VAL(BUS_DRIVER_PRESENT) == 0
    hal_spi_disable(inst->spi_num);
#endif

    /* De-Initialise task structures in uwb_dev */
    uwb_task_deinit(&inst->uwb_dev);
    hal_gpio_irq_disable(inst->irq_pin);
    hal_gpio_irq_release(inst->irq_pin);

    if (inst->uwb_dev.status.selfmalloc) {
        free(inst);
    } else {
        inst->uwb_dev.status.initialized = 0;
    }
}
