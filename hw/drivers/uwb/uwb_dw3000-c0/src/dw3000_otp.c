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
 * @file dw3000_otp.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief one time programmable memory
 *
 * @details This is the otp base class which utilises functions to read from the address specified in the OTP_ADDR register.
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dpl/dpl_cputime.h>

#include <dw3000-c0/dw3000_regs.h>
#include <dw3000-c0/dw3000_dev.h>
#include <dw3000-c0/dw3000_hal.h>
#include <dw3000-c0/dw3000_phy.h>
#include <dw3000-c0/dw3000_otp.h>

/**
 * API takes the given address and enables otp_read from the succeeding address.
 *
 * @param inst     Pointer to dw3000_dev_instance_t.
 * @param address  From where it starts reading.
 * @param buffer   Result is stored into buffer.
 * @param length   Represents length of the buffer.
 * @return void
 *
 */
void dw3000_phy_otp_read(struct _dw3000_dev_instance_t * inst, uint32_t address, uint32_t * buffer, uint16_t length)
{
    uint16_t i;
    dw3000_phy_sysclk_XTAL(inst); // NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _dwt_otpread are reliable

    for(i=0; i<length; i++) {
        buffer[i] = _dw3000_otp_read(inst, address + i);
    }

    dw3000_phy_sysclk_SEQ(inst);
}

/**
 * API to perform  read operation from the address specified.
 *
 * @param inst     Pointer to dw3000_dev_instance_t.
 * @param address  From where it starts reading
 * @return data value read from an OTP location.
 */

uint32_t _dw3000_otp_read(struct _dw3000_dev_instance_t * inst, uint16_t address)
{
    uint32_t ret_data;

    // Set manual access mode
    dw3000_write_reg(inst, NVM_CFG_ID, 0, 0x0001, sizeof(uint16_t));

    // set the address
    dw3000_write_reg(inst, NVM_ADDR_ID, 0, address, sizeof(uint16_t));

    // Assert the read strobe
    dw3000_write_reg(inst, NVM_CFG_ID, 0, 0x0002, sizeof(uint16_t));

    // attempt a read from OTP address
    ret_data = dw3000_read_reg(inst, NVM_RDATA_ID, 0, sizeof(uint32_t));

    return ret_data;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief For each value to send to OTP bloc, following two register writes are required as shown below
 *
 * @param val: 16-bit value to write to the OTP block
 */
void __dwt_otp_write_wdata_id_reg(struct _dw3000_dev_instance_t * inst, int16_t val)
{
    /* Pull the CS high to enable user interface for programming */
	/* 'val' is ignored in this instance by the OTP block */
    dw3000_write_reg(inst, NVM_WDATA_ID, 0, 0x0200 | val, sizeof(uint16_t));
	/* Send the relevant command to the OTP block */
    dw3000_write_reg(inst, NVM_WDATA_ID, 0, 0x0000 | val, sizeof(uint16_t));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief function to program the OTP memory.
 * Note the address is only 11 bits long.
 *
 * input parameters
 * @param data - data to write to given address
 * @param address - address to write to
 *
 * output parameters
 *
 * returns None
 */
int _dwt_otpprogword32(struct _dw3000_dev_instance_t * inst, uint32_t data, uint16_t address)
{
    uint16_t wr_buf[4];

    /* Read current register value */
    uint32_t ldo_tune = dw3000_read_reg(inst, LDO_TUNE_HI_ID, 0, sizeof(ldo_tune));

    /* Set VDDHV_TX LDO to max */
    dw3000_modify_reg(inst, LDO_TUNE_HI_ID, 0, 0xFFFFFFFFUL,
                      LDO_TUNE_HI_LDO_HVAUX_TUNE_BIT_MASK, sizeof(uint32_t));
    //printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));

    /* configure mode for programming */
    dw3000_write_reg(inst, NVM_CFG_ID, 0,
                     (NVM_CFG_NVM_MODE_SEL_BIT_MASK|NVM_CFG_NVM_WRITE_MR_BIT_MASK),
                     sizeof(uint16_t));

    /* Select fast programming */
    __dwt_otp_write_wdata_id_reg(inst, 0x0025);

    printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));

     /* Apply instruction to write the address */
    __dwt_otp_write_wdata_id_reg(inst, 0x0002);
    __dwt_otp_write_wdata_id_reg(inst, 0x01fc);

    /* Now sending the OTP address data (2 bytes) */
    wr_buf[0] = 0x0100 | (address & 0xFF);
    __dwt_otp_write_wdata_id_reg(inst, wr_buf[0]);

    /* Write data (upper byte of address) */
    __dwt_otp_write_wdata_id_reg(inst, 0x0100);

    /* Clean up */
    __dwt_otp_write_wdata_id_reg(inst, 0x0000);

    /* Apply instruction  to write data */
    __dwt_otp_write_wdata_id_reg(inst, 0x0002);
    __dwt_otp_write_wdata_id_reg(inst, 0x01c0);

    /* Write the data */
    wr_buf[0] = 0x100 | ((data >> 24) & 0xff);
    wr_buf[1] = 0x100 | ((data >> 16) & 0xff);
    wr_buf[2] = 0x100 | ((data >> 8) & 0xff);
    wr_buf[3] = 0x100 | (data & 0xff);
    __dwt_otp_write_wdata_id_reg(inst, wr_buf[3]);
    __dwt_otp_write_wdata_id_reg(inst, wr_buf[2]);
    __dwt_otp_write_wdata_id_reg(inst, wr_buf[1]);
    __dwt_otp_write_wdata_id_reg(inst, wr_buf[0]);

    /* Clean up */
    __dwt_otp_write_wdata_id_reg(inst, 0x0000);

    /* Enter prog mode */
    __dwt_otp_write_wdata_id_reg(inst, 0x003a);
    __dwt_otp_write_wdata_id_reg(inst, 0x01ff);
    __dwt_otp_write_wdata_id_reg(inst, 0x010a);
    /* Clean up */
    __dwt_otp_write_wdata_id_reg(inst, 0x0000);
#if 0
    printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));
	/* Enable state/status output */
    __dwt_otp_write_wdata_id_reg(inst, 0x003a);
    __dwt_otp_write_wdata_id_reg(inst, 0x01bf);
    __dwt_otp_write_wdata_id_reg(inst, 0x0100);
    /* Clean up !?!?! */
    //__dwt_otp_write_wdata_id_reg(inst, 0x0000);
    printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));
#endif
    /* Check that VDDHV is successful */
#if 0
    uint32_t reg;
    uint16_t timeout;
    printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));
    timeout = 0xf;
    reg = dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint32_t));
    printf("reg: %lX\n", reg);
    while ((reg & NVM_STATUS_NVM_VPP_OK_BIT_MASK) == 0 && --timeout) {
        dpl_cputime_delay_usecs(100);
        reg = dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint32_t));
        printf("reg: %lX\n", reg);
    }
    if (!timeout) {
        printf("VDDHV timeout\n");
        /* configure mode for reading */
        dw3000_write_reg(inst, NVM_CFG_ID, 0, 0x0000, sizeof(uint16_t));

        /* Restore LDO tune register */
        dw3000_write_reg(inst, LDO_TUNE_HI_ID, 0, ldo_tune, sizeof(uint32_t));
        return DPL_ERROR;
    }
#endif

    /* Start prog mode */
    __dwt_otp_write_wdata_id_reg(inst, 0x003a);
	__dwt_otp_write_wdata_id_reg(inst, 0x0101);
    dw3000_write_reg(inst, NVM_WDATA_ID, 0, 0x0002, sizeof(uint16_t));
    dw3000_write_reg(inst, NVM_WDATA_ID, 0, 0x0000, sizeof(uint16_t));

    //printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));
    /*
    read status after program command.
    The for loop will exit once the status indicates programming is complete or if it reaches the max 1000 iterations.
    1000 is more than sufficient for max OTP programming delay and max supported DW3000 SPI rate.
    Instead a delay of 2ms (as commented out below) can be used.
    Burn time is about 1.76ms
    */

    /*uint16	i;

    for (i = 0; i < 1000; i++)
    {
        uint32_t rd_buf;
        rd_buf = dwt_read32bitoffsetreg(NVM_STATUS_ID, 0);

        if (!(rd_buf & NVM_STATUS_NVM_PROG_DONE_BIT_MASK))
        {
            break;
        }
    }*/

    /* Uncomment this command if you don't want to use the loop above. It will take more time than the loop above. */
    dpl_cputime_delay_usecs(2000);
    printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));

     /* Stop prog mode */
    __dwt_otp_write_wdata_id_reg(inst, 0x003a);
    __dwt_otp_write_wdata_id_reg(inst, 0x0102);
    dw3000_write_reg(inst, NVM_WDATA_ID, 0, 0x0002, sizeof(uint16_t));
    dw3000_write_reg(inst, NVM_WDATA_ID, 0, 0x0000, sizeof(uint16_t));
    printf("nvm_status@%d: %04x\n", __LINE__, (uint16_t)dw3000_read_reg(inst, NVM_STATUS_ID, 0, sizeof(uint16_t)));

    /* configure mode for reading */
    dw3000_write_reg(inst, NVM_CFG_ID, 0, 0x0000, sizeof(uint16_t));

    /* Restore LDO tune register */
    dw3000_write_reg(inst, LDO_TUNE_HI_ID, 0, ldo_tune, sizeof(uint32_t));

    return DPL_OK;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to program 32-bit value into the DW3000 OTP memory.
 *
 * input parameters
 * @param inst     Pointer to dw3000_dev_instance_t.
 * @param value    the 32-bit value to be programmed into OTP
 * @param address  the 16-bit OTP address into which the 32-bit value is programmed
 *
 * output parameters
 *
 * returns DPL_OK for success, or DPL_ERROR for error
 */
int dw3000_phy_otp_writeandverify(struct _dw3000_dev_instance_t * inst, uint32_t value, uint16_t address)
{
    /* program the word */
    int rc = _dwt_otpprogword32(inst, value, address);
    if (rc) {
        return rc;
    }

    /* check it is programmed correctly */
    printf("read@%x: %"PRIx32", value: %"PRIx32"\n", address, _dw3000_otp_read(inst, address), value);
    if(_dw3000_otp_read(inst, address) == value) {
        return DPL_OK;
    } else {
        return DPL_ERROR;
    }
}
