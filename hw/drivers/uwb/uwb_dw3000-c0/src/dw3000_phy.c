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
 * @file dw3000_phy.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief physical layer
 *
 * @details This is the phy base class which utilises the functions to set the clocks,initializes phy layer and configures the required
 * parameters.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <math.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw3000-c0/dw3000_phy.h>

#if 0
void dw3000_phy_enable_ext_pa(struct _dw3000_dev_instance_t* inst, bool enable)
{
        uint8_t buf[2] = {0x00,0x00};

        if(enable == true)
        {
                dw3000_gpio4_config_ext_pa(inst);

                dw3000_gpio5_config_ext_txe(inst);

                //if an external power amplifier is being used, TX fine grain power dequeencing must be disabled
                dw3000_write(inst, PMSC_ID, PMSC_TXFINESEQ_OFFSET, buf, 2);
        }else
        {
                //TODO, config as gpio mode
        }
}

void dw3000_phy_enable_ext_lna(struct _dw3000_dev_instance_t* inst, bool enable)
{
        if(enable == true)
        {
                dw3000_gpio6_config_ext_rxe(inst);
        }else
        {
                //TODO, config as gpio mode
        }
}
#endif

/**
 * API that force system clock to be the 19.2 MHz XTI clock.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
inline void dw3000_phy_sysclk_XTAL(struct _dw3000_dev_instance_t * inst){
#if 0
    uint8_t reg = (uint8_t) dw3000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    reg |= (uint8_t) PMSC_CTRL0_SYSCLKS_19M;
    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
#else
    assert(0);
#endif

}

/**
 * API that force system clock to be the 125 MHz PLL clock.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
inline void dw3000_phy_sysclk_PLL(struct _dw3000_dev_instance_t * inst){
#if 0
    uint8_t reg = (uint8_t) dw3000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    reg |= (uint8_t) PMSC_CTRL0_SYSCLKS_125M;
    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
#else
    assert(0);
#endif

}

/**
 * API to enable running of the LDE algorithm.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
void dw3000_phy_sysclk_LDE(struct _dw3000_dev_instance_t * inst){
#if 0
    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, 0x01, sizeof(uint8_t));
    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET + 1 , 0x03, sizeof(uint8_t));
#else
    assert(0);
#endif

}

/**
 * API to enable PLL2 on/off sequencing by SNIFF mode.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
inline void dw3000_phy_sysclk_SEQ(struct _dw3000_dev_instance_t * inst){
#if 0
    uint8_t reg = (uint8_t) dw3000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    dw3000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
#else
    assert(0);
#endif

}

/**
 * API to enable PLL2 on/off sequencing by SNIFF mode through masking of pmsc_ctrl_lo and pmsc_ctrl_hi.
 *
 * @param inst   Pointer to dw3000_dev_instance_t.
 * @param mode   Switch to the case specified.
 * @return void
 */
void
dw3000_phy_sysclk_ACC(struct _dw3000_dev_instance_t * inst, uint8_t mode)
{
    if (mode) {
        dw3000_modify_reg(inst, CLK_CTRL_ID, 0x0, 0xffff,
                          CLK_CTRL_ACC_MEM_CLK_ON_BIT_MASK |
                          CLK_CTRL_FORCE_ACC_CLK_BIT_MASK, sizeof(uint16_t));
    } else {
        dw3000_modify_reg(inst, CLK_CTRL_ID, 0x0,
                          (uint16_t)~(CLK_CTRL_ACC_MEM_CLK_ON_BIT_MASK |
                                      CLK_CTRL_FORCE_ACC_CLK_BIT_MASK),
                          0x0000, sizeof(uint16_t));
    }
}

/**
 *
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param clocks    Set of clocks to enable/disable.
 *
 * @return void
 */
void
dw3000_phy_force_clocks(struct _dw3000_dev_instance_t * inst, int clocks)
{
    uint16_t reg;
    if (clocks == FORCE_CLK_SYS_TX) {
        reg = CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK | CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;

        /* SYS_CLK_SEL = PLL */
        reg |= ((uint16_t) FORCE_SYSCLK_PLL) << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

        /* TX_CLK_SEL = ON */
        reg |= ((uint16_t) FORCE_CLK_PLL) << CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;

        /* TX_BUF_CLK = ON */
        reg |= CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK;

        dw3000_write_reg(inst, CLK_CTRL_ID, 0x0, reg, sizeof(uint16_t));
    }

    if (clocks == FORCE_CLK_AUTO) {
        //Restore auto clock mode
        dw3000_write_reg(inst, CLK_CTRL_ID, 0x0,
                         (uint16_t) (CLK_CTRL_FORCE_NVM_CLK_EN_BIT_MASK |
                                     CLK_CTRL_RX_BUFF_AUTO_CLK_BIT_MASK |
                                     CLK_CTRL_CODE_MEM_AUTO_CLK_BIT_MASK),
                         sizeof(uint16_t));
    }
}


/**
 * @fn dw3000_phy_setdwstate()
 *
 * @brief This function can place DW3000 into IDLE or IDLE_RC mode when it is not actively in TX or RX.
 *
 * input parameters
 * @param mode - 1 to put DW3000 into IDLE mode
 *
 * output parameters none
 *
 * no return value
 */
void
dw3000_phy_setdwstate(struct _dw3000_dev_instance_t * inst, int mode)
{
    // Set the AUTO INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system
    if (mode == DWT_DW_IDLE) {
        // dwt_or8bitoffsetreg(SEQ_CTRL_ID, 0x01, 0x1);
        dw3000_modify_reg(inst, SEQ_CTRL_ID, 1, 0xff,
                          SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK>>8, sizeof(uint8_t));
    } else if (mode == DWT_DW_IDLE_RC) {
        //switch clock to FOSC
        dw3000_modify_reg(inst, CLK_CTRL_ID, 0, 0xff, FORCE_SYSCLK_FOSC,
                          sizeof(uint8_t));
        //clear the auto INIT2IDLE bit and set FORCE2INIT
        dw3000_modify_reg(inst, SEQ_CTRL_ID, 0,
                          (uint32_t) ~SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK,
                          SEQ_CTRL_FORCE2INIT_BIT_MASK, sizeof(uint32_t));
        //set FORCE2RC - this bit needs to be set when device is in INIT state
        dw3000_modify_reg(inst, CLK_CTRL_ID, 2, 0xffff,
                          (uint16_t)(SEQ_CTRL_FORCE2RC_BIT_MASK>>16),
                          sizeof(uint16_t));
        //clear force bits (device will stay in IDLE_RC)
        dw3000_modify_reg(inst, SEQ_CTRL_ID, 2,
                          (uint16_t) ~((SEQ_CTRL_FORCE2RC_BIT_MASK|SEQ_CTRL_FORCE2INIT_BIT_MASK)>>16),
                          0, sizeof(uint16_t));
        //switch clock to auto
        dw3000_phy_force_clocks(inst, FORCE_CLK_AUTO);
    } else {
        /* NOTE: the SPI rate needs to be <= 7MHz as device is switching to INIT_RC state */
        dw3000_modify_reg(inst, CLK_CTRL_ID, 0, 0xff,
                          FORCE_SYSCLK_FOSCDIV4, sizeof(uint8_t));
        /* clear the auto INIT2IDLE bit and set FORCE2INIT */
        dw3000_modify_reg(inst, SEQ_CTRL_ID, 0,
                          (uint32_t) ~SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK,
                          SEQ_CTRL_FORCE2INIT_BIT_MASK, sizeof(uint32_t));
        dw3000_modify_reg(inst, SEQ_CTRL_ID, 2, (uint8_t)~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16),
                          0, sizeof(uint8_t));
        /* switch clock to auto */
        dw3000_phy_force_clocks(inst, FORCE_CLK_AUTO);
    }
}

/**
 * @brief This function sets the default values of the lookup
 * tables depending on the channel selected
 *
 * input parameters
 * @param inst Pointer to struct _dw3000_dev_instance_t.
 * @param[in] channel - Channel that the device will be transmitting/receiving on.
 *
 * no return value
 */
void
dw3000_phy_config_mrx_lut(struct _dw3000_dev_instance_t * inst, int channel)
{
        /*
        Channel 5
        Gain                        SW      LNA1LNA2 MIXER  PGF
        NF + 0dB                    011100  000  01   1  111 101 10011 0
        MaxGain - 12dB, NF + 6dB    011100  010  00   1  111 101 10011 1
        MaxGain - 17dB, NF + 10dB   011100  011  00   1  111 101 10011 2
        MaxGain - 28dB, NF + 16dB   011100  011  11   1  111 101 10011 3
        MaxGain - 32dB, NF + 23dB   011100  111  10   1  111 101 10011 4
        MaxGain - 37dB, NF + 29dB   011100  111  11   1  111 101 10011 5
        MaxGain - 39dB, NF + 30dB   001111  111  11   1  111 101 10011 6

        Channel 9
        Gain                        SW      LNA1LNA2 MIXER  PGF
        Max Gain, NF                101010  000  00   1  111 101 10011 - 1 0101 0000 0111 1101 = 1507d
        Max Gain-12dB, NF+4dB       101010  001  11   1  111 101 10011
        Max Gain-20dB, NF+10dB      101010  010  10   1  111 101 10011
        Max Gain-24dB, NF+16dB      101010  011  10   1  111 101 10011
        Max Gain-30dB, NF+20dB      101010  011  11   1  111 101 10011
        Max Gain-33dB, NF+24dB      101010  110  10   1  111 101 10011
        Max Gain-40dB, NF+28dB      101010  111  11   1  111 101 10011*
        */
        //Channel5
        //[17:12]SW                 [11:9]LNA1
        //                                 [8:7]LNA2
        //                                      [6:3]MIXER [2:0]PGF
        //19 18 17 16--15 14 13 12--11 10 9 8--7 6 5 4--3 2 1 0
        //0  0  0  1 --1   1  0  0--0  0  0 0--1 1 1 1--1 1 0 1 = 1c0fd
        //0  0  0  1 --1   1  0  0--0  1  0 0--0 1 1 1--1 1 0 1 = 1c47d -12
        //0  0  0  1 --1   1  0  0--0  1  1 0--0 1 1 1--1 1 0 1 = 1c67d -17
        //0  0  0  1 --1   1  0  0--0  1  1 1--1 1 1 1--1 1 0 1 = 1c7fd -28
        //0  0  0  1 --1   1  0  0--1  1  1 1--0 1 1 1--1 1 0 1 = 1cf7d -32
        //0  0  0  1 --1   1  0  0--1  1  1 1--1 1 1 1--1 1 0 1 = 1cffd -37
        //0  0  0  0 --1   1  1  1--1  1  1 1--1 1 1 1--1 1 0 1 = 0fffd -39

        //Channel9
        //[17:12]SW                 [11:9]LNA1
        //                                 [8:7]LNA2
        //                                      [6:3]MIXER [2:0]PGF
        //19 18 17 16--15 14 13 12--11 10 9 8--7 6 5 4--3 2 1 0
        //0  0  1  0 --1   0  1  0--0  0  0 0--0 1 1 1--1 1 0 1 = 2a07d
        //0  0  1  0 --1   0  1  0--0  0  1 1--1 1 1 1--1 1 0 1 = 2a3fd -12
        //0  0  1  0 --1   0  1  0--0  1  0 1--0 1 1 1--1 1 0 1 = 2a57d -20
        //0  0  1  0 --1   0  1  0--0  1  1 1--0 1 1 1--1 1 0 1 = 2a77d -24
        //0  0  1  0 --1   0  1  0--0  1  1 1--1 1 1 1--1 1 0 1 = 2a7fd -30
        //0  0  1  0 --1   0  1  0--1  1  0 1--0 1 1 1--1 1 0 1 = 2ad7d -33
        //0  0  1  0 --1   0  1  0--1  1  1 1--1 1 1 1--1 1 0 1 = 2affd -40

    uint32_t lut0, lut1, lut2, lut3, lut4, lut5, lut6 = 0;


    if (channel == 5) {
        lut0 = (uint32_t)CH5_MAX_GAIN_NF;
        lut1 = (uint32_t)CH5_MAX_GAIN_MINUS_12DB_NF_PLUS_6DB;
        lut2 = (uint32_t)CH5_MAX_GAIN_MINUS_17DB_NF_PLUS_10DB;
        lut3 = (uint32_t)CH5_MAX_GAIN_MINUS_28DB_NF_PLUS_16DB;
        lut4 = (uint32_t)CH5_MAX_GAIN_MINUS_32DB_NF_PLUS_23DB;
        lut5 = (uint32_t)CH5_MAX_GAIN_MINUS_37DB_NF_PLUS_29DB;
        lut6 = (uint32_t)CH5_MAX_GAIN_MINUS_39DB_NF_PLUS_30DB;
    } else {
        lut0 = (uint32_t)CH9_MAX_GAIN_NF;
        lut1 = (uint32_t)CH9_MAX_GAIN_MINUS_12DB_NF_PLUS_4DB;
        lut2 = (uint32_t)CH9_MAX_GAIN_MINUS_20DB_NF_PLUS_10DB;
        lut3 = (uint32_t)CH9_MAX_GAIN_MINUS_24DB_NF_PLUS_16DB;
        lut4 = (uint32_t)CH9_MAX_GAIN_MINUS_30DB_NF_PLUS_20DB;
        lut5 = (uint32_t)CH9_MAX_GAIN_MINUS_33DB_NF_PLUS_24DB;
        lut6 = (uint32_t)CH9_MAX_GAIN_MINUS_40DB_NF_PLUS_28DB;
    }
    dw3000_write_reg(inst, DGC_DGC_LUT_0_CFG_ID, 0x0, lut0, sizeof(uint32_t));
    dw3000_write_reg(inst, DGC_DGC_LUT_1_CFG_ID, 0x0, lut1, sizeof(uint32_t));
    dw3000_write_reg(inst, DGC_DGC_LUT_2_CFG_ID, 0x0, lut2, sizeof(uint32_t));
    dw3000_write_reg(inst, DGC_DGC_LUT_3_CFG_ID, 0x0, lut3, sizeof(uint32_t));
    dw3000_write_reg(inst, DGC_DGC_LUT_4_CFG_ID, 0x0, lut4, sizeof(uint32_t));
    dw3000_write_reg(inst, DGC_DGC_LUT_5_CFG_ID, 0x0, lut5, sizeof(uint32_t));
    dw3000_write_reg(inst, DGC_DGC_LUT_6_CFG_ID, 0x0, lut6, sizeof(uint32_t));
}


/**
 * API to initialise the phy layer.
 *
 * @param inst         Pointer to dw3000_dev_instance_t.
 * @param txrf_config  Pointer to dw3000_dev_txrf_config_t.
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw3000_phy_init(struct _dw3000_dev_instance_t * inst, struct uwb_dev_txrf_config * txrf_config)
{
    uint32_t ldo_tune_lo, ldo_tune_hi;
    uint16_t bias_tune;

    if (txrf_config == NULL)
        txrf_config = &inst->uwb_dev.config.txrf;
    else
        memcpy(&inst->uwb_dev.config.txrf, txrf_config, sizeof(struct uwb_dev_txrf_config));

#if MYNEWT_VAL(DW3000_RXTX_LEDS)
    dw3000_gpio_config_leds(inst, DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
#endif

#if MYNEWT_VAL(DW3000_RXTX_GPIO)
    dw3000_gpio0_config_ext_txe(inst);
    dw3000_gpio1_config_ext_rxe(inst);
#endif

    /* Load LDO and bias tune values from OTP */
    ldo_tune_lo = _dw3000_otp_read(inst, OTP_LDOTUNELO_ADDRESS);
    ldo_tune_hi = _dw3000_otp_read(inst, OTP_LDOTUNEHI_ADDRESS);
    bias_tune = (uint16_t)_dw3000_otp_read(inst, OTP_BIAS_TUNE_ADDRESS);

    if ((ldo_tune_lo != 0) && (ldo_tune_hi != 0) && (bias_tune != 0)) {
        dw3000_modify_reg(inst, NVM_CFG_ID, 0, 0xffff, LDO_BIAS_KICK, sizeof(uint16_t));
        inst->sleep_mode |= DWT_LOADBIAS | DWT_LOADLDO;
    }

    // Load Part and Lot ID from OTP
    inst->part_id = _dw3000_otp_read(inst, OTP_PARTID_ADDRESS);
    inst->lot_id = _dw3000_otp_read(inst, OTP_LOTID_ADDRESS);

    // Load vbat and vtemp from OTP
    inst->otp_vbat = _dw3000_otp_read(inst, OTP_VBAT_ADDRESS);
    if (inst->otp_vbat == 0x0) {
        inst->otp_vbat = 0x74;
    }
    inst->otp_temp = _dw3000_otp_read(inst, OTP_VTEMP_ADDRESS);
    if (inst->otp_temp == 0x0) {
        inst->otp_temp = 0x85;
    }

    // Apply tx power settings */
    dw3000_phy_config_txrf(inst, txrf_config);

    // Read system register / store local copy
    inst->sys_cfg_reg = dw3000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)) ; // Read sysconfig register

    return inst->uwb_dev.status;
}

/** @fn dw3000_phy_setfinegraintxseq()
 *
 * @brief This function enables/disables the fine grain TX sequencing (enabled by default).
 *
 * input parameters
 * @param enable - 1 to enable fine grain TX sequencing, 0 to disable it.
 *
 * output parameters none
 *
 * no return value
 */
void
dw3000_phy_setfinegraintxseq(struct _dw3000_dev_instance_t * inst, int enable)
{
    /* NOTE: intentionally writing across register boundries */
    if (enable) {
        dw3000_write_reg(inst, PWR_UP_TIMES_LO_ID, 0x2,
                         PMSC_TXFINESEQ_ENABLE, sizeof(uint32_t));
    } else {
        dw3000_write_reg(inst, PWR_UP_TIMES_LO_ID, 0x2,
                         PMSC_TXFINESEQ_DISABLE, sizeof(uint32_t));
    }
}


/**
 * API for the configuration of the TX spectrum
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type dwt_txconfig_t that holds all the configurable items.
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param config    Pointer to dw3000_dev_txrf_config_t.
 * @return void
 */
void dw3000_phy_config_txrf(struct _dw3000_dev_instance_t * inst, struct uwb_dev_txrf_config *config)
{
    // Configure RF TX PG_DELAY
    dw3000_write_reg(inst, TX_CTRL_HI_ID, 0, config->PGdly, sizeof(uint8_t));
    // Configure TX power
    dw3000_write_reg(inst, TX_POWER_ID, 0, config->power, sizeof(uint32_t));

    /* Ensure we update the antenna delays when updating txrf as well */
    dw3000_phy_set_rx_antennadelay(inst, inst->uwb_dev.rx_antenna_delay);
    dw3000_phy_set_tx_antennadelay(inst, inst->uwb_dev.tx_antenna_delay);
    //return inst->uwb_dev.status;
}

#if 0

/**
 * API to read the temperature of the DW3000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep.
 *
 * @param inst    Pointer to dw3000_dev_instance_t.
 * @return float  value for temperature sensor in SI units (Degrees C).
 */
float dw3000_phy_read_wakeuptemp_SI(struct _dw3000_dev_instance_t * inst)
{
    return 1.14 * (dw3000_phy_read_wakeuptemp(inst) - inst->otp_temp) + 23;
}

/**
 * API to read the battery voltage of the DW3000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configure_sleep.
 *
 * @param inst    Pointer to dw3000_dev_instance_t.
 * @return float  Value of battery voltage sensor in SI units (Volts).
 */
float dw3000_phy_read_read_wakeupvbat_SI(struct _dw3000_dev_instance_t * inst)
{
    return (1.0/173) * (dw3000_phy_read_wakeupvbat(inst) - inst->otp_vbat) + 3.3;
}

#endif


/**
 * API to reset the receiver of the DW3000.
 *
 * @param inst   Pointer to dw3000_dev_instance_t.
 * @return void
 */
void dw3000_phy_rx_reset(struct _dw3000_dev_instance_t * inst)
{
    dpl_error_t err = dpl_mutex_pend(&inst->mutex, DPL_WAIT_FOREVER);
    if (err != DPL_OK) {
        inst->uwb_dev.status.mtx_error = 1;
        goto mtx_error;
    }

    // Set RX reset
    dw3000_write_reg(inst, SOFT_RST_ID, 0, DWT_RESET_RX, sizeof(uint8_t));

    // Clear RX reset
    dw3000_write_reg(inst, SOFT_RST_ID, 0, DWT_RESET_CLEAR, sizeof(uint8_t));

    err = dpl_mutex_release(&inst->mutex);
    assert(err == DPL_OK);
mtx_error:
    return;
}


/**
 * API to turn off the transceiver.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
void dw3000_phy_forcetrxoff(struct _dw3000_dev_instance_t * inst)
{
    struct uwb_mac_interface * cbs = NULL;
    uint32_t mask = dw3000_read_reg(inst, SYS_ENABLE_LO_ID, 0 , sizeof(uint32_t)) ; // Read set interrupt mask

    // Need to beware of interrupts occurring in the middle of following read modify write cycle
    // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
    // event has just happened before the radio was disabled)
    // thus we need to disable interrupt during this operation

    dpl_error_t err = dpl_mutex_pend(&inst->mutex, DPL_WAIT_FOREVER);
    if (err != DPL_OK) {
        inst->uwb_dev.status.mtx_error = 1;
        goto mtx_error;
    }

    /* C0-bug: A force txrxoff from idle-rc causes malfunction - check the state
     * before issuing txrxoff */
    if ((dw3000_read_reg(inst, SYS_STATE_LO_ID, 0, sizeof(uint32_t))&SYS_STATE_TX_RX_IDLE_MASK) == 0x00) {
        /* Already in idle, abort */
        goto already_idle_return;
    }

    /* Clear interrupt mask - so we don't get any unwanted events */
    dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, 0, sizeof(uint32_t));
    dw3000_write_fast_CMD(inst, CMD_TXRXOFF);

    /* Forcing Transceiver off - so we do not want to see any new
     * events that may have happened */
    dw3000_write_fast_CMD(inst, CMD_CLR_IRQS);
    /* Restore mask to what it was */
    dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, mask, sizeof(uint32_t));

already_idle_return:
    if(!(SLIST_EMPTY(&inst->uwb_dev.interface_cbs))){
        SLIST_FOREACH(cbs, &inst->uwb_dev.interface_cbs, next){
            if (cbs!=NULL && cbs->reset_cb)
                if(cbs->reset_cb((struct uwb_dev*)inst, cbs)) continue;
        }
    }
    /* Enable/restore interrupts again... */
    err = dpl_mutex_release(&inst->mutex);
    assert(err == DPL_OK);

    inst->control.wait4resp_enabled = 0;
    inst->control.rxauto_disable = false;

    /* Reset semaphore if needed */
    if (dpl_sem_get_count(&inst->tx_sem) == 0) {
        dpl_error_t err = dpl_sem_release(&inst->tx_sem);
        assert(err == DPL_OK);
        inst->uwb_dev.status.sem_force_released = 1;
    }
mtx_error:
    return;
}

/**
 * API to enable the specified events to trigger an interrupt.
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param bitmask   Sets the events which generates interrupt.
 * @param enable    If set, the interrupts are enabled else they are cleared.
 * @return void
 */
void dw3000_phy_interrupt_mask(struct _dw3000_dev_instance_t * inst, uint64_t bitmask, uint8_t enable)
{
    /* Critical region, atomic lock with mutex */
    dpl_error_t err = dpl_mutex_pend(&inst->mutex, DPL_WAIT_FOREVER);
    if (err != DPL_OK) {
        inst->uwb_dev.status.mtx_error = 1;
        goto mtx_error;
    }

    if(enable) {
        dw3000_modify_reg(inst, SYS_ENABLE_LO_ID, 0, 0xffffffffU, bitmask&0xFFFFFFFFUL, sizeof(uint32_t));
        dw3000_modify_reg(inst, SYS_ENABLE_HI_ID, 0, 0xffffffffU, (bitmask>>32)&0xFFFFFFFFUL, sizeof(uint32_t));
    } else {
        dw3000_modify_reg(inst, SYS_ENABLE_LO_ID, 0, ~(bitmask&0xFFFFFFFFUL), 0x0, sizeof(uint32_t));
        dw3000_modify_reg(inst, SYS_ENABLE_HI_ID, 0, ~((bitmask>>32)&0xFFFFFFFFUL), 0x0, sizeof(uint32_t));
    }

    /* Critical region, unlock mutex */
    err = dpl_mutex_release(&inst->mutex);
    assert(err == DPL_OK);
mtx_error:
    return;
}

#if 0
/**
 * API to synchronise DW3000 with external clocks or events or with other DW3000’s.
 * For example, this would be required in a TDOA RTLS system employing wired clock synchronisation of the
 * anchor nodes or AoA node for phase measurement.
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param delay     To configure DW3000 for OSRS mode, the delay value is set to the desired delay value.
 * @param enable    True/false.
 * @return void
 */
void dw3000_phy_external_sync(struct _dw3000_dev_instance_t * inst, uint8_t delay, bool enable){

    uint32_t reg = dw3000_read_reg(inst, EC_CTRL_ID, 0, sizeof(uint32_t));
    if (enable) {
        reg &= ~EC_CTRL_WAIT_MASK; //clear timer value, clear OSTRM
        reg |= EC_CTRL_OSTRM;      //External timebase reset mode enable
        reg |= ((((uint16_t) delay) & 0xff) << 3); //set new timer value

    }else {
        reg &= ~(EC_CTRL_WAIT_MASK | EC_CTRL_OSTRM); //clear timer value, clear OSTRM
    }
    dw3000_write_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, reg, sizeof(uint16_t));
}

#endif

/**
 * @brief This function enables repeated frames to be generated given a frame repetition rate.
 *
 * input parameters:
 * @param inst     Pointer to dw3000_dev_instance_t.
 * @param rate     Rate at which frames with be repeated in dtu, set to 0 to disable
 *
 * @return void
 */
void
dw3000_phy_repeated_frames(struct _dw3000_dev_instance_t * inst, uint64_t rate)
{
    uint32_t reg;
    reg = dw3000_read_reg(inst, TEST_CTRL0_ID, 0x0, sizeof(reg));

    if (!rate) {
        /* Disable repeated frames */
        reg &= ~TEST_CTRL0_TPSTM_BIT_MASK;
        dw3000_write_reg(inst, TEST_CTRL0_ID, 0x0, reg, sizeof(reg));
    } else {
        /* Enable Repeated Frames */
        reg |= TEST_CTRL0_TPSTM_BIT_MASK;
        dw3000_write_reg(inst, TEST_CTRL0_ID, 0x0, reg, sizeof(reg));

        if (rate < 4) {
            rate = 4;
        }
        dw3000_write_reg(inst, DX_TIME_ID, 0, rate>>8, sizeof(uint32_t));

        /* Trigger first frame */
        dw3000_write_reg(inst, SYS_CTRL_ID, 0x0, SYS_CTRL_TXSTRT_BIT_MASK, sizeof(uint8_t));
    }
}

/**
 * API to calculate the SHR (Preamble + SFD) duration. This is used to calculate the correct rx_timeout.
 * @param attrib    Pointer to _phy_attributes_t * struct. The phy attritubes are part of the IEEE802.15.4-2011 standard.
 * Note the morphology of the frame depends on the mode of operation, see the dw3000_hal.c for the default behaviour.
 *             ------------------------------------------------
 * Sts Mode 0: | Preamble | SFD | RMARKER | PHR | PHY PAYLOAD |
 *             -----------------------------------------------------
 * Sts Mode 1: | Preamble | SFD | RMARKER | STS | PHR | PHY PAYLOAD|
 *             -----------------------------------------------------
 * Sts Mode 2: | Preamble | SFD | RMARKER | PHR | PHY PAYLOAD| STS |
 *             -----------------------------------------------------
 * Sts Mode 3: | Preamble | SFD | RMARKER | STS |
 *             ----------------------------------
 * @return uint16_t duration in usec
 */
uint16_t
dw3000_phy_SHR_duration(struct uwb_phy_attributes * attrib)
{
    uint16_t duration;
    /* TXsym is represented as a float32_t */
#ifdef __KERNEL__
    duration = f32_to_ui32(
        f32_mul(attrib->Tpsym, ui32_to_f32(attrib->nsync + attrib->nsfd)),
        softfloat_round_max, false);
#else
    duration = DPL_FLOAT32_CEIL(
        DPL_FLOAT32_MUL(attrib->Tpsym,
                        DPL_FLOAT32_INIT(attrib->nsync + attrib->nsfd))
        );
#endif
    return duration;
}

/**
 * API to calculate the data duration (airtime), really everyting after the RMarker.
 * @param attrib    Pointer to _phy_attributes_t * struct. The phy attritubes are part of the IEEE802.15.4-2011 standard.
 * Note the morphology of the frame depends on the mode of operation, see the dw3000_hal.c for the default behaviour.
 * @param nlen      The length of the frame to be transmitted/received excluding crc. Set to 0 to estimate frame duration
 *                  when using Sts Mode 3 (no payload)
 * @return uint16_t duration in usec
 */
uint16_t
dw3000_phy_data_duration(struct uwb_phy_attributes * attrib, uint16_t nlen)
{
    /* TXsym is represented as a dpl_float32_t */
    uint16_t duration;
    uint16_t parity_data_bits = 48;
    uint32_t total_payload_bits;
    /* PHR symbols can be sent at base (standard) or data rate */
    dpl_float32_t phr_sym = (attrib->phr_rate) ? attrib->Tdsym : attrib->Tbsym;
    /* We need to add 48 parity bits for every 330 bits in the data payload (including crc)
     * I.e. for < 330 bits we need 48 parity bits, for > 330 bits we need 48+48 parity bits,
     * for > 660 bits we need 3*48 parity bits, etc. */
    parity_data_bits += ((8*(nlen+2))/330) * 48;
    total_payload_bits = 8*(nlen+2) + parity_data_bits;

#ifdef __KERNEL__
    {
        dpl_float32_t tmp;
        tmp = f32_mul(phr_sym, ui32_to_f32((uint32_t)attrib->nphr));
        tmp = f32_add(tmp, f32_mul(attrib->Tbsym, ui32_to_f32(attrib->nstssync)));
        tmp = f32_add(tmp, f32_mul(attrib->Tdsym, ui32_to_f32(total_payload_bits)));
        duration = f32_to_i32(f32_roundToInt(tmp, softfloat_round_max, false), softfloat_round_max, false);
    }
#else
    duration = (int)ceilf(phr_sym * attrib->nphr +
                          attrib->Tbsym * attrib->nstssync +
                          attrib->Tdsym * total_payload_bits);
#endif
    return duration;
}

/**
 * API to calculate the frame duration (airtime).
 * @param attrib    Pointer to _phy_attributes_t * struct. The phy attritubes are part of the IEEE802.15.4-2011 standard.
 * Note the morphology of the frame depends on the mode of operation, see the dw3000_hal.c for the default behaviour.
 * @param nlen      The length of the frame to be transmitted/received excluding crc. Set to 0 to estimate frame duration
 *                  when using Sts Mode 3 (no payload)
 * @return uint16_t duration in usec
 */
inline uint16_t
dw3000_phy_frame_duration(struct uwb_phy_attributes * attrib, uint16_t nlen)
{
    return dw3000_phy_SHR_duration(attrib) + dw3000_phy_data_duration(attrib, nlen);
}

/**
 * Translate coarse and fine power levels to a registry value used in struct uwb_dev_txrf_config.
 *
 * @param inst      Pointer to struct uwb_dev
 * @param reg       Pointer to where to store the registry value
 * @param coarse    Coarse power control value in dBm (DA)
 * @param fine      Fine power value in dBm (Mixer)
 * @return true on success, false otherwise
 */
bool
dw3000_phy_txrf_power_value(struct _dw3000_dev_instance_t * inst, uint8_t *reg, dpl_float32_t coarse, dpl_float32_t fine)
{
    int c = DPL_FLOAT32_INT(coarse);
    int f = DPL_FLOAT32_INT(fine);
    if (!reg) {
        return false;
    }
    if (f < 0 || f > 63) {
        return false;
    }

    switch (c) {
    case(9):  *reg = dw3000_power_value(DW3000_txrf_config_VMC11_9db, f);break;
    case(6):  *reg = dw3000_power_value(DW3000_txrf_config_VMC10_6db, f);break;
    case(3):  *reg = dw3000_power_value(DW3000_txrf_config_VMC01_3db, f);break;
    case(0):  *reg = dw3000_power_value(DW3000_txrf_config_VMC00_0db, f);break;
    default:
        return false;
    }
    return true;
}

/**
 * Enable and/or reset device event counters
 *
 * @param inst    Pointer to struct dw3000_dev_instance.
 * @param enable  Enables the device internal event counters
 * @param reset   If true, reset counters
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw3000_phy_event_cnt_ctrl(struct _dw3000_dev_instance_t *inst, bool enable, bool reset)
{
    if (reset) {
        /* A reset command must also write enable = 0 or the clearing will not happen */
        dw3000_write_reg(inst, EVENT_CTRL_ID, 0,
                         EVENT_CTRL_EVENT_COUNT_CLR_BIT_MASK, sizeof(uint32_t));
    }

    /* Intentionally writing 32 bits here and above as the register requires at least 16 bits
     * written to take effect. */
    dw3000_write_reg(inst, EVENT_CTRL_ID, 0,
                     (enable) ? EVENT_CTRL_EVENT_COUNT_EN_BIT_MASK : 0,
                     sizeof(uint32_t));
    return inst->uwb_dev.status;
}

/**
 * Read device event counters
 *
 * @param inst    Pointer to struct dw3000_dev_instance.
 * @param res     Pointer to struct uwb_dev_evcnt
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw3000_phy_event_cnt_read(struct _dw3000_dev_instance_t *inst, struct uwb_dev_evcnt *res)
{
    if (!res) {
        return inst->uwb_dev.status;
    }
    /* Read first 7 counter registers in one go, then the last separately */
    dw3000_read(inst, EVENT_COUNT0_ID, 0, (uint8_t*)res, offsetof(struct uwb_dev_evcnt, event_count7));
    res->event_count7 = dw3000_read_reg(inst, EVENT_COUNT7_ID, 0, sizeof(uint32_t));

    /* Apply masks */
    res->event_count0 &= (EVENT_COUNT0_COUNT_RXFSL_BIT_MASK  | EVENT_COUNT0_COUNT_RXPHE_BIT_MASK);
    res->event_count1 &= (EVENT_COUNT1_COUNT_RXFCE_BIT_MASK  | EVENT_COUNT1_COUNT_RXFCG_BIT_MASK);
    res->event_count2 &= (EVENT_COUNT2_COUNT_RXOVRR_BIT_MASK | EVENT_COUNT2_COUNT_ARFE_BIT_MASK);
    res->event_count3 &= (EVENT_COUNT3_COUNT_RXPTO_BIT_MASK  | EVENT_COUNT3_COUNT_RXSTO_BIT_MASK);
    res->event_count4 &= (EVENT_COUNT4_COUNT_TXFRS_BIT_MASK  | EVENT_COUNT4_COUNT_FWTO_BIT_MASK);
    res->event_count5 &= (EVENT_COUNT5_COUNT_SPICRC_BIT_MASK | EVENT_COUNT5_COUNT_HPWARN_BIT_MASK);
    res->event_count6 &= (EVENT_COUNT6_COUNT_RXPREJ_BIT_MASK);
    res->event_count7 &= (EVENT_COUNT7_COUNT_VWARN_BIT_MASK  | EVENT_COUNT7_COUNT_CPERR_BIT_MASK);

    return inst->uwb_dev.status;
}
