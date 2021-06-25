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
 * @file dw3000_hal.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Hardware Abstraction Layer
 *
 * @details This is the hal base class which utilises functions to perform the necessary actions at hal.
 *
 */

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#include <syscfg/syscfg.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw3000-c0/dw3000_hal.h>
#include <dpl/dpl_cputime.h>

#include <mcu/mcu.h>

#if MYNEWT_VAL(DW3000_DEVICE_0)

static dw3000_dev_instance_t hal_dw3000_instances[]= {
#if  MYNEWT_VAL(DW3000_DEVICE_0)
    [0] = {
            .uwb_dev = {
                .idx = 0,
                .task_prio = 0x10,
                .status = {0},
                .rx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_0_RX_ANT_DLY),
                .tx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_0_TX_ANT_DLY),
                .attrib = {                 //!< These values are now set in dw3000_dev_init
                    .nsfd = 8,              //!< Number of symbols in start of frame delimiter
                    .nsync = 128,           //!< Number of symbols in preamble sequence
                    .nstssync = 0,          //!< Number of symbols in sts preamble sequence
                    .nphr = 21,             //!< Number of symbols in phy header
                    .phr_rate = 0,          //!< Rate of phr symbols (0=base, 1=data)
                },
                .config = {
                    .channel = 5,                   //!< channel number {5, 9}
                    .dataRate = DWT_BR_6M8,             //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
                    .rx = {
                        .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size DWT_PAC8..DWT_PAC64 (Relates to RX preamble length)
                        .preambleCodeIndex = 9,         //!< RX preamble code
                        .sfdType = 1,                   //!< Boolean should we use non-standard SFD for better performance
                        .phrMode = DWT_PHRMODE_EXT,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                        .phrRate = DWT_PHRRATE_STD,     //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
                        .sfdTimeout = (128 + 1 + 8 - 8),//!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only.
                        .stsMode = DWT_STS_MODE_OFF,     //!< Sts mode (no sts, sts before PHR or sts after data)
                        .stsLength = (64>>3)-1,         //!< Sts length (the allowed values are from 7 to 255, 7 corresponds to 64 length and 254 is 2040 which is a max value)
                        .pdoaMode = DWT_PDOA_M0,        //!< PDOA mode
                        .timeToRxStable = 12,           //!< Time until the Receiver i stable, (in us)
                        .frameFilter = 0,               //!< No frame filtering by default
                        .xtalTrim = 0x2e,               //!< Centre trim value
                    },
                    .tx ={
                        .preambleCodeIndex = 9,         //!< TX preamble code
                        .preambleLength = DWT_PLEN_128  //!< DWT_PLEN_64..DWT_PLEN_4096
                    },
                    .txrf={
                        .PGdly = TC_PGDELAY_CH5,
                        .power = 0xfdfdfdfd,  // To match sstwr example
                        /* .BOOSTNORM = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP500 = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP250 = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP125 = dw3000_power_value(DW3000_txrf_config_9db, 2.5)    */
                    },
                    .trxoff_enable = 1,
                    .rxdiag_enable = 1,
                    .dblbuffon_enabled = 0,
#if MYNEWT_VAL(DW3000_BIAS_CORRECTION_ENABLED)
                    .bias_correction_enable = 1,
#endif
                    .LDE_enable = 1,
                    .LDO_enable = 0,
                    .sleep_enable = 1,
                    .wakeup_rx_enable = 1,     //!< Wakeup to Rx state
                    .rxauto_enable = 1,        //!< On error re-enable
                    .cir_enable = 0,            //!< Default behavior for CIR interface
                    .cir_pdoa_slave = 0,       //!< First instance should not act as pdoa slave
                    .blocking_spi_transfers = 0, //!< Nonblocking spi transfers allowed by default
                },
            },
            .rst_pin  = 0,
            .ss_pin = 0,
            .irq_pin  = 0,
#if MYNEWT_VAL(BUS_DRIVER_PRESENT) == 0
            .spi_settings = {
                .data_order = HAL_SPI_MSB_FIRST,
                .data_mode = HAL_SPI_MODE0,
                .baudrate = 0,
                .word_size = HAL_SPI_WORD_SIZE_8BIT
            },
            .spi_sem = 0,
#endif
    },
    #if  MYNEWT_VAL(DW3000_DEVICE_1)
    [1] = {
            .uwb_dev = {
                .idx = 1,
                .task_prio = 0x11,
                .status = {0},
                .rx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_1_RX_ANT_DLY),
                .tx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_1_TX_ANT_DLY),
                .attrib = {                 //!< These values are now set in dw3000_dev_init
                    .nsfd = 8,              //!< Number of symbols in start of frame delimiter
                    .nsync = 128,           //!< Number of symbols in preamble sequence
                    .nstssync = 0,          //!< Number of symbols in sts preamble sequence
                    .nphr = 21,             //!< Number of symbols in phy header
                    .phr_rate = 0,          //!< Rate of phr symbols (0=base, 1=data)
                },
                .config = {
                    .channel = 5,                   //!< channel number {5, 9}
                    .dataRate = DWT_BR_6M8,             //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
                    .rx = {
                        .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size DWT_PAC8..DWT_PAC64 (Relates to RX preamble length)
                        .preambleCodeIndex = 9,         //!< RX preamble code
                        .sfdType = 1,                   //!< Boolean should we use non-standard SFD for better performance
                        .phrMode = DWT_PHRMODE_EXT,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                        .phrRate = DWT_PHRRATE_STD,     //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
                        .sfdTimeout = (128 + 1 + 8 - 8),//!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only.
                        .stsMode = DWT_STS_MODE_OFF,     //!< Sts mode (no sts, sts before PHR or sts after data)
                        .stsLength = (64>>3)-1,         //!< Sts length (the allowed values are from 7 to 255, 7 corresponds to 64 length and 254 is 2040 which is a max value)
                        .pdoaMode = DWT_PDOA_M0,        //!< PDOA mode
                        .timeToRxStable = 12,           //!< Time until the Receiver i stable, (in us)
                        .frameFilter = 0,               //!< No frame filtering by default
                        .xtalTrim = 0x2e,               //!< Centre trim value
                    },
                    .tx ={
                        .preambleCodeIndex = 9,         //!< TX preamble code
                        .preambleLength = DWT_PLEN_128  //!< DWT_PLEN_64..DWT_PLEN_4096
                    },
                    .txrf={
                        .PGdly = TC_PGDELAY_CH5,
                        .power = 0xfdfdfdfd,  // To match sstwr example
                        /* .BOOSTNORM = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP500 = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP250 = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP125 = dw3000_power_value(DW3000_txrf_config_9db, 2.5)    */
                    },
                    .trxoff_enable = 1,
                    .rxdiag_enable = 1,
                    .dblbuffon_enabled = 0,
#if MYNEWT_VAL(DW3000_BIAS_CORRECTION_ENABLED)
                    .bias_correction_enable = 1,
#endif
                    .LDE_enable = 1,
                    .LDO_enable = 0,
                    .sleep_enable = 1,
                    .wakeup_rx_enable = 1,     //!< Wakeup to Rx state
                    .rxauto_enable = 1,        //!< On error re-enable
                    .cir_enable = 0,            //!< Default behavior for CIR interface
                    .cir_pdoa_slave = 0,       //!< First instance should not act as pdoa slave
                    .blocking_spi_transfers = 0, //!< Nonblocking spi transfers allowed by default
                },
            },
            .rst_pin  = 0,
            .ss_pin = 0,
            .irq_pin  = 0,
#if MYNEWT_VAL(BUS_DRIVER_PRESENT) == 0
            .spi_settings = {
                .data_order = HAL_SPI_MSB_FIRST,
                .data_mode = HAL_SPI_MODE0,
                .baudrate = 0,
                .word_size = HAL_SPI_WORD_SIZE_8BIT
            },
            .spi_sem = 0,
#endif
    },
    #if  MYNEWT_VAL(DW3000_DEVICE_2)
    [2] = {
            .uwb_dev = {
                .idx = 2,
                .task_prio = 0x12,
                .status = {0},
                .rx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_2_RX_ANT_DLY),
                .tx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_2_TX_ANT_DLY),
                .attrib = {                 //!< These values are now set in dw3000_dev_init
                    .nsfd = 8,              //!< Number of symbols in start of frame delimiter
                    .nsync = 128,           //!< Number of symbols in preamble sequence
                    .nstssync = 0,          //!< Number of symbols in sts preamble sequence
                    .nphr = 21,             //!< Number of symbols in phy header
                    .phr_rate = 0,          //!< Rate of phr symbols (0=base, 1=data)
                },
                .config = {
                    .channel = 5,                   //!< channel number {5, 9}
                    .dataRate = DWT_BR_6M8,             //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
                    .rx = {
                        .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size DWT_PAC8..DWT_PAC64 (Relates to RX preamble length)
                        .preambleCodeIndex = 9,         //!< RX preamble code
                        .sfdType = 1,                   //!< Boolean should we use non-standard SFD for better performance
                        .phrMode = DWT_PHRMODE_EXT,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                        .phrRate = DWT_PHRRATE_STD,     //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
                        .sfdTimeout = (128 + 1 + 8 - 8),//!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only.
                        .stsMode = DWT_STS_MODE_OFF,     //!< Sts mode (no sts, sts before PHR or sts after data)
                        .stsLength = (64>>3)-1,         //!< Sts length (the allowed values are from 7 to 255, 7 corresponds to 64 length and 254 is 2040 which is a max value)
                        .pdoaMode = DWT_PDOA_M0,        //!< PDOA mode
                        .timeToRxStable = 12,           //!< Time until the Receiver i stable, (in us)
                        .frameFilter = 0,               //!< No frame filtering by default
                        .xtalTrim = 0x2e,               //!< Centre trim value
                    },
                    .tx ={
                        .preambleCodeIndex = 9,         //!< TX preamble code
                        .preambleLength = DWT_PLEN_128  //!< DWT_PLEN_64..DWT_PLEN_4096
                    },
                    .txrf={
                        .PGdly = TC_PGDELAY_CH5,
                        .power = 0xfdfdfdfd,  // To match sstwr example
                        /* .BOOSTNORM = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP500 = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP250 = dw3000_power_value(DW3000_txrf_config_9db, 2.5), */
                        /* .BOOSTP125 = dw3000_power_value(DW3000_txrf_config_9db, 2.5)    */
                    },
                    .trxoff_enable = 1,
                    .rxdiag_enable = 1,
                    .dblbuffon_enabled = 0,
#if MYNEWT_VAL(DW3000_BIAS_CORRECTION_ENABLED)
                    .bias_correction_enable = 1,
#endif
                    .LDE_enable = 1,
                    .LDO_enable = 0,
                    .sleep_enable = 1,
                    .wakeup_rx_enable = 1,     //!< Wakeup to Rx state
                    .rxauto_enable = 1,        //!< On error re-enable
                    .cir_enable = 0,            //!< Default behavior for CIR interface
                    .cir_pdoa_slave = 0,       //!< First instance should not act as pdoa slave
                    .blocking_spi_transfers = 0, //!< Nonblocking spi transfers allowed by default
                },
            },
            .rst_pin  = 0,
            .ss_pin = 0,
            .irq_pin  = 0,
#if MYNEWT_VAL(BUS_DRIVER_PRESENT) == 0
            .spi_settings = {
                .data_order = HAL_SPI_MSB_FIRST,
                .data_mode = HAL_SPI_MODE0,
                .baudrate = 0,
                .word_size = HAL_SPI_WORD_SIZE_8BIT
            },
            .spi_sem = 0,
#endif
    }
    #endif
    #endif
    #endif
};
#endif

#if MYNEWT_VAL(DW3000_DEVICE_0) || MYNEWT_VAL(DW3000_DEVICE_1) || MYNEWT_VAL(DW3000_DEVICE_2)

/**
 * API to choose DW3000 instances based on parameters.
 *
 * @param idx  Indicates number of instances for the chosen bsp.
 * @return dw3000_dev_instance_t
 */

struct _dw3000_dev_instance_t *
hal_dw3000_inst(uint8_t idx)
{
    if (idx < ARRAY_SIZE(hal_dw3000_instances)) {
        return &hal_dw3000_instances[idx];
    }
    return 0;
}

/**
 * API to reset all the gpio pins.
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return void
 */
void
hal_dw3000_reset(struct _dw3000_dev_instance_t * inst)
{
    assert(inst);
    hal_gpio_init_out(inst->ss_pin, 1);
    hal_gpio_init_out(inst->rst_pin, 0);

    hal_gpio_write(inst->rst_pin, 0);
    dpl_cputime_delay_usecs(1);
    hal_gpio_write(inst->rst_pin, 1);
    hal_gpio_init_in(inst->rst_pin, HAL_GPIO_PULL_NONE);

    dpl_cputime_delay_usecs(1000);
}

/**
 * API to perform a blocking read over SPI
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Represents value based on the cmd attributes.
 * @param buffer    Results are stored into the buffer.
 * @param length    Represents buffer length.
 * @return int      DPL_OK if read is ok, error otherwise
 */
int
hal_dw3000_read(struct _dw3000_dev_instance_t * inst,
                const uint8_t * cmd, uint8_t cmd_size,
                uint8_t * buffer, uint16_t length)
{
    int rc = DPL_OK;

    DW3000_SPI_BT_ADD(inst, cmd, cmd_size, buffer, length, 0, 0);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    rc = bus_node_simple_write_read_transact(
        (struct os_dev *)&inst->uwb_dev.spi_node, cmd,
        cmd_size, buffer, length);
#else
    assert(inst->spi_sem);
    rc = dpl_sem_pend(inst->spi_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto early_exit;
    }

    hal_gpio_write(inst->ss_pin, 0);

#if !defined(MYNEWT)
    /* Linux mode really, for when we can't split the command and data */
    assert(cmd_size + length < inst->uwb_dev.txbuf_size);
    assert(cmd_size + length < MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT));

    memcpy(inst->uwb_dev.txbuf, cmd, cmd_size);
    memset(inst->uwb_dev.txbuf + cmd_size, 0, length);

    /* If there's an error in the below function rc
     * will be passed. */
    rc = hal_spi_txrx(inst->spi_num, inst->uwb_dev.txbuf,
                      inst->uwb_dev.txbuf, cmd_size+length);

    memcpy(buffer, inst->uwb_dev.txbuf + cmd_size, length);
#else
    rc = hal_spi_txrx(inst->spi_num, (void*)cmd, 0, cmd_size);
    assert(rc == DPL_OK);
    int step = (inst->uwb_dev.txbuf_size > MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT)) ?
        MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT) : inst->uwb_dev.txbuf_size;
    int bytes_left = length;
    for (int offset = 0;offset<length && rc == DPL_OK;offset+=step) {
        int bytes_to_read = (bytes_left > step) ? step : bytes_left;
        bytes_left-=bytes_to_read;
        rc = hal_spi_txrx(inst->spi_num, inst->uwb_dev.txbuf, buffer+offset, bytes_to_read);
    }
#endif
    hal_gpio_write(inst->ss_pin, 1);
    rc = dpl_sem_release(inst->spi_sem);
    assert(rc == DPL_OK);
early_exit:

#endif
    DW3000_SPI_BT_ADD_END(inst);
    return rc;
}


#if MYNEWT_VAL(BUS_DRIVER_PRESENT) == 0
/**
 * Interrupt context callback for nonblocking SPI-functions
 *
 * @param ev    pointer to os_event
 * @return void
 */
void
hal_dw3000_spi_txrx_cb(void *arg, int len)
{
    dpl_error_t err;
    struct _dw3000_dev_instance_t * inst = arg;
    assert(inst!=0);

    /* Check for longer nonblocking read/write op */
    if (dpl_sem_get_count(&inst->spi_nb_sem) == 0) {
        err = dpl_sem_release(&inst->spi_nb_sem);
        assert(err == DPL_OK);
    } else {
        hal_gpio_write(inst->ss_pin, 1);
        DW3000_SPI_BT_ADD_END(inst);
        /* Extra delay added due to bug in C0 spi interface */
        dpl_cputime_delay_usecs(5);
        err = dpl_sem_release(inst->spi_sem);
        assert(err == DPL_OK);
    }
}
#endif

/**
 * API to perform a non-blocking read from SPI
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Represents value based on the cmd attributes.
 * @param buffer    Results are stored into the buffer.
 * @param length    Represents buffer length.
 * @return int      DPL_OK if read is ok, error otherwise
 */
int
hal_dw3000_read_noblock(struct _dw3000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    int rc = DPL_OK;

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    return hal_dw3000_read(inst, cmd, cmd_size, buffer, length);
#else
    DW3000_SPI_BT_ADD(inst, cmd, cmd_size, buffer, length, 0, 1);
    assert(inst->spi_sem);

    rc = dpl_sem_pend(inst->spi_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto early_exit;
    }

    /* Reset the txrx_cb to make sure it has the correct instance
     * as argument */
    rc = hal_spi_disable(inst->spi_num);
    rc |= hal_spi_set_txrx_cb(inst->spi_num, hal_dw3000_spi_txrx_cb, (void*)inst);
    rc |= hal_spi_enable(inst->spi_num);
    if (rc != DPL_OK) {
        goto err_return;
    }

    hal_gpio_write(inst->ss_pin, 0);

    /* Faster read for shorter exchanges */
    if (cmd_size + length < inst->uwb_dev.txbuf_size &&
        cmd_size + length < MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT)) {
        memset(inst->uwb_dev.txbuf, 0, cmd_size + length);
        memcpy(inst->uwb_dev.txbuf, cmd, cmd_size);

        rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
        if (rc != DPL_OK) {
            inst->uwb_dev.status.sem_error = 1;
            goto err_return;
        }

        rc = hal_spi_txrx_noblock(inst->spi_num, (void*)inst->uwb_dev.txbuf,
                                  (void*)inst->uwb_dev.txbuf, cmd_size+length);

        rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
        if (rc != DPL_OK) {
            inst->uwb_dev.status.sem_error = 1;
            goto err_return;
        }
        rc = dpl_sem_release(&inst->spi_nb_sem);
        assert(rc == DPL_OK);
        hal_gpio_write(inst->ss_pin, 1);

        memcpy(buffer, inst->uwb_dev.txbuf+cmd_size, length);
        DW3000_SPI_BT_ADD_END(inst);
        rc = dpl_sem_release(inst->spi_sem);
        assert(rc == DPL_OK);
        return rc;
    }

#if MYNEWT_VAL(DW3000_HAL_SPI_BUFFER_SIZE) < 1024 || MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT) < 1028
    rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto err_return;
    }

    /* Send command portion */
    rc = hal_spi_txrx_noblock(inst->spi_num, (void*)cmd, inst->uwb_dev.txbuf, cmd_size);
    if (rc != DPL_OK) {
        goto err_return;
    }
    memset(inst->uwb_dev.txbuf, 0, (length < inst->uwb_dev.txbuf_size)? length : inst->uwb_dev.txbuf_size);

    /* Wait for command to send */
    rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        goto err_return;
    }
    rc = dpl_sem_release(&inst->spi_nb_sem);
    assert(rc == DPL_OK);

    /* Nonblocking reads can only do a maximum of
     * MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT) bytes at a time. And
     * not read more than what can fit in the inst->uwb_dev.txbuf at a time.
     */
    int step = (inst->uwb_dev.txbuf_size > MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT))?
        MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT) : inst->uwb_dev.txbuf_size;
    int bytes_left = length;
    int offset = 0;
    while (offset<length) {
        int bytes_to_read = (bytes_left > step) ? step : bytes_left;
        bytes_left-=bytes_to_read;

        /* Only use the spi_nb_sem if needed */
        if (bytes_left) {
            rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
            if (rc != DPL_OK) {
                inst->uwb_dev.status.sem_error = 1;
                goto err_return;
            }
        }
        rc = hal_spi_txrx_noblock(inst->spi_num, (void*)inst->uwb_dev.txbuf,
                                  (void*)buffer+offset, bytes_to_read);
        assert(rc==DPL_OK);

        /* Only wait for this round if there is more data to read */
        if (bytes_left) {
            rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
            if (rc != DPL_OK) {
                inst->uwb_dev.status.sem_error = 1;
                goto err_return;
            }

            rc = dpl_sem_release(&inst->spi_nb_sem);
            assert(rc == DPL_OK);
        }
        offset+=bytes_to_read;
    }

    /* Reaquire semaphore after rx complete */
    rc = dpl_sem_pend(inst->spi_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        goto err_return;
    }

#else
    assert(0);
#endif
err_return:
    rc = dpl_sem_release(inst->spi_sem);
    assert(rc == DPL_OK);

early_exit:
#endif
    DW3000_SPI_BT_ADD_END(inst);
    return rc;
}


/**
 * API to perform a blocking write over SPI
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Length of command array
 * @param buffer    Data buffer to be sent to device
 * @param length    Represents buffer length.
 * @return int      DPL_OK if read is ok, error otherwise
 */
int
hal_dw3000_write(struct _dw3000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    int rc = DPL_OK;
    DW3000_SPI_BT_ADD(inst, cmd, cmd_size, buffer, length, 1, 0);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    if (length) {
        assert(cmd_size + length < inst->uwb_dev.txbuf_size);
        memcpy(inst->uwb_dev.txbuf, cmd, cmd_size);
        memcpy(inst->uwb_dev.txbuf + cmd_size, buffer, length);
        rc = bus_node_simple_write((struct os_dev *)&inst->uwb_dev.spi_node,
                                   inst->uwb_dev.txbuf, cmd_size+length);
    } else {
        rc = bus_node_simple_write((struct os_dev *)&inst->uwb_dev.spi_node,
                                   cmd, cmd_size);
    }
#else
    assert(inst->spi_sem);
    rc = dpl_sem_pend(inst->spi_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto early_exit;
    }

    hal_gpio_write(inst->ss_pin, 0);

#if !defined(MYNEWT)
    /* Linux mode really, for when we can't split the command and data */
    assert(cmd_size + length < inst->uwb_dev.txbuf_size);
    assert(cmd_size + length < MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT));

    memcpy(inst->uwb_dev.txbuf, cmd, cmd_size);
    memcpy(inst->uwb_dev.txbuf+cmd_size, buffer, length);

    rc = hal_spi_txrx(inst->spi_num, inst->uwb_dev.txbuf,
                      0, cmd_size+length);
    if (rc != DPL_OK) {
        goto err_return;
    }
#else
    rc = hal_spi_txrx(inst->spi_num, (void*)cmd, 0, cmd_size);
    if (rc != DPL_OK) {
        goto err_return;
    }
    if (length) {
        hal_spi_txrx(inst->spi_num, (void*)buffer, 0, length);
    }
#endif

    hal_gpio_write(inst->ss_pin, 1);

    /* Extra delay added due to bug in C0 spi interface */
    dpl_cputime_delay_usecs(5);

err_return:
    rc = dpl_sem_release(inst->spi_sem);
    assert(rc == DPL_OK);
early_exit:
#endif
    DW3000_SPI_BT_ADD_END(inst);
    return rc;
}


/**
 * API to perform a nonblocking write over SPI
 *
 * @param inst      Pointer to dw3000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Length of command array
 * @param buffer    Data buffer to be sent to device
 * @param length    Represents buffer length.
 * @return int      DPL_OK if read is ok, error otherwise
 */
int
hal_dw3000_write_noblock(struct _dw3000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    int rc = DPL_OK;

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    return hal_dw3000_write(inst, cmd, cmd_size, buffer, length);
#else
    DW3000_SPI_BT_ADD(inst, cmd, cmd_size, buffer, length, 1, 1);
    assert(inst->spi_sem);
    rc = dpl_sem_pend(inst->spi_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto early_exit;
    }

    /* Reset the txrx_cb to make sure it has the correct instance
     * as argument */
    rc = hal_spi_disable(inst->spi_num);
    rc |= hal_spi_set_txrx_cb(inst->spi_num, hal_dw3000_spi_txrx_cb, (void*)inst);
    rc |= hal_spi_enable(inst->spi_num);
    if (rc != DPL_OK) {
        goto err_return;
    }

    hal_gpio_write(inst->ss_pin, 0);

    /* If command and data fit in inst->uwb_dev.txbuf, send immediately */
    if (cmd_size + length < inst->uwb_dev.txbuf_size &&
        cmd_size + length < MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT)) {
        memcpy(inst->uwb_dev.txbuf, cmd, cmd_size);
        memcpy(inst->uwb_dev.txbuf+cmd_size, buffer, length);

        rc = hal_spi_txrx_noblock(inst->spi_num, (void*)inst->uwb_dev.txbuf,
                                  inst->uwb_dev.txbuf, cmd_size + length);
        return rc;
    }

#if MYNEWT_VAL(DW3000_HAL_SPI_BUFFER_SIZE) < 1024 || MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT) < 1028
    rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto err_return;
    }

    rc = hal_spi_txrx_noblock(inst->spi_num, (void*)cmd, inst->uwb_dev.txbuf, cmd_size);
    if (rc != DPL_OK) {
        goto err_return;
    }

    /* Wait for cmd to finish sending */
    rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto err_return;
    }
    rc = dpl_sem_release(&inst->spi_nb_sem);
    assert(rc == DPL_OK);

    /* Nonblocking writes can only do a maximum of MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT)
     * bytes at a time */
    int step = (inst->uwb_dev.txbuf_size > MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT)) ?
        MYNEWT_VAL(DW3000_HAL_SPI_MAX_CNT) : inst->uwb_dev.txbuf_size;
    int bytes_left = length;
    int offset = 0;
    while (offset<length) {
        int bytes_to_write = (bytes_left > step) ? step : bytes_left;
        bytes_left-=bytes_to_write;

        /* Only use the spi_nb_sem if needed */
        if (bytes_left) {
            rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
            if (rc != DPL_OK) {
                inst->uwb_dev.status.sem_error = 1;
                goto err_return;
            }
        }

        rc = hal_spi_txrx_noblock(inst->spi_num, (void*)buffer+offset,
                                  inst->uwb_dev.txbuf, bytes_to_write);
        assert(rc==DPL_OK);

        /* Only wait for this round if there is more data to read */
        if (bytes_left) {
            /* Wait for this round of writing to complete */
            rc = dpl_sem_pend(&inst->spi_nb_sem, DPL_TIMEOUT_NEVER);
            if (rc != DPL_OK) {
                inst->uwb_dev.status.sem_error = 1;
                goto err_return;
            }

            rc = dpl_sem_release(&inst->spi_nb_sem);
            assert(rc == DPL_OK);
        }
        offset+=bytes_to_write;
    }
#else
    assert(0);
#endif
early_exit:
    return rc;

err_return:
    rc = dpl_sem_release(inst->spi_sem);
    assert(rc == DPL_OK);
#endif

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    DW3000_SPI_BT_ADD_END(inst);
#endif
    return rc;
}

/**
 * API to wait for a DMA transfer
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @param timeout  Time in ms to wait, use DPL_TIMEOUT_NEVER (UINT32_MAX) to wait indefinitely
 * @return int     0 if read is ok, error otherwise
 */
int
hal_dw3000_rw_noblock_wait(struct _dw3000_dev_instance_t * inst, uint32_t timeout_ms)
{
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    return 0;
#else
    int err;
    dpl_time_t ticks;
    if (timeout_ms != DPL_TIMEOUT_NEVER) {
        dpl_time_ms_to_ticks(timeout_ms, &ticks);
    } else {
        ticks = DPL_TIMEOUT_NEVER;
    }
    err = dpl_sem_pend(inst->spi_sem, ticks);
    if (dpl_sem_get_count(inst->spi_sem) == 0) {
        dpl_sem_release(inst->spi_sem);
    }
    return err;
#endif
}


/**
 * API to wake dw3000 from sleep mode
 *
 * @param inst  Pointer to dw3000_dev_instance_t.
 * @return int  DPL_OK if read is ok, error otherwise
 */
int
hal_dw3000_wakeup(struct _dw3000_dev_instance_t * inst)
{
    int rc = DPL_OK;
    os_sr_t sr;
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct os_dev *node = (struct os_dev *)&inst->uwb_dev.spi_node;
    const os_time_t tmo = BUS_NODE_LOCK_DEFAULT_TIMEOUT;
    rc = bus_node_lock(node, tmo);
    if (rc != OS_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto early_exit;
    }
#else
    assert(inst->spi_sem);
    rc = dpl_sem_pend(inst->spi_sem, DPL_TIMEOUT_NEVER);
    if (rc != DPL_OK) {
        inst->uwb_dev.status.sem_error = 1;
        goto early_exit;
    }
#endif
    OS_ENTER_CRITICAL(sr);

    hal_gpio_write(inst->ss_pin, 0);

    // Need to hold chip select for a minimum of 500us
    dpl_cputime_delay_usecs(600);

    hal_gpio_write(inst->ss_pin, 1);

    // Waiting for XTAL to start and stabilise - 5ms safe
    // (check PLL bit in IRQ?)
    dpl_cputime_delay_usecs(5000);

    OS_EXIT_CRITICAL(sr);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    bus_node_unlock(node);
#else
    rc = dpl_sem_release(inst->spi_sem);
    assert(rc == DPL_OK);
#endif
early_exit:
    return rc;
}

/**
 * API to read the current level of the rst pin.
 * When sleeping dw3000 will let this pin go low.
 *
 * @param inst  Pointer to dw3000_dev_instance_t
 * @return status of rst_pin
 */
int
hal_dw3000_get_rst(struct _dw3000_dev_instance_t * inst)
{
    return hal_gpio_read(inst->rst_pin);
}

#endif
