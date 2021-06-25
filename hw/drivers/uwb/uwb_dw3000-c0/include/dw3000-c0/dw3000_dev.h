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
 * @file dw3000_dev.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Device file
 *
 * @details This is the dev base class which utilises the functions to perform initialization and necessary configurations on device.
 *
 */


#ifndef _DW3000_DEV_H_
#define _DW3000_DEV_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <uwb/uwb.h>
#include <os/os_dev.h>
#include <dpl/dpl.h>
#include <hal/hal_spi.h>
#include <dw3000-c0/dw3000_regs.h>
#include <dw3000-c0/dw3000_stats.h>

#define DWT_DEVICE_ID   (0xDECA0300)            //!< Decawave Device ID

#define DWT_A0_DEV_ID       (0xDECA0300)        //!< DW3000 MPW A0 (non PDOA) silicon device ID
#define DWT_A0_PDOA_DEV_ID  (0xDECA0310)        //!< DW3000 MPW A0 (with PDOA) silicon device ID
#define DWT_B0_DEV_ID       (0xDECA0301)        //!< DW3000 MPW B0 (non PDOA) silicon device ID
#define DWT_B0_PDOA_DEV_ID  (0xDECA0311)        //!< DW3000 MPW B0 (with PDOA) silicon device ID
#define DWT_XX_PDOA_DEV_ID_MASK  (0x10)         //!< DW3000 MPW (with PDOA) silicon device ID bit

#define DWT_SUCCESS (0)              //!< DWT Success
#define DWT_ERROR   (-1)             //!< DWT Error
#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< DWT time units calculation

//! DW-IC SPI CRC-8 polynomial
#define POLYNOMIAL 0x07    /* x^8 + x^2 + x^1 + x^0 */
#define TOPBIT (1 << (8 - 1))

//! Macros and Enumerations for SPI & CLock blocks
#define DW3000_SPI_FAC      (0<<6 | 1<<0)
#define DW3000_SPI_FARW     (0<<6 | 0<<0)
#define DW3000_SPI_EAMRW    (1<<6)

typedef enum {
    DW3000_SPI_RD_BIT    = 0x0000U,
    DW3000_SPI_WR_BIT    = 0x8000U,
    DW3000_SPI_AND_OR_8  = 0x8001U,
    DW3000_SPI_AND_OR_16 = 0x8002U,
    DW3000_SPI_AND_OR_32 = 0x8003U,
}spi_modes_e;

// Defines for enable_clocks function
#define FORCE_CLK_SYS           (0)
#define FORCE_CLK_SYS_TX        (1)
#define FORCE_CLK_SYS_RX        (2)
#define FORCE_CLK_FOSC_DIV      (4)
#define FORCE_CLK_AUTO          (5)
#define FORCE_CLK_ALL           (6)
#define FORCE_CLK_FOSC          (7)
#define FORCE_CLK_MEM           (8)
#define NO_TRXBUFF_CLOCKS       (0x10)  //OR this with one of the clock options above to skip enabling of TX and RX buffer clocks

//SYSCLK
#define FORCE_SYSCLK_FOSC_DIV   (1)
#define FORCE_SYSCLK_PLL        (2)
#define FORCE_SYSCLK_FOSC       (3)

//RX and TX CLK
#define FORCE_CLK_PLL           (2)
#define FORCE_CLK_OFF           (3)

//! Device control status bits.
typedef struct _dw3000_dev_control_t{
    uint32_t wait4resp_enabled:1;           //!< Wait for the response
    uint32_t wait4resp_delay_enabled:1;     //!< Wait for the delayed response
    uint32_t delay_start_enabled:1;         //!< Transmit after delayed start
    uint32_t delay_wrt_rs_start_enabled:1;  //!< Delayed transmit with respect to rx timestamp
    uint32_t delay_wrt_ts_start_enabled:1;  //!< Delayed transmit with respect to tx timestamp
    uint32_t delay_wrt_ref_start_enabled:1; //!< Delayed transmit with respect to reference time
    uint32_t autoack_delay_enabled:1;       //!< Enables automatic acknowledgement feature with delay
    uint32_t start_rx_syncbuf_enabled:1;    //!< Enables receive syncbuffer
    uint32_t rx_timeout_enabled:1;          //!< Enables receive timeout
    uint32_t on_error_continue_enabled:1;   //!< Enables on_error_continue
    uint32_t sleep_after_tx:1;              //!< Enables to load LDE microcode on wake up
    uint32_t sleep_after_rx:1;              //!< Enables to load LDO tune value on wake up
    uint32_t cca_tx:1;                      //!<
    uint32_t cir_enable:1;                  //!< Enables reading CIR on this operation
    uint32_t rxauto_disable:1;              //!< Disable auto receive parameter
    uint32_t abs_timeout:1;                 //!< RX absolute timeout active
}dw3000_dev_control_t;

//! DW3000 receiver diagnostics parameters.
struct dw3000_rxdiag_sequence {
    union {
        struct rxds_peak {
            uint32_t    peak_padding:1; //!< Unused bit
            uint32_t    peak_idx:10;    //!< index of peak sample in sequence CIR
            uint32_t    peak:21;        //!< amplitude of peak sample in sequence CIR
        };
        uint32_t peak_union;          //!index [30:21] and amplitude [20:0] of peak sample in sequence CIR
    };
    uint32_t    power;                //!< channel area allows estimation of channel power for the sequence
    uint32_t    f1;                   //!< F1 for sequence
    uint32_t    f2;                   //!< F2 for sequence
    uint32_t    f3;                   //!< F3 for sequence
    uint32_t    noiseMAD;             //!< Noise MAD for sequence
    uint32_t    noisePeak;            //!< Noise Peak for sequence
    uint32_t    noiseMean;            //!< Noise Mean for sequence
    uint16_t    fpIndex;              //!< First path index for sequence
    uint16_t    fpIndex_pad;          //!< Padding
    uint32_t    fpConfidenceAndEarly; //!< First Path confidence and early fp
    uint32_t    carrierRotAndFpAngle; //!< Carrier correction phase and angle
    uint32_t    threshold;            //!< Threshold info for Ipatov sequence
    uint16_t    accumCount;           //!< Number accumulated symbols for Ipatov sequence (IpDiag12)
    uint16_t    accumCount_pad;       //!< Padding
} __attribute__((packed, aligned(1)));

typedef struct _dw3000_dev_rxdiag_t{
    struct uwb_dev_rxdiag rxd;

    uint64_t    ipatovRxTime;         //!< RX timestamp from Ipatov sequence (IP_TOA_LO_ID)
    uint16_t    ipatovPOA;            //!< POA of Ipatov
    uint8_t     ipatovRxStatus;       //!< RX status info for Ipatov sequence

    uint64_t    stsRxTime;            //!< RX timestamp from STS sequence
    uint16_t    stsPOA;               //!< POA of STS 1
    uint8_t     stsRxStatus;          //!< RX status info for STS sequence

    uint64_t    sts2RxTime;           //!< RX timestamp from STS sequence
    uint16_t    sts2POA;              //!< POA of STS 2
    uint8_t     sts2RxStatus;         //!< RX status info for STS sequence

    uint64_t    tdoa;                 //!< TDOA from two STS RX timestamps
    int16_t     pdoa;                 //!< PDOA from two STS POAs signed int [1:-11] in radians

    int16_t     sts_qual_idx;         //!< STS Cipher quality factor, qf >= 0 is good
    int16_t     sts_acc_qual;         //!< STS Cipher quality raw num accumulated symbols

    int16_t     xtalOffset;           //!< estimated xtal offset of remote device
    uint32_t    ciaDiag1;             //!< Diagnostics common to both sequences

    struct _dw3000_rxdiag_sequence_s{
        struct dw3000_rxdiag_sequence ipatov;
        struct dw3000_rxdiag_sequence sts;
        uint32_t cy0_padding[5]; /* Five 32bit regs in between CY0 and CY1 */
        struct dw3000_rxdiag_sequence sts2;
    } __attribute__((packed, aligned(1)));;
} dw3000_dev_rxdiag_t;

_Static_assert(sizeof(struct _dw3000_dev_rxdiag_t) <= MYNEWT_VAL(UWB_DEV_RXDIAG_MAXLEN),
               "UWB_DEV_RXDIAG_MAXLEN is too small to hold struct _dw3000_dev_rxdiag_t");

#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
#define DW3000_SYS_STATUS_BT_PTR(_I) _I->sys_status_bt[_I->sys_status_bt_idx%MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)]
#define DW3000_SYS_STATUS_BT_ADD(_I,_S,_U) _I->sys_status_bt[++_I->sys_status_bt_idx%MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)] = \
        (struct dw3000_sys_status_backtrace) {.utime=_U, .sys_status_lo=_S, .fctrl=0}
#define DW3000_SYS_STATUS_BT_HI(_I,_S) DW3000_SYS_STATUS_BT_PTR(_I).sys_status_hi = _S
#define DW3000_SYS_STATUS_BT_FCTRL(_I,_FCTRL) DW3000_SYS_STATUS_BT_PTR(_I).fctrl = _FCTRL
#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_HI)
#define DW3000_SYS_STATUS_ASSEMBLE(_P) (((uint64_t)_P->sys_status_hi)<<32)|(uint64_t)_P->sys_status_lo
#define DW3000_SYS_STATUS_ASSEMBLE_LEN (8)
#else
#define DW3000_SYS_STATUS_ASSEMBLE(_P) (uint64_t)_P->sys_status_lo
#define DW3000_SYS_STATUS_ASSEMBLE_LEN (4)
#endif

//! DW3000 Interrupt backtrace structure
struct dw3000_sys_status_backtrace {
    uint32_t utime;
    uint32_t sys_status_lo;
    uint32_t sys_status_hi;
    uint16_t fctrl;
    uint16_t interrupt_reentry:1;
    uint32_t utime_end;
};
#endif

#if MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)
/* TODO: Rework this so it can work with bus_drivers without
 * causing lines with missing ends added */
#define DW3000_SPI_BT_PTR(_I) _I->spi_bt[_I->spi_bt_idx%MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)]
#define DW3000_SPI_BT_ADD(_I,_CMD,_CLEN,_DATA,_DLEN,_IS_WR,_NB) if(!_I->spi_bt_lock){\
        struct dw3000_spi_backtrace *p=&_I->spi_bt[++_I->spi_bt_idx%MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)]; \
        *p = (struct dw3000_spi_backtrace){.utime=dpl_cputime_get32(), .cmd_len=_CLEN, .data_len=_DLEN, .is_write=_IS_WR, .non_blocking=_NB}; \
        memcpy(p->cmd,_CMD,_CLEN);_I->spi_bt_datap=_DATA;}
#define DW3000_SPI_BT_ADD_END(_I) if(!_I->spi_bt_lock){struct dw3000_spi_backtrace *p=&(DW3000_SPI_BT_PTR(_I)); \
        p->utime_end=dpl_cputime_get32();                                 \
        memcpy(p->data,_I->spi_bt_datap,(p->data_len>sizeof(p->data))?sizeof(p->data):p->data_len);}

//! DW3000 SPI backtrace structure
struct dw3000_spi_backtrace {
    uint32_t utime;
    uint8_t cmd[4];
    uint8_t cmd_len;
    uint8_t is_write:1;
    uint8_t non_blocking:1;
    uint8_t data[MYNEWT_VAL(DW3000_SPI_BACKTRACE_DATA_LEN)];
    uint16_t data_len;
    uint32_t utime_end;
};
#else
#define DW3000_SPI_BT_PTR(_I) {}
#define DW3000_SPI_BT_ADD(_I,_CMD,_CLEN,_DATA,_DLEN,_IS_WR,_NB) {}
#define DW3000_SPI_BT_ADD_END(_I) {}
#endif

//! Device instance parameters.
typedef struct _dw3000_dev_instance_t{
    struct uwb_dev uwb_dev;        //!< Common generalising struct uwb_dev

    struct dpl_sem spi_nb_sem;     //!< Semaphore for nonblocking rd/wr operations
    uint8_t irq_pin;               //!< Interrupt request pin
    uint8_t ss_pin;                //!< Slave select pin
    uint8_t rst_pin;               //!< Reset pin
#if MYNEWT_VAL(BUS_DRIVER_PRESENT) == 0
    uint8_t spi_num;                       //!< SPI number
    struct dpl_sem * spi_sem;              //!< Pointer to global spi bus semaphore
    struct hal_spi_settings spi_settings;  //!< Structure of SPI settings in hal layer
#endif

    struct dpl_sem tx_sem;         //!< semphore for low level mac/phy functions
    struct dpl_mutex mutex;        //!< os_mutex
    uint32_t part_id;              //!< Identifier of a particular part design
    uint32_t lot_id;               //!< Identification number assigned to a particular quantity
    uint16_t otp_rev;              //!< OTP parameter revision
    uint8_t otp_vbat;              //!< OTP parameter for voltage
    uint8_t otp_temp;              //!< OTP parameter for temperature
    uint8_t xtal_trim;             //!< Crystal trim
    uint32_t sys_cfg_reg;          //!< System config register
    uint32_t tx_fctrl;             //!< Transmit frame control register parameter
    uint32_t sys_status;           //!< SYS_STATUS_ID for current event
    uint16_t sys_status_hi;        //!< SYS_STATUS_HI_ID for current event
    uint64_t sys_status_mask;      //!< Interrupt mask used (both LO and HI)
    uint16_t sts_threshold;        //!< Threshold for good STS received
    uint16_t sleep_mode;           //!< Base sleep parameters

#if MYNEWT_VAL(CIR_ENABLED)
    struct cir_dw3000_instance * cir;      //!< CIR instance (duplicate of uwb_dev->cir)
#endif
    dw3000_dev_rxdiag_t rxdiag;     //!< DW3000 receive diagnostics
    dw3000_dev_control_t control;  //!< DW3000 device control parameters

#if MYNEWT_VAL(DW3000_LWIP)
    void (* lwip_rx_complete_cb) (struct _dw3000_dev_instance_t *);
#endif
#if MYNEWT_VAL(DW3000_MAC_STATS)
    STATS_SECT_DECL(dw3000_mac_stat_section) stat;
#endif

#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
    struct dw3000_sys_status_backtrace sys_status_bt[MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)];
    uint16_t sys_status_bt_idx;
    uint8_t sys_status_bt_lock;
#endif
#if MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)
    struct dw3000_spi_backtrace spi_bt[MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)];
    uint16_t spi_bt_idx;
    uint8_t spi_bt_lock;
    uint8_t* spi_bt_datap;
#endif
#if MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN) || MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
    /* To allow translation from ticks to usecs in gdb during backtrace*/
    uint32_t bt_ticks2usec;
#endif
} dw3000_dev_instance_t;

//! SPI parameters
struct dw3000_dev_cfg {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct bus_spi_node_cfg spi_node_cfg;
#else
    uint8_t spi_num;              //!< SPI number
    struct dpl_sem *spi_sem;      //!< Pointer to os_sem structure to lock spi bus
    int spi_baudrate;             //!< SPI Baudrate (<42MHz)
#endif

    uint8_t rst_pin;              //!< Reset pin
    uint8_t irq_pin;              //!< Interrupt request pin
    uint8_t ss_pin;               //!< Slave select pin

    uint16_t rx_antenna_delay;    //!< Receive antenna delay
    uint16_t tx_antenna_delay;    //!< Transmit antenna delay
    int32_t  ext_clock_delay;     //!< External clock delay
};

int dw3000_dev_init(struct os_dev *odev, void *arg);
void dw3000_dev_deinit(dw3000_dev_instance_t * inst);
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
int dw3000_create_spi_dev(struct bus_spi_node *node, const char *name, struct dw3000_dev_cfg *cfg);
#endif
int dw3000_dev_config(dw3000_dev_instance_t * inst);
void dw3000_softreset(dw3000_dev_instance_t * inst);
struct uwb_dev_status dw3000_read(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
struct uwb_dev_status dw3000_write(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
uint64_t dw3000_read_reg(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, size_t nsize);
void dw3000_write_reg(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint64_t val, size_t nsize);
void dw3000_modify_reg(dw3000_dev_instance_t * inst, uint32_t reg, uint16_t subaddress, uint32_t _and, uint32_t _or, size_t nbytes);
#define dw3000_write_fast_CMD(_INST, _CMD) dw3000_write(_INST, _CMD,0,0,0)
void dw3000_dev_set_sleep_timer(dw3000_dev_instance_t * inst, uint16_t count);
void dw3000_dev_configure_sleep(dw3000_dev_instance_t * inst);
struct uwb_dev_status dw3000_dev_enter_sleep(dw3000_dev_instance_t * inst);
struct uwb_dev_status dw3000_dev_wakeup(dw3000_dev_instance_t * inst);
struct uwb_dev_status dw3000_dev_enter_sleep_after_tx(dw3000_dev_instance_t * inst, uint8_t enable);
struct uwb_dev_status dw3000_dev_enter_sleep_after_rx(dw3000_dev_instance_t * inst, uint8_t enable);

int dw3000_dev_checkidlerc(dw3000_dev_instance_t * inst);
void dw3000_dev_force_clocks(struct _dw3000_dev_instance_t * inst, int clocks);

#define dw3000_dwt_usecs_to_usecs(_t) (double)( (_t) * (0x10000UL/(128*499.2)))
#define dw3000_usecs_to_dwt_usecs(_t) (double)( (_t) / dw3000_dwt_usecs_to_usecs(1.0))

#ifdef __cplusplus
}
#endif

#endif /* _DW3000_DEV_H_ */
