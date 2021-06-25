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
 * @file dw3000_mac.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Mac initialization
 *
 * @details This is the mac base class which utilizes the functions to do the configurations related to mac layer based on dependencies.
 *
 */

#ifndef _DW3000_MAC_H_
#define _DW3000_MAC_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <uwb/uwb_mac.h>
#include <uwb/uwb_encryption.h>
#include <hal/hal_spi.h>
#include <stats/stats.h>
#include <dw3000-c0/dw3000_dev.h>

//! fast commands
#define CMD_DB_TOGGLE     0x13   //!< Toggle double buffer pointer
#define CMD_CLR_IRQS      0x12   //!< Clear all events/clear interrupt
#define CMD_CCA_TX_W4R    0x11   //!< Check if channel clear prior to TX, enable RX when TX done
#define CMD_DTX_REF_W4R   0x10   //!< Start delayed TX (as DTX_REF below), enable RX when TX done
#define CMD_DTX_RS_W4R    0xF    //!< Start delayed TX (as DTX_RS below), enable RX when TX done
#define CMD_DTX_TS_W4R    0xE    //!< Start delayed TX (as DTX_TS below), enable RX when TX done
#define CMD_DTX_W4R       0xD    //!< Start delayed TX (as DTX below), enable RX when TX done
#define CMD_TX_W4R        0xC    //!< Start TX (as below), enable RX when TX done
#define CMD_CCA_TX        0xB    //!< Check if channel clear prior to TX
#define CMD_DRX_REF       0xA    //!< Enable RX @ time = DREF_TIME + DX_TIME
#define CMD_DTX_REF       0x9    //!< Start delayed TX (RMARKER will be @ time = DREF_TIME + DX_TIME)
#define CMD_DRX_RS        0x8    //!< Enable RX @ time = RX_TIME + DX_TIME
#define CMD_DTX_RS        0x7    //!< Start delayed TX (RMARKER will be @ time = RX_TIME + DX_TIME)
#define CMD_DRX_TS        0x6    //!< Enable RX @ time = TX_TIME + DX_TIME
#define CMD_DTX_TS        0x5    //!< Start delayed TX (RMARKER will be @ time = TX_TIME + DX_TIME)
#define CMD_DRX           0x4    //!< Enable RX @ time specified in DX_TIME register
#define CMD_DTX           0x3    //!< Start delayed TX (RMARKER will be @ time set in DX_TIME register)
#define CMD_RX            0x2    //!< Enable RX
#define CMD_TX            0x1    //!< Start TX
#define CMD_TXRXOFF       0x0    //!< Turn off TX or RX, clear any TX/RX events and put DW3000 into IDLE

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_850K     0   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      1   //!< UWB bit rate 6.8 Mbits/s

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8        0   //!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16       1   //!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32       2   //!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC4        3   //!< PAC  4 (recommended for RX of preamble length  < 127

#define DWT_PHRMODE_STD             0x0     //!< standard PHR mode
#define DWT_PHRMODE_EXT             0x3     //!< DW proprietary extended frames PHR mode

#define DWT_PHRRATE_STD         0x0     // standard PHR rate
#define DWT_PHRRATE_DTA         0x1     // PHR at data rate (6M81)

// Define DW3000 PDOA modes
#define DWT_PDOA_M0           0x0     // DW PDOA mode is off
#define DWT_PDOA_M1           0x1     // DW PDOA mode  mode 1
#define DWT_PDOA_M3           0x3     // DW PDOA mode  mode 3

// Define DW3000 STS Preamble lengths
#define DWT_STS_LEN_32     0
#define DWT_STS_LEN_64     1
#define DWT_STS_LEN_128    2
#define DWT_STS_LEN_256    3
#define DWT_STS_LEN_512    4
#define DWT_STS_LEN_1024   5
#define DWT_STS_LEN_2048   6

/* Returns the value to set in CP_CFG0_ID for STS length. The x is the DWT_STS_LEN_X value */
#define GET_STS_REG_SET_VALUE(x)     ((uint16_t)1<<((x)+2))

//! constants for specifying TX Cipher preamble MIN and MAX length in blocks of 8 symbols (0 == 8)
#define DWT_CPLEN_MIN   0x03    //! 32 symbol length - this is the minimum length supported for DW3000
#define DWT_CPLEN_MAX   0xFE    //! 2040 symbol length - this is the maximum length supported for DW3000

//! constants for advancins LFSR to help re-sync when using CP modes */
#define DWT_LFSR_ADVANCE_64     (0x1)           /* advance LFSR by 128 symbols */
#define DWT_LFSR_ADVANCE_32     (0x2)           /* advance LFSR by 64 symbols */
#define DWT_LFSR_ADVANCE_8      (0x4)           /* advance LFSR by 16 symbols */
#define DWT_LFSR_ADVANCE_1      (0x7)           /* advance LFSR by 2 symbols */
#define DWT_LFSR_MASK           (0x7)

//! Multiplication factors to convert carrier integrator value to a frequency offset in Hz
#define DWT_FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)
#define DWT_FREQ_OFFSET_MULTIPLIER_110KB    (998.4e6/2.0/8192.0/131072.0)

//! Frame filtering configuration options.
#define DWT_FF_NOTYPE_EN            0x000           //!< No frame types allowed (FF disabled)
#define DWT_FF_BEACON_EN            0x001           //!< beacon frames allowed
#define DWT_FF_DATA_EN              0x002           //!< data frames allowed
#define DWT_FF_ACK_EN               0x004           //!< ack frames allowed
#define DWT_FF_MAC_EN               0x008           //!< mac control frames allowed
#define DWT_FF_RSVD_EN              0x010           //!< reserved frame types allowed
#define DWT_FF_MULTI_EN             0x020           //!< multipurpose frames allowed
#define DWT_FF_FRAG_EN              0x040           //!< fragmented frame types allowed
#define DWT_FF_EXTEND_EN            0x080           //!< extended frame types allowed
#define DWT_FF_COORD_EN             0x100           //!< behave as coordinator (can receive frames with no dest address (PAN ID has to match))
#define DWT_FF_IMPBRCAST_EN         0x200           //!< allow MAC implicit broadcast


//! DW3000 SLEEP and WAKEUP configuration parameters.
#define DWT_GOTORX       0x0200
#define DWT_GOTOIDLE     0x0100
#define DWT_SEL_GEAR3    0x00C0
#define DWT_SEL_GEAR2    0x0080                     // Short gear table
#define DWT_SEL_GEAR1    0x0040                     // SCP
#define DWT_SEL_GEAR0    0x0000                     // Long gear table
#define DWT_ALT_GEAR     0x0020
#define DWT_LOADLDO      0x0010
#define DWT_LOADDGC      0x0008
#define DWT_LOADBIAS     0x0004
#define DWT_RUNSAR       0x0002
#define DWT_CONFIG       0x0001                     // download the AON array into the HIF (configuration download)

#define DWT_PRESRVE_SLP  0x20                       // allows for SLEEP_EN bit to be "preserved", although it will self - clear on wake up
#define DWT_WAKE_WK      0x10                       // wake up on WAKEUP PIN
#define DWT_WAKE_CS      0x8                        // wake up on chip select
#define DWT_BR_DET       0x4                        // enable brownout detector during sleep/deep sleep
#define DWT_SLEEP        0x2                        // enable sleep (if this bit is clear the device will enter deep sleep)
#define DWT_SLP_EN       0x1                        // enable sleep/deep sleep functionality

//! DW3000 OTP operating parameter set selection.
#define DWT_OPSET_LONG   (0x0UL<<22)     //!< DW3000 OTP operating parameter set selection
#define DWT_OPSET_SCP    (0x1UL<<22)     //!< DW3000 OTP operating parameter set selection
#define DWT_OPSET_SHORT  (0x2UL<<22)     //!< DW3000 OTP operating parameter set selection

//! MAC frame format.
#define MAC_FFORMAT_FCTRL 0x0         //!<  MAC frame format - Control parameter selection
#define MAC_FFORMAT_FCTRL_LEN 0x2     //!<  MAC frame format - Length parameter selection
#define MAC_FFORMAT_FTYPE 0           //!<  MAC frame format - Frame type parameter selection
#define MAC_FTYPE_BEACON  0x0         //!<  MAC frame format - BEACON parameter selection
#define MAC_FTYPE_DATA    0x1         //!<  MAC frame format - DATA parameter selection
#define MAC_FTYPE_ACK     0x2         //!<  MAC frame format - ACK parameter selection
#define MAC_FTYPE_COMMAND 0x3         //!<  MAC frame format - COMMAND parameter selection

//! Mac device parameters.
typedef struct _dw3000_mac_deviceentcnts_t{
    uint16_t PHE ;                    //!< Number of received header errors
    uint16_t RSL ;                    //!< Number of received frame sync loss events
    uint16_t CRCG ;                   //!< Number of good CRC received frames
    uint16_t CRCB ;                   //!< Number of bad CRC (CRC error) received frames
    uint16_t ARFE ;                   //!< Number of address filter errors
    uint16_t OVER ;                   //!< Number of receiver overflows (used in double buffer mode)
    uint16_t SFDTO ;                  //!< SFD timeouts
    uint16_t PTO ;                    //!< Preamble timeouts
    uint16_t RTO ;                    //!< RX frame wait timeouts
    uint16_t TXF ;                    //!< Number of transmitted frames
    uint16_t HPW ;                    //!< Half period warn
    uint16_t TXW ;                    //!< Power up warn
} dw3000_mac_deviceentcnts_t ;


struct uwb_dev_status dw3000_mac_init(struct _dw3000_dev_instance_t * inst, struct uwb_dev_config * config);
struct uwb_dev_status dw3000_mac_config(struct _dw3000_dev_instance_t * inst, struct uwb_dev_config * config);

void dw3000_set_xtal_trim(struct _dw3000_dev_instance_t * inst, uint8_t value);
void dw3000_mac_cp_config(struct _dw3000_dev_instance_t * inst, int mode, uint8_t prf, uint8_t cipherPreambleLength);
void dw3000_mac_cp_set_key(struct _dw3000_dev_instance_t * inst, struct uwb_sts_cp_key *key);
void dw3000_mac_cp_set_iv(struct _dw3000_dev_instance_t * inst, struct uwb_sts_cp_iv *iv);
void dw3000_mac_cp_load_iv(struct _dw3000_dev_instance_t * inst);
void dw3000_mac_cp_lfsr_advance(struct _dw3000_dev_instance_t * inst, uint8_t advanceAmount);
void dw3000_mac_cp_lfsr_read_enable(struct _dw3000_dev_instance_t * inst);
uint32_t dw3000_mac_cp_lfsr_read(struct _dw3000_dev_instance_t * inst);
int dw3000_read_sts_quality(struct _dw3000_dev_instance_t * inst, int16_t *sts_qual_idx);

void dw3000_tasks_init(struct _dw3000_dev_instance_t * inst);
struct uwb_dev_status dw3000_mac_framefilter(struct _dw3000_dev_instance_t * inst, uint16_t enable);
struct uwb_dev_status dw3000_write_tx(struct _dw3000_dev_instance_t * inst,  uint8_t *txFrameBytes, uint16_t txBufferOffset, uint16_t txFrameLength);
struct uwb_dev_status dw3000_read_rx(struct _dw3000_dev_instance_t * inst,  uint8_t *rxFrameBytes, uint16_t rxBufferOffset, uint16_t rxFrameLength);
struct uwb_dev_status dw3000_start_tx(struct _dw3000_dev_instance_t * inst);
int dw3000_tx_wait(struct _dw3000_dev_instance_t * inst, uint32_t timeout);
struct uwb_dev_status dw3000_set_delay_start(struct _dw3000_dev_instance_t * inst, uint64_t dx_time);
struct uwb_dev_status dw3000_set_delay_ref_start(struct _dw3000_dev_instance_t * inst, uint64_t dx_time);
struct uwb_dev_status dw3000_set_delay_rs_start(struct _dw3000_dev_instance_t * inst, uint64_t dx_time);
struct uwb_dev_status dw3000_set_delay_ts_start(struct _dw3000_dev_instance_t * inst, uint64_t dx_time);
struct uwb_dev_status dw3000_set_cca_tx(struct _dw3000_dev_instance_t * inst, bool enable);

struct uwb_dev_status dw3000_set_wait4resp(struct _dw3000_dev_instance_t * inst, bool enable);
struct uwb_dev_status dw3000_set_wait4resp_delay(struct _dw3000_dev_instance_t * inst, uint32_t delay);
struct uwb_dev_status dw3000_set_on_error_continue(struct _dw3000_dev_instance_t * inst, bool enable);
struct uwb_dev_status dw3000_set_rxauto_disable(struct _dw3000_dev_instance_t * inst, bool disable);
struct uwb_dev_status dw3000_start_rx(struct _dw3000_dev_instance_t * inst);
struct uwb_dev_status dw3000_stop_rx(struct _dw3000_dev_instance_t * inst);
void dw3000_write_tx_fctrl(struct _dw3000_dev_instance_t * inst, uint16_t txFrameLength, uint16_t txBufferOffset, struct uwb_fctrl_ext *ext);
struct uwb_dev_status dw3000_read_accdata(struct _dw3000_dev_instance_t * inst, uint8_t *buffer, uint16_t len, uint16_t accOffset);
struct uwb_dev_status dw3000_set_autoack(struct _dw3000_dev_instance_t * inst, bool enable);
struct uwb_dev_status dw3000_set_autoack_delay(struct _dw3000_dev_instance_t * inst, uint8_t delay);
struct uwb_dev_status dw3000_set_dblrxbuff(struct _dw3000_dev_instance_t * inst, bool flag);
struct uwb_dev_status dw3000_set_rx_timeout(struct _dw3000_dev_instance_t * inst, uint32_t timeout);
struct uwb_dev_status dw3000_adj_rx_timeout(struct _dw3000_dev_instance_t * inst, uint32_t timeout);
struct uwb_dev_status dw3000_set_rx_window(struct _dw3000_dev_instance_t * inst, uint64_t rx_start, uint64_t rx_end);
struct uwb_dev_status dw3000_set_rx_post_tx_window(struct _dw3000_dev_instance_t * inst, uint64_t tx_ts, uint32_t tx_post_rmarker_len,
                                                   uint64_t rx_end);
struct uwb_dev_status dw3000_set_abs_timeout(struct _dw3000_dev_instance_t * inst, uint64_t rx_end);

void dw3000_configciadiag(struct _dw3000_dev_instance_t * inst, uint8_t enable_mask);
dpl_float32_t dw3000_calc_rssi(struct _dw3000_dev_instance_t * inst, struct dw3000_rxdiag_sequence * seq);
dpl_float32_t dw3000_get_rssi(struct _dw3000_dev_instance_t * inst);
dpl_float32_t dw3000_calc_fppl(struct _dw3000_dev_instance_t * inst, struct dw3000_rxdiag_sequence * seq);
dpl_float32_t dw3000_get_fppl(struct _dw3000_dev_instance_t * inst);
dpl_float32_t dw3000_estimate_los(dpl_float32_t rssi, dpl_float32_t fppl);

int32_t dw3000_read_carrier_integrator(struct _dw3000_dev_instance_t * inst);
dpl_float64_t dw3000_calc_clock_offset_ratio(struct _dw3000_dev_instance_t * inst, int32_t integrator_val);
dpl_float64_t dw3000_calc_clock_offset_ratio_ttco(struct _dw3000_dev_instance_t * inst, int32_t ttcko);

void dw3000_read_rxdiag(struct _dw3000_dev_instance_t * inst, struct _dw3000_dev_rxdiag_t * diag, uint16_t diag_enabled);
#define dw3000_set_preamble_timeout(counts) dw3000_write_reg(inst, DRX_CONF_ID, DRX_PRETOC_OFFSET, counts, sizeof(uint16_t))
#define dw3000_set_panid(inst, pan_id) dw3000_write_reg(inst, PANADR_ID, PANADR_PAN_ID_BYTE_OFFSET, pan_id, sizeof(uint16_t))
#define dw3000_set_address16(inst, shortAddress) dw3000_write_reg(inst ,PANADR_ID, 0, shortAddress, sizeof(uint16_t))
#define dw3000_set_eui(inst, eui64) dw3000_write_reg(inst, EUI_64_LO_ID, 0, eui64, EUI_64_LO_LEN+EUI_64_HI_LEN)
#define dw3000_get_eui(inst) (uint64_t) dw3000_read_reg(inst, EUI_64_LO_ID, 0, EUI_64_LO_LEN+EUI_64_HI_LEN)
#define dw3000_checkoverrun(inst) (uint16_t)(dw3000_read_reg(inst, SYS_STATUS_ID, 2, sizeof(uint16_t)) & (SYS_STATUS_RXOVRR_BIT_MASK >> 16))

uint64_t dw3000_read_systime(struct _dw3000_dev_instance_t * inst);
uint32_t dw3000_read_systime_lo(struct _dw3000_dev_instance_t * inst);
uint64_t dw3000_read_rawrxtime(struct _dw3000_dev_instance_t * inst);
uint64_t dw3000_read_rxtime(struct _dw3000_dev_instance_t * inst);
uint64_t dw3000_read_txrawst(struct _dw3000_dev_instance_t * inst);
uint32_t dw3000_read_rxtime_lo(struct _dw3000_dev_instance_t * inst);
uint64_t dw3000_read_sts_rxtime(struct _dw3000_dev_instance_t * inst);
uint64_t dw3000_read_txtime(struct _dw3000_dev_instance_t * inst);
uint32_t dw3000_read_txtime_lo(struct _dw3000_dev_instance_t * inst);

uint32_t dw3000_read_pdoa(struct _dw3000_dev_instance_t * inst);

#define dw3000_get_irqstatus(inst) ((uint8_t) dw3000_read_reg(inst, SYS_STATUS_ID, SYS_STATUS_OFFSET,sizeof(uint8_t)) & (uint8_t) SYS_STATUS_IRQS)

void dw3000_configcwmode(struct _dw3000_dev_instance_t * inst, uint8_t chan);
void* dw3000_mac_find_cb_inst_ptr(dw3000_dev_instance_t * inst, uint16_t id);

#ifdef __cplusplus
}
#endif
#endif /* _DW3000_MAC_H_ */
