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
 * @file dw3000_regs.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2020
 * @brief Register file
 *
 * @details This is the reg base class which includes all the dw3000 register's data.
 */

#ifndef _DW3000_REGS_H_
#define _DW3000_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "dw3000_regs_vals.h"

/******************************************************************************
* @brief Bit definitions for register DEV_ID
**/
#define DEV_ID_ID                            0x0
#define DEV_ID_LEN                           (4U)
#define DEV_ID_MASK                          0xFFFFFFFFUL
#define DEV_ID_RIDTAG_BIT_OFFSET             (16U)
#define DEV_ID_RIDTAG_BIT_LEN                (16U)
#define DEV_ID_RIDTAG_BIT_MASK               0xffff0000UL
#define DEV_ID_MODEL_BIT_OFFSET              (8U)
#define DEV_ID_MODEL_BIT_LEN                 (8U)
#define DEV_ID_MODEL_BIT_MASK                0xff00U
#define DEV_ID_VER_BIT_OFFSET                (4U)
#define DEV_ID_VER_BIT_LEN                   (4U)
#define DEV_ID_VER_BIT_MASK                  0xf0U
#define DEV_ID_REV_BIT_OFFSET                (0U)
#define DEV_ID_REV_BIT_LEN                   (4U)
#define DEV_ID_REV_BIT_MASK                  0xfU

/******************************************************************************
* @brief Bit definitions for register EUI_64_LO
**/
#define EUI_64_LO_ID                         0x4
#define EUI_64_LO_LEN                        (4U)
#define EUI_64_LO_MASK                       0xFFFFFFFFUL
#define EUI_64_LO_EUI_64_BIT_OFFSET          (0U)
#define EUI_64_LO_EUI_64_BIT_LEN             (32U)
#define EUI_64_LO_EUI_64_BIT_MASK            0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register EUI_64_HI
**/
#define EUI_64_HI_ID                         0x8
#define EUI_64_HI_LEN                        (4U)
#define EUI_64_HI_MASK                       0xFFFFFFFFUL
#define EUI_64_HI_EUI_64_BIT_OFFSET          (0U)
#define EUI_64_HI_EUI_64_BIT_LEN             (32U)
#define EUI_64_HI_EUI_64_BIT_MASK            0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register PANADR
**/
#define PANADR_ID                            0xc
#define PANADR_LEN                           (4U)
#define PANADR_MASK                          0xFFFFFFFFUL
#define PANADR_PAN_ID_BIT_OFFSET             (16U)
#define PANADR_PAN_ID_BIT_LEN                (16U)
#define PANADR_PAN_ID_BIT_MASK               0xffff0000UL
#define PANADR_SHORT_ADDR_BIT_OFFSET         (0U)
#define PANADR_SHORT_ADDR_BIT_LEN            (16U)
#define PANADR_SHORT_ADDR_BIT_MASK           0xffffU

/******************************************************************************
* @brief Bit definitions for register SYS_CFG
**/
#define SYS_CFG_ID                           0x10
#define SYS_CFG_LEN                          (4U)
#define SYS_CFG_MASK                         0xFFFFFFFFUL
#define SYS_CFG_FAST_AAT_EN_BIT_OFFSET       (18U)
#define SYS_CFG_FAST_AAT_EN_BIT_LEN          (1U)
#define SYS_CFG_FAST_AAT_EN_BIT_MASK         0x40000UL
#define SYS_CFG_PDOA_MODE_BIT_OFFSET         (16U)
#define SYS_CFG_PDOA_MODE_BIT_LEN            (2U)
#define SYS_CFG_PDOA_MODE_BIT_MASK           0x30000UL
#define SYS_CFG_CP_SDC_BIT_OFFSET            (15U)
#define SYS_CFG_CP_SDC_BIT_LEN               (1U)
#define SYS_CFG_CP_SDC_BIT_MASK              0x8000U
#define SYS_CFG_CP_TYPE_BIT_OFFSET           (14U)
#define SYS_CFG_CP_TYPE_BIT_LEN              (1U)
#define SYS_CFG_CP_TYPE_BIT_MASK             0x4000U
#define SYS_CFG_CP_PROTOCOL_BIT_OFFSET       (12U)
#define SYS_CFG_CP_PROTOCOL_BIT_LEN          (2U)
#define SYS_CFG_CP_PROTOCOL_BIT_MASK         0x3000U
#define SYS_CFG_AUTO_ACK_BIT_OFFSET          (11U)
#define SYS_CFG_AUTO_ACK_BIT_LEN             (1U)
#define SYS_CFG_AUTO_ACK_BIT_MASK            0x800U
#define SYS_CFG_RXAUTR_BIT_OFFSET            (10U)
#define SYS_CFG_RXAUTR_BIT_LEN               (1U)
#define SYS_CFG_RXAUTR_BIT_MASK              0x400U
#define SYS_CFG_RXWTOE_BIT_OFFSET            (9U)
#define SYS_CFG_RXWTOE_BIT_LEN               (1U)
#define SYS_CFG_RXWTOE_BIT_MASK              0x200U
#define SYS_CFG_CIA_CIPHER_BIT_OFFSET        (8U)
#define SYS_CFG_CIA_CIPHER_BIT_LEN           (1U)
#define SYS_CFG_CIA_CIPHER_BIT_MASK          0x100U
#define SYS_CFG_CIA_IPATOV_BIT_OFFSET        (7U)
#define SYS_CFG_CIA_IPATOV_BIT_LEN           (1U)
#define SYS_CFG_CIA_IPATOV_BIT_MASK          0x80U
#define SYS_CFG_SPI_CRC_BIT_OFFSET           (6U)
#define SYS_CFG_SPI_CRC_BIT_LEN              (1U)
#define SYS_CFG_SPI_CRC_BIT_MASK             0x40U
#define SYS_CFG_PHR_6M8_BIT_OFFSET           (5U)
#define SYS_CFG_PHR_6M8_BIT_LEN              (1U)
#define SYS_CFG_PHR_6M8_BIT_MASK             0x20U
#define SYS_CFG_PHR_MODE_BIT_OFFSET          (4U)
#define SYS_CFG_PHR_MODE_BIT_LEN             (1U)
#define SYS_CFG_PHR_MODE_BIT_MASK            0x10U
#define SYS_CFG_EN_DRXB_BIT_OFFSET           (3U)
#define SYS_CFG_EN_DRXB_BIT_LEN              (1U)
#define SYS_CFG_EN_DRXB_BIT_MASK             0x8U
#define SYS_CFG_DIS_FCE_BIT_OFFSET           (2U)
#define SYS_CFG_DIS_FCE_BIT_LEN              (1U)
#define SYS_CFG_DIS_FCE_BIT_MASK             0x4U
#define SYS_CFG_DIS_FCS_TX_BIT_OFFSET        (1U)
#define SYS_CFG_DIS_FCS_TX_BIT_LEN           (1U)
#define SYS_CFG_DIS_FCS_TX_BIT_MASK          0x2U
#define SYS_CFG_FFEN_BIT_OFFSET              (0U)
#define SYS_CFG_FFEN_BIT_LEN                 (1U)
#define SYS_CFG_FFEN_BIT_MASK                0x1U

/******************************************************************************
* @brief Bit definitions for register ADR_FILT_CFG
**/
#define ADR_FILT_CFG_ID                      0x14
#define ADR_FILT_CFG_LEN                     (4U)
#define ADR_FILT_CFG_MASK                    0xFFFFFFFFUL
#define ADR_FILT_CFG_LSADRAPE_BIT_OFFSET     (15U)
#define ADR_FILT_CFG_LSADRAPE_BIT_LEN        (1U)
#define ADR_FILT_CFG_LSADRAPE_BIT_MASK       0x8000U
#define ADR_FILT_CFG_SSADRAPE_BIT_OFFSET     (14U)
#define ADR_FILT_CFG_SSADRAPE_BIT_LEN        (1U)
#define ADR_FILT_CFG_SSADRAPE_BIT_MASK       0x4000U
#define ADR_FILT_CFG_LE3_PEND_BIT_OFFSET     (13U)
#define ADR_FILT_CFG_LE3_PEND_BIT_LEN        (1U)
#define ADR_FILT_CFG_LE3_PEND_BIT_MASK       0x2000U
#define ADR_FILT_CFG_LE2_PEND_BIT_OFFSET     (12U)
#define ADR_FILT_CFG_LE2_PEND_BIT_LEN        (1U)
#define ADR_FILT_CFG_LE2_PEND_BIT_MASK       0x1000U
#define ADR_FILT_CFG_LE1_PEND_BIT_OFFSET     (11U)
#define ADR_FILT_CFG_LE1_PEND_BIT_LEN        (1U)
#define ADR_FILT_CFG_LE1_PEND_BIT_MASK       0x800U
#define ADR_FILT_CFG_LE0_PEND_BIT_OFFSET     (10U)
#define ADR_FILT_CFG_LE0_PEND_BIT_LEN        (1U)
#define ADR_FILT_CFG_LE0_PEND_BIT_MASK       0x400U
#define ADR_FILT_CFG_FFIB_BIT_OFFSET         (9U)
#define ADR_FILT_CFG_FFIB_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFIB_BIT_MASK           0x200U
#define ADR_FILT_CFG_FFBC_BIT_OFFSET         (8U)
#define ADR_FILT_CFG_FFBC_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFBC_BIT_MASK           0x100U
#define ADR_FILT_CFG_FFAE_BIT_OFFSET         (7U)
#define ADR_FILT_CFG_FFAE_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFAE_BIT_MASK           0x80U
#define ADR_FILT_CFG_FFAF_BIT_OFFSET         (6U)
#define ADR_FILT_CFG_FFAF_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFAF_BIT_MASK           0x40U
#define ADR_FILT_CFG_FFAMULTI_BIT_OFFSET     (5U)
#define ADR_FILT_CFG_FFAMULTI_BIT_LEN        (1U)
#define ADR_FILT_CFG_FFAMULTI_BIT_MASK       0x20U
#define ADR_FILT_CFG_FFAR_BIT_OFFSET         (4U)
#define ADR_FILT_CFG_FFAR_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFAR_BIT_MASK           0x10U
#define ADR_FILT_CFG_FFAM_BIT_OFFSET         (3U)
#define ADR_FILT_CFG_FFAM_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFAM_BIT_MASK           0x8U
#define ADR_FILT_CFG_FFAA_BIT_OFFSET         (2U)
#define ADR_FILT_CFG_FFAA_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFAA_BIT_MASK           0x4U
#define ADR_FILT_CFG_FFAD_BIT_OFFSET         (1U)
#define ADR_FILT_CFG_FFAD_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFAD_BIT_MASK           0x2U
#define ADR_FILT_CFG_FFAB_BIT_OFFSET         (0U)
#define ADR_FILT_CFG_FFAB_BIT_LEN            (1U)
#define ADR_FILT_CFG_FFAB_BIT_MASK           0x1U

/******************************************************************************
* @brief Bit definitions for register SPICRC_CFG
**/
#define SPICRC_CFG_ID                        0x18
#define SPICRC_CFG_LEN                       (4U)
#define SPICRC_CFG_MASK                      0xFFFFFFFFUL
#define SPICRC_CFG_READ_BIT_OFFSET           (0U)
#define SPICRC_CFG_READ_BIT_LEN              (8U)
#define SPICRC_CFG_READ_BIT_MASK             0xffU

/******************************************************************************
* @brief Bit definitions for register SYS_TIME
**/
#define SYS_TIME_ID                          0x1c
#define SYS_TIME_LEN                         (4U)
#define SYS_TIME_MASK                        0xFFFFFFFFUL
#define SYS_TIME_SYS_TIME_BIT_OFFSET         (1U)
#define SYS_TIME_SYS_TIME_BIT_LEN            (31U)
#define SYS_TIME_SYS_TIME_BIT_MASK           0xfffffffeUL

/******************************************************************************
* @brief Bit definitions for register TX_FCTRL
**/
#define TX_FCTRL_ID                          0x24
#define TX_FCTRL_LEN                         (4U)
#define TX_FCTRL_MASK                        0xFFFFFFFFUL
#define TX_FCTRL_TXB_OFFSET_BIT_OFFSET       (16U)
#define TX_FCTRL_TXB_OFFSET_BIT_LEN          (10U)
#define TX_FCTRL_TXB_OFFSET_BIT_MASK         0x3ff0000UL
#define TX_FCTRL_TXPSR_PE_BIT_OFFSET         (12U)
#define TX_FCTRL_TXPSR_PE_BIT_LEN            (4U)
#define TX_FCTRL_TXPSR_PE_BIT_MASK           0xf000U
#define TX_FCTRL_TR_BIT_OFFSET               (11U)
#define TX_FCTRL_TR_BIT_LEN                  (1U)
#define TX_FCTRL_TR_BIT_MASK                 0x800U
#define TX_FCTRL_TXBR_BIT_OFFSET             (10U)
#define TX_FCTRL_TXBR_BIT_LEN                (1U)
#define TX_FCTRL_TXBR_BIT_MASK               0x400U
#define TX_FCTRL_TXFLEN_BIT_OFFSET           (0U)
#define TX_FCTRL_TXFLEN_BIT_LEN              (10U)
#define TX_FCTRL_TXFLEN_BIT_MASK             0x3ffU

/******************************************************************************
* @brief Bit definitions for register TX_FCTRL_HI
**/
#define TX_FCTRL_HI_ID                       0x28
#define TX_FCTRL_HI_LEN                      (4U)
#define TX_FCTRL_HI_MASK                     0xFFFFFFFFUL
#define TX_FCTRL_HI_ACK_FINE_PLEN_BIT_OFFSET (16U)
#define TX_FCTRL_HI_ACK_FINE_PLEN_BIT_LEN    (8U)
#define TX_FCTRL_HI_ACK_FINE_PLEN_BIT_MASK   0xff0000UL
#define TX_FCTRL_HI_FINE_PLEN_BIT_OFFSET     (8U)
#define TX_FCTRL_HI_FINE_PLEN_BIT_LEN        (8U)
#define TX_FCTRL_HI_FINE_PLEN_BIT_MASK       0xff00U
#define TX_FCTRL_HI_IFS_DLY_BIT_OFFSET       (0U)
#define TX_FCTRL_HI_IFS_DLY_BIT_LEN          (8U)
#define TX_FCTRL_HI_IFS_DLY_BIT_MASK         0xffU

/******************************************************************************
* @brief Bit definitions for register DX_TIME
**/
#define DX_TIME_ID                           0x2c
#define DX_TIME_LEN                          (4U)
#define DX_TIME_MASK                         0xFFFFFFFFUL
#define DX_TIME_DX_TIME_BIT_OFFSET           (1U)
#define DX_TIME_DX_TIME_BIT_LEN              (31U)
#define DX_TIME_DX_TIME_BIT_MASK             0xfffffffeUL

/******************************************************************************
* @brief Bit definitions for register DREF_TIME
**/
#define DREF_TIME_ID                         0x30
#define DREF_TIME_LEN                        (4U)
#define DREF_TIME_MASK                       0xFFFFFFFFUL
#define DREF_TIME_DREF_BIT_OFFSET            (1U)
#define DREF_TIME_DREF_BIT_LEN               (31U)
#define DREF_TIME_DREF_BIT_MASK              0xfffffffeUL

/******************************************************************************
* @brief Bit definitions for register RX_FWTO
**/
#define RX_FWTO_ID                           0x34
#define RX_FWTO_LEN                          (4U)
#define RX_FWTO_MASK                         0xFFFFFFFFUL
#define RX_FWTO_FWTO_BIT_OFFSET              (0U)
#define RX_FWTO_FWTO_BIT_LEN                 (20U)
#define RX_FWTO_FWTO_BIT_MASK                0xfffffUL

/******************************************************************************
* @brief Bit definitions for register SYS_CTRL
**/
#define SYS_CTRL_ID                          0x38
#define SYS_CTRL_LEN                         (4U)
#define SYS_CTRL_MASK                        0xFFFFFFFFUL
#define SYS_CTRL_HSRBTOGGLE_BIT_OFFSET       (24U)
#define SYS_CTRL_HSRBTOGGLE_BIT_LEN          (1U)
#define SYS_CTRL_HSRBTOGGLE_BIT_MASK         0x1000000UL
#define SYS_CTRL_RXDLYE_BIT_OFFSET           (9U)
#define SYS_CTRL_RXDLYE_BIT_LEN              (1U)
#define SYS_CTRL_RXDLYE_BIT_MASK             0x200U
#define SYS_CTRL_RXENAB_BIT_OFFSET           (8U)
#define SYS_CTRL_RXENAB_BIT_LEN              (1U)
#define SYS_CTRL_RXENAB_BIT_MASK             0x100U
#define SYS_CTRL_WAIT4RESP_BIT_OFFSET        (7U)
#define SYS_CTRL_WAIT4RESP_BIT_LEN           (1U)
#define SYS_CTRL_WAIT4RESP_BIT_MASK          0x80U
#define SYS_CTRL_TRXOFF_BIT_OFFSET           (6U)
#define SYS_CTRL_TRXOFF_BIT_LEN              (1U)
#define SYS_CTRL_TRXOFF_BIT_MASK             0x40U
#define SYS_CTRL_TXDLYS_BIT_OFFSET           (2U)
#define SYS_CTRL_TXDLYS_BIT_LEN              (1U)
#define SYS_CTRL_TXDLYS_BIT_MASK             0x4U
#define SYS_CTRL_TXSTRT_BIT_OFFSET           (1U)
#define SYS_CTRL_TXSTRT_BIT_LEN              (1U)
#define SYS_CTRL_TXSTRT_BIT_MASK             0x2U

/******************************************************************************
* @brief Bit definitions for register SYS_ENABLE_LO
**/
#define SYS_ENABLE_LO_ID                     0x3c
#define SYS_ENABLE_LO_LEN                    (4U)
#define SYS_ENABLE_LO_MASK                   0xFFFFFFFFUL
#define SYS_ENABLE_LO_ARFE_ENABLE_BIT_OFFSET (29U)
#define SYS_ENABLE_LO_ARFE_ENABLE_BIT_LEN    (1U)
#define SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK   0x20000000UL
#define SYS_ENABLE_LO_CPERR_ENABLE_BIT_OFFSET (28U)
#define SYS_ENABLE_LO_CPERR_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_CPERR_ENABLE_BIT_MASK  0x10000000UL
#define SYS_ENABLE_LO_HPDWARN_ENABLE_BIT_OFFSET (27U)
#define SYS_ENABLE_LO_HPDWARN_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_LO_HPDWARN_ENABLE_BIT_MASK 0x8000000UL
#define SYS_ENABLE_LO_RXSTO_ENABLE_BIT_OFFSET (26U)
#define SYS_ENABLE_LO_RXSTO_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK  0x4000000UL
#define SYS_ENABLE_LO_PLL_HILO_ENABLE_BIT_OFFSET (25U)
#define SYS_ENABLE_LO_PLL_HILO_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_LO_PLL_HILO_ENABLE_BIT_MASK 0x2000000UL
#define SYS_ENABLE_LO_RCINIT_ENABLE_BIT_OFFSET (24U)
#define SYS_ENABLE_LO_RCINIT_ENABLE_BIT_LEN  (1U)
#define SYS_ENABLE_LO_RCINIT_ENABLE_BIT_MASK 0x1000000UL
#define SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_OFFSET (23U)
#define SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_LEN  (1U)
#define SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_MASK 0x800000UL
#define SYS_ENABLE_LO_LCSSERR_ENABLE_BIT_OFFSET (22U)
#define SYS_ENABLE_LO_LCSSERR_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_LO_LCSSERR_ENABLE_BIT_MASK 0x400000UL
#define SYS_ENABLE_LO_RXPTO_ENABLE_BIT_OFFSET (21U)
#define SYS_ENABLE_LO_RXPTO_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK  0x200000UL
#define SYS_ENABLE_LO_RXOVRR_ENABLE_BIT_OFFSET (20U)
#define SYS_ENABLE_LO_RXOVRR_ENABLE_BIT_LEN  (1U)
#define SYS_ENABLE_LO_RXOVRR_ENABLE_BIT_MASK 0x100000UL
#define SYS_ENABLE_LO_VWARN_ENABLE_BIT_OFFSET (19U)
#define SYS_ENABLE_LO_VWARN_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_VWARN_ENABLE_BIT_MASK  0x80000UL
#define SYS_ENABLE_LO_CIAERR_ENABLE_BIT_OFFSET (18U)
#define SYS_ENABLE_LO_CIAERR_ENABLE_BIT_LEN  (1U)
#define SYS_ENABLE_LO_CIAERR_ENABLE_BIT_MASK 0x40000UL
#define SYS_ENABLE_LO_RXFTO_ENABLE_BIT_OFFSET (17U)
#define SYS_ENABLE_LO_RXFTO_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK  0x20000UL
#define SYS_ENABLE_LO_RXFSL_ENABLE_BIT_OFFSET (16U)
#define SYS_ENABLE_LO_RXFSL_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK  0x10000UL
#define SYS_ENABLE_LO_RXFCE_ENABLE_BIT_OFFSET (15U)
#define SYS_ENABLE_LO_RXFCE_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK  0x8000U
#define SYS_ENABLE_LO_RXFCG_ENABLE_BIT_OFFSET (14U)
#define SYS_ENABLE_LO_RXFCG_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK  0x4000U
#define SYS_ENABLE_LO_RXFR_ENABLE_BIT_OFFSET (13U)
#define SYS_ENABLE_LO_RXFR_ENABLE_BIT_LEN    (1U)
#define SYS_ENABLE_LO_RXFR_ENABLE_BIT_MASK   0x2000U
#define SYS_ENABLE_LO_RXPHE_ENABLE_BIT_OFFSET (12U)
#define SYS_ENABLE_LO_RXPHE_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK  0x1000U
#define SYS_ENABLE_LO_RXPHD_ENABLE_BIT_OFFSET (11U)
#define SYS_ENABLE_LO_RXPHD_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXPHD_ENABLE_BIT_MASK  0x800U
#define SYS_ENABLE_LO_CIA_DONE_ENABLE_BIT_OFFSET (10U)
#define SYS_ENABLE_LO_CIA_DONE_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_LO_CIA_DONE_ENABLE_BIT_MASK 0x400U
#define SYS_ENABLE_LO_RXSFDD_ENABLE_BIT_OFFSET (9U)
#define SYS_ENABLE_LO_RXSFDD_ENABLE_BIT_LEN  (1U)
#define SYS_ENABLE_LO_RXSFDD_ENABLE_BIT_MASK 0x200U
#define SYS_ENABLE_LO_RXPRD_ENABLE_BIT_OFFSET (8U)
#define SYS_ENABLE_LO_RXPRD_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_RXPRD_ENABLE_BIT_MASK  0x100U
#define SYS_ENABLE_LO_TXFRS_ENABLE_BIT_OFFSET (7U)
#define SYS_ENABLE_LO_TXFRS_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK  0x80U
#define SYS_ENABLE_LO_TXPHS_ENABLE_BIT_OFFSET (6U)
#define SYS_ENABLE_LO_TXPHS_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_TXPHS_ENABLE_BIT_MASK  0x40U
#define SYS_ENABLE_LO_TXPRS_ENABLE_BIT_OFFSET (5U)
#define SYS_ENABLE_LO_TXPRS_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_TXPRS_ENABLE_BIT_MASK  0x20U
#define SYS_ENABLE_LO_TXFRB_ENABLE_BIT_OFFSET (4U)
#define SYS_ENABLE_LO_TXFRB_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_LO_TXFRB_ENABLE_BIT_MASK  0x10U
#define SYS_ENABLE_LO_AAT_ENABLE_BIT_OFFSET  (3U)
#define SYS_ENABLE_LO_AAT_ENABLE_BIT_LEN     (1U)
#define SYS_ENABLE_LO_AAT_ENABLE_BIT_MASK    0x8U
#define SYS_ENABLE_LO_SPICRCERR_ENABLE_BIT_OFFSET (2U)
#define SYS_ENABLE_LO_SPICRCERR_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_LO_SPICRCERR_ENABLE_BIT_MASK 0x4U
#define SYS_ENABLE_LO_CLK_PLL_LOCK_ENABLE_BIT_OFFSET (1U)
#define SYS_ENABLE_LO_CLK_PLL_LOCK_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_LO_CLK_PLL_LOCK_ENABLE_BIT_MASK 0x2U

/******************************************************************************
* @brief Bit definitions for register SYS_ENABLE_HI
**/
#define SYS_ENABLE_HI_ID                     0x40
#define SYS_ENABLE_HI_LEN                    (4U)
#define SYS_ENABLE_HI_MASK                   0xFFFFFFFFUL
#define SYS_ENABLE_HI_CCA_FAIL_ENABLE_BIT_OFFSET (12U)
#define SYS_ENABLE_HI_CCA_FAIL_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_CCA_FAIL_ENABLE_BIT_MASK 0x1000U
#define SYS_ENABLE_HI_SPI_COLLISION_ENABLE_BIT_OFFSET (11U)
#define SYS_ENABLE_HI_SPI_COLLISION_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_SPI_COLLISION_ENABLE_BIT_MASK 0x800U
#define SYS_ENABLE_HI_SPI_UNF_ENABLE_BIT_OFFSET (10U)
#define SYS_ENABLE_HI_SPI_UNF_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_SPI_UNF_ENABLE_BIT_MASK 0x400U
#define SYS_ENABLE_HI_SPI_OVF_ENABLE_BIT_OFFSET (9U)
#define SYS_ENABLE_HI_SPI_OVF_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_SPI_OVF_ENABLE_BIT_MASK 0x200U
#define SYS_ENABLE_HI_CMD_ERR_ENABLE_BIT_OFFSET (8U)
#define SYS_ENABLE_HI_CMD_ERR_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_CMD_ERR_ENABLE_BIT_MASK 0x100U
#define SYS_ENABLE_HI_AES_ERR_ENABLE_BIT_OFFSET (7U)
#define SYS_ENABLE_HI_AES_ERR_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_AES_ERR_ENABLE_BIT_MASK 0x80U
#define SYS_ENABLE_HI_AES_DONE_ENABLE_BIT_OFFSET (6U)
#define SYS_ENABLE_HI_AES_DONE_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_AES_DONE_ENABLE_BIT_MASK 0x40U
#define SYS_ENABLE_HI_GPIO_ENABLE_BIT_OFFSET (5U)
#define SYS_ENABLE_HI_GPIO_ENABLE_BIT_LEN    (1U)
#define SYS_ENABLE_HI_GPIO_ENABLE_BIT_MASK   0x20U
#define SYS_ENABLE_HI_VT_DET_ENABLE_BIT_OFFSET (4U)
#define SYS_ENABLE_HI_VT_DET_ENABLE_BIT_LEN  (1U)
#define SYS_ENABLE_HI_VT_DET_ENABLE_BIT_MASK 0x10U
#define SYS_ENABLE_HI_ESYNC_RST_ENABLE_BIT_OFFSET (3U)
#define SYS_ENABLE_HI_ESYNC_RST_ENABLE_BIT_LEN (1U)
#define SYS_ENABLE_HI_ESYNC_RST_ENABLE_BIT_MASK 0x8U
#define SYS_ENABLE_HI_RXPREJ_ENABLE_BIT_OFFSET (1U)
#define SYS_ENABLE_HI_RXPREJ_ENABLE_BIT_LEN  (1U)
#define SYS_ENABLE_HI_RXPREJ_ENABLE_BIT_MASK 0x2U
#define SYS_ENABLE_HI_RXSCE_ENABLE_BIT_OFFSET (0U)
#define SYS_ENABLE_HI_RXSCE_ENABLE_BIT_LEN   (1U)
#define SYS_ENABLE_HI_RXSCE_ENABLE_BIT_MASK  0x1U

/******************************************************************************
* @brief Bit definitions for register SYS_STATUS
**/
#define SYS_STATUS_ID                        0x44
#define SYS_STATUS_LEN                       (4U)
#define SYS_STATUS_MASK                      0xFFFFFFFFUL
#define SYS_STATUS_ARFE_BIT_OFFSET           (29U)
#define SYS_STATUS_ARFE_BIT_LEN              (1U)
#define SYS_STATUS_ARFE_BIT_MASK             0x20000000UL
#define SYS_STATUS_CPERR_BIT_OFFSET          (28U)
#define SYS_STATUS_CPERR_BIT_LEN             (1U)
#define SYS_STATUS_CPERR_BIT_MASK            0x10000000UL
#define SYS_STATUS_HPDWARN_BIT_OFFSET        (27U)
#define SYS_STATUS_HPDWARN_BIT_LEN           (1U)
#define SYS_STATUS_HPDWARN_BIT_MASK          0x8000000UL
#define SYS_STATUS_RXSTO_BIT_OFFSET          (26U)
#define SYS_STATUS_RXSTO_BIT_LEN             (1U)
#define SYS_STATUS_RXSTO_BIT_MASK            0x4000000UL
#define SYS_STATUS_PLL_HILO_BIT_OFFSET       (25U)
#define SYS_STATUS_PLL_HILO_BIT_LEN          (1U)
#define SYS_STATUS_PLL_HILO_BIT_MASK         0x2000000UL
#define SYS_STATUS_RCINIT_BIT_OFFSET         (24U)
#define SYS_STATUS_RCINIT_BIT_LEN            (1U)
#define SYS_STATUS_RCINIT_BIT_MASK           0x1000000UL
#define SYS_STATUS_SPIRDY_BIT_OFFSET         (23U)
#define SYS_STATUS_SPIRDY_BIT_LEN            (1U)
#define SYS_STATUS_SPIRDY_BIT_MASK           0x800000UL
#define SYS_STATUS_LCSSERR_BIT_OFFSET        (22U)
#define SYS_STATUS_LCSSERR_BIT_LEN           (1U)
#define SYS_STATUS_LCSSERR_BIT_MASK          0x400000UL
#define SYS_STATUS_RXPTO_BIT_OFFSET          (21U)
#define SYS_STATUS_RXPTO_BIT_LEN             (1U)
#define SYS_STATUS_RXPTO_BIT_MASK            0x200000UL
#define SYS_STATUS_RXOVRR_BIT_OFFSET         (20U)
#define SYS_STATUS_RXOVRR_BIT_LEN            (1U)
#define SYS_STATUS_RXOVRR_BIT_MASK           0x100000UL
#define SYS_STATUS_VWARN_BIT_OFFSET          (19U)
#define SYS_STATUS_VWARN_BIT_LEN             (1U)
#define SYS_STATUS_VWARN_BIT_MASK            0x80000UL
#define SYS_STATUS_CIAERR_BIT_OFFSET         (18U)
#define SYS_STATUS_CIAERR_BIT_LEN            (1U)
#define SYS_STATUS_CIAERR_BIT_MASK           0x40000UL
#define SYS_STATUS_RXFTO_BIT_OFFSET          (17U)
#define SYS_STATUS_RXFTO_BIT_LEN             (1U)
#define SYS_STATUS_RXFTO_BIT_MASK            0x20000UL
#define SYS_STATUS_RXFSL_BIT_OFFSET          (16U)
#define SYS_STATUS_RXFSL_BIT_LEN             (1U)
#define SYS_STATUS_RXFSL_BIT_MASK            0x10000UL
#define SYS_STATUS_RXFCE_BIT_OFFSET          (15U)
#define SYS_STATUS_RXFCE_BIT_LEN             (1U)
#define SYS_STATUS_RXFCE_BIT_MASK            0x8000U
#define SYS_STATUS_RXFCG_BIT_OFFSET          (14U)
#define SYS_STATUS_RXFCG_BIT_LEN             (1U)
#define SYS_STATUS_RXFCG_BIT_MASK            0x4000U
#define SYS_STATUS_RXFR_BIT_OFFSET           (13U)
#define SYS_STATUS_RXFR_BIT_LEN              (1U)
#define SYS_STATUS_RXFR_BIT_MASK             0x2000U
#define SYS_STATUS_RXPHE_BIT_OFFSET          (12U)
#define SYS_STATUS_RXPHE_BIT_LEN             (1U)
#define SYS_STATUS_RXPHE_BIT_MASK            0x1000U
#define SYS_STATUS_RXPHD_BIT_OFFSET          (11U)
#define SYS_STATUS_RXPHD_BIT_LEN             (1U)
#define SYS_STATUS_RXPHD_BIT_MASK            0x800U
#define SYS_STATUS_CIA_DONE_BIT_OFFSET       (10U)
#define SYS_STATUS_CIA_DONE_BIT_LEN          (1U)
#define SYS_STATUS_CIA_DONE_BIT_MASK         0x400U
#define SYS_STATUS_RXSFDD_BIT_OFFSET         (9U)
#define SYS_STATUS_RXSFDD_BIT_LEN            (1U)
#define SYS_STATUS_RXSFDD_BIT_MASK           0x200U
#define SYS_STATUS_RXPRD_BIT_OFFSET          (8U)
#define SYS_STATUS_RXPRD_BIT_LEN             (1U)
#define SYS_STATUS_RXPRD_BIT_MASK            0x100U
#define SYS_STATUS_TXFRS_BIT_OFFSET          (7U)
#define SYS_STATUS_TXFRS_BIT_LEN             (1U)
#define SYS_STATUS_TXFRS_BIT_MASK            0x80U
#define SYS_STATUS_TXPHS_BIT_OFFSET          (6U)
#define SYS_STATUS_TXPHS_BIT_LEN             (1U)
#define SYS_STATUS_TXPHS_BIT_MASK            0x40U
#define SYS_STATUS_TXPRS_BIT_OFFSET          (5U)
#define SYS_STATUS_TXPRS_BIT_LEN             (1U)
#define SYS_STATUS_TXPRS_BIT_MASK            0x20U
#define SYS_STATUS_TXFRB_BIT_OFFSET          (4U)
#define SYS_STATUS_TXFRB_BIT_LEN             (1U)
#define SYS_STATUS_TXFRB_BIT_MASK            0x10U
#define SYS_STATUS_AAT_BIT_OFFSET            (3U)
#define SYS_STATUS_AAT_BIT_LEN               (1U)
#define SYS_STATUS_AAT_BIT_MASK              0x8U
#define SYS_STATUS_SPICRCERR_BIT_OFFSET      (2U)
#define SYS_STATUS_SPICRCERR_BIT_LEN         (1U)
#define SYS_STATUS_SPICRCERR_BIT_MASK        0x4U
#define SYS_STATUS_CLK_PLL_LOCK_BIT_OFFSET   (1U)
#define SYS_STATUS_CLK_PLL_LOCK_BIT_LEN      (1U)
#define SYS_STATUS_CLK_PLL_LOCK_BIT_MASK     0x2U
#define SYS_STATUS_IRQS_BIT_OFFSET           (0U)
#define SYS_STATUS_IRQS_BIT_LEN              (1U)
#define SYS_STATUS_IRQS_BIT_MASK             0x1U

/******************************************************************************
* @brief Bit definitions for register SYS_STATUS_HI
**/
#define SYS_STATUS_HI_ID                     0x48
#define SYS_STATUS_HI_LEN                    (4U)
#define SYS_STATUS_HI_MASK                   0xFFFFFFFFUL
#define SYS_STATUS_HI_CCA_FAIL_BIT_OFFSET    (12U)
#define SYS_STATUS_HI_CCA_FAIL_BIT_LEN       (1U)
#define SYS_STATUS_HI_CCA_FAIL_BIT_MASK      0x1000U
#define SYS_STATUS_HI_SPI_COLLISION_BIT_OFFSET (11U)
#define SYS_STATUS_HI_SPI_COLLISION_BIT_LEN  (1U)
#define SYS_STATUS_HI_SPI_COLLISION_BIT_MASK 0x800U
#define SYS_STATUS_HI_SPI_UNF_BIT_OFFSET     (10U)
#define SYS_STATUS_HI_SPI_UNF_BIT_LEN        (1U)
#define SYS_STATUS_HI_SPI_UNF_BIT_MASK       0x400U
#define SYS_STATUS_HI_SPI_OVF_BIT_OFFSET     (9U)
#define SYS_STATUS_HI_SPI_OVF_BIT_LEN        (1U)
#define SYS_STATUS_HI_SPI_OVF_BIT_MASK       0x200U
#define SYS_STATUS_HI_CMD_ERR_BIT_OFFSET     (8U)
#define SYS_STATUS_HI_CMD_ERR_BIT_LEN        (1U)
#define SYS_STATUS_HI_CMD_ERR_BIT_MASK       0x100U
#define SYS_STATUS_HI_AES_ERR_BIT_OFFSET     (7U)
#define SYS_STATUS_HI_AES_ERR_BIT_LEN        (1U)
#define SYS_STATUS_HI_AES_ERR_BIT_MASK       0x80U
#define SYS_STATUS_HI_AES_DONE_BIT_OFFSET    (6U)
#define SYS_STATUS_HI_AES_DONE_BIT_LEN       (1U)
#define SYS_STATUS_HI_AES_DONE_BIT_MASK      0x40U
#define SYS_STATUS_HI_GPIO_BIT_OFFSET        (5U)
#define SYS_STATUS_HI_GPIO_BIT_LEN           (1U)
#define SYS_STATUS_HI_GPIO_BIT_MASK          0x20U
#define SYS_STATUS_HI_VT_DET_BIT_OFFSET      (4U)
#define SYS_STATUS_HI_VT_DET_BIT_LEN         (1U)
#define SYS_STATUS_HI_VT_DET_BIT_MASK        0x10U
#define SYS_STATUS_HI_ESYNC_RST_BIT_OFFSET   (3U)
#define SYS_STATUS_HI_ESYNC_RST_BIT_LEN      (1U)
#define SYS_STATUS_HI_ESYNC_RST_BIT_MASK     0x8U
#define SYS_STATUS_HI_RXPREJ_BIT_OFFSET      (1U)
#define SYS_STATUS_HI_RXPREJ_BIT_LEN         (1U)
#define SYS_STATUS_HI_RXPREJ_BIT_MASK        0x2U
#define SYS_STATUS_HI_RXSCE_BIT_OFFSET       (0U)
#define SYS_STATUS_HI_RXSCE_BIT_LEN          (1U)
#define SYS_STATUS_HI_RXSCE_BIT_MASK         0x1U

/******************************************************************************
* @brief Bit definitions for register RX_FINFO
**/
#define RX_FINFO_ID                          0x4c
#define RX_FINFO_LEN                         (4U)
#define RX_FINFO_MASK                        0xFFFFFFFFUL
#define RX_FINFO_RXP2SS_BIT_OFFSET           (20U)
#define RX_FINFO_RXP2SS_BIT_LEN              (12U)
#define RX_FINFO_RXP2SS_BIT_MASK             0xfff00000UL
#define RX_FINFO_RXPSR_BIT_OFFSET            (18U)
#define RX_FINFO_RXPSR_BIT_LEN               (2U)
#define RX_FINFO_RXPSR_BIT_MASK              0xc0000UL
#define RX_FINFO_RXPRF_BIT_OFFSET            (16U)
#define RX_FINFO_RXPRF_BIT_LEN               (2U)
#define RX_FINFO_RXPRF_BIT_MASK              0x30000UL
#define RX_FINFO_RNG_BIT_OFFSET              (15U)
#define RX_FINFO_RNG_BIT_LEN                 (1U)
#define RX_FINFO_RNG_BIT_MASK                0x8000U
#define RX_FINFO_RXBR_BIT_OFFSET             (13U)
#define RX_FINFO_RXBR_BIT_LEN                (1U)
#define RX_FINFO_RXBR_BIT_MASK               0x2000U
#define RX_FINFO_RXNSPL_BIT_OFFSET           (11U)
#define RX_FINFO_RXNSPL_BIT_LEN              (2U)
#define RX_FINFO_RXNSPL_BIT_MASK             0x1800U
#define RX_FINFO_RXFLEN_BIT_OFFSET           (0U)
#define RX_FINFO_RXFLEN_BIT_LEN              (10U)
#define RX_FINFO_RXFLEN_BIT_MASK             0x3ffU

/******************************************************************************
* @brief Bit definitions for register RX_FQUAL_LO
**/
#define RX_FQUAL_LO_ID                       0x50
#define RX_FQUAL_LO_LEN                      (4U)
#define RX_FQUAL_LO_MASK                     0xFFFFFFFFUL
#define RX_FQUAL_LO_RX_SNR_BIT_OFFSET        (16U)
#define RX_FQUAL_LO_RX_SNR_BIT_LEN           (16U)
#define RX_FQUAL_LO_RX_SNR_BIT_MASK          0xffff0000UL
#define RX_FQUAL_LO_RX_FOM_BIT_OFFSET        (0U)
#define RX_FQUAL_LO_RX_FOM_BIT_LEN           (16U)
#define RX_FQUAL_LO_RX_FOM_BIT_MASK          0xffffU

/******************************************************************************
* @brief Bit definitions for register RX_FQUAL_HI
**/
#define RX_FQUAL_HI_ID                       0x54
#define RX_FQUAL_HI_LEN                      (4U)
#define RX_FQUAL_HI_MASK                     0xFFFFFFFFUL
#define RX_FQUAL_HI_PERF_DIAG1_BIT_OFFSET    (16U)
#define RX_FQUAL_HI_PERF_DIAG1_BIT_LEN       (16U)
#define RX_FQUAL_HI_PERF_DIAG1_BIT_MASK      0xffff0000UL
#define RX_FQUAL_HI_PERF_DIAG0_BIT_OFFSET    (0U)
#define RX_FQUAL_HI_PERF_DIAG0_BIT_LEN       (16U)
#define RX_FQUAL_HI_PERF_DIAG0_BIT_MASK      0xffffU

/******************************************************************************
* @brief Bit definitions for register RX_TTCKO_LO
**/
#define RX_TTCKO_LO_ID                       0x5c
#define RX_TTCKO_LO_LEN                      (4U)
#define RX_TTCKO_LO_MASK                     0xFFFFFFFFUL
#define RX_TTCKO_LO_RNG_CAR_PHASE_BIT_OFFSET (0U)
#define RX_TTCKO_LO_RNG_CAR_PHASE_BIT_LEN    (7U)
#define RX_TTCKO_LO_RNG_CAR_PHASE_BIT_MASK   0x7fU

/******************************************************************************
* @brief Bit definitions for register RX_TTCKO_HI
**/
#define RX_TTCKO_HI_ID                       0x60
#define RX_TTCKO_HI_LEN                      (4U)
#define RX_TTCKO_HI_MASK                     0xFFFFFFFFUL
#define RX_TTCKO_HI_RSMP_DEL_BIT_OFFSET      (0U)
#define RX_TTCKO_HI_RSMP_DEL_BIT_LEN         (8U)
#define RX_TTCKO_HI_RSMP_DEL_BIT_MASK        0xffU

/******************************************************************************
* @brief Bit definitions for register RX_TIME_0
**/
#define RX_TIME_0_ID                         0x64
#define RX_TIME_0_LEN                        (4U)
#define RX_TIME_0_MASK                       0xFFFFFFFFUL
#define RX_TIME_0_RX_ADJUSTED_TIMESTAMP_LO_BIT_OFFSET (0U)
#define RX_TIME_0_RX_ADJUSTED_TIMESTAMP_LO_BIT_LEN (32U)
#define RX_TIME_0_RX_ADJUSTED_TIMESTAMP_LO_BIT_MASK 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register RX_TIME_1
**/
#define RX_TIME_1_ID                         0x68
#define RX_TIME_1_LEN                        (4U)
#define RX_TIME_1_MASK                       0xFFFFFFFFUL
#define RX_TIME_1_LE_RESULT1_LO_BIT_OFFSET   (24U)
#define RX_TIME_1_LE_RESULT1_LO_BIT_LEN      (8U)
#define RX_TIME_1_LE_RESULT1_LO_BIT_MASK     0xff000000UL
#define RX_TIME_1_LE_RESULT0_BIT_OFFSET      (8U)
#define RX_TIME_1_LE_RESULT0_BIT_LEN         (16U)
#define RX_TIME_1_LE_RESULT0_BIT_MASK        0xffff00UL
#define RX_TIME_1_RX_ADJUSTED_TIMESTAMP_HI_BIT_OFFSET (0U)
#define RX_TIME_1_RX_ADJUSTED_TIMESTAMP_HI_BIT_LEN (8U)
#define RX_TIME_1_RX_ADJUSTED_TIMESTAMP_HI_BIT_MASK 0xffU

/******************************************************************************
* @brief Bit definitions for register RX_TIME_2
**/
#define RX_TIME_2_ID                         0x6c
#define RX_TIME_2_LEN                        (4U)
#define RX_TIME_2_MASK                       0xFFFFFFFFUL
#define RX_TIME_2_LE_RESULT1_HI_BIT_OFFSET   (0U)
#define RX_TIME_2_LE_RESULT1_HI_BIT_LEN      (8U)
#define RX_TIME_2_LE_RESULT1_HI_BIT_MASK     0xffU

/******************************************************************************
* @brief Bit definitions for register RX_TIME_3
**/
#define RX_TIME_3_ID                         0x70
#define RX_TIME_3_LEN                        (4U)
#define RX_TIME_3_MASK                       0xFFFFFFFFUL
#define RX_TIME_3_RX_UNADJUSTED_TIMESTAMP_BIT_OFFSET (0U)
#define RX_TIME_3_RX_UNADJUSTED_TIMESTAMP_BIT_LEN (32U)
#define RX_TIME_3_RX_UNADJUSTED_TIMESTAMP_BIT_MASK 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register TX_TIME_LO
**/
#define TX_TIME_LO_ID                        0x74
#define TX_TIME_LO_LEN                       (4U)
#define TX_TIME_LO_MASK                      0xFFFFFFFFUL
#define TX_TIME_LO_TX_ADJUSTED_TIMESTAMP_LO_BIT_OFFSET (0U)
#define TX_TIME_LO_TX_ADJUSTED_TIMESTAMP_LO_BIT_LEN (32U)
#define TX_TIME_LO_TX_ADJUSTED_TIMESTAMP_LO_BIT_MASK 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register TX_TIME_HI
**/
#define TX_TIME_HI_ID                        0x78
#define TX_TIME_HI_LEN                       (4U)
#define TX_TIME_HI_MASK                      0xFFFFFFFFUL
#define TX_TIME_HI_TX_ADJUSTED_TIMESTAMP_HI_BIT_OFFSET (0U)
#define TX_TIME_HI_TX_ADJUSTED_TIMESTAMP_HI_BIT_LEN (8U)
#define TX_TIME_HI_TX_ADJUSTED_TIMESTAMP_HI_BIT_MASK 0xffU

/******************************************************************************
* @brief Bit definitions for register TX_TIME_2
**/
#define TX_TIME_2_ID                         0x10000
#define TX_TIME_2_LEN                        (4U)
#define TX_TIME_2_MASK                       0xFFFFFFFFUL
#define TX_TIME_2_TX_UNADJUSTED_TIMESTAMP_BIT_OFFSET (0U)
#define TX_TIME_2_TX_UNADJUSTED_TIMESTAMP_BIT_LEN (32U)
#define TX_TIME_2_TX_UNADJUSTED_TIMESTAMP_BIT_MASK 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register TX_ANTD
**/
#define TX_ANTD_ID                           0x10004
#define TX_ANTD_LEN                          (4U)
#define TX_ANTD_MASK                         0xFFFFFFFFUL
#define TX_ANTD_ANTD_BIT_OFFSET              (0U)
#define TX_ANTD_ANTD_BIT_LEN                 (16U)
#define TX_ANTD_ANTD_BIT_MASK                0xffffU

/******************************************************************************
* @brief Bit definitions for register ACK_RESP
**/
#define ACK_RESP_ID                          0x10008
#define ACK_RESP_LEN                         (4U)
#define ACK_RESP_MASK                        0xFFFFFFFFUL
#define ACK_RESP_ACK_TIM_BIT_OFFSET          (24U)
#define ACK_RESP_ACK_TIM_BIT_LEN             (8U)
#define ACK_RESP_ACK_TIM_BIT_MASK            0xff000000UL
#define ACK_RESP_WAIT4RESP_TIM_BIT_OFFSET    (0U)
#define ACK_RESP_WAIT4RESP_TIM_BIT_LEN       (20U)
#define ACK_RESP_WAIT4RESP_TIM_BIT_MASK      0xfffffUL

/******************************************************************************
* @brief Bit definitions for register TX_POWER
**/
#define TX_POWER_ID                          0x1000c
#define TX_POWER_LEN                         (4U)
#define TX_POWER_MASK                        0xFFFFFFFFUL
#define TX_POWER_TX_CP_PWR_BIT_OFFSET        (24U)
#define TX_POWER_TX_CP_PWR_BIT_LEN           (8U)
#define TX_POWER_TX_CP_PWR_BIT_MASK          0xff000000UL
#define TX_POWER_TX_SHR_PWR_BIT_OFFSET       (16U)
#define TX_POWER_TX_SHR_PWR_BIT_LEN          (8U)
#define TX_POWER_TX_SHR_PWR_BIT_MASK         0xff0000UL
#define TX_POWER_TX_PHR_PWR_BIT_OFFSET       (8U)
#define TX_POWER_TX_PHR_PWR_BIT_LEN          (8U)
#define TX_POWER_TX_PHR_PWR_BIT_MASK         0xff00U
#define TX_POWER_TX_DATA_PWR_BIT_OFFSET      (0U)
#define TX_POWER_TX_DATA_PWR_BIT_LEN         (8U)
#define TX_POWER_TX_DATA_PWR_BIT_MASK        0xffU

/******************************************************************************
* @brief Bit definitions for register CHAN_CTRL
**/
#define CHAN_CTRL_ID                         0x10014
#define CHAN_CTRL_LEN                        (4U)
#define CHAN_CTRL_MASK                       0xFFFFFFFFUL
#define CHAN_CTRL_RX_PCODE_BIT_OFFSET        (8U)
#define CHAN_CTRL_RX_PCODE_BIT_LEN           (5U)
#define CHAN_CTRL_RX_PCODE_BIT_MASK          0x1f00U
#define CHAN_CTRL_TX_PCODE_BIT_OFFSET        (3U)
#define CHAN_CTRL_TX_PCODE_BIT_LEN           (5U)
#define CHAN_CTRL_TX_PCODE_BIT_MASK          0xf8U
#define CHAN_CTRL_SFD_TYPE_BIT_OFFSET        (1U)
#define CHAN_CTRL_SFD_TYPE_BIT_LEN           (2U)
#define CHAN_CTRL_SFD_TYPE_BIT_MASK          0x6U
#define CHAN_CTRL_RF_CHAN_BIT_OFFSET         (0U)
#define CHAN_CTRL_RF_CHAN_BIT_LEN            (1U)
#define CHAN_CTRL_RF_CHAN_BIT_MASK           0x1U

/******************************************************************************
* @brief Bit definitions for register LE_PEND_01
**/
#define LE_PEND_01_ID                        0x10018
#define LE_PEND_01_LEN                       (4U)
#define LE_PEND_01_MASK                      0xFFFFFFFFUL
#define LE_PEND_01_LE_ADDR1_BIT_OFFSET       (16U)
#define LE_PEND_01_LE_ADDR1_BIT_LEN          (16U)
#define LE_PEND_01_LE_ADDR1_BIT_MASK         0xffff0000UL
#define LE_PEND_01_LE_ADDR0_BIT_OFFSET       (0U)
#define LE_PEND_01_LE_ADDR0_BIT_LEN          (16U)
#define LE_PEND_01_LE_ADDR0_BIT_MASK         0xffffU

/******************************************************************************
* @brief Bit definitions for register LE_PEND_23
**/
#define LE_PEND_23_ID                        0x1001c
#define LE_PEND_23_LEN                       (4U)
#define LE_PEND_23_MASK                      0xFFFFFFFFUL
#define LE_PEND_23_LE_ADDR3_BIT_OFFSET       (16U)
#define LE_PEND_23_LE_ADDR3_BIT_LEN          (16U)
#define LE_PEND_23_LE_ADDR3_BIT_MASK         0xffff0000UL
#define LE_PEND_23_LE_ADDR2_BIT_OFFSET       (0U)
#define LE_PEND_23_LE_ADDR2_BIT_LEN          (16U)
#define LE_PEND_23_LE_ADDR2_BIT_MASK         0xffffU

/******************************************************************************
* @brief Bit definitions for register SPI_COLLISION_STATUS
**/
#define SPI_COLLISION_STATUS_ID              0x10020
#define SPI_COLLISION_STATUS_LEN             (4U)
#define SPI_COLLISION_STATUS_MASK            0xFFFFFFFFUL
#define SPI_COLLISION_STATUS_SPI_COLLISION_STATUS_BIT_OFFSET (0U)
#define SPI_COLLISION_STATUS_SPI_COLLISION_STATUS_BIT_LEN (5U)
#define SPI_COLLISION_STATUS_SPI_COLLISION_STATUS_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register RDB_STATUS
**/
#define RDB_STATUS_ID                        0x10024
#define RDB_STATUS_LEN                       (4U)
#define RDB_STATUS_MASK                      0xFFFFFFFFUL
#define RDB_STATUS_CP_ERR1_BIT_OFFSET        (7U)
#define RDB_STATUS_CP_ERR1_BIT_LEN           (1U)
#define RDB_STATUS_CP_ERR1_BIT_MASK          0x80U
#define RDB_STATUS_CIADONE1_BIT_OFFSET       (6U)
#define RDB_STATUS_CIADONE1_BIT_LEN          (1U)
#define RDB_STATUS_CIADONE1_BIT_MASK         0x40U
#define RDB_STATUS_RXFR1_BIT_OFFSET          (5U)
#define RDB_STATUS_RXFR1_BIT_LEN             (1U)
#define RDB_STATUS_RXFR1_BIT_MASK            0x20U
#define RDB_STATUS_RXFCG1_BIT_OFFSET         (4U)
#define RDB_STATUS_RXFCG1_BIT_LEN            (1U)
#define RDB_STATUS_RXFCG1_BIT_MASK           0x10U
#define RDB_STATUS_CP_ERR0_BIT_OFFSET        (3U)
#define RDB_STATUS_CP_ERR0_BIT_LEN           (1U)
#define RDB_STATUS_CP_ERR0_BIT_MASK          0x8U
#define RDB_STATUS_CIADONE0_BIT_OFFSET       (2U)
#define RDB_STATUS_CIADONE0_BIT_LEN          (1U)
#define RDB_STATUS_CIADONE0_BIT_MASK         0x4U
#define RDB_STATUS_RXFR0_BIT_OFFSET          (1U)
#define RDB_STATUS_RXFR0_BIT_LEN             (1U)
#define RDB_STATUS_RXFR0_BIT_MASK            0x2U
#define RDB_STATUS_RXFCG0_BIT_OFFSET         (0U)
#define RDB_STATUS_RXFCG0_BIT_LEN            (1U)
#define RDB_STATUS_RXFCG0_BIT_MASK           0x1U

/******************************************************************************
* @brief Bit definitions for register RDB_DIAG_MODE
**/
#define RDB_DIAG_MODE_ID                     0x10028
#define RDB_DIAG_MODE_LEN                    (4U)
#define RDB_DIAG_MODE_MASK                   0xFFFFFFFFUL
#define RDB_DIAG_MODE_RDB_DIAG_MODE_BIT_OFFSET (0U)
#define RDB_DIAG_MODE_RDB_DIAG_MODE_BIT_LEN  (3U)
#define RDB_DIAG_MODE_RDB_DIAG_MODE_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register REGMAP_VER
**/
#define REGMAP_VER_ID                        0x1002c
#define REGMAP_VER_LEN                       (4U)
#define REGMAP_VER_MASK                      0xFFFFFFFFUL
#define REGMAP_VER_REGMAP_VER_BIT_OFFSET     (0U)
#define REGMAP_VER_REGMAP_VER_BIT_LEN        (32U)
#define REGMAP_VER_REGMAP_VER_BIT_MASK       0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register AES_CFG
**/
#define AES_CFG_ID                           0x10030
#define AES_CFG_LEN                          (4U)
#define AES_CFG_MASK                         0xFFFFFFFFUL
#define AES_CFG_AES_KEY_OTP_BIT_OFFSET       (12U)
#define AES_CFG_AES_KEY_OTP_BIT_LEN          (1U)
#define AES_CFG_AES_KEY_OTP_BIT_MASK         0x1000U
#define AES_CFG_AES_CORE_SEL_BIT_OFFSET      (11U)
#define AES_CFG_AES_CORE_SEL_BIT_LEN         (1U)
#define AES_CFG_AES_CORE_SEL_BIT_MASK        0x800U
#define AES_CFG_AES_TAG_SIZE_BIT_OFFSET      (8U)
#define AES_CFG_AES_TAG_SIZE_BIT_LEN         (3U)
#define AES_CFG_AES_TAG_SIZE_BIT_MASK        0x700U
#define AES_CFG_AES_KEY_SRC_BIT_OFFSET       (7U)
#define AES_CFG_AES_KEY_SRC_BIT_LEN          (1U)
#define AES_CFG_AES_KEY_SRC_BIT_MASK         0x80U
#define AES_CFG_AES_KEY_LOAD_BIT_OFFSET      (6U)
#define AES_CFG_AES_KEY_LOAD_BIT_LEN         (1U)
#define AES_CFG_AES_KEY_LOAD_BIT_MASK        0x40U
#define AES_CFG_AES_KEY_ADDR_BIT_OFFSET      (3U)
#define AES_CFG_AES_KEY_ADDR_BIT_LEN         (3U)
#define AES_CFG_AES_KEY_ADDR_BIT_MASK        0x38U
#define AES_CFG_AES_KEY_SIZE_BIT_OFFSET      (1U)
#define AES_CFG_AES_KEY_SIZE_BIT_LEN         (2U)
#define AES_CFG_AES_KEY_SIZE_BIT_MASK        0x6U
#define AES_CFG_AES_MODE_BIT_OFFSET          (0U)
#define AES_CFG_AES_MODE_BIT_LEN             (1U)
#define AES_CFG_AES_MODE_BIT_MASK            0x1U

/******************************************************************************
* @brief Bit definitions for register AES_IV0
**/
#define AES_IV0_ID                           0x10034
#define AES_IV0_LEN                          (4U)
#define AES_IV0_MASK                         0xFFFFFFFFUL
#define AES_IV0_IV0_BIT_OFFSET               (0U)
#define AES_IV0_IV0_BIT_LEN                  (32U)
#define AES_IV0_IV0_BIT_MASK                 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register AES_IV1
**/
#define AES_IV1_ID                           0x10038
#define AES_IV1_LEN                          (4U)
#define AES_IV1_MASK                         0xFFFFFFFFUL
#define AES_IV1_IV1_BIT_OFFSET               (0U)
#define AES_IV1_IV1_BIT_LEN                  (32U)
#define AES_IV1_IV1_BIT_MASK                 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register AES_IV2
**/
#define AES_IV2_ID                           0x1003c
#define AES_IV2_LEN                          (4U)
#define AES_IV2_MASK                         0xFFFFFFFFUL
#define AES_IV2_IV2_BIT_OFFSET               (0U)
#define AES_IV2_IV2_BIT_LEN                  (32U)
#define AES_IV2_IV2_BIT_MASK                 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register AES_IV3
**/
#define AES_IV3_ID                           0x10040
#define AES_IV3_LEN                          (4U)
#define AES_IV3_MASK                         0xFFFFFFFFUL
#define AES_IV3_IV3_BIT_OFFSET               (0U)
#define AES_IV3_IV3_BIT_LEN                  (32U)
#define AES_IV3_IV3_BIT_MASK                 0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register DMA_CFG0
**/
#define DMA_CFG0_ID                          0x10044
#define DMA_CFG0_LEN                         (4U)
#define DMA_CFG0_MASK                        0xFFFFFFFFUL
#define DMA_CFG0_DMA_CP_END_SEL_BIT_OFFSET   (26U)
#define DMA_CFG0_DMA_CP_END_SEL_BIT_LEN      (1U)
#define DMA_CFG0_DMA_CP_END_SEL_BIT_MASK     0x4000000UL
#define DMA_CFG0_DMA_DST_ADDR_BIT_OFFSET     (16U)
#define DMA_CFG0_DMA_DST_ADDR_BIT_LEN        (10U)
#define DMA_CFG0_DMA_DST_ADDR_BIT_MASK       0x3ff0000UL
#define DMA_CFG0_DMA_DST_PORT_BIT_OFFSET     (13U)
#define DMA_CFG0_DMA_DST_PORT_BIT_LEN        (3U)
#define DMA_CFG0_DMA_DST_PORT_BIT_MASK       0xe000U
#define DMA_CFG0_DMA_SRC_ADDR_BIT_OFFSET     (3U)
#define DMA_CFG0_DMA_SRC_ADDR_BIT_LEN        (10U)
#define DMA_CFG0_DMA_SRC_ADDR_BIT_MASK       0x1ff8U
#define DMA_CFG0_DMA_SRC_PORT_BIT_OFFSET     (0U)
#define DMA_CFG0_DMA_SRC_PORT_BIT_LEN        (3U)
#define DMA_CFG0_DMA_SRC_PORT_BIT_MASK       0x7U

/******************************************************************************
* @brief Bit definitions for register DMA_CFG1
**/
#define DMA_CFG1_ID                          0x10048
#define DMA_CFG1_LEN                         (4U)
#define DMA_CFG1_MASK                        0xFFFFFFFFUL
#define DMA_CFG1_DMA_PYLD_SIZE_BIT_OFFSET    (7U)
#define DMA_CFG1_DMA_PYLD_SIZE_BIT_LEN       (10U)
#define DMA_CFG1_DMA_PYLD_SIZE_BIT_MASK      0x1ff80UL
#define DMA_CFG1_DMA_HDR_SIZE_BIT_OFFSET     (0U)
#define DMA_CFG1_DMA_HDR_SIZE_BIT_LEN        (7U)
#define DMA_CFG1_DMA_HDR_SIZE_BIT_MASK       0x7fU

/******************************************************************************
* @brief Bit definitions for register AES_START
**/
#define AES_START_ID                         0x1004c
#define AES_START_LEN                        (4U)
#define AES_START_MASK                       0xFFFFFFFFUL
#define AES_START_AES_START_BIT_OFFSET       (0U)
#define AES_START_AES_START_BIT_LEN          (1U)
#define AES_START_AES_START_BIT_MASK         0x1U

/******************************************************************************
* @brief Bit definitions for register AES_STS
**/
#define AES_STS_ID                           0x10050
#define AES_STS_LEN                          (4U)
#define AES_STS_MASK                         0xFFFFFFFFUL
#define AES_STS_AES_SCRATCH_RAM_FULL_BIT_OFFSET (5U)
#define AES_STS_AES_SCRATCH_RAM_FULL_BIT_LEN (1U)
#define AES_STS_AES_SCRATCH_RAM_FULL_BIT_MASK 0x20U
#define AES_STS_AES_SCRATCH_RAM_EMPTY_BIT_OFFSET (4U)
#define AES_STS_AES_SCRATCH_RAM_EMPTY_BIT_LEN (1U)
#define AES_STS_AES_SCRATCH_RAM_EMPTY_BIT_MASK 0x10U
#define AES_STS_MEM_ACC_CONF_BIT_OFFSET      (3U)
#define AES_STS_MEM_ACC_CONF_BIT_LEN         (1U)
#define AES_STS_MEM_ACC_CONF_BIT_MASK        0x8U
#define AES_STS_DMA_TRANS_ERR_BIT_OFFSET     (2U)
#define AES_STS_DMA_TRANS_ERR_BIT_LEN        (1U)
#define AES_STS_DMA_TRANS_ERR_BIT_MASK       0x4U
#define AES_STS_AES_AUTH_ERR_BIT_OFFSET      (1U)
#define AES_STS_AES_AUTH_ERR_BIT_LEN         (1U)
#define AES_STS_AES_AUTH_ERR_BIT_MASK        0x2U
#define AES_STS_AES_DONE_BIT_OFFSET          (0U)
#define AES_STS_AES_DONE_BIT_LEN             (1U)
#define AES_STS_AES_DONE_BIT_MASK            0x1U

/******************************************************************************
* @brief Bit definitions for register AES_KEY0
**/
#define AES_KEY0_ID                          0x10054
#define AES_KEY0_LEN                         (4U)
#define AES_KEY0_MASK                        0xFFFFFFFFUL
#define AES_KEY0_KEY0_BIT_OFFSET             (0U)
#define AES_KEY0_KEY0_BIT_LEN                (32U)
#define AES_KEY0_KEY0_BIT_MASK               0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register AES_KEY1
**/
#define AES_KEY1_ID                          0x10058
#define AES_KEY1_LEN                         (4U)
#define AES_KEY1_MASK                        0xFFFFFFFFUL
#define AES_KEY1_KEY1_BIT_OFFSET             (0U)
#define AES_KEY1_KEY1_BIT_LEN                (32U)
#define AES_KEY1_KEY1_BIT_MASK               0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register AES_KEY2
**/
#define AES_KEY2_ID                          0x1005c
#define AES_KEY2_LEN                         (4U)
#define AES_KEY2_MASK                        0xFFFFFFFFUL
#define AES_KEY2_KEY2_BIT_OFFSET             (0U)
#define AES_KEY2_KEY2_BIT_LEN                (32U)
#define AES_KEY2_KEY2_BIT_MASK               0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register AES_KEY3
**/
#define AES_KEY3_ID                          0x10060
#define AES_KEY3_LEN                         (4U)
#define AES_KEY3_MASK                        0xFFFFFFFFUL
#define AES_KEY3_KEY3_BIT_OFFSET             (0U)
#define AES_KEY3_KEY3_BIT_LEN                (32U)
#define AES_KEY3_KEY3_BIT_MASK               0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_CFG0
**/
#define CP_CFG0_ID                           0x20000
#define CP_CFG0_LEN                          (4U)
#define CP_CFG0_MASK                         0xFFFFFFFFUL
#define CP_CFG0_CP_USER_MASK_BIT_OFFSET      (16U)
#define CP_CFG0_CP_USER_MASK_BIT_LEN         (1U)
#define CP_CFG0_CP_USER_MASK_BIT_MASK        0x10000UL
#define CP_CFG0_CP_COUNTER_MODE_BIT_OFFSET   (14U)
#define CP_CFG0_CP_COUNTER_MODE_BIT_LEN      (2U)
#define CP_CFG0_CP_COUNTER_MODE_BIT_MASK     0xc000U
#define CP_CFG0_CP_CIR_SHFT_BIT_OFFSET       (13U)
#define CP_CFG0_CP_CIR_SHFT_BIT_LEN          (1U)
#define CP_CFG0_CP_CIR_SHFT_BIT_MASK         0x2000U
#define CP_CFG0_CP_LCSS_BTF_BIT_OFFSET       (8U)
#define CP_CFG0_CP_LCSS_BTF_BIT_LEN          (5U)
#define CP_CFG0_CP_LCSS_BTF_BIT_MASK         0x1f00U
#define CP_CFG0_CP_BLOCKS_BIT_OFFSET         (0U)
#define CP_CFG0_CP_BLOCKS_BIT_LEN            (8U)
#define CP_CFG0_CP_BLOCKS_BIT_MASK           0xffU

/******************************************************************************
* @brief Bit definitions for register CP_CTRL
**/
#define CP_CTRL_ID                           0x20004
#define CP_CTRL_LEN                          (4U)
#define CP_CTRL_MASK                         0xFFFFFFFFUL
#define CP_CTRL_CP_RESTART_FROM_LAST_BIT_OFFSET (1U)
#define CP_CTRL_CP_RESTART_FROM_LAST_BIT_LEN (1U)
#define CP_CTRL_CP_RESTART_FROM_LAST_BIT_MASK 0x2U
#define CP_CTRL_CP_LOAD_IV_BIT_OFFSET        (0U)
#define CP_CTRL_CP_LOAD_IV_BIT_LEN           (1U)
#define CP_CTRL_CP_LOAD_IV_BIT_MASK          0x1U

/******************************************************************************
* @brief Bit definitions for register CP_STS
**/
#define CP_STS_ID                            0x20008
#define CP_STS_LEN                           (4U)
#define CP_STS_MASK                          0xFFFFFFFFUL
#define CP_STS_ACC_QUAL_BIT_OFFSET           (0U)
#define CP_STS_ACC_QUAL_BIT_LEN              (12U)
#define CP_STS_ACC_QUAL_BIT_MASK             0xfffU

/******************************************************************************
* @brief Bit definitions for register CP_KEY0
**/
#define CP_KEY0_ID                           0x2000c
#define CP_KEY0_LEN                          (4U)
#define CP_KEY0_MASK                         0xFFFFFFFFUL
#define CP_KEY0_KEY0_BIT_OFFSET              (0U)
#define CP_KEY0_KEY0_BIT_LEN                 (32U)
#define CP_KEY0_KEY0_BIT_MASK                0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_KEY1
**/
#define CP_KEY1_ID                           0x20010
#define CP_KEY1_LEN                          (4U)
#define CP_KEY1_MASK                         0xFFFFFFFFUL
#define CP_KEY1_KEY1_BIT_OFFSET              (0U)
#define CP_KEY1_KEY1_BIT_LEN                 (32U)
#define CP_KEY1_KEY1_BIT_MASK                0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_KEY2
**/
#define CP_KEY2_ID                           0x20014
#define CP_KEY2_LEN                          (4U)
#define CP_KEY2_MASK                         0xFFFFFFFFUL
#define CP_KEY2_KEY2_BIT_OFFSET              (0U)
#define CP_KEY2_KEY2_BIT_LEN                 (32U)
#define CP_KEY2_KEY2_BIT_MASK                0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_KEY3
**/
#define CP_KEY3_ID                           0x20018
#define CP_KEY3_LEN                          (4U)
#define CP_KEY3_MASK                         0xFFFFFFFFUL
#define CP_KEY3_KEY3_BIT_OFFSET              (0U)
#define CP_KEY3_KEY3_BIT_LEN                 (32U)
#define CP_KEY3_KEY3_BIT_MASK                0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_IV0
**/
#define CP_IV0_ID                            0x2001c
#define CP_IV0_LEN                           (4U)
#define CP_IV0_MASK                          0xFFFFFFFFUL
#define CP_IV0_IV0_BIT_OFFSET                (0U)
#define CP_IV0_IV0_BIT_LEN                   (32U)
#define CP_IV0_IV0_BIT_MASK                  0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_IV1
**/
#define CP_IV1_ID                            0x20020
#define CP_IV1_LEN                           (4U)
#define CP_IV1_MASK                          0xFFFFFFFFUL
#define CP_IV1_IV1_BIT_OFFSET                (0U)
#define CP_IV1_IV1_BIT_LEN                   (32U)
#define CP_IV1_IV1_BIT_MASK                  0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_IV2
**/
#define CP_IV2_ID                            0x20024
#define CP_IV2_LEN                           (4U)
#define CP_IV2_MASK                          0xFFFFFFFFUL
#define CP_IV2_IV2_BIT_OFFSET                (0U)
#define CP_IV2_IV2_BIT_LEN                   (32U)
#define CP_IV2_IV2_BIT_MASK                  0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CP_IV3
**/
#define CP_IV3_ID                            0x20028
#define CP_IV3_LEN                           (4U)
#define CP_IV3_MASK                          0xFFFFFFFFUL
#define CP_IV3_IV3_BIT_OFFSET                (0U)
#define CP_IV3_IV3_BIT_LEN                   (32U)
#define CP_IV3_IV3_BIT_MASK                  0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register USER_MASK0
**/
#define USER_MASK0_ID                        0x2002c
#define USER_MASK0_LEN                       (4U)
#define USER_MASK0_MASK                      0xFFFFFFFFUL
#define USER_MASK0_UMASK0L_BIT_OFFSET        (0U)
#define USER_MASK0_UMASK0L_BIT_LEN           (32U)
#define USER_MASK0_UMASK0L_BIT_MASK          0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register USER_MASK1
**/
#define USER_MASK1_ID                        0x20030
#define USER_MASK1_LEN                       (4U)
#define USER_MASK1_MASK                      0xFFFFFFFFUL
#define USER_MASK1_UMASK0M_BIT_OFFSET        (0U)
#define USER_MASK1_UMASK0M_BIT_LEN           (32U)
#define USER_MASK1_UMASK0M_BIT_MASK          0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register LCSS_MARGIN
**/
#define LCSS_MARGIN_ID                       0x20034
#define LCSS_MARGIN_LEN                      (4U)
#define LCSS_MARGIN_MASK                     0xFFFFFFFFUL
#define LCSS_MARGIN_PRF64_BIT_OFFSET         (0U)
#define LCSS_MARGIN_PRF64_BIT_LEN            (6U)
#define LCSS_MARGIN_PRF64_BIT_MASK           0x3fU

/******************************************************************************
* @brief Bit definitions for register MRX
**/
#define MRX_ID                               0x30000
#define MRX_LEN                              (4U)
#define MRX_MASK                             0xFFFFFFFFUL
#define MRX_CFG_PGF_0_BIT_OFFSET             (8U)
#define MRX_CFG_PGF_0_BIT_LEN                (5U)
#define MRX_CFG_PGF_0_BIT_MASK               0x1f00U
#define MRX_CFG_SWAP_IQ_BIT_OFFSET           (4U)
#define MRX_CFG_SWAP_IQ_BIT_LEN              (1U)
#define MRX_CFG_SWAP_IQ_BIT_MASK             0x10U
#define MRX_CFG_ADC_NEGEDGE_BIT_OFFSET       (2U)
#define MRX_CFG_ADC_NEGEDGE_BIT_LEN          (1U)
#define MRX_CFG_ADC_NEGEDGE_BIT_MASK         0x4U
#define MRX_CFG_ADC_SAMPLE_REVERSE_BIT_OFFSET (1U)
#define MRX_CFG_ADC_SAMPLE_REVERSE_BIT_LEN   (1U)
#define MRX_CFG_ADC_SAMPLE_REVERSE_BIT_MASK  0x2U
#define MRX_CFG_FREEZE_DIAGNOSTICS_BIT_OFFSET (0U)
#define MRX_CFG_FREEZE_DIAGNOSTICS_BIT_LEN   (1U)
#define MRX_CFG_FREEZE_DIAGNOSTICS_BIT_MASK  0x1U

/******************************************************************************
* @brief Bit definitions for register MRX_PGF_LUT
**/
#define MRX_PGF_LUT_ID                       0x30004
#define MRX_PGF_LUT_LEN                      (4U)
#define MRX_PGF_LUT_MASK                     0xFFFFFFFFUL
#define MRX_PGF_LUT_CFG_PGF_6_BIT_OFFSET     (25U)
#define MRX_PGF_LUT_CFG_PGF_6_BIT_LEN        (5U)
#define MRX_PGF_LUT_CFG_PGF_6_BIT_MASK       0x3e000000UL
#define MRX_PGF_LUT_CFG_PGF_5_BIT_OFFSET     (20U)
#define MRX_PGF_LUT_CFG_PGF_5_BIT_LEN        (5U)
#define MRX_PGF_LUT_CFG_PGF_5_BIT_MASK       0x1f00000UL
#define MRX_PGF_LUT_CFG_PGF_4_BIT_OFFSET     (15U)
#define MRX_PGF_LUT_CFG_PGF_4_BIT_LEN        (5U)
#define MRX_PGF_LUT_CFG_PGF_4_BIT_MASK       0xf8000UL
#define MRX_PGF_LUT_CFG_PGF_3_BIT_OFFSET     (10U)
#define MRX_PGF_LUT_CFG_PGF_3_BIT_LEN        (5U)
#define MRX_PGF_LUT_CFG_PGF_3_BIT_MASK       0x7c00U
#define MRX_PGF_LUT_CFG_PGF_2_BIT_OFFSET     (5U)
#define MRX_PGF_LUT_CFG_PGF_2_BIT_LEN        (5U)
#define MRX_PGF_LUT_CFG_PGF_2_BIT_MASK       0x3e0U
#define MRX_PGF_LUT_CFG_PGF_1_BIT_OFFSET     (0U)
#define MRX_PGF_LUT_CFG_PGF_1_BIT_LEN        (5U)
#define MRX_PGF_LUT_CFG_PGF_1_BIT_MASK       0x1fU

/******************************************************************************
* @brief Bit definitions for register ADC_CFG
**/
#define ADC_CFG_ID                           0x30008
#define ADC_CFG_LEN                          (4U)
#define ADC_CFG_MASK                         0xFFFFFFFFUL
#define ADC_CFG_CFG_ADC_TARGET_64_BIT_OFFSET (22U)
#define ADC_CFG_CFG_ADC_TARGET_64_BIT_LEN    (9U)
#define ADC_CFG_CFG_ADC_TARGET_64_BIT_MASK   0x7fc00000UL
#define ADC_CFG_CFG_ADC_TARGET_16_BIT_OFFSET (13U)
#define ADC_CFG_CFG_ADC_TARGET_16_BIT_LEN    (9U)
#define ADC_CFG_CFG_ADC_TARGET_16_BIT_MASK   0x3fe000UL
#define ADC_CFG_CFG_ADC_KI_FAST_BIT_OFFSET   (11U)
#define ADC_CFG_CFG_ADC_KI_FAST_BIT_LEN      (2U)
#define ADC_CFG_CFG_ADC_KI_FAST_BIT_MASK     0x1800U
#define ADC_CFG_CFG_ADC_KP_FAST_BIT_OFFSET   (9U)
#define ADC_CFG_CFG_ADC_KP_FAST_BIT_LEN      (2U)
#define ADC_CFG_CFG_ADC_KP_FAST_BIT_MASK     0x600U
#define ADC_CFG_CFG_ADC_KI_BIT_OFFSET        (7U)
#define ADC_CFG_CFG_ADC_KI_BIT_LEN           (2U)
#define ADC_CFG_CFG_ADC_KI_BIT_MASK          0x180U
#define ADC_CFG_CFG_ADC_KP_BIT_OFFSET        (5U)
#define ADC_CFG_CFG_ADC_KP_BIT_LEN           (2U)
#define ADC_CFG_CFG_ADC_KP_BIT_MASK          0x60U
#define ADC_CFG_SILENCE_BIT_OFFSET           (3U)
#define ADC_CFG_SILENCE_BIT_LEN              (2U)
#define ADC_CFG_SILENCE_BIT_MASK             0x18U
#define ADC_CFG_OVERRIDE_BIT_OFFSET          (2U)
#define ADC_CFG_OVERRIDE_BIT_LEN             (1U)
#define ADC_CFG_OVERRIDE_BIT_MASK            0x4U
#define ADC_CFG_CFG_ADC_CYPHEROFF_BIT_OFFSET (1U)
#define ADC_CFG_CFG_ADC_CYPHEROFF_BIT_LEN    (1U)
#define ADC_CFG_CFG_ADC_CYPHEROFF_BIT_MASK   0x2U
#define ADC_CFG_CFG_ADC_DEMODOFF_BIT_OFFSET  (0U)
#define ADC_CFG_CFG_ADC_DEMODOFF_BIT_LEN     (1U)
#define ADC_CFG_CFG_ADC_DEMODOFF_BIT_MASK    0x1U

/******************************************************************************
* @brief Bit definitions for register ADC_ZERO_THRESH_CFG
**/
#define ADC_ZERO_THRESH_CFG_ID               0x3000c
#define ADC_ZERO_THRESH_CFG_LEN              (4U)
#define ADC_ZERO_THRESH_CFG_MASK             0xFFFFFFFFUL
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_Q_NEG_BIT_OFFSET (24U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_Q_NEG_BIT_LEN (8U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_Q_NEG_BIT_MASK 0xff000000UL
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_Q_POS_BIT_OFFSET (16U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_Q_POS_BIT_LEN (8U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_Q_POS_BIT_MASK 0xff0000UL
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_I_NEG_BIT_OFFSET (8U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_I_NEG_BIT_LEN (8U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_I_NEG_BIT_MASK 0xff00U
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_I_POS_BIT_OFFSET (0U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_I_POS_BIT_LEN (8U)
#define ADC_ZERO_THRESH_CFG_CFG_ZEROTH_THRESH_I_POS_BIT_MASK 0xffU

/******************************************************************************
* @brief Bit definitions for register ADC_THRESH_CFG
**/
#define ADC_THRESH_CFG_ID                    0x30010
#define ADC_THRESH_CFG_LEN                   (4U)
#define ADC_THRESH_CFG_MASK                  0xFFFFFFFFUL
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_Q_NEG_BIT_OFFSET (24U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_Q_NEG_BIT_LEN (8U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_Q_NEG_BIT_MASK 0xff000000UL
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_Q_POS_BIT_OFFSET (16U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_Q_POS_BIT_LEN (8U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_Q_POS_BIT_MASK 0xff0000UL
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_I_NEG_BIT_OFFSET (8U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_I_NEG_BIT_LEN (8U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_I_NEG_BIT_MASK 0xff00U
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_I_POS_BIT_OFFSET (0U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_I_POS_BIT_LEN (8U)
#define ADC_THRESH_CFG_CFG_OVRD_ADC_THRESH_I_POS_BIT_MASK 0xffU

/******************************************************************************
* @brief Bit definitions for register AGC_CFG
**/
#define AGC_CFG_ID                           0x30014
#define AGC_CFG_LEN                          (4U)
#define AGC_CFG_MASK                         0xFFFFFFFFUL
#define AGC_CFG_CFG_AGC_MAX_THR_DIFF_MODE_BIT_OFFSET (23U)
#define AGC_CFG_CFG_AGC_MAX_THR_DIFF_MODE_BIT_LEN (1U)
#define AGC_CFG_CFG_AGC_MAX_THR_DIFF_MODE_BIT_MASK 0x800000UL
#define AGC_CFG_DELAY_BIT_OFFSET             (21U)
#define AGC_CFG_DELAY_BIT_LEN                (2U)
#define AGC_CFG_DELAY_BIT_MASK               0x600000UL
#define AGC_CFG_TARGET_BIT_OFFSET            (12U)
#define AGC_CFG_TARGET_BIT_LEN               (9U)
#define AGC_CFG_TARGET_BIT_MASK              0x1ff000UL
#define AGC_CFG_CFG_AGC_SCALING_BIT_OFFSET   (11U)
#define AGC_CFG_CFG_AGC_SCALING_BIT_LEN      (1U)
#define AGC_CFG_CFG_AGC_SCALING_BIT_MASK     0x800U
#define AGC_CFG_CFG_AGC_KI_BIT_OFFSET        (9U)
#define AGC_CFG_CFG_AGC_KI_BIT_LEN           (2U)
#define AGC_CFG_CFG_AGC_KI_BIT_MASK          0x600U
#define AGC_CFG_KP_BIT_OFFSET                (7U)
#define AGC_CFG_KP_BIT_LEN                   (2U)
#define AGC_CFG_KP_BIT_MASK                  0x180U
#define AGC_CFG_OVERRIDE_BIT_OFFSET          (6U)
#define AGC_CFG_OVERRIDE_BIT_LEN             (1U)
#define AGC_CFG_OVERRIDE_BIT_MASK            0x40U
#define AGC_CFG_CFG_OVRD_PGF_GAIN_BIT_OFFSET (3U)
#define AGC_CFG_CFG_OVRD_PGF_GAIN_BIT_LEN    (3U)
#define AGC_CFG_CFG_OVRD_PGF_GAIN_BIT_MASK   0x38U
#define AGC_CFG_PREOFF_BIT_OFFSET            (2U)
#define AGC_CFG_PREOFF_BIT_LEN               (1U)
#define AGC_CFG_PREOFF_BIT_MASK              0x4U
#define AGC_CFG_PREDETOFF_BIT_OFFSET         (1U)
#define AGC_CFG_PREDETOFF_BIT_LEN            (1U)
#define AGC_CFG_PREDETOFF_BIT_MASK           0x2U
#define AGC_CFG_CFG_AGC_ENABLE_BIT_OFFSET    (0U)
#define AGC_CFG_CFG_AGC_ENABLE_BIT_LEN       (1U)
#define AGC_CFG_CFG_AGC_ENABLE_BIT_MASK      0x1U

/******************************************************************************
* @brief Bit definitions for register DGC_CFG
**/
#define DGC_CFG_ID                           0x30018
#define DGC_CFG_LEN                          (4U)
#define DGC_CFG_MASK                         0xFFFFFFFFUL
#define DGC_CFG_DGC_TXRX_SWITCH_TEST_BIT_OFFSET (30U)
#define DGC_CFG_DGC_TXRX_SWITCH_TEST_BIT_LEN (1U)
#define DGC_CFG_DGC_TXRX_SWITCH_TEST_BIT_MASK 0x40000000UL
#define DGC_CFG_CFG_SYM_TO_SKIP_BIT_OFFSET   (27U)
#define DGC_CFG_CFG_SYM_TO_SKIP_BIT_LEN      (3U)
#define DGC_CFG_CFG_SYM_TO_SKIP_BIT_MASK     0x38000000UL
#define DGC_CFG_CFG_CORRELATOR_THR_16_BIT_OFFSET (21U)
#define DGC_CFG_CFG_CORRELATOR_THR_16_BIT_LEN (6U)
#define DGC_CFG_CFG_CORRELATOR_THR_16_BIT_MASK 0x7e00000UL
#define DGC_CFG_CFG_ACCUMULATOR_THR_16_BIT_OFFSET (15U)
#define DGC_CFG_CFG_ACCUMULATOR_THR_16_BIT_LEN (6U)
#define DGC_CFG_CFG_ACCUMULATOR_THR_16_BIT_MASK 0x1f8000UL
#define DGC_CFG_CFG_CORRELATOR_THR_64_BIT_OFFSET (9U)
#define DGC_CFG_CFG_CORRELATOR_THR_64_BIT_LEN (6U)
#define DGC_CFG_CFG_CORRELATOR_THR_64_BIT_MASK 0x7e00U
#define DGC_CFG_CFG_ACCUMULATOR_THR_64_BIT_OFFSET (3U)
#define DGC_CFG_CFG_ACCUMULATOR_THR_64_BIT_LEN (6U)
#define DGC_CFG_CFG_ACCUMULATOR_THR_64_BIT_MASK 0x1f8U
#define DGC_CFG_CFG_USE_2ND_DECISION_BIT_OFFSET (2U)
#define DGC_CFG_CFG_USE_2ND_DECISION_BIT_LEN (1U)
#define DGC_CFG_CFG_USE_2ND_DECISION_BIT_MASK 0x4U
#define DGC_CFG_CFG_USE_NRG_DETECTOR_BIT_OFFSET (1U)
#define DGC_CFG_CFG_USE_NRG_DETECTOR_BIT_LEN (1U)
#define DGC_CFG_CFG_USE_NRG_DETECTOR_BIT_MASK 0x2U
#define DGC_CFG_CFG_DGC_EN_BIT_OFFSET        (0U)
#define DGC_CFG_CFG_DGC_EN_BIT_LEN           (1U)
#define DGC_CFG_CFG_DGC_EN_BIT_MASK          0x1U

/******************************************************************************
* @brief Bit definitions for register DGC_ACC_CORR_LUT_CFG
**/
#define DGC_ACC_CORR_LUT_CFG_ID              0x3001c
#define DGC_ACC_CORR_LUT_CFG_LEN             (4U)
#define DGC_ACC_CORR_LUT_CFG_MASK            0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_CORR_LUT_7_16_CFG
**/
#define DGC_CORR_LUT_7_16_CFG_ID             0x30020
#define DGC_CORR_LUT_7_16_CFG_LEN            (4U)
#define DGC_CORR_LUT_7_16_CFG_MASK           0xFFFFFFFFUL
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_16_BIT_OFFSET (27U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_16_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_16_BIT_MASK 0x38000000UL
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_15_BIT_OFFSET (24U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_15_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_15_BIT_MASK 0x7000000UL
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_14_BIT_OFFSET (21U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_14_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_14_BIT_MASK 0xe00000UL
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_13_BIT_OFFSET (18U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_13_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_13_BIT_MASK 0x1c0000UL
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_12_BIT_OFFSET (15U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_12_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_12_BIT_MASK 0x38000UL
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_11_BIT_OFFSET (12U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_11_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_11_BIT_MASK 0x7000U
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_10_BIT_OFFSET (9U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_10_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_10_BIT_MASK 0xe00U
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_9_BIT_OFFSET (6U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_9_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_9_BIT_MASK 0x1c0U
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_8_BIT_OFFSET (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_8_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_8_BIT_MASK 0x38U
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_7_BIT_OFFSET (0U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_7_BIT_LEN (3U)
#define DGC_CORR_LUT_7_16_CFG_CFG_CORR_LUT_7_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register DGC_CORR_LUT_17_26_CFG
**/
#define DGC_CORR_LUT_17_26_CFG_ID            0x30024
#define DGC_CORR_LUT_17_26_CFG_LEN           (4U)
#define DGC_CORR_LUT_17_26_CFG_MASK          0xFFFFFFFFUL
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_26_BIT_OFFSET (27U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_26_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_26_BIT_MASK 0x38000000UL
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_25_BIT_OFFSET (24U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_25_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_25_BIT_MASK 0x7000000UL
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_24_BIT_OFFSET (21U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_24_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_24_BIT_MASK 0xe00000UL
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_23_BIT_OFFSET (18U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_23_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_23_BIT_MASK 0x1c0000UL
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_22_BIT_OFFSET (15U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_22_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_22_BIT_MASK 0x38000UL
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_21_BIT_OFFSET (12U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_21_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_21_BIT_MASK 0x7000U
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_20_BIT_OFFSET (9U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_20_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_20_BIT_MASK 0xe00U
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_19_BIT_OFFSET (6U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_19_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_19_BIT_MASK 0x1c0U
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_18_BIT_OFFSET (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_18_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_18_BIT_MASK 0x38U
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_17_BIT_OFFSET (0U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_17_BIT_LEN (3U)
#define DGC_CORR_LUT_17_26_CFG_CFG_CORR_LUT_17_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register DGC_CORR_NRG_LUT_CFG
**/
#define DGC_CORR_NRG_LUT_CFG_ID              0x30028
#define DGC_CORR_NRG_LUT_CFG_LEN             (4U)
#define DGC_CORR_NRG_LUT_CFG_MASK            0xFFFFFFFFUL
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_4_BIT_OFFSET (27U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_4_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_4_BIT_MASK 0x38000000UL
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_3_BIT_OFFSET (24U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_3_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_3_BIT_MASK 0x7000000UL
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_2_BIT_OFFSET (21U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_2_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_2_BIT_MASK 0xe00000UL
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_1_BIT_OFFSET (18U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_1_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_1_BIT_MASK 0x1c0000UL
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_0_BIT_OFFSET (15U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_0_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_NRG_LUT_0_BIT_MASK 0x38000UL
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_31_BIT_OFFSET (12U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_31_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_31_BIT_MASK 0x7000U
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_30_BIT_OFFSET (9U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_30_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_30_BIT_MASK 0xe00U
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_29_BIT_OFFSET (6U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_29_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_29_BIT_MASK 0x1c0U
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_28_BIT_OFFSET (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_28_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_28_BIT_MASK 0x38U
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_27_BIT_OFFSET (0U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_27_BIT_LEN (3U)
#define DGC_CORR_NRG_LUT_CFG_CFG_CORR_LUT_27_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register DGC_NRG_LUT_5_14_CFG
**/
#define DGC_NRG_LUT_5_14_CFG_ID              0x3002c
#define DGC_NRG_LUT_5_14_CFG_LEN             (4U)
#define DGC_NRG_LUT_5_14_CFG_MASK            0xFFFFFFFFUL
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_14_BIT_OFFSET (27U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_14_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_14_BIT_MASK 0x38000000UL
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_13_BIT_OFFSET (24U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_13_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_13_BIT_MASK 0x7000000UL
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_12_BIT_OFFSET (21U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_12_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_12_BIT_MASK 0xe00000UL
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_11_BIT_OFFSET (18U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_11_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_11_BIT_MASK 0x1c0000UL
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_10_BIT_OFFSET (15U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_10_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_10_BIT_MASK 0x38000UL
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_9_BIT_OFFSET (12U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_9_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_9_BIT_MASK 0x7000U
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_8_BIT_OFFSET (9U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_8_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_8_BIT_MASK 0xe00U
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_7_BIT_OFFSET (6U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_7_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_7_BIT_MASK 0x1c0U
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_6_BIT_OFFSET (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_6_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_6_BIT_MASK 0x38U
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_5_BIT_OFFSET (0U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_5_BIT_LEN (3U)
#define DGC_NRG_LUT_5_14_CFG_CFG_NRG_LUT_5_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register DGC_NRG_LUT_15_24_CFG
**/
#define DGC_NRG_LUT_15_24_CFG_ID             0x30030
#define DGC_NRG_LUT_15_24_CFG_LEN            (4U)
#define DGC_NRG_LUT_15_24_CFG_MASK           0xFFFFFFFFUL
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_24_BIT_OFFSET (27U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_24_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_24_BIT_MASK 0x38000000UL
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_23_BIT_OFFSET (24U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_23_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_23_BIT_MASK 0x7000000UL
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_22_BIT_OFFSET (21U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_22_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_22_BIT_MASK 0xe00000UL
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_21_BIT_OFFSET (18U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_21_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_21_BIT_MASK 0x1c0000UL
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_20_BIT_OFFSET (15U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_20_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_20_BIT_MASK 0x38000UL
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_19_BIT_OFFSET (12U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_19_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_19_BIT_MASK 0x7000U
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_18_BIT_OFFSET (9U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_18_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_18_BIT_MASK 0xe00U
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_17_BIT_OFFSET (6U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_17_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_17_BIT_MASK 0x1c0U
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_16_BIT_OFFSET (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_16_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_16_BIT_MASK 0x38U
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_15_BIT_OFFSET (0U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_15_BIT_LEN (3U)
#define DGC_NRG_LUT_15_24_CFG_CFG_NRG_LUT_15_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register DGC_NRG_LUT_CFG
**/
#define DGC_NRG_LUT_CFG_ID                   0x30034
#define DGC_NRG_LUT_CFG_LEN                  (4U)
#define DGC_NRG_LUT_CFG_MASK                 0xFFFFFFFFUL
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_31_BIT_OFFSET (18U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_31_BIT_LEN (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_31_BIT_MASK 0x1c0000UL
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_30_BIT_OFFSET (15U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_30_BIT_LEN (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_30_BIT_MASK 0x38000UL
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_29_BIT_OFFSET (12U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_29_BIT_LEN (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_29_BIT_MASK 0x7000U
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_28_BIT_OFFSET (9U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_28_BIT_LEN (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_28_BIT_MASK 0xe00U
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_27_BIT_OFFSET (6U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_27_BIT_LEN (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_27_BIT_MASK 0x1c0U
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_26_BIT_OFFSET (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_26_BIT_LEN (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_26_BIT_MASK 0x38U
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_25_BIT_OFFSET (0U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_25_BIT_LEN (3U)
#define DGC_NRG_LUT_CFG_CFG_NRG_LUT_25_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register DGC_DGC_LUT_0_CFG
**/
#define DGC_DGC_LUT_0_CFG_ID                 0x30038
#define DGC_DGC_LUT_0_CFG_LEN                (4U)
#define DGC_DGC_LUT_0_CFG_MASK               0xFFFFFFFFUL
#define DGC_DGC_LUT_0_CFG_CFG_DGC_LUT_0_BIT_OFFSET (0U)
#define DGC_DGC_LUT_0_CFG_CFG_DGC_LUT_0_BIT_LEN (18U)
#define DGC_DGC_LUT_0_CFG_CFG_DGC_LUT_0_BIT_MASK 0x3ffffUL

/******************************************************************************
* @brief Bit definitions for register DGC_DGC_LUT_1_CFG
**/
#define DGC_DGC_LUT_1_CFG_ID                 0x3003c
#define DGC_DGC_LUT_1_CFG_LEN                (4U)
#define DGC_DGC_LUT_1_CFG_MASK               0xFFFFFFFFUL
#define DGC_DGC_LUT_1_CFG_CFG_DGC_LUT_1_BIT_OFFSET (0U)
#define DGC_DGC_LUT_1_CFG_CFG_DGC_LUT_1_BIT_LEN (18U)
#define DGC_DGC_LUT_1_CFG_CFG_DGC_LUT_1_BIT_MASK 0x3ffffUL

/******************************************************************************
* @brief Bit definitions for register DGC_DGC_LUT_2_CFG
**/
#define DGC_DGC_LUT_2_CFG_ID                 0x30040
#define DGC_DGC_LUT_2_CFG_LEN                (4U)
#define DGC_DGC_LUT_2_CFG_MASK               0xFFFFFFFFUL
#define DGC_DGC_LUT_2_CFG_CFG_DGC_LUT_2_BIT_OFFSET (0U)
#define DGC_DGC_LUT_2_CFG_CFG_DGC_LUT_2_BIT_LEN (18U)
#define DGC_DGC_LUT_2_CFG_CFG_DGC_LUT_2_BIT_MASK 0x3ffffUL

/******************************************************************************
* @brief Bit definitions for register DGC_DGC_LUT_3_CFG
**/
#define DGC_DGC_LUT_3_CFG_ID                 0x30044
#define DGC_DGC_LUT_3_CFG_LEN                (4U)
#define DGC_DGC_LUT_3_CFG_MASK               0xFFFFFFFFUL
#define DGC_DGC_LUT_3_CFG_CFG_DGC_LUT_3_BIT_OFFSET (0U)
#define DGC_DGC_LUT_3_CFG_CFG_DGC_LUT_3_BIT_LEN (18U)
#define DGC_DGC_LUT_3_CFG_CFG_DGC_LUT_3_BIT_MASK 0x3ffffUL

/******************************************************************************
* @brief Bit definitions for register DGC_DGC_LUT_4_CFG
**/
#define DGC_DGC_LUT_4_CFG_ID                 0x30048
#define DGC_DGC_LUT_4_CFG_LEN                (4U)
#define DGC_DGC_LUT_4_CFG_MASK               0xFFFFFFFFUL
#define DGC_DGC_LUT_4_CFG_CFG_DGC_LUT_4_BIT_OFFSET (0U)
#define DGC_DGC_LUT_4_CFG_CFG_DGC_LUT_4_BIT_LEN (18U)
#define DGC_DGC_LUT_4_CFG_CFG_DGC_LUT_4_BIT_MASK 0x3ffffUL

/******************************************************************************
* @brief Bit definitions for register DGC_DGC_LUT_5_CFG
**/
#define DGC_DGC_LUT_5_CFG_ID                 0x3004c
#define DGC_DGC_LUT_5_CFG_LEN                (4U)
#define DGC_DGC_LUT_5_CFG_MASK               0xFFFFFFFFUL
#define DGC_DGC_LUT_5_CFG_CFG_DGC_LUT_5_BIT_OFFSET (0U)
#define DGC_DGC_LUT_5_CFG_CFG_DGC_LUT_5_BIT_LEN (18U)
#define DGC_DGC_LUT_5_CFG_CFG_DGC_LUT_5_BIT_MASK 0x3ffffUL

/******************************************************************************
* @brief Bit definitions for register DGC_DGC_LUT_6_CFG
**/
#define DGC_DGC_LUT_6_CFG_ID                 0x30050
#define DGC_DGC_LUT_6_CFG_LEN                (4U)
#define DGC_DGC_LUT_6_CFG_MASK               0xFFFFFFFFUL
#define DGC_DGC_LUT_6_CFG_CFG_DGC_LUT_6_BIT_OFFSET (0U)
#define DGC_DGC_LUT_6_CFG_CFG_DGC_LUT_6_BIT_LEN (18U)
#define DGC_DGC_LUT_6_CFG_CFG_DGC_LUT_6_BIT_MASK 0x3ffffUL

/******************************************************************************
* @brief Bit definitions for register DGC_MIN_METRIC_LIM_WNDW_CFG
**/
#define DGC_MIN_METRIC_LIM_WNDW_CFG_ID       0x30054
#define DGC_MIN_METRIC_LIM_WNDW_CFG_LEN      (4U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_MASK     0xFFFFFFFFUL
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_CORR_LIM_WNDW_LEN_BIT_OFFSET (21U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_CORR_LIM_WNDW_LEN_BIT_LEN (5U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_CORR_LIM_WNDW_LEN_BIT_MASK 0x3e00000UL
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_ACC_LIM_WNDW_LEN_BIT_OFFSET (18U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_ACC_LIM_WNDW_LEN_BIT_LEN (3U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_ACC_LIM_WNDW_LEN_BIT_MASK 0x1c0000UL
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_LIMITED_COUNT_EN_BIT_OFFSET (17U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_LIMITED_COUNT_EN_BIT_LEN (1U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_LIMITED_COUNT_EN_BIT_MASK 0x20000UL
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_NRG_BACKOFF_BIT_OFFSET (15U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_NRG_BACKOFF_BIT_LEN (2U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_NRG_BACKOFF_BIT_MASK 0x18000UL
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_NRG_METRIC_MIN_BIT_OFFSET (10U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_NRG_METRIC_MIN_BIT_LEN (5U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_NRG_METRIC_MIN_BIT_MASK 0x7c00U
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_CORR_SAT_MIN_BIT_OFFSET (4U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_CORR_SAT_MIN_BIT_LEN (6U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_CORR_SAT_MIN_BIT_MASK 0x3f0U
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_ACC_SAT_MIN_BIT_OFFSET (0U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_ACC_SAT_MIN_BIT_LEN (4U)
#define DGC_MIN_METRIC_LIM_WNDW_CFG_CFG_ACC_SAT_MIN_BIT_MASK 0xfU

/******************************************************************************
* @brief Bit definitions for register ADC_THRESH_DBG
**/
#define ADC_THRESH_DBG_ID                    0x30058
#define ADC_THRESH_DBG_LEN                   (4U)
#define ADC_THRESH_DBG_MASK                  0xFFFFFFFFUL
#define ADC_THRESH_DBG_DBG_ADC_THRESH_Q_NEG_BIT_OFFSET (24U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_Q_NEG_BIT_LEN (8U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_Q_NEG_BIT_MASK 0xff000000UL
#define ADC_THRESH_DBG_DBG_ADC_THRESH_Q_POS_BIT_OFFSET (16U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_Q_POS_BIT_LEN (8U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_Q_POS_BIT_MASK 0xff0000UL
#define ADC_THRESH_DBG_DBG_ADC_THRESH_I_NEG_BIT_OFFSET (8U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_I_NEG_BIT_LEN (8U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_I_NEG_BIT_MASK 0xff00U
#define ADC_THRESH_DBG_DBG_ADC_THRESH_I_POS_BIT_OFFSET (0U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_I_POS_BIT_LEN (8U)
#define ADC_THRESH_DBG_DBG_ADC_THRESH_I_POS_BIT_MASK 0xffU

/******************************************************************************
* @brief Bit definitions for register ADC_DBG
**/
#define ADC_DBG_ID                           0x3005c
#define ADC_DBG_LEN                          (4U)
#define ADC_DBG_MASK                         0xFFFFFFFFUL
#define ADC_DBG_DBG_STATS_Q_POS_BIT_OFFSET   (22U)
#define ADC_DBG_DBG_STATS_Q_POS_BIT_LEN      (10U)
#define ADC_DBG_DBG_STATS_Q_POS_BIT_MASK     0xffc00000UL
#define ADC_DBG_DBG_STATS_I_NEG_BIT_OFFSET   (12U)
#define ADC_DBG_DBG_STATS_I_NEG_BIT_LEN      (10U)
#define ADC_DBG_DBG_STATS_I_NEG_BIT_MASK     0x3ff000UL
#define ADC_DBG_DBG_STATS_I_POS_BIT_OFFSET   (2U)
#define ADC_DBG_DBG_STATS_I_POS_BIT_LEN      (10U)
#define ADC_DBG_DBG_STATS_I_POS_BIT_MASK     0xffcU
#define ADC_DBG_AGC_STATUS_BIT_OFFSET        (0U)
#define ADC_DBG_AGC_STATUS_BIT_LEN           (2U)
#define ADC_DBG_AGC_STATUS_BIT_MASK          0x3U

/******************************************************************************
* @brief Bit definitions for register ADC_DGC_DBG
**/
#define ADC_DGC_DBG_ID                       0x30060
#define ADC_DGC_DBG_LEN                      (4U)
#define ADC_DGC_DBG_MASK                     0xFFFFFFFFUL
#define ADC_DGC_DBG_DBG_SECOND_DECISION_BIT_OFFSET (28U)
#define ADC_DGC_DBG_DBG_SECOND_DECISION_BIT_LEN (3U)
#define ADC_DGC_DBG_DBG_SECOND_DECISION_BIT_MASK 0x70000000UL
#define ADC_DGC_DBG_DBG_FIRST_DECISION_BIT_OFFSET (25U)
#define ADC_DGC_DBG_DBG_FIRST_DECISION_BIT_LEN (3U)
#define ADC_DGC_DBG_DBG_FIRST_DECISION_BIT_MASK 0xe000000UL
#define ADC_DGC_DBG_DBG_ED_PGF_GAIN_BIT_OFFSET (22U)
#define ADC_DGC_DBG_DBG_ED_PGF_GAIN_BIT_LEN  (3U)
#define ADC_DGC_DBG_DBG_ED_PGF_GAIN_BIT_MASK 0x1c00000UL
#define ADC_DGC_DBG_DBG_ED_ADC_TH_DIFF_BIT_OFFSET (13U)
#define ADC_DGC_DBG_DBG_ED_ADC_TH_DIFF_BIT_LEN (9U)
#define ADC_DGC_DBG_DBG_ED_ADC_TH_DIFF_BIT_MASK 0x3fe000UL
#define ADC_DGC_DBG_DBG_PGF_GAIN_BIT_OFFSET  (10U)
#define ADC_DGC_DBG_DBG_PGF_GAIN_BIT_LEN     (3U)
#define ADC_DGC_DBG_DBG_PGF_GAIN_BIT_MASK    0x1c00U
#define ADC_DGC_DBG_DBG_STATS_Q_NEG_BIT_OFFSET (0U)
#define ADC_DGC_DBG_DBG_STATS_Q_NEG_BIT_LEN  (10U)
#define ADC_DGC_DBG_DBG_STATS_Q_NEG_BIT_MASK 0x3ffU

/******************************************************************************
* @brief Bit definitions for register DGC_DBG
**/
#define DGC_DBG_ID                           0x30064
#define DGC_DBG_LEN                          (4U)
#define DGC_DBG_MASK                         0xFFFFFFFFUL
#define DGC_DBG_DBG_CORR_METRIC_2ND_BIT_OFFSET (20U)
#define DGC_DBG_DBG_CORR_METRIC_2ND_BIT_LEN  (7U)
#define DGC_DBG_DBG_CORR_METRIC_2ND_BIT_MASK 0x7f00000UL
#define DGC_DBG_DBG_CORR_METRIC_1ST_BIT_OFFSET (13U)
#define DGC_DBG_DBG_CORR_METRIC_1ST_BIT_LEN  (7U)
#define DGC_DBG_DBG_CORR_METRIC_1ST_BIT_MASK 0xfe000UL
#define DGC_DBG_DBG_ACC_METRIC_BIT_OFFSET    (9U)
#define DGC_DBG_DBG_ACC_METRIC_BIT_LEN       (4U)
#define DGC_DBG_DBG_ACC_METRIC_BIT_MASK      0x1e00U
#define DGC_DBG_DBG_NRG_METRIC_BIT_OFFSET    (0U)
#define DGC_DBG_DBG_NRG_METRIC_BIT_LEN       (9U)
#define DGC_DBG_DBG_NRG_METRIC_BIT_MASK      0x1ffU

/******************************************************************************
* @brief Bit definitions for register EC_CTRL
**/
#define EC_CTRL_ID                           0x40000
#define EC_CTRL_LEN                          (4U)
#define EC_CTRL_MASK                         0xFFFFFFFFUL
#define EC_CTRL_SYNC_RST_N_BIT_OFFSET        (12U)
#define EC_CTRL_SYNC_RST_N_BIT_LEN           (1U)
#define EC_CTRL_SYNC_RST_N_BIT_MASK          0x1000U
#define EC_CTRL_OSTR_MODE_BIT_OFFSET         (11U)
#define EC_CTRL_OSTR_MODE_BIT_LEN            (1U)
#define EC_CTRL_OSTR_MODE_BIT_MASK           0x800U
#define EC_CTRL_OSTS_WAIT_BIT_OFFSET         (3U)
#define EC_CTRL_OSTS_WAIT_BIT_LEN            (8U)
#define EC_CTRL_OSTS_WAIT_BIT_MASK           0x7f8U
#define EC_CTRL_CLKPLL_LCK_DET_BIT_OFFSET    (2U)
#define EC_CTRL_CLKPLL_LCK_DET_BIT_LEN       (1U)
#define EC_CTRL_CLKPLL_LCK_DET_BIT_MASK      0x4U
#define EC_CTRL_OSRS_MODE_BIT_OFFSET         (1U)
#define EC_CTRL_OSRS_MODE_BIT_LEN            (1U)
#define EC_CTRL_OSRS_MODE_BIT_MASK           0x2U
#define EC_CTRL_OSTS_MODE_BIT_OFFSET         (0U)
#define EC_CTRL_OSTS_MODE_BIT_LEN            (1U)
#define EC_CTRL_OSTS_MODE_BIT_MASK           0x1U

/******************************************************************************
* @brief Bit definitions for register EC_RXTC
**/
#define EC_RXTC_ID                           0x40004
#define EC_RXTC_LEN                          (4U)
#define EC_RXTC_MASK                         0xFFFFFFFFUL
#define EC_RXTC_RX_TS_EST_BIT_OFFSET         (0U)
#define EC_RXTC_RX_TS_EST_BIT_LEN            (32U)
#define EC_RXTC_RX_TS_EST_BIT_MASK           0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register EC_GOLP
**/
#define EC_GOLP_ID                           0x40008
#define EC_GOLP_LEN                          (4U)
#define EC_GOLP_MASK                         0xFFFFFFFFUL
#define EC_GOLP_OFFSET_EXT_BIT_OFFSET        (0U)
#define EC_GOLP_OFFSET_EXT_BIT_LEN           (6U)
#define EC_GOLP_OFFSET_EXT_BIT_MASK          0x3fU

/******************************************************************************
* @brief Bit definitions for register PGF_CAL_CFG
**/
#define PGF_CAL_CFG_ID                       0x4000c
#define PGF_CAL_CFG_LEN                      (4U)
#define PGF_CAL_CFG_MASK                     0xFFFFFFFFUL
#define PGF_CAL_CFG_COMP_DLY_BIT_OFFSET      (16U)
#define PGF_CAL_CFG_COMP_DLY_BIT_LEN         (4U)
#define PGF_CAL_CFG_COMP_DLY_BIT_MASK        0xf0000UL
#define PGF_CAL_CFG_PGF_GAIN_BIT_OFFSET      (8U)
#define PGF_CAL_CFG_PGF_GAIN_BIT_LEN         (5U)
#define PGF_CAL_CFG_PGF_GAIN_BIT_MASK        0x1f00U
#define PGF_CAL_CFG_CAL_EN_BIT_OFFSET        (4U)
#define PGF_CAL_CFG_CAL_EN_BIT_LEN           (1U)
#define PGF_CAL_CFG_CAL_EN_BIT_MASK          0x10U
#define PGF_CAL_CFG_PGF_MODE_BIT_OFFSET      (0U)
#define PGF_CAL_CFG_PGF_MODE_BIT_LEN         (2U)
#define PGF_CAL_CFG_PGF_MODE_BIT_MASK        0x3U

/******************************************************************************
* @brief Bit definitions for register PGF_I_CTRL0
**/
#define PGF_I_CTRL0_ID                       0x40010
#define PGF_I_CTRL0_LEN                      (4U)
#define PGF_I_CTRL0_MASK                     0xFFFFFFFFUL
#define PGF_I_CTRL0_COMP_CLK_BIT_OFFSET      (8U)
#define PGF_I_CTRL0_COMP_CLK_BIT_LEN         (1U)
#define PGF_I_CTRL0_COMP_CLK_BIT_MASK        0x100U
#define PGF_I_CTRL0_BQ1_SHORT_EN_BIT_OFFSET  (7U)
#define PGF_I_CTRL0_BQ1_SHORT_EN_BIT_LEN     (1U)
#define PGF_I_CTRL0_BQ1_SHORT_EN_BIT_MASK    0x80U
#define PGF_I_CTRL0_BQ2_SHORT_EN_BIT_OFFSET  (6U)
#define PGF_I_CTRL0_BQ2_SHORT_EN_BIT_LEN     (1U)
#define PGF_I_CTRL0_BQ2_SHORT_EN_BIT_MASK    0x40U
#define PGF_I_CTRL0_BQ3_SHORT_EN_BIT_OFFSET  (5U)
#define PGF_I_CTRL0_BQ3_SHORT_EN_BIT_LEN     (1U)
#define PGF_I_CTRL0_BQ3_SHORT_EN_BIT_MASK    0x20U
#define PGF_I_CTRL0_LOAD_SHORT_EN_BIT_OFFSET (4U)
#define PGF_I_CTRL0_LOAD_SHORT_EN_BIT_LEN    (1U)
#define PGF_I_CTRL0_LOAD_SHORT_EN_BIT_MASK   0x10U
#define PGF_I_CTRL0_HI_BUF_SHORT_EN_BIT_OFFSET (3U)
#define PGF_I_CTRL0_HI_BUF_SHORT_EN_BIT_LEN  (1U)
#define PGF_I_CTRL0_HI_BUF_SHORT_EN_BIT_MASK 0x8U
#define PGF_I_CTRL0_LO_BUF_SHORT_EN_BIT_OFFSET (2U)
#define PGF_I_CTRL0_LO_BUF_SHORT_EN_BIT_LEN  (1U)
#define PGF_I_CTRL0_LO_BUF_SHORT_EN_BIT_MASK 0x4U
#define PGF_I_CTRL0_HI_BUF_CAL_EN_BIT_OFFSET (1U)
#define PGF_I_CTRL0_HI_BUF_CAL_EN_BIT_LEN    (1U)
#define PGF_I_CTRL0_HI_BUF_CAL_EN_BIT_MASK   0x2U
#define PGF_I_CTRL0_LO_BUF_CAL_EN_BIT_OFFSET (0U)
#define PGF_I_CTRL0_LO_BUF_CAL_EN_BIT_LEN    (1U)
#define PGF_I_CTRL0_LO_BUF_CAL_EN_BIT_MASK   0x1U

/******************************************************************************
* @brief Bit definitions for register PGF_I_CTRL1
**/
#define PGF_I_CTRL1_ID                       0x40014
#define PGF_I_CTRL1_LEN                      (4U)
#define PGF_I_CTRL1_MASK                     0xFFFFFFFFUL
#define PGF_I_CTRL1_BQ1_OS_BIT_OFFSET        (24U)
#define PGF_I_CTRL1_BQ1_OS_BIT_LEN           (5U)
#define PGF_I_CTRL1_BQ1_OS_BIT_MASK          0x1f000000UL
#define PGF_I_CTRL1_BQ2_OS_BIT_OFFSET        (20U)
#define PGF_I_CTRL1_BQ2_OS_BIT_LEN           (4U)
#define PGF_I_CTRL1_BQ2_OS_BIT_MASK          0xf00000UL
#define PGF_I_CTRL1_BQ3_OS_BIT_OFFSET        (16U)
#define PGF_I_CTRL1_BQ3_OS_BIT_LEN           (4U)
#define PGF_I_CTRL1_BQ3_OS_BIT_MASK          0xf0000UL
#define PGF_I_CTRL1_LOAD_OS_BIT_OFFSET       (10U)
#define PGF_I_CTRL1_LOAD_OS_BIT_LEN          (6U)
#define PGF_I_CTRL1_LOAD_OS_BIT_MASK         0xfc00U
#define PGF_I_CTRL1_HI_BUF_OS_BIT_OFFSET     (5U)
#define PGF_I_CTRL1_HI_BUF_OS_BIT_LEN        (5U)
#define PGF_I_CTRL1_HI_BUF_OS_BIT_MASK       0x3e0U
#define PGF_I_CTRL1_LO_BUF_OS_BIT_OFFSET     (0U)
#define PGF_I_CTRL1_LO_BUF_OS_BIT_LEN        (5U)
#define PGF_I_CTRL1_LO_BUF_OS_BIT_MASK       0x1fU

/******************************************************************************
* @brief Bit definitions for register PGF_Q_CTRL0
**/
#define PGF_Q_CTRL0_ID                       0x40018
#define PGF_Q_CTRL0_LEN                      (4U)
#define PGF_Q_CTRL0_MASK                     0xFFFFFFFFUL
#define PGF_Q_CTRL0_COMP_CLK_BIT_OFFSET      (8U)
#define PGF_Q_CTRL0_COMP_CLK_BIT_LEN         (1U)
#define PGF_Q_CTRL0_COMP_CLK_BIT_MASK        0x100U
#define PGF_Q_CTRL0_BQ1_SHORT_EN_BIT_OFFSET  (7U)
#define PGF_Q_CTRL0_BQ1_SHORT_EN_BIT_LEN     (1U)
#define PGF_Q_CTRL0_BQ1_SHORT_EN_BIT_MASK    0x80U
#define PGF_Q_CTRL0_BQ2_SHORT_EN_BIT_OFFSET  (6U)
#define PGF_Q_CTRL0_BQ2_SHORT_EN_BIT_LEN     (1U)
#define PGF_Q_CTRL0_BQ2_SHORT_EN_BIT_MASK    0x40U
#define PGF_Q_CTRL0_BQ3_SHORT_EN_BIT_OFFSET  (5U)
#define PGF_Q_CTRL0_BQ3_SHORT_EN_BIT_LEN     (1U)
#define PGF_Q_CTRL0_BQ3_SHORT_EN_BIT_MASK    0x20U
#define PGF_Q_CTRL0_LOAD_SHORT_EN_BIT_OFFSET (4U)
#define PGF_Q_CTRL0_LOAD_SHORT_EN_BIT_LEN    (1U)
#define PGF_Q_CTRL0_LOAD_SHORT_EN_BIT_MASK   0x10U
#define PGF_Q_CTRL0_HI_BUF_SHORT_EN_BIT_OFFSET (3U)
#define PGF_Q_CTRL0_HI_BUF_SHORT_EN_BIT_LEN  (1U)
#define PGF_Q_CTRL0_HI_BUF_SHORT_EN_BIT_MASK 0x8U
#define PGF_Q_CTRL0_LO_BUF_SHORT_EN_BIT_OFFSET (2U)
#define PGF_Q_CTRL0_LO_BUF_SHORT_EN_BIT_LEN  (1U)
#define PGF_Q_CTRL0_LO_BUF_SHORT_EN_BIT_MASK 0x4U
#define PGF_Q_CTRL0_HI_BUF_CAL_EN_BIT_OFFSET (1U)
#define PGF_Q_CTRL0_HI_BUF_CAL_EN_BIT_LEN    (1U)
#define PGF_Q_CTRL0_HI_BUF_CAL_EN_BIT_MASK   0x2U
#define PGF_Q_CTRL0_LO_BUF_CAL_EN_BIT_OFFSET (0U)
#define PGF_Q_CTRL0_LO_BUF_CAL_EN_BIT_LEN    (1U)
#define PGF_Q_CTRL0_LO_BUF_CAL_EN_BIT_MASK   0x1U

/******************************************************************************
* @brief Bit definitions for register PGF_Q_CTRL1
**/
#define PGF_Q_CTRL1_ID                       0x4001c
#define PGF_Q_CTRL1_LEN                      (4U)
#define PGF_Q_CTRL1_MASK                     0xFFFFFFFFUL
#define PGF_Q_CTRL1_BQ1_OS_BIT_OFFSET        (24U)
#define PGF_Q_CTRL1_BQ1_OS_BIT_LEN           (5U)
#define PGF_Q_CTRL1_BQ1_OS_BIT_MASK          0x1f000000UL
#define PGF_Q_CTRL1_BQ2_OS_BIT_OFFSET        (20U)
#define PGF_Q_CTRL1_BQ2_OS_BIT_LEN           (4U)
#define PGF_Q_CTRL1_BQ2_OS_BIT_MASK          0xf00000UL
#define PGF_Q_CTRL1_BQ3_OS_BIT_OFFSET        (16U)
#define PGF_Q_CTRL1_BQ3_OS_BIT_LEN           (4U)
#define PGF_Q_CTRL1_BQ3_OS_BIT_MASK          0xf0000UL
#define PGF_Q_CTRL1_LOAD_OS_BIT_OFFSET       (10U)
#define PGF_Q_CTRL1_LOAD_OS_BIT_LEN          (6U)
#define PGF_Q_CTRL1_LOAD_OS_BIT_MASK         0xfc00U
#define PGF_Q_CTRL1_HI_BUF_OS_BIT_OFFSET     (5U)
#define PGF_Q_CTRL1_HI_BUF_OS_BIT_LEN        (5U)
#define PGF_Q_CTRL1_HI_BUF_OS_BIT_MASK       0x3e0U
#define PGF_Q_CTRL1_LO_BUF_OS_BIT_OFFSET     (0U)
#define PGF_Q_CTRL1_LO_BUF_OS_BIT_LEN        (5U)
#define PGF_Q_CTRL1_LO_BUF_OS_BIT_MASK       0x1fU

/******************************************************************************
* @brief Bit definitions for register PGF_CAL_STS
**/
#define PGF_CAL_STS_ID                       0x40020
#define PGF_CAL_STS_LEN                      (4U)
#define PGF_CAL_STS_MASK                     0xFFFFFFFFUL
#define PGF_CAL_STS_CAL_DONE_BIT_OFFSET      (0U)
#define PGF_CAL_STS_CAL_DONE_BIT_LEN         (1U)
#define PGF_CAL_STS_CAL_DONE_BIT_MASK        0x1U

/******************************************************************************
* @brief Bit definitions for register PGF_COMP_STS
**/
#define PGF_COMP_STS_ID                      0x40024
#define PGF_COMP_STS_LEN                     (4U)
#define PGF_COMP_STS_MASK                    0xFFFFFFFFUL
#define PGF_COMP_STS_Q_COMP_OUT_BIT_OFFSET   (1U)
#define PGF_COMP_STS_Q_COMP_OUT_BIT_LEN      (1U)
#define PGF_COMP_STS_Q_COMP_OUT_BIT_MASK     0x2U
#define PGF_COMP_STS_I_COMP_OUT_BIT_OFFSET   (0U)
#define PGF_COMP_STS_I_COMP_OUT_BIT_LEN      (1U)
#define PGF_COMP_STS_I_COMP_OUT_BIT_MASK     0x1U

/******************************************************************************
* @brief Bit definitions for register MFIO_MODE
**/
#define MFIO_MODE_ID                         0x50000
#define MFIO_MODE_LEN                        (4U)
#define MFIO_MODE_MASK                       0xFFFFFFFFUL
#define MFIO_MODE_MFIO8_MODE_BIT_OFFSET      (24U)
#define MFIO_MODE_MFIO8_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO8_MODE_BIT_MASK        0x7000000UL
#define MFIO_MODE_MFIO7_MODE_BIT_OFFSET      (21U)
#define MFIO_MODE_MFIO7_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO7_MODE_BIT_MASK        0xe00000UL
#define MFIO_MODE_MFIO6_MODE_BIT_OFFSET      (18U)
#define MFIO_MODE_MFIO6_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO6_MODE_BIT_MASK        0x1c0000UL
#define MFIO_MODE_MFIO5_MODE_BIT_OFFSET      (15U)
#define MFIO_MODE_MFIO5_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO5_MODE_BIT_MASK        0x38000UL
#define MFIO_MODE_MFIO4_MODE_BIT_OFFSET      (12U)
#define MFIO_MODE_MFIO4_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO4_MODE_BIT_MASK        0x7000U
#define MFIO_MODE_MFIO3_MODE_BIT_OFFSET      (9U)
#define MFIO_MODE_MFIO3_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO3_MODE_BIT_MASK        0xe00U
#define MFIO_MODE_MFIO2_MODE_BIT_OFFSET      (6U)
#define MFIO_MODE_MFIO2_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO2_MODE_BIT_MASK        0x1c0U
#define MFIO_MODE_MFIO1_MODE_BIT_OFFSET      (3U)
#define MFIO_MODE_MFIO1_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO1_MODE_BIT_MASK        0x38U
#define MFIO_MODE_MFIO0_MODE_BIT_OFFSET      (0U)
#define MFIO_MODE_MFIO0_MODE_BIT_LEN         (3U)
#define MFIO_MODE_MFIO0_MODE_BIT_MASK        0x7U

/******************************************************************************
* @brief Bit definitions for register GPIO_PULL_EN
**/
#define GPIO_PULL_EN_ID                      0x50004
#define GPIO_PULL_EN_LEN                     (4U)
#define GPIO_PULL_EN_MASK                    0xFFFFFFFFUL
#define GPIO_PULL_EN_DIG_IO_DRV_BIT_OFFSET   (13U)
#define GPIO_PULL_EN_DIG_IO_DRV_BIT_LEN      (1U)
#define GPIO_PULL_EN_DIG_IO_DRV_BIT_MASK     0x2000U
#define GPIO_PULL_EN_DIG_MOSI_DRV_BIT_OFFSET (12U)
#define GPIO_PULL_EN_DIG_MOSI_DRV_BIT_LEN    (1U)
#define GPIO_PULL_EN_DIG_MOSI_DRV_BIT_MASK   0x1000U
#define GPIO_PULL_EN_MFIO_PULL_EN_BIT_OFFSET (0U)
#define GPIO_PULL_EN_MFIO_PULL_EN_BIT_LEN    (12U)
#define GPIO_PULL_EN_MFIO_PULL_EN_BIT_MASK   0xfffU

/******************************************************************************
* @brief Bit definitions for register GPIO_DIR
**/
#define GPIO_DIR_ID                          0x50008
#define GPIO_DIR_LEN                         (4U)
#define GPIO_DIR_MASK                        0xFFFFFFFFUL
#define GPIO_DIR_GPIO8_DIR_BIT_OFFSET        (8U)
#define GPIO_DIR_GPIO8_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO8_DIR_BIT_MASK          0x100U
#define GPIO_DIR_GPIO7_DIR_BIT_OFFSET        (7U)
#define GPIO_DIR_GPIO7_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO7_DIR_BIT_MASK          0x80U
#define GPIO_DIR_GPIO6_DIR_BIT_OFFSET        (6U)
#define GPIO_DIR_GPIO6_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO6_DIR_BIT_MASK          0x40U
#define GPIO_DIR_GPIO5_DIR_BIT_OFFSET        (5U)
#define GPIO_DIR_GPIO5_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO5_DIR_BIT_MASK          0x20U
#define GPIO_DIR_GPIO4_DIR_BIT_OFFSET        (4U)
#define GPIO_DIR_GPIO4_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO4_DIR_BIT_MASK          0x10U
#define GPIO_DIR_GPIO3_DIR_BIT_OFFSET        (3U)
#define GPIO_DIR_GPIO3_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO3_DIR_BIT_MASK          0x8U
#define GPIO_DIR_GPIO2_DIR_BIT_OFFSET        (2U)
#define GPIO_DIR_GPIO2_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO2_DIR_BIT_MASK          0x4U
#define GPIO_DIR_GPIO1_DIR_BIT_OFFSET        (1U)
#define GPIO_DIR_GPIO1_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO1_DIR_BIT_MASK          0x2U
#define GPIO_DIR_GPIO0_DIR_BIT_OFFSET        (0U)
#define GPIO_DIR_GPIO0_DIR_BIT_LEN           (1U)
#define GPIO_DIR_GPIO0_DIR_BIT_MASK          0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_OUT
**/
#define GPIO_OUT_ID                          0x5000c
#define GPIO_OUT_LEN                         (4U)
#define GPIO_OUT_MASK                        0xFFFFFFFFUL
#define GPIO_OUT_GPIO8_OUT_BIT_OFFSET        (8U)
#define GPIO_OUT_GPIO8_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO8_OUT_BIT_MASK          0x100U
#define GPIO_OUT_GPIO7_OUT_BIT_OFFSET        (7U)
#define GPIO_OUT_GPIO7_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO7_OUT_BIT_MASK          0x80U
#define GPIO_OUT_GPIO6_OUT_BIT_OFFSET        (6U)
#define GPIO_OUT_GPIO6_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO6_OUT_BIT_MASK          0x40U
#define GPIO_OUT_GPIO5_OUT_BIT_OFFSET        (5U)
#define GPIO_OUT_GPIO5_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO5_OUT_BIT_MASK          0x20U
#define GPIO_OUT_GPIO4_OUT_BIT_OFFSET        (4U)
#define GPIO_OUT_GPIO4_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO4_OUT_BIT_MASK          0x10U
#define GPIO_OUT_GPIO3_OUT_BIT_OFFSET        (3U)
#define GPIO_OUT_GPIO3_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO3_OUT_BIT_MASK          0x8U
#define GPIO_OUT_GPIO2_OUT_BIT_OFFSET        (2U)
#define GPIO_OUT_GPIO2_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO2_OUT_BIT_MASK          0x4U
#define GPIO_OUT_GPIO1_OUT_BIT_OFFSET        (1U)
#define GPIO_OUT_GPIO1_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO1_OUT_BIT_MASK          0x2U
#define GPIO_OUT_GPIO0_OUT_BIT_OFFSET        (0U)
#define GPIO_OUT_GPIO0_OUT_BIT_LEN           (1U)
#define GPIO_OUT_GPIO0_OUT_BIT_MASK          0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_INT_EN
**/
#define GPIO_INT_EN_ID                       0x50010
#define GPIO_INT_EN_LEN                      (4U)
#define GPIO_INT_EN_MASK                     0xFFFFFFFFUL
#define GPIO_INT_EN_GPIO8_INT_EN_BIT_OFFSET  (8U)
#define GPIO_INT_EN_GPIO8_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO8_INT_EN_BIT_MASK    0x100U
#define GPIO_INT_EN_GPIO7_INT_EN_BIT_OFFSET  (7U)
#define GPIO_INT_EN_GPIO7_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO7_INT_EN_BIT_MASK    0x80U
#define GPIO_INT_EN_GPIO6_INT_EN_BIT_OFFSET  (6U)
#define GPIO_INT_EN_GPIO6_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO6_INT_EN_BIT_MASK    0x40U
#define GPIO_INT_EN_GPIO5_INT_EN_BIT_OFFSET  (5U)
#define GPIO_INT_EN_GPIO5_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO5_INT_EN_BIT_MASK    0x20U
#define GPIO_INT_EN_GPIO4_INT_EN_BIT_OFFSET  (4U)
#define GPIO_INT_EN_GPIO4_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO4_INT_EN_BIT_MASK    0x10U
#define GPIO_INT_EN_GPIO3_INT_EN_BIT_OFFSET  (3U)
#define GPIO_INT_EN_GPIO3_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO3_INT_EN_BIT_MASK    0x8U
#define GPIO_INT_EN_GPIO2_INT_EN_BIT_OFFSET  (2U)
#define GPIO_INT_EN_GPIO2_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO2_INT_EN_BIT_MASK    0x4U
#define GPIO_INT_EN_GPIO1_INT_EN_BIT_OFFSET  (1U)
#define GPIO_INT_EN_GPIO1_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO1_INT_EN_BIT_MASK    0x2U
#define GPIO_INT_EN_GPIO0_INT_EN_BIT_OFFSET  (0U)
#define GPIO_INT_EN_GPIO0_INT_EN_BIT_LEN     (1U)
#define GPIO_INT_EN_GPIO0_INT_EN_BIT_MASK    0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_INT_STATUS
**/
#define GPIO_INT_STATUS_ID                   0x50014
#define GPIO_INT_STATUS_LEN                  (4U)
#define GPIO_INT_STATUS_MASK                 0xFFFFFFFFUL
#define GPIO_INT_STATUS_GPIO8_STATUS_BIT_OFFSET (8U)
#define GPIO_INT_STATUS_GPIO8_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO8_STATUS_BIT_MASK 0x100U
#define GPIO_INT_STATUS_GPIO7_STATUS_BIT_OFFSET (7U)
#define GPIO_INT_STATUS_GPIO7_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO7_STATUS_BIT_MASK 0x80U
#define GPIO_INT_STATUS_GPIO6_STATUS_BIT_OFFSET (6U)
#define GPIO_INT_STATUS_GPIO6_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO6_STATUS_BIT_MASK 0x40U
#define GPIO_INT_STATUS_GPIO5_STATUS_BIT_OFFSET (5U)
#define GPIO_INT_STATUS_GPIO5_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO5_STATUS_BIT_MASK 0x20U
#define GPIO_INT_STATUS_GPIO4_STATUS_BIT_OFFSET (4U)
#define GPIO_INT_STATUS_GPIO4_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO4_STATUS_BIT_MASK 0x10U
#define GPIO_INT_STATUS_GPIO3_STATUS_BIT_OFFSET (3U)
#define GPIO_INT_STATUS_GPIO3_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO3_STATUS_BIT_MASK 0x8U
#define GPIO_INT_STATUS_GPIO2_STATUS_BIT_OFFSET (2U)
#define GPIO_INT_STATUS_GPIO2_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO2_STATUS_BIT_MASK 0x4U
#define GPIO_INT_STATUS_GPIO1_STATUS_BIT_OFFSET (1U)
#define GPIO_INT_STATUS_GPIO1_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO1_STATUS_BIT_MASK 0x2U
#define GPIO_INT_STATUS_GPIO0_STATUS_BIT_OFFSET (0U)
#define GPIO_INT_STATUS_GPIO0_STATUS_BIT_LEN (1U)
#define GPIO_INT_STATUS_GPIO0_STATUS_BIT_MASK 0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_INT_EDGE
**/
#define GPIO_INT_EDGE_ID                     0x50018
#define GPIO_INT_EDGE_LEN                    (4U)
#define GPIO_INT_EDGE_MASK                   0xFFFFFFFFUL
#define GPIO_INT_EDGE_GPIO8_INT_EDGE_BIT_OFFSET (8U)
#define GPIO_INT_EDGE_GPIO8_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO8_INT_EDGE_BIT_MASK 0x100U
#define GPIO_INT_EDGE_GPIO7_INT_EDGE_BIT_OFFSET (7U)
#define GPIO_INT_EDGE_GPIO7_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO7_INT_EDGE_BIT_MASK 0x80U
#define GPIO_INT_EDGE_GPIO6_INT_EDGE_BIT_OFFSET (6U)
#define GPIO_INT_EDGE_GPIO6_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO6_INT_EDGE_BIT_MASK 0x40U
#define GPIO_INT_EDGE_GPIO5_INT_EDGE_BIT_OFFSET (5U)
#define GPIO_INT_EDGE_GPIO5_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO5_INT_EDGE_BIT_MASK 0x20U
#define GPIO_INT_EDGE_GPIO4_INT_EDGE_BIT_OFFSET (4U)
#define GPIO_INT_EDGE_GPIO4_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO4_INT_EDGE_BIT_MASK 0x10U
#define GPIO_INT_EDGE_GPIO3_INT_EDGE_BIT_OFFSET (3U)
#define GPIO_INT_EDGE_GPIO3_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO3_INT_EDGE_BIT_MASK 0x8U
#define GPIO_INT_EDGE_GPIO2_INT_EDGE_BIT_OFFSET (2U)
#define GPIO_INT_EDGE_GPIO2_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO2_INT_EDGE_BIT_MASK 0x4U
#define GPIO_INT_EDGE_GPIO1_INT_EDGE_BIT_OFFSET (1U)
#define GPIO_INT_EDGE_GPIO1_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO1_INT_EDGE_BIT_MASK 0x2U
#define GPIO_INT_EDGE_GPIO0_INT_EDGE_BIT_OFFSET (0U)
#define GPIO_INT_EDGE_GPIO0_INT_EDGE_BIT_LEN (1U)
#define GPIO_INT_EDGE_GPIO0_INT_EDGE_BIT_MASK 0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_INT_TYPE
**/
#define GPIO_INT_TYPE_ID                     0x5001c
#define GPIO_INT_TYPE_LEN                    (4U)
#define GPIO_INT_TYPE_MASK                   0xFFFFFFFFUL
#define GPIO_INT_TYPE_GPIO8_INT_TYPE_BIT_OFFSET (8U)
#define GPIO_INT_TYPE_GPIO8_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO8_INT_TYPE_BIT_MASK 0x100U
#define GPIO_INT_TYPE_GPIO7_INT_TYPE_BIT_OFFSET (7U)
#define GPIO_INT_TYPE_GPIO7_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO7_INT_TYPE_BIT_MASK 0x80U
#define GPIO_INT_TYPE_GPIO6_INT_TYPE_BIT_OFFSET (6U)
#define GPIO_INT_TYPE_GPIO6_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO6_INT_TYPE_BIT_MASK 0x40U
#define GPIO_INT_TYPE_GPIO5_INT_TYPE_BIT_OFFSET (5U)
#define GPIO_INT_TYPE_GPIO5_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO5_INT_TYPE_BIT_MASK 0x20U
#define GPIO_INT_TYPE_GPIO4_INT_TYPE_BIT_OFFSET (4U)
#define GPIO_INT_TYPE_GPIO4_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO4_INT_TYPE_BIT_MASK 0x10U
#define GPIO_INT_TYPE_GPIO3_INT_TYPE_BIT_OFFSET (3U)
#define GPIO_INT_TYPE_GPIO3_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO3_INT_TYPE_BIT_MASK 0x8U
#define GPIO_INT_TYPE_GPIO2_INT_TYPE_BIT_OFFSET (2U)
#define GPIO_INT_TYPE_GPIO2_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO2_INT_TYPE_BIT_MASK 0x4U
#define GPIO_INT_TYPE_GPIO1_INT_TYPE_BIT_OFFSET (1U)
#define GPIO_INT_TYPE_GPIO1_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO1_INT_TYPE_BIT_MASK 0x2U
#define GPIO_INT_TYPE_GPIO0_INT_TYPE_BIT_OFFSET (0U)
#define GPIO_INT_TYPE_GPIO0_INT_TYPE_BIT_LEN (1U)
#define GPIO_INT_TYPE_GPIO0_INT_TYPE_BIT_MASK 0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_INT_TYPE2
**/
#define GPIO_INT_TYPE2_ID                    0x50020
#define GPIO_INT_TYPE2_LEN                   (4U)
#define GPIO_INT_TYPE2_MASK                  0xFFFFFFFFUL
#define GPIO_INT_TYPE2_GPIO8_INT_TYPE2_BIT_OFFSET (8U)
#define GPIO_INT_TYPE2_GPIO8_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO8_INT_TYPE2_BIT_MASK 0x100U
#define GPIO_INT_TYPE2_GPIO7_INT_TYPE2_BIT_OFFSET (7U)
#define GPIO_INT_TYPE2_GPIO7_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO7_INT_TYPE2_BIT_MASK 0x80U
#define GPIO_INT_TYPE2_GPIO6_INT_TYPE2_BIT_OFFSET (6U)
#define GPIO_INT_TYPE2_GPIO6_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO6_INT_TYPE2_BIT_MASK 0x40U
#define GPIO_INT_TYPE2_GPIO5_INT_TYPE2_BIT_OFFSET (5U)
#define GPIO_INT_TYPE2_GPIO5_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO5_INT_TYPE2_BIT_MASK 0x20U
#define GPIO_INT_TYPE2_GPIO4_INT_TYPE2_BIT_OFFSET (4U)
#define GPIO_INT_TYPE2_GPIO4_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO4_INT_TYPE2_BIT_MASK 0x10U
#define GPIO_INT_TYPE2_GPIO3_INT_TYPE2_BIT_OFFSET (3U)
#define GPIO_INT_TYPE2_GPIO3_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO3_INT_TYPE2_BIT_MASK 0x8U
#define GPIO_INT_TYPE2_GPIO2_INT_TYPE2_BIT_OFFSET (2U)
#define GPIO_INT_TYPE2_GPIO2_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO2_INT_TYPE2_BIT_MASK 0x4U
#define GPIO_INT_TYPE2_GPIO1_INT_TYPE2_BIT_OFFSET (1U)
#define GPIO_INT_TYPE2_GPIO1_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO1_INT_TYPE2_BIT_MASK 0x2U
#define GPIO_INT_TYPE2_GPIO0_INT_TYPE2_BIT_OFFSET (0U)
#define GPIO_INT_TYPE2_GPIO0_INT_TYPE2_BIT_LEN (1U)
#define GPIO_INT_TYPE2_GPIO0_INT_TYPE2_BIT_MASK 0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_INT_CLR
**/
#define GPIO_INT_CLR_ID                      0x50024
#define GPIO_INT_CLR_LEN                     (4U)
#define GPIO_INT_CLR_MASK                    0xFFFFFFFFUL
#define GPIO_INT_CLR_GPIO8_INT_CLR_BIT_OFFSET (8U)
#define GPIO_INT_CLR_GPIO8_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO8_INT_CLR_BIT_MASK  0x100U
#define GPIO_INT_CLR_GPIO7_INT_CLR_BIT_OFFSET (7U)
#define GPIO_INT_CLR_GPIO7_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO7_INT_CLR_BIT_MASK  0x80U
#define GPIO_INT_CLR_GPIO6_INT_CLR_BIT_OFFSET (6U)
#define GPIO_INT_CLR_GPIO6_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO6_INT_CLR_BIT_MASK  0x40U
#define GPIO_INT_CLR_GPIO5_INT_CLR_BIT_OFFSET (5U)
#define GPIO_INT_CLR_GPIO5_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO5_INT_CLR_BIT_MASK  0x20U
#define GPIO_INT_CLR_GPIO4_INT_CLR_BIT_OFFSET (4U)
#define GPIO_INT_CLR_GPIO4_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO4_INT_CLR_BIT_MASK  0x10U
#define GPIO_INT_CLR_GPIO3_INT_CLR_BIT_OFFSET (3U)
#define GPIO_INT_CLR_GPIO3_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO3_INT_CLR_BIT_MASK  0x8U
#define GPIO_INT_CLR_GPIO2_INT_CLR_BIT_OFFSET (2U)
#define GPIO_INT_CLR_GPIO2_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO2_INT_CLR_BIT_MASK  0x4U
#define GPIO_INT_CLR_GPIO1_INT_CLR_BIT_OFFSET (1U)
#define GPIO_INT_CLR_GPIO1_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO1_INT_CLR_BIT_MASK  0x2U
#define GPIO_INT_CLR_GPIO0_INT_CLR_BIT_OFFSET (0U)
#define GPIO_INT_CLR_GPIO0_INT_CLR_BIT_LEN   (1U)
#define GPIO_INT_CLR_GPIO0_INT_CLR_BIT_MASK  0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_DBNC_EN
**/
#define GPIO_DBNC_EN_ID                      0x50028
#define GPIO_DBNC_EN_LEN                     (4U)
#define GPIO_DBNC_EN_MASK                    0xFFFFFFFFUL
#define GPIO_DBNC_EN_GPIO8_DBNC_EN_BIT_OFFSET (8U)
#define GPIO_DBNC_EN_GPIO8_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO8_DBNC_EN_BIT_MASK  0x100U
#define GPIO_DBNC_EN_GPIO7_DBNC_EN_BIT_OFFSET (7U)
#define GPIO_DBNC_EN_GPIO7_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO7_DBNC_EN_BIT_MASK  0x80U
#define GPIO_DBNC_EN_GPIO6_DBNC_EN_BIT_OFFSET (6U)
#define GPIO_DBNC_EN_GPIO6_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO6_DBNC_EN_BIT_MASK  0x40U
#define GPIO_DBNC_EN_GPIO5_DBNC_EN_BIT_OFFSET (5U)
#define GPIO_DBNC_EN_GPIO5_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO5_DBNC_EN_BIT_MASK  0x20U
#define GPIO_DBNC_EN_GPIO4_DBNC_EN_BIT_OFFSET (4U)
#define GPIO_DBNC_EN_GPIO4_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO4_DBNC_EN_BIT_MASK  0x10U
#define GPIO_DBNC_EN_GPIO3_DBNC_EN_BIT_OFFSET (3U)
#define GPIO_DBNC_EN_GPIO3_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO3_DBNC_EN_BIT_MASK  0x8U
#define GPIO_DBNC_EN_GPIO2_DBNC_EN_BIT_OFFSET (2U)
#define GPIO_DBNC_EN_GPIO2_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO2_DBNC_EN_BIT_MASK  0x4U
#define GPIO_DBNC_EN_GPIO1_DBNC_EN_BIT_OFFSET (1U)
#define GPIO_DBNC_EN_GPIO1_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO1_DBNC_EN_BIT_MASK  0x2U
#define GPIO_DBNC_EN_GPIO0_DBNC_EN_BIT_OFFSET (0U)
#define GPIO_DBNC_EN_GPIO0_DBNC_EN_BIT_LEN   (1U)
#define GPIO_DBNC_EN_GPIO0_DBNC_EN_BIT_MASK  0x1U

/******************************************************************************
* @brief Bit definitions for register GPIO_RAW_DATA
**/
#define GPIO_RAW_DATA_ID                     0x5002c
#define GPIO_RAW_DATA_LEN                    (4U)
#define GPIO_RAW_DATA_MASK                   0xFFFFFFFFUL
#define GPIO_RAW_DATA_GPIO8_RAW_DAT_BIT_OFFSET (8U)
#define GPIO_RAW_DATA_GPIO8_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO8_RAW_DAT_BIT_MASK 0x100U
#define GPIO_RAW_DATA_GPIO7_RAW_DAT_BIT_OFFSET (7U)
#define GPIO_RAW_DATA_GPIO7_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO7_RAW_DAT_BIT_MASK 0x80U
#define GPIO_RAW_DATA_GPIO6_RAW_DAT_BIT_OFFSET (6U)
#define GPIO_RAW_DATA_GPIO6_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO6_RAW_DAT_BIT_MASK 0x40U
#define GPIO_RAW_DATA_GPIO5_RAW_DAT_BIT_OFFSET (5U)
#define GPIO_RAW_DATA_GPIO5_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO5_RAW_DAT_BIT_MASK 0x20U
#define GPIO_RAW_DATA_GPIO4_RAW_DAT_BIT_OFFSET (4U)
#define GPIO_RAW_DATA_GPIO4_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO4_RAW_DAT_BIT_MASK 0x10U
#define GPIO_RAW_DATA_GPIO3_RAW_DAT_BIT_OFFSET (3U)
#define GPIO_RAW_DATA_GPIO3_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO3_RAW_DAT_BIT_MASK 0x8U
#define GPIO_RAW_DATA_GPIO2_RAW_DAT_BIT_OFFSET (2U)
#define GPIO_RAW_DATA_GPIO2_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO2_RAW_DAT_BIT_MASK 0x4U
#define GPIO_RAW_DATA_GPIO1_RAW_DAT_BIT_OFFSET (1U)
#define GPIO_RAW_DATA_GPIO1_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO1_RAW_DAT_BIT_MASK 0x2U
#define GPIO_RAW_DATA_GPIO0_RAW_DAT_BIT_OFFSET (0U)
#define GPIO_RAW_DATA_GPIO0_RAW_DAT_BIT_LEN  (1U)
#define GPIO_RAW_DATA_GPIO0_RAW_DAT_BIT_MASK 0x1U

/******************************************************************************
* @brief Bit definitions for register DTUNE0
**/
#define DTUNE0_ID                            0x60000
#define DTUNE0_LEN                           (4U)
#define DTUNE0_MASK                          0xFFFFFFFFUL
#define DTUNE0_RX_SFD_TOC_BIT_OFFSET         (16U)
#define DTUNE0_RX_SFD_TOC_BIT_LEN            (16U)
#define DTUNE0_RX_SFD_TOC_BIT_MASK           0xffff0000UL
#define DTUNE0_TR_SEED_GEAR_BIT_OFFSET       (8U)
#define DTUNE0_TR_SEED_GEAR_BIT_LEN          (8U)
#define DTUNE0_TR_SEED_GEAR_BIT_MASK         0xff00U
#define DTUNE0_RESERVED_BIT_OFFSET           (5U)
#define DTUNE0_RESERVED_BIT_LEN              (3U)
#define DTUNE0_RESERVED_BIT_MASK             0xe0U
#define DTUNE0_CP_CMF_EN_BIT_OFFSET          (4U)
#define DTUNE0_CP_CMF_EN_BIT_LEN             (1U)
#define DTUNE0_CP_CMF_EN_BIT_MASK            0x10U
#define DTUNE0_TIM_SEED_POL_BIT_OFFSET       (3U)
#define DTUNE0_TIM_SEED_POL_BIT_LEN          (1U)
#define DTUNE0_TIM_SEED_POL_BIT_MASK         0x8U
#define DTUNE0_PRE_QUAL_WIN_BIT_OFFSET       (2U)
#define DTUNE0_PRE_QUAL_WIN_BIT_LEN          (1U)
#define DTUNE0_PRE_QUAL_WIN_BIT_MASK         0x4U
#define DTUNE0_PRE_PAC_SYM_BIT_OFFSET        (0U)
#define DTUNE0_PRE_PAC_SYM_BIT_LEN           (2U)
#define DTUNE0_PRE_PAC_SYM_BIT_MASK          0x3U

/******************************************************************************
* @brief Bit definitions for register DTUNE1
**/
#define DTUNE1_ID                            0x60004
#define DTUNE1_LEN                           (4U)
#define DTUNE1_MASK                          0xFFFFFFFFUL
#define DTUNE1_CAR_INT_SEED_BIT_OFFSET       (20U)
#define DTUNE1_CAR_INT_SEED_BIT_LEN          (9U)
#define DTUNE1_CAR_INT_SEED_BIT_MASK         0x1ff00000UL
#define DTUNE1_RX_PRE_CONF_THD_BIT_OFFSET    (16U)
#define DTUNE1_RX_PRE_CONF_THD_BIT_LEN       (4U)
#define DTUNE1_RX_PRE_CONF_THD_BIT_MASK      0xf0000UL
#define DTUNE1_PRE_TOC_BIT_OFFSET            (0U)
#define DTUNE1_PRE_TOC_BIT_LEN               (16U)
#define DTUNE1_PRE_TOC_BIT_MASK              0xffffU

/******************************************************************************
* @brief Bit definitions for register DTUNE2
**/
#define DTUNE2_ID                            0x60008
#define DTUNE2_LEN                           (4U)
#define DTUNE2_MASK                          0xFFFFFFFFUL
#define DTUNE2_PRE_CORE_16_32_BIT_OFFSET     (23U)
#define DTUNE2_PRE_CORE_16_32_BIT_LEN        (9U)
#define DTUNE2_PRE_CORE_16_32_BIT_MASK       0xff800000UL
#define DTUNE2_PRE_CORE_16_16_BIT_OFFSET     (15U)
#define DTUNE2_PRE_CORE_16_16_BIT_LEN        (8U)
#define DTUNE2_PRE_CORE_16_16_BIT_MASK       0x7f8000UL
#define DTUNE2_PRE_CORE_16_8_BIT_OFFSET      (7U)
#define DTUNE2_PRE_CORE_16_8_BIT_LEN         (8U)
#define DTUNE2_PRE_CORE_16_8_BIT_MASK        0x7f80U
#define DTUNE2_PRE_CORE_16_4_BIT_OFFSET      (0U)
#define DTUNE2_PRE_CORE_16_4_BIT_LEN         (7U)
#define DTUNE2_PRE_CORE_16_4_BIT_MASK        0x7fU

/******************************************************************************
* @brief Bit definitions for register DTUNE3
**/
#define DTUNE3_ID                            0x6000c
#define DTUNE3_LEN                           (4U)
#define DTUNE3_MASK                          0xFFFFFFFFUL
#define DTUNE3_PRE_CORE_64_32_BIT_OFFSET     (23U)
#define DTUNE3_PRE_CORE_64_32_BIT_LEN        (9U)
#define DTUNE3_PRE_CORE_64_32_BIT_MASK       0xff800000UL
#define DTUNE3_PRE_CORE_64_16_BIT_OFFSET     (15U)
#define DTUNE3_PRE_CORE_64_16_BIT_LEN        (8U)
#define DTUNE3_PRE_CORE_64_16_BIT_MASK       0x7f8000UL
#define DTUNE3_PRE_CORE_64_8_BIT_OFFSET      (7U)
#define DTUNE3_PRE_CORE_64_8_BIT_LEN         (8U)
#define DTUNE3_PRE_CORE_64_8_BIT_MASK        0x7f80U
#define DTUNE3_PRE_CORE_64_4_BIT_OFFSET      (0U)
#define DTUNE3_PRE_CORE_64_4_BIT_LEN         (7U)
#define DTUNE3_PRE_CORE_64_4_BIT_MASK        0x7fU

/******************************************************************************
* @brief Bit definitions for register DTUNE4
**/
#define DTUNE4_ID                            0x60010
#define DTUNE4_LEN                           (4U)
#define DTUNE4_MASK                          0xFFFFFFFFUL
#define DTUNE4_RX_SFD_HLDOFF_BIT_OFFSET      (24U)
#define DTUNE4_RX_SFD_HLDOFF_BIT_LEN         (8U)
#define DTUNE4_RX_SFD_HLDOFF_BIT_MASK        0xff000000UL
#define DTUNE4_CP_CORE_16_BIT_OFFSET         (16U)
#define DTUNE4_CP_CORE_16_BIT_LEN            (8U)
#define DTUNE4_CP_CORE_16_BIT_MASK           0xff0000UL
#define DTUNE4_CONF_CORE_16_BIT_OFFSET       (8U)
#define DTUNE4_CONF_CORE_16_BIT_LEN          (8U)
#define DTUNE4_CONF_CORE_16_BIT_MASK         0xff00U
#define DTUNE4_RX_SFD_CORE_16_BIT_OFFSET     (0U)
#define DTUNE4_RX_SFD_CORE_16_BIT_LEN        (8U)
#define DTUNE4_RX_SFD_CORE_16_BIT_MASK       0xffU

/******************************************************************************
* @brief Bit definitions for register DTUNE5
**/
#define DTUNE5_ID                            0x60014
#define DTUNE5_LEN                           (4U)
#define DTUNE5_MASK                          0xFFFFFFFFUL
#define DTUNE5_RX_SFD_CORE_SCP_BIT_OFFSET    (24U)
#define DTUNE5_RX_SFD_CORE_SCP_BIT_LEN       (8U)
#define DTUNE5_RX_SFD_CORE_SCP_BIT_MASK      0xff000000UL
#define DTUNE5_CP_CORE_64_BIT_OFFSET         (16U)
#define DTUNE5_CP_CORE_64_BIT_LEN            (8U)
#define DTUNE5_CP_CORE_64_BIT_MASK           0xff0000UL
#define DTUNE5_CONF_CORE_64_BIT_OFFSET       (8U)
#define DTUNE5_CONF_CORE_64_BIT_LEN          (8U)
#define DTUNE5_CONF_CORE_64_BIT_MASK         0xff00U
#define DTUNE5_RX_SFD_CORE_64_BIT_OFFSET     (0U)
#define DTUNE5_RX_SFD_CORE_64_BIT_LEN        (8U)
#define DTUNE5_RX_SFD_CORE_64_BIT_MASK       0xffU

/******************************************************************************
* @brief Bit definitions for register DTUNE6
**/
#define DTUNE6_ID                            0x60018
#define DTUNE6_LEN                           (4U)
#define DTUNE6_MASK                          0xFFFFFFFFUL
#define DTUNE6_PREAMBLE_THD_BIT_OFFSET       (24U)
#define DTUNE6_PREAMBLE_THD_BIT_LEN          (8U)
#define DTUNE6_PREAMBLE_THD_BIT_MASK         0xff000000UL
#define DTUNE6_PRE_CORE_SCP_32_BIT_OFFSET    (16U)
#define DTUNE6_PRE_CORE_SCP_32_BIT_LEN       (8U)
#define DTUNE6_PRE_CORE_SCP_32_BIT_MASK      0xff0000UL
#define DTUNE6_PRE_CORE_SCP_16_BIT_OFFSET    (8U)
#define DTUNE6_PRE_CORE_SCP_16_BIT_LEN       (8U)
#define DTUNE6_PRE_CORE_SCP_16_BIT_MASK      0xff00U
#define DTUNE6_CONF_CORE_SCP_BIT_OFFSET      (0U)
#define DTUNE6_CONF_CORE_SCP_BIT_LEN         (8U)
#define DTUNE6_CONF_CORE_SCP_BIT_MASK        0xffU

/******************************************************************************
* @brief Bit definitions for register DTUNE7
**/
#define DTUNE7_ID                            0x6001c
#define DTUNE7_LEN                           (1U)
#define DTUNE7_MASK                          0xFFU
#define DTUNE7_CP_CORE_SCP_BIT_OFFSET        (0U)
#define DTUNE7_CP_CORE_SCP_BIT_LEN           (8U)
#define DTUNE7_CP_CORE_SCP_BIT_MASK          0xffU

/******************************************************************************
* @brief Bit definitions for register DRX_DIAG0
**/
#define DRX_DIAG0_ID                         0x6001d              /*  */
#define DRX_DIAG0_LEN                        (4U)
#define DRX_DIAG0_MASK                       0xFFFFFFFFUL
#define DRX_DIAG0_SYMS_ACC_ADJ_BIT_OFFSET    (8U)
#define DRX_DIAG0_SYMS_ACC_ADJ_BIT_LEN       (12U)
#define DRX_DIAG0_SYMS_ACC_ADJ_BIT_MASK      0xfff00UL
#define DRX_DIAG0_SFD_RSMP_DEL_BIT_OFFSET    (0U)
#define DRX_DIAG0_SFD_RSMP_DEL_BIT_LEN       (8U)
#define DRX_DIAG0_SFD_RSMP_DEL_BIT_MASK      0xffU

/******************************************************************************
* @brief Bit definitions for register DRX_DIAG1
**/
#define DRX_DIAG1_ID                         0x60021              /*  */
#define DRX_DIAG1_LEN                        (4U)
#define DRX_DIAG1_MASK                       0xFFFFFFFFUL
#define DRX_DIAG1_PRE_VAL_BIT_OFFSET         (15U)
#define DRX_DIAG1_PRE_VAL_BIT_LEN            (15U)
#define DRX_DIAG1_PRE_VAL_BIT_MASK           0x3fff8000UL
#define DRX_DIAG1_WIND_MAX_BIT_OFFSET        (0U)
#define DRX_DIAG1_WIND_MAX_BIT_LEN           (15U)
#define DRX_DIAG1_WIND_MAX_BIT_MASK          0x7fffU

/******************************************************************************
* @brief Bit definitions for register DRX_DIAG2
**/
#define DRX_DIAG2_ID                         0x60025              /*  */
#define DRX_DIAG2_LEN                        (4U)
#define DRX_DIAG2_MASK                       0xFFFFFFFFUL
#define DRX_DIAG2_SFD_MEAN_BIT_OFFSET        (12U)
#define DRX_DIAG2_SFD_MEAN_BIT_LEN           (17U)
#define DRX_DIAG2_SFD_MEAN_BIT_MASK          0x1ffff000UL
#define DRX_DIAG2_SYMS_ACC_BIT_OFFSET        (0U)
#define DRX_DIAG2_SYMS_ACC_BIT_LEN           (12U)
#define DRX_DIAG2_SYMS_ACC_BIT_MASK          0xfffU

/******************************************************************************
* @brief Bit definitions for register DRX_DIAG3
**/
#define DRX_DIAG3_ID                         0x60029              /*  */
#define DRX_DIAG3_LEN                        (4U)
#define DRX_DIAG3_MASK                       0xFFFFFFFFUL
#define DRX_DIAG3_CAR_PHASE_BIT_OFFSET       (24U)
#define DRX_DIAG3_CAR_PHASE_BIT_LEN          (7U)
#define DRX_DIAG3_CAR_PHASE_BIT_MASK         0x7f000000UL
#define DRX_DIAG3_CAR_INT_BIT_OFFSET         (0U)
#define DRX_DIAG3_CAR_INT_BIT_LEN            (21U)
#define DRX_DIAG3_CAR_INT_BIT_MASK           0x1fffffUL

/******************************************************************************
* @brief Bit definitions for register DRX_DIAG4
**/
#define DRX_DIAG4_ID                         0x6002d              /*  */
#define DRX_DIAG4_LEN                        (4U)
#define DRX_DIAG4_MASK                       0xFFFFFFFFUL
#define DRX_DIAG4_ACC_NORM_BIT_OFFSET        (12U)
#define DRX_DIAG4_ACC_NORM_BIT_LEN           (4U)
#define DRX_DIAG4_ACC_NORM_BIT_MASK          0xf000U
#define DRX_DIAG4_RX_ACC_NOSAT_BIT_OFFSET    (0U)
#define DRX_DIAG4_RX_ACC_NOSAT_BIT_LEN       (12U)
#define DRX_DIAG4_RX_ACC_NOSAT_BIT_MASK      0xfffU

/******************************************************************************
* @brief Bit definitions for register DRX_DIAG5
**/
#define DRX_DIAG5_ID                         0x60031              /*  */
#define DRX_DIAG5_LEN                        (4U)
#define DRX_DIAG5_MASK                       0xFFFFFFFFUL
#define DRX_DIAG5_CY_ROTATION_Q_BIT_OFFSET   (15U)
#define DRX_DIAG5_CY_ROTATION_Q_BIT_LEN      (5U)
#define DRX_DIAG5_CY_ROTATION_Q_BIT_MASK     0xf8000UL
#define DRX_DIAG5_CY_ROTATION_I_BIT_OFFSET   (10U)
#define DRX_DIAG5_CY_ROTATION_I_BIT_LEN      (5U)
#define DRX_DIAG5_CY_ROTATION_I_BIT_MASK     0x7c00U
#define DRX_DIAG5_IP_ROTATION_Q_BIT_OFFSET   (5U)
#define DRX_DIAG5_IP_ROTATION_Q_BIT_LEN      (5U)
#define DRX_DIAG5_IP_ROTATION_Q_BIT_MASK     0x3e0U
#define DRX_DIAG5_IP_ROTATION_I_BIT_OFFSET   (0U)
#define DRX_DIAG5_IP_ROTATION_I_BIT_LEN      (5U)
#define DRX_DIAG5_IP_ROTATION_I_BIT_MASK     0x1fU

/******************************************************************************
* @brief Bit definitions for register DRX_DIAG6
**/
#define DRX_DIAG6_ID                         0x60035              /*  */
#define DRX_DIAG6_LEN                        (4U)
#define DRX_DIAG6_MASK                       0xFFFFFFFFUL
#define DRX_DIAG6_PHR_CAR_INT_BIT_OFFSET     (0U)
#define DRX_DIAG6_PHR_CAR_INT_BIT_LEN        (21U)
#define DRX_DIAG6_PHR_CAR_INT_BIT_MASK       0x1fffffUL

/******************************************************************************
* @brief Bit definitions for register DSTAT
**/
#define DSTAT_ID                             0x60039              /*  */
#define DSTAT_LEN                            (4U)
#define DSTAT_MASK                           0xFFFFFFFFUL
#define DSTAT_RX_PHR_BITS_BIT_OFFSET         (6U)
#define DSTAT_RX_PHR_BITS_BIT_LEN            (19U)
#define DSTAT_RX_PHR_BITS_BIT_MASK           0x1ffffc0UL
#define DSTAT_RX_SFDL_BIT_OFFSET             (0U)
#define DSTAT_RX_SFDL_BIT_LEN                (6U)
#define DSTAT_RX_SFDL_BIT_MASK               0x3fU

/******************************************************************************
* @brief Bit definitions for register RF_OVR
**/
#define RF_OVR_ID                            0x70000
#define RF_OVR_LEN                           (4U)
#define RF_OVR_MASK                          0xFFFFFFFFUL
#define RF_OVR_TX_CH4_EN_BIT_OFFSET          (31U)
#define RF_OVR_TX_CH4_EN_BIT_LEN             (1U)
#define RF_OVR_TX_CH4_EN_BIT_MASK            0x80000000UL
#define RF_OVR_TX_CH3_EN_BIT_OFFSET          (30U)
#define RF_OVR_TX_CH3_EN_BIT_LEN             (1U)
#define RF_OVR_TX_CH3_EN_BIT_MASK            0x40000000UL
#define RF_OVR_TX_CH2_EN_BIT_OFFSET          (29U)
#define RF_OVR_TX_CH2_EN_BIT_LEN             (1U)
#define RF_OVR_TX_CH2_EN_BIT_MASK            0x20000000UL
#define RF_OVR_TX_CH1_EN_BIT_OFFSET          (28U)
#define RF_OVR_TX_CH1_EN_BIT_LEN             (1U)
#define RF_OVR_TX_CH1_EN_BIT_MASK            0x10000000UL
#define RF_OVR_PLL_RX_PRE_EN_BIT_OFFSET      (27U)
#define RF_OVR_PLL_RX_PRE_EN_BIT_LEN         (1U)
#define RF_OVR_PLL_RX_PRE_EN_BIT_MASK        0x8000000UL
#define RF_OVR_PLL_TX_PRE_EN_BIT_OFFSET      (26U)
#define RF_OVR_PLL_TX_PRE_EN_BIT_LEN         (1U)
#define RF_OVR_PLL_TX_PRE_EN_BIT_MASK        0x4000000UL
#define RF_OVR_TX_SW_EN_BIT_OFFSET           (25U)
#define RF_OVR_TX_SW_EN_BIT_LEN              (1U)
#define RF_OVR_TX_SW_EN_BIT_MASK             0x2000000UL
#define RF_OVR_RX9_SW_EN_BIT_OFFSET          (24U)
#define RF_OVR_RX9_SW_EN_BIT_LEN             (1U)
#define RF_OVR_RX9_SW_EN_BIT_MASK            0x1000000UL
#define RF_OVR_RX5_SW_EN_BIT_OFFSET          (23U)
#define RF_OVR_RX5_SW_EN_BIT_LEN             (1U)
#define RF_OVR_RX5_SW_EN_BIT_MASK            0x800000UL
#define RF_OVR_ED_EN_BIT_OFFSET              (22U)
#define RF_OVR_ED_EN_BIT_LEN                 (1U)
#define RF_OVR_ED_EN_BIT_MASK                0x400000UL
#define RF_OVR_PLL_TX_EN_BIT_OFFSET          (21U)
#define RF_OVR_PLL_TX_EN_BIT_LEN             (1U)
#define RF_OVR_PLL_TX_EN_BIT_MASK            0x200000UL
#define RF_OVR_PLL_RX_EN_BIT_OFFSET          (20U)
#define RF_OVR_PLL_RX_EN_BIT_LEN             (1U)
#define RF_OVR_PLL_RX_EN_BIT_MASK            0x100000UL
#define RF_OVR_PLL_CH9_BIT_OFFSET            (18U)
#define RF_OVR_PLL_CH9_BIT_LEN               (1U)
#define RF_OVR_PLL_CH9_BIT_MASK              0x40000UL
#define RF_OVR_PLL_CH5_BIT_OFFSET            (17U)
#define RF_OVR_PLL_CH5_BIT_LEN               (1U)
#define RF_OVR_PLL_CH5_BIT_MASK              0x20000UL
#define RF_OVR_PLL_LOOPCH_BIT_OFFSET         (16U)
#define RF_OVR_PLL_LOOPCH_BIT_LEN            (1U)
#define RF_OVR_PLL_LOOPCH_BIT_MASK           0x10000UL
#define RF_OVR_PLL_RST_N_BIT_OFFSET          (15U)
#define RF_OVR_PLL_RST_N_BIT_LEN             (1U)
#define RF_OVR_PLL_RST_N_BIT_MASK            0x8000U
#define RF_OVR_PLL_EN_BIT_OFFSET             (14U)
#define RF_OVR_PLL_EN_BIT_LEN                (1U)
#define RF_OVR_PLL_EN_BIT_MASK               0x4000U
#define RF_OVR_TX_CH5_BIT_OFFSET             (13U)
#define RF_OVR_TX_CH5_BIT_LEN                (1U)
#define RF_OVR_TX_CH5_BIT_MASK               0x2000U
#define RF_OVR_TX_EN_BIT_OFFSET              (12U)
#define RF_OVR_TX_EN_BIT_LEN                 (1U)
#define RF_OVR_TX_EN_BIT_MASK                0x1000U
#define RF_OVR_TX_EN_BUF_BIT_OFFSET          (11U)
#define RF_OVR_TX_EN_BUF_BIT_LEN             (1U)
#define RF_OVR_TX_EN_BUF_BIT_MASK            0x800U
#define RF_OVR_TX_BIAS_EN_BIT_OFFSET         (10U)
#define RF_OVR_TX_BIAS_EN_BIT_LEN            (1U)
#define RF_OVR_TX_BIAS_EN_BIT_MASK           0x400U
#define RF_OVR_LNA9_CORE_EN_BIT_OFFSET       (9U)
#define RF_OVR_LNA9_CORE_EN_BIT_LEN          (1U)
#define RF_OVR_LNA9_CORE_EN_BIT_MASK         0x200U
#define RF_OVR_LNA9_BIAS_EN_BIT_OFFSET       (8U)
#define RF_OVR_LNA9_BIAS_EN_BIT_LEN          (1U)
#define RF_OVR_LNA9_BIAS_EN_BIT_MASK         0x100U
#define RF_OVR_RXMIX9_EN_BIT_OFFSET          (7U)
#define RF_OVR_RXMIX9_EN_BIT_LEN             (1U)
#define RF_OVR_RXMIX9_EN_BIT_MASK            0x80U
#define RF_OVR_LNA5_CORE_EN_BIT_OFFSET       (6U)
#define RF_OVR_LNA5_CORE_EN_BIT_LEN          (1U)
#define RF_OVR_LNA5_CORE_EN_BIT_MASK         0x40U
#define RF_OVR_LNA5_BIAS_EN_BIT_OFFSET       (5U)
#define RF_OVR_LNA5_BIAS_EN_BIT_LEN          (1U)
#define RF_OVR_LNA5_BIAS_EN_BIT_MASK         0x20U
#define RF_OVR_RXMIX5_EN_BIT_OFFSET          (4U)
#define RF_OVR_RXMIX5_EN_BIT_LEN             (1U)
#define RF_OVR_RXMIX5_EN_BIT_MASK            0x10U
#define RF_OVR_MIXRC_EN_BIT_OFFSET           (3U)
#define RF_OVR_MIXRC_EN_BIT_LEN              (1U)
#define RF_OVR_MIXRC_EN_BIT_MASK             0x8U
#define RF_OVR_PGF_EN_BIT_OFFSET             (2U)
#define RF_OVR_PGF_EN_BIT_LEN                (1U)
#define RF_OVR_PGF_EN_BIT_MASK               0x4U
#define RF_OVR_ADCQ_EN_BIT_OFFSET            (1U)
#define RF_OVR_ADCQ_EN_BIT_LEN               (1U)
#define RF_OVR_ADCQ_EN_BIT_MASK              0x2U
#define RF_OVR_ADCI_EN_BIT_OFFSET            (0U)
#define RF_OVR_ADCI_EN_BIT_LEN               (1U)
#define RF_OVR_ADCI_EN_BIT_MASK              0x1U

/******************************************************************************
* @brief Bit definitions for register RF_CTRL_MASK
**/
#define RF_CTRL_MASK_ID                      0x70004
#define RF_CTRL_MASK_LEN                     (4U)
#define RF_CTRL_MASK_MASK                    0xFFFFFFFFUL
#define RF_CTRL_MASK_TX_CH4_EN_BIT_OFFSET    (31U)
#define RF_CTRL_MASK_TX_CH4_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_TX_CH4_EN_BIT_MASK      0x80000000UL
#define RF_CTRL_MASK_TX_CH3_EN_BIT_OFFSET    (30U)
#define RF_CTRL_MASK_TX_CH3_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_TX_CH3_EN_BIT_MASK      0x40000000UL
#define RF_CTRL_MASK_TX_CH2_EN_BIT_OFFSET    (29U)
#define RF_CTRL_MASK_TX_CH2_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_TX_CH2_EN_BIT_MASK      0x20000000UL
#define RF_CTRL_MASK_TX_CH1_EN_BIT_OFFSET    (28U)
#define RF_CTRL_MASK_TX_CH1_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_TX_CH1_EN_BIT_MASK      0x10000000UL
#define RF_CTRL_MASK_PLL_RX_PRE_EN_BIT_OFFSET (27U)
#define RF_CTRL_MASK_PLL_RX_PRE_EN_BIT_LEN   (1U)
#define RF_CTRL_MASK_PLL_RX_PRE_EN_BIT_MASK  0x8000000UL
#define RF_CTRL_MASK_PLL_TX_PRE_EN_BIT_OFFSET (26U)
#define RF_CTRL_MASK_PLL_TX_PRE_EN_BIT_LEN   (1U)
#define RF_CTRL_MASK_PLL_TX_PRE_EN_BIT_MASK  0x4000000UL
#define RF_CTRL_MASK_TX_SW_EN_BIT_OFFSET     (25U)
#define RF_CTRL_MASK_TX_SW_EN_BIT_LEN        (1U)
#define RF_CTRL_MASK_TX_SW_EN_BIT_MASK       0x2000000UL
#define RF_CTRL_MASK_RX9_SW_EN_BIT_OFFSET    (24U)
#define RF_CTRL_MASK_RX9_SW_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_RX9_SW_EN_BIT_MASK      0x1000000UL
#define RF_CTRL_MASK_RX5_SW_EN_BIT_OFFSET    (23U)
#define RF_CTRL_MASK_RX5_SW_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_RX5_SW_EN_BIT_MASK      0x800000UL
#define RF_CTRL_MASK_ED_EN_BIT_OFFSET        (22U)
#define RF_CTRL_MASK_ED_EN_BIT_LEN           (1U)
#define RF_CTRL_MASK_ED_EN_BIT_MASK          0x400000UL
#define RF_CTRL_MASK_PLL_TX_EN_BIT_OFFSET    (21U)
#define RF_CTRL_MASK_PLL_TX_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_PLL_TX_EN_BIT_MASK      0x200000UL
#define RF_CTRL_MASK_PLL_RX_EN_BIT_OFFSET    (20U)
#define RF_CTRL_MASK_PLL_RX_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_PLL_RX_EN_BIT_MASK      0x100000UL
#define RF_CTRL_MASK_PLL_CH9_BIT_OFFSET      (18U)
#define RF_CTRL_MASK_PLL_CH9_BIT_LEN         (1U)
#define RF_CTRL_MASK_PLL_CH9_BIT_MASK        0x40000UL
#define RF_CTRL_MASK_PLL_CH5_BIT_OFFSET      (17U)
#define RF_CTRL_MASK_PLL_CH5_BIT_LEN         (1U)
#define RF_CTRL_MASK_PLL_CH5_BIT_MASK        0x20000UL
#define RF_CTRL_MASK_PLL_LOOPCH_BIT_OFFSET   (16U)
#define RF_CTRL_MASK_PLL_LOOPCH_BIT_LEN      (1U)
#define RF_CTRL_MASK_PLL_LOOPCH_BIT_MASK     0x10000UL
#define RF_CTRL_MASK_PLL_RST_N_BIT_OFFSET    (15U)
#define RF_CTRL_MASK_PLL_RST_N_BIT_LEN       (1U)
#define RF_CTRL_MASK_PLL_RST_N_BIT_MASK      0x8000U
#define RF_CTRL_MASK_PLL_EN_BIT_OFFSET       (14U)
#define RF_CTRL_MASK_PLL_EN_BIT_LEN          (1U)
#define RF_CTRL_MASK_PLL_EN_BIT_MASK         0x4000U
#define RF_CTRL_MASK_TX_CH5_BIT_OFFSET       (13U)
#define RF_CTRL_MASK_TX_CH5_BIT_LEN          (1U)
#define RF_CTRL_MASK_TX_CH5_BIT_MASK         0x2000U
#define RF_CTRL_MASK_TX_EN_BIT_OFFSET        (12U)
#define RF_CTRL_MASK_TX_EN_BIT_LEN           (1U)
#define RF_CTRL_MASK_TX_EN_BIT_MASK          0x1000U
#define RF_CTRL_MASK_TX_EN_BUF_BIT_OFFSET    (11U)
#define RF_CTRL_MASK_TX_EN_BUF_BIT_LEN       (1U)
#define RF_CTRL_MASK_TX_EN_BUF_BIT_MASK      0x800U
#define RF_CTRL_MASK_TX_BIAS_EN_BIT_OFFSET   (10U)
#define RF_CTRL_MASK_TX_BIAS_EN_BIT_LEN      (1U)
#define RF_CTRL_MASK_TX_BIAS_EN_BIT_MASK     0x400U
#define RF_CTRL_MASK_LNA9_CORE_EN_BIT_OFFSET (9U)
#define RF_CTRL_MASK_LNA9_CORE_EN_BIT_LEN    (1U)
#define RF_CTRL_MASK_LNA9_CORE_EN_BIT_MASK   0x200U
#define RF_CTRL_MASK_LNA9_BIAS_EN_BIT_OFFSET (8U)
#define RF_CTRL_MASK_LNA9_BIAS_EN_BIT_LEN    (1U)
#define RF_CTRL_MASK_LNA9_BIAS_EN_BIT_MASK   0x100U
#define RF_CTRL_MASK_RXMIX9_EN_BIT_OFFSET    (7U)
#define RF_CTRL_MASK_RXMIX9_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_RXMIX9_EN_BIT_MASK      0x80U
#define RF_CTRL_MASK_LNA5_CORE_EN_BIT_OFFSET (6U)
#define RF_CTRL_MASK_LNA5_CORE_EN_BIT_LEN    (1U)
#define RF_CTRL_MASK_LNA5_CORE_EN_BIT_MASK   0x40U
#define RF_CTRL_MASK_LNA5_BIAS_EN_BIT_OFFSET (5U)
#define RF_CTRL_MASK_LNA5_BIAS_EN_BIT_LEN    (1U)
#define RF_CTRL_MASK_LNA5_BIAS_EN_BIT_MASK   0x20U
#define RF_CTRL_MASK_RXMIX5_EN_BIT_OFFSET    (4U)
#define RF_CTRL_MASK_RXMIX5_EN_BIT_LEN       (1U)
#define RF_CTRL_MASK_RXMIX5_EN_BIT_MASK      0x10U
#define RF_CTRL_MASK_MIXRC_EN_BIT_OFFSET     (3U)
#define RF_CTRL_MASK_MIXRC_EN_BIT_LEN        (1U)
#define RF_CTRL_MASK_MIXRC_EN_BIT_MASK       0x8U
#define RF_CTRL_MASK_PGF_EN_BIT_OFFSET       (2U)
#define RF_CTRL_MASK_PGF_EN_BIT_LEN          (1U)
#define RF_CTRL_MASK_PGF_EN_BIT_MASK         0x4U
#define RF_CTRL_MASK_ADCQ_EN_BIT_OFFSET      (1U)
#define RF_CTRL_MASK_ADCQ_EN_BIT_LEN         (1U)
#define RF_CTRL_MASK_ADCQ_EN_BIT_MASK        0x2U
#define RF_CTRL_MASK_ADCI_EN_BIT_OFFSET      (0U)
#define RF_CTRL_MASK_ADCI_EN_BIT_LEN         (1U)
#define RF_CTRL_MASK_ADCI_EN_BIT_MASK        0x1U

/******************************************************************************
* @brief Bit definitions for register RX_CTRL_LO
**/
#define RX_CTRL_LO_ID                        0x70008
#define RX_CTRL_LO_LEN                       (4U)
#define RX_CTRL_LO_MASK                      0xFFFFFFFFUL
#define RX_CTRL_LO_RX5_LNA2_FCBW_ADJ_BIT_OFFSET (29U)
#define RX_CTRL_LO_RX5_LNA2_FCBW_ADJ_BIT_LEN (2U)
#define RX_CTRL_LO_RX5_LNA2_FCBW_ADJ_BIT_MASK 0x60000000UL
#define RX_CTRL_LO_RX5_NOTCH2_ADJ_BIT_OFFSET (27U)
#define RX_CTRL_LO_RX5_NOTCH2_ADJ_BIT_LEN    (2U)
#define RX_CTRL_LO_RX5_NOTCH2_ADJ_BIT_MASK   0x18000000UL
#define RX_CTRL_LO_RX5_IPROG_LNA1_BIT_OFFSET (24U)
#define RX_CTRL_LO_RX5_IPROG_LNA1_BIT_LEN    (3U)
#define RX_CTRL_LO_RX5_IPROG_LNA1_BIT_MASK   0x7000000UL
#define RX_CTRL_LO_RX5_INDEPENDENTNOTCH1_EN_BIT_OFFSET (23U)
#define RX_CTRL_LO_RX5_INDEPENDENTNOTCH1_EN_BIT_LEN (1U)
#define RX_CTRL_LO_RX5_INDEPENDENTNOTCH1_EN_BIT_MASK 0x800000UL
#define RX_CTRL_LO_MIX_R_CODE_BIT_OFFSET     (21U)
#define RX_CTRL_LO_MIX_R_CODE_BIT_LEN        (2U)
#define RX_CTRL_LO_MIX_R_CODE_BIT_MASK       0x600000UL
#define RX_CTRL_LO_MIX_C_CODE_BIT_OFFSET     (16U)
#define RX_CTRL_LO_MIX_C_CODE_BIT_LEN        (5U)
#define RX_CTRL_LO_MIX_C_CODE_BIT_MASK       0x1f0000UL
#define RX_CTRL_LO_PGF_RPOLY_TRIM_BIT_OFFSET (11U)
#define RX_CTRL_LO_PGF_RPOLY_TRIM_BIT_LEN    (5U)
#define RX_CTRL_LO_PGF_RPOLY_TRIM_BIT_MASK   0xf800U
#define RX_CTRL_LO_PGF_PTAT_SEL_BIT_OFFSET   (10U)
#define RX_CTRL_LO_PGF_PTAT_SEL_BIT_LEN      (1U)
#define RX_CTRL_LO_PGF_PTAT_SEL_BIT_MASK     0x400U
#define RX_CTRL_LO_PGF_BUF_BIAS_BIT_OFFSET   (8U)
#define RX_CTRL_LO_PGF_BUF_BIAS_BIT_LEN      (2U)
#define RX_CTRL_LO_PGF_BUF_BIAS_BIT_MASK     0x300U
#define RX_CTRL_LO_PGF_VCM_IN_SEL_BIT_OFFSET (6U)
#define RX_CTRL_LO_PGF_VCM_IN_SEL_BIT_LEN    (2U)
#define RX_CTRL_LO_PGF_VCM_IN_SEL_BIT_MASK   0xc0U
#define RX_CTRL_LO_PGF_VCM_OUT_SEL_BIT_OFFSET (4U)
#define RX_CTRL_LO_PGF_VCM_OUT_SEL_BIT_LEN   (2U)
#define RX_CTRL_LO_PGF_VCM_OUT_SEL_BIT_MASK  0x30U
#define RX_CTRL_LO_ADC_BIAS_CTRL_BIT_OFFSET  (2U)
#define RX_CTRL_LO_ADC_BIAS_CTRL_BIT_LEN     (2U)
#define RX_CTRL_LO_ADC_BIAS_CTRL_BIT_MASK    0xcU
#define RX_CTRL_LO_ADC_VCM_CTRL_BIT_OFFSET   (0U)
#define RX_CTRL_LO_ADC_VCM_CTRL_BIT_LEN      (2U)
#define RX_CTRL_LO_ADC_VCM_CTRL_BIT_MASK     0x3U

/******************************************************************************
* @brief Bit definitions for register RX_CTRL_MID
**/
#define RX_CTRL_MID_ID                       0x7000c
#define RX_CTRL_MID_LEN                      (4U)
#define RX_CTRL_MID_MASK                     0xFFFFFFFFUL
#define RX_CTRL_MID_RX5_NOTCH1_ADJ_BIT_OFFSET (28U)
#define RX_CTRL_MID_RX5_NOTCH1_ADJ_BIT_LEN   (2U)
#define RX_CTRL_MID_RX5_NOTCH1_ADJ_BIT_MASK  0x30000000UL
#define RX_CTRL_MID_RX5_INDEPENDENTLOAD2_EN_BIT_OFFSET (27U)
#define RX_CTRL_MID_RX5_INDEPENDENTLOAD2_EN_BIT_LEN (1U)
#define RX_CTRL_MID_RX5_INDEPENDENTLOAD2_EN_BIT_MASK 0x8000000UL
#define RX_CTRL_MID_RX5_LNA_LOAD2_PROG_BIT_OFFSET (22U)
#define RX_CTRL_MID_RX5_LNA_LOAD2_PROG_BIT_LEN (5U)
#define RX_CTRL_MID_RX5_LNA_LOAD2_PROG_BIT_MASK 0x7c00000UL
#define RX_CTRL_MID_RX5_INDEPENDENTNOTCH2_EN_BIT_OFFSET (21U)
#define RX_CTRL_MID_RX5_INDEPENDENTNOTCH2_EN_BIT_LEN (1U)
#define RX_CTRL_MID_RX5_INDEPENDENTNOTCH2_EN_BIT_MASK 0x200000UL
#define RX_CTRL_MID_RX5_LNA_NOTCH2_PROG_BIT_OFFSET (16U)
#define RX_CTRL_MID_RX5_LNA_NOTCH2_PROG_BIT_LEN (5U)
#define RX_CTRL_MID_RX5_LNA_NOTCH2_PROG_BIT_MASK 0x1f0000UL
#define RX_CTRL_MID_RX5_LNA_LOAD1_PROG_BIT_OFFSET (11U)
#define RX_CTRL_MID_RX5_LNA_LOAD1_PROG_BIT_LEN (5U)
#define RX_CTRL_MID_RX5_LNA_LOAD1_PROG_BIT_MASK 0xf800U
#define RX_CTRL_MID_RX5_LNA1_FCBW_ADJ_BIT_OFFSET (9U)
#define RX_CTRL_MID_RX5_LNA1_FCBW_ADJ_BIT_LEN (2U)
#define RX_CTRL_MID_RX5_LNA1_FCBW_ADJ_BIT_MASK 0x600U
#define RX_CTRL_MID_RX5_IPROG_LNA2_BIT_OFFSET (5U)
#define RX_CTRL_MID_RX5_IPROG_LNA2_BIT_LEN   (4U)
#define RX_CTRL_MID_RX5_IPROG_LNA2_BIT_MASK  0x1e0U
#define RX_CTRL_MID_RX5_LNA_NOTCH1_PROG_BIT_OFFSET (0U)
#define RX_CTRL_MID_RX5_LNA_NOTCH1_PROG_BIT_LEN (5U)
#define RX_CTRL_MID_RX5_LNA_NOTCH1_PROG_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register RX_CTRL_HI
**/
#define RX_CTRL_HI_ID                        0x70010
#define RX_CTRL_HI_LEN                       (4U)
#define RX_CTRL_HI_MASK                      0xFFFFFFFFUL
#define RX_CTRL_HI_RX9_RPROG_LV_BIT_OFFSET   (25U)
#define RX_CTRL_HI_RX9_RPROG_LV_BIT_LEN      (4U)
#define RX_CTRL_HI_RX9_RPROG_LV_BIT_MASK     0x1e000000UL
#define RX_CTRL_HI_RX9_LOADLNA_PROG1_BIT_OFFSET (20U)
#define RX_CTRL_HI_RX9_LOADLNA_PROG1_BIT_LEN (5U)
#define RX_CTRL_HI_RX9_LOADLNA_PROG1_BIT_MASK 0x1f00000UL
#define RX_CTRL_HI_RX9_LOADLNA_PROG2_BIT_OFFSET (15U)
#define RX_CTRL_HI_RX9_LOADLNA_PROG2_BIT_LEN (5U)
#define RX_CTRL_HI_RX9_LOADLNA_PROG2_BIT_MASK 0xf8000UL
#define RX_CTRL_HI_RX9_LNA1_FCBW_ADJ_BIT_OFFSET (13U)
#define RX_CTRL_HI_RX9_LNA1_FCBW_ADJ_BIT_LEN (2U)
#define RX_CTRL_HI_RX9_LNA1_FCBW_ADJ_BIT_MASK 0x6000U
#define RX_CTRL_HI_RX9_LNA2_FCBW_ADJ_BIT_OFFSET (11U)
#define RX_CTRL_HI_RX9_LNA2_FCBW_ADJ_BIT_LEN (2U)
#define RX_CTRL_HI_RX9_LNA2_FCBW_ADJ_BIT_MASK 0x1800U
#define RX_CTRL_HI_RX9_IPROG_LNA1_BIT_OFFSET (4U)
#define RX_CTRL_HI_RX9_IPROG_LNA1_BIT_LEN    (3U)
#define RX_CTRL_HI_RX9_IPROG_LNA1_BIT_MASK   0x70U
#define RX_CTRL_HI_RX9_IPROG_LNA2_BIT_OFFSET (0U)
#define RX_CTRL_HI_RX9_IPROG_LNA2_BIT_LEN    (4U)
#define RX_CTRL_HI_RX9_IPROG_LNA2_BIT_MASK   0xfU

/******************************************************************************
* @brief Bit definitions for register PDOA_CTRL
**/
#define PDOA_CTRL_ID                         0x70014
#define PDOA_CTRL_LEN                        (4U)
#define PDOA_CTRL_MASK                       0xFFFFFFFFUL
#define PDOA_CTRL_TXRX_SW_OVR_CTRL_BIT_OFFSET (24U)
#define PDOA_CTRL_TXRX_SW_OVR_CTRL_BIT_LEN   (6U)
#define PDOA_CTRL_TXRX_SW_OVR_CTRL_BIT_MASK  0x3f000000UL
#define PDOA_CTRL_TXRX_SW_OVR_EN_BIT_OFFSET  (16U)
#define PDOA_CTRL_TXRX_SW_OVR_EN_BIT_LEN     (1U)
#define PDOA_CTRL_TXRX_SW_OVR_EN_BIT_MASK    0x10000UL
#define PDOA_CTRL_ANT_EXT_DEFAULT_PORT_BIT_OFFSET (15U)
#define PDOA_CTRL_ANT_EXT_DEFAULT_PORT_BIT_LEN (1U)
#define PDOA_CTRL_ANT_EXT_DEFAULT_PORT_BIT_MASK 0x8000U
#define PDOA_CTRL_ANT_SW_OVR_CTRL_BIT_OFFSET (12U)
#define PDOA_CTRL_ANT_SW_OVR_CTRL_BIT_LEN    (3U)
#define PDOA_CTRL_ANT_SW_OVR_CTRL_BIT_MASK   0x7000U
#define PDOA_CTRL_ANT_SW_OVR_EN_BIT_OFFSET   (8U)
#define PDOA_CTRL_ANT_SW_OVR_EN_BIT_LEN      (1U)
#define PDOA_CTRL_ANT_SW_OVR_EN_BIT_MASK     0x100U
#define PDOA_CTRL_ANT_DEFAULT_PORT_BIT_OFFSET (1U)
#define PDOA_CTRL_ANT_DEFAULT_PORT_BIT_LEN   (1U)
#define PDOA_CTRL_ANT_DEFAULT_PORT_BIT_MASK  0x2U
#define PDOA_CTRL_ANT_SW_NO_TOGGLE_BIT_OFFSET (0U)
#define PDOA_CTRL_ANT_SW_NO_TOGGLE_BIT_LEN   (1U)
#define PDOA_CTRL_ANT_SW_NO_TOGGLE_BIT_MASK  0x1U

/******************************************************************************
* @brief Bit definitions for register TX_CTRL_LO
**/
#define TX_CTRL_LO_ID                        0x70018
#define TX_CTRL_LO_LEN                       (4U)
#define TX_CTRL_LO_MASK                      0xFFFFFFFFUL
#define TX_CTRL_LO_TX_BLEED_CTRL_BIT_OFFSET  (25U)
#define TX_CTRL_LO_TX_BLEED_CTRL_BIT_LEN     (3U)
#define TX_CTRL_LO_TX_BLEED_CTRL_BIT_MASK    0xe000000UL
#define TX_CTRL_LO_TX_LOBUF_CTRL_BIT_OFFSET  (20U)
#define TX_CTRL_LO_TX_LOBUF_CTRL_BIT_LEN     (5U)
#define TX_CTRL_LO_TX_LOBUF_CTRL_BIT_MASK    0x1f00000UL
#define TX_CTRL_LO_TX_VBULK_CTRL_BIT_OFFSET  (18U)
#define TX_CTRL_LO_TX_VBULK_CTRL_BIT_LEN     (2U)
#define TX_CTRL_LO_TX_VBULK_CTRL_BIT_MASK    0xc0000UL
#define TX_CTRL_LO_TX_VCASC_CTRL_BIT_OFFSET  (16U)
#define TX_CTRL_LO_TX_VCASC_CTRL_BIT_LEN     (2U)
#define TX_CTRL_LO_TX_VCASC_CTRL_BIT_MASK    0x30000UL
#define TX_CTRL_LO_TX_VCM_CTRL_BIT_OFFSET    (8U)
#define TX_CTRL_LO_TX_VCM_CTRL_BIT_LEN       (8U)
#define TX_CTRL_LO_TX_VCM_CTRL_BIT_MASK      0xff00U
#define TX_CTRL_LO_TX_DELAY_SEL_BIT_OFFSET   (6U)
#define TX_CTRL_LO_TX_DELAY_SEL_BIT_LEN      (2U)
#define TX_CTRL_LO_TX_DELAY_SEL_BIT_MASK     0xc0U
#define TX_CTRL_LO_TX_CF_CTRL_BIT_OFFSET     (1U)
#define TX_CTRL_LO_TX_CF_CTRL_BIT_LEN        (5U)
#define TX_CTRL_LO_TX_CF_CTRL_BIT_MASK       0x3eU
#define TX_CTRL_LO_TX_CF_FORCE_BIT_OFFSET    (0U)
#define TX_CTRL_LO_TX_CF_FORCE_BIT_LEN       (1U)
#define TX_CTRL_LO_TX_CF_FORCE_BIT_MASK      0x1U

/******************************************************************************
* @brief Bit definitions for register TX_CTRL_HI
**/
#define TX_CTRL_HI_ID                        0x7001c
#define TX_CTRL_HI_LEN                       (4U)
#define TX_CTRL_HI_MASK                      0xFFFFFFFFUL
#define TX_CTRL_HI_TX_PULSE_SHAPE_BIT_OFFSET (31U)
#define TX_CTRL_HI_TX_PULSE_SHAPE_BIT_LEN    (1U)
#define TX_CTRL_HI_TX_PULSE_SHAPE_BIT_MASK   0x80000000UL
#define TX_CTRL_HI_TX_OFF_SW_STATE_BIT_OFFSET (23U)
#define TX_CTRL_HI_TX_OFF_SW_STATE_BIT_LEN   (6U)
#define TX_CTRL_HI_TX_OFF_SW_STATE_BIT_MASK  0x1f800000UL
#define TX_CTRL_HI_TX_OFF_SW_DLY_BIT_OFFSET  (21U)
#define TX_CTRL_HI_TX_OFF_SW_DLY_BIT_LEN     (2U)
#define TX_CTRL_HI_TX_OFF_SW_DLY_BIT_MASK    0x600000UL
#define TX_CTRL_HI_TX_CTUNE_LO_BIT_OFFSET    (16U)
#define TX_CTRL_HI_TX_CTUNE_LO_BIT_LEN       (4U)
#define TX_CTRL_HI_TX_CTUNE_LO_BIT_MASK      0xf0000UL
#define TX_CTRL_HI_TX_CTUNE_LOAD_P_BIT_OFFSET (12U)
#define TX_CTRL_HI_TX_CTUNE_LOAD_P_BIT_LEN   (4U)
#define TX_CTRL_HI_TX_CTUNE_LOAD_P_BIT_MASK  0xf000U
#define TX_CTRL_HI_TX_CTUNE_LOAD_M_BIT_OFFSET (8U)
#define TX_CTRL_HI_TX_CTUNE_LOAD_M_BIT_LEN   (4U)
#define TX_CTRL_HI_TX_CTUNE_LOAD_M_BIT_MASK  0xf00U
#define TX_CTRL_HI_TX_PG_START_NUM_BIT_OFFSET (6U)
#define TX_CTRL_HI_TX_PG_START_NUM_BIT_LEN   (2U)
#define TX_CTRL_HI_TX_PG_START_NUM_BIT_MASK  0xc0U
#define TX_CTRL_HI_TX_PG_DELAY_BIT_OFFSET    (0U)
#define TX_CTRL_HI_TX_PG_DELAY_BIT_LEN       (6U)
#define TX_CTRL_HI_TX_PG_DELAY_BIT_MASK      0x3fU

/******************************************************************************
* @brief Bit definitions for register ED_CTRL
**/
#define ED_CTRL_ID                           0x70020
#define ED_CTRL_LEN                          (4U)
#define ED_CTRL_MASK                         0xFFFFFFFFUL
#define ED_CTRL_DIG_ENDET1_EN_BUFF_X2GAIN_ULV_BIT_OFFSET (31U)
#define ED_CTRL_DIG_ENDET1_EN_BUFF_X2GAIN_ULV_BIT_LEN (1U)
#define ED_CTRL_DIG_ENDET1_EN_BUFF_X2GAIN_ULV_BIT_MASK 0x80000000UL
#define ED_CTRL_DIG_ENDET1_DDA_GAIN_ULV_BIT_OFFSET (25U)
#define ED_CTRL_DIG_ENDET1_DDA_GAIN_ULV_BIT_LEN (6U)
#define ED_CTRL_DIG_ENDET1_DDA_GAIN_ULV_BIT_MASK 0x7e000000UL
#define ED_CTRL_DIG_ENDET1_DDA_OS_TRIM_ULV_BIT_OFFSET (16U)
#define ED_CTRL_DIG_ENDET1_DDA_OS_TRIM_ULV_BIT_LEN (9U)
#define ED_CTRL_DIG_ENDET1_DDA_OS_TRIM_ULV_BIT_MASK 0x1ff0000UL
#define ED_CTRL_DIG_ENDET0_EN_BUFF_X2GAIN_ULV_BIT_OFFSET (15U)
#define ED_CTRL_DIG_ENDET0_EN_BUFF_X2GAIN_ULV_BIT_LEN (1U)
#define ED_CTRL_DIG_ENDET0_EN_BUFF_X2GAIN_ULV_BIT_MASK 0x8000U
#define ED_CTRL_DIG_ENDET0_DDA_GAIN_ULV_BIT_OFFSET (9U)
#define ED_CTRL_DIG_ENDET0_DDA_GAIN_ULV_BIT_LEN (6U)
#define ED_CTRL_DIG_ENDET0_DDA_GAIN_ULV_BIT_MASK 0x7e00U
#define ED_CTRL_DIG_ENDET0_DDA_OS_TRIM_ULV_BIT_OFFSET (0U)
#define ED_CTRL_DIG_ENDET0_DDA_OS_TRIM_ULV_BIT_LEN (9U)
#define ED_CTRL_DIG_ENDET0_DDA_OS_TRIM_ULV_BIT_MASK 0x1ffU

/******************************************************************************
* @brief Bit definitions for register RX_TEST
**/
#define RX_TEST_ID                           0x70024
#define RX_TEST_LEN                          (4U)
#define RX_TEST_MASK                         0xFFFFFFFFUL
#define RX_TEST_PLL_TEST_SEL_BIT_OFFSET      (30U)
#define RX_TEST_PLL_TEST_SEL_BIT_LEN         (2U)
#define RX_TEST_PLL_TEST_SEL_BIT_MASK        0xc0000000UL
#define RX_TEST_RXCH9_DCTST_EN_BIT_OFFSET    (29U)
#define RX_TEST_RXCH9_DCTST_EN_BIT_LEN       (1U)
#define RX_TEST_RXCH9_DCTST_EN_BIT_MASK      0x20000000UL
#define RX_TEST_RXCH9_DCTST_SEL_BIT_OFFSET   (27U)
#define RX_TEST_RXCH9_DCTST_SEL_BIT_LEN      (2U)
#define RX_TEST_RXCH9_DCTST_SEL_BIT_MASK     0x18000000UL
#define RX_TEST_RXCH5_DCTST_EN_BIT_OFFSET    (26U)
#define RX_TEST_RXCH5_DCTST_EN_BIT_LEN       (1U)
#define RX_TEST_RXCH5_DCTST_EN_BIT_MASK      0x4000000UL
#define RX_TEST_RXCH5_DCTST_SEL_BIT_OFFSET   (24U)
#define RX_TEST_RXCH5_DCTST_SEL_BIT_LEN      (2U)
#define RX_TEST_RXCH5_DCTST_SEL_BIT_MASK     0x3000000UL
#define RX_TEST_ADC_CLK_EDGE_I_BIT_OFFSET    (23U)
#define RX_TEST_ADC_CLK_EDGE_I_BIT_LEN       (1U)
#define RX_TEST_ADC_CLK_EDGE_I_BIT_MASK      0x800000UL
#define RX_TEST_ADC_CLK_EDGE_Q_BIT_OFFSET    (22U)
#define RX_TEST_ADC_CLK_EDGE_Q_BIT_LEN       (1U)
#define RX_TEST_ADC_CLK_EDGE_Q_BIT_MASK      0x400000UL
#define RX_TEST_ADC_DIG_TEST_IN_BIT_OFFSET   (21U)
#define RX_TEST_ADC_DIG_TEST_IN_BIT_LEN      (1U)
#define RX_TEST_ADC_DIG_TEST_IN_BIT_MASK     0x200000UL
#define RX_TEST_PLL_EN_TST_125CLK_OUT_BIT_OFFSET (20U)
#define RX_TEST_PLL_EN_TST_125CLK_OUT_BIT_LEN (1U)
#define RX_TEST_PLL_EN_TST_125CLK_OUT_BIT_MASK 0x100000UL
#define RX_TEST_PLL_EN_TST_BUFF_OUT_BIT_OFFSET (19U)
#define RX_TEST_PLL_EN_TST_BUFF_OUT_BIT_LEN  (1U)
#define RX_TEST_PLL_EN_TST_BUFF_OUT_BIT_MASK 0x80000UL
#define RX_TEST_RXMIX_IPTST_PGF_SEL_BIT_OFFSET (17U)
#define RX_TEST_RXMIX_IPTST_PGF_SEL_BIT_LEN  (2U)
#define RX_TEST_RXMIX_IPTST_PGF_SEL_BIT_MASK 0x60000UL
#define RX_TEST_RXMIX_ACTST_MIX_SEL_BIT_OFFSET (15U)
#define RX_TEST_RXMIX_ACTST_MIX_SEL_BIT_LEN  (2U)
#define RX_TEST_RXMIX_ACTST_MIX_SEL_BIT_MASK 0x18000UL
#define RX_TEST_RXMIX_ACTST_PGF_SEL_BIT_OFFSET (13U)
#define RX_TEST_RXMIX_ACTST_PGF_SEL_BIT_LEN  (2U)
#define RX_TEST_RXMIX_ACTST_PGF_SEL_BIT_MASK 0x6000U
#define RX_TEST_PGF_AC_BYP_EN_BIT_OFFSET     (12U)
#define RX_TEST_PGF_AC_BYP_EN_BIT_LEN        (1U)
#define RX_TEST_PGF_AC_BYP_EN_BIT_MASK       0x1000U
#define RX_TEST_PGF_AC_SHORT_EN_BIT_OFFSET   (11U)
#define RX_TEST_PGF_AC_SHORT_EN_BIT_LEN      (1U)
#define RX_TEST_PGF_AC_SHORT_EN_BIT_MASK     0x800U
#define RX_TEST_PGF_HF_TEST_EN_BIT_OFFSET    (10U)
#define RX_TEST_PGF_HF_TEST_EN_BIT_LEN       (1U)
#define RX_TEST_PGF_HF_TEST_EN_BIT_MASK      0x400U
#define RX_TEST_PGF_Q_DC_TEST_EN_BIT_OFFSET  (9U)
#define RX_TEST_PGF_Q_DC_TEST_EN_BIT_LEN     (1U)
#define RX_TEST_PGF_Q_DC_TEST_EN_BIT_MASK    0x200U
#define RX_TEST_PGF_I_DC_TEST_EN_BIT_OFFSET  (8U)
#define RX_TEST_PGF_I_DC_TEST_EN_BIT_LEN     (1U)
#define RX_TEST_PGF_I_DC_TEST_EN_BIT_MASK    0x100U
#define RX_TEST_PGF_TEST_DC_BIT_OFFSET       (5U)
#define RX_TEST_PGF_TEST_DC_BIT_LEN          (3U)
#define RX_TEST_PGF_TEST_DC_BIT_MASK         0xe0U
#define RX_TEST_ADC_TEST_CTRL_BIT_OFFSET     (2U)
#define RX_TEST_ADC_TEST_CTRL_BIT_LEN        (3U)
#define RX_TEST_ADC_TEST_CTRL_BIT_MASK       0x1cU
#define RX_TEST_ADC_ANATEST_CTRL_BIT_OFFSET  (0U)
#define RX_TEST_ADC_ANATEST_CTRL_BIT_LEN     (2U)
#define RX_TEST_ADC_ANATEST_CTRL_BIT_MASK    0x3U

/******************************************************************************
* @brief Bit definitions for register TX_TEST
**/
#define TX_TEST_ID                           0x70028
#define TX_TEST_LEN                          (4U)
#define TX_TEST_MASK                         0xFFFFFFFFUL
#define TX_TEST_PGTEST_EN_CH4_BIT_OFFSET     (27U)
#define TX_TEST_PGTEST_EN_CH4_BIT_LEN        (1U)
#define TX_TEST_PGTEST_EN_CH4_BIT_MASK       0x8000000UL
#define TX_TEST_PGTEST_EN_CH3_BIT_OFFSET     (26U)
#define TX_TEST_PGTEST_EN_CH3_BIT_LEN        (1U)
#define TX_TEST_PGTEST_EN_CH3_BIT_MASK       0x4000000UL
#define TX_TEST_PGTEST_EN_CH2_BIT_OFFSET     (25U)
#define TX_TEST_PGTEST_EN_CH2_BIT_LEN        (1U)
#define TX_TEST_PGTEST_EN_CH2_BIT_MASK       0x2000000UL
#define TX_TEST_PGTEST_EN_CH1_BIT_OFFSET     (24U)
#define TX_TEST_PGTEST_EN_CH1_BIT_LEN        (1U)
#define TX_TEST_PGTEST_EN_CH1_BIT_MASK       0x1000000UL
#define TX_TEST_XTAL_ANATEST_EN_BIT_OFFSET   (18U)
#define TX_TEST_XTAL_ANATEST_EN_BIT_LEN      (1U)
#define TX_TEST_XTAL_ANATEST_EN_BIT_MASK     0x40000UL
#define TX_TEST_XTAL_ANATEST_SEL_BIT_OFFSET  (15U)
#define TX_TEST_XTAL_ANATEST_SEL_BIT_LEN     (3U)
#define TX_TEST_XTAL_ANATEST_SEL_BIT_MASK    0x38000UL
#define TX_TEST_TX_VCM_CTRL_HI_BIT_OFFSET    (13U)
#define TX_TEST_TX_VCM_CTRL_HI_BIT_LEN       (2U)
#define TX_TEST_TX_VCM_CTRL_HI_BIT_MASK      0x6000U
#define TX_TEST_TX_VCM_CTRL_LO_BIT_OFFSET    (9U)
#define TX_TEST_TX_VCM_CTRL_LO_BIT_LEN       (4U)
#define TX_TEST_TX_VCM_CTRL_LO_BIT_MASK      0x1e00U
#define TX_TEST_TX_DC_TEST_BIT_OFFSET        (5U)
#define TX_TEST_TX_DC_TEST_BIT_LEN           (4U)
#define TX_TEST_TX_DC_TEST_BIT_MASK          0x1e0U
#define TX_TEST_TX_DC_TEST_EN_BIT_OFFSET     (4U)
#define TX_TEST_TX_DC_TEST_EN_BIT_LEN        (1U)
#define TX_TEST_TX_DC_TEST_EN_BIT_MASK       0x10U
#define TX_TEST_TX_ENTEST_CH1_BIT_OFFSET     (3U)
#define TX_TEST_TX_ENTEST_CH1_BIT_LEN        (1U)
#define TX_TEST_TX_ENTEST_CH1_BIT_MASK       0x8U
#define TX_TEST_TX_ENTEST_CH2_BIT_OFFSET     (2U)
#define TX_TEST_TX_ENTEST_CH2_BIT_LEN        (1U)
#define TX_TEST_TX_ENTEST_CH2_BIT_MASK       0x4U
#define TX_TEST_TX_ENTEST_CH3_BIT_OFFSET     (1U)
#define TX_TEST_TX_ENTEST_CH3_BIT_LEN        (1U)
#define TX_TEST_TX_ENTEST_CH3_BIT_MASK       0x2U
#define TX_TEST_TX_ENTEST_CH4_BIT_OFFSET     (0U)
#define TX_TEST_TX_ENTEST_CH4_BIT_LEN        (1U)
#define TX_TEST_TX_ENTEST_CH4_BIT_MASK       0x1U

/******************************************************************************
* @brief Bit definitions for register SPARE_IO
**/
#define SPARE_IO_ID                          0x7002c
#define SPARE_IO_LEN                         (4U)
#define SPARE_IO_MASK                        0xFFFFFFFFUL
#define SPARE_IO_SPARE_BIT_OFFSET            (0U)
#define SPARE_IO_SPARE_BIT_LEN               (24U)
#define SPARE_IO_SPARE_BIT_MASK              0xffffffUL

/******************************************************************************
* @brief Bit definitions for register TESTMUX
**/
#define TESTMUX_ID                           0x70030
#define TESTMUX_LEN                          (4U)
#define TESTMUX_MASK                         0xFFFFFFFFUL
#define TESTMUX_TMUX_DC_TEST_ENABLEP_BIT_OFFSET (31U)
#define TESTMUX_TMUX_DC_TEST_ENABLEP_BIT_LEN (1U)
#define TESTMUX_TMUX_DC_TEST_ENABLEP_BIT_MASK 0x80000000UL
#define TESTMUX_TMUX_DC_TEST_ENABLEM_BIT_OFFSET (30U)
#define TESTMUX_TMUX_DC_TEST_ENABLEM_BIT_LEN (1U)
#define TESTMUX_TMUX_DC_TEST_ENABLEM_BIT_MASK 0x40000000UL
#define TESTMUX_TMUX_DC_TEST_BYPASSP_BIT_OFFSET (29U)
#define TESTMUX_TMUX_DC_TEST_BYPASSP_BIT_LEN (1U)
#define TESTMUX_TMUX_DC_TEST_BYPASSP_BIT_MASK 0x20000000UL
#define TESTMUX_TMUX_DC_TEST_BYPASSM_BIT_OFFSET (28U)
#define TESTMUX_TMUX_DC_TEST_BYPASSM_BIT_LEN (1U)
#define TESTMUX_TMUX_DC_TEST_BYPASSM_BIT_MASK 0x10000000UL
#define TESTMUX_TMUX_DC_TEST_SELP_BIT_OFFSET (23U)
#define TESTMUX_TMUX_DC_TEST_SELP_BIT_LEN    (5U)
#define TESTMUX_TMUX_DC_TEST_SELP_BIT_MASK   0xf800000UL
#define TESTMUX_TMUX_DC_TEST_SELM_BIT_OFFSET (18U)
#define TESTMUX_TMUX_DC_TEST_SELM_BIT_LEN    (5U)
#define TESTMUX_TMUX_DC_TEST_SELM_BIT_MASK   0x7c0000UL
#define TESTMUX_TMUX_DC_RTESTP_EN_BIT_OFFSET (17U)
#define TESTMUX_TMUX_DC_RTESTP_EN_BIT_LEN    (1U)
#define TESTMUX_TMUX_DC_RTESTP_EN_BIT_MASK   0x20000UL
#define TESTMUX_TMUX_DC_RTESTM_EN_BIT_OFFSET (16U)
#define TESTMUX_TMUX_DC_RTESTM_EN_BIT_LEN    (1U)
#define TESTMUX_TMUX_DC_RTESTM_EN_BIT_MASK   0x10000UL
#define TESTMUX_TMUX_HFTEST_ENI_BIT_OFFSET   (15U)
#define TESTMUX_TMUX_HFTEST_ENI_BIT_LEN      (1U)
#define TESTMUX_TMUX_HFTEST_ENI_BIT_MASK     0x8000U
#define TESTMUX_TMUX_HFTEST_ENQ_BIT_OFFSET   (14U)
#define TESTMUX_TMUX_HFTEST_ENQ_BIT_LEN      (1U)
#define TESTMUX_TMUX_HFTEST_ENQ_BIT_MASK     0x4000U
#define TESTMUX_TMUX_HFTEST_SELI_BIT_OFFSET  (11U)
#define TESTMUX_TMUX_HFTEST_SELI_BIT_LEN     (3U)
#define TESTMUX_TMUX_HFTEST_SELI_BIT_MASK    0x3800U
#define TESTMUX_TMUX_HFTEST_SELQ_BIT_OFFSET  (8U)
#define TESTMUX_TMUX_HFTEST_SELQ_BIT_LEN     (3U)
#define TESTMUX_TMUX_HFTEST_SELQ_BIT_MASK    0x700U
#define TESTMUX_TMUX_HFTEST_DACP_BIT_OFFSET  (4U)
#define TESTMUX_TMUX_HFTEST_DACP_BIT_LEN     (4U)
#define TESTMUX_TMUX_HFTEST_DACP_BIT_MASK    0xf0U
#define TESTMUX_TMUX_HFTEST_DACN_BIT_OFFSET  (0U)
#define TESTMUX_TMUX_HFTEST_DACN_BIT_LEN     (4U)
#define TESTMUX_TMUX_HFTEST_DACN_BIT_MASK    0xfU

/******************************************************************************
* @brief Bit definitions for register SAR_TEST
**/
#define SAR_TEST_ID                          0x70034
#define SAR_TEST_LEN                         (4U)
#define SAR_TEST_MASK                        0xFFFFFFFFUL
#define SAR_TEST_DIG_AUXADC_BUFBYP_ULV_BIT_OFFSET (30U)
#define SAR_TEST_DIG_AUXADC_BUFBYP_ULV_BIT_LEN (1U)
#define SAR_TEST_DIG_AUXADC_BUFBYP_ULV_BIT_MASK 0x40000000UL
#define SAR_TEST_DIG_AUXADC_FILTSEL_ULV_BIT_OFFSET (29U)
#define SAR_TEST_DIG_AUXADC_FILTSEL_ULV_BIT_LEN (1U)
#define SAR_TEST_DIG_AUXADC_FILTSEL_ULV_BIT_MASK 0x20000000UL
#define SAR_TEST_DIG_AUXADC_FILTEN_ULV_BIT_OFFSET (28U)
#define SAR_TEST_DIG_AUXADC_FILTEN_ULV_BIT_LEN (1U)
#define SAR_TEST_DIG_AUXADC_FILTEN_ULV_BIT_MASK 0x10000000UL
#define SAR_TEST_DIG_AUXADC_REF_CTRL_ULV_BIT_OFFSET (26U)
#define SAR_TEST_DIG_AUXADC_REF_CTRL_ULV_BIT_LEN (2U)
#define SAR_TEST_DIG_AUXADC_REF_CTRL_ULV_BIT_MASK 0xc000000UL
#define SAR_TEST_DIG_AUXADC_BIAS_CTRL_ULV_BIT_OFFSET (23U)
#define SAR_TEST_DIG_AUXADC_BIAS_CTRL_ULV_BIT_LEN (3U)
#define SAR_TEST_DIG_AUXADC_BIAS_CTRL_ULV_BIT_MASK 0x3800000UL
#define SAR_TEST_DIG_AUXADC_ATTN_EN_ULV_BIT_OFFSET (22U)
#define SAR_TEST_DIG_AUXADC_ATTN_EN_ULV_BIT_LEN (1U)
#define SAR_TEST_DIG_AUXADC_ATTN_EN_ULV_BIT_MASK 0x400000UL
#define SAR_TEST_DIG_AUXADC_ATTN_SEL_ULV_BIT_OFFSET (21U)
#define SAR_TEST_DIG_AUXADC_ATTN_SEL_ULV_BIT_LEN (1U)
#define SAR_TEST_DIG_AUXADC_ATTN_SEL_ULV_BIT_MASK 0x200000UL
#define SAR_TEST_DIG_AUXADC_TESTSEL_ULV_BIT_OFFSET (18U)
#define SAR_TEST_DIG_AUXADC_TESTSEL_ULV_BIT_LEN (3U)
#define SAR_TEST_DIG_AUXADC_TESTSEL_ULV_BIT_MASK 0x1c0000UL
#define SAR_TEST_DIG_TSENSE_REF_RES_ULV_BIT_OFFSET (16U)
#define SAR_TEST_DIG_TSENSE_REF_RES_ULV_BIT_LEN (2U)
#define SAR_TEST_DIG_TSENSE_REF_RES_ULV_BIT_MASK 0x30000UL
#define SAR_TEST_ENDET_EN_SHORT_BIT_OFFSET   (15U)
#define SAR_TEST_ENDET_EN_SHORT_BIT_LEN      (1U)
#define SAR_TEST_ENDET_EN_SHORT_BIT_MASK     0x8000U
#define SAR_TEST_ENDET_EN_BYPASS_BIT_OFFSET  (14U)
#define SAR_TEST_ENDET_EN_BYPASS_BIT_LEN     (1U)
#define SAR_TEST_ENDET_EN_BYPASS_BIT_MASK    0x4000U
#define SAR_TEST_AUXADC_WAIT4EOC_BIT_OFFSET  (13U)
#define SAR_TEST_AUXADC_WAIT4EOC_BIT_LEN     (1U)
#define SAR_TEST_AUXADC_WAIT4EOC_BIT_MASK    0x2000U
#define SAR_TEST_AUXADC_CONT_SAMP_FAST_BIT_OFFSET (12U)
#define SAR_TEST_AUXADC_CONT_SAMP_FAST_BIT_LEN (1U)
#define SAR_TEST_AUXADC_CONT_SAMP_FAST_BIT_MASK 0x1000U
#define SAR_TEST_AUXADC_CONT_SAMP_BIT_OFFSET (11U)
#define SAR_TEST_AUXADC_CONT_SAMP_BIT_LEN    (1U)
#define SAR_TEST_AUXADC_CONT_SAMP_BIT_MASK   0x800U
#define SAR_TEST_AUXADC_TESTSEL_BIT_OFFSET   (10U)
#define SAR_TEST_AUXADC_TESTSEL_BIT_LEN      (1U)
#define SAR_TEST_AUXADC_TESTSEL_BIT_MASK     0x400U
#define SAR_TEST_ENDET_OVR_DDA_SHRT_BIT_OFFSET (9U)
#define SAR_TEST_ENDET_OVR_DDA_SHRT_BIT_LEN  (1U)
#define SAR_TEST_ENDET_OVR_DDA_SHRT_BIT_MASK 0x200U
#define SAR_TEST_ENDET_OVR_EN_BIT_OFFSET     (8U)
#define SAR_TEST_ENDET_OVR_EN_BIT_LEN        (1U)
#define SAR_TEST_ENDET_OVR_EN_BIT_MASK       0x100U
#define SAR_TEST_ENDET_OVR_MODE_BIT_OFFSET   (7U)
#define SAR_TEST_ENDET_OVR_MODE_BIT_LEN      (1U)
#define SAR_TEST_ENDET_OVR_MODE_BIT_MASK     0x80U
#define SAR_TEST_ENDET_EN_DDA_TRIM_TMUX_BIT_OFFSET (5U)
#define SAR_TEST_ENDET_EN_DDA_TRIM_TMUX_BIT_LEN (2U)
#define SAR_TEST_ENDET_EN_DDA_TRIM_TMUX_BIT_MASK 0x60U
#define SAR_TEST_ENDET_EN_DIODE_TMUX_BIT_OFFSET (3U)
#define SAR_TEST_ENDET_EN_DIODE_TMUX_BIT_LEN (2U)
#define SAR_TEST_ENDET_EN_DIODE_TMUX_BIT_MASK 0x18U
#define SAR_TEST_TSENSE_OVR_EN_BIT_OFFSET    (2U)
#define SAR_TEST_TSENSE_OVR_EN_BIT_LEN       (1U)
#define SAR_TEST_TSENSE_OVR_EN_BIT_MASK      0x4U
#define SAR_TEST_TSENSE_ITEST_EN_BIT_OFFSET  (1U)
#define SAR_TEST_TSENSE_ITEST_EN_BIT_LEN     (1U)
#define SAR_TEST_TSENSE_ITEST_EN_BIT_MASK    0x2U
#define SAR_TEST_TSENSE_VTEST_EN_BIT_OFFSET  (0U)
#define SAR_TEST_TSENSE_VTEST_EN_BIT_LEN     (1U)
#define SAR_TEST_TSENSE_VTEST_EN_BIT_MASK    0x1U

/******************************************************************************
* @brief Bit definitions for register PG_TST_DATA
**/
#define PG_TST_DATA_ID                       0x70038
#define PG_TST_DATA_LEN                      (4U)
#define PG_TST_DATA_MASK                     0xFFFFFFFFUL
#define PG_TST_DATA_PG_TEST_DATA_BIT_OFFSET  (0U)
#define PG_TST_DATA_PG_TEST_DATA_BIT_LEN     (32U)
#define PG_TST_DATA_PG_TEST_DATA_BIT_MASK    0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register RF_STATUS
**/
#define RF_STATUS_ID                         0x7003c
#define RF_STATUS_LEN                        (4U)
#define RF_STATUS_MASK                       0xFFFFFFFFUL
#define RF_STATUS_PLL1_MID_FLAG_BIT_OFFSET   (3U)
#define RF_STATUS_PLL1_MID_FLAG_BIT_LEN      (1U)
#define RF_STATUS_PLL1_MID_FLAG_BIT_MASK     0x8U
#define RF_STATUS_PLL1_HI_FLAG_BIT_OFFSET    (2U)
#define RF_STATUS_PLL1_HI_FLAG_BIT_LEN       (1U)
#define RF_STATUS_PLL1_HI_FLAG_BIT_MASK      0x4U
#define RF_STATUS_PLL1_LO_FLAG_BIT_OFFSET    (1U)
#define RF_STATUS_PLL1_LO_FLAG_BIT_LEN       (1U)
#define RF_STATUS_PLL1_LO_FLAG_BIT_MASK      0x2U
#define RF_STATUS_PLL1_LOCK_BIT_OFFSET       (0U)
#define RF_STATUS_PLL1_LOCK_BIT_LEN          (1U)
#define RF_STATUS_PLL1_LOCK_BIT_MASK         0x1U

/******************************************************************************
* @brief Bit definitions for register LDO_TUNE_LO
**/
#define LDO_TUNE_LO_ID                       0x70040
#define LDO_TUNE_LO_LEN                      (4U)
#define LDO_TUNE_LO_MASK                     0xFFFFFFFFUL
#define LDO_TUNE_LO_LDO_IF1_TUNE_BIT_OFFSET  (28U)
#define LDO_TUNE_LO_LDO_IF1_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_IF1_TUNE_BIT_MASK    0xf0000000UL
#define LDO_TUNE_LO_LDO_TX2_TUNE_BIT_OFFSET  (24U)
#define LDO_TUNE_LO_LDO_TX2_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_TX2_TUNE_BIT_MASK    0xf000000UL
#define LDO_TUNE_LO_LDO_TX1_TUNE_BIT_OFFSET  (20U)
#define LDO_TUNE_LO_LDO_TX1_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_TX1_TUNE_BIT_MASK    0xf00000UL
#define LDO_TUNE_LO_LDO_PLL_TUNE_BIT_OFFSET  (16U)
#define LDO_TUNE_LO_LDO_PLL_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_PLL_TUNE_BIT_MASK    0xf0000UL
#define LDO_TUNE_LO_LDO_VCO_TUNE_BIT_OFFSET  (12U)
#define LDO_TUNE_LO_LDO_VCO_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_VCO_TUNE_BIT_MASK    0xf000U
#define LDO_TUNE_LO_LDO_MS3_TUNE_BIT_OFFSET  (8U)
#define LDO_TUNE_LO_LDO_MS3_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_MS3_TUNE_BIT_MASK    0xf00U
#define LDO_TUNE_LO_LDO_MS2_TUNE_BIT_OFFSET  (4U)
#define LDO_TUNE_LO_LDO_MS2_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_MS2_TUNE_BIT_MASK    0xf0U
#define LDO_TUNE_LO_LDO_MS1_TUNE_BIT_OFFSET  (0U)
#define LDO_TUNE_LO_LDO_MS1_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_LO_LDO_MS1_TUNE_BIT_MASK    0xfU

/******************************************************************************
* @brief Bit definitions for register LDO_TUNE_HI
**/
#define LDO_TUNE_HI_ID                       0x70044
#define LDO_TUNE_HI_LEN                      (4U)
#define LDO_TUNE_HI_MASK                     0xFFFFFFFFUL
#define LDO_TUNE_HI_LDO_HVTX_TUNE_BIT_OFFSET (24U)
#define LDO_TUNE_HI_LDO_HVTX_TUNE_BIT_LEN    (4U)
#define LDO_TUNE_HI_LDO_HVTX_TUNE_BIT_MASK   0xf000000UL
#define LDO_TUNE_HI_LDO_HVXTAL_TUNE_BIT_OFFSET (16U)
#define LDO_TUNE_HI_LDO_HVXTAL_TUNE_BIT_LEN  (8U)
#define LDO_TUNE_HI_LDO_HVXTAL_TUNE_BIT_MASK 0xff0000UL
#define LDO_TUNE_HI_LDO_HVAUX_TUNE_BIT_OFFSET (12U)
#define LDO_TUNE_HI_LDO_HVAUX_TUNE_BIT_LEN   (4U)
#define LDO_TUNE_HI_LDO_HVAUX_TUNE_BIT_MASK  0xf000U
#define LDO_TUNE_HI_LDO_RFCH9_TUNE_BIT_OFFSET (8U)
#define LDO_TUNE_HI_LDO_RFCH9_TUNE_BIT_LEN   (4U)
#define LDO_TUNE_HI_LDO_RFCH9_TUNE_BIT_MASK  0xf00U
#define LDO_TUNE_HI_LDO_RFCH5_TUNE_BIT_OFFSET (4U)
#define LDO_TUNE_HI_LDO_RFCH5_TUNE_BIT_LEN   (4U)
#define LDO_TUNE_HI_LDO_RFCH5_TUNE_BIT_MASK  0xf0U
#define LDO_TUNE_HI_LDO_IF2_TUNE_BIT_OFFSET  (0U)
#define LDO_TUNE_HI_LDO_IF2_TUNE_BIT_LEN     (4U)
#define LDO_TUNE_HI_LDO_IF2_TUNE_BIT_MASK    0xfU

/******************************************************************************
* @brief Bit definitions for register LDO_CTRL
**/
#define LDO_CTRL_ID                          0x70048
#define LDO_CTRL_LEN                         (4U)
#define LDO_CTRL_MASK                        0xFFFFFFFFUL
#define LDO_CTRL_LDO_VDDHVTX_VREF_BIT_OFFSET (27U)
#define LDO_CTRL_LDO_VDDHVTX_VREF_BIT_LEN    (1U)
#define LDO_CTRL_LDO_VDDHVTX_VREF_BIT_MASK   0x8000000UL
#define LDO_CTRL_LDO_VDDRFCH9_VREF_BIT_OFFSET (26U)
#define LDO_CTRL_LDO_VDDRFCH9_VREF_BIT_LEN   (1U)
#define LDO_CTRL_LDO_VDDRFCH9_VREF_BIT_MASK  0x4000000UL
#define LDO_CTRL_LDO_VDDRFCH5_VREF_BIT_OFFSET (25U)
#define LDO_CTRL_LDO_VDDRFCH5_VREF_BIT_LEN   (1U)
#define LDO_CTRL_LDO_VDDRFCH5_VREF_BIT_MASK  0x2000000UL
#define LDO_CTRL_LDO_VDDIF2_VREF_BIT_OFFSET  (24U)
#define LDO_CTRL_LDO_VDDIF2_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDIF2_VREF_BIT_MASK    0x1000000UL
#define LDO_CTRL_LDO_VDDIF1_VREF_BIT_OFFSET  (23U)
#define LDO_CTRL_LDO_VDDIF1_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDIF1_VREF_BIT_MASK    0x800000UL
#define LDO_CTRL_LDO_VDDTX2_VREF_BIT_OFFSET  (22U)
#define LDO_CTRL_LDO_VDDTX2_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDTX2_VREF_BIT_MASK    0x400000UL
#define LDO_CTRL_LDO_VDDTX1_VREF_BIT_OFFSET  (21U)
#define LDO_CTRL_LDO_VDDTX1_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDTX1_VREF_BIT_MASK    0x200000UL
#define LDO_CTRL_LDO_VDDPLL_VREF_BIT_OFFSET  (20U)
#define LDO_CTRL_LDO_VDDPLL_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDPLL_VREF_BIT_MASK    0x100000UL
#define LDO_CTRL_LDO_VDDVCO_VREF_BIT_OFFSET  (19U)
#define LDO_CTRL_LDO_VDDVCO_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDVCO_VREF_BIT_MASK    0x80000UL
#define LDO_CTRL_LDO_VDDMS3_VREF_BIT_OFFSET  (18U)
#define LDO_CTRL_LDO_VDDMS3_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDMS3_VREF_BIT_MASK    0x40000UL
#define LDO_CTRL_LDO_VDDMS2_VREF_BIT_OFFSET  (17U)
#define LDO_CTRL_LDO_VDDMS2_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDMS2_VREF_BIT_MASK    0x20000UL
#define LDO_CTRL_LDO_VDDMS1_VREF_BIT_OFFSET  (16U)
#define LDO_CTRL_LDO_VDDMS1_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDMS1_VREF_BIT_MASK    0x10000UL
#define LDO_CTRL_LDO_VDDHVTX_EN_BIT_OFFSET   (11U)
#define LDO_CTRL_LDO_VDDHVTX_EN_BIT_LEN      (1U)
#define LDO_CTRL_LDO_VDDHVTX_EN_BIT_MASK     0x800U
#define LDO_CTRL_LDO_VDDRFCH9_EN_BIT_OFFSET  (10U)
#define LDO_CTRL_LDO_VDDRFCH9_EN_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDRFCH9_EN_BIT_MASK    0x400U
#define LDO_CTRL_LDO_VDDRFCH5_EN_BIT_OFFSET  (9U)
#define LDO_CTRL_LDO_VDDRFCH5_EN_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDRFCH5_EN_BIT_MASK    0x200U
#define LDO_CTRL_LDO_VDDIF2_EN_BIT_OFFSET    (8U)
#define LDO_CTRL_LDO_VDDIF2_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK      0x100U
#define LDO_CTRL_LDO_VDDIF1_EN_BIT_OFFSET    (7U)
#define LDO_CTRL_LDO_VDDIF1_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDIF1_EN_BIT_MASK      0x80U
#define LDO_CTRL_LDO_VDDTX2_EN_BIT_OFFSET    (6U)
#define LDO_CTRL_LDO_VDDTX2_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDTX2_EN_BIT_MASK      0x40U
#define LDO_CTRL_LDO_VDDTX1_EN_BIT_OFFSET    (5U)
#define LDO_CTRL_LDO_VDDTX1_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDTX1_EN_BIT_MASK      0x20U
#define LDO_CTRL_LDO_VDDPLL_EN_BIT_OFFSET    (4U)
#define LDO_CTRL_LDO_VDDPLL_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK      0x10U
#define LDO_CTRL_LDO_VDDVCO_EN_BIT_OFFSET    (3U)
#define LDO_CTRL_LDO_VDDVCO_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDVCO_EN_BIT_MASK      0x8U
#define LDO_CTRL_LDO_VDDMS3_EN_BIT_OFFSET    (2U)
#define LDO_CTRL_LDO_VDDMS3_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK      0x4U
#define LDO_CTRL_LDO_VDDMS2_EN_BIT_OFFSET    (1U)
#define LDO_CTRL_LDO_VDDMS2_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDMS2_EN_BIT_MASK      0x2U
#define LDO_CTRL_LDO_VDDMS1_EN_BIT_OFFSET    (0U)
#define LDO_CTRL_LDO_VDDMS1_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK      0x1U

/******************************************************************************
* @brief Bit definitions for register LDO_VOUT
**/
#define LDO_VOUT_ID                          0x7004c
#define LDO_VOUT_LEN                         (4U)
#define LDO_VOUT_MASK                        0xFFFFFFFFUL
#define LDO_VOUT_LDO_VDDHVTX_VOUT_SEL_BIT_OFFSET (26U)
#define LDO_VOUT_LDO_VDDHVTX_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDHVTX_VOUT_SEL_BIT_MASK 0xc000000UL
#define LDO_VOUT_LDO_VDDHVXTAL_VOUT_SEL_BIT_OFFSET (24U)
#define LDO_VOUT_LDO_VDDHVXTAL_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDHVXTAL_VOUT_SEL_BIT_MASK 0x3000000UL
#define LDO_VOUT_LDO_VDDHVAUX_VOUT_SEL_BIT_OFFSET (22U)
#define LDO_VOUT_LDO_VDDHVAUX_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDHVAUX_VOUT_SEL_BIT_MASK 0xc00000UL
#define LDO_VOUT_LDO_VDDRFCH9_VOUT_SEL_BIT_OFFSET (0U)
#define LDO_VOUT_LDO_VDDRFCH9_VOUT_SEL_BIT_LEN (1U)
#define LDO_VOUT_LDO_VDDRFCH9_VOUT_SEL_BIT_MASK 0x1U
#define LDO_VOUT_LDO_VDDRFCH5_VOUT_SEL_BIT_OFFSET (0U)
#define LDO_VOUT_LDO_VDDRFCH5_VOUT_SEL_BIT_LEN (1U)
#define LDO_VOUT_LDO_VDDRFCH5_VOUT_SEL_BIT_MASK 0x1U
#define LDO_VOUT_LDO_VDDIF2_VOUT_SEL_BIT_OFFSET (0U)
#define LDO_VOUT_LDO_VDDIF2_VOUT_SEL_BIT_LEN (1U)
#define LDO_VOUT_LDO_VDDIF2_VOUT_SEL_BIT_MASK 0x1U
#define LDO_VOUT_LDO_VDDIF1_VOUT_SEL_BIT_OFFSET (14U)
#define LDO_VOUT_LDO_VDDIF1_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDIF1_VOUT_SEL_BIT_MASK 0xc000U
#define LDO_VOUT_LDO_VDDTX2_VOUT_SEL_BIT_OFFSET (12U)
#define LDO_VOUT_LDO_VDDTX2_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDTX2_VOUT_SEL_BIT_MASK 0x3000U
#define LDO_VOUT_LDO_VDDTX1_VOUT_SEL_BIT_OFFSET (10U)
#define LDO_VOUT_LDO_VDDTX1_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDTX1_VOUT_SEL_BIT_MASK 0xc00U
#define LDO_VOUT_LDO_VDDPLL_VOUT_SEL_BIT_OFFSET (8U)
#define LDO_VOUT_LDO_VDDPLL_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDPLL_VOUT_SEL_BIT_MASK 0x300U
#define LDO_VOUT_LDO_VDDVCO_VOUT_SEL_BIT_OFFSET (6U)
#define LDO_VOUT_LDO_VDDVCO_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDVCO_VOUT_SEL_BIT_MASK 0xc0U
#define LDO_VOUT_LDO_VDDMS3_VOUT_SEL_BIT_OFFSET (4U)
#define LDO_VOUT_LDO_VDDMS3_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDMS3_VOUT_SEL_BIT_MASK 0x30U
#define LDO_VOUT_LDO_VDDMS2_VOUT_SEL_BIT_OFFSET (2U)
#define LDO_VOUT_LDO_VDDMS2_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDMS2_VOUT_SEL_BIT_MASK 0xcU
#define LDO_VOUT_LDO_VDDMS1_VOUT_SEL_BIT_OFFSET (0U)
#define LDO_VOUT_LDO_VDDMS1_VOUT_SEL_BIT_LEN (2U)
#define LDO_VOUT_LDO_VDDMS1_VOUT_SEL_BIT_MASK 0x3U

/******************************************************************************
* @brief Bit definitions for register LDO_RLOAD
**/
#define LDO_RLOAD_ID                         0x70050
#define LDO_RLOAD_LEN                        (4U)
#define LDO_RLOAD_MASK                       0xFFFFFFFFUL
#define LDO_RLOAD_LDO_VDDHVTX_RLOADON_SEL_BIT_OFFSET (28U)
#define LDO_RLOAD_LDO_VDDHVTX_RLOADON_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDHVTX_RLOADON_SEL_BIT_MASK 0x30000000UL
#define LDO_RLOAD_LDO_VDDHVTX_RLOAD_SEL_BIT_OFFSET (26U)
#define LDO_RLOAD_LDO_VDDHVTX_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDHVTX_RLOAD_SEL_BIT_MASK 0xc000000UL
#define LDO_RLOAD_LDO_VDDHVXTAL_RLOAD_SEL_BIT_OFFSET (24U)
#define LDO_RLOAD_LDO_VDDHVXTAL_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDHVXTAL_RLOAD_SEL_BIT_MASK 0x3000000UL
#define LDO_RLOAD_LDO_VDDHVAUX_RLOAD_SEL_BIT_OFFSET (22U)
#define LDO_RLOAD_LDO_VDDHVAUX_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDHVAUX_RLOAD_SEL_BIT_MASK 0xc00000UL
#define LDO_RLOAD_LDO_VDDRFCH9_RLOAD_SEL_BIT_OFFSET (0U)
#define LDO_RLOAD_LDO_VDDRFCH9_RLOAD_SEL_BIT_LEN (1U)
#define LDO_RLOAD_LDO_VDDRFCH9_RLOAD_SEL_BIT_MASK 0x1U
#define LDO_RLOAD_LDO_VDDRFCH5_RLOAD_SEL_BIT_OFFSET (0U)
#define LDO_RLOAD_LDO_VDDRFCH5_RLOAD_SEL_BIT_LEN (1U)
#define LDO_RLOAD_LDO_VDDRFCH5_RLOAD_SEL_BIT_MASK 0x1U
#define LDO_RLOAD_LDO_VDDIF2_RLOAD_SEL_BIT_OFFSET (0U)
#define LDO_RLOAD_LDO_VDDIF2_RLOAD_SEL_BIT_LEN (1U)
#define LDO_RLOAD_LDO_VDDIF2_RLOAD_SEL_BIT_MASK 0x1U
#define LDO_RLOAD_LDO_VDDIF1_RLOAD_SEL_BIT_OFFSET (14U)
#define LDO_RLOAD_LDO_VDDIF1_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDIF1_RLOAD_SEL_BIT_MASK 0xc000U
#define LDO_RLOAD_LDO_VDDTX2_RLOAD_SEL_BIT_OFFSET (12U)
#define LDO_RLOAD_LDO_VDDTX2_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDTX2_RLOAD_SEL_BIT_MASK 0x3000U
#define LDO_RLOAD_LDO_VDDTX1_RLOAD_SEL_BIT_OFFSET (10U)
#define LDO_RLOAD_LDO_VDDTX1_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDTX1_RLOAD_SEL_BIT_MASK 0xc00U
#define LDO_RLOAD_LDO_VDDPLL_RLOAD_SEL_BIT_OFFSET (8U)
#define LDO_RLOAD_LDO_VDDPLL_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDPLL_RLOAD_SEL_BIT_MASK 0x300U
#define LDO_RLOAD_LDO_VDDVCO_RLOAD_SEL_BIT_OFFSET (6U)
#define LDO_RLOAD_LDO_VDDVCO_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDVCO_RLOAD_SEL_BIT_MASK 0xc0U
#define LDO_RLOAD_LDO_VDDMS3_RLOAD_SEL_BIT_OFFSET (4U)
#define LDO_RLOAD_LDO_VDDMS3_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDMS3_RLOAD_SEL_BIT_MASK 0x30U
#define LDO_RLOAD_LDO_VDDMS2_RLOAD_SEL_BIT_OFFSET (2U)
#define LDO_RLOAD_LDO_VDDMS2_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDMS2_RLOAD_SEL_BIT_MASK 0xcU
#define LDO_RLOAD_LDO_VDDMS1_RLOAD_SEL_BIT_OFFSET (0U)
#define LDO_RLOAD_LDO_VDDMS1_RLOAD_SEL_BIT_LEN (2U)
#define LDO_RLOAD_LDO_VDDMS1_RLOAD_SEL_BIT_MASK 0x3U

/******************************************************************************
* @brief Bit definitions for register LDO_BYPASS
**/
#define LDO_BYPASS_ID                        0x70054
#define LDO_BYPASS_LEN                       (4U)
#define LDO_BYPASS_MASK                      0xFFFFFFFFUL
#define LDO_BYPASS_DIG_SMPS_TEST_SEL_BIT_OFFSET (29U)
#define LDO_BYPASS_DIG_SMPS_TEST_SEL_BIT_LEN (2U)
#define LDO_BYPASS_DIG_SMPS_TEST_SEL_BIT_MASK 0x60000000UL
#define LDO_BYPASS_DIG_SMPS_TEST_EN_BIT_OFFSET (28U)
#define LDO_BYPASS_DIG_SMPS_TEST_EN_BIT_LEN  (1U)
#define LDO_BYPASS_DIG_SMPS_TEST_EN_BIT_MASK 0x10000000UL
#define LDO_BYPASS_OVR_LDO_PWRGOOD_BIT_OFFSET (27U)
#define LDO_BYPASS_OVR_LDO_PWRGOOD_BIT_LEN   (1U)
#define LDO_BYPASS_OVR_LDO_PWRGOOD_BIT_MASK  0x8000000UL
#define LDO_BYPASS_LDO_VDDTX1_FAST_EN_BIT_OFFSET (26U)
#define LDO_BYPASS_LDO_VDDTX1_FAST_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDTX1_FAST_EN_BIT_MASK 0x4000000UL
#define LDO_BYPASS_LDO_VDDHVTX_HIBW_BIT_OFFSET (25U)
#define LDO_BYPASS_LDO_VDDHVTX_HIBW_BIT_LEN  (1U)
#define LDO_BYPASS_LDO_VDDHVTX_HIBW_BIT_MASK 0x2000000UL
#define LDO_BYPASS_LDO_VDDHVTX_FAST_PD_BIT_OFFSET (24U)
#define LDO_BYPASS_LDO_VDDHVTX_FAST_PD_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDHVTX_FAST_PD_BIT_MASK 0x1000000UL
#define LDO_BYPASS_LDO_VDDHVTX_DC_TST_EN_BIT_OFFSET (21U)
#define LDO_BYPASS_LDO_VDDHVTX_DC_TST_EN_BIT_LEN (3U)
#define LDO_BYPASS_LDO_VDDHVTX_DC_TST_EN_BIT_MASK 0xe00000UL
#define LDO_BYPASS_LDO_VDDHVXTAL_DC_TST_EN_BIT_OFFSET (18U)
#define LDO_BYPASS_LDO_VDDHVXTAL_DC_TST_EN_BIT_LEN (3U)
#define LDO_BYPASS_LDO_VDDHVXTAL_DC_TST_EN_BIT_MASK 0x1c0000UL
#define LDO_BYPASS_LDO_VDDHVAUX_DC_TST_EN_BIT_OFFSET (15U)
#define LDO_BYPASS_LDO_VDDHVAUX_DC_TST_EN_BIT_LEN (3U)
#define LDO_BYPASS_LDO_VDDHVAUX_DC_TST_EN_BIT_MASK 0x38000UL
#define LDO_BYPASS_LDO_VDDRFCH9_DC_TST_EN_LO_BIT_OFFSET (14U)
#define LDO_BYPASS_LDO_VDDRFCH9_DC_TST_EN_LO_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDRFCH9_DC_TST_EN_LO_BIT_MASK 0x4000U
#define LDO_BYPASS_LDO_VDDHVTX_BYP_EN_BIT_OFFSET (13U)
#define LDO_BYPASS_LDO_VDDHVTX_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDHVTX_BYP_EN_BIT_MASK 0x2000U
#define LDO_BYPASS_LDO_VDDHVXTAL_BYP_EN_BIT_OFFSET (12U)
#define LDO_BYPASS_LDO_VDDHVXTAL_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDHVXTAL_BYP_EN_BIT_MASK 0x1000U
#define LDO_BYPASS_LDO_VDDHVAUX_BYP_EN_BIT_OFFSET (11U)
#define LDO_BYPASS_LDO_VDDHVAUX_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDHVAUX_BYP_EN_BIT_MASK 0x800U
#define LDO_BYPASS_LDO_VDDRFCH9_BYP_EN_BIT_OFFSET (10U)
#define LDO_BYPASS_LDO_VDDRFCH9_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDRFCH9_BYP_EN_BIT_MASK 0x400U
#define LDO_BYPASS_LDO_VDDRCH5_BYP_EN_BIT_OFFSET (9U)
#define LDO_BYPASS_LDO_VDDRCH5_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDRCH5_BYP_EN_BIT_MASK 0x200U
#define LDO_BYPASS_LDO_VDDIF2_BYP_EN_BIT_OFFSET (8U)
#define LDO_BYPASS_LDO_VDDIF2_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDIF2_BYP_EN_BIT_MASK 0x100U
#define LDO_BYPASS_LDO_VDDIF1_BYP_EN_BIT_OFFSET (7U)
#define LDO_BYPASS_LDO_VDDIF1_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDIF1_BYP_EN_BIT_MASK 0x80U
#define LDO_BYPASS_LDO_VDDTX2_BYP_EN_BIT_OFFSET (6U)
#define LDO_BYPASS_LDO_VDDTX2_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDTX2_BYP_EN_BIT_MASK 0x40U
#define LDO_BYPASS_LDO_VDDTX1_BYP_EN_BIT_OFFSET (5U)
#define LDO_BYPASS_LDO_VDDTX1_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDTX1_BYP_EN_BIT_MASK 0x20U
#define LDO_BYPASS_LDO_VDDPLL_BYP_EN_BIT_OFFSET (4U)
#define LDO_BYPASS_LDO_VDDPLL_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDPLL_BYP_EN_BIT_MASK 0x10U
#define LDO_BYPASS_LDO_VDDVCO_BYP_EN_BIT_OFFSET (3U)
#define LDO_BYPASS_LDO_VDDVCO_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDVCO_BYP_EN_BIT_MASK 0x8U
#define LDO_BYPASS_LDO_VDDMS3_BYP_EN_BIT_OFFSET (2U)
#define LDO_BYPASS_LDO_VDDMS3_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDMS3_BYP_EN_BIT_MASK 0x4U
#define LDO_BYPASS_LDO_VDDMS2_BYP_EN_BIT_OFFSET (1U)
#define LDO_BYPASS_LDO_VDDMS2_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDMS2_BYP_EN_BIT_MASK 0x2U
#define LDO_BYPASS_LDO_VDDMS1_BYP_EN_BIT_OFFSET (0U)
#define LDO_BYPASS_LDO_VDDMS1_BYP_EN_BIT_LEN (1U)
#define LDO_BYPASS_LDO_VDDMS1_BYP_EN_BIT_MASK 0x1U

/******************************************************************************
* @brief Bit definitions for register LDO_DC_TST
**/
#define LDO_DC_TST_ID                        0x70058
#define LDO_DC_TST_LEN                       (4U)
#define LDO_DC_TST_MASK                      0xFFFFFFFFUL
#define LDO_DC_TST_LDO_VDDRFCH9_DC_TST_EN_HI_BIT_OFFSET (30U)
#define LDO_DC_TST_LDO_VDDRFCH9_DC_TST_EN_HI_BIT_LEN (2U)
#define LDO_DC_TST_LDO_VDDRFCH9_DC_TST_EN_HI_BIT_MASK 0xc0000000UL
#define LDO_DC_TST_LDO_VDDRCH5_DC_TST_EN_BIT_OFFSET (27U)
#define LDO_DC_TST_LDO_VDDRCH5_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDRCH5_DC_TST_EN_BIT_MASK 0x38000000UL
#define LDO_DC_TST_LDO_VDDIF2_DC_TST_EN_BIT_OFFSET (24U)
#define LDO_DC_TST_LDO_VDDIF2_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDIF2_DC_TST_EN_BIT_MASK 0x7000000UL
#define LDO_DC_TST_LDO_VDDIF1_DC_TST_EN_BIT_OFFSET (21U)
#define LDO_DC_TST_LDO_VDDIF1_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDIF1_DC_TST_EN_BIT_MASK 0xe00000UL
#define LDO_DC_TST_LDO_VDDTX2_DC_TST_EN_BIT_OFFSET (18U)
#define LDO_DC_TST_LDO_VDDTX2_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDTX2_DC_TST_EN_BIT_MASK 0x1c0000UL
#define LDO_DC_TST_LDO_VDDTX1_DC_TST_EN_BIT_OFFSET (15U)
#define LDO_DC_TST_LDO_VDDTX1_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDTX1_DC_TST_EN_BIT_MASK 0x38000UL
#define LDO_DC_TST_LDO_VDDPLL_DC_TST_EN_BIT_OFFSET (12U)
#define LDO_DC_TST_LDO_VDDPLL_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDPLL_DC_TST_EN_BIT_MASK 0x7000U
#define LDO_DC_TST_LDO_VDDVCO_DC_TST_EN_BIT_OFFSET (9U)
#define LDO_DC_TST_LDO_VDDVCO_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDVCO_DC_TST_EN_BIT_MASK 0xe00U
#define LDO_DC_TST_LDO_VDDMS3_DC_TST_EN_BIT_OFFSET (6U)
#define LDO_DC_TST_LDO_VDDMS3_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDMS3_DC_TST_EN_BIT_MASK 0x1c0U
#define LDO_DC_TST_LDO_VDDMS2_DC_TST_EN_BIT_OFFSET (3U)
#define LDO_DC_TST_LDO_VDDMS2_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDMS2_DC_TST_EN_BIT_MASK 0x38U
#define LDO_DC_TST_LDO_VDDMS1_DC_TST_EN_BIT_OFFSET (0U)
#define LDO_DC_TST_LDO_VDDMS1_DC_TST_EN_BIT_LEN (3U)
#define LDO_DC_TST_LDO_VDDMS1_DC_TST_EN_BIT_MASK 0x7U

/******************************************************************************
* @brief Bit definitions for register SAR_CTRL
**/
#define SAR_CTRL_ID                          0x80000
#define SAR_CTRL_LEN                         (4U)
#define SAR_CTRL_MASK                        0xFFFFFFFFUL
#define SAR_CTRL_SAR_SEL_TXLVL_BIT_OFFSET    (15U)
#define SAR_CTRL_SAR_SEL_TXLVL_BIT_LEN       (1U)
#define SAR_CTRL_SAR_SEL_TXLVL_BIT_MASK      0x8000U
#define SAR_CTRL_SAR_FORCE_SEL_BIT_OFFSET    (12U)
#define SAR_CTRL_SAR_FORCE_SEL_BIT_LEN       (3U)
#define SAR_CTRL_SAR_FORCE_SEL_BIT_MASK      0x7000U
#define SAR_CTRL_SAR_OVR_ADC_EN_BIT_OFFSET   (11U)
#define SAR_CTRL_SAR_OVR_ADC_EN_BIT_LEN      (1U)
#define SAR_CTRL_SAR_OVR_ADC_EN_BIT_MASK     0x800U
#define SAR_CTRL_SAR_OVR_SCT_BIT_OFFSET      (6U)
#define SAR_CTRL_SAR_OVR_SCT_BIT_LEN         (5U)
#define SAR_CTRL_SAR_OVR_SCT_BIT_MASK        0x7c0U
#define SAR_CTRL_SAR_OVR_MUX_EN_BIT_OFFSET   (5U)
#define SAR_CTRL_SAR_OVR_MUX_EN_BIT_LEN      (1U)
#define SAR_CTRL_SAR_OVR_MUX_EN_BIT_MASK     0x20U
#define SAR_CTRL_SAR_OVR_START_BIT_OFFSET    (4U)
#define SAR_CTRL_SAR_OVR_START_BIT_LEN       (1U)
#define SAR_CTRL_SAR_OVR_START_BIT_MASK      0x10U
#define SAR_CTRL_SAR_OVR_RSTN_BIT_OFFSET     (3U)
#define SAR_CTRL_SAR_OVR_RSTN_BIT_LEN        (1U)
#define SAR_CTRL_SAR_OVR_RSTN_BIT_MASK       0x8U
#define SAR_CTRL_SAR_OVR_CLK_BIT_OFFSET      (2U)
#define SAR_CTRL_SAR_OVR_CLK_BIT_LEN         (1U)
#define SAR_CTRL_SAR_OVR_CLK_BIT_MASK        0x4U
#define SAR_CTRL_SAR_OVR_EN_BIT_OFFSET       (1U)
#define SAR_CTRL_SAR_OVR_EN_BIT_LEN          (1U)
#define SAR_CTRL_SAR_OVR_EN_BIT_MASK         0x2U
#define SAR_CTRL_SAR_START_BIT_OFFSET        (0U)
#define SAR_CTRL_SAR_START_BIT_LEN           (1U)
#define SAR_CTRL_SAR_START_BIT_MASK          0x1U

/******************************************************************************
* @brief Bit definitions for register SAR_STATUS
**/
#define SAR_STATUS_ID                        0x80004
#define SAR_STATUS_LEN                       (4U)
#define SAR_STATUS_MASK                      0xFFFFFFFFUL
#define SAR_STATUS_SAR_STATUS_BIT_OFFSET     (1U)
#define SAR_STATUS_SAR_STATUS_BIT_LEN        (6U)
#define SAR_STATUS_SAR_STATUS_BIT_MASK       0x7eU
#define SAR_STATUS_SAR_CONV_DONE_BIT_OFFSET  (0U)
#define SAR_STATUS_SAR_CONV_DONE_BIT_LEN     (1U)
#define SAR_STATUS_SAR_CONV_DONE_BIT_MASK    0x1U

/******************************************************************************
* @brief Bit definitions for register SAR_READING
**/
#define SAR_READING_ID                       0x80008
#define SAR_READING_LEN                      (4U)
#define SAR_READING_MASK                     0xFFFFFFFFUL
#define SAR_READING_SAR_READING_TXLVL_BIT_OFFSET (16U)
#define SAR_READING_SAR_READING_TXLVL_BIT_LEN (8U)
#define SAR_READING_SAR_READING_TXLVL_BIT_MASK 0xff0000UL
#define SAR_READING_SAR_READING_TEMP_BIT_OFFSET (8U)
#define SAR_READING_SAR_READING_TEMP_BIT_LEN (8U)
#define SAR_READING_SAR_READING_TEMP_BIT_MASK 0xff00U
#define SAR_READING_SAR_READING_VBAT_BIT_OFFSET (0U)
#define SAR_READING_SAR_READING_VBAT_BIT_LEN (8U)
#define SAR_READING_SAR_READING_VBAT_BIT_MASK 0xffU

/******************************************************************************
* @brief Bit definitions for register SAR_OLD_READ
**/
#define SAR_OLD_READ_ID                      0x8000c
#define SAR_OLD_READ_LEN                     (4U)
#define SAR_OLD_READ_MASK                    0xFFFFFFFFUL
#define SAR_OLD_READ_SAR_LAST_TEMP_BIT_OFFSET (8U)
#define SAR_OLD_READ_SAR_LAST_TEMP_BIT_LEN   (8U)
#define SAR_OLD_READ_SAR_LAST_TEMP_BIT_MASK  0xff00U
#define SAR_OLD_READ_SAR_LAST_VBAT_BIT_OFFSET (0U)
#define SAR_OLD_READ_SAR_LAST_VBAT_BIT_LEN   (8U)
#define SAR_OLD_READ_SAR_LAST_VBAT_BIT_MASK  0xffU

/******************************************************************************
* @brief Bit definitions for register PGC_CTRL
**/
#define PGC_CTRL_ID                          0x80010
#define PGC_CTRL_LEN                         (4U)
#define PGC_CTRL_MASK                        0xFFFFFFFFUL
#define PGC_CTRL_PGC_START_VAL_BIT_OFFSET    (8U)
#define PGC_CTRL_PGC_START_VAL_BIT_LEN       (1U)
#define PGC_CTRL_PGC_START_VAL_BIT_MASK      0x100U
#define PGC_CTRL_CAL_DIR_BIT_OFFSET          (7U)
#define PGC_CTRL_CAL_DIR_BIT_LEN             (1U)
#define PGC_CTRL_CAL_DIR_BIT_MASK            0x80U
#define PGC_CTRL_CAL_ON_TX_BIT_OFFSET        (6U)
#define PGC_CTRL_CAL_ON_TX_BIT_LEN           (1U)
#define PGC_CTRL_CAL_ON_TX_BIT_MASK          0x40U
#define PGC_CTRL_PGC_TMEAS_BIT_OFFSET        (2U)
#define PGC_CTRL_PGC_TMEAS_BIT_LEN           (4U)
#define PGC_CTRL_PGC_TMEAS_BIT_MASK          0x3cU
#define PGC_CTRL_PGC_AUTO_CAL_BIT_OFFSET     (1U)
#define PGC_CTRL_PGC_AUTO_CAL_BIT_LEN        (1U)
#define PGC_CTRL_PGC_AUTO_CAL_BIT_MASK       0x2U
#define PGC_CTRL_PGC_START_BIT_OFFSET        (0U)
#define PGC_CTRL_PGC_START_BIT_LEN           (1U)
#define PGC_CTRL_PGC_START_BIT_MASK          0x1U

/******************************************************************************
* @brief Bit definitions for register PGC_STATUS
**/
#define PGC_STATUS_ID                        0x80014
#define PGC_STATUS_LEN                       (4U)
#define PGC_STATUS_MASK                      0xFFFFFFFFUL
#define PGC_STATUS_AUTOCAL_DONE_BIT_OFFSET   (12U)
#define PGC_STATUS_AUTOCAL_DONE_BIT_LEN      (1U)
#define PGC_STATUS_AUTOCAL_DONE_BIT_MASK     0x1000U
#define PGC_STATUS_PG_DELAY_COUNT_BIT_OFFSET (0U)
#define PGC_STATUS_PG_DELAY_COUNT_BIT_LEN    (12U)
#define PGC_STATUS_PG_DELAY_COUNT_BIT_MASK   0xfffU

/******************************************************************************
* @brief Bit definitions for register PG_TEST
**/
#define PG_TEST_ID                           0x80018
#define PG_TEST_LEN                          (4U)
#define PG_TEST_MASK                         0xFFFFFFFFUL
#define PG_TEST_TX_TEST_CH4_BIT_OFFSET       (12U)
#define PG_TEST_TX_TEST_CH4_BIT_LEN          (4U)
#define PG_TEST_TX_TEST_CH4_BIT_MASK         0xf000U
#define PG_TEST_TX_TEST_CH3_BIT_OFFSET       (8U)
#define PG_TEST_TX_TEST_CH3_BIT_LEN          (4U)
#define PG_TEST_TX_TEST_CH3_BIT_MASK         0xf00U
#define PG_TEST_TX_TEST_CH2_BIT_OFFSET       (4U)
#define PG_TEST_TX_TEST_CH2_BIT_LEN          (4U)
#define PG_TEST_TX_TEST_CH2_BIT_MASK         0xf0U
#define PG_TEST_TX_TEST_CH1_BIT_OFFSET       (0U)
#define PG_TEST_TX_TEST_CH1_BIT_LEN          (4U)
#define PG_TEST_TX_TEST_CH1_BIT_MASK         0xfU

/******************************************************************************
* @brief Bit definitions for register PG_CAL_TARGET
**/
#define PG_CAL_TARGET_ID                     0x8001c
#define PG_CAL_TARGET_LEN                    (4U)
#define PG_CAL_TARGET_MASK                   0xFFFFFFFFUL
#define PG_CAL_TARGET_TARGET_BIT_OFFSET      (0U)
#define PG_CAL_TARGET_TARGET_BIT_LEN         (12U)
#define PG_CAL_TARGET_TARGET_BIT_MASK        0xfffU

/******************************************************************************
* @brief Bit definitions for register DELTA_VBAT
**/
#define DELTA_VBAT_ID                        0x80020
#define DELTA_VBAT_LEN                       (4U)
#define DELTA_VBAT_MASK                      0xFFFFFFFFUL
#define DELTA_VBAT_VBAT_EN_BIT_OFFSET        (7U)
#define DELTA_VBAT_VBAT_EN_BIT_LEN           (1U)
#define DELTA_VBAT_VBAT_EN_BIT_MASK          0x80U
#define DELTA_VBAT_VBAT_BIT_OFFSET           (0U)
#define DELTA_VBAT_VBAT_BIT_LEN              (7U)
#define DELTA_VBAT_VBAT_BIT_MASK             0x7fU

/******************************************************************************
* @brief Bit definitions for register DELTA_TEMP
**/
#define DELTA_TEMP_ID                        0x80024
#define DELTA_TEMP_LEN                       (4U)
#define DELTA_TEMP_MASK                      0xFFFFFFFFUL
#define DELTA_TEMP_TEMP_EN_BIT_OFFSET        (7U)
#define DELTA_TEMP_TEMP_EN_BIT_LEN           (1U)
#define DELTA_TEMP_TEMP_EN_BIT_MASK          0x80U
#define DELTA_TEMP_TEMP_BIT_OFFSET           (0U)
#define DELTA_TEMP_TEMP_BIT_LEN              (7U)
#define DELTA_TEMP_TEMP_BIT_MASK             0x7fU

/******************************************************************************
* @brief Bit definitions for register DELTA_IRQ
**/
#define DELTA_IRQ_ID                         0x80028
#define DELTA_IRQ_LEN                        (4U)
#define DELTA_IRQ_MASK                       0xFFFFFFFFUL
#define DELTA_IRQ_UPDATE_VBAT_COMP_BIT_OFFSET (3U)
#define DELTA_IRQ_UPDATE_VBAT_COMP_BIT_LEN   (1U)
#define DELTA_IRQ_UPDATE_VBAT_COMP_BIT_MASK  0x8U
#define DELTA_IRQ_UPDATE_TEMP_COMP_BIT_OFFSET (2U)
#define DELTA_IRQ_UPDATE_TEMP_COMP_BIT_LEN   (1U)
#define DELTA_IRQ_UPDATE_TEMP_COMP_BIT_MASK  0x4U
#define DELTA_IRQ_VTEMP_IRQ_STATUS_BIT_OFFSET (1U)
#define DELTA_IRQ_VTEMP_IRQ_STATUS_BIT_LEN   (1U)
#define DELTA_IRQ_VTEMP_IRQ_STATUS_BIT_MASK  0x2U
#define DELTA_IRQ_VBAT_IRQ_STATUS_BIT_OFFSET (0U)
#define DELTA_IRQ_VBAT_IRQ_STATUS_BIT_LEN    (1U)
#define DELTA_IRQ_VBAT_IRQ_STATUS_BIT_MASK   0x1U

/******************************************************************************
* @brief Bit definitions for register FOSC_CAL
**/
#define FOSC_CAL_ID                          0x8002c
#define FOSC_CAL_LEN                         (4U)
#define FOSC_CAL_MASK                        0xFFFFFFFFUL
#define FOSC_CAL_FOSC_CAL_DONE_BIT_OFFSET    (16U)
#define FOSC_CAL_FOSC_CAL_DONE_BIT_LEN       (1U)
#define FOSC_CAL_FOSC_CAL_DONE_BIT_MASK      0x10000UL
#define FOSC_CAL_FOSC_COUNT_BIT_OFFSET       (8U)
#define FOSC_CAL_FOSC_COUNT_BIT_LEN          (8U)
#define FOSC_CAL_FOSC_COUNT_BIT_MASK         0xff00U
#define FOSC_CAL_FOSC_CAL_EN_BIT_OFFSET      (0U)
#define FOSC_CAL_FOSC_CAL_EN_BIT_LEN         (1U)
#define FOSC_CAL_FOSC_CAL_EN_BIT_MASK        0x1U

/******************************************************************************
* @brief Bit definitions for register PLL_CFG
**/
#define PLL_CFG_ID                           0x90000
#define PLL_CFG_LEN                          (4U)
#define PLL_CFG_MASK                         0xFFFFFFFFUL
#define PLL_CFG_PLL_LD_SEL_BIT_OFFSET        (12U)
#define PLL_CFG_PLL_LD_SEL_BIT_LEN           (2U)
#define PLL_CFG_PLL_LD_SEL_BIT_MASK          0x3000U
#define PLL_CFG_CH5_CP_ITUNE_BIT_OFFSET      (9U)
#define PLL_CFG_CH5_CP_ITUNE_BIT_LEN         (3U)
#define PLL_CFG_CH5_CP_ITUNE_BIT_MASK        0xe00U
#define PLL_CFG_CH5_WD_COMP_TUNE_BIT_OFFSET  (6U)
#define PLL_CFG_CH5_WD_COMP_TUNE_BIT_LEN     (3U)
#define PLL_CFG_CH5_WD_COMP_TUNE_BIT_MASK    0x1c0U
#define PLL_CFG_CH9_CP_ITUNE_BIT_OFFSET      (3U)
#define PLL_CFG_CH9_CP_ITUNE_BIT_LEN         (3U)
#define PLL_CFG_CH9_CP_ITUNE_BIT_MASK        0x38U
#define PLL_CFG_CH9_WD_COMP_TUNE_BIT_OFFSET  (0U)
#define PLL_CFG_CH9_WD_COMP_TUNE_BIT_LEN     (3U)
#define PLL_CFG_CH9_WD_COMP_TUNE_BIT_MASK    0x7U

/******************************************************************************
* @brief Bit definitions for register PLL_COARSE_CODE
**/
#define PLL_COARSE_CODE_ID                   0x90004
#define PLL_COARSE_CODE_LEN                  (4U)
#define PLL_COARSE_CODE_MASK                 0xFFFFFFFFUL
#define PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_OFFSET (24U)
#define PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_LEN (1U)
#define PLL_COARSE_CODE_CH9_RVCO_FREQ_BOOST_BIT_MASK 0x1000000UL
#define PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_OFFSET (8U)
#define PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_LEN (14U)
#define PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_MASK 0x3fff00UL
#define PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET (0U)
#define PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN (5U)
#define PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register PLL_CAL
**/
#define PLL_CAL_ID                           0x90008
#define PLL_CAL_LEN                          (4U)
#define PLL_CAL_MASK                         0xFFFFFFFFUL
#define PLL_CAL_PLL_WD_EN_BIT_OFFSET         (19U)
#define PLL_CAL_PLL_WD_EN_BIT_LEN            (1U)
#define PLL_CAL_PLL_WD_EN_BIT_MASK           0x80000UL
#define PLL_CAL_PLL_DBG_CLR_BIT_OFFSET       (18U)
#define PLL_CAL_PLL_DBG_CLR_BIT_LEN          (1U)
#define PLL_CAL_PLL_DBG_CLR_BIT_MASK         0x40000UL
#define PLL_CAL_PLL_DBG_EN_BIT_OFFSET        (17U)
#define PLL_CAL_PLL_DBG_EN_BIT_LEN           (1U)
#define PLL_CAL_PLL_DBG_EN_BIT_MASK          0x20000UL
#define PLL_CAL_PLL_CAL_CLR_BIT_OFFSET       (16U)
#define PLL_CAL_PLL_CAL_CLR_BIT_LEN          (1U)
#define PLL_CAL_PLL_CAL_CLR_BIT_MASK         0x10000UL
#define PLL_CAL_PLL_CAL_EN_BIT_OFFSET        (8U)
#define PLL_CAL_PLL_CAL_EN_BIT_LEN           (1U)
#define PLL_CAL_PLL_CAL_EN_BIT_MASK          0x100U
#define PLL_CAL_PLL_LOCK_DLY_BIT_OFFSET      (3U)
#define PLL_CAL_PLL_LOCK_DLY_BIT_LEN         (5U)
#define PLL_CAL_PLL_LOCK_DLY_BIT_MASK        0xf8U
#define PLL_CAL_PLL_TUNE_OVR_BIT_OFFSET      (2U)
#define PLL_CAL_PLL_TUNE_OVR_BIT_LEN         (1U)
#define PLL_CAL_PLL_TUNE_OVR_BIT_MASK        0x4U
#define PLL_CAL_PLL_USE_OLD_BIT_OFFSET       (1U)
#define PLL_CAL_PLL_USE_OLD_BIT_LEN          (1U)
#define PLL_CAL_PLL_USE_OLD_BIT_MASK         0x2U
#define PLL_CAL_PLL_CH9_FB_OVR_BIT_OFFSET    (0U)
#define PLL_CAL_PLL_CH9_FB_OVR_BIT_LEN       (1U)
#define PLL_CAL_PLL_CH9_FB_OVR_BIT_MASK      0x1U

/******************************************************************************
* @brief Bit definitions for register PLL_STATUS
**/
#define PLL_STATUS_ID                        0x9000c
#define PLL_STATUS_LEN                       (4U)
#define PLL_STATUS_MASK                      0xFFFFFFFFUL
#define PLL_STATUS_LD_CODE_BIT_OFFSET        (8U)
#define PLL_STATUS_LD_CODE_BIT_LEN           (5U)
#define PLL_STATUS_LD_CODE_BIT_MASK          0x1f00U
#define PLL_STATUS_XTAL_AMP_SETTLED_BIT_OFFSET (6U)
#define PLL_STATUS_XTAL_AMP_SETTLED_BIT_LEN  (1U)
#define PLL_STATUS_XTAL_AMP_SETTLED_BIT_MASK 0x40U
#define PLL_STATUS_VCO_TUNE_UPDATE_BIT_OFFSET (5U)
#define PLL_STATUS_VCO_TUNE_UPDATE_BIT_LEN   (1U)
#define PLL_STATUS_VCO_TUNE_UPDATE_BIT_MASK  0x20U
#define PLL_STATUS_PLL_OVRFLOW_BIT_OFFSET    (4U)
#define PLL_STATUS_PLL_OVRFLOW_BIT_LEN       (1U)
#define PLL_STATUS_PLL_OVRFLOW_BIT_MASK      0x10U
#define PLL_STATUS_PLL_HI_FLAG_BIT_OFFSET    (3U)
#define PLL_STATUS_PLL_HI_FLAG_BIT_LEN       (1U)
#define PLL_STATUS_PLL_HI_FLAG_BIT_MASK      0x8U
#define PLL_STATUS_PLL_LO_FLAG_N_BIT_OFFSET  (2U)
#define PLL_STATUS_PLL_LO_FLAG_N_BIT_LEN     (1U)
#define PLL_STATUS_PLL_LO_FLAG_N_BIT_MASK    0x4U
#define PLL_STATUS_PLL_LOCK_FLAG_BIT_OFFSET  (1U)
#define PLL_STATUS_PLL_LOCK_FLAG_BIT_LEN     (1U)
#define PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK    0x2U
#define PLL_STATUS_CPC_CAL_DONE_BIT_OFFSET   (0U)
#define PLL_STATUS_CPC_CAL_DONE_BIT_LEN      (1U)
#define PLL_STATUS_CPC_CAL_DONE_BIT_MASK     0x1U

/******************************************************************************
* @brief Bit definitions for register PLL_COMMON
**/
#define PLL_COMMON_ID                        0x90010
#define PLL_COMMON_LEN                       (4U)
#define PLL_COMMON_MASK                      0xFFFFFFFFUL
#define PLL_COMMON_PLL_BIAS_TRIM_BIT_OFFSET  (13U)
#define PLL_COMMON_PLL_BIAS_TRIM_BIT_LEN     (3U)
#define PLL_COMMON_PLL_BIAS_TRIM_BIT_MASK    0xe000U
#define PLL_COMMON_DIG_PLL_WD_SEL_REF_CLK_DIVBY16_ULV_BIT_OFFSET (12U)
#define PLL_COMMON_DIG_PLL_WD_SEL_REF_CLK_DIVBY16_ULV_BIT_LEN (1U)
#define PLL_COMMON_DIG_PLL_WD_SEL_REF_CLK_DIVBY16_ULV_BIT_MASK 0x1000U
#define PLL_COMMON_DIG_PLL_WD_SEL_EXT_CLK_ULV_BIT_OFFSET (11U)
#define PLL_COMMON_DIG_PLL_WD_SEL_EXT_CLK_ULV_BIT_LEN (1U)
#define PLL_COMMON_DIG_PLL_WD_SEL_EXT_CLK_ULV_BIT_MASK 0x800U
#define PLL_COMMON_DIG_WD_EXT_CLK_ULV_BIT_OFFSET (10U)
#define PLL_COMMON_DIG_WD_EXT_CLK_ULV_BIT_LEN (1U)
#define PLL_COMMON_DIG_WD_EXT_CLK_ULV_BIT_MASK 0x400U
#define PLL_COMMON_PLL_WD_HYST_BOOST_BIT_OFFSET (9U)
#define PLL_COMMON_PLL_WD_HYST_BOOST_BIT_LEN (1U)
#define PLL_COMMON_PLL_WD_HYST_BOOST_BIT_MASK 0x200U
#define PLL_COMMON_PLL_PTAT_SEL_BIT_OFFSET   (8U)
#define PLL_COMMON_PLL_PTAT_SEL_BIT_LEN      (1U)
#define PLL_COMMON_PLL_PTAT_SEL_BIT_MASK     0x100U
#define PLL_COMMON_PLL_PFD_DLY_TUNE_BIT_OFFSET (4U)
#define PLL_COMMON_PLL_PFD_DLY_TUNE_BIT_LEN  (4U)
#define PLL_COMMON_PLL_PFD_DLY_TUNE_BIT_MASK 0xf0U
#define PLL_COMMON_PLL_LD_DLY_TUNE_BIT_OFFSET (0U)
#define PLL_COMMON_PLL_LD_DLY_TUNE_BIT_LEN   (4U)
#define PLL_COMMON_PLL_LD_DLY_TUNE_BIT_MASK  0xfU

/******************************************************************************
* @brief Bit definitions for register XTAL
**/
#define XTAL_ID                              0x90014
#define XTAL_LEN                             (4U)
#define XTAL_MASK                            0xFFFFFFFFUL
#define XTAL_XTAL_SPARE_BIT_OFFSET           (28U)
#define XTAL_XTAL_SPARE_BIT_LEN              (4U)
#define XTAL_XTAL_SPARE_BIT_MASK             0xf0000000UL
#define XTAL_XTAL_R2R_SEQ_EN_BIT_OFFSET      (27U)
#define XTAL_XTAL_R2R_SEQ_EN_BIT_LEN         (1U)
#define XTAL_XTAL_R2R_SEQ_EN_BIT_MASK        0x8000000UL
#define XTAL_XTAL_R2R_GEN_BIT_OFFSET         (26U)
#define XTAL_XTAL_R2R_GEN_BIT_LEN            (1U)
#define XTAL_XTAL_R2R_GEN_BIT_MASK           0x4000000UL
#define XTAL_XTAL_AMP_OK_OVERRIDE_ULV_BIT_OFFSET (25U)
#define XTAL_XTAL_AMP_OK_OVERRIDE_ULV_BIT_LEN (1U)
#define XTAL_XTAL_AMP_OK_OVERRIDE_ULV_BIT_MASK 0x2000000UL
#define XTAL_XTAL_CLK_CAPDAC_SEL_ULV_BIT_OFFSET (23U)
#define XTAL_XTAL_CLK_CAPDAC_SEL_ULV_BIT_LEN (2U)
#define XTAL_XTAL_CLK_CAPDAC_SEL_ULV_BIT_MASK 0x1800000UL
#define XTAL_XTAL_DIGTEST_EN_ULV_BIT_OFFSET  (22U)
#define XTAL_XTAL_DIGTEST_EN_ULV_BIT_LEN     (1U)
#define XTAL_XTAL_DIGTEST_EN_ULV_BIT_MASK    0x400000UL
#define XTAL_XTAL_DIGTEST_SEL_ULV_BIT_OFFSET (19U)
#define XTAL_XTAL_DIGTEST_SEL_ULV_BIT_LEN    (3U)
#define XTAL_XTAL_DIGTEST_SEL_ULV_BIT_MASK   0x380000UL
#define XTAL_XTAL_BIASSEL_ULV_BIT_OFFSET     (17U)
#define XTAL_XTAL_BIASSEL_ULV_BIT_LEN        (2U)
#define XTAL_XTAL_BIASSEL_ULV_BIT_MASK       0x60000UL
#define XTAL_XTAL_IFIXED_PROG_ULV_BIT_OFFSET (14U)
#define XTAL_XTAL_IFIXED_PROG_ULV_BIT_LEN    (3U)
#define XTAL_XTAL_IFIXED_PROG_ULV_BIT_MASK   0x1c000UL
#define XTAL_XTAL_IVAR_PROG_ULV_BIT_OFFSET   (11U)
#define XTAL_XTAL_IVAR_PROG_ULV_BIT_LEN      (3U)
#define XTAL_XTAL_IVAR_PROG_ULV_BIT_MASK     0x3800U
#define XTAL_XTAL_REFCLK_COMP_EN_ULV_BIT_OFFSET (10U)
#define XTAL_XTAL_REFCLK_COMP_EN_ULV_BIT_LEN (1U)
#define XTAL_XTAL_REFCLK_COMP_EN_ULV_BIT_MASK 0x400U
#define XTAL_XTAL_VAMP_DET_RANGE_CRTL_ULV_BIT_OFFSET (8U)
#define XTAL_XTAL_VAMP_DET_RANGE_CRTL_ULV_BIT_LEN (2U)
#define XTAL_XTAL_VAMP_DET_RANGE_CRTL_ULV_BIT_MASK 0x300U
#define XTAL_XTAL_TRIM_BIT_OFFSET            (0U)
#define XTAL_XTAL_TRIM_BIT_LEN               (7U)
#define XTAL_XTAL_TRIM_BIT_MASK              0x7fU

/******************************************************************************
* @brief Bit definitions for register PLL_LOCK_TIME_DBG
**/
#define PLL_LOCK_TIME_DBG_ID                 0x90018
#define PLL_LOCK_TIME_DBG_LEN                (4U)
#define PLL_LOCK_TIME_DBG_MASK               0xFFFFFFFFUL
#define PLL_LOCK_TIME_DBG_CLKPLL_CH5_LOCK_EVENTS_BIT_OFFSET (8U)
#define PLL_LOCK_TIME_DBG_CLKPLL_CH5_LOCK_EVENTS_BIT_LEN (4U)
#define PLL_LOCK_TIME_DBG_CLKPLL_CH5_LOCK_EVENTS_BIT_MASK 0xf00U
#define PLL_LOCK_TIME_DBG_CLKPLL_CH5_LOCK_TIME_BIT_OFFSET (0U)
#define PLL_LOCK_TIME_DBG_CLKPLL_CH5_LOCK_TIME_BIT_LEN (8U)
#define PLL_LOCK_TIME_DBG_CLKPLL_CH5_LOCK_TIME_BIT_MASK 0xffU

/******************************************************************************
* @brief Bit definitions for register AON_DIG_CFG
**/
#define AON_DIG_CFG_ID                       0xa0000
#define AON_DIG_CFG_LEN                      (4U)
#define AON_DIG_CFG_MASK                     0xFFFFFFFFUL
#define AON_DIG_CFG_ONWAKE_LOAD_DGC_ALT_BIT_OFFSET (18U)
#define AON_DIG_CFG_ONWAKE_LOAD_DGC_ALT_BIT_LEN (1U)
#define AON_DIG_CFG_ONWAKE_LOAD_DGC_ALT_BIT_MASK 0x40000UL
#define AON_DIG_CFG_VWARN_COMP_POL_BIT_OFFSET (17U)
#define AON_DIG_CFG_VWARN_COMP_POL_BIT_LEN   (1U)
#define AON_DIG_CFG_VWARN_COMP_POL_BIT_MASK  0x20000UL
#define AON_DIG_CFG_VWARN_COMP_TRIM_BIT_OFFSET (13U)
#define AON_DIG_CFG_VWARN_COMP_TRIM_BIT_LEN  (4U)
#define AON_DIG_CFG_VWARN_COMP_TRIM_BIT_MASK 0x1e000UL
#define AON_DIG_CFG_VWARN_COMP_EN_BIT_OFFSET (12U)
#define AON_DIG_CFG_VWARN_COMP_EN_BIT_LEN    (1U)
#define AON_DIG_CFG_VWARN_COMP_EN_BIT_MASK   0x1000U
#define AON_DIG_CFG_ONWAKE_PGFCAL_BIT_OFFSET (11U)
#define AON_DIG_CFG_ONWAKE_PGFCAL_BIT_LEN    (1U)
#define AON_DIG_CFG_ONWAKE_PGFCAL_BIT_MASK   0x800U
#define AON_DIG_CFG_ONWAKE_PLLVAL_BIT_OFFSET (10U)
#define AON_DIG_CFG_ONWAKE_PLLVAL_BIT_LEN    (1U)
#define AON_DIG_CFG_ONWAKE_PLLVAL_BIT_MASK   0x400U
#define AON_DIG_CFG_ONWAKE_GO2RX_BIT_OFFSET  (9U)
#define AON_DIG_CFG_ONWAKE_GO2RX_BIT_LEN     (1U)
#define AON_DIG_CFG_ONWAKE_GO2RX_BIT_MASK    0x200U
#define AON_DIG_CFG_ONWAKE_GO2IDLE_BIT_OFFSET (8U)
#define AON_DIG_CFG_ONWAKE_GO2IDLE_BIT_LEN   (1U)
#define AON_DIG_CFG_ONWAKE_GO2IDLE_BIT_MASK  0x100U
#define AON_DIG_CFG_ONWAKE_GEAR_SEL_BIT_OFFSET (6U)
#define AON_DIG_CFG_ONWAKE_GEAR_SEL_BIT_LEN  (2U)
#define AON_DIG_CFG_ONWAKE_GEAR_SEL_BIT_MASK 0xc0U
#define AON_DIG_CFG_ONWAKE_LOAD_GEAR_BIT_OFFSET (5U)
#define AON_DIG_CFG_ONWAKE_LOAD_GEAR_BIT_LEN (1U)
#define AON_DIG_CFG_ONWAKE_LOAD_GEAR_BIT_MASK 0x20U
#define AON_DIG_CFG_ONWAKE_LOAD_LDO_BIT_OFFSET (4U)
#define AON_DIG_CFG_ONWAKE_LOAD_LDO_BIT_LEN  (1U)
#define AON_DIG_CFG_ONWAKE_LOAD_LDO_BIT_MASK 0x10U
#define AON_DIG_CFG_ONWAKE_LOAD_DGC_BIT_OFFSET (3U)
#define AON_DIG_CFG_ONWAKE_LOAD_DGC_BIT_LEN  (1U)
#define AON_DIG_CFG_ONWAKE_LOAD_DGC_BIT_MASK 0x8U
#define AON_DIG_CFG_ONWAKE_LOAD_BIAS_BIT_OFFSET (2U)
#define AON_DIG_CFG_ONWAKE_LOAD_BIAS_BIT_LEN (1U)
#define AON_DIG_CFG_ONWAKE_LOAD_BIAS_BIT_MASK 0x4U
#define AON_DIG_CFG_ONWAKE_RUN_SAR_BIT_OFFSET (1U)
#define AON_DIG_CFG_ONWAKE_RUN_SAR_BIT_LEN   (1U)
#define AON_DIG_CFG_ONWAKE_RUN_SAR_BIT_MASK  0x2U
#define AON_DIG_CFG_ONWAKE_AON_DLD_BIT_OFFSET (0U)
#define AON_DIG_CFG_ONWAKE_AON_DLD_BIT_LEN   (1U)
#define AON_DIG_CFG_ONWAKE_AON_DLD_BIT_MASK  0x1U

/******************************************************************************
* @brief Bit definitions for register AON_CTRL
**/
#define AON_CTRL_ID                          0xa0004
#define AON_CTRL_LEN                         (4U)
#define AON_CTRL_MASK                        0xFFFFFFFFUL
#define AON_CTRL_OVERRIDE_EN_BIT_OFFSET      (7U)
#define AON_CTRL_OVERRIDE_EN_BIT_LEN         (1U)
#define AON_CTRL_OVERRIDE_EN_BIT_MASK        0x80U
#define AON_CTRL_AON_CLK_EDGE_SEL_BIT_OFFSET (6U)
#define AON_CTRL_AON_CLK_EDGE_SEL_BIT_LEN    (1U)
#define AON_CTRL_AON_CLK_EDGE_SEL_BIT_MASK   0x40U
#define AON_CTRL_OVR_WR_CFG_EN_BIT_OFFSET    (5U)
#define AON_CTRL_OVR_WR_CFG_EN_BIT_LEN       (1U)
#define AON_CTRL_OVR_WR_CFG_EN_BIT_MASK      0x20U
#define AON_CTRL_OVR_WRITE_EN_BIT_OFFSET     (4U)
#define AON_CTRL_OVR_WRITE_EN_BIT_LEN        (1U)
#define AON_CTRL_OVR_WRITE_EN_BIT_MASK       0x10U
#define AON_CTRL_OVR_READ_EN_BIT_OFFSET      (3U)
#define AON_CTRL_OVR_READ_EN_BIT_LEN         (1U)
#define AON_CTRL_OVR_READ_EN_BIT_MASK        0x8U
#define AON_CTRL_CONFIG_UPLOAD_BIT_OFFSET    (2U)
#define AON_CTRL_CONFIG_UPLOAD_BIT_LEN       (1U)
#define AON_CTRL_CONFIG_UPLOAD_BIT_MASK      0x4U
#define AON_CTRL_ARRAY_UPLOAD_BIT_OFFSET     (1U)
#define AON_CTRL_ARRAY_UPLOAD_BIT_LEN        (1U)
#define AON_CTRL_ARRAY_UPLOAD_BIT_MASK       0x2U
#define AON_CTRL_ARRAY_DOWNLOAD_BIT_OFFSET   (0U)
#define AON_CTRL_ARRAY_DOWNLOAD_BIT_LEN      (1U)
#define AON_CTRL_ARRAY_DOWNLOAD_BIT_MASK     0x1U

/******************************************************************************
* @brief Bit definitions for register AON_RDATA
**/
#define AON_RDATA_ID                         0xa0008
#define AON_RDATA_LEN                        (4U)
#define AON_RDATA_MASK                       0xFFFFFFFFUL
#define AON_RDATA_RDATA_BIT_OFFSET           (0U)
#define AON_RDATA_RDATA_BIT_LEN              (8U)
#define AON_RDATA_RDATA_BIT_MASK             0xffU

/******************************************************************************
* @brief Bit definitions for register AON_ADDR
**/
#define AON_ADDR_ID                          0xa000c
#define AON_ADDR_LEN                         (4U)
#define AON_ADDR_MASK                        0xFFFFFFFFUL
#define AON_ADDR_ADDR_BIT_OFFSET             (0U)
#define AON_ADDR_ADDR_BIT_LEN                (9U)
#define AON_ADDR_ADDR_BIT_MASK               0x1ffU

/******************************************************************************
* @brief Bit definitions for register AON_WDATA
**/
#define AON_WDATA_ID                         0xa0010
#define AON_WDATA_LEN                        (4U)
#define AON_WDATA_MASK                       0xFFFFFFFFUL
#define AON_WDATA_WDATA_BIT_OFFSET           (0U)
#define AON_WDATA_WDATA_BIT_LEN              (8U)
#define AON_WDATA_WDATA_BIT_MASK             0xffU

/******************************************************************************
* @brief Bit definitions for register ANA_CFG
**/
#define ANA_CFG_ID                           0xa0014
#define ANA_CFG_LEN                          (4U)
#define ANA_CFG_MASK                         0xFFFFFFFFUL
#define ANA_CFG_SLEEP_CNT_RPT_BIT_OFFSET     (6U)
#define ANA_CFG_SLEEP_CNT_RPT_BIT_LEN        (1U)
#define ANA_CFG_SLEEP_CNT_RPT_BIT_MASK       0x40U
#define ANA_CFG_SLEEP_PRESERVE_BIT_OFFSET    (5U)
#define ANA_CFG_SLEEP_PRESERVE_BIT_LEN       (1U)
#define ANA_CFG_SLEEP_PRESERVE_BIT_MASK      0x20U
#define ANA_CFG_WAKEON_WUP_BIT_OFFSET        (4U)
#define ANA_CFG_WAKEON_WUP_BIT_LEN           (1U)
#define ANA_CFG_WAKEON_WUP_BIT_MASK          0x10U
#define ANA_CFG_WAKEON_CSN_BIT_OFFSET        (3U)
#define ANA_CFG_WAKEON_CSN_BIT_LEN           (1U)
#define ANA_CFG_WAKEON_CSN_BIT_MASK          0x8U
#define ANA_CFG_BROUT_KEEP_ON_BIT_OFFSET     (2U)
#define ANA_CFG_BROUT_KEEP_ON_BIT_LEN        (1U)
#define ANA_CFG_BROUT_KEEP_ON_BIT_MASK       0x4U
#define ANA_CFG_DEEPSLEEP_EN_BIT_OFFSET      (1U)
#define ANA_CFG_DEEPSLEEP_EN_BIT_LEN         (1U)
#define ANA_CFG_DEEPSLEEP_EN_BIT_MASK        0x2U
#define ANA_CFG_SLEEP_EN_BIT_OFFSET          (0U)
#define ANA_CFG_SLEEP_EN_BIT_LEN             (1U)
#define ANA_CFG_SLEEP_EN_BIT_MASK            0x1U

/******************************************************************************
* @brief Bit definitions for register NVM_WDATA
**/
#define NVM_WDATA_ID                         0xb0000
#define NVM_WDATA_LEN                        (4U)
#define NVM_WDATA_MASK                       0xFFFFFFFFUL
#define NVM_WDATA_NVM_WDATA_BIT_OFFSET       (0U)
#define NVM_WDATA_NVM_WDATA_BIT_LEN          (32U)
#define NVM_WDATA_NVM_WDATA_BIT_MASK         0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register NVM_ADDR
**/
#define NVM_ADDR_ID                          0xb0004
#define NVM_ADDR_LEN                         (4U)
#define NVM_ADDR_MASK                        0xFFFFFFFFUL
#define NVM_ADDR_NVM_ADDR_BIT_OFFSET         (0U)
#define NVM_ADDR_NVM_ADDR_BIT_LEN            (11U)
#define NVM_ADDR_NVM_ADDR_BIT_MASK           0x7ffU

/******************************************************************************
* @brief Bit definitions for register NVM_CFG
**/
#define NVM_CFG_ID                           0xb0008
#define NVM_CFG_LEN                          (4U)
#define NVM_CFG_MASK                         0xFFFFFFFFUL
#define NVM_CFG_DGC_SEL_BIT_OFFSET           (13U)
#define NVM_CFG_DGC_SEL_BIT_LEN              (1U)
#define NVM_CFG_DGC_SEL_BIT_MASK             0x2000U
#define NVM_CFG_GEAR_ID_BIT_OFFSET           (11U)
#define NVM_CFG_GEAR_ID_BIT_LEN              (2U)
#define NVM_CFG_GEAR_ID_BIT_MASK             0x1800U
#define NVM_CFG_GEAR_KICK_BIT_OFFSET         (10U)
#define NVM_CFG_GEAR_KICK_BIT_LEN            (1U)
#define NVM_CFG_GEAR_KICK_BIT_MASK           0x400U
#define NVM_CFG_NVM_PD_BIT_OFFSET            (9U)
#define NVM_CFG_NVM_PD_BIT_LEN               (1U)
#define NVM_CFG_NVM_PD_BIT_MASK              0x200U
#define NVM_CFG_BIAS_KICK_BIT_OFFSET         (8U)
#define NVM_CFG_BIAS_KICK_BIT_LEN            (1U)
#define NVM_CFG_BIAS_KICK_BIT_MASK           0x100U
#define NVM_CFG_LDO_KICK_BIT_OFFSET          (7U)
#define NVM_CFG_LDO_KICK_BIT_LEN             (1U)
#define NVM_CFG_LDO_KICK_BIT_MASK            0x80U
#define NVM_CFG_DGC_KICK_BIT_OFFSET          (6U)
#define NVM_CFG_DGC_KICK_BIT_LEN             (1U)
#define NVM_CFG_DGC_KICK_BIT_MASK            0x40U
#define NVM_CFG_ADDR_INC_BIT_OFFSET          (5U)
#define NVM_CFG_ADDR_INC_BIT_LEN             (1U)
#define NVM_CFG_ADDR_INC_BIT_MASK            0x20U
#define NVM_CFG_NVM_MODE_SEL_BIT_OFFSET      (4U)
#define NVM_CFG_NVM_MODE_SEL_BIT_LEN         (1U)
#define NVM_CFG_NVM_MODE_SEL_BIT_MASK        0x10U
#define NVM_CFG_NVM_WRITE_MR_BIT_OFFSET      (3U)
#define NVM_CFG_NVM_WRITE_MR_BIT_LEN         (1U)
#define NVM_CFG_NVM_WRITE_MR_BIT_MASK        0x8U
#define NVM_CFG_NVM_WRITE_BIT_OFFSET         (2U)
#define NVM_CFG_NVM_WRITE_BIT_LEN            (1U)
#define NVM_CFG_NVM_WRITE_BIT_MASK           0x4U
#define NVM_CFG_NVM_READ_BIT_OFFSET          (1U)
#define NVM_CFG_NVM_READ_BIT_LEN             (1U)
#define NVM_CFG_NVM_READ_BIT_MASK            0x2U
#define NVM_CFG_NVM_MAN_CTR_EN_BIT_OFFSET    (0U)
#define NVM_CFG_NVM_MAN_CTR_EN_BIT_LEN       (1U)
#define NVM_CFG_NVM_MAN_CTR_EN_BIT_MASK      0x1U

/******************************************************************************
* @brief Bit definitions for register NVM_STATUS
**/
#define NVM_STATUS_ID                        0xb000c
#define NVM_STATUS_LEN                       (4U)
#define NVM_STATUS_MASK                      0xFFFFFFFFUL
#define NVM_STATUS_NVM_UNLOCKED_BIT_OFFSET   (11U)
#define NVM_STATUS_NVM_UNLOCKED_BIT_LEN      (1U)
#define NVM_STATUS_NVM_UNLOCKED_BIT_MASK     0x800U
#define NVM_STATUS_NVM_STATE_BIT_OFFSET      (7U)
#define NVM_STATUS_NVM_STATE_BIT_LEN         (4U)
#define NVM_STATUS_NVM_STATE_BIT_MASK        0x780U
#define NVM_STATUS_NVM_SEQ_STATE_BIT_OFFSET  (3U)
#define NVM_STATUS_NVM_SEQ_STATE_BIT_LEN     (4U)
#define NVM_STATUS_NVM_SEQ_STATE_BIT_MASK    0x78U
#define NVM_STATUS_NVM_SO_BIT_OFFSET         (2U)
#define NVM_STATUS_NVM_SO_BIT_LEN            (1U)
#define NVM_STATUS_NVM_SO_BIT_MASK           0x4U
#define NVM_STATUS_NVM_VPP_OK_BIT_OFFSET     (1U)
#define NVM_STATUS_NVM_VPP_OK_BIT_LEN        (1U)
#define NVM_STATUS_NVM_VPP_OK_BIT_MASK       0x2U
#define NVM_STATUS_NVM_PROG_DONE_BIT_OFFSET  (0U)
#define NVM_STATUS_NVM_PROG_DONE_BIT_LEN     (1U)
#define NVM_STATUS_NVM_PROG_DONE_BIT_MASK    0x1U

/******************************************************************************
* @brief Bit definitions for register NVM_RDATA
**/
#define NVM_RDATA_ID                         0xb0010
#define NVM_RDATA_LEN                        (4U)
#define NVM_RDATA_MASK                       0xFFFFFFFFUL
#define NVM_RDATA_NVM_RDATA_BIT_OFFSET       (0U)
#define NVM_RDATA_NVM_RDATA_BIT_LEN          (32U)
#define NVM_RDATA_NVM_RDATA_BIT_MASK         0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register NVM_SRDATA
**/
#define NVM_SRDATA_ID                        0xb0014
#define NVM_SRDATA_LEN                       (4U)
#define NVM_SRDATA_MASK                      0xFFFFFFFFUL
#define NVM_SRDATA_NVM_SRDATA_BIT_OFFSET     (0U)
#define NVM_SRDATA_NVM_SRDATA_BIT_LEN        (32U)
#define NVM_SRDATA_NVM_SRDATA_BIT_MASK       0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register IP_TOA_LO
**/
#define IP_TOA_LO_ID                         0xc0000
#define IP_TOA_LO_LEN                        (4U)
#define IP_TOA_LO_MASK                       0xFFFFFFFFUL
#define IP_TOA_LO_RX_TOA_BIT_OFFSET          (0U)
#define IP_TOA_LO_RX_TOA_BIT_LEN             (32U)
#define IP_TOA_LO_RX_TOA_BIT_MASK            0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register IP_TOA_HI
**/
#define IP_TOA_HI_ID                         0xc0004
#define IP_TOA_HI_LEN                        (4U)
#define IP_TOA_HI_MASK                       0xFFFFFFFFUL
#define IP_TOA_HI_STATUS_BIT_OFFSET          (24U)
#define IP_TOA_HI_STATUS_BIT_LEN             (8U)
#define IP_TOA_HI_STATUS_BIT_MASK            0xff000000UL
#define IP_TOA_HI_POA_BIT_OFFSET             (8U)
#define IP_TOA_HI_POA_BIT_LEN                (14U)
#define IP_TOA_HI_POA_BIT_MASK               0x3fff00UL
#define IP_TOA_HI_RX_TOA_BIT_OFFSET          (0U)
#define IP_TOA_HI_RX_TOA_BIT_LEN             (8U)
#define IP_TOA_HI_RX_TOA_BIT_MASK            0xffU

/******************************************************************************
* @brief Bit definitions for register CY0_TOA_LO
**/
#define CY0_TOA_LO_ID                        0xc0008
#define CY0_TOA_LO_LEN                       (4U)
#define CY0_TOA_LO_MASK                      0xFFFFFFFFUL
#define CY0_TOA_LO_RX_TOA_BIT_OFFSET         (0U)
#define CY0_TOA_LO_RX_TOA_BIT_LEN            (32U)
#define CY0_TOA_LO_RX_TOA_BIT_MASK           0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_TOA_HI
**/
#define CY0_TOA_HI_ID                        0xc000c
#define CY0_TOA_HI_LEN                       (4U)
#define CY0_TOA_HI_MASK                      0xFFFFFFFFUL
#define CY0_TOA_HI_STATUS_BIT_OFFSET         (23U)
#define CY0_TOA_HI_STATUS_BIT_LEN            (9U)
#define CY0_TOA_HI_STATUS_BIT_MASK           0xff800000UL
#define CY0_TOA_HI_POA_BIT_OFFSET            (8U)
#define CY0_TOA_HI_POA_BIT_LEN               (14U)
#define CY0_TOA_HI_POA_BIT_MASK              0x3fff00UL
#define CY0_TOA_HI_RXTOA_BIT_OFFSET          (0U)
#define CY0_TOA_HI_RXTOA_BIT_LEN             (8U)
#define CY0_TOA_HI_RXTOA_BIT_MASK            0xffU

/******************************************************************************
* @brief Bit definitions for register CY1_TOA_LO
**/
#define CY1_TOA_LO_ID                        0xc0010
#define CY1_TOA_LO_LEN                       (4U)
#define CY1_TOA_LO_MASK                      0xFFFFFFFFUL
#define CY1_TOA_LO_RX_TOA_BIT_OFFSET         (0U)
#define CY1_TOA_LO_RX_TOA_BIT_LEN            (32U)
#define CY1_TOA_LO_RX_TOA_BIT_MASK           0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_TOA_HI
**/
#define CY1_TOA_HI_ID                        0xc0014
#define CY1_TOA_HI_LEN                       (4U)
#define CY1_TOA_HI_MASK                      0xFFFFFFFFUL
#define CY1_TOA_HI_STATUS_BIT_OFFSET         (23U)
#define CY1_TOA_HI_STATUS_BIT_LEN            (9U)
#define CY1_TOA_HI_STATUS_BIT_MASK           0xff800000UL
#define CY1_TOA_HI_POA_BIT_OFFSET            (8U)
#define CY1_TOA_HI_POA_BIT_LEN               (14U)
#define CY1_TOA_HI_POA_BIT_MASK              0x3fff00UL
#define CY1_TOA_HI_RXTOA_BIT_OFFSET          (0U)
#define CY1_TOA_HI_RXTOA_BIT_LEN             (8U)
#define CY1_TOA_HI_RXTOA_BIT_MASK            0xffU

/******************************************************************************
* @brief Bit definitions for register CIA_TDOA_0
**/
#define CIA_TDOA_0_ID                        0xc0018
#define CIA_TDOA_0_LEN                       (4U)
#define CIA_TDOA_0_MASK                      0xFFFFFFFFUL
#define CIA_TDOA_0_RX_TDOA_BIT_OFFSET        (0U)
#define CIA_TDOA_0_RX_TDOA_BIT_LEN           (32U)
#define CIA_TDOA_0_RX_TDOA_BIT_MASK          0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register CIA_TDOA_1_PDOA
**/
#define CIA_TDOA_1_PDOA_ID                   0xc001c
#define CIA_TDOA_1_PDOA_LEN                  (4U)
#define CIA_TDOA_1_PDOA_MASK                 0xFFFFFFFFUL
#define CIA_TDOA_1_PDOA_FP_AGREED_BIT_OFFSET (30U)
#define CIA_TDOA_1_PDOA_FP_AGREED_BIT_LEN    (1U)
#define CIA_TDOA_1_PDOA_FP_AGREED_BIT_MASK   0x40000000UL
#define CIA_TDOA_1_PDOA_RX_PDOA_BIT_OFFSET   (16U)
#define CIA_TDOA_1_PDOA_RX_PDOA_BIT_LEN      (14U)
#define CIA_TDOA_1_PDOA_RX_PDOA_BIT_MASK     0x3fff0000UL
#define CIA_TDOA_1_PDOA_RX_TDOA_BIT_OFFSET   (0U)
#define CIA_TDOA_1_PDOA_RX_TDOA_BIT_LEN      (9U)
#define CIA_TDOA_1_PDOA_RX_TDOA_BIT_MASK     0x1ffU

/******************************************************************************
* @brief Bit definitions for register CIA_DIAG_0
**/
#define CIA_DIAG_0_ID                        0xc0020
#define CIA_DIAG_0_LEN                       (4U)
#define CIA_DIAG_0_MASK                      0xFFFFFFFFUL
#define CIA_DIAG_0_ISOLATION_INDEX0_BIT_OFFSET (27U)
#define CIA_DIAG_0_ISOLATION_INDEX0_BIT_LEN  (5U)
#define CIA_DIAG_0_ISOLATION_INDEX0_BIT_MASK 0xf8000000UL
#define CIA_DIAG_0_PDOA_OFFSET_BIT_OFFSET    (13U)
#define CIA_DIAG_0_PDOA_OFFSET_BIT_LEN       (14U)
#define CIA_DIAG_0_PDOA_OFFSET_BIT_MASK      0x7ffe000UL
#define CIA_DIAG_0_XTALOFFSET_BIT_OFFSET     (0U)
#define CIA_DIAG_0_XTALOFFSET_BIT_LEN        (13U)
#define CIA_DIAG_0_XTALOFFSET_BIT_MASK       0x1fffU

/******************************************************************************
* @brief Bit definitions for register CIA_DIAG_1
**/
#define CIA_DIAG_1_ID                        0xc0024
#define CIA_DIAG_1_LEN                       (4U)
#define CIA_DIAG_1_MASK                      0xFFFFFFFFUL
#define CIA_DIAG_1_ISOLATION_INDEX1_BIT_OFFSET (30U)
#define CIA_DIAG_1_ISOLATION_INDEX1_BIT_LEN  (2U)
#define CIA_DIAG_1_ISOLATION_INDEX1_BIT_MASK 0xc0000000UL
#define CIA_DIAG_1_CARRIERINT_BIT_OFFSET     (8U)
#define CIA_DIAG_1_CARRIERINT_BIT_LEN        (21U)
#define CIA_DIAG_1_CARRIERINT_BIT_MASK       0x1fffff00UL
#define CIA_DIAG_1_RESAMPLERDELAY_BIT_OFFSET (0U)
#define CIA_DIAG_1_RESAMPLERDELAY_BIT_LEN    (8U)
#define CIA_DIAG_1_RESAMPLERDELAY_BIT_MASK   0xffU

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_0
**/
#define IP_DIAG_0_ID                         0xc0028
#define IP_DIAG_0_LEN                        (4U)
#define IP_DIAG_0_MASK                       0xFFFFFFFFUL
#define IP_DIAG_0_PEAKLOC_BIT_OFFSET         (21U)
#define IP_DIAG_0_PEAKLOC_BIT_LEN            (10U)
#define IP_DIAG_0_PEAKLOC_BIT_MASK           0x7fe00000UL
#define IP_DIAG_0_PEAKAMP_BIT_OFFSET         (0U)
#define IP_DIAG_0_PEAKAMP_BIT_LEN            (21U)
#define IP_DIAG_0_PEAKAMP_BIT_MASK           0x1fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_1
**/
#define IP_DIAG_1_ID                         0xc002c
#define IP_DIAG_1_LEN                        (4U)
#define IP_DIAG_1_MASK                       0xFFFFFFFFUL
#define IP_DIAG_1_IPCHANNELAREA_BIT_OFFSET   (0U)
#define IP_DIAG_1_IPCHANNELAREA_BIT_LEN      (17U)
#define IP_DIAG_1_IPCHANNELAREA_BIT_MASK     0x1ffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_2
**/
#define IP_DIAG_2_ID                         0xc0030
#define IP_DIAG_2_LEN                        (4U)
#define IP_DIAG_2_MASK                       0xFFFFFFFFUL
#define IP_DIAG_2_IPF1_BIT_OFFSET            (0U)
#define IP_DIAG_2_IPF1_BIT_LEN               (22U)
#define IP_DIAG_2_IPF1_BIT_MASK              0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_3
**/
#define IP_DIAG_3_ID                         0xc0034
#define IP_DIAG_3_LEN                        (4U)
#define IP_DIAG_3_MASK                       0xFFFFFFFFUL
#define IP_DIAG_3_IPF2_BIT_OFFSET            (0U)
#define IP_DIAG_3_IPF2_BIT_LEN               (22U)
#define IP_DIAG_3_IPF2_BIT_MASK              0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_4
**/
#define IP_DIAG_4_ID                         0xc0038
#define IP_DIAG_4_LEN                        (4U)
#define IP_DIAG_4_MASK                       0xFFFFFFFFUL
#define IP_DIAG_4_IPF3_BIT_OFFSET            (0U)
#define IP_DIAG_4_IPF3_BIT_LEN               (22U)
#define IP_DIAG_4_IPF3_BIT_MASK              0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_5
**/
#define IP_DIAG_5_ID                         0xc003c
#define IP_DIAG_5_LEN                        (4U)
#define IP_DIAG_5_MASK                       0xFFFFFFFFUL
#define IP_DIAG_5_NOISELOC_BIT_OFFSET        (22U)
#define IP_DIAG_5_NOISELOC_BIT_LEN           (10U)
#define IP_DIAG_5_NOISELOC_BIT_MASK          0xffc00000UL
#define IP_DIAG_5_NOISEMAD_BIT_OFFSET        (0U)
#define IP_DIAG_5_NOISEMAD_BIT_LEN           (22U)
#define IP_DIAG_5_NOISEMAD_BIT_MASK          0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_6
**/
#define IP_DIAG_6_ID                         0xc0040
#define IP_DIAG_6_LEN                        (4U)
#define IP_DIAG_6_MASK                       0xFFFFFFFFUL
#define IP_DIAG_6_IPMAXLOC_BIT_OFFSET        (22U)
#define IP_DIAG_6_IPMAXLOC_BIT_LEN           (10U)
#define IP_DIAG_6_IPMAXLOC_BIT_MASK          0xffc00000UL
#define IP_DIAG_6_IPNOISEPEAK_BIT_OFFSET     (0U)
#define IP_DIAG_6_IPNOISEPEAK_BIT_LEN        (22U)
#define IP_DIAG_6_IPNOISEPEAK_BIT_MASK       0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_7
**/
#define IP_DIAG_7_ID                         0xc0044
#define IP_DIAG_7_LEN                        (4U)
#define IP_DIAG_7_MASK                       0xFFFFFFFFUL
#define IP_DIAG_7_IPNOISEMEAN_BIT_OFFSET     (0U)
#define IP_DIAG_7_IPNOISEMEAN_BIT_LEN        (22U)
#define IP_DIAG_7_IPNOISEMEAN_BIT_MASK       0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_8
**/
#define IP_DIAG_8_ID                         0xc0048
#define IP_DIAG_8_LEN                        (4U)
#define IP_DIAG_8_MASK                       0xFFFFFFFFUL
#define IP_DIAG_8_IPFPLOC_BIT_OFFSET         (0U)
#define IP_DIAG_8_IPFPLOC_BIT_LEN            (16U)
#define IP_DIAG_8_IPFPLOC_BIT_MASK           0xffffU

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_9
**/
#define IP_DIAG_9_ID                         0xc004c
#define IP_DIAG_9_LEN                        (4U)
#define IP_DIAG_9_MASK                       0xFFFFFFFFUL
#define IP_DIAG_9_IPFPCONFIDENCELEVEL_BIT_OFFSET (16U)
#define IP_DIAG_9_IPFPCONFIDENCELEVEL_BIT_LEN (4U)
#define IP_DIAG_9_IPFPCONFIDENCELEVEL_BIT_MASK 0xf0000UL
#define IP_DIAG_9_IPEARLYFPLOC_BIT_OFFSET    (0U)
#define IP_DIAG_9_IPEARLYFPLOC_BIT_LEN       (16U)
#define IP_DIAG_9_IPEARLYFPLOC_BIT_MASK      0xffffU

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_10
**/
#define IP_DIAG_10_ID                        0xc0050
#define IP_DIAG_10_LEN                       (4U)
#define IP_DIAG_10_MASK                      0xFFFFFFFFUL
#define IP_DIAG_10_IPCARRIERROTATION_BIT_OFFSET (14U)
#define IP_DIAG_10_IPCARRIERROTATION_BIT_LEN (14U)
#define IP_DIAG_10_IPCARRIERROTATION_BIT_MASK 0xfffc000UL
#define IP_DIAG_10_IPFPANGLE_BIT_OFFSET      (0U)
#define IP_DIAG_10_IPFPANGLE_BIT_LEN         (14U)
#define IP_DIAG_10_IPFPANGLE_BIT_MASK        0x3fffU

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_11
**/
#define IP_DIAG_11_ID                        0xc0054
#define IP_DIAG_11_LEN                       (4U)
#define IP_DIAG_11_MASK                      0xFFFFFFFFUL
#define IP_DIAG_11_IPTHRESHOLD_BIT_OFFSET    (0U)
#define IP_DIAG_11_IPTHRESHOLD_BIT_LEN       (22U)
#define IP_DIAG_11_IPTHRESHOLD_BIT_MASK      0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register IP_DIAG_12
**/
#define IP_DIAG_12_ID                        0xc0058
#define IP_DIAG_12_LEN                       (4U)
#define IP_DIAG_12_MASK                      0xFFFFFFFFUL
#define IP_DIAG_12_IPNACC_BIT_OFFSET         (0U)
#define IP_DIAG_12_IPNACC_BIT_LEN            (12U)
#define IP_DIAG_12_IPNACC_BIT_MASK           0xfffU

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_0
**/
#define CY0_DIAG_0_ID                        0xc005c
#define CY0_DIAG_0_LEN                       (4U)
#define CY0_DIAG_0_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_0_PEAKLOC_BIT_OFFSET        (21U)
#define CY0_DIAG_0_PEAKLOC_BIT_LEN           (9U)
#define CY0_DIAG_0_PEAKLOC_BIT_MASK          0x3fe00000UL
#define CY0_DIAG_0_PEAKAMP_BIT_OFFSET        (0U)
#define CY0_DIAG_0_PEAKAMP_BIT_LEN           (21U)
#define CY0_DIAG_0_PEAKAMP_BIT_MASK          0x1fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_1
**/
#define CY0_DIAG_1_ID                        0xc0060
#define CY0_DIAG_1_LEN                       (4U)
#define CY0_DIAG_1_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_1_CY0CHANNELAREA_BIT_OFFSET (0U)
#define CY0_DIAG_1_CY0CHANNELAREA_BIT_LEN    (16U)
#define CY0_DIAG_1_CY0CHANNELAREA_BIT_MASK   0xffffU

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_2
**/
#define CY0_DIAG_2_ID                        0xc0064
#define CY0_DIAG_2_LEN                       (4U)
#define CY0_DIAG_2_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_2_CY0F1_BIT_OFFSET          (0U)
#define CY0_DIAG_2_CY0F1_BIT_LEN             (22U)
#define CY0_DIAG_2_CY0F1_BIT_MASK            0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_3
**/
#define CY0_DIAG_3_ID                        0xc0068
#define CY0_DIAG_3_LEN                       (4U)
#define CY0_DIAG_3_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_3_CY0F2_BIT_OFFSET          (0U)
#define CY0_DIAG_3_CY0F2_BIT_LEN             (22U)
#define CY0_DIAG_3_CY0F2_BIT_MASK            0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_4
**/
#define CY0_DIAG_4_ID                        0xd0000
#define CY0_DIAG_4_LEN                       (4U)
#define CY0_DIAG_4_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_4_CY0F3_BIT_OFFSET          (0U)
#define CY0_DIAG_4_CY0F3_BIT_LEN             (22U)
#define CY0_DIAG_4_CY0F3_BIT_MASK            0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_5
**/
#define CY0_DIAG_5_ID                        0xd0004
#define CY0_DIAG_5_LEN                       (4U)
#define CY0_DIAG_5_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_5_CY0NOISEMAD_BIT_OFFSET    (0U)
#define CY0_DIAG_5_CY0NOISEMAD_BIT_LEN       (22U)
#define CY0_DIAG_5_CY0NOISEMAD_BIT_MASK      0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_6
**/
#define CY0_DIAG_6_ID                        0xd0008
#define CY0_DIAG_6_LEN                       (4U)
#define CY0_DIAG_6_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_6_CY0MAXLOC_BIT_OFFSET      (22U)
#define CY0_DIAG_6_CY0MAXLOC_BIT_LEN         (9U)
#define CY0_DIAG_6_CY0MAXLOC_BIT_MASK        0x7fc00000UL
#define CY0_DIAG_6_CY0NOISEPEAK_BIT_OFFSET   (0U)
#define CY0_DIAG_6_CY0NOISEPEAK_BIT_LEN      (22U)
#define CY0_DIAG_6_CY0NOISEPEAK_BIT_MASK     0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_7
**/
#define CY0_DIAG_7_ID                        0xd000c
#define CY0_DIAG_7_LEN                       (4U)
#define CY0_DIAG_7_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_7_CY0NOISEMEAN_BIT_OFFSET   (0U)
#define CY0_DIAG_7_CY0NOISEMEAN_BIT_LEN      (22U)
#define CY0_DIAG_7_CY0NOISEMEAN_BIT_MASK     0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_8
**/
#define CY0_DIAG_8_ID                        0xd0010
#define CY0_DIAG_8_LEN                       (4U)
#define CY0_DIAG_8_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_8_CY0FPLOC_BIT_OFFSET       (0U)
#define CY0_DIAG_8_CY0FPLOC_BIT_LEN          (15U)
#define CY0_DIAG_8_CY0FPLOC_BIT_MASK         0x7fffU

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_9
**/
#define CY0_DIAG_9_ID                        0xd0014
#define CY0_DIAG_9_LEN                       (4U)
#define CY0_DIAG_9_MASK                      0xFFFFFFFFUL
#define CY0_DIAG_9_CY0FPCONFIDENCELEVEL_BIT_OFFSET (16U)
#define CY0_DIAG_9_CY0FPCONFIDENCELEVEL_BIT_LEN (4U)
#define CY0_DIAG_9_CY0FPCONFIDENCELEVEL_BIT_MASK 0xf0000UL
#define CY0_DIAG_9_CY0EARLYFPLOC_BIT_OFFSET  (0U)
#define CY0_DIAG_9_CY0EARLYFPLOC_BIT_LEN     (15U)
#define CY0_DIAG_9_CY0EARLYFPLOC_BIT_MASK    0x7fffU

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_10
**/
#define CY0_DIAG_10_ID                       0xd0018
#define CY0_DIAG_10_LEN                      (4U)
#define CY0_DIAG_10_MASK                     0xFFFFFFFFUL
#define CY0_DIAG_10_CY0CARRIERROTATION_BIT_OFFSET (14U)
#define CY0_DIAG_10_CY0CARRIERROTATION_BIT_LEN (14U)
#define CY0_DIAG_10_CY0CARRIERROTATION_BIT_MASK 0xfffc000UL
#define CY0_DIAG_10_CY0FPANGLE_BIT_OFFSET    (0U)
#define CY0_DIAG_10_CY0FPANGLE_BIT_LEN       (14U)
#define CY0_DIAG_10_CY0FPANGLE_BIT_MASK      0x3fffU

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_11
**/
#define CY0_DIAG_11_ID                       0xd001c
#define CY0_DIAG_11_LEN                      (4U)
#define CY0_DIAG_11_MASK                     0xFFFFFFFFUL
#define CY0_DIAG_11_CY0THRESHOLD_BIT_OFFSET  (0U)
#define CY0_DIAG_11_CY0THRESHOLD_BIT_LEN     (22U)
#define CY0_DIAG_11_CY0THRESHOLD_BIT_MASK    0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY0_DIAG_12
**/
#define CY0_DIAG_12_ID                       0xd0020
#define CY0_DIAG_12_LEN                      (4U)
#define CY0_DIAG_12_MASK                     0xFFFFFFFFUL
#define CY0_DIAG_12_SFDPHASECOUNT_BIT_OFFSET (0U)
#define CY0_DIAG_12_SFDPHASECOUNT_BIT_LEN    (1U)
#define CY0_DIAG_12_SFDPHASECOUNT_BIT_MASK   0x1U
#define CY0_DIAG_12_CYNACC_BIT_OFFSET        (0U)
#define CY0_DIAG_12_CYNACC_BIT_LEN           (12U)
#define CY0_DIAG_12_CYNACC_BIT_MASK          0xfffU

#define CY0_DIAG_13_ID                       0xd0024
#define CY0_DIAG_14_ID                       0xd0028
#define CY0_DIAG_15_ID                       0xd002C
#define CY0_DIAG_16_ID                       0xd0030
#define CY0_DIAG_17_ID                       0xd0034

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_0
**/
#define CY1_DIAG_0_ID                        0xd0038
#define CY1_DIAG_0_LEN                       (4U)
#define CY1_DIAG_0_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_0_PEAKLOC_BIT_OFFSET        (21U)
#define CY1_DIAG_0_PEAKLOC_BIT_LEN           (9U)
#define CY1_DIAG_0_PEAKLOC_BIT_MASK          0x3fe00000UL
#define CY1_DIAG_0_PEAKAMP_BIT_OFFSET        (0U)
#define CY1_DIAG_0_PEAKAMP_BIT_LEN           (21U)
#define CY1_DIAG_0_PEAKAMP_BIT_MASK          0x1fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_1
**/
#define CY1_DIAG_1_ID                        0xd003c
#define CY1_DIAG_1_LEN                       (4U)
#define CY1_DIAG_1_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_1_CY1CHANNELAREA_BIT_OFFSET (0U)
#define CY1_DIAG_1_CY1CHANNELAREA_BIT_LEN    (16U)
#define CY1_DIAG_1_CY1CHANNELAREA_BIT_MASK   0xffffU

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_2
**/
#define CY1_DIAG_2_ID                        0xd0040
#define CY1_DIAG_2_LEN                       (4U)
#define CY1_DIAG_2_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_2_CY1F1_BIT_OFFSET          (0U)
#define CY1_DIAG_2_CY1F1_BIT_LEN             (22U)
#define CY1_DIAG_2_CY1F1_BIT_MASK            0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_3
**/
#define CY1_DIAG_3_ID                        0xd0044
#define CY1_DIAG_3_LEN                       (4U)
#define CY1_DIAG_3_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_3_CY1F2_BIT_OFFSET          (0U)
#define CY1_DIAG_3_CY1F2_BIT_LEN             (22U)
#define CY1_DIAG_3_CY1F2_BIT_MASK            0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_4
**/
#define CY1_DIAG_4_ID                        0xd0048
#define CY1_DIAG_4_LEN                       (4U)
#define CY1_DIAG_4_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_4_CY1F3_BIT_OFFSET          (0U)
#define CY1_DIAG_4_CY1F3_BIT_LEN             (22U)
#define CY1_DIAG_4_CY1F3_BIT_MASK            0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_5
**/
#define CY1_DIAG_5_ID                        0xd004c
#define CY1_DIAG_5_LEN                       (4U)
#define CY1_DIAG_5_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_5_CY1NOISEMAD_BIT_OFFSET    (0U)
#define CY1_DIAG_5_CY1NOISEMAD_BIT_LEN       (22U)
#define CY1_DIAG_5_CY1NOISEMAD_BIT_MASK      0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_6
**/
#define CY1_DIAG_6_ID                        0xd0050
#define CY1_DIAG_6_LEN                       (4U)
#define CY1_DIAG_6_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_6_CY1MAXLOC_BIT_OFFSET      (22U)
#define CY1_DIAG_6_CY1MAXLOC_BIT_LEN         (9U)
#define CY1_DIAG_6_CY1MAXLOC_BIT_MASK        0x7fc00000UL
#define CY1_DIAG_6_CY1NOISEPEAK_BIT_OFFSET   (0U)
#define CY1_DIAG_6_CY1NOISEPEAK_BIT_LEN      (22U)
#define CY1_DIAG_6_CY1NOISEPEAK_BIT_MASK     0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_7
**/
#define CY1_DIAG_7_ID                        0xd0054
#define CY1_DIAG_7_LEN                       (4U)
#define CY1_DIAG_7_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_7_CY1NOISEMEAN_BIT_OFFSET   (0U)
#define CY1_DIAG_7_CY1NOISEMEAN_BIT_LEN      (22U)
#define CY1_DIAG_7_CY1NOISEMEAN_BIT_MASK     0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_8
**/
#define CY1_DIAG_8_ID                        0xd0058
#define CY1_DIAG_8_LEN                       (4U)
#define CY1_DIAG_8_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_8_CY1FPLOC_BIT_OFFSET       (0U)
#define CY1_DIAG_8_CY1FPLOC_BIT_LEN          (15U)
#define CY1_DIAG_8_CY1FPLOC_BIT_MASK         0x7fffU

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_9
**/
#define CY1_DIAG_9_ID                        0xd005c
#define CY1_DIAG_9_LEN                       (4U)
#define CY1_DIAG_9_MASK                      0xFFFFFFFFUL
#define CY1_DIAG_9_CY1FPCONFIDENCELEVEL_BIT_OFFSET (16U)
#define CY1_DIAG_9_CY1FPCONFIDENCELEVEL_BIT_LEN (4U)
#define CY1_DIAG_9_CY1FPCONFIDENCELEVEL_BIT_MASK 0xf0000UL
#define CY1_DIAG_9_CY1EARLYFPLOC_BIT_OFFSET  (0U)
#define CY1_DIAG_9_CY1EARLYFPLOC_BIT_LEN     (15U)
#define CY1_DIAG_9_CY1EARLYFPLOC_BIT_MASK    0x7fffU

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_10
**/
#define CY1_DIAG_10_ID                       0xd0060
#define CY1_DIAG_10_LEN                      (4U)
#define CY1_DIAG_10_MASK                     0xFFFFFFFFUL
#define CY1_DIAG_10_CY1FPANGLE_BIT_OFFSET    (0U)
#define CY1_DIAG_10_CY1FPANGLE_BIT_LEN       (14U)
#define CY1_DIAG_10_CY1FPANGLE_BIT_MASK      0x3fffU

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_11
**/
#define CY1_DIAG_11_ID                       0xd0064
#define CY1_DIAG_11_LEN                      (4U)
#define CY1_DIAG_11_MASK                     0xFFFFFFFFUL
#define CY1_DIAG_11_CY1THRESHOLD_BIT_OFFSET  (0U)
#define CY1_DIAG_11_CY1THRESHOLD_BIT_LEN     (22U)
#define CY1_DIAG_11_CY1THRESHOLD_BIT_MASK    0x3fffffUL

/******************************************************************************
* @brief Bit definitions for register CY1_DIAG_12
**/
#define CY1_DIAG_12_ID                       0xd0068
#define CY1_DIAG_12_LEN                      (4U)
#define CY1_DIAG_12_MASK                     0xFFFFFFFFUL
#define CY1_DIAG_12_CY1NACC_BIT_OFFSET       (0U)
#define CY1_DIAG_12_CY1NACC_BIT_LEN          (12U)
#define CY1_DIAG_12_CY1NACC_BIT_MASK         0xfffU

/******************************************************************************
* @brief Bit definitions for register RX_ANTENNA_DELAY
**/
#define RX_ANTENNA_DELAY_ID                  0xe0000
#define RX_ANTENNA_DELAY_LEN                 (4U)
#define RX_ANTENNA_DELAY_MASK                0xFFFFFFFFUL
#define RX_ANTENNA_DELAY_MINDIAG_BIT_OFFSET  (20U)
#define RX_ANTENNA_DELAY_MINDIAG_BIT_LEN     (1U)
#define RX_ANTENNA_DELAY_MINDIAG_BIT_MASK    0x100000UL
#define RX_ANTENNA_DELAY_REWINDOWLIMIT_BIT_OFFSET (0U)
#define RX_ANTENNA_DELAY_REWINDOWLIMIT_BIT_LEN (1U)
#define RX_ANTENNA_DELAY_REWINDOWLIMIT_BIT_MASK 0x1U
#define RX_ANTENNA_DELAY_RXANTENNADELAY_BIT_OFFSET (0U)
#define RX_ANTENNA_DELAY_RXANTENNADELAY_BIT_LEN (16U)
#define RX_ANTENNA_DELAY_RXANTENNADELAY_BIT_MASK 0xffffU

/******************************************************************************
* @brief Bit definitions for register FP_CONFIDENCE_LIMIT
**/
#define FP_CONFIDENCE_LIMIT_ID               0xe0004
#define FP_CONFIDENCE_LIMIT_LEN              (4U)
#define FP_CONFIDENCE_LIMIT_MASK             0xFFFFFFFFUL
#define FP_CONFIDENCE_LIMIT_TEMPERATURE_DIFFERENCE_BIT_OFFSET (22U)
#define FP_CONFIDENCE_LIMIT_TEMPERATURE_DIFFERENCE_BIT_LEN (10U)
#define FP_CONFIDENCE_LIMIT_TEMPERATURE_DIFFERENCE_BIT_MASK 0xffc00000UL
#define FP_CONFIDENCE_LIMIT_ACCUMULATOR_ANALYSIS_EN_BIT_OFFSET (21U)
#define FP_CONFIDENCE_LIMIT_ACCUMULATOR_ANALYSIS_EN_BIT_LEN (1U)
#define FP_CONFIDENCE_LIMIT_ACCUMULATOR_ANALYSIS_EN_BIT_MASK 0x200000UL
#define FP_CONFIDENCE_LIMIT_TEMP_DELAY_COMPENSATION_EN_BIT_OFFSET (20U)
#define FP_CONFIDENCE_LIMIT_TEMP_DELAY_COMPENSATION_EN_BIT_LEN (1U)
#define FP_CONFIDENCE_LIMIT_TEMP_DELAY_COMPENSATION_EN_BIT_MASK 0x100000UL
#define FP_CONFIDENCE_LIMIT_ARX_DELAY_COMPENSATION_EN_BIT_OFFSET (19U)
#define FP_CONFIDENCE_LIMIT_ARX_DELAY_COMPENSATION_EN_BIT_LEN (1U)
#define FP_CONFIDENCE_LIMIT_ARX_DELAY_COMPENSATION_EN_BIT_MASK 0x80000UL
#define FP_CONFIDENCE_LIMIT_CAL_TEMPERATURE_BIT_OFFSET (11U)
#define FP_CONFIDENCE_LIMIT_CAL_TEMPERATURE_BIT_LEN (8U)
#define FP_CONFIDENCE_LIMIT_CAL_TEMPERATURE_BIT_MASK 0x7f800UL
#define FP_CONFIDENCE_LIMIT_FP_AGREED_THRESHOLD_BIT_OFFSET (8U)
#define FP_CONFIDENCE_LIMIT_FP_AGREED_THRESHOLD_BIT_LEN (3U)
#define FP_CONFIDENCE_LIMIT_FP_AGREED_THRESHOLD_BIT_MASK 0x700U
#define FP_CONFIDENCE_LIMIT_ARX_DELAY_TABLE_ENTRY_BIT_OFFSET (5U)
#define FP_CONFIDENCE_LIMIT_ARX_DELAY_TABLE_ENTRY_BIT_LEN (3U)
#define FP_CONFIDENCE_LIMIT_ARX_DELAY_TABLE_ENTRY_BIT_MASK 0xe0U
#define FP_CONFIDENCE_LIMIT_FP_CONFIDENCE_LIM_BIT_OFFSET (0U)
#define FP_CONFIDENCE_LIMIT_FP_CONFIDENCE_LIM_BIT_LEN (5U)
#define FP_CONFIDENCE_LIMIT_FP_CONFIDENCE_LIM_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register FP_CONFIDENCE_THRESHOLDS
**/
#define FP_CONFIDENCE_THRESHOLDS_ID          0xe0008
#define FP_CONFIDENCE_THRESHOLDS_LEN         (4U)
#define FP_CONFIDENCE_THRESHOLDS_MASK        0xFFFFFFFFUL
#define FP_CONFIDENCE_THRESHOLDS_IPATOV_FP_CONF_THRES_BIT_OFFSET (8U)
#define FP_CONFIDENCE_THRESHOLDS_IPATOV_FP_CONF_THRES_BIT_LEN (5U)
#define FP_CONFIDENCE_THRESHOLDS_IPATOV_FP_CONF_THRES_BIT_MASK 0x1f00U
#define FP_CONFIDENCE_THRESHOLDS_CYPHER_FP_CONF_THRES_BIT_OFFSET (0U)
#define FP_CONFIDENCE_THRESHOLDS_CYPHER_FP_CONF_THRES_BIT_LEN (5U)
#define FP_CONFIDENCE_THRESHOLDS_CYPHER_FP_CONF_THRES_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register IP_CONFIG_LO
**/
#define IP_CONFIG_LO_ID                      0xe000c
#define IP_CONFIG_LO_LEN                     (2U)
#define IP_CONFIG_LO_MASK                    0xFFFFU
#define IP_CONFIG_LO_MANUALIGNOREWINDOW_BIT_OFFSET (12U)
#define IP_CONFIG_LO_MANUALIGNOREWINDOW_BIT_LEN (1U)
#define IP_CONFIG_LO_MANUALIGNOREWINDOW_BIT_MASK 0x1000U
#define IP_CONFIG_LO_REWINDOW_BIT_OFFSET     (11U)
#define IP_CONFIG_LO_REWINDOW_BIT_LEN        (1U)
#define IP_CONFIG_LO_REWINDOW_BIT_MASK       0x800U
#define IP_CONFIG_LO_EARLYSEARCH_BIT_OFFSET  (10U)
#define IP_CONFIG_LO_EARLYSEARCH_BIT_LEN     (1U)
#define IP_CONFIG_LO_EARLYSEARCH_BIT_MASK    0x400U
#define IP_CONFIG_LO_DCOFFSETREMOVAL_BIT_OFFSET (9U)
#define IP_CONFIG_LO_DCOFFSETREMOVAL_BIT_LEN (1U)
#define IP_CONFIG_LO_DCOFFSETREMOVAL_BIT_MASK 0x200U
#define IP_CONFIG_LO_IPNOISELENGTH_BIT_OFFSET (7U)
#define IP_CONFIG_LO_IPNOISELENGTH_BIT_LEN   (2U)
#define IP_CONFIG_LO_IPNOISELENGTH_BIT_MASK  0x180U
#define IP_CONFIG_LO_PEAKMULTIPLIER_BIT_OFFSET (5U)
#define IP_CONFIG_LO_PEAKMULTIPLIER_BIT_LEN  (2U)
#define IP_CONFIG_LO_PEAKMULTIPLIER_BIT_MASK 0x60U
#define IP_CONFIG_LO_NOISETHRESHOLDMULTIPLIER_BIT_OFFSET (0U)
#define IP_CONFIG_LO_NOISETHRESHOLDMULTIPLIER_BIT_LEN (5U)
#define IP_CONFIG_LO_NOISETHRESHOLDMULTIPLIER_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register IP_CONFIG_HI
**/
#define IP_CONFIG_HI_ID                      0xe000e              /*  { aliased = true}  */
#define IP_CONFIG_HI_LEN                     (4U)
#define IP_CONFIG_HI_MASK                    0xFFFFFFFFUL
#define IP_CONFIG_HI_IGNOREWINDOW_BIT_OFFSET (22U)
#define IP_CONFIG_HI_IGNOREWINDOW_BIT_LEN    (9U)
#define IP_CONFIG_HI_IGNOREWINDOW_BIT_MASK   0x7fc00000UL
#define IP_CONFIG_HI_OVERRIDEREPLICACOEFFICIENT_BIT_OFFSET (6U)
#define IP_CONFIG_HI_OVERRIDEREPLICACOEFFICIENT_BIT_LEN (16U)
#define IP_CONFIG_HI_OVERRIDEREPLICACOEFFICIENT_BIT_MASK 0x3fffc0UL
#define IP_CONFIG_HI_USEOVERRIDEREPLICACOEFFICIENT_BIT_OFFSET (5U)
#define IP_CONFIG_HI_USEOVERRIDEREPLICACOEFFICIENT_BIT_LEN (1U)
#define IP_CONFIG_HI_USEOVERRIDEREPLICACOEFFICIENT_BIT_MASK 0x20U
#define IP_CONFIG_HI_REPLICATHRESHOLDMULTIPLIER_BIT_OFFSET (0U)
#define IP_CONFIG_HI_REPLICATHRESHOLDMULTIPLIER_BIT_LEN (5U)
#define IP_CONFIG_HI_REPLICATHRESHOLDMULTIPLIER_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register CY_CONFIG_LO
**/
#define CY_CONFIG_LO_ID                      0xe0012              /*  { aliased = true}  */
#define CY_CONFIG_LO_LEN                     (4U)
#define CY_CONFIG_LO_MASK                    0xFFFFFFFFUL
#define CY_CONFIG_LO_MANUALLOWERBOUND_BIT_OFFSET (16U)
#define CY_CONFIG_LO_MANUALLOWERBOUND_BIT_LEN (7U)
#define CY_CONFIG_LO_MANUALLOWERBOUND_BIT_MASK 0x7f0000UL
#define CY_CONFIG_LO_CYIGNOREWINDOW_BIT_OFFSET (11U)
#define CY_CONFIG_LO_CYIGNOREWINDOW_BIT_LEN  (4U)
#define CY_CONFIG_LO_CYIGNOREWINDOW_BIT_MASK 0x7800U
#define CY_CONFIG_LO_REWINDOW_BIT_OFFSET     (10U)
#define CY_CONFIG_LO_REWINDOW_BIT_LEN        (1U)
#define CY_CONFIG_LO_REWINDOW_BIT_MASK       0x400U
#define CY_CONFIG_LO_EARLYSEARCH_BIT_OFFSET  (9U)
#define CY_CONFIG_LO_EARLYSEARCH_BIT_LEN     (1U)
#define CY_CONFIG_LO_EARLYSEARCH_BIT_MASK    0x200U
#define CY_CONFIG_LO_DCOFFSETREMOVAL_BIT_OFFSET (8U)
#define CY_CONFIG_LO_DCOFFSETREMOVAL_BIT_LEN (1U)
#define CY_CONFIG_LO_DCOFFSETREMOVAL_BIT_MASK 0x100U
#define CY_CONFIG_LO_CYNOISELENGTH_BIT_OFFSET (7U)
#define CY_CONFIG_LO_CYNOISELENGTH_BIT_LEN   (1U)
#define CY_CONFIG_LO_CYNOISELENGTH_BIT_MASK  0x80U
#define CY_CONFIG_LO_PEAKMULTIPLIER_BIT_OFFSET (5U)
#define CY_CONFIG_LO_PEAKMULTIPLIER_BIT_LEN  (2U)
#define CY_CONFIG_LO_PEAKMULTIPLIER_BIT_MASK 0x60U
#define CY_CONFIG_LO_NOISETHRESHOLDMULTIPLIER_BIT_OFFSET (0U)
#define CY_CONFIG_LO_NOISETHRESHOLDMULTIPLIER_BIT_LEN (5U)
#define CY_CONFIG_LO_NOISETHRESHOLDMULTIPLIER_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register CY_CONFIG_HI
**/
#define CY_CONFIG_HI_ID                      0xe0016              /*  { aliased = true}  */
#define CY_CONFIG_HI_LEN                     (4U)
#define CY_CONFIG_HI_MASK                    0xFFFFFFFFUL
#define CY_CONFIG_HI_PEAK_GROWTH_EN_BIT_OFFSET (31U)
#define CY_CONFIG_HI_PEAK_GROWTH_EN_BIT_LEN  (1U)
#define CY_CONFIG_HI_PEAK_GROWTH_EN_BIT_MASK 0x80000000UL
#define CY_CONFIG_HI_ADC_COUNT_EN_BIT_OFFSET (30U)
#define CY_CONFIG_HI_ADC_COUNT_EN_BIT_LEN    (1U)
#define CY_CONFIG_HI_ADC_COUNT_EN_BIT_MASK   0x40000000UL
#define CY_CONFIG_HI_SFD_COUNT_EN_BIT_OFFSET (29U)
#define CY_CONFIG_HI_SFD_COUNT_EN_BIT_LEN    (1U)
#define CY_CONFIG_HI_SFD_COUNT_EN_BIT_MASK   0x20000000UL
#define CY_CONFIG_HI_FP_AGREED_EN_BIT_OFFSET (28U)
#define CY_CONFIG_HI_FP_AGREED_EN_BIT_LEN    (1U)
#define CY_CONFIG_HI_FP_AGREED_EN_BIT_MASK   0x10000000UL
#define CY_CONFIG_HI_LOGISTIC_REGRESSION_EN_BIT_OFFSET (27U)
#define CY_CONFIG_HI_LOGISTIC_REGRESSION_EN_BIT_LEN (1U)
#define CY_CONFIG_HI_LOGISTIC_REGRESSION_EN_BIT_MASK 0x8000000UL
#define CY_CONFIG_HI_PEAKGROWTHHIGHTHRESHOLD_BIT_OFFSET (17U)
#define CY_CONFIG_HI_PEAKGROWTHHIGHTHRESHOLD_BIT_LEN (5U)
#define CY_CONFIG_HI_PEAKGROWTHHIGHTHRESHOLD_BIT_MASK 0x3e0000UL
#define CY_CONFIG_HI_PEAKGROWTHLOWTHRESHOLD_BIT_OFFSET (12U)
#define CY_CONFIG_HI_PEAKGROWTHLOWTHRESHOLD_BIT_LEN (5U)
#define CY_CONFIG_HI_PEAKGROWTHLOWTHRESHOLD_BIT_MASK 0x1f000UL
#define CY_CONFIG_HI_ADCCOUNTHIGHTHRESHOLD_BIT_OFFSET (8U)
#define CY_CONFIG_HI_ADCCOUNTHIGHTHRESHOLD_BIT_LEN (4U)
#define CY_CONFIG_HI_ADCCOUNTHIGHTHRESHOLD_BIT_MASK 0xf00U
#define CY_CONFIG_HI_ADCCOUNTLOWTHRESHOLD_BIT_OFFSET (4U)
#define CY_CONFIG_HI_ADCCOUNTLOWTHRESHOLD_BIT_LEN (4U)
#define CY_CONFIG_HI_ADCCOUNTLOWTHRESHOLD_BIT_MASK 0xf0U
#define CY_CONFIG_HI_SFDCOUNTTHRESHOLD_BIT_OFFSET (0U)
#define CY_CONFIG_HI_SFDCOUNTTHRESHOLD_BIT_LEN (4U)
#define CY_CONFIG_HI_SFDCOUNTTHRESHOLD_BIT_MASK 0xfU

/******************************************************************************
* @brief Bit definitions for register CIA_COEFFICIENT_ADJUST
**/
#define CIA_COEFFICIENT_ADJUST_ID            0xe001a              /* {aliased=true} */
#define CIA_COEFFICIENT_ADJUST_LEN           (4U)
#define CIA_COEFFICIENT_ADJUST_MASK          0xFFFFFFFFUL
#define CIA_COEFFICIENT_ADJUST_PDOA_ISOLATION_SCALING_BIT_OFFSET (24U)
#define CIA_COEFFICIENT_ADJUST_PDOA_ISOLATION_SCALING_BIT_LEN (4U)
#define CIA_COEFFICIENT_ADJUST_PDOA_ISOLATION_SCALING_BIT_MASK 0xf000000UL
#define CIA_COEFFICIENT_ADJUST_TEMP_CORRECTION_COEFF_BIT_OFFSET (16U)
#define CIA_COEFFICIENT_ADJUST_TEMP_CORRECTION_COEFF_BIT_LEN (7U)
#define CIA_COEFFICIENT_ADJUST_TEMP_CORRECTION_COEFF_BIT_MASK 0x7f0000UL
#define CIA_COEFFICIENT_ADJUST_PDOA_ADJUST_OFFSET_BIT_OFFSET (0U)
#define CIA_COEFFICIENT_ADJUST_PDOA_ADJUST_OFFSET_BIT_LEN (14U)
#define CIA_COEFFICIENT_ADJUST_PDOA_ADJUST_OFFSET_BIT_MASK 0x3fffU

/******************************************************************************
* @brief Bit definitions for register PGF_DELAY_COMP_LO
**/
#define PGF_DELAY_COMP_LO_ID                 0xe001e              /* {aliased=true} */
#define PGF_DELAY_COMP_LO_LEN                (4U)
#define PGF_DELAY_COMP_LO_MASK               0xFFFFFFFFUL
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET3_BIT_OFFSET (24U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET3_BIT_LEN (6U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET3_BIT_MASK 0x3f000000UL
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET2_BIT_OFFSET (16U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET2_BIT_LEN (6U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET2_BIT_MASK 0x3f0000UL
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET1_BIT_OFFSET (8U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET1_BIT_LEN (6U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET1_BIT_MASK 0x3f00U
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET0_BIT_OFFSET (0U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET0_BIT_LEN (6U)
#define PGF_DELAY_COMP_LO_PGF_DELAY_OFFSET0_BIT_MASK 0x3fU

/******************************************************************************
* @brief Bit definitions for register PGF_DELAY_COMP_HI
**/
#define PGF_DELAY_COMP_HI_ID                 0xe0022              /* {aliased=true} */
#define PGF_DELAY_COMP_HI_LEN                (4U)
#define PGF_DELAY_COMP_HI_MASK               0xFFFFFFFFUL
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET6_BIT_OFFSET (16U)
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET6_BIT_LEN (6U)
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET6_BIT_MASK 0x3f0000UL
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET5_BIT_OFFSET (8U)
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET5_BIT_LEN (6U)
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET5_BIT_MASK 0x3f00U
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET4_BIT_OFFSET (0U)
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET4_BIT_LEN (6U)
#define PGF_DELAY_COMP_HI_PGF_DELAY_OFFSET4_BIT_MASK 0x3fU

/******************************************************************************
* @brief Bit definitions for register EVENT_CTRL
**/
#define EVENT_CTRL_ID                        0xf0000
#define EVENT_CTRL_LEN                       (4U)
#define EVENT_CTRL_MASK                      0xFFFFFFFFUL
#define EVENT_CTRL_EVENT_COUNT_CLR_BIT_OFFSET (1U)
#define EVENT_CTRL_EVENT_COUNT_CLR_BIT_LEN   (1U)
#define EVENT_CTRL_EVENT_COUNT_CLR_BIT_MASK  0x2U
#define EVENT_CTRL_EVENT_COUNT_EN_BIT_OFFSET (0U)
#define EVENT_CTRL_EVENT_COUNT_EN_BIT_LEN    (1U)
#define EVENT_CTRL_EVENT_COUNT_EN_BIT_MASK   0x1U

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT0
**/
#define EVENT_COUNT0_ID                      0xf0004
#define EVENT_COUNT0_LEN                     (4U)
#define EVENT_COUNT0_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT0_COUNT_RXFSL_BIT_OFFSET  (16U)
#define EVENT_COUNT0_COUNT_RXFSL_BIT_LEN     (12U)
#define EVENT_COUNT0_COUNT_RXFSL_BIT_MASK    0xfff0000UL
#define EVENT_COUNT0_COUNT_RXPHE_BIT_OFFSET  (0U)
#define EVENT_COUNT0_COUNT_RXPHE_BIT_LEN     (12U)
#define EVENT_COUNT0_COUNT_RXPHE_BIT_MASK    0xfffU

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT1
**/
#define EVENT_COUNT1_ID                      0xf0008
#define EVENT_COUNT1_LEN                     (4U)
#define EVENT_COUNT1_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT1_COUNT_RXFCE_BIT_OFFSET  (16U)
#define EVENT_COUNT1_COUNT_RXFCE_BIT_LEN     (12U)
#define EVENT_COUNT1_COUNT_RXFCE_BIT_MASK    0xfff0000UL
#define EVENT_COUNT1_COUNT_RXFCG_BIT_OFFSET  (0U)
#define EVENT_COUNT1_COUNT_RXFCG_BIT_LEN     (12U)
#define EVENT_COUNT1_COUNT_RXFCG_BIT_MASK    0xfffU

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT2
**/
#define EVENT_COUNT2_ID                      0xf000c
#define EVENT_COUNT2_LEN                     (4U)
#define EVENT_COUNT2_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT2_COUNT_RXOVRR_BIT_OFFSET (16U)
#define EVENT_COUNT2_COUNT_RXOVRR_BIT_LEN    (8U)
#define EVENT_COUNT2_COUNT_RXOVRR_BIT_MASK   0xff0000UL
#define EVENT_COUNT2_COUNT_ARFE_BIT_OFFSET   (0U)
#define EVENT_COUNT2_COUNT_ARFE_BIT_LEN      (8U)
#define EVENT_COUNT2_COUNT_ARFE_BIT_MASK     0xffU

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT3
**/
#define EVENT_COUNT3_ID                      0xf0010
#define EVENT_COUNT3_LEN                     (4U)
#define EVENT_COUNT3_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT3_COUNT_RXPTO_BIT_OFFSET  (16U)
#define EVENT_COUNT3_COUNT_RXPTO_BIT_LEN     (12U)
#define EVENT_COUNT3_COUNT_RXPTO_BIT_MASK    0xfff0000UL
#define EVENT_COUNT3_COUNT_RXSTO_BIT_OFFSET  (0U)
#define EVENT_COUNT3_COUNT_RXSTO_BIT_LEN     (12U)
#define EVENT_COUNT3_COUNT_RXSTO_BIT_MASK    0xfffU

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT4
**/
#define EVENT_COUNT4_ID                      0xf0014
#define EVENT_COUNT4_LEN                     (4U)
#define EVENT_COUNT4_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT4_COUNT_TXFRS_BIT_OFFSET  (16U)
#define EVENT_COUNT4_COUNT_TXFRS_BIT_LEN     (12U)
#define EVENT_COUNT4_COUNT_TXFRS_BIT_MASK    0xfff0000UL
#define EVENT_COUNT4_COUNT_FWTO_BIT_OFFSET   (0U)
#define EVENT_COUNT4_COUNT_FWTO_BIT_LEN      (8U)
#define EVENT_COUNT4_COUNT_FWTO_BIT_MASK     0xffU

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT5
**/
#define EVENT_COUNT5_ID                      0xf0018
#define EVENT_COUNT5_LEN                     (4U)
#define EVENT_COUNT5_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT5_COUNT_SPICRC_BIT_OFFSET (16U)
#define EVENT_COUNT5_COUNT_SPICRC_BIT_LEN    (8U)
#define EVENT_COUNT5_COUNT_SPICRC_BIT_MASK   0xff0000UL
#define EVENT_COUNT5_COUNT_HPWARN_BIT_OFFSET (0U)
#define EVENT_COUNT5_COUNT_HPWARN_BIT_LEN    (8U)
#define EVENT_COUNT5_COUNT_HPWARN_BIT_MASK   0xffU

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT6
**/
#define EVENT_COUNT6_ID                      0xf001c
#define EVENT_COUNT6_LEN                     (4U)
#define EVENT_COUNT6_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT6_COUNT_RXPREJ_BIT_OFFSET (0U)
#define EVENT_COUNT6_COUNT_RXPREJ_BIT_LEN    (12U)
#define EVENT_COUNT6_COUNT_RXPREJ_BIT_MASK   0xfffU

/******************************************************************************
* @brief Bit definitions for register ADC_MEM_PTR
**/
#define ADC_MEM_PTR_ID                       0xf0020
#define ADC_MEM_PTR_LEN                      (4U)
#define ADC_MEM_PTR_MASK                     0xFFFFFFFFUL
#define ADC_MEM_PTR_ADC_MEM_PTR_BIT_OFFSET   (0U)
#define ADC_MEM_PTR_ADC_MEM_PTR_BIT_LEN      (12U)
#define ADC_MEM_PTR_ADC_MEM_PTR_BIT_MASK     0xfffU

/******************************************************************************
* @brief Bit definitions for register TEST_CTRL0
**/
#define TEST_CTRL0_ID                        0xf0024
#define TEST_CTRL0_LEN                       (4U)
#define TEST_CTRL0_MASK                      0xFFFFFFFFUL
#define TEST_CTRL0_CIA_MAN_RUN_BIT_OFFSET    (26U)
#define TEST_CTRL0_CIA_MAN_RUN_BIT_LEN       (1U)
#define TEST_CTRL0_CIA_MAN_RUN_BIT_MASK      0x4000000UL
#define TEST_CTRL0_CIA_ABORT_BIT_OFFSET      (25U)
#define TEST_CTRL0_CIA_ABORT_BIT_LEN         (1U)
#define TEST_CTRL0_CIA_ABORT_BIT_MASK        0x2000000UL
#define TEST_CTRL0_CIA_WDOG_EN_BIT_OFFSET    (24U)
#define TEST_CTRL0_CIA_WDOG_EN_BIT_LEN       (1U)
#define TEST_CTRL0_CIA_WDOG_EN_BIT_MASK      0x1000000UL
#define TEST_CTRL0_CIA_BOOT_SEL_BIT_OFFSET   (23U)
#define TEST_CTRL0_CIA_BOOT_SEL_BIT_LEN      (1U)
#define TEST_CTRL0_CIA_BOOT_SEL_BIT_MASK     0x800000UL
#define TEST_CTRL0_CIA_ROMCPY_BIT_OFFSET     (22U)
#define TEST_CTRL0_CIA_ROMCPY_BIT_LEN        (1U)
#define TEST_CTRL0_CIA_ROMCPY_BIT_MASK       0x400000UL
#define TEST_CTRL0_HIRQ_POL_BIT_OFFSET       (21U)
#define TEST_CTRL0_HIRQ_POL_BIT_LEN          (1U)
#define TEST_CTRL0_HIRQ_POL_BIT_MASK         0x200000UL
#define TEST_CTRL0_FCS_INIT_BIT_OFFSET       (20U)
#define TEST_CTRL0_FCS_INIT_BIT_LEN          (1U)
#define TEST_CTRL0_FCS_INIT_BIT_MASK         0x100000UL
#define TEST_CTRL0_DIS_RSDE_BIT_OFFSET       (19U)
#define TEST_CTRL0_DIS_RSDE_BIT_LEN          (1U)
#define TEST_CTRL0_DIS_RSDE_BIT_MASK         0x80000UL
#define TEST_CTRL0_PRNG_LFSR_RD_BIT_OFFSET   (18U)
#define TEST_CTRL0_PRNG_LFSR_RD_BIT_LEN      (1U)
#define TEST_CTRL0_PRNG_LFSR_RD_BIT_MASK     0x40000UL
#define TEST_CTRL0_TXB_DBG_BIT_OFFSET        (17U)
#define TEST_CTRL0_TXB_DBG_BIT_LEN           (1U)
#define TEST_CTRL0_TXB_DBG_BIT_MASK          0x20000UL
#define TEST_CTRL0_SPICRC_DBG_BIT_OFFSET     (16U)
#define TEST_CTRL0_SPICRC_DBG_BIT_LEN        (1U)
#define TEST_CTRL0_SPICRC_DBG_BIT_MASK       0x10000UL
#define TEST_CTRL0_TST_LCSS_BIT_OFFSET       (15U)
#define TEST_CTRL0_TST_LCSS_BIT_LEN          (1U)
#define TEST_CTRL0_TST_LCSS_BIT_MASK         0x8000U
#define TEST_CTRL0_DIG_LOOPBACK_MODE_BIT_OFFSET (13U)
#define TEST_CTRL0_DIG_LOOPBACK_MODE_BIT_LEN (2U)
#define TEST_CTRL0_DIG_LOOPBACK_MODE_BIT_MASK 0x6000U
#define TEST_CTRL0_ADCTST_RXON_BIT_OFFSET    (12U)
#define TEST_CTRL0_ADCTST_RXON_BIT_LEN       (1U)
#define TEST_CTRL0_ADCTST_RXON_BIT_MASK      0x1000U
#define TEST_CTRL0_TXDATONLY_BIT_OFFSET      (11U)
#define TEST_CTRL0_TXDATONLY_BIT_LEN         (1U)
#define TEST_CTRL0_TXDATONLY_BIT_MASK        0x800U
#define TEST_CTRL0_RX_ALWAYS_EN_BIT_OFFSET   (10U)
#define TEST_CTRL0_RX_ALWAYS_EN_BIT_LEN      (1U)
#define TEST_CTRL0_RX_ALWAYS_EN_BIT_MASK     0x400U
#define TEST_CTRL0_TRD_BIT_OFFSET            (9U)
#define TEST_CTRL0_TRD_BIT_LEN               (1U)
#define TEST_CTRL0_TRD_BIT_MASK              0x200U
#define TEST_CTRL0_CRD_BIT_OFFSET            (8U)
#define TEST_CTRL0_CRD_BIT_LEN               (1U)
#define TEST_CTRL0_CRD_BIT_MASK              0x100U
#define TEST_CTRL0_RSDDIS_BIT_OFFSET         (7U)
#define TEST_CTRL0_RSDDIS_BIT_LEN            (1U)
#define TEST_CTRL0_RSDDIS_BIT_MASK           0x80U
#define TEST_CTRL0_ADC_MEMWRAP_BIT_OFFSET    (6U)
#define TEST_CTRL0_ADC_MEMWRAP_BIT_LEN       (1U)
#define TEST_CTRL0_ADC_MEMWRAP_BIT_MASK      0x40U
#define TEST_CTRL0_ADC_MEMTEST_BIT_OFFSET    (5U)
#define TEST_CTRL0_ADC_MEMTEST_BIT_LEN       (1U)
#define TEST_CTRL0_ADC_MEMTEST_BIT_MASK      0x20U
#define TEST_CTRL0_TPSTM_BIT_OFFSET          (4U)
#define TEST_CTRL0_TPSTM_BIT_LEN             (1U)
#define TEST_CTRL0_TPSTM_BIT_MASK            0x10U
#define TEST_CTRL0_TXSYNC_BIT_OFFSET         (3U)
#define TEST_CTRL0_TXSYNC_BIT_LEN            (1U)
#define TEST_CTRL0_TXSYNC_BIT_MASK           0x8U
#define TEST_CTRL0_TXPRE_BIT_OFFSET          (2U)
#define TEST_CTRL0_TXPRE_BIT_LEN             (1U)
#define TEST_CTRL0_TXPRE_BIT_MASK            0x4U
#define TEST_CTRL0_RF_LOOPBACK_BIT_OFFSET    (1U)
#define TEST_CTRL0_RF_LOOPBACK_BIT_LEN       (1U)
#define TEST_CTRL0_RF_LOOPBACK_BIT_MASK      0x2U
#define TEST_CTRL0_DIG_LOOPBACK_BIT_OFFSET   (0U)
#define TEST_CTRL0_DIG_LOOPBACK_BIT_LEN      (1U)
#define TEST_CTRL0_DIG_LOOPBACK_BIT_MASK     0x1U

/******************************************************************************
* @brief Bit definitions for register EVENT_COUNT7
**/
#define EVENT_COUNT7_ID                      0xf0028
#define EVENT_COUNT7_LEN                     (4U)
#define EVENT_COUNT7_MASK                    0xFFFFFFFFUL
#define EVENT_COUNT7_COUNT_VWARN_BIT_OFFSET  (16U)
#define EVENT_COUNT7_COUNT_VWARN_BIT_LEN     (8U)
#define EVENT_COUNT7_COUNT_VWARN_BIT_MASK    0xff0000UL
#define EVENT_COUNT7_COUNT_CPERR_BIT_OFFSET  (0U)
#define EVENT_COUNT7_COUNT_CPERR_BIT_LEN     (8U)
#define EVENT_COUNT7_COUNT_CPERR_BIT_MASK    0xffU

/******************************************************************************
* @brief Bit definitions for register SPI_MODE
**/
#define SPI_MODE_ID                          0xf002c
#define SPI_MODE_LEN                         (4U)
#define SPI_MODE_MASK                        0xFFFFFFFFUL
#define SPI_MODE_SPI_MODE_BIT_OFFSET         (0U)
#define SPI_MODE_SPI_MODE_BIT_LEN            (2U)
#define SPI_MODE_SPI_MODE_BIT_MASK           0x3U

/******************************************************************************
* @brief Bit definitions for register SYS_STATE_LO
**/
#define SYS_STATE_LO_ID                      0xf0030
#define SYS_STATE_LO_LEN                     (4U)
#define SYS_STATE_LO_MASK                    0xFFFFFFFFUL
#define SYS_STATE_LO_ACC_STATE_BIT_OFFSET    (24U)
#define SYS_STATE_LO_ACC_STATE_BIT_LEN       (8U)
#define SYS_STATE_LO_ACC_STATE_BIT_MASK      0xff000000UL
#define SYS_STATE_LO_TSE_STATE_BIT_OFFSET    (16U)
#define SYS_STATE_LO_TSE_STATE_BIT_LEN       (5U)
#define SYS_STATE_LO_TSE_STATE_BIT_MASK      0x1f0000UL
#define SYS_STATE_LO_RX_STATE_BIT_OFFSET     (8U)
#define SYS_STATE_LO_RX_STATE_BIT_LEN        (6U)
#define SYS_STATE_LO_RX_STATE_BIT_MASK       0x3f00U
#define SYS_STATE_LO_TX_STATE_BIT_OFFSET     (0U)
#define SYS_STATE_LO_TX_STATE_BIT_LEN        (4U)
#define SYS_STATE_LO_TX_STATE_BIT_MASK       0xfU

/******************************************************************************
* @brief Bit definitions for register BIST_CTRL
**/
#define BIST_CTRL_ID                         0xf0034
#define BIST_CTRL_LEN                        (4U)
#define BIST_CTRL_MASK                       0xFFFFFFFFUL
#define BIST_CTRL_ROM_WSOR_BIT_OFFSET        (23U)
#define BIST_CTRL_ROM_WSOR_BIT_LEN           (1U)
#define BIST_CTRL_ROM_WSOR_BIT_MASK          0x800000UL
#define BIST_CTRL_ROM_BIST_WSO_BIT_OFFSET    (22U)
#define BIST_CTRL_ROM_BIST_WSO_BIT_LEN       (1U)
#define BIST_CTRL_ROM_BIST_WSO_BIT_MASK      0x400000UL
#define BIST_CTRL_ROM_BIST_FAIL_BIT_OFFSET   (21U)
#define BIST_CTRL_ROM_BIST_FAIL_BIT_LEN      (1U)
#define BIST_CTRL_ROM_BIST_FAIL_BIT_MASK     0x200000UL
#define BIST_CTRL_ROM_BIST_READY_BIT_OFFSET  (20U)
#define BIST_CTRL_ROM_BIST_READY_BIT_LEN     (1U)
#define BIST_CTRL_ROM_BIST_READY_BIT_MASK    0x100000UL
#define BIST_CTRL_BIST_WSOR_BIT_OFFSET       (19U)
#define BIST_CTRL_BIST_WSOR_BIT_LEN          (1U)
#define BIST_CTRL_BIST_WSOR_BIT_MASK         0x80000UL
#define BIST_CTRL_BIST_WSO_BIT_OFFSET        (18U)
#define BIST_CTRL_BIST_WSO_BIT_LEN           (1U)
#define BIST_CTRL_BIST_WSO_BIT_MASK          0x40000UL
#define BIST_CTRL_BIST_FAIL_BIT_OFFSET       (17U)
#define BIST_CTRL_BIST_FAIL_BIT_LEN          (1U)
#define BIST_CTRL_BIST_FAIL_BIT_MASK         0x20000UL
#define BIST_CTRL_BIST_READY_BIT_OFFSET      (16U)
#define BIST_CTRL_BIST_READY_BIT_LEN         (1U)
#define BIST_CTRL_BIST_READY_BIT_MASK        0x10000UL
#define BIST_CTRL_SEL_ROM_BIST_SERIAL_BIT_OFFSET (14U)
#define BIST_CTRL_SEL_ROM_BIST_SERIAL_BIT_LEN (1U)
#define BIST_CTRL_SEL_ROM_BIST_SERIAL_BIT_MASK 0x4000U
#define BIST_CTRL_RUN_ROM_BIST_BIT_OFFSET    (13U)
#define BIST_CTRL_RUN_ROM_BIST_BIT_LEN       (1U)
#define BIST_CTRL_RUN_ROM_BIST_BIT_MASK      0x2000U
#define BIST_CTRL_AON_BIST_RUN_BIT_OFFSET    (12U)
#define BIST_CTRL_AON_BIST_RUN_BIT_LEN       (1U)
#define BIST_CTRL_AON_BIST_RUN_BIT_MASK      0x1000U
#define BIST_CTRL_BIST_DM2_BIT_OFFSET        (11U)
#define BIST_CTRL_BIST_DM2_BIT_LEN           (1U)
#define BIST_CTRL_BIST_DM2_BIT_MASK          0x800U
#define BIST_CTRL_BIST_DM1_BIT_OFFSET        (10U)
#define BIST_CTRL_BIST_DM1_BIT_LEN           (1U)
#define BIST_CTRL_BIST_DM1_BIT_MASK          0x400U
#define BIST_CTRL_BIST_DM0_BIT_OFFSET        (9U)
#define BIST_CTRL_BIST_DM0_BIT_LEN           (1U)
#define BIST_CTRL_BIST_DM0_BIT_MASK          0x200U
#define BIST_CTRL_BIST_WRSTN_BIT_OFFSET      (8U)
#define BIST_CTRL_BIST_WRSTN_BIT_LEN         (1U)
#define BIST_CTRL_BIST_WRSTN_BIT_MASK        0x100U
#define BIST_CTRL_BIST_WSI_BIT_OFFSET        (7U)
#define BIST_CTRL_BIST_WSI_BIT_LEN           (1U)
#define BIST_CTRL_BIST_WSI_BIT_MASK          0x80U
#define BIST_CTRL_BIST_UPDATE_WR_BIT_OFFSET  (6U)
#define BIST_CTRL_BIST_UPDATE_WR_BIT_LEN     (1U)
#define BIST_CTRL_BIST_UPDATE_WR_BIT_MASK    0x40U
#define BIST_CTRL_BIST_SHIFT_WR_BIT_OFFSET   (5U)
#define BIST_CTRL_BIST_SHIFT_WR_BIT_LEN      (1U)
#define BIST_CTRL_BIST_SHIFT_WR_BIT_MASK     0x20U
#define BIST_CTRL_BIST_SELECT_WIR_BIT_OFFSET (4U)
#define BIST_CTRL_BIST_SELECT_WIR_BIT_LEN    (1U)
#define BIST_CTRL_BIST_SELECT_WIR_BIT_MASK   0x10U
#define BIST_CTRL_BIST_CAPTURE_WR_BIT_OFFSET (3U)
#define BIST_CTRL_BIST_CAPTURE_WR_BIT_LEN    (1U)
#define BIST_CTRL_BIST_CAPTURE_WR_BIT_MASK   0x8U
#define BIST_CTRL_BIST_WCLK_BIT_OFFSET       (2U)
#define BIST_CTRL_BIST_WCLK_BIT_LEN          (1U)
#define BIST_CTRL_BIST_WCLK_BIT_MASK         0x4U
#define BIST_CTRL_BIST_SMART_EN_BIT_OFFSET   (1U)
#define BIST_CTRL_BIST_SMART_EN_BIT_LEN      (1U)
#define BIST_CTRL_BIST_SMART_EN_BIT_MASK     0x2U
#define BIST_CTRL_SRAM_BIST_RUN_BIT_OFFSET   (0U)
#define BIST_CTRL_SRAM_BIST_RUN_BIT_LEN      (1U)
#define BIST_CTRL_SRAM_BIST_RUN_BIT_MASK     0x1U

/******************************************************************************
* @brief Bit definitions for register TST_DBG
**/
#define TST_DBG_ID                           0xf0038
#define TST_DBG_LEN                          (4U)
#define TST_DBG_MASK                         0xFFFFFFFFUL
#define TST_DBG_DBUG_MUX_SEL_BIT_OFFSET      (2U)
#define TST_DBG_DBUG_MUX_SEL_BIT_LEN         (6U)
#define TST_DBG_DBUG_MUX_SEL_BIT_MASK        0xfcU
#define TST_DBG_TST_DBG_BIT_OFFSET           (0U)
#define TST_DBG_TST_DBG_BIT_LEN              (2U)
#define TST_DBG_TST_DBG_BIT_MASK             0x3U

/******************************************************************************
* @brief Bit definitions for register FCMD_STATUS
**/
#define FCMD_STATUS_ID                       0xf003c
#define FCMD_STATUS_LEN                      (4U)
#define FCMD_STATUS_MASK                     0xFFFFFFFFUL
#define FCMD_STATUS_FCMD_STATUS_BIT_OFFSET   (0U)
#define FCMD_STATUS_FCMD_STATUS_BIT_LEN      (5U)
#define FCMD_STATUS_FCMD_STATUS_BIT_MASK     0x1fU

/******************************************************************************
* @brief Bit definitions for register TEST_LOGGING
**/
#define TEST_LOGGING_ID                      0xf0040
#define TEST_LOGGING_LEN                     (4U)
#define TEST_LOGGING_MASK                    0xFFFFFFFFUL
#define TEST_LOGGING_LOGGER_CLR_BIT_OFFSET   (8U)
#define TEST_LOGGING_LOGGER_CLR_BIT_LEN      (1U)
#define TEST_LOGGING_LOGGER_CLR_BIT_MASK     0x100U
#define TEST_LOGGING_LOGGER_TRIG_BIT_OFFSET  (2U)
#define TEST_LOGGING_LOGGER_TRIG_BIT_LEN     (3U)
#define TEST_LOGGING_LOGGER_TRIG_BIT_MASK    0x1cU
#define TEST_LOGGING_LOGGER_MODE_BIT_OFFSET  (0U)
#define TEST_LOGGING_LOGGER_MODE_BIT_LEN     (2U)
#define TEST_LOGGING_LOGGER_MODE_BIT_MASK    0x3U

/******************************************************************************
* @brief Bit definitions for register STATUS_LOGGING
**/
#define STATUS_LOGGING_ID                    0xf0044
#define STATUS_LOGGING_LEN                   (4U)
#define STATUS_LOGGING_MASK                  0xFFFFFFFFUL
#define STATUS_LOGGING_LOGGED_LOCS_BIT_OFFSET (0U)
#define STATUS_LOGGING_LOGGED_LOCS_BIT_LEN   (10U)
#define STATUS_LOGGING_LOGGED_LOCS_BIT_MASK  0x3ffU

/******************************************************************************
* @brief Bit definitions for register CTR_DBG
**/
#define CTR_DBG_ID                           0xf0048
#define CTR_DBG_LEN                          (4U)
#define CTR_DBG_MASK                         0xFFFFFFFFUL
#define CTR_DBG_CTR_DBG_BIT_OFFSET           (0U)
#define CTR_DBG_CTR_DBG_BIT_LEN              (32U)
#define CTR_DBG_CTR_DBG_BIT_MASK             0xffffffffUL

/******************************************************************************
* @brief Bit definitions for register INIT
**/
#define INIT_ID                              0xf004c
#define INIT_LEN                             (4U)
#define INIT_MASK                            0xFFFFFFFFUL
#define INIT_INIT_BIT_OFFSET                 (0U)
#define INIT_INIT_BIT_LEN                    (8U)
#define INIT_INIT_BIT_MASK                   0xffU

/******************************************************************************
* @brief Bit definitions for register SOFT_RST
**/
#define SOFT_RST_ID                          0x110000
#define SOFT_RST_LEN                         (4U)
#define SOFT_RST_MASK                        0xFFFFFFFFUL
#define SOFT_RST_BLOCK_CLR_BIT_OFFSET        (9U)
#define SOFT_RST_BLOCK_CLR_BIT_LEN           (7U)
#define SOFT_RST_BLOCK_CLR_BIT_MASK          0xfe00U
#define SOFT_RST_GPIO_RST_N_BIT_OFFSET       (8U)
#define SOFT_RST_GPIO_RST_N_BIT_LEN          (1U)
#define SOFT_RST_GPIO_RST_N_BIT_MASK         0x100U
#define SOFT_RST_PMSC_RST_N_BIT_OFFSET       (7U)
#define SOFT_RST_PMSC_RST_N_BIT_LEN          (1U)
#define SOFT_RST_PMSC_RST_N_BIT_MASK         0x80U
#define SOFT_RST_HIF_RST_N_BIT_OFFSET        (6U)
#define SOFT_RST_HIF_RST_N_BIT_LEN           (1U)
#define SOFT_RST_HIF_RST_N_BIT_MASK          0x40U
#define SOFT_RST_TX_RST_N_BIT_OFFSET         (5U)
#define SOFT_RST_TX_RST_N_BIT_LEN            (1U)
#define SOFT_RST_TX_RST_N_BIT_MASK           0x20U
#define SOFT_RST_RX_RST_N_BIT_OFFSET         (4U)
#define SOFT_RST_RX_RST_N_BIT_LEN            (1U)
#define SOFT_RST_RX_RST_N_BIT_MASK           0x10U
#define SOFT_RST_BIST_RST_N_BIT_OFFSET       (3U)
#define SOFT_RST_BIST_RST_N_BIT_LEN          (1U)
#define SOFT_RST_BIST_RST_N_BIT_MASK         0x8U
#define SOFT_RST_CIA_RST_N_BIT_OFFSET        (2U)
#define SOFT_RST_CIA_RST_N_BIT_LEN           (1U)
#define SOFT_RST_CIA_RST_N_BIT_MASK          0x4U
#define SOFT_RST_PRNG_RST_N_BIT_OFFSET       (1U)
#define SOFT_RST_PRNG_RST_N_BIT_LEN          (1U)
#define SOFT_RST_PRNG_RST_N_BIT_MASK         0x2U
#define SOFT_RST_ARM_RST_N_BIT_OFFSET        (0U)
#define SOFT_RST_ARM_RST_N_BIT_LEN           (1U)
#define SOFT_RST_ARM_RST_N_BIT_MASK          0x1U

/******************************************************************************
* @brief Bit definitions for register CLK_CTRL
**/
#define CLK_CTRL_ID                          0x110004
#define CLK_CTRL_LEN                         (4U)
#define CLK_CTRL_MASK                        0xFFFFFFFFUL
#define CLK_CTRL_BIST_CLK_EN_BIT_OFFSET      (26U)
#define CLK_CTRL_BIST_CLK_EN_BIT_LEN         (1U)
#define CLK_CTRL_BIST_CLK_EN_BIT_MASK        0x4000000UL
#define CLK_CTRL_PLL_LOCK_TIMER_EN_BIT_OFFSET (25U)
#define CLK_CTRL_PLL_LOCK_TIMER_EN_BIT_LEN   (1U)
#define CLK_CTRL_PLL_LOCK_TIMER_EN_BIT_MASK  0x2000000UL
#define CLK_CTRL_SLEEP_MODE_BIT_OFFSET       (24U)
#define CLK_CTRL_SLEEP_MODE_BIT_LEN          (1U)
#define CLK_CTRL_SLEEP_MODE_BIT_MASK         0x1000000UL
#define CLK_CTRL_LP_CLK_EN_BIT_OFFSET        (23U)
#define CLK_CTRL_LP_CLK_EN_BIT_LEN           (1U)
#define CLK_CTRL_LP_CLK_EN_BIT_MASK          0x800000UL
#define CLK_CTRL_CIA_WR_CLK_EN_BIT_OFFSET    (22U)
#define CLK_CTRL_CIA_WR_CLK_EN_BIT_LEN       (1U)
#define CLK_CTRL_CIA_WR_CLK_EN_BIT_MASK      0x400000UL
#define CLK_CTRL_RX_BUFF_AUTO_CLK_BIT_OFFSET (21U)
#define CLK_CTRL_RX_BUFF_AUTO_CLK_BIT_LEN    (1U)
#define CLK_CTRL_RX_BUFF_AUTO_CLK_BIT_MASK   0x200000UL
#define CLK_CTRL_CODE_MEM_AUTO_CLK_BIT_OFFSET (20U)
#define CLK_CTRL_CODE_MEM_AUTO_CLK_BIT_LEN   (1U)
#define CLK_CTRL_CODE_MEM_AUTO_CLK_BIT_MASK  0x100000UL
#define CLK_CTRL_GPIO_DBNC_RST_N_BIT_OFFSET  (19U)
#define CLK_CTRL_GPIO_DBNC_RST_N_BIT_LEN     (1U)
#define CLK_CTRL_GPIO_DBNC_RST_N_BIT_MASK    0x80000UL
#define CLK_CTRL_GPIO_DBNC_CLK_EN_BIT_OFFSET (18U)
#define CLK_CTRL_GPIO_DBNC_CLK_EN_BIT_LEN    (1U)
#define CLK_CTRL_GPIO_DBNC_CLK_EN_BIT_MASK   0x40000UL
#define CLK_CTRL_GPIO_CLK_EN_BIT_OFFSET      (16U)
#define CLK_CTRL_GPIO_CLK_EN_BIT_LEN         (1U)
#define CLK_CTRL_GPIO_CLK_EN_BIT_MASK        0x10000UL
#define CLK_CTRL_ACC_MEM_CLK_ON_BIT_OFFSET   (15U)
#define CLK_CTRL_ACC_MEM_CLK_ON_BIT_LEN      (1U)
#define CLK_CTRL_ACC_MEM_CLK_ON_BIT_MASK     0x8000U
#define CLK_CTRL_RSD_CLK_ON_BIT_OFFSET       (14U)
#define CLK_CTRL_RSD_CLK_ON_BIT_LEN          (1U)
#define CLK_CTRL_RSD_CLK_ON_BIT_MASK         0x4000U
#define CLK_CTRL_LOOPBACK_CLK_EN_BIT_OFFSET  (13U)
#define CLK_CTRL_LOOPBACK_CLK_EN_BIT_LEN     (1U)
#define CLK_CTRL_LOOPBACK_CLK_EN_BIT_MASK    0x2000U
#define CLK_CTRL_TX_BUF_CLK_ON_BIT_OFFSET    (12U)
#define CLK_CTRL_TX_BUF_CLK_ON_BIT_LEN       (1U)
#define CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK      0x1000U
#define CLK_CTRL_RX_BUF_CLK_ON_BIT_OFFSET    (11U)
#define CLK_CTRL_RX_BUF_CLK_ON_BIT_LEN       (1U)
#define CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK      0x800U
#define CLK_CTRL_FORCE_SAR_CLK_EN_BIT_OFFSET (10U)
#define CLK_CTRL_FORCE_SAR_CLK_EN_BIT_LEN    (1U)
#define CLK_CTRL_FORCE_SAR_CLK_EN_BIT_MASK   0x400U
#define CLK_CTRL_FORCE_NVM_CLK_EN_BIT_OFFSET (9U)
#define CLK_CTRL_FORCE_NVM_CLK_EN_BIT_LEN    (1U)
#define CLK_CTRL_FORCE_NVM_CLK_EN_BIT_MASK   0x200U
#define CLK_CTRL_FORCE_CIA_CLKS_ON_BIT_OFFSET (8U)
#define CLK_CTRL_FORCE_CIA_CLKS_ON_BIT_LEN   (1U)
#define CLK_CTRL_FORCE_CIA_CLKS_ON_BIT_MASK  0x100U
#define CLK_CTRL_RX_CLK_GATE_DISABLE_BIT_OFFSET (7U)
#define CLK_CTRL_RX_CLK_GATE_DISABLE_BIT_LEN (1U)
#define CLK_CTRL_RX_CLK_GATE_DISABLE_BIT_MASK 0x80U
#define CLK_CTRL_FORCE_ACC_CLK_BIT_OFFSET    (6U)
#define CLK_CTRL_FORCE_ACC_CLK_BIT_LEN       (1U)
#define CLK_CTRL_FORCE_ACC_CLK_BIT_MASK      0x40U
#define CLK_CTRL_TX_CLK_SEL_BIT_OFFSET       (4U)
#define CLK_CTRL_TX_CLK_SEL_BIT_LEN          (2U)
#define CLK_CTRL_TX_CLK_SEL_BIT_MASK         0x30U
#define CLK_CTRL_RX_CLK_SEL_BIT_OFFSET       (2U)
#define CLK_CTRL_RX_CLK_SEL_BIT_LEN          (2U)
#define CLK_CTRL_RX_CLK_SEL_BIT_MASK         0xcU
#define CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET      (0U)
#define CLK_CTRL_SYS_CLK_SEL_BIT_LEN         (2U)
#define CLK_CTRL_SYS_CLK_SEL_BIT_MASK        0x3U

/******************************************************************************
* @brief Bit definitions for register SEQ_CTRL
**/
#define SEQ_CTRL_ID                          0x110008
#define SEQ_CTRL_LEN                         (4U)
#define SEQ_CTRL_MASK                        0xFFFFFFFFUL
#define SEQ_CTRL_LP_CLK_DIV_BIT_OFFSET       (26U)
#define SEQ_CTRL_LP_CLK_DIV_BIT_LEN          (6U)
#define SEQ_CTRL_LP_CLK_DIV_BIT_MASK         0xfc000000UL
#define SEQ_CTRL_FORCE_SYNC_BIT_OFFSET       (25U)
#define SEQ_CTRL_FORCE_SYNC_BIT_LEN          (1U)
#define SEQ_CTRL_FORCE_SYNC_BIT_MASK         0x2000000UL
#define SEQ_CTRL_FORCE2RC_BIT_OFFSET         (24U)
#define SEQ_CTRL_FORCE2RC_BIT_LEN            (1U)
#define SEQ_CTRL_FORCE2RC_BIT_MASK           0x1000000UL
#define SEQ_CTRL_FORCE2INIT_BIT_OFFSET       (23U)
#define SEQ_CTRL_FORCE2INIT_BIT_LEN          (1U)
#define SEQ_CTRL_FORCE2INIT_BIT_MASK         0x800000UL
#define SEQ_CTRL_FORCE2IDLE_BIT_OFFSET       (22U)
#define SEQ_CTRL_FORCE2IDLE_BIT_LEN          (1U)
#define SEQ_CTRL_FORCE2IDLE_BIT_MASK         0x400000UL
#define SEQ_CTRL_RX_RST_MODE_BIT_OFFSET      (21U)
#define SEQ_CTRL_RX_RST_MODE_BIT_LEN         (1U)
#define SEQ_CTRL_RX_RST_MODE_BIT_MASK        0x200000UL
#define SEQ_CTRL_FORCE_RX_STATE_BIT_OFFSET   (20U)
#define SEQ_CTRL_FORCE_RX_STATE_BIT_LEN      (1U)
#define SEQ_CTRL_FORCE_RX_STATE_BIT_MASK     0x100000UL
#define SEQ_CTRL_FORCE_TX_STATE_BIT_OFFSET   (19U)
#define SEQ_CTRL_FORCE_TX_STATE_BIT_LEN      (1U)
#define SEQ_CTRL_FORCE_TX_STATE_BIT_MASK     0x80000UL
#define SEQ_CTRL_FORCE_CAL_MODE_BIT_OFFSET   (18U)
#define SEQ_CTRL_FORCE_CAL_MODE_BIT_LEN      (1U)
#define SEQ_CTRL_FORCE_CAL_MODE_BIT_MASK     0x40000UL
#define SEQ_CTRL_CIA_SEQ_EN_BIT_OFFSET       (17U)
#define SEQ_CTRL_CIA_SEQ_EN_BIT_LEN          (1U)
#define SEQ_CTRL_CIA_SEQ_EN_BIT_MASK         0x20000UL
#define SEQ_CTRL_RX_OFF_EARLY_EN_BIT_OFFSET  (16U)
#define SEQ_CTRL_RX_OFF_EARLY_EN_BIT_LEN     (1U)
#define SEQ_CTRL_RX_OFF_EARLY_EN_BIT_MASK    0x10000UL
#define SEQ_CTRL_PLL_SYNC_MODE_BIT_OFFSET    (15U)
#define SEQ_CTRL_PLL_SYNC_MODE_BIT_LEN       (1U)
#define SEQ_CTRL_PLL_SYNC_MODE_BIT_MASK      0x8000U
#define SEQ_CTRL_SNOOZE_REPEAT_BIT_OFFSET    (14U)
#define SEQ_CTRL_SNOOZE_REPEAT_BIT_LEN       (1U)
#define SEQ_CTRL_SNOOZE_REPEAT_BIT_MASK      0x4000U
#define SEQ_CTRL_SNOOZE_EN_BIT_OFFSET        (13U)
#define SEQ_CTRL_SNOOZE_EN_BIT_LEN           (1U)
#define SEQ_CTRL_SNOOZE_EN_BIT_MASK          0x2000U
#define SEQ_CTRL_AUTO_RX2SLP_BIT_OFFSET      (12U)
#define SEQ_CTRL_AUTO_RX2SLP_BIT_LEN         (1U)
#define SEQ_CTRL_AUTO_RX2SLP_BIT_MASK        0x1000U
#define SEQ_CTRL_AUTO_TX2SLP_BIT_OFFSET      (11U)
#define SEQ_CTRL_AUTO_TX2SLP_BIT_LEN         (1U)
#define SEQ_CTRL_AUTO_TX2SLP_BIT_MASK        0x800U
#define SEQ_CTRL_AUTO_RX_SEQ_BIT_OFFSET      (10U)
#define SEQ_CTRL_AUTO_RX_SEQ_BIT_LEN         (1U)
#define SEQ_CTRL_AUTO_RX_SEQ_BIT_MASK        0x400U
#define SEQ_CTRL_AUTO_TX_SEQ_BIT_OFFSET      (9U)
#define SEQ_CTRL_AUTO_TX_SEQ_BIT_LEN         (1U)
#define SEQ_CTRL_AUTO_TX_SEQ_BIT_MASK        0x200U
#define SEQ_CTRL_AUTO_INIT2IDLE_BIT_OFFSET   (8U)
#define SEQ_CTRL_AUTO_INIT2IDLE_BIT_LEN      (1U)
#define SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK     0x100U
#define SEQ_CTRL_AUTOINIT_MODE_BIT_OFFSET    (7U)
#define SEQ_CTRL_AUTOINIT_MODE_BIT_LEN       (1U)
#define SEQ_CTRL_AUTOINIT_MODE_BIT_MASK      0x80U
#define SEQ_CTRL_DA_CAL_MASK_BIT_OFFSET      (6U)
#define SEQ_CTRL_DA_CAL_MASK_BIT_LEN         (1U)
#define SEQ_CTRL_DA_CAL_MASK_BIT_MASK        0x40U
#define SEQ_CTRL_AON_CLK_CTRL_BIT_OFFSET     (5U)
#define SEQ_CTRL_AON_CLK_CTRL_BIT_LEN        (1U)
#define SEQ_CTRL_AON_CLK_CTRL_BIT_MASK       0x20U
#define SEQ_CTRL_LDO_CTRL_BIT_OFFSET         (4U)
#define SEQ_CTRL_LDO_CTRL_BIT_LEN            (1U)
#define SEQ_CTRL_LDO_CTRL_BIT_MASK           0x10U
#define SEQ_CTRL_RF_CTRL_BIT_OFFSET          (3U)
#define SEQ_CTRL_RF_CTRL_BIT_LEN             (1U)
#define SEQ_CTRL_RF_CTRL_BIT_MASK            0x8U
#define SEQ_CTRL_AUTOTX2INIT_BIT_OFFSET      (2U)
#define SEQ_CTRL_AUTOTX2INIT_BIT_LEN         (1U)
#define SEQ_CTRL_AUTOTX2INIT_BIT_MASK        0x4U
#define SEQ_CTRL_AUTORX2INIT_BIT_OFFSET      (1U)
#define SEQ_CTRL_AUTORX2INIT_BIT_LEN         (1U)
#define SEQ_CTRL_AUTORX2INIT_BIT_MASK        0x2U
#define SEQ_CTRL_SYS_TIME_ON_BIT_OFFSET      (0U)
#define SEQ_CTRL_SYS_TIME_ON_BIT_LEN         (1U)
#define SEQ_CTRL_SYS_TIME_ON_BIT_MASK        0x1U

/******************************************************************************
* @brief Bit definitions for register SNOOZE_CNT
**/
#define SNOOZE_CNT_ID                        0x11000c
#define SNOOZE_CNT_LEN                       (4U)
#define SNOOZE_CNT_MASK                      0xFFFFFFFFUL
#define SNOOZE_CNT_SNZ_CNT_BIT_OFFSET        (0U)
#define SNOOZE_CNT_SNZ_CNT_BIT_LEN           (8U)
#define SNOOZE_CNT_SNZ_CNT_BIT_MASK          0xffU

/******************************************************************************
* @brief Bit definitions for register PWR_UP_TIMES_LO
**/
#define PWR_UP_TIMES_LO_ID                   0x110010
#define PWR_UP_TIMES_LO_LEN                  (4U)
#define PWR_UP_TIMES_LO_MASK                 0xFFFFFFFFUL
#define PWR_UP_TIMES_LO_EXT_DA_DLY_BIT_OFFSET (29U)
#define PWR_UP_TIMES_LO_EXT_DA_DLY_BIT_LEN   (3U)
#define PWR_UP_TIMES_LO_EXT_DA_DLY_BIT_MASK  0xe0000000UL
#define PWR_UP_TIMES_LO_TXC_RF_OFF_DLY_BIT_OFFSET (27U)
#define PWR_UP_TIMES_LO_TXC_RF_OFF_DLY_BIT_LEN (2U)
#define PWR_UP_TIMES_LO_TXC_RF_OFF_DLY_BIT_MASK 0x18000000UL
#define PWR_UP_TIMES_LO_TXC_MIX_DLY_BIT_OFFSET (24U)
#define PWR_UP_TIMES_LO_TXC_MIX_DLY_BIT_LEN  (3U)
#define PWR_UP_TIMES_LO_TXC_MIX_DLY_BIT_MASK 0x7000000UL
#define PWR_UP_TIMES_LO_TXC_DA_DLY_BIT_OFFSET (21U)
#define PWR_UP_TIMES_LO_TXC_DA_DLY_BIT_LEN   (3U)
#define PWR_UP_TIMES_LO_TXC_DA_DLY_BIT_MASK  0xe00000UL
#define PWR_UP_TIMES_LO_TX_HOP_DLY_BIT_OFFSET (18U)
#define PWR_UP_TIMES_LO_TX_HOP_DLY_BIT_LEN   (3U)
#define PWR_UP_TIMES_LO_TX_HOP_DLY_BIT_MASK  0x1c0000UL
#define PWR_UP_TIMES_LO_HIF_TIM_CLK_DLY_BIT_OFFSET (14U)
#define PWR_UP_TIMES_LO_HIF_TIM_CLK_DLY_BIT_LEN (4U)
#define PWR_UP_TIMES_LO_HIF_TIM_CLK_DLY_BIT_MASK 0x3c000UL
#define PWR_UP_TIMES_LO_HIF_TIM_DAC_DLY_BIT_OFFSET (10U)
#define PWR_UP_TIMES_LO_HIF_TIM_DAC_DLY_BIT_LEN (4U)
#define PWR_UP_TIMES_LO_HIF_TIM_DAC_DLY_BIT_MASK 0x3c00U
#define PWR_UP_TIMES_LO_HIF_TIM_LDO_DLY_BIT_OFFSET (6U)
#define PWR_UP_TIMES_LO_HIF_TIM_LDO_DLY_BIT_LEN (4U)
#define PWR_UP_TIMES_LO_HIF_TIM_LDO_DLY_BIT_MASK 0x3c0U
#define PWR_UP_TIMES_LO_PLL_LOCK_CNT_EN_BIT_OFFSET (5U)
#define PWR_UP_TIMES_LO_PLL_LOCK_CNT_EN_BIT_LEN (1U)
#define PWR_UP_TIMES_LO_PLL_LOCK_CNT_EN_BIT_MASK 0x20U
#define PWR_UP_TIMES_LO_PLL_LOCK_TIME_BIT_OFFSET (0U)
#define PWR_UP_TIMES_LO_PLL_LOCK_TIME_BIT_LEN (5U)
#define PWR_UP_TIMES_LO_PLL_LOCK_TIME_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register PWR_UP_TIMES_HI
**/
#define PWR_UP_TIMES_HI_ID                   0x110014
#define PWR_UP_TIMES_HI_LEN                  (2U)
#define PWR_UP_TIMES_HI_MASK                 0xFFFFU
#define PWR_UP_TIMES_HI_TXC_LONG_DATA_EN_BIT_OFFSET (12U)
#define PWR_UP_TIMES_HI_TXC_LONG_DATA_EN_BIT_LEN (1U)
#define PWR_UP_TIMES_HI_TXC_LONG_DATA_EN_BIT_MASK 0x1000U
#define PWR_UP_TIMES_HI_TXC_MIX_LO_EN_BIT_OFFSET (9U)
#define PWR_UP_TIMES_HI_TXC_MIX_LO_EN_BIT_LEN (3U)
#define PWR_UP_TIMES_HI_TXC_MIX_LO_EN_BIT_MASK 0xe00U
#define PWR_UP_TIMES_HI_DX_DSPRD_OFF_DLY_BIT_OFFSET (5U)
#define PWR_UP_TIMES_HI_DX_DSPRD_OFF_DLY_BIT_LEN (4U)
#define PWR_UP_TIMES_HI_DX_DSPRD_OFF_DLY_BIT_MASK 0x1e0U
#define PWR_UP_TIMES_HI_RX_DSPRD_ON_DLY_BIT_OFFSET (0U)
#define PWR_UP_TIMES_HI_RX_DSPRD_ON_DLY_BIT_LEN (5U)
#define PWR_UP_TIMES_HI_RX_DSPRD_ON_DLY_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register LED_CTRL
**/
#define LED_CTRL_ID                          0x110016             /*  */
#define LED_CTRL_LEN                         (4U)
#define LED_CTRL_MASK                        0xFFFFFFFFUL
#define LED_CTRL_FORCE_SOLID_BIT_OFFSET      (24U)
#define LED_CTRL_FORCE_SOLID_BIT_LEN         (4U)
#define LED_CTRL_FORCE_SOLID_BIT_MASK        0xf000000UL
#define LED_CTRL_FORCE_TOGGLE_BIT_OFFSET     (20U)
#define LED_CTRL_FORCE_TOGGLE_BIT_LEN        (4U)
#define LED_CTRL_FORCE_TOGGLE_BIT_MASK       0xf00000UL
#define LED_CTRL_FORCE_TRIGGER_BIT_OFFSET    (16U)
#define LED_CTRL_FORCE_TRIGGER_BIT_LEN       (4U)
#define LED_CTRL_FORCE_TRIGGER_BIT_MASK      0xf0000UL
#define LED_CTRL_BLINK_EN_BIT_OFFSET         (8U)
#define LED_CTRL_BLINK_EN_BIT_LEN            (1U)
#define LED_CTRL_BLINK_EN_BIT_MASK           0x100U
#define LED_CTRL_BLINK_CNT_BIT_OFFSET        (0U)
#define LED_CTRL_BLINK_CNT_BIT_LEN           (8U)
#define LED_CTRL_BLINK_CNT_BIT_MASK          0xffU

/******************************************************************************
* @brief Bit definitions for register RX_PPM
**/
#define RX_PPM_ID                            0x11001a             /*  { aliased = true}  */
#define RX_PPM_LEN                           (4U)
#define RX_PPM_MASK                          0xFFFFFFFFUL
#define RX_PPM_PPM_OFF_BIT_OFFSET            (8U)
#define RX_PPM_PPM_OFF_BIT_LEN               (8U)
#define RX_PPM_PPM_OFF_BIT_MASK              0xff00U
#define RX_PPM_PPM_ON_BIT_OFFSET             (0U)
#define RX_PPM_PPM_ON_BIT_LEN                (4U)
#define RX_PPM_PPM_ON_BIT_MASK               0xfU

/******************************************************************************
* @brief Bit definitions for register FOSC_CTRL
**/
#define FOSC_CTRL_ID                         0x11001e            /*  { aliased = true}  */
#define FOSC_CTRL_LEN                        (1U)
#define FOSC_CTRL_MASK                       0xFFU
#define FOSC_CTRL_FOSC_TRIM_BIT_OFFSET       (0U)
#define FOSC_CTRL_FOSC_TRIM_BIT_LEN          (6U)
#define FOSC_CTRL_FOSC_TRIM_BIT_MASK         0x3fU

/******************************************************************************
* @brief Bit definitions for register BIAS_CTRL
**/
#define BIAS_CTRL_ID                         0x11001f             /*  */
#define BIAS_CTRL_LEN                        (4U)
#define BIAS_CTRL_MASK                       0xFFFFFFFFUL
#define BIAS_CTRL_DIG_BIAS_CTRL_TC_R3_ULV_BIT_OFFSET (12U)
#define BIAS_CTRL_DIG_BIAS_CTRL_TC_R3_ULV_BIT_LEN (2U)
#define BIAS_CTRL_DIG_BIAS_CTRL_TC_R3_ULV_BIT_MASK 0x3000U
#define BIAS_CTRL_DIG_BIAS_TESTSEL_ULV_BIT_OFFSET (9U)
#define BIAS_CTRL_DIG_BIAS_TESTSEL_ULV_BIT_LEN (3U)
#define BIAS_CTRL_DIG_BIAS_TESTSEL_ULV_BIT_MASK 0xe00U
#define BIAS_CTRL_DIG_BIAS_CTRL_TC_R1_ULV_BIT_OFFSET (5U)
#define BIAS_CTRL_DIG_BIAS_CTRL_TC_R1_ULV_BIT_LEN (4U)
#define BIAS_CTRL_DIG_BIAS_CTRL_TC_R1_ULV_BIT_MASK 0x1e0U
#define BIAS_CTRL_DIG_BIAS_DAC_ULV_BIT_OFFSET (0U)
#define BIAS_CTRL_DIG_BIAS_DAC_ULV_BIT_LEN   (5U)
#define BIAS_CTRL_DIG_BIAS_DAC_ULV_BIT_MASK  0x1fU

/******************************************************************************
* @brief Bit definitions for register FINT_STAT
**/
#define FINT_STAT_ID                         0x1F0000
#define FINT_STAT_LEN                        (4U)
#define FINT_STAT_MASK                       0xFFFFFFFFUL
#define FINT_STAT_SYS_PANIC_BIT_OFFSET       (7U)
#define FINT_STAT_SYS_PANIC_BIT_LEN          (1U)
#define FINT_STAT_SYS_PANIC_BIT_MASK         0x80U
#define FINT_STAT_SYS_EVENT_BIT_OFFSET       (6U)
#define FINT_STAT_SYS_EVENT_BIT_LEN          (1U)
#define FINT_STAT_SYS_EVENT_BIT_MASK         0x40U
#define FINT_STAT_RXTO_BIT_OFFSET            (5U)
#define FINT_STAT_RXTO_BIT_LEN               (1U)
#define FINT_STAT_RXTO_BIT_MASK              0x20U
#define FINT_STAT_RXERR_BIT_OFFSET           (4U)
#define FINT_STAT_RXERR_BIT_LEN              (1U)
#define FINT_STAT_RXERR_BIT_MASK             0x10U
#define FINT_STAT_RXOK_BIT_OFFSET            (3U)
#define FINT_STAT_RXOK_BIT_LEN               (1U)
#define FINT_STAT_RXOK_BIT_MASK              0x8U
#define FINT_STAT_RXTSERR_BIT_OFFSET         (2U)
#define FINT_STAT_RXTSERR_BIT_LEN            (1U)
#define FINT_STAT_RXTSERR_BIT_MASK           0x4U
#define FINT_STAT_CCA_FAIL_AAT_BIT_OFFSET    (1U)
#define FINT_STAT_CCA_FAIL_AAT_BIT_LEN       (1U)
#define FINT_STAT_CCA_FAIL_AAT_BIT_MASK      0x2U
#define FINT_STAT_TXOK_BIT_OFFSET            (0U)
#define FINT_STAT_TXOK_BIT_LEN               (1U)
#define FINT_STAT_TXOK_BIT_MASK              0x1U

/******************************************************************************
* @brief Bit definitions for register INDIRECT_ADDR_A
**/
#define INDIRECT_ADDR_A_ID                   0x1f0004
#define INDIRECT_ADDR_A_LEN                  (4U)
#define INDIRECT_ADDR_A_MASK                 0xFFFFFFFFUL
#define INDIRECT_ADDR_A_INDIRECT_ADDR_A_BIT_OFFSET (0U)
#define INDIRECT_ADDR_A_INDIRECT_ADDR_A_BIT_LEN (5U)
#define INDIRECT_ADDR_A_INDIRECT_ADDR_A_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register ADDR_OFFSET_A
**/
#define ADDR_OFFSET_A_ID                     0x1f0008
#define ADDR_OFFSET_A_LEN                    (4U)
#define ADDR_OFFSET_A_MASK                   0xFFFFFFFFUL
#define ADDR_OFFSET_A_ADDR_OFFSET_A_BIT_OFFSET (0U)
#define ADDR_OFFSET_A_ADDR_OFFSET_A_BIT_LEN  (15U)
#define ADDR_OFFSET_A_ADDR_OFFSET_A_BIT_MASK 0x7fffU

/******************************************************************************
* @brief Bit definitions for register INDIRECT_ADDR_B
**/
#define INDIRECT_ADDR_B_ID                   0x1f000c
#define INDIRECT_ADDR_B_LEN                  (4U)
#define INDIRECT_ADDR_B_MASK                 0xFFFFFFFFUL
#define INDIRECT_ADDR_B_INDIRECT_ADDR_B_BIT_OFFSET (0U)
#define INDIRECT_ADDR_B_INDIRECT_ADDR_B_BIT_LEN (5U)
#define INDIRECT_ADDR_B_INDIRECT_ADDR_B_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register ADDR_OFFSET_B
**/
#define ADDR_OFFSET_B_ID                     0x1f0010
#define ADDR_OFFSET_B_LEN                    (4U)
#define ADDR_OFFSET_B_MASK                   0xFFFFFFFFUL
#define ADDR_OFFSET_B_ADDR_OFFSET_B_BIT_OFFSET (0U)
#define ADDR_OFFSET_B_ADDR_OFFSET_B_BIT_LEN  (15U)
#define ADDR_OFFSET_B_ADDR_OFFSET_B_BIT_MASK 0x7fffU

/* END DW3000 REGISTER DEFINITION */

#ifdef __cplusplus
}
#endif

#endif
