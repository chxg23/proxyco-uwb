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
 * @brief Register values and aliases
 *
 * @details Complimentary registers and aliases for dw3000
 */

#ifndef _DW3000_REGS_VALS_H_
#define _DW3000_REGS_VALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define FCS_LEN                 (2)
#define RX_BUFFER_A_ID          0x120000        /* Receive Data Buffer (in double buffer set) */
#define RX_BUFFER_B_ID          0x130000        /* Receive Data Buffer (in double buffer set) */

#define RF_TXCTRL_CH5           0x1C071134UL    /* */
#define RF_TXCTRL_CH9           0x1C010034UL    /* */
#define RF_RXCTRL_CH9           0x08B5A833UL    /* */

#define RF_PLL_CFG_CH5          0x1F3C
#define RF_PLL_CFG_CH9          0x0F3C

#define TC_PGDELAY_CH5 (0x34)   /* Taken from simple_tx.c example. TODO: calibrate / fix */
#define TC_PGDELAY_CH9 (0x34)   /* Taken from simple_tx.c example. TODO: calibrate / fix */

#define PD_THRESH_NO_DATA       0xAF5F35CC      /* PD threshold for no data STS mode*/


#define ACC_BUFFER_MAX_LEN      (12288 + 1)     /* +1 is needed for "dummy" read */

#define REG_DIRECT_OFFSET_MAX_LEN   (127)
#define EXT_FRAME_LEN               (1023)
#define INDIRECT_POINTER_A_ID       0x1D0000    /* pointer to access indirect access buffer A */
#define INDIRECT_POINTER_B_ID       0x1E0000    /* pointer to access indirect access buffer B */

#define ACC_MEM_18B_ID              0x150000

#define TX_BUFFER_ID            0x140000        /* Transmit Data Buffer */
#define SCRATCH_RAM_ID          0x160000

#define CIA_I_RX_TIME_LEN       5
#define CIA_C_RX_TIME_LEN       5
#define CIA_I_STAT_OFFSET       3               /* RX status info for Ipatov sequence */
#define CIA_C_STAT_OFFSET       2               /* RX status info for ciphered sequence */
#define CIA_DIAGNOSTIC_OFF      (0x1 << 4)      /* bit4 in RX_ANTENNA_DELAY + 1 */

#define DW_CIA_DIAG_LOG_MAX (0x8)   ///< CIA to copy to swinging set a maximum set of diagnostic registers in Double Buffer mode
#define DW_CIA_DIAG_LOG_MID (0x4)   ///< CIA to copy to swinging set a medium set of diagnostic registers in Double Buffer mode
#define DW_CIA_DIAG_LOG_MIN (0x2)   ///< CIA to copy to swinging set a minimal set of diagnostic registers in Double Buffer mode
#define DW_CIA_DIAG_LOG_ALL (0x1)   ///< CIA to log all diagnostic registers
#define DW_CIA_DIAG_LOG_OFF (0x0)   ///< CIA to log reduced set of diagnostic registers

#define LDO_BIAS_KICK       (0x600) ///< LDO and BIAS tune kick, writing to bit 10 and 9

#define CP_ACC_CP_QUAL_SIGNTST  0x0800          /* sign test */
#define CP_ACC_CP_QUAL_SIGNEXT  0xF000          /* 12 bit to 16 bit sign extension */
#define CP_IV_LENGTH            16              /* CP initial value is 16 bytes or 128 bits*/

#define CP_KEY_LENGTH           16              /* CP AES key is 16 bytes or 128 bits*/

#define PMSC_TXFINESEQ_ENABLE   0x4d28874
#define PMSC_TXFINESEQ_DISABLE  0x0d20874

#define TXRXSWITCH_TX           0x01011100
#define TXRXSWITCH_AUTO         0x1C000000

#define SYS_STATE_TX_RX_IDLE_MASK      (SYS_STATE_LO_ACC_STATE_BIT_MASK|SYS_STATE_LO_RX_STATE_BIT_MASK|SYS_STATE_LO_TX_STATE_BIT_MASK)

#define EVC_EN                  0x00000001UL    /* Event Counters Enable bit */
#define EVC_CLR                 0x00000002UL
#define LOGGER_MEM_ID           0x180000

#define PANADR_PAN_ID_BYTE_OFFSET       2
#define PMSC_CTRL0_PLL2_SEQ_EN          0x01000000UL    /* Enable PLL2 on/off sequencing by SNIFF mode */
#define RX_BUFFER_MAX_LEN               (1023)
#define TX_BUFFER_MAX_LEN               (1024)
#define RX_FINFO_STD_RXFLEN_MASK        0x0000007FUL    /* Receive Frame Length (0 to 127) */
#define RX_TIME_RX_STAMP_LEN            (5)             /* read only 5 bytes (the adjusted timestamp (40:0)) */
#define TX_TIME_TX_STAMP_LEN            (5)             /* 40-bits = 5 bytes */
#define SCRATCH_BUFFER_MAX_LEN          (128)           /* AES scratch memory */
#define STD_FRAME_LEN                   (127)

// Defines for enable_clocks function
#define FORCE_CLK_SYS_TX        (1)
#define FORCE_CLK_AUTO          (5)
//SYSCLK
#define FORCE_SYSCLK_PLL        (2)
#define FORCE_SYSCLK_FOSCDIV4   (1)
#define FORCE_SYSCLK_FOSC       (3)
//RX and TX CLK
#define FORCE_CLK_PLL           (2)

#define TX_CHANGEABLE_DATA              (10)            /* Can change the length of TX data by this size */
#define MINIMAL_DATA_LENGTH             (11)            /* The TX length will be at least the buffer below */
#define SIMPLE_TX_DATA_SIZE             (MINIMAL_DATA_LENGTH+TX_CHANGEABLE_DATA)
#define TX_LENGTH_BYTE_POS              (2)             /* This byte index represent the total length of the TX data */

/* User defined RX timeouts (frame wait timeout and preamble detect timeout) mask. */
#define SYS_STATUS_ALL_RX_TO   (SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK)

/* All RX errors mask. */
#define SYS_STATUS_ALL_RX_ERR  (SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXSTO_BIT_MASK \
                                | SYS_STATUS_ARFE_BIT_MASK | SYS_STATUS_CIAERR_BIT_MASK | SYS_STATUS_LCSSERR_BIT_MASK)

/* All RX events after a correct packet reception mask. */
#define SYS_STATUS_ALL_RX_GOOD (SYS_STATUS_RXFR_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_RXPRD_BIT_MASK | \
        SYS_STATUS_RXSFDD_BIT_MASK | SYS_STATUS_RXPHD_BIT_MASK | SYS_STATUS_CIA_DONE_BIT_MASK)

/* All TX events mask. */
#define SYS_STATUS_ALL_TX      (SYS_STATUS_AAT_BIT_MASK | SYS_STATUS_TXFRB_BIT_MASK | SYS_STATUS_TXPRS_BIT_MASK | \
        SYS_STATUS_TXPHS_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK )

#define SYS_STATUS_RXOK         (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_CIA_DONE_BIT_MASK)
#define SYS_STATUS_LO_ALL       (0xFFFFFFFEUL)
#define SYS_STATUS_HI_SPI_ERR   (SYS_STATUS_HI_SPI_OVF_BIT_MASK | SYS_STATUS_HI_SPI_UNF_BIT_MASK | SYS_STATUS_HI_SPI_COLLISION_BIT_MASK)

#define MAX_OFFSET_ALLOWED      (0xFFF)
#define MIN_INDERECT_ADDR       (0x1000)

#define BUF0_FINFO                  0x180000    //part of min set
#define BUF0_LATEST_TOA0            0x180004    //part of min set (RX time ~ RX_TIME_O)
#define BUF0_LATEST_TOA1            0x180008    //part of min set
#define BUF0_CIA_DIAG0              0x18000C    //part of min set
#define BUF0_CIA_TDOA0              0x180010    //part of min set
#define BUF0_CIA_PDOA_TDOA1         0x180014    //part of min set
#define BUF0_DGC_or_PGF_index       0x180018    //part of min set (---)
#define BUF0_Ip_Nacc                0x18001C    //part of min set (Ip_DIAG12)
#define BUF0_ipTOA0                 0x180020    //part of mid set
#define BUF0_ipTOA1                 0x180024    //part of mid set
#define BUF0_cy0TOA0                0x180028    //part of mid set
#define BUF0_cy0TOA1                0x18002C    //part of mid set
#define BUF0_cy1TOA0                0x180030    //part of mid set
#define BUF0_cy1TOA1                0x180034    //part of mid set
#define BUF0_ciaDiag1               0x180038    //part of max set
#define BUF0_ipDiag0                0x18003C    //part of max set
#define BUF0_ipDiag1                0x180040    //part of max set
#define BUF0_ipDiag2                0x180044    //...
#define BUF0_ipDiag3                0x180048
#define BUF0_ipDiag4                0x18004C
#define BUF0_ipDiag5                0x180050
#define BUF0_ipDiag6                0x180054
#define BUF0_ipDiag7                0x180058
#define BUF0_ipDiag8                0x18005C
#define BUF0_ipDiag9                0x180060
#define BUF0_ipDiag10               0x180064
#define BUF0_ipDiag11               0x180068
#define BUF0_cyDiag0                0x18006C
#define BUF0_cyDiag1                0x180070
#define BUF0_cyDiag2                0x180074
#define BUF0_cyDiag3                0x180078
#define BUF0_cyDiag4                0x18007C
#define BUF0_cyDiag5                0x180080
#define BUF0_cyDiag6                0x180084
#define BUF0_cyDiag7                0x180088
#define BUF0_cyDiag8                0x18008C
#define BUF0_cyDiag9                0x180090
#define BUF0_cyDiag10               0x180094
#define BUF0_cyDiag11               0x180098
#define BUF0_cyDiag12               0x18009C
#define BUF0_cyDiag13               0x1800A0
#define BUF0_cyDiag14               0x1800A4
#define BUF0_cyDiag15               0x1800A8
#define BUF0_cyDiag16               0x1800AC
#define BUF0_cyDiag17               0x1800B0
#define BUF0_cy1Diag0               0x1800B4
#define BUF0_cy1Diag1               0x1800B8
#define BUF0_cy1Diag2               0x1800BC
#define BUF0_cy1Diag3               0x1800C0
#define BUF0_cy1Diag4               0x1800C4
#define BUF0_cy1Diag5               0x1800C8
#define BUF0_cy1Diag6               0x1800CC
#define BUF0_cy1Diag7               0x1800D0
#define BUF0_cy1Diag8               0x1800D4
#define BUF0_cy1Diag9               0x1800D8
#define BUF0_cy1Diag10              0x1800DC
#define BUF0_cy1Diag11              0x1800E0
#define BUF0_cy1Diag12              0x1800E4


#define BUF1_OFFSET                 0x0000E8    /* Added by Niklas */

#define BUF1_FINFO                  0x1800E8    //part of min set
#define BUF1_LATEST_TOA0            0x1800EC    //part of min set
#define BUF1_LATEST_TOA1            0x1800F0    //part of min set
#define BUF1_CIA_DIAG0              0x1800F4    //part of min set
#define BUF1_CIA_TDOA0              0x1800F8    //part of min set
#define BUF1_CIA_PDOA_TDOA1         0x1800FC    //part of min set
#define BUF1_DGC_or_PGF_index       0x180100    //part of min set
#define BUF1_Ip_Nacc                0x180104    //part of min set (Ip_DIAG12)
#define BUF1_ipTOA0                 0x180108    //part of mid set
#define BUF1_ipTOA1                 0x18010C    //part of mid set
#define BUF1_cy0TOA0                0x180110    //part of mid set
#define BUF1_cy0TOA1                0x180114    //part of mid set
#define BUF1_cy1TOA0                0x180118    //part of mid set
#define BUF1_cy1TOA1                0x18011C    //part of mid set
#define BUF1_ciaDiag1               0x180120    //part of max set
#define BUF1_ipDiag0                0x180124    //part of max set
#define BUF1_ipDiag1                0x180128    //part of max set
#define BUF1_ipDiag2                0x18012C    //...
#define BUF1_ipDiag3                0x180130
#define BUF1_ipDiag4                0x180134
#define BUF1_ipDiag5                0x180138
#define BUF1_ipDiag6                0x18013C
#define BUF1_ipDiag7                0x180140
#define BUF1_ipDiag8                0x180144
#define BUF1_ipDiag9                0x180148
#define BUF1_ipDiag10               0x18014C
#define BUF1_ipDiag11               0x180150
#define BUF1_cyDiag0                0x180154
#define BUF1_cyDiag1                0x180158
#define BUF1_cyDiag2                0x18015C
#define BUF1_cyDiag3                0x180160
#define BUF1_cyDiag4                0x180164
#define BUF1_cyDiag5                0x180168
#define BUF1_cyDiag6                0x18016C
#define BUF1_cyDiag7                0x180170
#define BUF1_cyDiag8                0x180174
#define BUF1_cyDiag9                0x180178
#define BUF1_cyDiag10               0x18017C
#define BUF1_cyDiag11               0x180180
#define BUF1_cyDiag12               0x180184
#define BUF1_cyDiag13               0x180188
#define BUF1_cyDiag14               0x18018C
#define BUF1_cyDiag15               0x180190
#define BUF1_cyDiag16               0x180194
#define BUF1_cyDiag17               0x180198
#define BUF1_cy1Diag0               0x18019C
#define BUF1_cy1Diag1               0x1801A0
#define BUF1_cy1Diag2               0x1801A4
#define BUF1_cy1Diag3               0x1801A8
#define BUF1_cy1Diag4               0x1801AC
#define BUF1_cy1Diag5               0x1801B0
#define BUF1_cy1Diag6               0x1801B4
#define BUF1_cy1Diag7               0x1801B8
#define BUF1_cy1Diag8               0x1801BC
#define BUF1_cy1Diag9               0x1801C0
#define BUF1_cy1Diag10              0x1801C4
#define BUF1_cy1Diag11              0x1801C8
#define BUF1_cy1Diag12              0x1801CC

/*
        Lookup table default values for channel 5
        Channel 5
        Gain                        SW      LNA1LNA2 MIXER  PGF
        NF + 0dB                    011100  000  01   1  111 101 10011 0
        MaxGain - 12dB, NF + 6dB    011100  010  00   1  111 101 10011 1
        MaxGain - 17dB, NF + 10dB   011100  011  00   1  111 101 10011 2
        MaxGain - 28dB, NF + 16dB   011100  011  11   1  111 101 10011 3
        MaxGain - 32dB, NF + 23dB   011100  111  10   1  111 101 10011 4
        MaxGain - 37dB, NF + 29dB   011100  111  11   1  111 101 10011 5
        MaxGain - 39dB, NF + 30dB   001111  111  11   1  111 101 10011 6
*/
typedef enum
{
        CH5_MAX_GAIN_NF = 0x0001c0fd,
        CH5_MAX_GAIN_MINUS_12DB_NF_PLUS_6DB = 0x0001c47d,
        CH5_MAX_GAIN_MINUS_17DB_NF_PLUS_10DB = 0x0001c67d,
        CH5_MAX_GAIN_MINUS_28DB_NF_PLUS_16DB = 0x0001c7fd,
        CH5_MAX_GAIN_MINUS_32DB_NF_PLUS_23DB = 0x0001cf7d,
        CH5_MAX_GAIN_MINUS_37DB_NF_PLUS_29DB = 0x0001cffd,
        CH5_MAX_GAIN_MINUS_39DB_NF_PLUS_30DB = 0x0000fffd
} dwt_configmrxlut_ch5_e;

/*
        Lookup table default values for channel 9
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
typedef enum
{
        CH9_MAX_GAIN_NF = 0x0002a07d,
        CH9_MAX_GAIN_MINUS_12DB_NF_PLUS_4DB = 0x0002a3fd,
        CH9_MAX_GAIN_MINUS_20DB_NF_PLUS_10DB = 0x0002a57d,
        CH9_MAX_GAIN_MINUS_24DB_NF_PLUS_16DB = 0x0002a77d,
        CH9_MAX_GAIN_MINUS_30DB_NF_PLUS_20DB = 0x0002a7fd,
        CH9_MAX_GAIN_MINUS_33DB_NF_PLUS_24DB = 0x0002ad7d,
        CH9_MAX_GAIN_MINUS_40DB_NF_PLUS_28DB = 0x0002affd
} dwt_configmrxlut_ch9_e;


//#define RX_FINFO_0_ID     0x180000//Same struct like RX_FINFO
//#define RX_FINFO_1_ID     0x1800E8//Same struct like RX_FINFO


#ifdef __cplusplus
}
#endif

#endif
