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
 * @file dw3000_cli.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Command debug interface
 *
 * @details
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <dpl/dpl_cputime.h>
#include <hal/hal_gpio.h>
#include <dw3000-c0/dw3000_hal.h>
#include <dw3000-c0/dw3000_dev.h>
#include <dw3000-c0/dw3000_regs.h>
#include "dw3000_cli_priv.h"

#if MYNEWT_VAL(DW3000_CLI)

#include <shell/shell.h>
#include <console/console.h>
#include <console/ticks.h>
#if MYNEWT_VAL(UWB_CCP_ENABLED)
#include <uwb_ccp/uwb_ccp.h>
#endif
#if MYNEWT_VAL(UWB_RNG_ENABLED)
#include <uwb_rng/uwb_rng.h>
#endif

#ifdef __KERNEL__
int dw3000_sysfs_init(struct _dw3000_dev_instance_t *inst);
void dw3000_sysfs_deinit(int idx);
void dw3000_debugfs_init(void);
void dw3000_debugfs_deinit(void);
#endif

/* On slow consoles we insert a delay between lines to avoid corrupted output */
#if MYNEWT_VAL(CONSOLE_UART_BAUD < 1000000)
#define CONSOLE_DELAY(__X) dpl_time_delay(DPL_TICKS_PER_SEC/127)
#else
#define CONSOLE_DELAY(__X)
#endif


static int dw3000_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_dw3000_param[] = {
    {"dump", "[inst] dump all registers"},
    {"ev", "<inst> <on|reset|dump> event counters"},
    {"cw", "<inst> [1=on(def),0=off] tx CW on current channel"},
    {"arx", "<inst> [1=on(def),0=off] Auto-RX"},
    {"ctx", "<inst> <delay in ns,0=off> [frame len] Continuous TX"},
    {"da", "<inst> <addr> [length], dump area"},
    {"rd", "<inst> <addr> <subaddr> <length>, read register"},
    {"wr", "<inst> <addr> <subaddr> <value> <length>, write value to register"},
    {"gpio", "<dir|mode|read|write> <inst> [pin] [value]"},
    {"xtal", "<inst> [value]"},
    {"rssi", "[inst] rssi for last rx"},
#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
    {"ibt", "[instance [verbose-num]] interrupt backtrace"},
    {"status2txt", "<sys_status> to text"},
    {"fctrl2txt", "<fctrl> to text"},
#endif
#if MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)
    {"spibt", "[instance [verbose-num]] spi backtrace"},
#endif
#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN) && MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)
    {"bt", "[instance [verbose-num]] spi+irq backtrace"},
#endif
#if MYNEWT_VAL(DW3000_OTP_CLI)
    {"otp", "<inst> <read|dump|write|conf-wr> [addr] [value]"},
#endif
    {NULL,NULL},
};

const struct shell_cmd_help cmd_dw3000_help = {
        "dw3000 dbg", "dw3000 debug", cmd_dw3000_param
};
#endif

static struct shell_cmd shell_dw3000_cmd =
    SHELL_CMD_EXT("dw3000", dw3000_cli_cmd, &cmd_dw3000_help);

void
dw3000_cli_dump_registers(struct _dw3000_dev_instance_t * inst, struct streamer *streamer)
{
    uint64_t reg = 0;
    uint32_t i, m32 = 0xffffffffUL;
    const char fmt32[] = "{\"reg[%06lX]\"=\"0x%08llX\"}\n";
    const char fmt40[] = "{\"reg[%06lX]\"=\"0x%010llX\"}\n";

    /* Delays used below to not get corrupted print on slow
     * consoles */
    for(i=0; i<0x7C; i+=4) {
        switch (i) {
        case (RX_TIME_0_ID):
            reg = dw3000_read_reg(inst, i, 0, 5);
            streamer_printf(streamer, fmt40, i, reg&0xffffffffffll);
            break;
        default:
            reg = dw3000_read_reg(inst, i, 0, 4);
            streamer_printf(streamer, fmt32, i, reg&m32);
        }
        CONSOLE_DELAY();
    }
    for(i=0x10000; i<0x1006C; i+=4) {
        reg = dw3000_read_reg(inst, i, 0, 4);
        streamer_printf(streamer, fmt32, i, reg&m32);
        CONSOLE_DELAY();
    }

    for(i=0x60000; i<0x6001d; i+=4) {
        reg = dw3000_read_reg(inst, i, 0, 4);
        streamer_printf(streamer, fmt32, i, reg&m32);
        CONSOLE_DELAY();
    }

    for(i=0x70000; i<0x7005C; i+=4) {
        reg = dw3000_read_reg(inst, i, 0, 4);
        streamer_printf(streamer, fmt32, i, reg&m32);
        CONSOLE_DELAY();
    }

    for(i=0x90000; i<0x9001C; i+=4) {
        reg = dw3000_read_reg(inst, i, 0, 4);
        streamer_printf(streamer, fmt32, i, reg&m32);
        CONSOLE_DELAY();
    }

    reg = dw3000_read_reg(inst, SYS_STATE_LO_ID, 0, 4);
    streamer_printf(streamer, fmt32, (uint32_t)SYS_STATE_LO_ID, reg&m32);
    reg = dw3000_read_reg(inst, FCMD_STATUS_ID, 0, 4);
    streamer_printf(streamer, fmt32, (uint32_t)FCMD_STATUS_ID, reg&m32);
    reg = dw3000_read_reg(inst, CLK_CTRL_ID, 0, 4);
    streamer_printf(streamer, fmt32, (uint32_t)CLK_CTRL_ID, reg&m32);

    streamer_printf(streamer, "{\"inst->irq_sem\"=%d}\n", dpl_sem_get_count(&inst->uwb_dev.irq_sem));
    streamer_printf(streamer, "{\"inst->tx_sem\"=%d}\n", dpl_sem_get_count(&inst->tx_sem));
#if MYNEWT_VAL(UWB_RNG_ENABLED)
    {
        struct uwb_rng_instance *rng = (struct uwb_rng_instance*)uwb_mac_find_cb_inst_ptr(&inst->uwb_dev, UWBEXT_RNG);
        if (rng)
            streamer_printf(streamer, "{\"rng->sem\"=%d}\n", dpl_sem_get_count(&rng->sem));
    }
#endif

#if defined(MYNEWT)
#if MYNEWT_VAL(UWB_CCP_ENABLED)
    {
        struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(&inst->uwb_dev, UWBEXT_CCP);
        if (ccp)
            streamer_printf(streamer, "{\"ccp->sem\"=%d}\n", dpl_sem_get_count(&ccp->sem));
    }
#endif
#endif
}

void
dw3000_dump_address(struct _dw3000_dev_instance_t * inst, uint32_t addr, uint16_t length, struct streamer *streamer)
{
#define DUMP_STEP (16)
    int i, step = DUMP_STEP;
    uint8_t b[DUMP_STEP];
    uint32_t rda;

    dw3000_modify_reg(inst, CLK_CTRL_ID, 0, 0xffffffff,
                      CLK_CTRL_FORCE_CIA_CLKS_ON_BIT_MASK | CLK_CTRL_CIA_WR_CLK_EN_BIT_MASK,
                      sizeof(uint32_t));

    streamer_printf(streamer, "Dump starting at %06"PRIX32":\n", addr);
    for (i=0;i<length;i+=step) {
        memset(b,0,sizeof(b));
        if(i <= REG_DIRECT_OFFSET_MAX_LEN) {
            dw3000_read(inst, addr, i, b, step);
        } else {
            rda = addr + i;
            /* Program the indirect offset registers A for specified offset to RX buffer */
            dw3000_write_reg(inst, INDIRECT_ADDR_A_ID, 0, (rda >> 16), sizeof(uint32_t));
            dw3000_write_reg(inst, ADDR_OFFSET_A_ID, 0, rda&0xffff, sizeof(uint32_t));

            /* Indirectly read data from the IC to the buffer */
            dw3000_read(inst, INDIRECT_POINTER_A_ID, 0, b, step);
        }

        streamer_printf(streamer, "%04X: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
               i, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
               b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
    }
    dw3000_modify_reg(inst, CLK_CTRL_ID, 0,
                      ~((uint32_t)CLK_CTRL_FORCE_CIA_CLKS_ON_BIT_MASK | (uint32_t)CLK_CTRL_CIA_WR_CLK_EN_BIT_MASK), 0,
                      sizeof(uint32_t));
}

void
dw3000_cli_dump_event_counters(struct _dw3000_dev_instance_t * inst, struct streamer *streamer)
{
    struct uwb_dev_evcnt cnt;

    dw3000_phy_event_cnt_read(inst, &cnt);

    streamer_printf(streamer, "Event counters:\n");
    streamer_printf(streamer, "  RXPHE:  %6d  # rx PHR err, 12bit\n", cnt.ev0s.count_rxphe);
    streamer_printf(streamer, "  RXFSL:  %6d  # rx sync loss, 12bit\n", cnt.ev0s.count_rxfsl);
    streamer_printf(streamer, "  RXFCG:  %6d  # rx CRC OK, 12bit\n", cnt.ev1s.count_rxfcg);
    streamer_printf(streamer, "  RXFCE:  %6d  # rx CRC Errors, 12bit\n", cnt.ev1s.count_rxfce);
    streamer_printf(streamer, "  ARFE:   %6d  # addr filt err, 8bit\n", cnt.ev2s.count_arfe);
    streamer_printf(streamer, "  RXOVRR: %6d  # rx overflow, 8bit\n", cnt.ev2s.count_rxovrr);
    streamer_printf(streamer, "  RXSTO:  %6d  # rx SFD TO, 12bit\n", cnt.ev3s.count_rxsto);
    streamer_printf(streamer, "  RXPTO:  %6d  # pream search TO, 12bit\n", cnt.ev3s.count_rxpto);
    streamer_printf(streamer, "  FWTO:   %6d  # rx frame wait TO, 8bit\n", cnt.ev4s.count_fwto);
    streamer_printf(streamer, "  TXFRS:  %6d  # tx frames sent, 12bit\n", cnt.ev4s.count_txfrs);
    streamer_printf(streamer, "  HPWARN: %6d  # half period warn, 8bit\n", cnt.ev5s.count_hpwarn);
    streamer_printf(streamer, "  SPICRC: %6d  # spi CRC err, 8bit\n", cnt.ev5s.count_spicrc);
    streamer_printf(streamer, "  RXPREJ: %6d  # rx prem rejects, 12bit\n", cnt.ev6s.count_rxprej);
    streamer_printf(streamer, "  CPERR:  %6d  # STS qual errs, 8bit\n", cnt.ev7s.count_cperr);
    streamer_printf(streamer, "  VWARN:  %6d  # vwarn flag evs, 8bit\n", cnt.ev7s.count_vwarn);
}


#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
static char*
sys_status_to_string(uint64_t s)
{
    static char b[128];
    char *bp = b;
    memset(b,0,sizeof(b));
    if (s & 0x100000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "CCAfail|");
    if (s & 0x080000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "SpiCollision|");
    if (s & 0x040000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "SpiUnf|");
    if (s & 0x020000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "SpiOvf|");
    if (s & 0x010000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "CmdErr|");
    if (s & 0x008000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "AesDmaErr|");
    if (s & 0x004000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "AesDmaDone|");
    if (s & 0x002000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "GPIOIrq|");
    if (s & 0x001000000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "VorTVar|");
    if (s & 0x000800000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "ExtSync|");
    /* if (s & 0x000400000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "Unknown|"); */
    if (s & 0x000200000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPreambRej|");
    if (s & 0x000100000000ULL) bp += snprintf(bp,sizeof(b)-(bp-b), "RxReedSolCorr|");
    if (s & 0x80000000) bp += snprintf(bp,sizeof(b)-(bp-b), "Unknown|");
    if (s & 0x40000000) bp += snprintf(bp,sizeof(b)-(bp-b), "Unknown|");
    if (s & 0x20000000) bp += snprintf(bp,sizeof(b)-(bp-b), "AutFrameFiltRej|");
    if (s & 0x10000000) bp += snprintf(bp,sizeof(b)-(bp-b), "STSPreambErr|");
    if (s & 0x08000000) bp += snprintf(bp,sizeof(b)-(bp-b), "HalfPeriodDelayWarn|");
    if (s & 0x04000000) bp += snprintf(bp,sizeof(b)-(bp-b), "RXSFDTimeout|");
    if (s & 0x02000000) bp += snprintf(bp,sizeof(b)-(bp-b), "PLLHiLoWarn|");
    if (s & 0x01000000) bp += snprintf(bp,sizeof(b)-(bp-b), "RCInit|");
    if (s & 0x00800000) bp += snprintf(bp,sizeof(b)-(bp-b), "SPIRdy|");
    if (s & 0x00400000) bp += snprintf(bp,sizeof(b)-(bp-b), "LCSSErr|");
    if (s & 0x00200000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPreamTimeout|");
    if (s & 0x00100000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxOvErr|");
    if (s & 0x00080000) bp += snprintf(bp,sizeof(b)-(bp-b), "BrownOutWarn|");
    if (s & 0x00040000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxCIAerr|");
    if (s & 0x00020000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxTimeout|");
    if (s & 0x00010000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxRSFrameSyncLoss|");
    if (s & 0x00008000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxFCSErr|");
    if (s & 0x00004000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxFCSGood|");
    if (s & 0x00002000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxFrmRdy|");
    if (s & 0x00001000) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPHYErr|");
    if (s & 0x00000800) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPHYDet|");
    if (s & 0x00000400) bp += snprintf(bp,sizeof(b)-(bp-b), "RxCIAdone|");
    if (s & 0x00000200) bp += snprintf(bp,sizeof(b)-(bp-b), "RxSFDet|");
    if (s & 0x00000100) bp += snprintf(bp,sizeof(b)-(bp-b), "RxPreamDet|");
    if (s & 0x00000080) bp += snprintf(bp,sizeof(b)-(bp-b), "TxFrameSent|");
    if (s & 0x00000040) bp += snprintf(bp,sizeof(b)-(bp-b), "TxPHYDone|");
    if (s & 0x00000020) bp += snprintf(bp,sizeof(b)-(bp-b), "TxPreamDone|");
    if (s & 0x00000010) bp += snprintf(bp,sizeof(b)-(bp-b), "TxStart|");
    if (s & 0x00000008) bp += snprintf(bp,sizeof(b)-(bp-b), "AutoAck|");
    if (s & 0x00000004) bp += snprintf(bp,sizeof(b)-(bp-b), "SPICRCErr|");
    if (s & 0x00000002) bp += snprintf(bp,sizeof(b)-(bp-b), "Clock PLL Lock|");
    if (s & 0x00000001) bp += snprintf(bp,sizeof(b)-(bp-b), "IRS");
    return b;
}

static char*
fctrl_to_string(uint16_t s)
{
    static char b[40];
    char *bp = b;
    memset(b,0,sizeof(b));
    if ((s & 0x0007) == 0x0001) bp += snprintf(bp,sizeof(b)-(bp-b), "D|");  /* Data */
    if ((s & 0x0007) == 0x0002) bp += snprintf(bp,sizeof(b)-(bp-b), "A|");  /* Acknowledge */
    if ((s & 0x0007) == 0x0003) bp += snprintf(bp,sizeof(b)-(bp-b), "M|");  /* Reserved */
    if ((s & 0x0007) == 0x0004) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");
    if ((s & 0x0007) == 0x0005) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");
    if ((s & 0x0007) == 0x0006) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");
    if ((s & 0x0007) == 0x0007) bp += snprintf(bp,sizeof(b)-(bp-b), "R|");

    if ((s & 0x0008) == 0x0008) bp += snprintf(bp,sizeof(b)-(bp-b), "Secr|"); /* Security Enabeled */
    if ((s & 0x0010) == 0x0010) bp += snprintf(bp,sizeof(b)-(bp-b), "fPnd|"); /* Frame Pending */
    if ((s & 0x0020) == 0x0020) bp += snprintf(bp,sizeof(b)-(bp-b), "ACKr|"); /* Ack requested */
    if ((s & 0x0040) == 0x0040) bp += snprintf(bp,sizeof(b)-(bp-b), "PANc|"); /* PANID Compress */

    if ((s & 0x0C00) == 0x0000) bp += snprintf(bp,sizeof(b)-(bp-b), "DstNo|"); /* No destination address */
    if ((s & 0x0C00) == 0x0400) bp += snprintf(bp,sizeof(b)-(bp-b), "DstRs|"); /* Reserved */
    if ((s & 0x0C00) == 0x0800) bp += snprintf(bp,sizeof(b)-(bp-b), "Dst16|"); /* 16-bit destination address */
    if ((s & 0x0C00) == 0x0C00) bp += snprintf(bp,sizeof(b)-(bp-b), "Dst64|"); /* 64-bit destination address */

    if ((s & 0x3000) == 0x0000) bp += snprintf(bp,sizeof(b)-(bp-b), "I2003|");
    if ((s & 0x3000) == 0x1000) bp += snprintf(bp,sizeof(b)-(bp-b), "I|");
    if ((s & 0x3000) == 0x2000) bp += snprintf(bp,sizeof(b)-(bp-b), "iFv|"); /* Invalid frame version */
    if ((s & 0x3000) == 0x3000) bp += snprintf(bp,sizeof(b)-(bp-b), "iFv|");

    if ((s & 0xC000) == 0x0000) bp += snprintf(bp,sizeof(b)-(bp-b), "SrcNo"); /* No destination address */
    if ((s & 0xC000) == 0x4000) bp += snprintf(bp,sizeof(b)-(bp-b), "SrcRs"); /* Reserved */
    if ((s & 0xC000) == 0x8000) bp += snprintf(bp,sizeof(b)-(bp-b), "Src16"); /* 16-bit destination address */
    if ((s & 0xC000) == 0xC000) bp += snprintf(bp,sizeof(b)-(bp-b), "Src64"); /* 64-bit destination address */

    return b;
}

static void
fctrl_ledgend(struct streamer *streamer)
{
    streamer_printf(streamer, "   D=Data, A=Ack, M=Mac\n");
    streamer_printf(streamer, "   Secr: Security enabled, fPnd: Frame pending, ACKr: Ack requested, PANc: PANID Compress\n");
    streamer_printf(streamer, "   Dst: No=no dest addres, Rs=Reserved, 16-bit address, 64-bit address\n");
    streamer_printf(streamer, "   Frame version: I-IEEE 802.15.4, I2003-IEEE 802.15.4-2003, iFv-Invalid Frame Version\n");
    streamer_printf(streamer, "   Src: No=no src addres, Rs=Reserved, 16-bit address, 64-bit address\n");
}

static int
print_interrupt_bt_line(uint32_t *start_t, uint16_t verbose,
                        struct dw3000_sys_status_backtrace *p,
                        struct dw3000_sys_status_backtrace *p_last,
                        struct streamer *streamer)
{
    int32_t diff = (p_last)? p->utime - p_last->utime : 0;
    if (!p->utime) return 0;
    if (!*start_t) *start_t = p->utime;

    if (diff < 0) diff = 0;
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime));
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime-*start_t));
    streamer_printf(streamer, " %8lu ", dpl_cputime_ticks_to_usecs(diff));
    streamer_printf(streamer, " %6lu ", dpl_cputime_ticks_to_usecs(p->utime_end-p->utime));
    if (p->fctrl) {
        if (verbose&0x1) {
            char *s = fctrl_to_string(p->fctrl);
            streamer_printf(streamer, " %02X %02X (%s)%*s", p->fctrl&0xff, p->fctrl>>8, s, 32-strlen(s), " ");
        } else {
            streamer_printf(streamer, " %02X %02X ", p->fctrl&0xff, p->fctrl>>8);
        }
    } else {
        streamer_printf(streamer, "       ");
        if (verbose&0x1) {
            streamer_printf(streamer, " %32s ", "");
        }
    }
    streamer_printf(streamer, " %0*llX ", 2*DW3000_SYS_STATUS_ASSEMBLE_LEN, DW3000_SYS_STATUS_ASSEMBLE(p));
    streamer_printf(streamer, " %s", sys_status_to_string(DW3000_SYS_STATUS_ASSEMBLE(p)));
    return 1;
}

void
dw3000_cli_interrupt_backtrace(struct _dw3000_dev_instance_t * inst, uint16_t verbose, struct streamer *streamer)
{
    int i;
    uint32_t start_t = 0;
    struct dw3000_sys_status_backtrace *p, *p_last=0;
    streamer_printf(streamer, " %10s ", "abs");
    streamer_printf(streamer, " %10s ", "usec");
    streamer_printf(streamer, " %8s ", "diff");
    streamer_printf(streamer, " %6s ", "dur");
    streamer_printf(streamer, " %5s", "fctrl");
    if (verbose&0x1) {
        streamer_printf(streamer, "(fctrl2txt)%21s ", "");
    }
    streamer_printf(streamer, " %*s ", 2*DW3000_SYS_STATUS_ASSEMBLE_LEN, "status");
    streamer_printf(streamer, "   status2txt\n");
    for (i=0;i<80;i++) streamer_printf(streamer, "-");
    if (verbose&0x1) {
        for (i=0;i<34;i++) streamer_printf(streamer, "-");
    }
    streamer_printf(streamer, "\n");

    inst->sys_status_bt_lock = 1;
    for (i=0;i<MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN);i++) {
        uint16_t i_mod = (inst->sys_status_bt_idx + i + 1) % MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN);
        p = &inst->sys_status_bt[i_mod];
        if (print_interrupt_bt_line(&start_t, verbose, p, p_last, streamer)) {
            streamer_printf(streamer, "\n");
        }
        p_last = p;
    }
    inst->sys_status_bt_lock = 0;

    if (verbose&0x1) {
        streamer_printf(streamer, "----\n fctrl2txt: \n");
        fctrl_ledgend(streamer);
    }
}
#endif

#if MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)
static char*
cmd_to_string(uint8_t *cmd, uint8_t cmd_len)
{
    static char b[40];
    char *bp = b;
    memset(b,0,sizeof(b));
    if (cmd_len==1 && (cmd[0]&0x1)) {
        uint8_t fac = (0x7E&cmd[0])>>1;
        switch (fac) {
        case (CMD_DB_TOGGLE): bp += snprintf(bp,sizeof(b)-(bp-b), "DBTOGGLE");break;
        case (CMD_CLR_IRQS): bp += snprintf(bp,sizeof(b)-(bp-b), "CLRIRQS");break;
        case (CMD_TXRXOFF): bp += snprintf(bp,sizeof(b)-(bp-b), "TRXOFF");break;
        case (CMD_TX): bp += snprintf(bp,sizeof(b)-(bp-b), "TX");break;
        case (CMD_RX): bp += snprintf(bp,sizeof(b)-(bp-b), "RX");break;
        case (CMD_DRX): bp += snprintf(bp,sizeof(b)-(bp-b), "DRX");break;
        case (CMD_DTX): bp += snprintf(bp,sizeof(b)-(bp-b), "DTX");break;
        case (CMD_DTX_W4R): bp += snprintf(bp,sizeof(b)-(bp-b), "DTX_W4R");break;
        case (CMD_TX_W4R): bp += snprintf(bp,sizeof(b)-(bp-b), "TX_W4R");break;
        default: bp += snprintf(bp,sizeof(b)-(bp-b), "FAC(%x)", fac);
        }
        return b;
    }

    if ((cmd[0] & 0x80)==0 && (cmd[1]&0x3)==0) bp += snprintf(bp,sizeof(b)-(bp-b), "R");
    if ((cmd[0] & 0x80) && (cmd[1]&0x3)==0)    bp += snprintf(bp,sizeof(b)-(bp-b), "W");
    if ((cmd[0] & 0x80) && (cmd[1]&0x3)==1)    bp += snprintf(bp,sizeof(b)-(bp-b), "&or8");
    if ((cmd[0] & 0x80) && (cmd[1]&0x3)==2)    bp += snprintf(bp,sizeof(b)-(bp-b), "&or16");
    if ((cmd[0] & 0x80) && (cmd[1]&0x3)==3)    bp += snprintf(bp,sizeof(b)-(bp-b), "&or32");
    //if (cmd[0] & DW3000_SPI_EAMRW)

    {
        uint16_t addr = ((uint16_t)cmd[0] & 0x3F)<<8 | (cmd[1]&0xFC);
        uint16_t reg_file = 0x1F&(addr >> 9);
        uint16_t reg_offset = 0x7F&(addr >> 2);
        uint32_t regFileID = (uint32_t)reg_file << 16;
        uint16_t indx = reg_offset;
        bp += snprintf(bp,sizeof(b)-(bp-b), "@%"PRIX32":%x", regFileID, indx);
    }
    return b;
}

static int
print_spi_bt_line(uint32_t *start_t, uint16_t verbose,
                  struct dw3000_spi_backtrace *p,
                  struct dw3000_spi_backtrace *p_last,
                  struct streamer *streamer)
{
    int j;
    int32_t diff = (p_last)? p->utime - p_last->utime : 0;
    if (!p->utime) return 0;
    if (!*start_t) *start_t = p->utime;

    if (diff < 0) diff = 0;
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime));
    streamer_printf(streamer, " %10lu ", dpl_cputime_ticks_to_usecs(p->utime-*start_t));
    streamer_printf(streamer, " %8lu ", dpl_cputime_ticks_to_usecs(diff));
    streamer_printf(streamer, " %6lu ", dpl_cputime_ticks_to_usecs(p->utime_end-p->utime));
    streamer_printf(streamer, " %3s%s  ", (p->non_blocking)?"NB-":"", (p->is_write)?"W":"R");

    streamer_printf(streamer, " ");
    for (j=0;j<4;j++) {
        if (j<p->cmd_len) {
            streamer_printf(streamer, "%02X", p->cmd[j]);
        } else {
            streamer_printf(streamer, "  ");
        }
    }

    if (verbose&0x1) {
        char *s = cmd_to_string(p->cmd, p->cmd_len);
        streamer_printf(streamer, " (%s)%*s", s, 17-strlen(s), " ");
    }
    streamer_printf(streamer, " %4d ", p->data_len);
    for (j=0;j<p->data_len&&j<sizeof(p->data);j++) {
        streamer_printf(streamer, "%02X", p->data[j]);
    }
    return 1;
}

void
dw3000_cli_spi_backtrace(struct _dw3000_dev_instance_t * inst, uint16_t verbose,
                         struct streamer *streamer)
{
    int i;
    uint32_t start_t = 0;
    struct dw3000_spi_backtrace *p, *p_last=0;
    streamer_printf(streamer, " %10s ", "abs");
    streamer_printf(streamer, " %10s ", "usec");
    streamer_printf(streamer, " %8s ", "diff");
    streamer_printf(streamer, " %6s ", "dur");
    streamer_printf(streamer, " %5s", "flags");
    streamer_printf(streamer, " %s      ", "cmd");
    if (verbose&0x1) {
        streamer_printf(streamer, "(cmd2txt)%21s ", "");
    }
    streamer_printf(streamer, " %4s ", "dlen");
    streamer_printf(streamer, " %s\n", "data");
    for (i=0;i<80;i++) streamer_printf(streamer, "-");
    if (verbose&0x1) {
        for (i=0;i<34;i++) streamer_printf(streamer, "-");
    }
    streamer_printf(streamer, "\n");

    inst->spi_bt_lock = 1;
    for (i=0;i<MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN);i++) {
        uint16_t i_mod = (inst->spi_bt_idx + i + 1) % MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN);
        p = &inst->spi_bt[i_mod];
        if (print_spi_bt_line(&start_t, verbose, p, p_last, streamer)) {
            streamer_printf(streamer, "\n");
        }
        p_last = p;
    }
    inst->spi_bt_lock = 0;
}
#endif

#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN) && MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)

void
dw3000_cli_backtrace(struct _dw3000_dev_instance_t * inst, uint16_t verbose,
                     struct streamer *streamer)
{
    int i;
    uint16_t spi_i=0, irq_i=0;
    uint32_t start_t = 0;
    struct dw3000_spi_backtrace *spi_p, *spi_p_last=0;
    struct dw3000_sys_status_backtrace *irq_p, *irq_p_last=0;

    streamer_printf(streamer, " %10s ", "abs");
    streamer_printf(streamer, " %10s ", "usec");
    streamer_printf(streamer, " %8s ", "diff");
    streamer_printf(streamer, " %6s ", "dur");
    streamer_printf(streamer, " %5s", "flags");
    streamer_printf(streamer, " cmd/status data\n");
    for (i=0;i<80;i++) streamer_printf(streamer, "-");
    if (verbose&0x1) {
        for (i=0;i<34;i++) streamer_printf(streamer, "-");
    }
    streamer_printf(streamer, "\n");

    inst->spi_bt_lock = 1;
    inst->sys_status_bt_lock = 1;
    while (spi_i<MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN) || irq_i<MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)) {
        uint16_t spi_i_mod = (inst->spi_bt_idx + spi_i + 1) % MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN);
        uint16_t irq_i_mod = (inst->sys_status_bt_idx + irq_i + 1) % MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN);
        spi_p = &inst->spi_bt[spi_i_mod];
        irq_p = &inst->sys_status_bt[irq_i_mod];
        if ((spi_p->utime < irq_p->utime && spi_i < MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)) ||
            irq_i >= MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)) {
            if (print_spi_bt_line(&start_t, verbose, spi_p, spi_p_last, streamer)) {
                streamer_printf(streamer, "\n");
            }
            spi_p_last = spi_p;
            spi_i++;
        } else {
            streamer_printf(streamer, "\e[93m");   /* Set to bright yellow */
            if (print_interrupt_bt_line(&start_t, verbose, irq_p, irq_p_last, streamer)) {
                streamer_printf(streamer, "\e[39m\n");   /* Set to default colour */
            } else {
                streamer_printf(streamer, "\e[39m");
            }
            irq_p_last = irq_p;
            irq_i++;
        }
        CONSOLE_DELAY();
    }
    inst->spi_bt_lock = 0;
    inst->sys_status_bt_lock = 0;
}

#endif

#ifndef __KERNEL__
static void
dw3000_cli_too_few_args(struct streamer *streamer)
{
    streamer_printf(streamer, "Too few args\n");
}

static int
gpio_cmd(int argc, char **argv, struct streamer *streamer)
{
    /* dw3000 gpio <read|write> <inst> [pin] [[value]] */
    struct _dw3000_dev_instance_t * inst = 0;
    int8_t pin=-1, value=-1, inst_n = 0;
    if (argc < 3) {
        dw3000_cli_too_few_args(streamer);
        return 0;
    }

    if (argc > 4) {
        pin = strtol(argv[4], NULL, 0);
    }
    if (argc > 5) {
        value = strtol(argv[5], NULL, 0);
    }
    inst_n = strtol(argv[3], NULL, 0);
    inst = hal_dw3000_inst(inst_n);
    if (!inst) return -1;

    if (!strcmp(argv[2], "write")) {
        if (pin < 0 || pin > 8) {
            return -1;
        }
        dw3000_gpio_init_out(inst, pin, (value!=0));
        streamer_printf(streamer, "gpio(%d): out with val %d\n", pin, value);
    } else if (!strcmp(argv[2], "dir")) {
        if (pin < 0 || pin > 8) {
            return -1;
        } else {
            /* Invert value as 1=input, 0=output for dwX000 */
            dw3000_gpio_set_direction(inst, pin, (value==0));
            streamer_printf(streamer, "gpio(%d): %s\n", pin, (value==0)?"in":"out");
        }
    } else if (!strcmp(argv[2], "mode")) {
        if (pin < 0 || pin > 8) {
            return -1;
        }
        if (value > -1) {
            dw3000_gpio_set_mode(inst, pin, value);
        } else {
            value = dw3000_gpio_get_mode(inst, pin);
        }
        streamer_printf(streamer, "gpio(%d) mode: %x\n", pin, value);
    } else {
        streamer_printf(streamer, "gpio read pin:%d val:%d\n", pin, value);
        if (pin < 0 || pin > 8) {
            streamer_printf(streamer, "Raw gpio values: 0x%lx\n", dw3000_gpio_get_values(inst));
            return 0;
        } else {
            streamer_printf(streamer, "gpio(%d): %d\n", pin, dw3000_gpio_read(inst, pin));
        }
    }
    return 0;
}
#endif  /* ifndef __KERNEL__ */

int
dw3000_cli_cw(struct _dw3000_dev_instance_t *inst, int enable)
{
    if (enable) {
        /* Clear interrupt mask - so we don't get any unwanted events */
        dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, 0, sizeof(uint32_t));
        dw3000_configcwmode(inst, inst->uwb_dev.config.channel);
    } else {
        dw3000_softreset(inst);
        dw3000_mac_config(inst, NULL);
        dw3000_phy_config_txrf(inst, &inst->uwb_dev.config.txrf);
        /* Reenable interrupts */
        dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, inst->sys_status_mask, sizeof(uint32_t));
    }
    return 0;
}

int
dw3000_cli_autorx(struct _dw3000_dev_instance_t *inst, int enable)
{
    if (enable) {
        /* Clear interrupt mask - so we don't get any unwanted events */
        dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, 0, sizeof(uint32_t));
        /* Enable diagnostics */
        dw3000_configciadiag(inst, DW_CIA_DIAG_LOG_ALL|DW_CIA_DIAG_LOG_MAX);
        /* Disable timeout and enable auto-rx */
        dw3000_modify_reg(inst, SYS_CFG_ID, 0, ~((uint16_t)SYS_CFG_RXWTOE_BIT_MASK), SYS_CFG_RXAUTR_BIT_MASK, sizeof(uint16_t));
        dw3000_modify_reg(inst, TEST_CTRL0_ID, 0, 0xFFFFFFFFUL, TEST_CTRL0_RX_ALWAYS_EN_BIT_MASK, sizeof(uint32_t));
        dw3000_write_fast_CMD(inst, CMD_RX);
    } else {
        /* Stop rx */
        dw3000_write_fast_CMD(inst, CMD_TXRXOFF);
        /* Disable auto-rx and rx-always-on */
        dw3000_modify_reg(inst, SYS_CFG_ID, 0, ~((uint16_t)SYS_CFG_RXAUTR_BIT_MASK), 0, sizeof(uint16_t));
        dw3000_modify_reg(inst, TEST_CTRL0_ID, 0, ~((uint32_t)TEST_CTRL0_RX_ALWAYS_EN_BIT_MASK), 0, sizeof(uint32_t));
        /* Reenable interrupts */
        dw3000_write_fast_CMD(inst, CMD_CLR_IRQS);
        dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, inst->sys_status_mask, sizeof(uint32_t));
    }
    return 0;
}

int
dw3000_cli_continuous_tx(struct _dw3000_dev_instance_t *inst, int frame_length, uint32_t delay_ns)
{
    int i, offs;
    uint64_t delay_dtu;
    uint8_t buf[] = {0xC5, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    if (delay_ns) {
        /* Clear interrupt mask - so we don't get any unwanted events */
        dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, 0, sizeof(uint32_t));

        /* Prepare data to send */
        offs = 0;
        for (i=0;i<frame_length;i+=sizeof(buf)) {
            dw3000_write_tx(inst, buf, offs, sizeof(buf));
            offs += sizeof(buf);
        }
        /* Set length and offset of frame for tranceiver to use,
         * - 2 due to CRC added automatically */
        dw3000_write_tx_fctrl(inst, frame_length > 0 ? frame_length-2 : 0, 0, 0);

        /* Recalculate delay into dtu: 1000 ns = 65535 dtu */
        /* 65535 * 1024 / 1000 = 67107 - saves a 64bit division in kernel */
        delay_dtu = ((uint64_t)delay_ns * 67107)>>10;

        /* If delay_ns is so small that it would result in a zero delay_dtu use
         * the frame duration. Note: A value shorter than the frame duration will
         * result in frames being sent back to back anyway. */
        if (!delay_dtu) {
            delay_dtu = 65535*uwb_usecs_to_dwt_usecs(dw3000_phy_frame_duration(&inst->uwb_dev.attrib, frame_length));
            printf("%s:%d Using auto tx delay %"PRIu64" dtu\n", __func__, __LINE__, delay_dtu);
        }
        printf("%s:%d tx delay %"PRIu64" dtu, %"PRIu64" uwb-usec\n", __func__, __LINE__,
               delay_dtu, delay_dtu>>16);
        dw3000_phy_repeated_frames(inst, delay_dtu);

    } else {
        dw3000_write_fast_CMD(inst, CMD_TXRXOFF);
        dw3000_write_fast_CMD(inst, CMD_CLR_IRQS);
        dw3000_phy_repeated_frames(inst, 0);
        /* Reenable interrupts */
        dw3000_write_reg(inst, SYS_ENABLE_LO_ID, 0, inst->sys_status_mask, sizeof(uint32_t));
    }
    return 0;
}

static int
dw3000_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer)
{
#ifndef __KERNEL__
    struct _dw3000_dev_instance_t * inst = 0;
    uint16_t inst_n = 0;

    if (argc < 2) {
        dw3000_cli_too_few_args(streamer);
        return 0;
    }

    if (!strcmp(argv[1], "dump")) {
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        dw3000_cli_dump_registers(inst, streamer);
    } else if (!strcmp(argv[1], "ev")) {
        if (argc<4) {
            dw3000_cli_too_few_args(streamer);
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        inst = hal_dw3000_inst(inst_n);
        if (!strcmp(argv[3], "on")) {
            dw3000_phy_event_cnt_ctrl(inst, true, true);
            streamer_printf(streamer, "ev on\n");
        } else if (!strcmp(argv[3], "reset")) {
            dw3000_phy_event_cnt_ctrl(inst, false, true);
            streamer_printf(streamer, "ev reset\n");
        } else if (!strcmp(argv[3], "dump")) {
            console_no_ticks();
            dw3000_cli_dump_event_counters(inst, streamer);
            console_yes_ticks();
        }
    } else if (!strcmp(argv[1], "da")) {
        if (argc<3) {
            dw3000_cli_too_few_args(streamer);
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        uint32_t addr = strtol(argv[3], NULL, 0);
        int length = 128;
        if (argc>4) {
            length = strtol(argv[4], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        dw3000_dump_address(inst, addr, length, streamer);
    } else if (!strcmp(argv[1], "cw")) {
        int onoff = 1;
        if (argc<3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc>3) {
            onoff = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        dw3000_cli_cw(inst, onoff);
        if (onoff) {
            streamer_printf(streamer, "Device[%d] now in CW mode on ch %d.\n",
                            inst_n, inst->uwb_dev.config.channel);
        } else {
            streamer_printf(streamer, "CW disabled\n");
        }
    } else if (!strcmp(argv[1], "arx")) {
        int onoff = 1;
        if (argc<3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc>3) {
            onoff = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        dw3000_cli_autorx(inst, onoff);
        if (onoff) {
            streamer_printf(streamer, "Device is now in Auto-RX\n");
        } else {
            streamer_printf(streamer, "Auto-RX disabled\n");
        }
    } else if (!strcmp(argv[1], "ctx")) {
        int frame_len = 10;
        uint32_t delay_ns = 0;
        if (argc<3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc>3) {
            delay_ns = strtol(argv[3], NULL, 0);
        }
        if (argc>4) {
            frame_len = strtol(argv[4], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        dw3000_cli_continuous_tx(inst, frame_len, delay_ns);
        if (delay_ns) {
            streamer_printf(streamer, "Continuous-TX: delay=%ldns len=%d\n",
                            delay_ns, frame_len);
        } else {
            streamer_printf(streamer, "Continuous-TX disabled\n");
        }
    } else if (!strcmp(argv[1], "wr")) {
        if (argc < 7) {
            dw3000_cli_too_few_args(streamer);
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        uint32_t addr = strtol(argv[3], NULL, 0);
        uint16_t sub  = strtol(argv[4], NULL, 0);
        uint64_t val  = strtol(argv[5], NULL, 0);
        int length = strtol(argv[6], NULL, 0);
        dw3000_write_reg(hal_dw3000_inst(inst_n), addr, sub, val, length);
        streamer_printf(streamer, "0x%06lX,0x%X w: 0x%llX\n", addr, sub, val);
    } else if (!strcmp(argv[1], "rd")) {
        if (argc < 6) {
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        uint32_t addr = strtol(argv[3], NULL, 0);
        uint16_t sub  = strtol(argv[4], NULL, 0);
        int length = strtol(argv[5], NULL, 0);
        uint64_t reg = dw3000_read_reg(hal_dw3000_inst(inst_n), addr, sub, length);
        streamer_printf(streamer, "0x%06lX,0x%04X: 0x%llX\n", addr, sub, reg);
    } else if (!strcmp(argv[1], "gpio")) {
        gpio_cmd(argc, argv, streamer);
    } else if (!strcmp(argv[1], "xtal")) {
        int8_t trim;
        if (argc > 2) inst_n = strtol(argv[2], NULL, 0);
        inst = hal_dw3000_inst(inst_n);
        if (argc < 4) {
            streamer_printf(streamer, "xtal trim: %d\n",
                           (int8_t)dw3000_read_reg(inst,
                                                   XTAL_ID, 0, sizeof(uint8_t)));
            return 0;
        }
        trim = strtol(argv[3], NULL, 0);
        dw3000_set_xtal_trim(inst, trim);
    } else if (!strcmp(argv[1], "rssi")) {
        dpl_float32_t rssi;
        if (argc > 2) inst_n = strtol(argv[2], NULL, 0);
        inst = hal_dw3000_inst(inst_n);
        dw3000_read_rxdiag(inst, &inst->rxdiag, inst->rxdiag.rxd.enabled|UWB_RXDIAG_IPATOV);
        rssi = dw3000_get_rssi(inst);
        streamer_printf(streamer, "Last rx rssi: "DPL_FLOAT32_PRINTF_PRIM" dBm\n",
                        DPL_FLOAT32_PRINTF_VALS(rssi));
#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN)
    } else if (!strcmp(argv[1], "ibt")){
        uint8_t d=0;
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc < 4) {
            d=0;
        } else {
            d = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        console_no_ticks();
        dw3000_cli_interrupt_backtrace(inst, d, streamer);
        console_yes_ticks();
    } else if (!strcmp(argv[1], "status2txt")){
        uint64_t d = strtoll(argv[2], NULL, 0);
        streamer_printf(streamer, "%010llX: %s\n", d, sys_status_to_string(d));
    } else if (!strcmp(argv[1], "fctrl2txt")){
        uint8_t d=0,d2=0;
        if (argc < 4) {
            streamer_printf(streamer, "2 bytes needed\n");
            return 0;
        } else {
            d = strtol(argv[2], NULL, 16);
            d2 = strtol(argv[3], NULL, 16);
        }
        streamer_printf(streamer, "%02X %02X: %s\n", (uint8_t)d, (uint8_t)d2, fctrl_to_string((d2<<8)|d));
        streamer_printf(streamer, "----\n ledgend: \n");
        fctrl_ledgend(streamer);
#endif
#if MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)
    } else if (!strcmp(argv[1], "spibt")){
        uint8_t d=0;
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc < 4) {
            d=0;
        } else {
            d = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        console_no_ticks();
        dw3000_cli_spi_backtrace(inst, d, streamer);
        console_yes_ticks();
#endif
#if MYNEWT_VAL(DW3000_SYS_STATUS_BACKTRACE_LEN) && MYNEWT_VAL(DW3000_SPI_BACKTRACE_LEN)
    } else if (!strcmp(argv[1], "bt")){
        uint8_t d=0;
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        if (argc < 4) {
            d=0;
        } else {
            d = strtol(argv[3], NULL, 0);
        }
        inst = hal_dw3000_inst(inst_n);
        console_no_ticks();
        dw3000_cli_backtrace(inst, d, streamer);
        console_yes_ticks();
#endif
#if MYNEWT_VAL(DW3000_OTP_CLI)
    } else if (!strcmp(argv[1], "otp")) {
        if (argc < 4) {
            dw3000_cli_too_few_args(streamer);
            return 0;
        }
        inst_n = strtol(argv[2], NULL, 0);
        inst = hal_dw3000_inst(inst_n);
        dw3000_cli_otp(inst, argc, argv, streamer);
#endif
    } else {
        streamer_printf(streamer, "Unknown cmd\n");
    }
#endif  /* ifndef __KERNEL__ */

    return 0;
}

#endif


int
dw3000_cli_register(void)
{
#if MYNEWT_VAL(DW3000_CLI)
    int rc;
    rc = shell_cmd_register(&shell_dw3000_cmd);
#ifdef __KERNEL__
    {
        int i;
        struct _dw3000_dev_instance_t *inst;
        for (i=0;i<MYNEWT_VAL(UWB_DEVICE_MAX);i++) {
            inst = hal_dw3000_inst(i);
            if (!inst) continue;
            if (!inst->uwb_dev.status.initialized) continue;
            dw3000_sysfs_init(inst);
        }
        dw3000_debugfs_init();
    }
#endif
    return rc;
#else
    return 0;
#endif
}

int
dw3000_cli_down(int reason)
{
#ifdef __KERNEL__
    int i;
    struct _dw3000_dev_instance_t *inst;
    for (i=0;i<MYNEWT_VAL(UWB_DEVICE_MAX);i++) {
        inst = hal_dw3000_inst(i);
        if (!inst) continue;
        dw3000_sysfs_deinit(i);
    }
    dw3000_debugfs_deinit();
#endif
    return 0;
}
