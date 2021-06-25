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

#include <dw3000-c0/dw3000_otp.h>
#include <dw3000-c0/dw3000_hal.h>
#include <dpl/dpl_cputime.h>
#include "dw3000_cli_priv.h"

#if MYNEWT_VAL(DW3000_CLI) && MYNEWT_VAL(DW3000_OTP_CLI)

static uint16_t wr_address = 0xffff;
static uint32_t wr_value = 0xffffffff;

void
dw3000_cli_otp(struct _dw3000_dev_instance_t *inst, int argc, char **argv, struct streamer *streamer)
{
    int rc = -1;
    uint16_t address;
    uint32_t value;
    if (!strcmp(argv[3], "read")) {
        if (argc < 5) {
            streamer_printf(streamer, "OTP read: please provide address\n");
            return;
        }
        address = strtol(argv[4], NULL, 0);
        if (address > 0x7F) {
            streamer_printf(streamer, "OTP read: max address is 0x7f\n");
            return;
        }
        value = _dw3000_otp_read(inst, address);
        streamer_printf(streamer, "OTP[0x%02X]: 0x%08lX\n", address, value);
    } else if(!strcmp(argv[3], "dump")) {
        for (address = 0;address < 0x80; address++) {
            value = _dw3000_otp_read(inst, address);
            streamer_printf(streamer, "OTP[0x%02X]: 0x%08lX\n", address, value);
        }
    } else if (!strcmp(argv[3], "write")) {
        if (argc < 6) {
            streamer_printf(streamer, "OTP write: please provide address and value\n");
            return;
        }
        address = strtol(argv[4], NULL, 0);
        if (address > 0x7F) {
            streamer_printf(streamer, "OTP write: max address is 0x7f\n");
            return;
        }
        value = strtol(argv[5], NULL, 0);
        streamer_printf(streamer, "Preapared to write 0x%04lX to OTP[0x%02X]\n", value, address);
        streamer_printf(streamer, "To complete write, issue a conf-wr operation with the same values\n");
        wr_address = address;
        wr_value = value;
    } else if (!strcmp(argv[3], "conf-wr")) {
        if (argc < 6) {
            streamer_printf(streamer, "OTP write: please provide address and value\n");
            return;
        }
        address = strtol(argv[4], NULL, 0);
        value = strtol(argv[5], NULL, 0);
        if (wr_address != address || wr_value != value) {
            streamer_printf(streamer, "OTP Write abort, address and value must match prev write op (now: 0x%04lX to OTP[0x%02X])\n",
                            value, address);
            wr_address = 0xffff;
            wr_value = 0xffffffffUL;
            return;
        }
        if (wr_address > 0x7F) {
            streamer_printf(streamer, "OTP write: max address is 0x7f\n");
            wr_address = 0xffff;
            wr_value = 0xffffffffUL;
            return;
        }
        streamer_printf(streamer, "Writing 0x%04lX to OTP[0x%02X]\n", wr_value, wr_address);
        rc = dw3000_phy_otp_writeandverify(inst, wr_value, wr_address);
        streamer_printf(streamer, "Write completed with return value %d:%s\n", rc, rc==DPL_OK?"OK":"ERR");
    }
}

#endif
