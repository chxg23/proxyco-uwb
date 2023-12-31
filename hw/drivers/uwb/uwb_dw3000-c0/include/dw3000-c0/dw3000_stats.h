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
 * @file dw3000_stats.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 10/24/18
 * @brief ftypes file
 *
 * @details
 */

#ifndef _DW3000_STATS_H_
#define _DW3000_STATS_H_

#include <stdlib.h>
#include <stdint.h>
#include <stats/stats.h>

#ifdef __cplusplus
extern "C" {
#endif

#if MYNEWT_VAL(DW3000_MAC_STATS)
STATS_SECT_START(dw3000_mac_stat_section)
    STATS_SECT_ENTRY(tx_bytes)
    STATS_SECT_ENTRY(rx_bytes)
    STATS_SECT_ENTRY(DFR_cnt)
    STATS_SECT_ENTRY(RTO_cnt)
    STATS_SECT_ENTRY(ROV_err)
    STATS_SECT_ENTRY(TFG_cnt)
    STATS_SECT_ENTRY(LDE_err)
    STATS_SECT_ENTRY(RX_err)
    STATS_SECT_ENTRY(TXBUF_err)
    STATS_SECT_ENTRY(FCMD_err)
    STATS_SECT_ENTRY(SPI_err)
    STATS_SECT_ENTRY(STSPream_err)
    STATS_SECT_ENTRY(PLL_LL_err)
    STATS_SECT_ENTRY(empty_irq)
STATS_SECT_END
#endif

#ifdef __cplusplus
}
#endif
#endif /* _DW3000_STATS_H_ */
