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

#ifndef _BSP_H_
#define _BSP_H_

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Define special stackos sections */
#define sec_data_core   __attribute__((section(".data.core")))
#define sec_bss_core    __attribute__((section(".bss.core")))
#define sec_bss_nz_core __attribute__((section(".bss.core.nz")))

/* More convenient section placement macros. */
#define bssnz_t         sec_bss_nz_core

extern uint8_t _ram_start;
#define RAM_SIZE        0x80000

#if MYNEWT_VAL(PN5180_ONB)
void
config_pn5180(void);
#endif

#if MYNEWT_VAL(M41T82_ONB)
int
config_m41t82_sensor(void);
#endif

#if MYNEWT_VAL(PAC1921_ONB)
int
config_pac1921_sensor(void);
#endif

#if MYNEWT_VAL(LIS2DT_ONB)
int
config_lis2dt_sensor(void);
#endif

const struct hal_flash *
hal_bsp_flash_dev(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* _BSP_H_ */
