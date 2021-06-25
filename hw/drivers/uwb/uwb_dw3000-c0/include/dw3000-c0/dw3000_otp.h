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
 * @file dw3000_otp.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief one time programmable memory
 *
 * @details This is the otp base class which utilises functions to read from the address specified in the OTP_ADDR register.
 *
 */

#ifndef _DW3000_OTP_H_
#define _DW3000_OTP_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw3000-c0/dw3000_regs.h>
#include <dw3000-c0/dw3000_dev.h>

//! OTP addresses definitions
#define OTP_LDOTUNELO_ADDRESS (0x04)      //!< OTP address definition for LDO voltage tuning (lo reg)
#define OTP_LDOTUNEHI_ADDRESS (0x05)      //!< OTP address definition for LDO voltage tuning (hi reg)
#define OTP_PARTID_ADDRESS    (0x06)      //!< OTP address definition for PARTID
#define OTP_LOTID_ADDRESS     (0x07)      //!< OTP address definition for LOTID
#define OTP_VBAT_ADDRESS      (0x08)      //!< OTP address definition for voltage
#define OTP_VTEMP_ADDRESS     (0x09)      //!< OTP address definition for temperature
#define OTP_BIAS_TUNE_ADDRESS (0x0A)      //!< OTP address definition for bias tuning
#define OTP_XTRIM_ADDRESS     (0x1E)      //!< OTP address definition for crystal trim
#define OTP_OTPREV_ADDRESS    (0x1F)      //!< OTP address definition for otp revision

uint32_t _dw3000_otp_read(struct _dw3000_dev_instance_t * inst, uint16_t address);
void dw3000_opt_read(struct _dw3000_dev_instance_t * inst, uint32_t address, uint32_t * buffer, uint16_t length);
int dw3000_phy_otp_writeandverify(struct _dw3000_dev_instance_t * inst, uint32_t value, uint16_t address);

#ifdef __cplusplus
}
#endif

#endif /* _DW3000_OTP_H_ */
