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
 * @file dw3000_hal.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Hardware Abstraction Layer
 *
 * @details This is the hal base class which utilises functions to perform the necessary actions at hal.
 *
 */

#ifndef _DW3000_HAL_H_
#define _DW3000_HAL_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <dw3000-c0/dw3000_dev.h>
#include <dw3000-c0/dw3000_phy.h>

struct _dw3000_dev_instance_t * hal_dw3000_inst(uint8_t idx);     //!< Structure of hal instances.
void hal_dw3000_reset(struct _dw3000_dev_instance_t * inst);
int hal_dw3000_read(struct _dw3000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length);
int hal_dw3000_read_noblock(struct _dw3000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length);
int hal_dw3000_write(struct _dw3000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length);
int hal_dw3000_write_noblock(struct _dw3000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length);
int hal_dw3000_rw_noblock_wait(struct _dw3000_dev_instance_t * inst, dpl_time_t timeout);

int hal_dw3000_wakeup(struct _dw3000_dev_instance_t * inst);
int hal_dw3000_get_rst(struct _dw3000_dev_instance_t * inst);
void hal_dw3000_spi_txrx_cb(void *arg, int len);
#ifdef __cplusplus
}
#endif

#endif /* _DW3000_HAL_H_ */
