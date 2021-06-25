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
 * @file dw3000_gpio.h
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief General Purpose Input Output
 *
 * @details This is the gpio base class which utilises functions to enable/disable all the configurations related to GPIO.
 */

#ifndef _DW3000_GPIO_H_
#define _DW3000_GPIO_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw3000-c0/dw3000_regs.h>
#include <dw3000-c0/dw3000_dev.h>

//! Defined constants for "mode" bit field parameter passed to dwt_set_leds() function.
typedef enum _dw3000_led_modes_t{
    DWT_LEDS_DISABLE = 1 << 0,   //!< Set for disabling LEDS
    DWT_LEDS_ENABLE = 1 << 1,    //!< Set for enabling LEDS
    DWT_LEDS_INIT_BLINK = 1 << 2 //!< Set for initiation blink
}dw3000_led_modes_t;

//! Default blink time. Blink time is expressed in multiples of 14 ms.
//  The value defined here is ~225 ms.
#define DWT_LEDS_BLINK_TIME_DEF 0x10

#define GPIO_PIN2_RXLED         0x00000040UL    /* The pin operates as the RXLED output */
#define GPIO_PIN3_TXLED         0x00000200UL    /* The pin operates as the TXLED output */

void dw3000_gpio_set_mode(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum, uint8_t mode);
uint8_t dw3000_gpio_get_mode(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum);
void dw3000_gpio0_config_ext_txe(struct _dw3000_dev_instance_t * inst);
void dw3000_gpio1_config_ext_rxe(struct _dw3000_dev_instance_t * inst);
void dw3000_gpio4_config_ext_pa(struct _dw3000_dev_instance_t * inst);

void dw3000_gpio_config_leds(struct _dw3000_dev_instance_t * inst, dw3000_led_modes_t mode);
void dw3000_gpio_set_value(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum, uint8_t value);
void dw3000_gpio_set_direction(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum, uint8_t direction);
uint8_t dw3000_gpio_get_direction(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum);
uint32_t dw3000_gpio_get_values(struct _dw3000_dev_instance_t * inst);

void dw3000_gpio_init_out(struct _dw3000_dev_instance_t * inst, int gpioNum, int val);
void dw3000_gpio_init_in(struct _dw3000_dev_instance_t * inst, int gpioNum);
int dw3000_gpio_read(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum);
void dw3000_gpio_write(struct _dw3000_dev_instance_t * inst, int gpioNum, int val);

#ifdef __cplusplus
}
#endif

#endif /* _DW3000_GPIO_H_ */
