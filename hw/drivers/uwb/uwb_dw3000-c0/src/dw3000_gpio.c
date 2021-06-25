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
 * @file dw3000_gpio.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief General Purpose Input Output
 *
 * @details This is the gpio base class which utilises functions to enable/disable all the configurations related to GPIO.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw3000-c0/dw3000_gpio.h>

/**
 * Set the mode on a gpio
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to configure (0-8)
 * @param mode          See manual for available modes for different pins
 *
 * @return void
 */
void
dw3000_gpio_set_mode(struct _dw3000_dev_instance_t * inst,
                     uint8_t gpioNum, uint8_t mode)
{
    dw3000_modify_reg(inst, MFIO_MODE_ID, 0,
                      ~(MFIO_MODE_MFIO0_MODE_BIT_MASK << (
                            MFIO_MODE_MFIO1_MODE_BIT_OFFSET*gpioNum)),
                      (uint32_t)(mode & MFIO_MODE_MFIO0_MODE_BIT_MASK) << (
                            MFIO_MODE_MFIO1_MODE_BIT_OFFSET*gpioNum),
                      sizeof(uint32_t));
}

/**
 * Reads the current mode on a gpio
 *
 * @param inst          pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to configure (0-8)
 *
 * @return void
 */
uint8_t
dw3000_gpio_get_mode(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum)
{
    uint32_t reg32 = dw3000_read_reg(inst, MFIO_MODE_ID, 0, sizeof(uint32_t));
    reg32 &= ((uint32_t)MFIO_MODE_MFIO0_MODE_BIT_MASK) << (MFIO_MODE_MFIO1_MODE_BIT_OFFSET*gpioNum);
    reg32 >>= (MFIO_MODE_MFIO1_MODE_BIT_OFFSET*gpioNum);
    return (uint8_t)(reg32 & MFIO_MODE_MFIO0_MODE_BIT_MASK);
}

/**
 * API to set up gpio 4/5/6 as pa/lna mode
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 *
 * @return void
 */
#if 0
void dw3000_gpio4_config_ext_pa(struct _dw3000_dev_instance_t * inst)
{
    uint32_t reg;
    /* PA mode requires fine grain tx sequencing */
    dw3000_phy_setfinegraintxseq(inst, 1);
    reg = (uint32_t) dw3000_read_reg(inst, GPIO_CTRL_ID, GPIO_MODE_OFFSET, sizeof(uint32_t));
    reg &= ~GPIO_MSGP4_MASK;
    reg |= GPIO_PIN4_EXTPA;
    dw3000_write_reg(inst, GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg, sizeof(uint32_t));
}
#endif

void dw3000_gpio0_config_ext_txe(struct _dw3000_dev_instance_t * inst)
{
    dw3000_gpio_set_mode(inst, 0, 0x2);
}

void dw3000_gpio1_config_ext_rxe(struct _dw3000_dev_instance_t * inst)
{
    dw3000_gpio_set_mode(inst, 1, 0x2);
}

/**
 * API to set up Tx/Rx GPIOs which could be used to control LEDs.
 * Note: not completely IC dependent, also needs board with LEDS fitted on right I/O lines
 *       this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000.
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 * @param mode  This is a bit field interpreted as follows:
 *          - bit 0: 1 to enable LEDs, 0 to disable them
 *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
 *          - bit 2 to 7: reserved
 *
 * @return void
 */
void dw3000_gpio_config_leds(struct _dw3000_dev_instance_t * inst, dw3000_led_modes_t mode)
{
    uint32_t reg;

    if (mode & DWT_LEDS_ENABLE){
        // Set up MFIO for LED output.
        dw3000_gpio_set_mode(inst, 2, 0x1);
        dw3000_gpio_set_mode(inst, 3, 0x1);

        // Enable LP Oscillator to run from counter and turn on de-bounce clock.
        dw3000_modify_reg(inst, CLK_CTRL_ID, 0, 0xFFFFFFFF,
                          (CLK_CTRL_GPIO_DBNC_CLK_EN_BIT_MASK | CLK_CTRL_LP_CLK_EN_BIT_MASK),
                          sizeof(uint32_t));

        // Enable LEDs to blink and set default blink time.
        reg = LED_CTRL_BLINK_EN_BIT_MASK | DWT_LEDS_BLINK_TIME_DEF;

        if (mode & DWT_LEDS_INIT_BLINK){
            // Single blink sign-of-life.
            reg |= LED_CTRL_FORCE_TRIGGER_BIT_MASK;
        }
        dw3000_write_reg(inst, LED_CTRL_ID, 0, reg, sizeof(uint32_t));

        if (mode & DWT_LEDS_INIT_BLINK){
            // Single blink sign-of-life.
            reg &= ~LED_CTRL_FORCE_TRIGGER_BIT_MASK;
            dw3000_write_reg(inst, LED_CTRL_ID, 0, reg, sizeof(uint32_t));
        }
    }else{
        // Clear the GPIO bits that are used for LED control.
        dw3000_modify_reg(inst, MFIO_MODE_ID, 0,
                          ~(MFIO_MODE_MFIO3_MODE_BIT_MASK | MFIO_MODE_MFIO2_MODE_BIT_MASK),
                          0, sizeof(uint32_t));
    }
}

/**
 * API to set GPIO direction as an input (1) or output (0).
 *
 * @param inst          pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to configure (0-8)
 * @param direction     This sets the GPIO direction input (1) or output (0)
 *
 * @return void
 */
void
dw3000_gpio_set_direction(struct _dw3000_dev_instance_t * inst,
                          uint8_t gpioNum, uint8_t direction)
{
    assert(gpioNum < 9);

    /* Activate GPIO Clock if not already active */
    dw3000_modify_reg(inst, CLK_CTRL_ID, 0, 0xffffffffUL,
                      CLK_CTRL_GPIO_CLK_EN_BIT_MASK, sizeof(uint32_t));

    if (direction) {
        /* 1 = Input */
        dw3000_modify_reg(inst, GPIO_DIR_ID, 0, 0xffffffffUL,
                          0x1UL << gpioNum, sizeof(uint32_t));
    } else {
        /* 0 = output */
        dw3000_modify_reg(inst, GPIO_DIR_ID, 0, ~(0x1UL << gpioNum),
                          0, sizeof(uint32_t));
    }
}

/**
 * Read the GPIO direction of a given gpio
 *
 * @param inst        pointer to _dw3000_dev_instance_t.
 * @param gpioNum     This is the GPIO to configure (0-8)
 *
 * @return uint8_t    direction set to pin, input (1) or output (0)
 */
uint8_t
dw3000_gpio_get_direction(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum)
{
    uint32_t reg32;
    uint8_t direction;
    assert(gpioNum < 9);

    reg32 = dw3000_read_reg(inst, GPIO_DIR_ID, 0, sizeof(uint32_t));
    reg32 &= (0x1UL << gpioNum);
    reg32 >>= gpioNum;
    direction = (uint8_t)(reg32 & 0x1);

    return direction;
}

/**
 * API to set GPIO value as (1) or (0) only applies if the GPIO is configured as output.
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to configure (0-8)
 * @param value         This sets the GPIO HIGH (1) or LOW (0)
 *
 * @return void
 */
void
dw3000_gpio_set_value(struct _dw3000_dev_instance_t * inst,
                      uint8_t gpioNum, uint8_t value)
{
    assert(gpioNum < 9);
    if (value) {
        dw3000_modify_reg(inst, GPIO_OUT_ID, 0, 0xffffffffUL,
                          0x1UL << gpioNum, sizeof(uint32_t));
    } else {
        dw3000_modify_reg(inst, GPIO_OUT_ID, 0, ~(0x1UL << gpioNum),
                          0, sizeof(uint32_t));
    }
}

/**
 * API to get GPIO value from an GPIO pin configured as input
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 *
 * @return uint32_t     The raw state on the pins currently
 */
uint32_t
dw3000_gpio_get_values(struct _dw3000_dev_instance_t * inst)
{
    uint32_t reg;
    reg = (uint32_t) dw3000_read_reg(inst, GPIO_RAW_DATA_ID, 0,
                                     sizeof(uint32_t));
    return (reg&GPIO_RAW_DATA_MASK);
}

/**
 * API that matches Mynewt's hal_gpio_init_out
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to configure (0-7)
 * @param val           This sets the GPIO HIGH (1) or LOW (0)
 *
 * @return void
 */
void
dw3000_gpio_init_out(struct _dw3000_dev_instance_t * inst, int gpioNum, int val)
{
    dw3000_gpio_set_direction(inst, gpioNum, 0);
    dw3000_gpio_set_value(inst, gpioNum, val);
}

/**
 * API that matches Mynewt's hal_gpio_init_in
 *
 * @param inst          pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to configure (0-7)
 *
 * @return void
 */
void
dw3000_gpio_init_in(struct _dw3000_dev_instance_t * inst, int gpioNum)
{
    dw3000_gpio_set_direction(inst, gpioNum, 1);
}

/**
 * API to get GPIO value from an GPIO pin configured as input
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to read from (0-8)
 *
 * @return int          The raw state on the pin currently
 */
int
dw3000_gpio_read(struct _dw3000_dev_instance_t * inst, uint8_t gpioNum)
{
    uint32_t reg = dw3000_gpio_get_values(inst);
    return ((reg&(1<<gpioNum)) != 0);
}

/**
 * API that matches Mynewt's hal_gpio_write
 *
 * @param inst  pointer to _dw3000_dev_instance_t.
 * @param gpioNum       This is the GPIO to configure (0-7)
 * @param val           This sets the GPIO HIGH (1) or LOW (0)
 *
 * @return void
 */
void
dw3000_gpio_write(struct _dw3000_dev_instance_t * inst, int gpioNum, int val)
{
    dw3000_gpio_set_value(inst, gpioNum, val);
}
