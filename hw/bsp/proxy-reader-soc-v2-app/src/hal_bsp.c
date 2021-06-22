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

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <os/mynewt.h>
#include <nrfx.h>
#include <flash_map/flash_map.h>
#include <hal/hal_bsp.h>
#include <hal/hal_flash.h>
#include <hal/hal_system.h>
#include <mcu/nrf5340_hal.h>
#include <mcu/nrf5340_periph.h>
#include <bsp/bsp.h>
#if MYNEWT_VAL(M41T82_ONB)
#include "m41t82/m41t82.h"
static struct m41t82 m41t82;
#endif
#if MYNEWT_VAL(PAC1921_ONB)
#include "pac1921/pac1921.h"
static struct pac1921 pac1921;
#endif
#if MYNEWT_VAL(LIS2DT_ONB)
#include <lis2dw12/lis2dw12.h>
static struct lis2dw12 lis2dw12;
#endif
#if MYNEWT_VAL(FRAM_ONB) && !MYNEWT_VAL(BOOT_LOADER)
#include "fram/fram.h"
#endif

#if MYNEWT_VAL(NEOPIXELS_ONB) && !MYNEWT_VAL(BOOT_LOADER)
#include "neopixel/neopixel.h"
#endif

#if MYNEWT_VAL(SPIFLASH)
#include <spiflash/spiflash.h>
#endif

#if MYNEWT_VAL(PN5180_ONB) && !MYNEWT_VAL(BOOT_LOADER)
#include "../../../lib/nxp_nfc/src/comps/phhalHw/src/Pn5180/phhalHw_Pn5180.h"
static struct pn5180 g_pn5180;
struct pn5180_itf g_pn5180_itf = {
  .pi_ints = {
    {
      .host_pin = MYNEWT_VAL(PN5180_ONB_INT_PIN),
      .device_pin = MYNEWT_VAL(PN5180_INT1_PIN_DEVICE),
      .active = MYNEWT_VAL(PN5180_INT1_CFG_ACTIVE)
    },
  }
};
#endif

#if MYNEWT_VAL(M41T82_ONB)
static const struct bus_i2c_node_cfg m41t82_node_cfg = {
  .node_cfg = {
    .bus_name = MYNEWT_VAL(M41T82_ONB_I2C_BUS),
  },
  .addr = MYNEWT_VAL(M41T82_I2C_ADDR),
  .freq = 390,
};
static struct m41t82_itf m41t82_itf;
#endif

#if MYNEWT_VAL(PAC1921_ONB)
static const struct bus_i2c_node_cfg pac1921_node_cfg = {
  .node_cfg = {
    .bus_name = MYNEWT_VAL(PAC1921_ONB_I2C_BUS),
  },
  .addr = MYNEWT_VAL(PAC1921_I2C_ADDR),
  .freq = 390,
};
static struct pac1921_itf pac1921_itf;
#endif

#if MYNEWT_VAL(LIS2DT_ONB)
static const struct bus_i2c_node_cfg lis2dw12_node_cfg = {
  .node_cfg = {
    .bus_name = MYNEWT_VAL(LIS2DT_ONB_I2C_BUS),
  },
  .addr = MYNEWT_VAL(LIS2DT_ONB_I2C_ADDR),
  .freq = 390,
};
static struct sensor_itf lis2dw12_itf = {
  .si_ints = {
    {
      MYNEWT_VAL(LIS2DT_ONB_INT1_PIN_HOST), 1,
      MYNEWT_VAL(LIS2DT_ONB_INT_CFG_ACTIVE)
    },
    { -1, 0, 0 },
  }
};
#endif

#if MYNEWT_VAL(FRAM_ONB) && !MYNEWT_VAL(BOOT_LOADER)
static const struct hal_flash_funcs fram_fm25_funcs = {
  .hff_read         = (int (*)(const struct hal_flash * dev, uint32_t address, void *dst, uint32_t num_bytes)) fram_fm25_read,
  .hff_write        = (int (*)(const struct hal_flash * dev, uint32_t address, const void *src, uint32_t num_bytes))fram_fm25_write,
  .hff_init         = (int (*)(const struct hal_flash * dev)) fram_fm25_init,
};

static struct fram_fm25_dev fram_fm25_default_dev = {
  /* struct hal_flash for compatibility */
  .hal = {
    .hf_itf        = &fram_fm25_funcs,
  },

  /* SPI settings + updates baudrate on _init */
  .spi_settings       = NULL,

  /* Configurable fields that must be populated by user app */
  .spi_num            = MYNEWT_VAL(FRAM_SPI_NUM),
  .spi_cfg            = NULL,
  .ss_pin             = MYNEWT_VAL(SPI_FRAM_CS_PIN),
  .baudrate           = MYNEWT_VAL(FRAM_SPI_BAUDRATE),

};
#endif

#if MYNEWT_VAL(BUS_DRIVER_PRESENT) && MYNEWT_VAL(PN5180_ONB) && !MYNEWT_VAL(BOOT_LOADER)
struct bus_spi_node_cfg pn5180_spi_cfg = {
  .node_cfg.bus_name = MYNEWT_VAL(PN5180_ONB_BUS),
  .pin_cs = MYNEWT_VAL(PN5180_ONB_CS),
  .mode = BUS_SPI_MODE_0,
  .data_order = HAL_SPI_MSB_FIRST,
  .freq = MYNEWT_VAL(PN5180_ONB_BAUDRATE),
  .quirks = 0
};
#endif

#if MYNEWT_VAL(SPIFLASH)
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
struct bus_spi_node_cfg flash_spi_cfg = {
  .node_cfg.bus_name = MYNEWT_VAL(FLASH_ONB_SPI_BUS),
  .pin_cs = MYNEWT_VAL(SPIFLASH_SPI_CS_PIN),
  .mode = BUS_SPI_MODE_3,
  .data_order = HAL_SPI_MSB_FIRST,
  .freq = MYNEWT_VAL(SPIFLASH_BAUDRATE),
};
#endif
#endif

static const struct hal_flash *flash_devs[] = {
  [0] = &nrf5340_flash_dev,
#if MYNEWT_VAL(SPIFLASH)
  [1] = &spiflash_dev.hal,
#endif
};

/*
 * What memory to include in coredump.
 */
static const struct hal_bsp_mem_dump dump_cfg[] = {
  [0] = {
    .hbmd_start = &_ram_start,
    .hbmd_size = RAM_SIZE
  }
};

const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
  /*
   * Internal flash mapped to id 0.
   */
  if (id == 0) {
    return flash_devs[0];
  }
  /*
   * External flash mapped to id 1.
   */
  if (id == 1) {
    return flash_devs[1];
  }

  return NULL;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
  *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
  return dump_cfg;
}

int
hal_bsp_power_state(int state)
{
  return 0;
}

/**
 * M41T82 RTC chip default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_m41t82_sensor(void)
{
#if MYNEWT_VAL(M41T82_ONB)
  int rc;
  struct os_dev *dev;
  struct m41t82_cfg cfg;

  dev = (struct os_dev *) os_dev_open("m41t82_0", OS_TIMEOUT_NEVER, NULL);
  assert(dev != NULL);

  memset(&cfg, 0, sizeof(cfg));

  rc = m41t82_config(&((struct m41t82 *) dev)->itf, &cfg);
  SYSINIT_PANIC_ASSERT(rc == 0);

  os_dev_close(dev);
#endif
  return 0;
}

/**
 * PAC1921 current/power measurement chip
 * default configuration.
 *
 * @return 0 on success, non-zero on failure.
 */
int
config_pac1921_sensor(void)
{
#if MYNEWT_VAL(PAC1921_ONB)
  int rc;
  struct os_dev *dev;
  struct pac1921_cfg cfg;
  memset(&cfg, 0, sizeof(cfg));

  /* Start at highest internal ADC resolution
   * 11-bit resolution is recommended if the fastest
   * integration time is required. 14-bit resolution will
   * provide more accurate and highly averaged measurements
   */
  cfg.ires = PAC1921_GCR_IRES_ADC_14_BIT;
  cfg.vres = PAC1921_GCR_VRES_ADC_14_BIT;
  /* See TABLE 4-6 of the datasheet
   * Onboard sense resistor is 0.01ohm and max VSENSE is 0.1V
   * Given current budget of 250mA, adjust this gain value to 32x as
   * a good starting point. This makes the effective current range
   * (0.3125mV/0.01ohm) = 312.5mA
   * Gain can be dynamically adjusted by the application
   */
  cfg.di_gain = PAC1921_GCR_DI_GAIN_32X;
  /* See TABLE 4-7 of the datasheet
   * Given we run at standard 24V, 1x gain is sufficient
   */
  cfg.dv_gain = PAC1921_GCR_DV_GAIN_1X;
  /* See TABLE 4-5: FREE RUN INTEGRATION PERIODS of the datasheet
   * With 14bit ADC and post filter enabled
   * 16 samples of VSENSE expected to take 11.5 ms */
  cfg.samples = PAC1921_ICR_SMPL_16;
  /* ADC post filtering improves signal quality and
   * increases conversion time by 50%
   */
  cfg.vsfen = PAC1921_ICR_VSENSE_FILTER_ENABLE;
  cfg.vbfen = PAC1921_ICR_VBUS_FILTER_ENABLE;
  /* Allow overriding READ/INT pin from i2c comms */
  cfg.riov = PAC1921_ICR_OVERRIDE_ENABLE;
  cfg.inten = PAC1921_ICR_INTEN_INTEGRATE_MODE;
  /* Start with sensing current */
  cfg.mxsl = PAC1921_CR_MXSL_VSENSE_FREERUN;
  /* Keep full scale output for the OUT pin at 3V */
  cfg.ofsr = PAC1921_CR_OFSR_3V;
  cfg.tout = PAC1921_CR_TOUT_DISABLE;
  cfg.sleep = PAC1921_CR_SLEEP_DISABLE;
  cfg.slpov = PAC1921_CR_SLPOV_DISABLE;
  cfg.rdac = PAC1921_CR_RDAC_DISABLE;

  dev = (struct os_dev *) os_dev_open("pac1921_0", OS_TIMEOUT_NEVER, NULL);
  assert(dev != NULL);

  rc = pac1921_config(&((struct pac1921 *) dev)->itf, &cfg);
  SYSINIT_PANIC_ASSERT(rc == 0);

  os_dev_close(dev);
#endif
  return 0;
}

int
config_lis2dt_sensor(void)
{
#if MYNEWT_VAL(LIS2DT_ONB)
  int rc;
  struct os_dev *dev;
  struct lis2dw12_cfg cfg = {0};

  dev = (struct os_dev *) os_dev_open("lis2dw12_0", OS_TIMEOUT_NEVER, NULL);
  assert(dev != NULL);

  /* Valid Tap ODRs are 400 Hz, 800 Hz and 1600 Hz  AN5038 5.6.3 */
  cfg.rate = LIS2DW12_DATA_RATE_400HZ;
  cfg.fs = LIS2DW12_FS_2G;

  cfg.offset_x = 0;
  cfg.offset_y = 0;
  cfg.offset_z = 0;
  cfg.offset_weight = 0;
  cfg.offset_en = 0;

  cfg.filter_bw = LIS2DW12_FILTER_BW_ODR_DIV_2;
  cfg.high_pass = 0;

  cfg.tap.en_x = 1;
  cfg.tap.en_y = 1;
  cfg.tap.en_z = 1;
  cfg.tap.en_4d = 0;
  cfg.tap.ths_6d = LIS2DW12_6D_THS_80_DEG;
  cfg.tap.tap_priority = LIS2DW12_TAP_PRIOR_XYZ;
  cfg.tap.tap_ths_x = 0x3; /* 1875mg = (3 * FS / 32) */
  cfg.tap.tap_ths_y = 0x3; /* 1875mg = (3 * FS / 32) */
  cfg.tap.tap_ths_z = 0x3; /* 1875mg = (3 * FS / 32) */
  cfg.tap.latency = 8; /* 640ms (= 8 * 32 / ODR) */
  cfg.tap.quiet = 0; /* 5 ms  (= 2 / ODR */
  cfg.tap.shock = 3; /* 60 ms (= 3 * 8 / ODR) */

  cfg.double_tap_event_enable = 0;

  cfg.freefall_dur = 6; /* 15ms (= 6/ODR) */
  cfg.freefall_ths = 3; /* ~312mg (= 31.25 mg * 10) */

  cfg.int1_pin_cfg = 0;
  cfg.int2_pin_cfg = 0;
  cfg.int_enable = 0;

  cfg.int_pp_od = 0;
  cfg.int_latched = 0;
  cfg.int_active_low = 0;
  cfg.slp_mode = 0;

  cfg.fifo_mode = LIS2DW12_FIFO_M_BYPASS;
  cfg.fifo_threshold = 32;

  cfg.wake_up_ths = 0; /* 0 mg (= 0 * FS / 64) */
  cfg.wake_up_dur = 0; /* 0 ms (= 0 * 1 / ODR) */
  cfg.sleep_duration = 0; /* 0 ms (= 0 * 512 / ODR) */

  cfg.stationary_detection_enable = 0;

  cfg.power_mode = LIS2DW12_PM_HIGH_PERF;
  cfg.inactivity_sleep_enable = 0;
  cfg.low_noise_enable = 1;

  cfg.read_mode.mode = LIS2DW12_READ_M_POLL;

  cfg.mask = SENSOR_TYPE_ACCELEROMETER;

  rc = lis2dw12_config((struct lis2dw12 *) dev, &cfg);
  assert(rc == 0);

  os_dev_close(dev);
  return rc;
#endif
  return 0;
}

static void
rtc_dev_create(void)
{
  int rc;
  (void)rc;

#if MYNEWT_VAL(M41T82_ONB)
  rc = m41t82_create_i2c_sensor_dev(&m41t82.i2c_node, "m41t82_0",
          &m41t82_node_cfg, &m41t82_itf);
  SYSINIT_PANIC_ASSERT(rc == 0);
#endif
}

static void
csense_dev_create(void)
{
  int rc;
  (void)rc;

#if MYNEWT_VAL(PAC1921_ONB)
  rc = pac1921_create_i2c_sensor_dev(&pac1921.i2c_node, "pac1921_0",
          &pac1921_node_cfg, &pac1921_itf);
  SYSINIT_PANIC_ASSERT(rc == 0);
#endif
}

static void
accel_dev_create(void)
{
  int rc;
  (void)rc;

#if MYNEWT_VAL(LIS2DT_ONB)
  rc = lis2dw12_create_i2c_sensor_dev((struct bus_i2c_node *)&lis2dw12, "lis2dw12_0",
          &lis2dw12_node_cfg, &lis2dw12_itf);
  SYSINIT_PANIC_ASSERT(rc == 0);
#endif
}

#if MYNEWT_VAL(PN5180_ONB) && !MYNEWT_VAL(BOOT_LOADER)
void
config_pn5180(void)
{
  struct os_dev *dev;
  struct pn5180 *pn5180;

  dev = (struct os_dev *) os_dev_open("pn5180_0", OS_TIMEOUT_NEVER, NULL);
  assert(dev != NULL);

  pn5180 = (struct pn5180 *)dev;
  pn5180->cfg.wId      = PH_COMP_DRIVER;
  pn5180->cfg.bBalType = PHBAL_REG_TYPE_SPI;
}
#endif

static void
pn5180_dev_create(void)
{
#if MYNEWT_VAL(PN5180_ONB) && !MYNEWT_VAL(BOOT_LOADER)
  int rc;
  rc = pn5180_create_spi_dev(&g_pn5180.spi_node, "pn5180_0",
          &pn5180_spi_cfg, &g_pn5180_itf);
  SYSINIT_PANIC_ASSERT(rc == 0);
#endif
}

static void
fram_dev_create(void)
{
#if MYNEWT_VAL(FRAM_ONB) && !MYNEWT_VAL(BOOT_LOADER)
  int rc;
  static struct os_dev os_bsp_fram;
  rc = os_dev_create((struct os_dev *) &os_bsp_fram, "fram_fm25",
          OS_DEV_INIT_KERNEL, 0, fram_fm25_init, &fram_fm25_default_dev);
  SYSINIT_PANIC_ASSERT(rc == 0);
#endif
}

static void
ext_flash_dev_create(void)
{
#if MYNEWT_VAL(FLASH_ONB) && !MYNEWT_VAL(BOOT_LOADER)
  /* Create external flash dev */
  int rc = spiflash_create_spi_dev(&spiflash_dev.dev,
          MYNEWT_VAL(FLASH_ONB_SPI_BUS), &flash_spi_cfg);
  assert(rc == 0);
#endif
}

static void
proxy_reader_v2_periph_init(void)
{
  fram_dev_create();

  ext_flash_dev_create();

  rtc_dev_create();

  accel_dev_create();

  csense_dev_create();

  pn5180_dev_create();

#if MYNEWT_VAL(NEOPIXELS_ONB) && !MYNEWT_VAL(BOOT_LOADER)
  neopixel_init();
#endif

  /* note: spiram device is created via pkg init */
}

void
hal_bsp_init(void)
{
  /* Make sure system clocks have started */
  hal_system_clock_start();

  /* Create all available nRF5340 peripherals */
  nrf5340_periph_create();

  /* Create and initialize peripherals onboard reader v2 */
  proxy_reader_v2_periph_init();
}

void
hal_bsp_deinit(void)
{
}
