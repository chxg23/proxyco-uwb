
#include "common.h"

#if MYNEWT_VAL(DW3000_DEVICE_0)

#if MYNEWT_VAL_CHOICE(DW3000_DEVICE_VERSION, C0)
#include "dw3000-c0/dw3000_dev.h"
#include "dw3000-c0/dw3000_hal.h"
#elif MYNEWT_VAL_CHOICE(DW3000_DEVICE_VERSION, D0)
#include "dw3000-d0/dw3000_dev.h"
#include "dw3000-d0/dw3000_hal.h"
#endif

/*
 * dw3000 device structure defined in dw3000_hal.c
 */
static dw3000_dev_instance_t *dw3000_0 = 0;
static struct dw3000_dev_cfg dw3000_0_cfg = {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    .spi_node_cfg = {
        .node_cfg.bus_name = "spi1",
        .pin_cs = MYNEWT_VAL(DW3000_DEVICE_0_SS),
        .mode = BUS_SPI_MODE_0,
        .data_order = HAL_SPI_MSB_FIRST,
        .freq = MYNEWT_VAL(DW3000_DEVICE_BAUDRATE),
    },
#else
    .spi_sem = &g_spi0_sem,
    .spi_num  = 1,
    .spi_baudrate = MYNEWT_VAL(DW3000_DEVICE_BAUDRATE),
#endif
    .ss_pin = MYNEWT_VAL(DW3000_DEVICE_0_SS),
    .rst_pin  = MYNEWT_VAL(DW3000_DEVICE_0_RST),
    .irq_pin  = MYNEWT_VAL(DW3000_DEVICE_0_IRQ),
    .rx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_0_RX_ANT_DLY),
    .tx_antenna_delay = MYNEWT_VAL(DW3000_DEVICE_0_TX_ANT_DLY),
    .ext_clock_delay = 0
};


int bsp_init_dw3000(void)
{
    int rc;
    dw3000_0 = hal_dw3000_inst(0);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    rc = dw3000_create_spi_dev(&dw3000_0->uwb_dev.spi_node, "dw3000_0", &dw3000_0_cfg);
#else
    rc = os_dev_create((struct os_dev *) dw3000_0, "dw3000_0",
      OS_DEV_INIT_PRIMARY, 0, dw3000_dev_init, (void *)&dw3000_0_cfg);
#endif
    assert(rc == 0);

    return 0;
}
#else

int bsp_init_dw3000(void)
{
    return 0;
}
#endif
