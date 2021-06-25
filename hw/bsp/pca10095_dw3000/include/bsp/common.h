#ifndef BSP_COMMON_H_
#define BSP_COMMON_H_

#include "os/mynewt.h"
//#include "flash_map/flash_map.h"
#include "hal/hal_bsp.h"
#include "hal/hal_system.h"
#include "hal/hal_flash.h"
#include "hal/hal_spi.h"
#include "hal/hal_watchdog.h"
#include "hal/hal_i2c.h"
#include "hal/hal_gpio.h"

#include "bsp.h"


#ifndef STRINGIFY
/// add double quote around param X
#define STRINGIFY(X) ___STRINGIFY_VIA(X)
// Via expand macro.
#define ___STRINGIFY_VIA(X) #X
#endif

#endif
