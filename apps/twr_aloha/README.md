<!--
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#
-->

# Ultrawideband TWR_ALOHA Example

## Overview

WIP

1. To erase the default flash image that shipped with the DWM1001 boards.

```no-highlight
$ JLinkExe -device nRF52 -speed 4000 -if SWD
J-Link>erase
J-Link>exit
$
```

2. On 1st dwm1001-dev board build the **node** application for the DWM1001 module.

```no-highlight

newt target create twr_node
newt target set twr_node app=apps/twr_aloha
newt target set twr_node bsp=@ultrawideband-core/hw/bsp/dwm1001
newt target set twr_node build_profile=debug
newt target amend twr_node syscfg=LOG_LEVEL=1:UWBCFG_DEF_ROLE='"0x4"'
newt run twr_node 0

```

3. On 2nd dwm1001-dev board build the **tag** application for the DWM1001 module.

```no-highlight

newt target create twr_tag
newt target set twr_tag app=apps/twr_aloha
newt target set twr_tag bsp=@ultrawideband-core/hw/bsp/dwm1001
newt target set twr_tag build_profile=debug
newt run twr_tag 0

```

4. On the console you should see the following expected result.

Use telnet or nc if you are using CONSOLE_RTT: 1 in syscfg.yml
Or screen or socat if you are using CONSOLE_UART: 1 in syscfg.yml

For Human readable JSON you can use add FLOAT_USER
```no-highlight
newt target amend twr_node syscfg=FLOAT_USER=1
```

```no-highlight

telnet localhost 19021
....
{"utime": 15531276,"c": 274,"uid": 186,"ouid": 4660,"raz": [0.696],"rssi": [-79.186],"los": [1.000]}
{"utime": 15546935,"c": 278,"uid": 186,"ouid": 4660,"raz": [0.709],"rssi": [-79.019],"los": [1.000]}
{"utime": 15580982,"c": 291,"uid": 4660,"ouid": 186,"raz": [0.752],"rssi": [-79.060],"los": [1.000]}
....
{"utime": 15604413,"c": 296,"uid": 4660,"ouid": 186,"raz": [0.767],"rssi": [-78.898],"los": [1.000]}
{"utime": 15624994,"c": 274,"uid": 186,"ouid": 4660,"raz": [0.743],"rssi": [-79.019],"los": [1.000]}
{"utime": 15640652,"c": 278,"uid": 186,"ouid": 4660,"raz": [0.723],"rssi": [-78.858],"los": [1.000]}

```

where the different fields are:
 - utime: mcu time in microseconds
 - c: range type code, see main.c around line 166
 - uid and ouid: The two board ids who partook in the range exchange
 - raz: Rance Azimuth Zenith, in meter and radians. If only one value is present that is the range.
 - rssi: Receiver Signal Strenght Indicator, in dbm
 - los: Probability of LOS as estimated by the chip (based on ratio of rssi to fppl)



5. Trying different ranging algorithms

The method for ranging used is selected by modifying the mode variable in the uwb_ev_cb
function in main.c. By default, it will use one of the modes available and setting the
```mode``` variable accordingly. By editing main you can select another mode if that
is available:

```
...// around line 166 in src/main.c
        /* Uncomment the next line to force the range mode */
        // mode = UWB_DATA_CODE_SS_TWR_ACK;
...
```

## Using DW3000 as node and tag on ST nucleo

```no-highlight
newt target create nucleo-f429zi_twr_aloha_tag
newt target set nucleo-f429zi_twr_aloha_tag app=apps/twr_aloha
newt target set nucleo-f429zi_twr_aloha_tag bsp=@ultrawideband-core/hw/bsp/nucleo-f429zi
newt target set nucleo-f429zi_twr_aloha_tag build_profile=debug
newt target amend nucleo-f429zi_twr_aloha_tag syscfg=BLE_ENABLED=0:CONSOLE_UART=1:CONSOLE_RTT=0
newt run nucleo-f429zi_twr_aloha_tag 0
```
```no-highlight
newt target create nucleo-f429zi_twr_aloha_node
newt target set nucleo-f429zi_twr_aloha_node app=apps/twr_aloha
newt target set nucleo-f429zi_twr_aloha_node bsp=@ultrawideband-core/hw/bsp/nucleo-f429zi
newt target set nucleo-f429zi_twr_aloha_node build_profile=debug
newt target amend nucleo-f429zi_twr_aloha_node syscfg=LBLE_ENABLED=0:LOG_LEVEL=1:UWBCFG_DEF_ROLE='"0x4"':CONSOLE_UART=1:CONSOLE_RTT=0
newt run nucleo-f429zi_twr_aloha_node 0
```