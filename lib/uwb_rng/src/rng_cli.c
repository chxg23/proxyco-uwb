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

#include <dpl/dpl.h>
#include <syscfg/syscfg.h>

#if MYNEWT_VAL(UWB_RNG_CLI)

#include <string.h>

#include <shell/shell.h>
#include <console/ticks.h>
#include <console/console.h>
#include <streamer/streamer.h>

#include <uwb/uwb_mac.h>
#include "uwb_rng/uwb_rng.h"

static int rng_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer);
static int rng_request(struct uwb_rng_instance *rng, uint64_t addr, const char* rng_type, struct streamer *streamer);
static bool complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool rx_timeout_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);


#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_rng_param[] = {
    {"rx", "[inst] [timeout us, 0=no timeout] Listen for requests"},
    {"req", "<inst> <dst-addr> [rng-type] twr request"},
    {"creq", "<inst> <dst-addr> <interval us> [rng-type] Continuous twr requests"},
    {"abort", "[inst] Stop listening / requesting"},
    {"reqtypes", "List rng types available"},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_rng_help = {
    "uwb rng commands", "<cmd>", cmd_rng_param
};
#endif

static struct shell_cmd shell_rng_cmd = SHELL_CMD_EXT("rng", rng_cli_cmd, &cmd_rng_help);

/* Timer callout */
static struct dpl_event slot_event = {0};
static struct dpl_callout tx_callout;
static uint8_t role = 0;
static uint8_t repeat_req_shown = 0;
static char repeat_rng_type[16] = {0};
static uint64_t repeat_addr = 0;
static uint64_t repeat_timeout = 0;
static uint64_t repeat_interval_ticks = DPL_TICKS_PER_SEC/60;
struct streamer *delayed_streamer = 0;

static struct uwb_mac_interface g_cbs[] = {
    [0] = {
        .id = UWBEXT_APP1,
        .inst_ptr = 0,
        .complete_cb = complete_cb,
        .rx_timeout_cb = rx_timeout_cb,
    },
#if MYNEWT_VAL(UWB_DEVICE_1)
    [1] = {
        .id = UWBEXT_APP1,
        .inst_ptr = 0,
        .complete_cb = complete_cb,
        .rx_timeout_cb = rx_timeout_cb,
    },
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
    [2] = {
        .id = UWBEXT_APP1,
        .inst_ptr = 0,
        .complete_cb = complete_cb,
        .rx_timeout_cb = rx_timeout_cb,
    }
#endif
};

static void
slot_complete_cb(struct dpl_event *ev)
{
    assert(ev != NULL);

    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)dpl_event_get_arg(ev);
    struct uwb_dev * inst = rng->dev_inst;

    if (inst->role&UWB_ROLE_ANCHOR) {
        uwb_rng_listen(rng, 0xfffff, UWB_NONBLOCKING);
    }
}

static bool
complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    struct uwb_rng_instance * rng = (struct uwb_rng_instance*)cbs->inst_ptr;
    if (!dpl_event_is_queued(&slot_event)) {
        dpl_event_init(&slot_event, slot_complete_cb, rng);
        dpl_eventq_put(dpl_eventq_dflt_get(), &slot_event);
    }
    return true;
}

static bool
rx_timeout_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    struct uwb_rng_instance * rng = (struct uwb_rng_instance*)cbs->inst_ptr;
    if (role & UWB_ROLE_ANCHOR) {
        uwb_phy_forcetrxoff(inst);
        uwb_rng_listen(rng, repeat_timeout, UWB_NONBLOCKING);
    } else if (!repeat_addr) {
        streamer_printf(delayed_streamer, "Rng Timeout\n");
    }
    return true;
}

static void
uwb_ev_cb(struct dpl_event *ev)
{
    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)dpl_event_get_arg(ev);

    if (role & UWB_ROLE_ANCHOR) {
        if(dpl_sem_get_count(&rng->sem) == 1){
            uwb_rng_listen(rng, 0xfffff, UWB_NONBLOCKING);
        }
        dpl_callout_reset(&tx_callout, repeat_interval_ticks);
    } else if (repeat_addr) {
        if (!rng_request(rng, repeat_addr, repeat_rng_type, delayed_streamer)) {
            dpl_callout_reset(&tx_callout, repeat_interval_ticks);
        }
    }
}


static void
list_reqtypes(struct uwb_rng_instance *rng, struct streamer *streamer)
{
    int n_rngtypes=0;
    struct rng_config_list * cfgs;

    console_no_ticks();
    streamer_printf(streamer, "Available rng types:\n");
    if(!(SLIST_EMPTY(&rng->rng_configs))){
        SLIST_FOREACH(cfgs, &rng->rng_configs, next){
            streamer_printf(streamer, "  %s\n", cfgs->name);
            n_rngtypes++;
        }
    }
    streamer_printf(streamer, "Total: %d types\n", n_rngtypes);
    console_yes_ticks();
}

static int
rng_request(struct uwb_rng_instance *rng, uint64_t addr, const char* rng_type, struct streamer *streamer)
{
    struct rng_config_list * cfgs;

    if(!(SLIST_EMPTY(&rng->rng_configs))){
        SLIST_FOREACH(cfgs, &rng->rng_configs, next){
            if (!strcmp(cfgs->name, rng_type) || !strlen(rng_type)) {
                if (!repeat_req_shown) {
                    console_no_ticks();
                    streamer_printf(streamer, "  rng request to dest:0x%"PRIx64" type:'%s' \n", addr, cfgs->name);
                    console_yes_ticks();
                    repeat_req_shown = 1;
                }
                uwb_rng_request(rng, addr, cfgs->rng_code);
                return 0;
            }
        }
    }
    streamer_printf(streamer, "error, could not find rng-type: '%s'\n", rng_type);
    return -1;
}

static int
rng_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer)
{
    int inst_n = 0;
    uint64_t addr = 0;
    uint32_t param = 0;
    struct uwb_dev *dev = 0;
    struct uwb_rng_instance *rng;

    if (argc < 2) {
        streamer_printf(streamer, "Too few args\n");
        return 0;
    }

    if (argc > 2) {
        inst_n = strtol(argv[2], NULL, 0);
    }
    dev = uwb_dev_idx_lookup(inst_n);
    if (!dev) {
        streamer_printf(streamer, "No such instance %d\n", inst_n);
        return -1;
    }
    rng = (struct uwb_rng_instance*)uwb_mac_find_cb_inst_ptr(dev, UWBEXT_RNG);
    if (!rng) {
        streamer_printf(streamer, "No such instance %d\n", inst_n);
        return -1;
    }

    if (!g_cbs[inst_n].inst_ptr) {
        g_cbs[inst_n].inst_ptr = rng;
        uwb_mac_append_interface(dev, &g_cbs[inst_n]);
    }

    if (!dpl_callout_is_active(&tx_callout)) {
        dpl_callout_init(&tx_callout, dpl_eventq_dflt_get(), uwb_ev_cb, rng);
    }

    if (!strcmp(argv[1], "reqtypes")) {
        list_reqtypes(rng, streamer);
    } else if (!strcmp(argv[1], "rx")) {
        if (argc > 3) {
            param = strtol(argv[3], NULL, 0);
        }
        role = UWB_ROLE_ANCHOR;
        if(dpl_sem_get_count(&rng->sem) == 0) {
            /* Stop anything running already */
            dpl_callout_stop(&tx_callout);
            repeat_addr = 0;
            uwb_phy_forcetrxoff(dev);
        }

#if MYNEWT_VAL(TWR_SS_ACK_ENABLED)
        uwb_set_autoack(dev, true);
        uwb_set_autoack_delay(dev, 12);
        if (!(dev->config.rx.frameFilter&UWB_FF_ACK_EN)) {
            streamer_printf(streamer, "WARNING: FF_ACK_EN not present in uwb/frame_filter config, twr_ss_ack will not work\n");
        }
#endif
        repeat_timeout = param;
        uwb_rng_listen(rng, param, UWB_NONBLOCKING);
        streamer_printf(streamer, "Listening on addr:0x%x with timeout: %"PRIu32" uus\n", dev->uid, param);
        repeat_interval_ticks = DPL_TICKS_PER_SEC/60;
        dpl_callout_reset(&tx_callout, repeat_interval_ticks);
    } else if (!strcmp(argv[1], "req")) {
        if (argc < 4) {
            streamer_printf(streamer, "Too few args\n");
            return -1;
        }
        addr = strtol(argv[3], NULL, 0);
        role = 0;
        repeat_addr = 0;
        repeat_req_shown = 0;
        memset(repeat_rng_type, 0, sizeof(repeat_rng_type));
        delayed_streamer = streamer;

        if(dpl_sem_get_count(&rng->sem) == 0) {
            /* Stop anything running already */
            dpl_callout_stop(&tx_callout);
            uwb_phy_forcetrxoff(dev);
        }

        return rng_request(rng, addr, argv[4], streamer);
    } else if (!strcmp(argv[1], "creq")) {
        if (argc < 5) {
            streamer_printf(streamer, "Too few args\n");
            return -1;
        }
        repeat_addr = addr = strtol(argv[3], NULL, 0);
        if (argc > 4) {
            repeat_interval_ticks = (strtol(argv[4], NULL, 0) * DPL_TICKS_PER_SEC)/1000000;
            if (repeat_interval_ticks < 1)
            {
                repeat_interval_ticks = 1;
            }
        }
        role = 0;
        repeat_req_shown = 0;
        strncpy(repeat_rng_type, argv[5], sizeof(repeat_rng_type)-1);
        delayed_streamer = streamer;
        if(dpl_sem_get_count(&rng->sem) == 0) {
            /* Stop anything running already */
            dpl_callout_stop(&tx_callout);
            uwb_phy_forcetrxoff(dev);
        }

        dpl_callout_reset(&tx_callout, 1);
    } else if (!strcmp(argv[1], "abort")) {
        dpl_callout_stop(&tx_callout);
        role = 0;
        repeat_addr = 0;
        uwb_phy_forcetrxoff(dev);
        streamer_printf(streamer, "Rng aborted\n");
    } else {
        streamer_printf(streamer, "Unknown cmd\n");
    }
    return 0;
}

int
uwb_rng_cli_register(void)
{
    dpl_callout_init(&tx_callout, dpl_eventq_dflt_get(), uwb_ev_cb, 0);
    return shell_cmd_register(&shell_rng_cmd);
}
#endif /* MYNEWT_VAL(UWB_RNG_CLI) */
