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
 * @file twr_ds.c
 * @author UWB Core <uwbcore@gmail.com>
 * @date 2018
 * @brief Range
 *
 * @details This is the rng base class which utilises the functions to enable/disable the configurations related to rng.
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <dpl/dpl.h>
#include <dpl/dpl_cputime.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>

#include <stats/stats.h>
#include <uwb/uwb.h>
#include <uwb_rng/uwb_rng.h>
#if MYNEWT_VAL(UWB_WCS_ENABLED)
#include <uwb_wcs/uwb_wcs.h>
#endif

#if MYNEWT_VAL(RNG_VERBOSE)
#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif
#endif

static bool rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);

static struct uwb_mac_interface g_cbs[] = {
        [0] = {
            .id = UWBEXT_RNG_DS,
            .rx_complete_cb = rx_complete_cb,
        },
#if MYNEWT_VAL(UWB_DEVICE_1) || MYNEWT_VAL(UWB_DEVICE_2)
        [1] = {
            .id = UWBEXT_RNG_DS,
            .rx_complete_cb = rx_complete_cb,
        },
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
        [2] = {
            .id = UWBEXT_RNG_DS,
            .rx_complete_cb = rx_complete_cb,
        }
#endif
};

STATS_SECT_START(twr_ds_stat_section)
    STATS_SECT_ENTRY(complete)
    STATS_SECT_ENTRY(start_tx_error)
STATS_SECT_END

STATS_NAME_START(twr_ds_stat_section)
    STATS_NAME(twr_ds_stat_section, complete)
    STATS_NAME(twr_ds_stat_section, start_tx_error)
STATS_NAME_END(twr_ds_stat_section)

STATS_SECT_DECL(twr_ds_stat_section) g_twr_ds_stat;

static struct uwb_rng_config g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_DS_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(TWR_DS_RX_TIMEOUT)       // Receive response timeout in usec
};

static struct rng_config_list g_rng_cfgs[] = {
    [0] = {
        .rng_code = UWB_DATA_CODE_DS_TWR,
        .name = "twr_ds",
        .config = &g_config
    },
#if MYNEWT_VAL(UWB_DEVICE_1) ||  MYNEWT_VAL(UWB_DEVICE_2)
    [1] = {
        .rng_code = UWB_DATA_CODE_DS_TWR,
        .name = "twr_ds",
        .config = &g_config
    },
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
    [2] = {
        .rng_code = UWB_DATA_CODE_DS_TWR,
        .name = "twr_ds",
        .config = &g_config
    },
#endif
};

/**
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */

void twr_ds_pkg_init(void)
{
    int i, rc;
    struct uwb_dev *udev;
#if MYNEWT_VAL(UWB_PKG_INIT_LOG)
    printf("{\"utime\": %"PRIu32",\"msg\": \"twr_ds_pkg_init\"}\n",
           dpl_cputime_ticks_to_usecs(dpl_cputime_get32()));
#endif

    for (i=0;i < sizeof(g_cbs)/sizeof(g_cbs[0]);i++) {
        udev = uwb_dev_idx_lookup(i);
        if (!udev) {
            continue;
        }
        g_cbs[i].inst_ptr = (struct uwb_rng_instance*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_RNG);
        uwb_mac_append_interface(udev, &g_cbs[i]);
        uwb_rng_append_config(g_cbs[i].inst_ptr, &g_rng_cfgs[i]);
    }

    rc = stats_init(
        STATS_HDR(g_twr_ds_stat),
        STATS_SIZE_INIT_PARMS(g_twr_ds_stat, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(twr_ds_stat_section));
    assert(rc == 0);

    rc = stats_register("twr_ds", STATS_HDR(g_twr_ds_stat));
    assert(rc == 0);

}

/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to struct uwb_rng_instance.
 *
 * @return void
 */
void
twr_ds_free(struct uwb_dev * inst){
    assert(inst);
    uwb_mac_remove_interface(inst, UWBEXT_RNG_DS);
}

int
twr_ds_pkg_down(int reason)
{
    int i;
    struct uwb_rng_instance * rng;

    for (i=0;i < sizeof(g_cbs)/sizeof(g_cbs[0]);i++) {
        rng = (struct uwb_rng_instance *)g_cbs[i].inst_ptr;
        if (!rng) continue;
        uwb_rng_remove_config(g_cbs[i].inst_ptr, g_rng_cfgs[i].rng_code);
        twr_ds_free(rng->dev_inst);
    }
    return 0;
}

/**
 * API for receive complete callback.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return true on sucess
 */
static bool
rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    struct uwb_rng_txd txd;
    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)cbs->inst_ptr;
    assert(rng);
    if(dpl_sem_get_count(&rng->sem) == 1) {
        // unsolicited inbound
        return false;
    }

    int frame_idx = (rng->idx)%rng->nframes;
    int frame_idx_p1 = (rng->idx+1)%rng->nframes;
    int frame_idx_m1 = (rng->idx-1)%rng->nframes;
    /* Frame already read within loader layers. */
    twr_frame_t * frame = &rng->frames[frame_idx];

    switch(rng->code){
       case UWB_DATA_CODE_DS_TWR:
            {
                // This code executes on the device that is responding to a original request

                if (inst->frame_len != sizeof(ieee_rng_request_frame_t))
                    break;

                uint64_t request_timestamp = inst->rxtimestamp;
                uwb_rng_calc_rel_tx(rng, &txd, &g_config, request_timestamp, inst->frame_len);

                frame->reception_timestamp =  (uint32_t) (request_timestamp & 0xFFFFFFFFUL);
                frame->transmission_timestamp =  (uint32_t) (txd.response_timestamp & 0xFFFFFFFFUL);

                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
#if MYNEWT_VAL(UWB_WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = - inst->carrier_integrator;
#endif
                frame->code = UWB_DATA_CODE_DS_TWR_T1;

                uwb_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                uwb_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0);
                uwb_set_wait4resp(inst, true);

                uwb_set_delay_start(inst, txd.response_tx_delay);
                // Disable default behavor, do not RXENAB on RXFCG thereby avoiding rx timeout events
                uwb_set_rxauto_disable(inst, true);

                /* Start tx now, the remaining settings can be done whilst sending anyway */
                if (uwb_start_tx(inst).start_tx_error){
                    STATS_INC(g_twr_ds_stat, start_tx_error);
                    dpl_sem_release(&rng->sem);
                }

                /* Setup when to listen for response, relative the end of our transmitted frame */
                uwb_set_wait4resp_delay(inst, g_config.tx_holdoff_delay -
                                        inst->config.rx.timeToRxStable);
                uwb_set_rx_timeout(inst, uwb_usecs_to_dwt_usecs(uwb_phy_frame_duration(inst, sizeof(ieee_rng_response_frame_t), NULL)) +
                                   g_config.rx_timeout_delay + inst->config.rx.timeToRxStable);
                break;
            }
        case UWB_DATA_CODE_DS_TWR_T1:
            {
                // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                // The 1st frame now contains a local copy of the initial first side of the double sided scheme.

                if(inst->status.lde_error)
                    break;
                if (inst->frame_len != sizeof(ieee_rng_response_frame_t))
                    break;

                twr_frame_t * next_frame = &rng->frames[frame_idx_p1];

                uint64_t request_timestamp = inst->rxtimestamp;
                frame->request_timestamp = next_frame->request_timestamp = uwb_read_txtime_lo32(inst); // This corresponds to when the original request was actually sent
                frame->response_timestamp = next_frame->response_timestamp = (uint32_t)(request_timestamp & 0xFFFFFFFFUL); // This corresponds to the response just received

                uint16_t src_address = frame->src_address;
                uint8_t seq_num = frame->seq_num;

#if MYNEWT_VAL(UWB_WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = inst->carrier_integrator;
#endif
                // Note:: Advance to next frame
                frame = next_frame;
                frame->dst_address = src_address;
                frame->src_address = inst->my_short_address;
                frame->seq_num = seq_num + 1;
                frame->code = UWB_DATA_CODE_DS_TWR_T2;

                if(inst->status.lde_error)
                    break;

                uwb_rng_calc_rel_tx(rng, &txd, &g_config, request_timestamp, inst->frame_len);

                frame->reception_timestamp =  (uint32_t) (request_timestamp & 0xFFFFFFFFUL);
                frame->transmission_timestamp =  (uint32_t) (txd.response_timestamp & 0xFFFFFFFFUL);

                uwb_write_tx(inst, frame->array, 0, sizeof(twr_frame_final_t));
                uwb_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0);
                uwb_set_wait4resp(inst, true);
                uwb_set_delay_start(inst, txd.response_tx_delay);

                // Disable default behavor, do not RXENAB on RXFCG thereby avoiding rx timeout events on sucess
                uwb_set_rxauto_disable(inst, true);

                if (uwb_start_tx(inst).start_tx_error){
                    STATS_INC(g_twr_ds_stat, start_tx_error);
                    dpl_sem_release(&rng->sem);
                }
                /* Setup when to listen for response, relative the end of our transmitted frame */
                uwb_set_wait4resp_delay(inst, g_config.tx_holdoff_delay -
                                        inst->config.rx.timeToRxStable);
                uwb_set_rx_timeout(inst, uwb_usecs_to_dwt_usecs(uwb_phy_frame_duration(inst, sizeof(twr_frame_final_t), NULL)) +
                                   g_config.rx_timeout_delay + inst->config.rx.timeToRxStable);

                break;
            }
        case UWB_DATA_CODE_DS_TWR_T2:
            {
                // This code executes on the device that responded to the original request, and is now preparing the final timestamps
                if(inst->status.lde_error)
                    break;
                if (inst->frame_len != sizeof(twr_frame_final_t))
                    break;

                twr_frame_t * previous_frame = &rng->frames[frame_idx_m1];

                previous_frame->request_timestamp = frame->request_timestamp;
                previous_frame->response_timestamp = frame->response_timestamp;

                uint64_t request_timestamp = inst->rxtimestamp;
                frame->request_timestamp = uwb_read_txtime_lo32(inst);   // This corresponds to when the original request was actually sent
                frame->response_timestamp = (uint32_t) (request_timestamp & 0xFFFFFFFFUL);  // This corresponds to the response just received

                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
#if MYNEWT_VAL(UWB_WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = - inst->carrier_integrator;
#endif
                frame->code = UWB_DATA_CODE_DS_TWR_FINAL;

                // Transmit timestamp final report
                uwb_write_tx(inst, frame->array, 0, sizeof(twr_frame_final_t));
                uwb_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0);

                uwb_rng_calc_rel_tx(rng, &txd, &g_config, inst->rxtimestamp, inst->frame_len);
                uwb_set_delay_start(inst, txd.response_tx_delay);

                if (uwb_start_tx(inst).start_tx_error) {
                    STATS_INC(g_twr_ds_stat, start_tx_error);
                    dpl_sem_release(&rng->sem);
                    rng_issue_complete(inst);
                } else {
                    STATS_INC(g_twr_ds_stat, complete);
                    rng->control.complete_after_tx = 1;
                }

                break;
            }
        case  UWB_DATA_CODE_DS_TWR_FINAL:
            {
                // This code executes on the device that initialed the original request, and has now receive the final response timestamp.
                // This marks the completion of the double-single-two-way request.

                STATS_INC(g_twr_ds_stat, complete);
                dpl_sem_release(&rng->sem);
                rng_issue_complete(inst);
                break;
            }
        default:
                return false;
                break;
    }
    return true;
}
