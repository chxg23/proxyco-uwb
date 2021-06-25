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

#include <syscfg/syscfg.h>

#ifdef __KERNEL__
#include <dpl/dpl.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include "uwbcore.h"
#include <dw3000-c0/dw3000_hal.h>
#include <dw3000-c0/dw3000_otp.h>
#include "dw3000_cli_priv.h"

#define slog(fmt, ...) \
    pr_info("uwbcore: %s(): %d: "fmt, __func__, __LINE__, ##__VA_ARGS__)

#define OTP_REG_NAME_MAXLEN (8)

struct dw3000cli_sysfs_data {
    struct kobject *kobj;
    char device_name[16];
    struct mutex local_lock;
    struct _dw3000_dev_instance_t *inst;
    u16 cont_tx_frame_len;

    struct {
        struct kobj_attribute *dev_attr_cfg;
        struct attribute** attrs;
        struct attribute_group attribute_group;
    } cli;
    struct kobject *kobj_otp;
    struct {
        struct kobj_attribute *dev_attr_cfg;
        struct attribute** attrs;
        struct attribute_group attribute_group;
        char* reg_name;
        u16 write_en_address;
    } otp;
};

static struct dw3000cli_sysfs_data dw3000cli_sysfs_inst[MYNEWT_VAL(UWB_DEVICE_MAX)] = {0};

static struct dw3000cli_sysfs_data* get_instance(struct kobject* kobj)
{
    struct kobject* ko = kobj;
    /* Walk up in parent objects until they end with a number we can use
     * to figure out our instance */
    while (ko) {
        char s = ko->name[strlen(ko->name)-1];
        if (s >= '0' && s<= '9') {
            u8 i = s-'0';
            if (i<MYNEWT_VAL(UWB_DEVICE_MAX)) {
                return &dw3000cli_sysfs_inst[i];
            }
        }
        ko = ko->parent;
    }
    return 0;
}

static ssize_t cmd_show(struct kobject *kobj,
    struct kobj_attribute *attr, char *buf)
{
    int gpio_num;
    unsigned int copied;
    const char nodev_errmsg[] = "err, no device\n";
    const char antmux_help[] = "# Antenna Mux examples:\n"
        "#  Force ant 1, pdoa start on ant 1:  0x1100\n"
        "#  Force ant 2, pdoa start on ant 2:  0x2102\n"
        "#  Force ant 1, no ant-sw in pdoa_md: 0x1101\n"
        "#  Force ant 2, no ant-sw in pdoa_md: 0x2103\n"
        "#  Default: 0x0000\n";
    struct dw3000cli_sysfs_data *ed;
    struct _dw3000_dev_instance_t *inst;

    ed = get_instance(kobj);
    if (!ed) {
        slog("ERROR: Could not get instance, kobj->name = '%s'\n", kobj->name);
        return -EINVAL;
    }
    inst = ed->inst;

    if (!inst->uwb_dev.status.initialized) {
        memcpy(buf, nodev_errmsg, strlen(nodev_errmsg));
        return strlen(nodev_errmsg);
    }

    if (!strcmp(attr->attr.name, "device_id")) {
        copied = snprintf(buf, PAGE_SIZE, "0x%08X\n", inst->uwb_dev.device_id);
    }

    if (!strcmp(attr->attr.name, "addr") ||
        !strcmp(attr->attr.name, "uid")) {
        copied = snprintf(buf, PAGE_SIZE, "0x%04X\n", inst->uwb_dev.uid);
    }

    if (!strcmp(attr->attr.name, "panid")) {
        copied = snprintf(buf, PAGE_SIZE, "0x%04X\n", inst->uwb_dev.pan_id);
    }

    if (!strcmp(attr->attr.name, "euid")) {
        copied = snprintf(buf, PAGE_SIZE, "0x%016llX\n", inst->uwb_dev.euid);
    }

    if (!strcmp(attr->attr.name, "antmux")) {
        copied = snprintf(buf, PAGE_SIZE, "%s0x%04X\n",
                          antmux_help,
                          (uint16_t)(0xffff&dw3000_read_reg(inst, PDOA_CTRL_ID, 0, 2))
            );
    }

    if (!strcmp(attr->attr.name, "xtal")) {
        u8 xtal_trim = dw3000_read_reg(inst, XTAL_ID, 0, sizeof(uint8_t));
        copied = snprintf(buf, PAGE_SIZE, "%d\n", xtal_trim);
    }

    if (!strcmp(attr->attr.name, "rssi")) {
        dpl_float32_t rssi;
        dw3000_read_rxdiag(inst, &inst->rxdiag, inst->rxdiag.rxd.enabled|UWB_RXDIAG_IPATOV);
        rssi = dw3000_get_rssi(inst);
        copied = snprintf(buf, PAGE_SIZE, DPL_FLOAT32_PRINTF_PRIM" dBm\n",
                        DPL_FLOAT32_PRINTF_VALS(rssi));
    }

    if (!strcmp(attr->attr.name, "continuoustx_framelen")) {
        copied = snprintf(buf, PAGE_SIZE, "%u\n", ed->cont_tx_frame_len);
    }

    if (!strncmp(attr->attr.name, "gpio", 4)) {
        gpio_num = attr->attr.name[4] - '0';
        slog("gpio %d", gpio_num);
        if (!strncmp(attr->attr.name+5, "_mode", 5)) {
            snprintf(buf, PAGE_SIZE, "%d\n", dw3000_gpio_get_mode(inst, gpio_num));
        } else if (!strncmp(attr->attr.name+5, "_dir", 5)) {
            /* Invert value as 1=input, 0=output for dwX000 */
            snprintf(buf, PAGE_SIZE, "%d\n", !dw3000_gpio_get_direction(inst, gpio_num));
        } else {
            snprintf(buf, PAGE_SIZE, "%d\n", dw3000_gpio_read(inst, gpio_num));
        }
    }

    return copied;
}

static ssize_t cmd_store(struct kobject *kobj,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    int ret, gpio_num;
    u64 res;
    struct dw3000cli_sysfs_data *ed;
    struct _dw3000_dev_instance_t *inst;

    ed = get_instance(kobj);
    if (!ed) {
        slog("ERROR: Could not get instance, kobj->name = '%s'\n", kobj->name);
        return -EINVAL;
    }
    inst = ed->inst;

    if (!inst->uwb_dev.status.initialized) {
        return count;
    }

    if (!strcmp(attr->attr.name, "addr") ||
        !strcmp(attr->attr.name, "uid")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            inst->uwb_dev.uid = res;
            uwb_set_uid(&inst->uwb_dev,
                        inst->uwb_dev.uid);
        }
    }

    if (!strcmp(attr->attr.name, "panid")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            inst->uwb_dev.uid = res;
            uwb_set_panid(&inst->uwb_dev,
                          inst->uwb_dev.uid);
        }
    }

    if (!strcmp(attr->attr.name, "euid")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            inst->uwb_dev.euid = res;
            uwb_set_euid(&inst->uwb_dev,
                         inst->uwb_dev.euid);
        }
    }

    if (!strcmp(attr->attr.name, "cw")) {
        ret = kstrtoll(buf, 0, &res);
        dw3000_cli_cw(inst, res);
        if (res) {
            slog("Device now in CW mode on ch %d\n", inst->uwb_dev.config.channel);
        } else {
            slog("CW disabled\n");
        }
    }

    if (!strcmp(attr->attr.name, "autorx")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            dw3000_cli_autorx(inst, res);
            if (res) {
                slog("Device is now in Auto-RX\n");
            } else {
                slog("Auto-RX disabled\n");
            }
        }
    }

    if (!strcmp(attr->attr.name, "continuoustx")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            dw3000_cli_continuous_tx(inst, ed->cont_tx_frame_len, res);
            if (res) {
                slog("Continuous-TX with delay=%"PRIu32"ns len=%d\n", (u32)res, ed->cont_tx_frame_len);
            } else {
                slog("Continuous-TX disabled\n");
            }
        }
    }

    if (!strcmp(attr->attr.name, "continuoustx_framelen")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            ed->cont_tx_frame_len = res;
        }
    }

    if (!strcmp(attr->attr.name, "antmux")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            /* Mask to limit bits controlled */
            res &= (PDOA_CTRL_ANT_SW_OVR_CTRL_BIT_MASK|
                    PDOA_CTRL_ANT_SW_OVR_EN_BIT_MASK|
                    PDOA_CTRL_ANT_DEFAULT_PORT_BIT_MASK|PDOA_CTRL_ANT_SW_NO_TOGGLE_BIT_MASK);
            dw3000_write_reg(inst, PDOA_CTRL_ID, 0, res, 2);
        }
    }

    if (!strcmp(attr->attr.name, "ev")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            dw3000_phy_event_cnt_ctrl(inst, (res != 0), true);
        }
    }

    if (!strcmp(attr->attr.name, "xtal")) {
        ret = kstrtoll(buf, 0, &res);
        if (!ret) {
            dw3000_set_xtal_trim(inst, res&0x7F);
        }
    }

    if (!strncmp(attr->attr.name, "gpio", 4)) {
        gpio_num = attr->attr.name[4] - '0';
        ret = kstrtoll(buf, 0, &res);
        if (ret) {
            return -EINVAL;
        }
        if (!strncmp(attr->attr.name+5, "_mode", 5) && res < 5) {
            slog("gpio %d mode %"PRIu64, gpio_num, res);
            dw3000_gpio_set_mode(inst, gpio_num, res);
        } else if (!strncmp(attr->attr.name+5, "_dir", 5) && res < 2) {
            /* Invert value as 1=input, 0=output for dwX000 */
            dw3000_gpio_set_direction(inst, gpio_num, (res==0));
            slog("gpio(%d): %s\n", gpio_num, (res==0)?"in":"out");
        } else {
            slog("gpio %d write %"PRIu64"\n", gpio_num, res);
            dw3000_gpio_init_out(inst, gpio_num, res);
        }
    }
    return count;
}

static ssize_t otp_show(struct kobject *kobj,
    struct kobj_attribute *attr, char *buf)
{
    int ret;
    u64 res;
    unsigned int copied = 0;
    const char nodev_errmsg[] = "err, no device\n";
    const char write_en_help[] = "# Write the otp address to enable writes for.\n";
    struct dw3000cli_sysfs_data *ed;
    struct _dw3000_dev_instance_t *inst;

    ed = get_instance(kobj);
    if (!ed) {
        slog("ERROR: Could not get instance, kobj->name = '%s'\n", kobj->name);
        return -EINVAL;
    }
    inst = ed->inst;

    if (!inst->uwb_dev.status.initialized) {
        memcpy(buf, nodev_errmsg, strlen(nodev_errmsg));
        return strlen(nodev_errmsg);
    }

    if (!strcmp(attr->attr.name, "write_enable")) {
        copied = snprintf(buf, PAGE_SIZE, "%s0x%02X\n",
                          write_en_help, ed->otp.write_en_address
            );
    }

    if (!strncmp(attr->attr.name, "0x", 2)) {
        ret = kstrtoll(attr->attr.name, 16, &res);
        if (!ret) {
            copied = snprintf(buf, PAGE_SIZE, "0x%08X\n", _dw3000_otp_read(inst, (u8)(res&0x7F)));
        }
    }

    return copied;
}

static ssize_t otp_store(struct kobject *kobj,
    struct kobj_attribute *attr, const char *buf, size_t count)
{
    int ret;
    u64 address, value;
    struct dw3000cli_sysfs_data *ed;
    struct _dw3000_dev_instance_t *inst;

    ed = get_instance(kobj);
    if (!ed) {
        slog("ERROR: Could not get instance, kobj->name = '%s'\n", kobj->name);
        return -EINVAL;
    }
    inst = ed->inst;

    if (!inst->uwb_dev.status.initialized) {
        return count;
    }

    if (!strcmp(attr->attr.name, "write_enable")) {
        ret = kstrtoll(buf, 0, &address);
        if (!ret) {
            ed->otp.write_en_address = address&0x7F;
            slog("otp write enabled for addr %x", ed->otp.write_en_address);
        }
    }

    if (!strncmp(attr->attr.name, "0x", 2)) {
        ret = kstrtoll(attr->attr.name, 0, &address);
        if (ret) {
            slog("Error whilst reading address\n");
            return -EINVAL;
        }
        if (ed->otp.write_en_address != address) {
            slog("OTP Write abort, address isn't write-enabled\n");
            return -EINVAL;
        }

        ret = kstrtoll(buf, 0, &value);
        if (ret) {
            slog("Error whilst reading value\n");
            return -EINVAL;
        }

        if (_dw3000_otp_read(inst, address) != 0) {
            slog("OTP Write abort, address has already been written to\n");
            return -EINVAL;
        }
        /* Write */
        slog("Writing 0x%08"PRIX32" to OTP[0x%02X]\n", (u32)value, ed->otp.write_en_address);
        ret = dw3000_phy_otp_writeandverify(inst, value, ed->otp.write_en_address);
        slog("Write completed with return value %d:%s\n", ret, ret==DPL_OK?"OK":"ERR");
    }

    return count;
}

#define DEV_ATTR_ROW(__X)                                               \
    (struct kobj_attribute){                                            \
        .attr = {.name = __X,                                           \
                 .mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO)},            \
            .show = cmd_show, .store = cmd_store}
#define DEV_ATTR_ROW_W(__X)                                             \
    (struct kobj_attribute){                                            \
        .attr = {.name = __X,                                           \
                 .mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO|S_IWUSR|S_IWGRP)}, \
            .show = cmd_show, .store = cmd_store}
#define OTP_DEV_ATTR(__X)                                               \
    (struct kobj_attribute){                                            \
        .attr = {.name = __X,                                           \
                 .mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO|S_IWUSR|S_IWGRP)}, \
                 .show = otp_show, .store = otp_store}

/* Read only interface(s) */
static const char* ro_attr[] = {
    "device_id",
    "rssi",
};

/* Read-Write interfaces */
static const char* rw_attr[] = {
    "addr",
    "uid",
    "panid",
    "euid",
    "cw",
    "autorx",
    "continuoustx",
    "continuoustx_framelen",
    "antmux",
    "ev",
    "xtal",
    "gpio0_mode", "gpio1_mode", "gpio2_mode",
    "gpio3_mode", "gpio4_mode", "gpio5_mode",
    "gpio6_mode", "gpio7_mode", "gpio8_mode",
    "gpio0_dir", "gpio1_dir", "gpio2_dir",
    "gpio3_dir", "gpio4_dir", "gpio5_dir",
    "gpio6_dir", "gpio7_dir", "gpio8_dir",
    "gpio0", "gpio1", "gpio2",
    "gpio3", "gpio4", "gpio5",
    "gpio6", "gpio7", "gpio8"
};

#define MAX_OTP_ADDR (0x7F)
#define NUM_FIXED_OTP_ATTR (1)
static const char* otp_attr[] = {
    "write_enable"
};

int dw3000_sysfs_init(struct _dw3000_dev_instance_t *inst)
{
    int ret, i, k;
    struct dw3000cli_sysfs_data *ed;
    if (inst->uwb_dev.idx >= ARRAY_SIZE(dw3000cli_sysfs_inst)) {
        return -EINVAL;
    }
    ed = &dw3000cli_sysfs_inst[inst->uwb_dev.idx];
    ed->inst = inst;
    ed->cont_tx_frame_len = 10;

    snprintf(ed->device_name, sizeof(ed->device_name)-1, "dw3000_cli%d", inst->uwb_dev.idx);
    ed->kobj = kobject_create_and_add(ed->device_name, uwbcore_get_kobj());
    if (!ed->kobj) {
        slog("Failed to create %s\n", ed->device_name);
        return -ENOMEM;
    }

    ed->cli.dev_attr_cfg = kzalloc((ARRAY_SIZE(ro_attr) + ARRAY_SIZE(rw_attr))*sizeof(struct kobj_attribute), GFP_KERNEL);
    if (!ed->cli.dev_attr_cfg) {
        slog("Failed to create dev_attr_cfg\n");
        goto err_dev_attr_cfg;
    }

    ed->cli.attrs = kzalloc((ARRAY_SIZE(ro_attr) + ARRAY_SIZE(rw_attr) + 1)*sizeof(struct attribute *), GFP_KERNEL);
    if (!ed->cli.attrs) {
        slog("Failed to create sysfs_attrs\n");
        goto err_sysfs_attrs;
    }

    k=0;
    for (i=0;i<ARRAY_SIZE(ro_attr);i++) {
        ed->cli.dev_attr_cfg[k] = DEV_ATTR_ROW(ro_attr[i]);
        ed->cli.attrs[k] = &ed->cli.dev_attr_cfg[k].attr;
        k++;
    }
    for (i=0;i<ARRAY_SIZE(rw_attr);i++) {
        ed->cli.dev_attr_cfg[k] = DEV_ATTR_ROW_W(rw_attr[i]);
        ed->cli.attrs[k] = &ed->cli.dev_attr_cfg[k].attr;
        k++;
    }

    ed->cli.attribute_group.attrs = ed->cli.attrs;
    ret = sysfs_create_group(ed->kobj, &ed->cli.attribute_group);
    if (ret) {
        slog("Failed to create sysfs group\n");
        goto err_sysfs;
    }

    /* OTP */
    ed->otp.write_en_address = 0xffff;
    ed->kobj_otp = kobject_create_and_add("otp", ed->kobj);
    if (!ed->kobj_otp) {
        slog("Failed to create otp/\n");
        goto err_sysfs;
    }

    ed->otp.reg_name = kzalloc((MAX_OTP_ADDR + 1) * OTP_REG_NAME_MAXLEN, GFP_KERNEL);
    if (!ed->otp.reg_name) {
        slog("Failed to allocate reg_name\n");
        goto err_otp_reg_name;
    }

    ed->otp.dev_attr_cfg = kzalloc((ARRAY_SIZE(otp_attr) + MAX_OTP_ADDR + 1)*sizeof(struct kobj_attribute), GFP_KERNEL);
    if (!ed->otp.dev_attr_cfg) {
        slog("Failed to create otp-dev_attr_cfg\n");
        goto err_otp_attr_cfg;
    }

    ed->otp.attrs = kzalloc((ARRAY_SIZE(otp_attr) + MAX_OTP_ADDR + 2)*sizeof(struct attribute *), GFP_KERNEL);
    if (!ed->otp.attrs) {
        slog("Failed to create sysfs_attrs\n");
        goto err_otp_sysfs_attrs;
    }

    k=0;
    for (i=0;i<ARRAY_SIZE(otp_attr);i++) {
        ed->otp.dev_attr_cfg[k] = OTP_DEV_ATTR(otp_attr[i]);
        ed->otp.attrs[k] = &ed->otp.dev_attr_cfg[k].attr;
        k++;
    }
    for (i=0;i<(MAX_OTP_ADDR+1);i++) {
        snprintf(ed->otp.reg_name + i*OTP_REG_NAME_MAXLEN, OTP_REG_NAME_MAXLEN-1, "0x%02X", i);
        ed->otp.dev_attr_cfg[k] = OTP_DEV_ATTR(ed->otp.reg_name + i*OTP_REG_NAME_MAXLEN);
        ed->otp.attrs[k] = &ed->otp.dev_attr_cfg[k].attr;
        k++;
    }

    ed->otp.attribute_group.attrs = ed->otp.attrs;
    ret = sysfs_create_group(ed->kobj_otp, &ed->otp.attribute_group);
    if (ret) {
        slog("Failed to create otp sysfs group\n");
        goto err_otp_sysfs;
    }

    return 0;

err_otp_sysfs:
    kfree(ed->otp.attrs);
err_otp_sysfs_attrs:
    kfree(ed->otp.dev_attr_cfg);
err_otp_attr_cfg:
    kobject_put(ed->kobj_otp);
    kfree(ed->otp.reg_name);
err_otp_reg_name:
    sysfs_remove_group(ed->kobj, &ed->cli.attribute_group);
err_sysfs:
    kfree(ed->cli.attrs);
err_sysfs_attrs:
    kfree(ed->cli.dev_attr_cfg);
err_dev_attr_cfg:
    kobject_put(ed->kobj);
    return -ENOMEM;
}

void dw3000_sysfs_deinit(int idx)
{
    struct dw3000cli_sysfs_data *ed;
    if (idx >= ARRAY_SIZE(dw3000cli_sysfs_inst)) {
        return;
    }
    ed = &dw3000cli_sysfs_inst[idx];

    if (ed->kobj) {
        slog("");
        mutex_destroy(&ed->local_lock);
        sysfs_remove_group(ed->kobj, &ed->cli.attribute_group);
        kfree(ed->cli.attrs);
        kfree(ed->cli.dev_attr_cfg);
        sysfs_remove_group(ed->kobj_otp, &ed->otp.attribute_group);
        kfree(ed->otp.attrs);
        kfree(ed->otp.dev_attr_cfg);
        kfree(ed->otp.reg_name);
        kobject_put(ed->kobj_otp);
        kobject_put(ed->kobj);
    }
}
#endif
