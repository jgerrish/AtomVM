/*
 * This file is part of AtomVM.
 *
 * Copyright 2026 Joshua Gerrish <jgerrish@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
 */

#ifndef _I2S_DRIVER_H_
#define _I2S_DRIVER_H_

#include <sdkconfig.h>
#ifdef CONFIG_AVM_ENABLE_I2S_PORT_DRIVER

#include <driver/i2s_std.h>

struct I2SChannelData
{
    /* Store the list head in the I2SChannelData itself */
    /* Then we can look it up quickly and remove it */
    struct ListHead head;
    i2s_chan_handle_t channel;
};

/**
 * TODO: There might need to be some enum or structure name changes in
 * i2s_driver.c to meet the C style guidelines.  enums use Camel case
 * which matches some other modules but in the C style documentation
 * lower snake-case with _t suffix is recommended.
 */
typedef enum
{
    InvalidDuplex = -1,
    FullDuplex = 1,
    HalfDuplex = 0,
} DuplexMode;

struct I2SData
{
    struct ListHead head;

    bool tx_enabled, rx_enabled;

    /* I2S tx channel handler */
    struct I2SChannelData tx_chan;
    /* I2S rx channel handler */
    struct I2SChannelData rx_chan;

    DuplexMode duplex_mode;

    /* Some of this is redundant, maybe refactor */
    i2s_std_config_t std_cfg, tx_std_cfg, rx_std_cfg;

    /* TODO: IMPORTANT Review this and other concurrency requirements */
    int ref_count;
};

#endif

#endif
