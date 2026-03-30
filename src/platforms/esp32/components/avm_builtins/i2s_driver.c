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

/*
 * This file is based on the I2C driver, and some of the code in the
 * mailbox handler and other functions is directly copied from it.  It
 * is currently very feature incomplete.  It includes basic driver /
 * port creation and destruction callbacks but no data reading or
 * writing.  It includes the following:
 *
 * A basic AtomVM port / driver for i2c: i2c_driver.c and i2c_driver.h
 * avm_esp32 library support with i2s.erl
 *
 * New eavmlib library HAL support for the driver with i2s_hal.erl
 *
 * Test skeleton with test_i2s.erl
 *
 * Example Erlang and Elixir applications for the Texas Instruments
 * PCM5102 I2S Stereo DAC: pcm5102.erl and PCM5102.ex
 *
 *
 * Current issues:
 *
 * I wanted to get a basic port driver up and running quickly.
 *
 * The info logging is verbose and needs to be tuned.
 *
 * There might need to be some enum or structure name changes in
 * i2s_driver.c to meet the C style guidelines.  enums use Camel case
 * which matches some other modules but in the C style documentation
 * lower snake-case with _t suffix is recommended.
 *
 * Port data read and write commands need to be added to the driver.
 * And any other appropriate commands for I2S functionality needs to
 * be added.
 *
 *
 *
 * Locking and concurrency requirements need to be examined for each
 * of the additional commands and initialization and destruction.
 * There are several models of locking I've seen in the existing
 * peripheral driver codebases.  Locking with
 * globalcontext_get_process_unlock in each individual command
 * function like the I2C driver and locking around the mailbox message
 * handler as in the SPI driver.  I honestly need to do further
 * research on the locking model in the BEAM and Erlang and the ESP32
 * I2S module concurrency requirements to understand this.
 *
 * TODO: When power management is enabled, there are locking
 * requirements for I2S.  See the Power Management section in
 * docs/en/api-reference/peripherals/i2s.rst in the ESP-iDF source for
 * additional details. It looks like this is maybe already handled by
 * the ESP-IDF driver.
 *
 * From the above document in the Thread Safety section: "All of the
 * I2S APIs are guaranteed to be thread safe by the driver, which
 * means users can call them from different RTOS tasks without
 * protection by extra locks.  Notice that the I2S driver uses mutex
 * locks to ensure thread safety, thus these APIs are not allowed to
 * be used in ISR."  So I think we're actually safe in terms of thread
 * safety.
 *
 * BUT this doesn't mean the behavior will be what is expected.  If
 * users create multiple ports, they will expect those ports to have
 * the same configuration and behavior between actions.  So, if we
 * allow multiple I2S ports, we will need to save and restore the
 * state of the I2S device between calls.
 *
 * That's a lot of state management and increases the scope a lot.
 *
 * TODO: Investigate some scope management options:
 *   1. Allowing only one port of a type at a time in Erlang / BEAM /
 *      AtomVM.
 *   2. Have a spec that allows very coarse-grained state management.
 *      e.g.  Buffers have to be flushed before switching ports. Only
 *      the basic configuration of the I2S driver is preserved between
 *      port action switches, no in-transit or buffered data.  I don't
 *      know how this would work with pre-emptive BEAM stuff.
 *
 *
 * Error handling needs to be more fine-grained and comprehensive.
 * Don't use ESP_ERROR_CHECK, use custom error-handling code.
 *
 */
#include <sdkconfig.h>
#ifdef CONFIG_AVM_ENABLE_I2S_PORT_DRIVER

#include <string.h>

#include <driver/i2s_std.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "atom.h"
#include "bif.h"
#include "context.h"
#include "debug.h"
#include "defaultatoms.h"
#include "globalcontext.h"
#include "interop.h"
#include "mailbox.h"
#include "module.h"
#include "platform_defaultatoms.h"
#include "port.h"
#include "scheduler.h"
#include "term.h"
#include "utils.h"

// #define ENABLE_TRACE
#include "trace.h"

#include "esp32_sys.h"
#include "sys.h"

#include "include/i2s_driver.h"

#define TAG "i2s_driver"

static const char *const rx_enabled_atom = ATOM_STR("\x0A", "rx_enabled");
static const char *const tx_enabled_atom = ATOM_STR("\x0A", "tx_enabled");
static const char *const duplex_mode_atom = ATOM_STR("\x0B", "duplex_mode");
static const char *const bclk_io1_atom = ATOM_STR("\x08", "bclk_io1");
static const char *const ws_io1_atom = ATOM_STR("\x06", "ws_io1");
static const char *const dout_io1_atom = ATOM_STR("\x08", "dout_io1");
static const char *const din_io1_atom = ATOM_STR("\x07", "din_io1");
static const char *const bclk_io2_atom = ATOM_STR("\x08", "bclk_io2");
static const char *const ws_io2_atom = ATOM_STR("\x06", "ws_io2");
static const char *const dout_io2_atom = ATOM_STR("\x08", "dout_io2");
static const char *const din_io2_atom = ATOM_STR("\x07", "din_io2");
static const char *const buff_size_atom = ATOM_STR("\x09", "buff_size");

static const AtomStringIntPair duplex_mode_table[] = {
    { ATOM_STR("\x0B", "full_duplex"), FullDuplex },
    { ATOM_STR("\x0B", "half_duplex"), HalfDuplex },
    SELECT_INT_DEFAULT(InvalidDuplex)
};

static void i2s_driver_init(GlobalContext *global);
static void i2s_driver_destroy(GlobalContext *global);
static Context *i2s_driver_create_port(GlobalContext *global, term opts);

static NativeHandlerResult i2sdriver_consume_mailbox(Context *ctx);

static const char *const i2s_driver_atom = ATOM_STR("\xA", "i2s_driver");
static term i2s_driver;

enum i2s_cmd
{
    I2SInvalidCmd = 0,
    I2SCloseCmd = 1,
    I2SDumpCmd = 2,
};

static const AtomStringIntPair cmd_table[] = {
    { ATOM_STR("\x5", "close"), I2SCloseCmd },
    { ATOM_STR("\x4", "dump"), I2SDumpCmd },
    SELECT_INT_DEFAULT(I2SInvalidCmd)
};

struct I2SConfig
{
    bool rx_enabled;
    bool tx_enabled;
    DuplexMode duplex_mode;
    int32_t bclk_io1;
    int32_t ws_io1;
    int32_t dout_io1;
    int32_t din_io1;

    int32_t bclk_io2;
    int32_t ws_io2;
    int32_t dout_io2;
    int32_t din_io2;

    int32_t buff_size;
};

void log_i2s_config(const struct I2SConfig *config)
{
    ESP_LOGI(TAG, "Parsed I2S config:");

    ESP_LOGI(TAG, "rx_enabled: %d", config->rx_enabled);
    ESP_LOGI(TAG, "tx_enabled: %d", config->tx_enabled);
    ESP_LOGI(TAG, "duplex_mode: %d", config->duplex_mode);
    ESP_LOGI(TAG, "bclk_io1: %d", config->bclk_io1);
    ESP_LOGI(TAG, "ws_io1: %d", config->ws_io1);
    ESP_LOGI(TAG, "dout_io1: %d", config->dout_io1);
    ESP_LOGI(TAG, "din_io1: %d", config->din_io1);

    ESP_LOGI(TAG, "bclk_io2: %d", config->bclk_io2);
    ESP_LOGI(TAG, "ws_io2: %d", config->ws_io2);
    ESP_LOGI(TAG, "dout_io2: %d", config->dout_io2);
    ESP_LOGI(TAG, "din_io2: %d", config->din_io2);

    ESP_LOGI(TAG, "buff_size: %d", config->buff_size);
}

/**
 * Set a bool variable true for TRUE_ATOM and false for a FALSE_ATOM.
 *
 * This function is based on code from socket_driver.c
 *
 * It changes the return result from that in socket_driver.c.  The
 * function in socket_driver.c returns false if b is a FALSE_ATOM or
 * an unexpected atom type.
 *
 * @param b the term to parse into a bool
 * @param ok the boolean to set to true or false depending on whether
 *           the term is TRUE_ATOM or FALSE_ATOM
 *
 * @return Returns true if the term is actually TRUE_ATOM or FALSE_ATOM
 *         Returns false if it is neither of these.
 */
static bool bool_term_to_bool(term b, bool *ok)
{
    switch (b) {
        case TRUE_ATOM:
            *ok = true;
            return true;

        case FALSE_ATOM:
            *ok = true;
            return true;

        default:
            *ok = false;
            return false;
    }
}

/**
 * @brief Initialize the I2S driver
 *
 * @details Initialize the I2S driver.
 *
 * @param global the global context
 */
void i2s_driver_init(GlobalContext *global)
{
    TRACE("Initializing I2S driver\n");
    ESP_LOGI(TAG, "Initializing I2S driver");

    i2s_driver = globalcontext_make_atom(global, i2s_driver_atom);

    struct ESP32PlatformData *platform = global->platform_data;
    synclist_init(&platform->i2s_channels);

    TRACE("I2S driver init: done\n");
    ESP_LOGI(TAG, "I2S driver init: done");
}

void i2s_driver_do_channel_delete(i2s_chan_handle_t *channel)
{
    /* This relies on the ordering of the I2S state machine, so it
     * might be a little fragile.  But it is the same method used
     * in the esp-idf code. It relies on private headers and
     * access to atomic_load. */
    /* if (atomic_load(channel->channel->state) >= I2S_CHAN_STATE_RUNNING) */

    /* We just log the result of the i2s_channel_disable call.
     * It's not a good practice, but we don't have clean access to
     * the state of the I2S channel without bringing in private
     * headers.  The only failure path in i2s_channel_disable
     * right now is if the channel not disabled, but that may
     * change in the future, so it's still a bad idea to ignore
     * the error.
     *
     * We could also keep a separate state machine to track
     * disabled/enabled.
     */
    esp_err_t res = i2s_channel_disable(*channel);
    if (UNLIKELY(res != ESP_OK)) {
        ESP_LOGW(TAG, "I2S attempted to disable channel that was not enabled");
    }

    i2s_del_channel(*channel);
}

/**
 * @brief Destroy the I2S driver
 *
 * @details Destroy the I2S driver
 *
 * @param global the global context
 *
 * @warning There's no easy way to cleanup the I2S hardware without
 *          access to additional state here.  So we use a synclist.
 *
 *          This may break the Erlang process encapsulation model of
 *          no shared state.  Some relevant details: We have a
 *          separate context for each port we create.  This should be
 *          reviewed some more.
 *
 *          But other than that the behavior between this and
 *          i2s_driver_unref should be correct.  We want to be able to
 *          call both at anytime, including the expected case where
 *          all "close" messages are received and there are no
 *          channels to delete in here AND the unexpected case where
 *          some channels are closed by port actions but not all AND
 *          the case where no channels are closed by port mailbox
 *          messages.
 */
void i2s_driver_destroy(GlobalContext *global)
{
    TRACE("Destroying I2S driver\n");
    ESP_LOGI(TAG, "Destroying I2S driver");

    struct ESP32PlatformData *platform = global->platform_data;
    struct ListHead *i2s_channels = synclist_wrlock(&platform->i2s_channels);
    struct ListHead *item;
    struct ListHead *tmp;
    MUTABLE_LIST_FOR_EACH (item, tmp, i2s_channels) {
        struct I2SChannelData *channel = GET_LIST_ENTRY(item, struct I2SChannelData, head);
        i2s_driver_do_channel_delete(&channel->channel);
        list_remove(item);
    }
    synclist_unlock(&platform->i2s_channels);

    /* Finally destroy the synclist which was initialized in
     * i2s_driver_init */
    synclist_destroy(&platform->i2s_channels);

    /* We may not have manually sent a kill signal after receiving a
     * "close" mailbox message.  Hopefully AtomVM kills the context
     * eventually and frees up the I2SData.  We don't have access to
     * the context, so we can't destroy it from here. */

    TRACE("I2S driver destroy: done\n");
    ESP_LOGI(TAG, "I2S driver destroy: done");
}

/**
 * @brief create a new channel
 *
 * @param global the global context
 * @param chan_cfg the standard configuration for the channel
 * @param channel the I2S channel data structure
 *
 * TODO: Review chan_cfg and i2s_chan_config_t vs i2s_std_config_t
 * I may have a mistake there
 *
 * TODO: Rename this (it calls i2s_channel_init_std_mode, NOT i2s_new_channel)
 * TODO: Refactor so that it or another function really does call
 *       i2s_new_channel to prevent out-of-sync channel creation
 *       failures.
 */
void i2s_driver_new_channel(GlobalContext *global, i2s_std_config_t *chan_cfg,
    struct I2SChannelData *channel)
{
    ESP_LOGI(TAG, "I2S initializing channel");

    struct ESP32PlatformData *platform = global->platform_data;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(channel->channel, chan_cfg));
    synclist_append(&platform->i2s_channels, &channel->head);

    ESP_LOGI(TAG, "I2S done initializing channel");
}

/**
 * @brief delete a channel
 *
 * @param global the global context
 * @param channel the I2SChannelData
 */
void i2s_driver_del_channel(GlobalContext *global, struct I2SChannelData *channel)
{
    TRACE("I2S deleting channel\n");
    ESP_LOGI(TAG, "I2S deleting channel");

    struct ESP32PlatformData *platform = global->platform_data;

    i2s_driver_do_channel_delete(&channel->channel);

    synclist_remove(&(platform->i2s_channels), &(channel->head));
    /* We don't need to free the item here.  The I2SChannelData is
     * owned by the I2SData structure, which is allocated on
     * driver init and freed on driver destruction. */
    /* TODO: We could allocate memory for each channel separately, since
       both channels are not always needed. */
    /* free(item); */

    TRACE("I2S deleted channel: done\n");
    ESP_LOGI(TAG, "I2S deleted channel: done");
}

static NativeHandlerResult i2s_driver_unref(Context *ctx)
{
    TRACE("I2S deleting I2S driver\n");
    ESP_LOGI(TAG, "I2S deleting I2S driver");

    if (ctx == NULL) {
        return NativeTerminate;
    }

    Context *target = globalcontext_get_process_lock(ctx->global, ctx->process_id);

    struct I2SData *i2s_data = ctx->platform_data;

    if (i2s_data == NULL) {
        goto i2s_driver_unref_unlock_and_exit;
    }

    ctx->native_handler = NULL;
    ctx->platform_data = NULL;

    if (i2s_data->rx_enabled) {
        i2s_driver_del_channel(ctx->global, &i2s_data->rx_chan);
    }
    if (i2s_data->tx_enabled) {
        i2s_driver_del_channel(ctx->global, &i2s_data->tx_chan);
    }

    /* Sending a KillSignal sets the Killed flag in
     * context_process_kill_signal.  The Killed flag is checked on a
     * scheduler run and context_destroy is called if it is
     * set. context_destroy destroys the context, freeing the Context
     * platform_data, which is I2SData, so we don't need to do it
     * manually. */
    /* TODO: Read up and see which atom makes sense as the last
     * argument */
    mailbox_send_term_signal(target, KillSignal, NORMAL_ATOM);

i2s_driver_unref_unlock_and_exit:
    globalcontext_get_process_unlock(ctx->global, target);

    ESP_LOGI(TAG, "I2S deleted I2S driver");

    return NativeTerminate;
}

static NativeHandlerResult i2s_driver_dump(Context *ctx)
{
    /* TRACE("I2S dumping I2S driver state\n"); */
    ESP_LOGI(TAG, "I2S dumping I2S driver state");

    if (ctx == NULL) {
        return NativeTerminate;
    }

    Context *target = globalcontext_get_process_lock(ctx->global, ctx->process_id);

    struct ESP32PlatformData *platform = ctx->global->platform_data;

    /* TODO: Dump the driver state here */
    /* TODO: Look into the Erlang process_info API and how we can
     * provide compatible information here. */

    globalcontext_get_process_unlock(ctx->global, target);

    /* TRACE("I2S dumped I2S driver state\n"); */
    ESP_LOGI(TAG, "I2S dumped I2S driver state");

    return NativeContinue;
}

typedef enum
{
    ChannelInitOk,
    ChannelInitError
} ChannelInitResult;

/**
 * TODO: Return more fine-grained errors so they can be used
 * TODO: Don't use ESP_ERROR_CHECK, use custom error-handling code
 *
 * This code is based on the ESP-IDF i2s_std_example_main.c code
 */
ChannelInitResult i2s_init_std_full_duplex(Context *ctx)
{
    ESP_LOGI(TAG, "Initializing I2S channels in full-duplex mode");
    struct I2SData *i2s_data = ctx->platform_data;

    struct GlobalContext *global = ctx->global;
    /* struct ESP32PlatformData *platform = global->platform_data; */

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg,
        i2s_data->tx_enabled ? &i2s_data->tx_chan.channel : NULL,
        i2s_data->tx_enabled ? &i2s_data->rx_chan.channel : NULL));

    /* Initialize the channels */
    if (i2s_data->tx_enabled) {
        i2s_driver_new_channel(global, &i2s_data->std_cfg, &i2s_data->tx_chan);
    }
    if (i2s_data->rx_enabled) {
        i2s_driver_new_channel(global, &i2s_data->std_cfg, &i2s_data->rx_chan);
    }

    return ChannelInitOk;
}

/**
 * TODO: Return more fine-grained errors so they can be used
 * TODO: Don't use ESP_ERROR_CHECK, use custom error-handling code
 *
 * This code is based on the ESP-IDF i2s_std_example_main.c code
 */
ChannelInitResult i2s_init_std_half_duplex(Context *ctx)
{
    ESP_LOGI(TAG, "Initializing I2S channels in half-duplex mode");
    GlobalContext *global = ctx->global;
    struct I2SData *i2s_data = ctx->platform_data;

    if (i2s_data->tx_enabled) {
        ESP_LOGI(TAG, "Building I2S TX channel config with I2S_NUM_AUTO: %d and I2S_ROLE_MASTER: %d",
            I2S_NUM_AUTO, I2S_ROLE_MASTER);
        i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
        ESP_LOGI(TAG, "Creating new I2S TX channel with tx_chan_cfg: %p", tx_chan_cfg);

        ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &(i2s_data->tx_chan.channel), NULL));
    }
    if (i2s_data->rx_enabled) {
        ESP_LOGI(TAG, "Building I2S RX channel config with I2S_NUM_AUTO: %d and I2S_ROLE_MASTER: %d",
            I2S_NUM_AUTO, I2S_ROLE_MASTER);
        i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
        ESP_LOGI(TAG, "Creating new I2S RX channel with tx_chan_cfg: %p", rx_chan_cfg);
        ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &(i2s_data->rx_chan.channel)));
    }

    if (i2s_data->tx_enabled) {
        ESP_LOGI(TAG, "Initializing standard mode I2S RX channel with tx_chan: %p",
            i2s_data->tx_chan);

        i2s_driver_new_channel(global, &i2s_data->tx_std_cfg, &i2s_data->tx_chan);
    }

    /* Default is only receiving left slot in mono mode,
     * update to right here to show how to change the default configuration */
    if (i2s_data->rx_enabled) {
        i2s_data->rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
        i2s_driver_new_channel(global, &i2s_data->rx_std_cfg, &i2s_data->rx_chan);
    }

    return ChannelInitOk;
}

typedef enum
{
    TermParseOk,
    TermParseInvalidTerm,
    TermParseError
} TermParseResult;

typedef enum
{
    ConfigParseOk,
    ConfigParseError
} ConfigParseResult;

/**
 * @brief Get an int32_t term from the options
 *
 * @param global The global context
 * @param opts The options passed in to the port create call
 * @param key the key of the option
 * @param value where to place to found the value of the found option
 * @return A TermParseResult indicating success or failure
 */
TermParseResult get_int32_term(GlobalContext *global, term opts,
    const char *key, int32_t *value)
{
    TermParseResult result = TermParseError;

    term value_term = interop_kv_get_value(opts, key, global);

    if (term_is_invalid_term(value_term)) {
        ESP_LOGE(TAG, "Invalid parameter: term %s is an invalid term", key);
        return TermParseInvalidTerm;
    }

    if (!term_is_int(value_term)) {
        ESP_LOGE(TAG, "Invalid parameter: term %s is not an integer", key);
        return result;
    }
    if (!term_is_int32(value_term)) {
        ESP_LOGE(TAG, "Invalid parameter: term %s will not fit in an int32_t", key);
        return result;
    }

    result = TermParseOk;
    *value = term_to_int32(value_term);

    return result;
}

/**
 * @brief Get a term from the options
 *
 * This function performs the first part of retriving a term from a
 * term dictionary.  it doesn't do any validation besides checking if
 * the term is a valid term.
 *
 * @param global The global context
 * @param opts The options passed in to the port create call
 * @param key the key of the option
 * @param value where to place to found the value of the found option
 * @return A TermParseResult indicating success or failure
 */
TermParseResult get_term(GlobalContext *global, term opts,
    const char *key, term *value)
{
    *value = interop_kv_get_value(opts, key, global);

    if (term_is_invalid_term(*value)) {
        ESP_LOGE(TAG, "Invalid parameter: term %s is an invalid term", key);
        return TermParseInvalidTerm;
    }

    return TermParseOk;
}

/**
 * @brief Get a bool term from the options
 *
 * @param global The global context
 * @param opts The options passed in to the port create call
 * @param key the key of the option
 * @param value where to place to found the value of the found option
 * @return A TermParseResult indicating success or failure
 */
TermParseResult get_bool_term(GlobalContext *global, term opts,
    const char *key, bool *value)
{
    TermParseResult result = TermParseError;

    term value_term = interop_kv_get_value(opts, key, global);

    if (term_is_invalid_term(value_term)) {
        ESP_LOGE(TAG, "Invalid parameter: term %s is an invalid term", key);
        return TermParseInvalidTerm;
    }

    if (!bool_term_to_bool(value_term, value)) {
        ESP_LOGE(TAG, "Invalid parameter: term %s is an invalid bool", key);
        return TermParseInvalidTerm;
    }

    result = TermParseOk;

    return result;
}

/**
 * @brief Parse the configuration for the I2S port
 *
 * TODO: This parse_config function changes not only the config object
 * but also some fields on ctx->platform data.  This either needs to
 * be refactored or documented better.
 *
 * @param ctx the context object
 * @param opts the options passed in to the port create call
 * @param config an I2SConfig object to fill with the configuration
 * @return a ConfigParseResult indicating success or failure
 */
ConfigParseResult parse_config(Context *ctx, term opts, struct I2SConfig *config)
{
    GlobalContext *global = ctx->global;
    struct I2SData *i2s_data = ctx->platform_data;

    if (UNLIKELY(i2s_data == NULL)) {
        ESP_LOGE(TAG, "Invalid I2SData structure");
        return ConfigParseError;
    }

    if (get_bool_term(global, opts, rx_enabled_atom, &(config->rx_enabled)) != TermParseOk) {
        return ConfigParseError;
    }
    if (get_bool_term(global, opts, tx_enabled_atom, &(config->tx_enabled)) != TermParseOk) {
        return ConfigParseError;
    }

    term duplex_mode_term;
    if (get_term(global, opts, duplex_mode_atom, &duplex_mode_term) != TermParseOk) {
        return ConfigParseError;
    }

    config->duplex_mode = interop_atom_term_select_int(duplex_mode_table, duplex_mode_term, global);
    if (UNLIKELY(config->duplex_mode == InvalidDuplex)) {
        return ConfigParseError;
    }

    if (get_int32_term(global, opts, bclk_io1_atom, &(config->bclk_io1)) != TermParseOk) {
        return ConfigParseError;
    }
    if (get_int32_term(global, opts, ws_io1_atom, &(config->ws_io1)) != TermParseOk) {
        return ConfigParseError;
    }
    if (get_int32_term(global, opts, dout_io1_atom, &(config->dout_io1)) != TermParseOk) {
        return ConfigParseError;
    }
    if (get_int32_term(global, opts, din_io1_atom, &(config->din_io1)) != TermParseOk) {
        return ConfigParseError;
    }

    i2s_data->rx_enabled = config->rx_enabled;
    i2s_data->tx_enabled = config->tx_enabled;

    i2s_data->duplex_mode = config->duplex_mode;

    if ((i2s_data->duplex_mode == HalfDuplex) && i2s_data->rx_enabled) {
        if (get_int32_term(global, opts, bclk_io2_atom, &(config->bclk_io2)) != TermParseOk) {
            return ConfigParseError;
        }
        if (get_int32_term(global, opts, ws_io2_atom, &(config->ws_io2)) != TermParseOk) {
            return ConfigParseError;
        }
        if (get_int32_term(global, opts, dout_io2_atom, &(config->dout_io2)) != TermParseOk) {
            return ConfigParseError;
        }
        if (get_int32_term(global, opts, din_io2_atom, &(config->din_io2)) != TermParseOk) {
            return ConfigParseError;
        }
    }

    if (get_int32_term(global, opts, buff_size_atom, &(config->buff_size)) != TermParseOk) {
        return ConfigParseError;
    }

    return ConfigParseOk;
}

/**
 * @brief Initialize the I2S channels
 *
 * @param ctx The context object
 * @param config An I2SConfig object with I2S configuration
 * @return A ChannelInitResult indicating success or failure
 */
ChannelInitResult init_channels(Context *ctx, struct I2SConfig *config)
{
    ESP_LOGI(TAG, "Initializing I2S channels");
    struct I2SData *i2s_data = ctx->platform_data;

    ChannelInitResult channel_init_result = ChannelInitOk;

    if (i2s_data->duplex_mode == FullDuplex) {
        /* TODO: Decide how to deal with rx_enabled and tx_enabled
         * here */
        i2s_std_config_t std_cfg = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
            .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = config->bclk_io1,
                .ws = config->ws_io1,
                .dout = config->dout_io1,
                .din = config->dout_io1,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
            },
        };

        i2s_data->std_cfg = std_cfg;
        channel_init_result = i2s_init_std_full_duplex(ctx);
    } else {
        if (i2s_data->tx_enabled) {
            i2s_std_config_t tx_std_cfg = {
                .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
                .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
                .gpio_cfg = {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = config->bclk_io1,
                    .ws = config->ws_io1,
                    .dout = config->dout_io1,
                    .din = config->din_io1,
                    .invert_flags = {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
                },
            };
            i2s_data->tx_std_cfg = tx_std_cfg;
        }

        if (i2s_data->rx_enabled) {
            i2s_std_config_t rx_std_cfg = {
                .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
                .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
                .gpio_cfg = {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = config->bclk_io2,
                    .ws = config->ws_io2,
                    .dout = config->dout_io2,
                    .din = config->din_io2,
                    .invert_flags = {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
                },
            };
            i2s_data->rx_std_cfg = rx_std_cfg;
            /* Default is only receiving left slot in mono mode,
             * update to right here to show how to change the default configuration */
            i2s_data->rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
        }
        channel_init_result = i2s_init_std_half_duplex(ctx);
    }

    return channel_init_result;
}

/**
 * @brief Create a port for the I2S driver
 *
 * @param global the global context
 * @param opts a key-value store of configuration options
 *
 * @return the task context
 *
 * @warning the returned context must be destroyed with
 *          context_destroy in the driver destroy callback and/or
 *          after receiving a "close" message.
 *
 * @warning i2s_data is allocated here but is owned by the Context
 *          object.  Calling destroy_context will free i2s_data.
 */
Context *i2s_driver_create_port(GlobalContext *global, term opts)
{
    ESP_LOGI(TAG, "Creating port for i2s driver");

    /* struct ESP32PlatformData *platform = global->platform_data; */

    Context *ctx = NULL;

    struct I2SData *i2s_data = calloc(1, sizeof(struct I2SData));
    if (UNLIKELY(i2s_data == NULL)) {
        ESP_LOGE(TAG, "Couldn't allocate I2SData structure");
        goto free_and_exit;
    }

    i2s_data->ref_count = 1;

    ctx = context_new(global);
    if (UNLIKELY(ctx == NULL)) {
        ESP_LOGE(TAG, "Couldn't create new context");
        goto free_and_exit;
    }
    ctx->native_handler = i2sdriver_consume_mailbox;
    ctx->platform_data = i2s_data;

    struct I2SConfig config;
    ConfigParseResult config_parse_result = parse_config(ctx, opts, &config);
    if (UNLIKELY(config_parse_result != ConfigParseOk)) {
        ESP_LOGE(TAG, "Failed to parse I2S parameters.");
        goto free_and_exit;
    } else {
        ESP_LOGI(TAG, "Parsed I2S parameters");
    }

    ChannelInitResult init_channels_result = init_channels(ctx, &config);

    if (UNLIKELY(init_channels_result != ChannelInitOk)) {
        ESP_LOGE(TAG, "Failed to initialize I2S parameters.");
        goto free_and_exit;
    } else {
        ESP_LOGI(TAG, "I2S driver initialized");
    }

    return ctx;

free_and_exit:
    if (ctx) {
        ctx->native_handler = NULL;
        /* context_destroy frees platform_data, so no need to free
         * i2s_data too */
        context_destroy(ctx);
    } else {
        free(i2s_data);
    }

    return NULL;
}

/* TODO: Additional port actions need to be added for reading and writing to the port,
 * and any additional I2S appropriate actions. */

static term create_pair(Context *ctx, term term1, term term2)
{
    term ret = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(ret, 0, term1);
    term_put_tuple_element(ret, 1, term2);

    return ret;
}

/**
 * @brief Consume a single mailbox message
 *
 * TODO: Locking and concurrency requirements need to be examined for
 * each of the additional commands and initialization and destruction.
 * There are several models of locking I've seen in the existing
 * peripheral driver codebases.  Locking with
 * globalcontext_get_process_unlock in each individual command
 * function like the I2C driver and locking around the mailbox message
 * handler as in the SPI driver.  I honestly need to do further
 * research on the locking model in the BEAM and Erlang and the ESP32
 * I2S module concurrency requirements to understand this.
 */
static NativeHandlerResult i2sdriver_consume_mailbox(Context *ctx)
{
    ESP_LOGI(TAG, "Consuming I2S mailbox message");

    Message *message = mailbox_first(&ctx->mailbox);
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        ESP_LOGW(TAG, "Received invalid message.");
        mailbox_remove_message(&ctx->mailbox, &ctx->heap);
        return NativeContinue;
    }

#ifdef ENABLE_TRACE
    TRACE("message: ");
    term_display(stdout, msg, ctx);
    TRACE("\n");
#endif

    term cmd_term = term_get_tuple_element(gen_message.req, 0);

    int local_process_id = term_to_local_process_id(gen_message.pid);

    term ret;
    NativeHandlerResult handler_result = NativeContinue;

    enum i2s_cmd cmd = interop_atom_term_select_int(cmd_table, cmd_term, ctx->global);
    switch (cmd) {
        case I2SCloseCmd:
            handler_result = i2s_driver_unref(ctx);
            ret = OK_ATOM;
            break;

        case I2SDumpCmd:
            handler_result = i2s_driver_dump(ctx);
            ret = OK_ATOM;
            break;

        default:
            ESP_LOGE(TAG, "i2s: error: unrecognized command: %x", cmd);
            ret = ERROR_ATOM;
    }

    term ret_msg;
    if (UNLIKELY(memory_ensure_free_with_roots(ctx, TUPLE_SIZE(2), 1, &ret, MEMORY_CAN_SHRINK) != MEMORY_GC_OK)) {
        ret_msg = OUT_OF_MEMORY_ATOM;
    } else {
        ret_msg = create_pair(ctx, gen_message.ref, ret);
    }

    globalcontext_send_message(ctx->global, local_process_id, ret_msg);

    mailbox_remove_message(&ctx->mailbox, &ctx->heap);

    return handler_result;
}

//
// entrypoints
//

REGISTER_PORT_DRIVER(i2s, i2s_driver_init, i2s_driver_destroy, i2s_driver_create_port)

#endif
