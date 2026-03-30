%
% This file is part of AtomVM.
%
% Copyright 2026 Joshua Gerrish <jgerrish@gmail.com>
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%    http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%
% SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
%

%%-----------------------------------------------------------------------------
%% @doc AtomVM I2s interface
%%
%% This module provides and interface into the AtomVM I2S driver.
%%
%% Use this module to communicate with devices connected to your ESP32
%% device via the 2-wire I2S interface.
%%
%% Using this interface, you can read or write data to an I2S device
%% at a given I2S address.  In addition, you may read from or write to
%% specific registers on the I2S device.
%% @end
%%-----------------------------------------------------------------------------
-module(i2s).

-behaviour(i2s_hal).

-export([
    open/1,
    close/1,
    dump/1
]).

-define(I2S_RSRC, {'$i2s', _Resource, _Ref}).

-type pin() :: non_neg_integer().
%% -type freq_hz() :: non_neg_integer().
-type peripheral() :: string() | binary().
-type rx_enabled() :: bool. %% non_neg_integer().
-type tx_enabled() :: bool. %% non_neg_integer().
%% The ESP-IDF project uses the terms duplex to refer to full-duplex
%% mode and simplex to refer to half-duplex mode.  We will use
%% full-duplex (full_duplex) and half-duplex here (half_duplex), which
%% matches the ESP32 hardware specifications.
-type duplex_mode() :: full_duplex | half_duplex.
-type bclk_io1() :: pin().
-type ws_io1() :: pin().
-type dout_io1() :: pin().
-type din_io1() :: pin().
-type bclk_io2() :: pin().
-type ws_io2() :: pin().
-type dout_io2() :: pin().
-type din_io2() :: pin().
-type buff_size() :: non_neg_integer().
-type param() ::
      {peripheral, peripheral()}
      | {rx_enabled, rx_enabled()}
      | {tx_enabled, tx_enabled()}
      | {duplex_mode, duplex_mode()}
      | {bclk_io1, bclk_io1()}
      | {ws_io1, ws_io1()}
      | {dout_io1, dout_io1()}
      | {din_io1, din_io1()}
      | {bclk_io2, bclk_io2()}
      | {ws_io2, ws_io2()}
      | {dout_io2, dout_io2()}
      | {din_io2, din_io2()}
      | {buff_size, buff_size()}.

-type params() :: [param()].
-type i2s() :: port() | {'$i2s', term(), reference()}.
%% -type address() :: non_neg_integer().
%% -type register() :: non_neg_integer().

-export_type([
    i2s/0
]).

%%-----------------------------------------------------------------------------
%% @param   Params Initialization parameters
%% @returns process id of the driver.
%% @doc     Open a connection to the I2S driver
%%
%%          This function will open a connection to the I2S driver.
%% @end
%%-----------------------------------------------------------------------------
-spec open(Params :: params()) -> i2s().
open(Params) ->
    open_port({spawn, "i2s"}, Params).

%%-----------------------------------------------------------------------------
%% @param   I2S I2S instance created via `open/1'
%% @returns `ok' atom
%% @doc     Closes the connection to the I2S driver
%%
%%          This function will close the connection to the I2S driver and
%%          free any resources in use by it.
%% @end
%%-----------------------------------------------------------------------------
-spec close(I2S :: i2s()) -> ok | {error, Reason :: term()}.
close(I2S) ->
    port:call(I2S, {close}).

%%-----------------------------------------------------------------------------
%% @param   I2S I2S instance created via `open/1'
%% @returns `ok' atom
%% @doc     Dumps the I2S driver state
%% @end
%%-----------------------------------------------------------------------------
-spec dump(I2S :: i2s()) -> ok | {error, Reason :: term()}.
dump(I2S) ->
    port:call(I2S, {dump}).
