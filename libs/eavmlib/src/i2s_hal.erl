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

%% @doc I2S Hardware Abstraction Layer behavior
%%
%% This module defines the behavior that platform-specific I2S modules
%% must implement. It provides a common interface for I2S (Inter-Integrated
%% Circuit) operations across all supported platforms.
%%
%% Currently, only ESP32 provides an I2S implementation.
%%
%% <h3>Lifecycle</h3>
%%
%% An I2S bus is opened with `open/1' and closed with `close/1'. The
%% returned handle is passed to all subsequent operations.
%%
%% <h3>Configuration parameters</h3>
%%
%% The `open/1' function accepts a proplist of configuration parameters.
%% Common parameters include:
%%
%% <ul>
%% <li>`{rx_enabled, boolean()}' - RX Enabled</li>
%% <li>`{tx_enabled, boolean()}' - TX Enabled</li>
%% <li>`{duplex_mode, full_duplex | half_duplex}' - Duplex mode</li>
%% <li>`{bclk_io1, pin()}' - BCLK (Bit Clock) IO1 pin number</li>
%% <li>`{ws_io1, pin()}' - WS (Word Select) IO1 pin number</li>
%% <li>`{dout_io1, pin()}' - DOUT (Data Out) IO1 pin number</li>
%% <li>`{din_io1, pin()}' - DIN (Data In) IO1 pin number</li>
%% <li>`{bclk_io2, pin()}' - BCLK (Bit Clock) IO2 pin number</li>
%% <li>`{ws_io2, pin()}' - WS (Word Select) IO2 pin number</li>
%% <li>`{dout_io2, pin()}' - DOUT (Data Out) IO2 pin number</li>
%% <li>`{din_io2, pin()}' - DIN (Data In) IO2 pin number</li>
%% <li>`{buff_size, non_neg_integer())}' - Buffer size</li>
%% <li>`{peripheral, string() | binary()}' - I2S peripheral name (e.g.
%% `"i2s0"', `"i2s1"')</li>
%% </ul>
%%
%% <h3>Example</h3>
%%
%% ```
%% I2S = i2s:open([{rx_enabled, true}, {tx_enabled: true}, {duplex_mode, full_duplex}, {bclk_io1, 4}, {ws_io1, 5}, {dout_io1, 18}, {din_io1, 19}, {bclk_io2, 22}, {ws_io2, 23}, {dout_io2, 25}, {din_io2, 26}, {buff_size, 2048}]),
%% i2s:close(I2S).
%% '''
%% @end
%%-----------------------------------------------------------------------------
-module(i2s_hal).

-type i2s() :: port() | pid() | term().
%% Handle returned by `open/1'.
%% On ESP32, this is either a port (port driver mode) or a resource
%% tuple (NIF mode).

-type params() :: [term()].
%% Initialization parameters for the I2S bus.
%% See the module documentation for common parameters.

-export_type([i2s/0, params/0]).

% Open an I2S bus with the given configuration parameters.
%
% Returns a handle that must be passed to all subsequent I2S
% operations.
-callback open(Params :: params()) -> i2s().

% Close an I2S bus and release its resources.
-callback close(I2S :: i2s()) -> ok | {error, Reason :: term()}.

% Dump an I2S bus state to console or log
-callback dump(I2S :: i2s()) -> ok | {error, Reason :: term()}.
