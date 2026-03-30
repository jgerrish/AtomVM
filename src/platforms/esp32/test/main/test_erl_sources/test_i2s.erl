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

-module(test_i2s).
-export([start/0]).

start() ->
    ok = test_i2s_driver(true, binary),
    0.

test_i2s_driver(true, BinaryOpt) ->
    I2SDriver = open_port({spawn, "i2s_driver"}, []),
    Params = [
	      {rx_enabled, true},
	      {tx_enabled, false},
	      {duplex_mode, half_duplex},
	      {bclk_io1, 4},
	      {ws_io1, 5},
	      {dout_io1, 18},
	      {din_io1, 19},
	      {bclk_io2, 22},
	      {ws_io2, 23},
	      {dout_io2, 25},
	      {din_io2, 25},
	      {buff_size, 2048},
	      BinaryOpt
    ],
    ok = call(I2SDriver, {init, Params}, 30000),
    ok = call(I2SDriver, {close})
