#
# This file is part of AtomVM.
#
# Copyright 2026 Joshua Gerrish <jgerrish@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
#

defmodule I2S do
  @compile {:no_warn_undefined, [AVMPort]}
  @moduledoc """
  Functions for interacting with i2s.
  """

  @typedoc """
  Whether to enable the RX channel
  """
  @type rx_enabled() :: boolean

  @typedoc """
  Whether to enable the TX channel
  """
  @type tx_enabled() :: boolean

  @typedoc """
  The mode of the I2S port

  The ESP-IDF project uses the terms duplex to refer to full-duplex
  mode and simplex to refer to half-duplex mode.  We will use
  full-duplex and half-duplex here, which matches the ESP32 hardware
  specifications.
  """
  @type duplex_mode() :: :full_duplex | :half_duplex

  @typedoc """
  Valid GPIO pin number for bit clock io number 1

  The actual number of pins that are broken out vary by board and module.
  """
  @type bclk_io1() :: 0..48

  @typedoc """
  Valid GPIO pin number for word select io number 1

  The actual number of pins that are broken out vary by board and module.
  """
  @type ws_io1() :: 0..48

  @typedoc """
  Valid GPIO pin number for data out io number 1

  The actual number of pins that are broken out vary by board and module.
  """
  @type dout_io1() :: 0..48

  @typedoc """
  Valid GPIO pin number for data in io number 1

  The actual number of pins that are broken out vary by board and module.
  """
  @type din_io1() :: 0..48

  @typedoc """
  Valid GPIO pin number for bit clock io number 2

  The actual number of pins that are broken out vary by board and module.
  """
  @type bclk_io2() :: 0..48

  @typedoc """
  Valid GPIO pin number for word select io number 2

  The actual number of pins that are broken out vary by board and module.
  """
  @type ws_io2() :: 0..48

  @typedoc """
  Valid GPIO pin number for data out io number 2

  The actual number of pins that are broken out vary by board and module.
  """
  @type dout_io2() :: 0..48

  @typedoc """
  Valid GPIO pin number for data in io number 2

  The actual number of pins that are broken out vary by board and module.
  """
  @type din_io2() :: 0..48

  @typedoc """
  Buffer size to allocate
  """
  @type buff_size() :: non_neg_integer()

  @typedoc """
  Valid parameters.

  Used to set the I2S parameters
  """
  @type param() ::
          {:rx_enabled, rx_enabled()}
          | {:tx_enabled, tx_enabled()}
          | {:duplex_mode, duplex_mode()}
	  | {:bclk_io1, bclk_io1()}
          | {:ws_io1, ws_io1()}
	  | {:dout_io1, dout_io1()}
	  | {:din_io1, din_io1()}
	  | {:bclk_io2, bclk_io2()}
	  | {:ws_io2, ws_io2()}
	  | {:dout_io2, dout_io2()}
	  | {:din_io2, din_io2()}
	  | {:buff_size, buff_size()}

  @type params() :: [param()]

  @doc """
  Open a connection to the I2S driver.

  Start the driver with a list of initialization parameters,
  see type param(). Returns the process id of the driver.

  ## Example:

  `I2S.open([{:rx_enabled, true}, {:tx_enabled: true}, {:duplex_mode, :full_duplex},
  {:bclk_io1, 4}, {:ws_io1, 5}, {:dout_io1, 18}, {:din_io1, 19},
  {:bclk_io2, 22}, {:ws_io2, 23}, {:dout_io2, 25}, {:din_io2, 26},
  {:buff_size, 2048}])`
  """
  @spec open(params()) :: pid()
  def open(configuration) do
    AVMPort.open({:spawn, "i2s"}, configuration)
  end

  @doc """
  Close the connection to the I2S driver

  ## Parameters
    - driver:   pid returned by I2S.open()

  This function will close the connection to the I2S driver and
  free any resources in use by it.
  """
  @spec close(pid()) :: :ok | {:error, term()}
  def close(driver) do
    AVMPort.call(driver, {:close})
  end

  @doc """
  Dump the I2S driver state

  ## Parameters
    - driver:   pid returned by I2S.open()

  This function will dump the state of the I2S driver to the console or
  log.
  """
  @spec dump(pid()) :: :ok | {:error, term()}
  def dump(driver) do
    AVMPort.call(driver, {:dump})
  end
end
