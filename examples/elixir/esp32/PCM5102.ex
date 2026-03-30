#
#  This file is part of AtomVM.
#
#  Copyright 2026 Joshua Gerrish <jgerrish@gmail.com>
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
#  SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
#

defmodule PCM5102 do
  # suppress warnings when compiling the VM
  # not needed or recommended for user apps.
  @compile {:no_warn_undefined, [I2S]}

  def start do
    i2s = I2S.open([{:rx_enabled, false}, {:tx_enabled, true}, {:duplex_mode, :half_duplex},
		    {:bclk_io1, 4}, {:ws_io1, 5}, {:dout_io1, 18}, {:din_io1, 19},
		    # These aren't needed for half-duplex mode without
		    # RX, but we include them
		    {:bclk_io2, 22}, {:ws_io2, 23}, {:dout_io2, 25}, {:din_io2, 26},
		    {:buff_size, 2048}])
    # IO.puts("I2S: #{i2s}")

    # Test dumping the driver state
    i2s.dump()

    # Try sending a close message to shut down this port
    i2s.close()

    # Try total shutdown of system without a close message
    # :erlang.exit({})
  end
end
