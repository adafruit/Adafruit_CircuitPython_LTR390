# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ltr390`
================================================================================

Adafruit CircuitPython library for the LTR390


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* Adafruit LTR390 Breakout <https://www.adafruit.com/product/38XX>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

from time import sleep
from struct import unpack_from
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device
from adafruit_register.i2c_struct import ROUnaryStruct, Struct
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_bit import RWBit, ROBit

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_LTR390.git"

_DEFAULT_I2C_ADDR = const(0x53)
_CTRL = const(0x00)  # Main control register
_MEAS_RATE = const(0x04)  # Resolution and data rate
_GAIN = const(0x05)  # ALS and UVS gain range
_PART_ID = const(0x06)  # Part id/revision register
_STATUS = const(0x07)  # Main status register
_ALSDATA = const(0x0D)         # ALS data lowest byte
_UVSDATA = const(0x10)  # UVS data lowest byte
# _INT_CFG = const(0x19)         # Interrupt configuration
# _INT_PST = const(0x1A)         # Interrupt persistance config
# _THRESH_UP = const(0x21)       # Upper threshold, low byte
# _THRESH_LOW = const(0x24)      # Lower threshold, low byte

# readALS

# setMode
# getMode
# setGain
# getGain
# setResolution
# getResolution
# setThresholds
# configInterrupt


class UnalignedStruct(Struct):
    """Class for reading multi-byte data registers with a data length less than the full bitwidth
    of the registers. Most registers of this sort are left aligned to preserve the sign bit"""

    def __init__(self, register_address, struct_format, bitwidth, length):
        super().__init__(register_address, struct_format)
        self._width = bitwidth
        self._num_bytes = length

    def __get__(self, obj, objtype=None):
        # read bytes into buffer at correct alignment
        with obj.i2c_device as i2c:
            i2c.write_then_readinto(
                self.buffer,
                self.buffer,
                out_start=0,
                out_end=1,
                in_start=2,  # right aligned
                # in_end=4 # right aligned
            )
        raw_value = unpack_from(self.format, self.buffer, offset=1)[0]
        return raw_value >> 8


class LTR390:
    """Class to use the LTR390 Ambient Light and UV sensor"""

    _id_reg = ROUnaryStruct(_PART_ID, "<B")
    _reset = RWBit(_CTRL, 4)
    _enable = RWBit(_CTRL, 1)
    _mode = RWBit(_CTRL, 3)
    _gain = RWBits(3, _GAIN, 0)
    _resolution = RWBits(3, _MEAS_RATE, 4)
    data_ready = ROBit(_STATUS, 3)

    # _uvs_data = ROUnaryStruct(_UVSDATA, "")
    # def __init__(self, register_address, struct_format, bitwidth, length):

    _uvs_data = UnalignedStruct(_UVSDATA, "<I", 24, 3)
    _als_data = UnalignedStruct(_ALSDATA, "<I", 24, 3)
    """Ask the sensor if new data is available"""

    def __init__(self, i2c, address=_DEFAULT_I2C_ADDR):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        if self._id_reg != 0xB2:

            raise RuntimeError("Unable to find LTR390; check your wiring")

        self.reset()
        self._mode = 1  # UV index/UVS, ALS=0
        self._mode_cache = self._mode
        self._gain = 1  # GAIN_3
        self._resolution = 4  # RESOLUTION_16BIT

        # ltr.setThresholds(100, 1000);
        # ltr.configInterrupt(true, LTR390_MODE_UVS);

    def reset(self):
        """Reset the sensor to it's initial unconfigured state"""
        # The write to the reset bit will fail because it seems to not ACK
        # before it resets, however the reset does reset
        try:
            self._reset = True
        except OSError:
            print("got expected OS error on reset")

        sleep(0.1)
        # check that reset is complete w/ the bit unset
        if self._reset:
            raise RuntimeError("Unable to reset sensor")

        self._enable = True
        if not self._enable:
            raise RuntimeError("Unable to enable sensor")

    def initialize(self):
        """Configure the sensor with sensible defaults so it can be used"""

    # something is wrong here; I had to add a sleep to the loop to get both to update correctly
    @property
    def uv_index(self):
        """The calculated UV Index"""
        if self._mode_cache != 1:
            self._mode = 1
            self._mode_cache = 1
            sleep(0.030)
        while not self.data_ready:
            sleep(0.010)
        return self._uvs_data

    @property
    def light(self):
        """The currently measured ambient light level"""
        if self._mode_cache != 0:
            self._mode = 0
            self._mode_cache = 0
            sleep(0.030)
        while not self.data_ready:
            sleep(0.010)
        return self._als_data