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
_ALSDATA = const(0x0D)  # ALS data lowest byte
_UVSDATA = const(0x10)  # UVS data lowest byte
_ALS = 0
_UV = 1
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


class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples):
        """Add CV values to the class"""
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value):
        """Validate that a given value is a member"""
        return value in cls.string


class Gain(CV):
    """Options for `gain`"""


Gain.add_values(
    (
        ("GAIN_1X", 0, "1X", None),
        ("GAIN_3X", 1, "3X", None),
        ("GAIN_6X", 2, "6X", None),
        ("GAIN_9X", 3, "9X", None),
        ("GAIN_18X", 4, "18X", None),
    )
)


class Resolution(CV):
    """Options for `resolution`"""


Resolution.add_values(
    (
        ("RESOLUTION_20BIT", 0, "20", None),
        ("RESOLUTION_19BIT", 1, "19", None),
        ("RESOLUTION_18BIT", 2, "18", None),
        ("RESOLUTION_17BIT", 3, "17", None),
        ("RESOLUTION_16BIT", 4, "16", None),
        ("RESOLUTION_13BIT", 5, "13", None),
    )
)


class LTR390:
    """Class to use the LTR390 Ambient Light and UV sensor"""

    _reset_bit = RWBit(_CTRL, 4)
    _enable_bit = RWBit(_CTRL, 1)
    _mode_bit = RWBit(_CTRL, 3)

    _gain_bits = RWBits(3, _GAIN, 0)
    _resolution_bits = RWBits(3, _MEAS_RATE, 4)

    _id_reg = ROUnaryStruct(_PART_ID, "<B")
    _uvs_data_reg = UnalignedStruct(_UVSDATA, "<I", 24, 3)
    _als_data_reg = UnalignedStruct(_ALSDATA, "<I", 24, 3)

    data_ready = ROBit(_STATUS, 3)
    """Ask the sensor if new data is available"""

    def __init__(self, i2c, address=_DEFAULT_I2C_ADDR):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        if self._id_reg != 0xB2:

            raise RuntimeError("Unable to find LTR390; check your wiring")

        self._mode_cache = None
        self.initialize()

    def initialize(self):
        """Reset the sensor to it's initial unconfigured state and configure it with sensible
        defaults so it can be used"""

        self._reset()
        self._enable_bit = True
        if not self._enable_bit:
            raise RuntimeError("Unable to enable sensor")
        self._mode = _UV
        self.gain = Gain.GAIN_3X  # pylint:disable=no-member
        self.resolution = Resolution.RESOLUTION_16BIT  # pylint:disable=no-member

        # ltr.setThresholds(100, 1000);
        # ltr.configInterrupt(true, LTR390_MODE_UVS);

    def _reset(self):
        try:
            self._reset_bit = True
        except OSError:
            # The write to the reset bit will fail because it seems to not ACK before it resets
            pass

        sleep(0.1)
        # check that reset is complete w/ the bit unset
        if self._reset_bit:
            raise RuntimeError("Unable to reset sensor")

    @property
    def _mode(self):
        return self._mode_bit

    @_mode.setter
    def _mode(self, value):
        if not value in [_ALS, _UV]:
            raise AttributeError("Mode must be _ALS or _UV")
        if self._mode_cache != value:
            self._mode_bit = value
            self._mode_cache = value
            sleep(0.030)

    # something is wrong here; I had to add a sleep to the loop to get both to update correctly
    @property
    def uv_index(self):
        """The calculated UV Index"""
        self._mode = _UV
        while not self.data_ready:
            sleep(0.010)
        return self._uvs_data_reg

    @property
    def light(self):
        """The currently measured ambient light level"""
        self._mode = _ALS
        while not self.data_ready:
            sleep(0.010)
        return self._als_data_reg

    @property
    def gain(self):
        """The amount of gain the raw measurements are multiplied by"""
        return self._gain_bits

    @gain.setter
    def gain(self, value):
        if not Gain.is_valid(value):
            raise AttributeError("gain must be a Gain")
        self._gain_bits = value

    @property
    def resolution(self):
        """Set the precision of the internal ADC used to read the light measurements"""
        return self._resolution_bits

    @resolution.setter
    def resolution(self, value):
        if not Resolution.is_valid(value):
            raise AttributeError("resolution must be a Resolution")
        self._resolution_bits = value
