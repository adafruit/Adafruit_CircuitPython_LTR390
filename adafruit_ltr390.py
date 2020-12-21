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
import adafruit_bus_device.i2c_device as i2c_device
from micropython import const
from adafruit_register.i2c_struct import ROUnaryStruct
from adafruit_register.i2c_bit import RWBit, ROBit

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_LTR390.git"

_DEFAULT_I2C_ADDR = const(0x53)
# _I2CADDR_DEFAULT = const(0x53) # I2C address
_CTRL = const(0x00)  # Main control register
# _MEAS_RATE = const(0x04)       # Resolution and data rate
# _GAIN = const(0x05)            # ALS and UVS gain range
_PART_ID = const(0x06)  # Part id/revision register
_STATUS = const(0x07)  # Main status register
# _ALSDATA = const(0x0D)         # ALS data lowest byte
# _UVSDATA = const(0x10)         # UVS data lowest byte
# _INT_CFG = const(0x19)         # Interrupt configuration
# _INT_PST = const(0x1A)         # Interrupt persistance config
# _THRESH_UP = const(0x21)       # Upper threshold, low byte
# _THRESH_LOW = const(0x24)      # Lower threshold, low byte
# begin
# reset
# newDataAvailable
# readALS
# readUVS
# enable
# enabled
# setMode
# getMode
# setGain
# getGain
# setResolution
# getResolution
# setThresholds
# configInterrupt
class LTR390:
    """Class to use the LTR390 Ambient Light and UV sensor"""

    _id_reg = ROUnaryStruct(_PART_ID, "<B")
    _reset = RWBit(_CTRL, 4)
    _enable = RWBit(_CTRL, 1)
    data_ready = ROBit(_STATUS, 3)
    """Ask the sensor if new data is available"""

    def __init__(self, i2c, address=_DEFAULT_I2C_ADDR):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        if self._id_reg != 0xB2:

            raise RuntimeError("Unable to find LTR390; check your wiring")

        self.reset()

        # StatusReg = new Adafruit_I2CRegister(i2c_dev, LTR390_MAIN_STATUS);
        # DataReadyBit = new Adafruit_I2CRegisterBits(StatusReg, 1, 3);

        # return true;

    def reset(self):
        """Reset the sensor to it's initial unconfigured state"""
        # The write to the reset bit will fail because it seems to not ACK
        # before it resets, however the reset does reset
        try:
            self._reset = True
        except OSError:
            print("got expected OS error on reset")

        sleep(0.010)
        # check that reset is complete w/ the bit unset
        if self._reset:
            raise RuntimeError("Unable to reset sensor")

        self._enable = True
        if not self._enable:
            raise RuntimeError("Unable to enable sensor")

    def initialize(self):
        """Configure the sensor with sensible defaults so it can be used"""

    @property
    def uv_index(self):
        """The calculated UV Index"""
        return 1000
        # self._take_reading()
        # return ((self._uvacalc * self._uvaresp) + (self._uvbcalc * self._uvbresp)) / 2
