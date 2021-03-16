# LTR360 test

import time
import busio
import board
from digitalio import DigitalInOut, Direction, Pull

from adafruit_ltr390 import LTR390, MeasurementDelay, Resolution, Gain

i2c = busio.I2C(board.SCL, board.SDA)
ltr = LTR390(i2c)

outboard_led = DigitalInOut(board.D13)
outboard_led.direction =Direction.OUTPUT

#ltr.resolution = Resolution.RESOLUTION_20BIT
print("Measurement resolution is", Resolution.string[ltr.resolution])

#ltr.gain = Gain.GAIN_18X
print("Measurement gain is", Gain.string[ltr.gain])

#ltr.measurement_delay = MeasurementDelay.DELAY_100MS
print("Measurement delay is", MeasurementDelay.string[ltr.measurement_delay])
print("")
while True:
    print()
    print(f"resolution: {Resolution.string[ltr.resolution]}  gain: {Gain.string[ltr.gain]}  delay(rate): {MeasurementDelay.string[ltr.measurement_delay]}  resolution.integration: {Resolution.integration[ltr.resolution]}  window_factor: {ltr.window_factor}")
    outboard_led.value = True
    t0 = time.monotonic()
    uvs = ltr.uvs
    t1 = round(time.monotonic() - t0, 3)
    uvi = ltr.uvi
    print(f"UVs: {uvs}   UVi: {uvi}  read_time: {t1} sec  ")

    t0 = time.monotonic()
    light = round(ltr.light, 3)
    t1 = round(time.monotonic() - t0, 3)
    lux = ltr.lux
    print(f"light: {light}   Lux: {lux}  read_time: {t1} sec  ")

    print((uvi * 100, lux))

    outboard_led.value = False
    time.sleep(0.5)
