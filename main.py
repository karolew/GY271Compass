# Example compatible with ESP32 WROOM.

import time
from machine import Pin, I2C
from gy271compass import QMC5883L

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
compass = QMC5883L(i2c, corrections={"x_offset": 162, "x_scale": 1.04, "y_offset": -211, "y_scale": 0.97})

previous_heading = None
while True:
    time.sleep(0.05)
    heading = compass.get_heading()
    if heading != previous_heading:
        previous_heading = heading
        print(heading)
