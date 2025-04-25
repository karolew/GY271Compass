# QMC5883L (GY271) Compass

This is a micropython custom driver implementation for QMC5883L (GY271 board) compass module.
Correct support for x and y directions.

Configuration is hardcoded in the `__init__` function.
By default it is: 10Hz update, continuous mode control, RNG 2G, OSR 512. 

Callibration data can be provided as `corrections` dict. Calibration process is out of scope.

# Usage

1. Copy `gy271compass.py` to your project
2. Initialize I2C with specified pins. Example ESP32:</br>
`i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)`
3. Create compas instance. Example:</br>
`compass = QMC5883L(i2c, corrections={"x_offset": -930, "x_scale": 0.9, "y_offset": -1894, "y_scale": 1.13})`
4. Get heading in degres:</br>
`heading = compass.get_heading()`
