import time

from machine import I2C, Pin

from gy271compass import QMC5883L

if __name__ == "__main__":
    # Example corrections.
    # Hard Iron Offsets: X=-2364, Y=-496, Z=68
    # Soft Iron Matrix: [[1.118951, 0.0, 0.0], [0.0, 1.07733, 0.0], [0.0, 0.0, 0.8488354]]

    i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)
    qmc = QMC5883L(i2c,
                   None,
                   (-2364, -496, 68),
                   [[1.118951, 0.0, 0.0], [0.0, 1.07733, 0.0], [0.0, 0.0, 0.8488354]])

    heading_precious = 0
    while True:
        try:
            while True:
                x, y, z = qmc.read_calibrated_data()
                heading = qmc.get_heading()
                if heading and abs(heading_precious - heading) > 2:
                    heading_precious = heading
                    print(f"X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Heading={heading:.1f}°")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nStopped reading")
        except Exception as e:
            print(f"Error compass: {e}")
