from machine import I2C, Pin
import time
import math
import struct


class QMC5883L:
    def __init__(self, i2c, addr=0x0D):
        self.i2c = i2c
        self.addr = addr

        # Initialize the sensor
        self.i2c.writeto_mem(self.addr, 0x0B, b'\x01')  # Set to Standby mode
        self.i2c.writeto_mem(self.addr, 0x09, b'\x1D')  # Set register 0x09 (OSR = 512, RNG = 8G, ODR = 200Hz, MODE = Continuous)
        time.sleep(0.1)

        # Calibration parameters - hard iron (offsets)
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0

        # Calibration parameters - soft iron (transformation matrix)
        # Default to identity matrix
        self.transform_matrix = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]

    def read_raw_data(self):
        """Read raw magnetometer data."""
        data = self.i2c.readfrom_mem(self.addr, 0x00, 6)
        x = data[0] | (data[1] << 8)
        if x > 32767:
            x -= 65536
        y = data[2] | (data[3] << 8)
        if y > 32767:
            y -= 65536
        z = data[4] | (data[5] << 8)
        if z > 32767:
            z -= 65536
        return x, y, z

    def read_calibrated_data(self):
        """Read and apply calibration to data."""
        # Get raw data
        x, y, z = self.read_raw_data()

        # Apply hard iron correction (offset)
        x_offset = x - self.offset_x
        y_offset = y - self.offset_y
        z_offset = z - self.offset_z

        # Apply soft iron correction (transformation matrix)
        x_cal = (self.transform_matrix[0][0] * x_offset +
                 self.transform_matrix[0][1] * y_offset +
                 self.transform_matrix[0][2] * z_offset)

        y_cal = (self.transform_matrix[1][0] * x_offset +
                 self.transform_matrix[1][1] * y_offset +
                 self.transform_matrix[1][2] * z_offset)

        z_cal = (self.transform_matrix[2][0] * x_offset +
                 self.transform_matrix[2][1] * y_offset +
                 self.transform_matrix[2][2] * z_offset)

        return x_cal, y_cal, z_cal

    def collect_calibration_data(self, samples=20000, delay_ms=10):
        """Collect data points for calibration by rotating sensor in all directions."""
        print("\nBegin collecting calibration data...")
        print("Rotate the sensor slowly in all directions to cover all possible orientations.")
        print(f"Collecting {samples} samples...")

        data_points = []

        for i in range(samples):
            x, y, z = self.read_raw_data()
            data_points.append((x, y, z))

            if i % 100 == 0:
                print(f"Progress: {i / samples * 100:.1f}%")

            time.sleep_ms(delay_ms)

        print("Data collection complete!")
        return data_points

    def calculate_calibration_parameters(self, data_points):
        """Calculate calibration parameters from collected data."""
        # Hard iron correction: find the center of the ellipsoid
        min_x = min_y = min_z = 32767
        max_x = max_y = max_z = -32768

        for x, y, z in data_points:
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            min_z = min(min_z, z)
            max_z = max(max_z, z)

        # Calculate offsets (center of ellipsoid)
        self.offset_x = (min_x + max_x) // 2
        self.offset_y = (min_y + max_y) // 2
        self.offset_z = (min_z + max_z) // 2

        # Calculate average radii
        radius_x = (max_x - min_x) / 2
        radius_y = (max_y - min_y) / 2
        radius_z = (max_z - min_z) / 2

        # Calculate average radius
        avg_radius = (radius_x + radius_y + radius_z) / 3

        # Calculate scale factors to normalize each axis
        scale_x = avg_radius / radius_x if radius_x > 0 else 1
        scale_y = avg_radius / radius_y if radius_y > 0 else 1
        scale_z = avg_radius / radius_z if radius_z > 0 else 1

        # Update transformation matrix for scaling (simple approach)
        # For a more accurate approach, eigenvalue decomposition would be needed
        self.transform_matrix = [
            [scale_x, 0, 0],
            [0, scale_y, 0],
            [0, 0, scale_z]
        ]

        return {
            'offset_x': self.offset_x,
            'offset_y': self.offset_y,
            'offset_z': self.offset_z,
            'scale_x': scale_x,
            'scale_y': scale_y,
            'scale_z': scale_z,
            'transform_matrix': self.transform_matrix
        }

    def advanced_calibrate(self, samples=20000, delay_ms=10):
        """Perform advanced calibration of the magnetometer."""
        print("Starting advanced calibration...")

        # Collect calibration data
        data_points = self.collect_calibration_data(samples, delay_ms)

        # Calculate calibration parameters
        cal_params = self.calculate_calibration_parameters(data_points)

        print("\nCalibration Complete!")
        print(f"Hard Iron Offsets: X={self.offset_x}, Y={self.offset_y}, Z={self.offset_z}")
        print(
            f"Soft Iron Scale Factors: X={cal_params['scale_x']:.4f}, Y={cal_params['scale_y']:.4f}, Z={cal_params['scale_z']:.4f}")

        return cal_params

    def save_calibration(self, filename='mag_calibration.dat'):
        """Save calibration parameters to a file."""
        try:
            with open(filename, 'wb') as f:
                # Format: offset_x, offset_y, offset_z, followed by 9 transform matrix values
                data = struct.pack("<iii9f",
                                   self.offset_x, self.offset_y, self.offset_z,
                                   self.transform_matrix[0][0], self.transform_matrix[0][1],
                                   self.transform_matrix[0][2],
                                   self.transform_matrix[1][0], self.transform_matrix[1][1],
                                   self.transform_matrix[1][2],
                                   self.transform_matrix[2][0], self.transform_matrix[2][1], self.transform_matrix[2][2]
                                   )
                f.write(data)
            print(f"Calibration saved to {filename}")
            return True
        except Exception as e:
            print(f"Error saving calibration: {e}")
            return False

    def load_calibration(self, filename='mag_calibration.dat'):
        """Load calibration parameters from a file."""
        try:
            with open(filename, 'rb') as f:
                data = f.read()

            # Unpack the data
            values = struct.unpack("<iii9f", data)

            # Set offsets
            self.offset_x = values[0]
            self.offset_y = values[1]
            self.offset_z = values[2]

            # Set transform matrix
            self.transform_matrix = [
                [values[3], values[4], values[5]],
                [values[6], values[7], values[8]],
                [values[9], values[10], values[11]]
            ]

            print(f"Calibration loaded from {filename}")
            print(f"Hard Iron Offsets: X={self.offset_x}, Y={self.offset_y}, Z={self.offset_z}")
            print(f"Soft Iron Matrix: {self.transform_matrix}")
            return True
        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False

    def calculate_heading(self):
        """Calculate heading in degrees (0-360)."""
        x, y, z = self.read_calibrated_data()

        # Calculate heading
        heading = math.atan2(y, x)

        # Convert to degrees
        heading_deg = heading * 180.0 / math.pi

        # Normalize to 0-360
        if heading_deg < 0:
            heading_deg += 360.0

        return heading_deg

    def get_tilt_compensated_heading(self):
        """Calculate tilt-compensated heading using accelerometer data."""
        # Note: This is a placeholder. For actual tilt compensation,
        # you would need accelerometer data to determine pitch and roll.
        return self.calculate_heading()


def visualize_calibration(mag, num_samples=200):
    """Visualize calibration by printing data points before and after calibration."""
    print("\nVisualizing calibration effectiveness...")
    print("Collecting raw and calibrated samples. Please rotate sensor slowly...")

    for i in range(num_samples):
        raw_x, raw_y, raw_z = mag.read_raw_data()
        cal_x, cal_y, cal_z = mag.read_calibrated_data()

        # Calculate magnitudes
        raw_mag = math.sqrt(raw_x ** 2 + raw_y ** 2 + raw_z ** 2)
        cal_mag = math.sqrt(cal_x ** 2 + cal_y ** 2 + cal_z ** 2)

        if i % 20 == 0:
            print(f"Raw: ({raw_x}, {raw_y}, {raw_z}) Mag: {raw_mag:.1f}")
            print(f"Cal: ({cal_x:.1f}, {cal_y:.1f}, {cal_z:.1f}) Mag: {cal_mag:.1f}")
            print("---")

        time.sleep_ms(50)

    print("Visualization complete!")


def main():
    # Setup I2C
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)

    # Initialize sensor
    qmc = QMC5883L(i2c)

    print("QMC5883L Advanced Calibration")
    print("-----------------------------")
    print("1: Perform new calibration")
    print("2: Load existing calibration")
    print("3: Test readings")
    print("4: Visualize calibration")
    print("5: Exit")

    while True:
        choice = input("Enter choice (1-5): ")

        if choice == '1':
            # Perform calibration
            cal_params = qmc.advanced_calibrate(samples=1000, delay_ms=50)

            # Save calibration
            qmc.save_calibration()

        elif choice == '2':
            # Load calibration
            qmc.load_calibration()

        elif choice == '3':
            # Test readings
            print("\nTesting calibrated readings... Press Ctrl+C to stop")
            try:
                while True:
                    x, y, z = qmc.read_calibrated_data()
                    heading = qmc.calculate_heading()
                    print(f"X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Heading={heading:.1f}°")
                    time.sleep(0.5)
            except KeyboardInterrupt:
                print("\nStopped reading")

        elif choice == '4':
            # Visualize calibration
            visualize_calibration(qmc)

        elif choice == '5':
            print("Exiting...")
            break

        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    main()
