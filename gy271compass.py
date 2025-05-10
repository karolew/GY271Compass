import math
import time

from machine import I2C


class QMC5883L:
    # QMC5883L (GY-271) register addresses.
    # https://qstcorp.com/upload/pdf/202202/13-52-04%20QMC5883L%20Datasheet%20Rev.%20A(1).pdf
    REG_XOUT_LSB = 0x00     # XOUT[7:0]
    REG_XOUT_MSB = 0x01     # XOUT[15:8]
    REG_YOUT_LSB = 0x02     # YOUT[7:0]
    REG_YOUT_MSB = 0x03     # YOUT[15:8]
    REG_ZOUT_LSB = 0x04     # ZOUT[7:0]
    REG_ZOUT_MSB = 0x05     # ZOUT[15:8]
    REG_STATUS = 0x06       # bit 0 is DRDY: “0”: no new data, “1”: new data is ready
                            # bit 1 is OVL: “0”: normal, “1”: data overflow x, y or z not in range -32768 to 32767
    REG_TOUT_LSB = 0x07
    REG_TOUT_MSB = 0x08
    REG_CONTROL1 = 0x09
    REG_CONTROL2 = 0x0A
    REG_SET_RESET = 0x0B

    _16_bit_value_max = 65536
    _16_bit_value_half = 32767

    def __init__(self, i2c: I2C,
                 address: int = None,
                 calibration_offsets: tuple = None,
                 calibration_transform_matrix: list = None) -> None:
        """
        i2c:
            I2C interface.
        address:
            Address of the compass SoC. default 0x0D.
        calibration_offsets: tuple
            hard iron calibration
        transform_matrix
            soft iron calibration
        """
        self.i2c = i2c
        self.address = address if address else 0x0D     # Typical QMC5883L address.

        # Calibration parameters - hard iron (offsets)
        # Example (-2364, -496, 68)
        self.offset_x, self.offset_y, self.offset_z = calibration_offsets

        # Calibration parameters - soft iron (transformation matrix)
        # Example [[1.118951, 0.0, 0.0], [0.0, 1.07733, 0.0], [0.0, 0.0, 0.8488354]])
        self.transform_matrix = calibration_transform_matrix

        # Soft reset
        self.i2c.writeto_mem(self.address, self.REG_CONTROL2, bytes([0b10000000]))
        time.sleep(0.1)

        # Set to continuous measurement mode, 2G range, 50Hz output rate, over sampling ratio 512.
        self.i2c.writeto_mem(self.address, self.REG_CONTROL1, bytes([0b00010101]))
        time.sleep(0.1)

        # Set/Reset period
        self.i2c.writeto_mem(self.address, self.REG_SET_RESET, bytes([0b00000001]))
        time.sleep(0.1)

    def read_raw_data(self) -> tuple:
        # Read 6 bytes of data from register(0x00)
        try:
            data = self.i2c.readfrom_mem(self.address, self.REG_XOUT_LSB, 6)
            # Convert the data to signed 16-bit values
            x = data[0] | (data[1] << 8)
            if x > self._16_bit_value_half:
                x -= self._16_bit_value_max

            y = data[2] | (data[3] << 8)
            if y > self._16_bit_value_half:
                y -= self._16_bit_value_max

            z = data[4] | (data[5] << 8)
            if z > self._16_bit_value_half:
                z -= self._16_bit_value_max

            return x, y, z
        except Exception as e:
            print(f"Error reading compass data: {e}")
            return None, None, None

    def read_calibrated_data(self) -> tuple:
        """Read and apply calibration to data."""
        # Get raw data
        x, y, z = self.read_raw_data()

        if x and y and z:
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
        return None, None, None

    def get_heading(self) -> float | None:
        x, y, z = self.read_calibrated_data()

        if x and y:
            # Calculate heading in degrees
            heading = math.atan2(y, x)

            # Correct for when signs are reversed.
            if heading < 0:
                heading += 2 * math.pi

            # Check for wrap due to addition of declination.
            if heading > 2 * math.pi:
                heading -= 2 * math.pi

            # Convert radians to degrees.
            heading_degrees = heading * 180 / math.pi

            return heading_degrees
        return None

    @staticmethod
    def get_direction(angle_degrees) -> str:
        directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"]
        return directions[round(angle_degrees / 45) % 8]
