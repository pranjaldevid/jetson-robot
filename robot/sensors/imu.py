"""MPU6050 6-axis IMU — I2C bus 7 on Jetson Orin Nano (header pins 3/5)."""
import time
import smbus2


class MPU6050:
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT = 0x3B
    GYRO_XOUT = 0x43
    WHO_AM_I = 0x75
    ACCEL_SCALE = 16384.0   # +/-2 g
    GYRO_SCALE = 131.0      # +/-250 dps

    def __init__(self, bus=7, addr=0x68):
        self.addr = addr
        self.bus = smbus2.SMBus(bus)
        self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0)  # wake
        time.sleep(0.1)

    def _word(self, reg):
        hi = self.bus.read_byte_data(self.addr, reg)
        lo = self.bus.read_byte_data(self.addr, reg + 1)
        val = (hi << 8) | lo
        return val - 65536 if val > 32767 else val

    def who_am_i(self):
        return self.bus.read_byte_data(self.addr, self.WHO_AM_I)

    def read_accel(self):
        return tuple(self._word(self.ACCEL_XOUT + 2 * i) / self.ACCEL_SCALE for i in range(3))

    def read_gyro(self):
        return tuple(self._word(self.GYRO_XOUT + 2 * i) / self.GYRO_SCALE for i in range(3))


if __name__ == "__main__":
    imu = MPU6050()
    print(f"WHO_AM_I = 0x{imu.who_am_i():02x} (expect 0x68)")
    while True:
        ax, ay, az = imu.read_accel()
        gx, gy, gz = imu.read_gyro()
        print(f"accel g: {ax:+.2f} {ay:+.2f} {az:+.2f}   gyro dps: {gx:+.1f} {gy:+.1f} {gz:+.1f}")
        time.sleep(0.2)
