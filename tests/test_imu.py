from robot.sensors.imu import MPU6050


def test_who_am_i():
    imu = MPU6050()
    assert imu.who_am_i() == 0x68


def test_gravity_present():
    imu = MPU6050()
    mag = sum(a * a for a in imu.read_accel()) ** 0.5
    assert 0.8 < mag < 1.2   # ~1 g at rest
