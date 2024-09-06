"""BME280 node."""
# !/usr/bin/env python3
import bme280  # BME280 library
import time
import datetime
import numpy as np

from autopilot_pkg.lar_common import QGC_mode, Kayak_mode
from lar_interfaces.msg import BmeData, TimeSample, KayakMode, QgcMode, Imu
from lar_interfaces.srv import SendBmeData

import rclpy
from rclpy.node import Node
from rclpy import qos
import math
import smbus2  # Open the I2C port (on HAT40 connector)


class I2CDevices(Node):
    """BME280 measurements node."""

    def __init__(self):
        """BME280 node initialization, service, and open I2C bus."""
        super().__init__('i2c_node')
        self.bme_port = '/dev/i2c-lar'
        self.measure = False
        self.wdt_bme = 0
        self.sm_flag_bme = 0
        self.epoch_bme = int(time.time() * 1e6)
        self.kayak_mode_publisher = self.create_publisher(KayakMode, 'kayak_mode', 10)
        self.qgc_mode_publisher = self.create_publisher(QgcMode, 'qgc_mode', 10)
        self.bme_publisher = self.create_publisher(BmeData, 'BME_measurements', qos_profile=qos.qos_profile_sensor_data)
        self.bmi_publisher = self.create_publisher(Imu, 'BMI_measurements', qos_profile=qos.qos_profile_sensor_data)
        self.get_logger().info('I2C server has been started')
        self.bme_bus = smbus2.SMBus(self.bme_port)
        time.sleep(0.5)
        self.init_bmi088()

        self.bme_timer = self.create_timer(0.5, self.bme_take_measurement)
        self.bmi_timer = self.create_timer(0.5, self.bmi_take_measurement)

    def twos_comp(self, val):
        """Calculate two's complement of a value."""
        return val if val <= 32767 else (-1) * (65535 + 1 - val)

    def init_bmi088(self):
        """Initialize BMI088 sensor."""
        port = "/dev/i2c-4"
        accelerometer_address = 0x18
        acc_enable = 0x7D
        normal_mode = 0x04
        self.bmi_bus = smbus2.SMBus(port)
        self.bmi_bus.write_byte_data(accelerometer_address, acc_enable, normal_mode)
        time.sleep(0.5)

    def yawpitchroll_calc(self, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z):
        """Calculate yaw, pitch, and roll from gyroscope and accelerometer data."""
        roll = math.atan2(acc_y, acc_z) * 180 / math.pi
        pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / math.pi
        yaw = math.atan2(gyro_y, gyro_x) * 180 / math.pi
        return yaw, pitch, roll

    def bmi_take_measurement(self):
        """Take measurement from BMI088 sensor."""
        accelerometer_address = 0x18
        gyroscope_address = 0x68
        g_force = 9.80665
        imu_msg = Imu()
        acc_err = 0.3
        gyro_err = 20.0  # 20mg
        kilo = 1000
        linear_acceleration_error = np.array([acc_err, 0, 0, 0, acc_err, 0, 0, 0, acc_err])
        angular_velocity_error = np.array([gyro_err, 0, 0, 0, gyro_err, 0, 0, 0, gyro_err])

        try:
            # Read Accelerometer
            acc_data = self.read_accelerometer_data(accelerometer_address)
            accel_x_in_ms2, accel_y_in_ms2, accel_z_in_ms2 = self.convert_accelerometer_data(acc_data, kilo, g_force)

            # Read Gyroscope
            gyro_data = self.read_gyroscope_data(gyroscope_address)
            rate_x, rate_y, rate_z = gyro_data

            # Calculate yaw, pitch, roll from gyroscope and accelerometer data using a complementary filter
            yaw, pitch, roll = self.yawpitchroll_calc(rate_x, rate_y, rate_z, accel_x_in_ms2, accel_y_in_ms2, accel_z_in_ms2)

            # Assemble IMU message
            imu_msg = self.assemble_imu_message(imu_msg, accel_x_in_ms2, accel_y_in_ms2, accel_z_in_ms2, linear_acceleration_error, rate_x, rate_y, rate_z, angular_velocity_error)
            self.bmi_publisher.publish(imu_msg)

        except Exception as e:
            self.handle_bmi_exception(e)

    def read_accelerometer_data(self, address):
        """Read data from the accelerometer."""
        data = [self.bmi_bus.read_byte_data(address, i) for i in range(0x12, 0x18)]
        return data

    def convert_accelerometer_data(self, data, kilo, g_force):
        """Convert accelerometer data to m/s^2."""
        acc_int16 = [self.twos_comp(data[i] * 256 + data[i + 1]) for i in range(0, 6, 2)]
        acc_range = self.bmi_bus.read_byte_data(0x18, 0x41)
        acc_in_mg = [acc_int16[i] / 32768 * kilo * pow(2, (acc_range + 1)) * 1.5 for i in range(3)]
        acc_in_ms2 = [acc_in_mg[i] / kilo * g_force for i in range(3)]
        return acc_in_ms2

    def read_gyroscope_data(self, address):
        """Read data from the gyroscope."""
        data = [self.bmi_bus.read_byte_data(address, i) for i in range(0x02, 0x08)]
        gyro_int16 = [self.twos_comp(data[i] * 256 + data[i + 1]) for i in range(0, 6, 2)]
        return gyro_int16

    def assemble_imu_message(self, imu_msg, acc_x, acc_y, acc_z, acc_err, rate_x, rate_y, rate_z, gyro_err):
        """Assemble the IMU message with the current data."""
        take_time = datetime.datetime.now()
        current_time = TimeSample(
            year=take_time.year,
            month=take_time.month,
            day=take_time.day,
            hour=take_time.hour,
            minute=take_time.minute,
            second=take_time.second
        )
        imu_msg.date_time = current_time
        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = acc_z
        imu_msg.linear_acceleration_covariance = acc_err
        imu_msg.angular_velocity.x = float(rate_x)
        imu_msg.angular_velocity.y = float(rate_y)
        imu_msg.angular_velocity.z = float(rate_z)
        imu_msg.angular_velocity_covariance = gyro_err
        return imu_msg

    def handle_bmi_exception(self, exception):
        """Handle exceptions that occur during BMI measurements."""
        port = "/dev/i2c-4"
        self.bmi_bus = smbus2.SMBus(port)
        self.bmi_bus.write_byte_data(0x18, 0x7D, 0x04)
        self.get_logger().info("Re-open Bus")
        self.get_logger().error('Exception: %r' % exception)

    def bme_take_measurement(self):
        """Take measurement from BME280."""
        address = 0x76
        try:
            calibration_params = bme280.load_calibration_params(self.bme_bus, address)
            data = bme280.sample(self.bme_bus, address, calibration_params)

            self.epoch_bme = int(time.time() * 1e6)

            bme_msg = BmeData(
                timestamp=self.epoch_bme,
                temperature=data.temperature,
                pressure=data.pressure,
                humidity=data.humidity
            )
            self.sm_flag_bme == 0
            self.bme_publisher.publish(bme_msg)
        except Exception as e:
            self.handle_bme_exception(e)

    def handle_bme_exception(self, exception):
        """Handle exceptions that occur during BME measurements."""
        self.wdt_bme = (int(time.time() * 1e6) - self.epoch_bme) / 1e6
        self.bme_bus.close()
        self.bme_bus = smbus2.SMBus(self.bme_port)
        self.get_logger().info('Re-open Bus')
        if self.wdt_bme > 5:
            self.get_logger().error('Service call failed: %r' % exception)
        if self.wdt_bme > 10 and self.sm_flag_bme == 0:
            self.sm_flag_bme = 1
            qgc_msg = QgcMode(qgc_mode=QGC_mode.STOP.value)
            self.qgc_mode_publisher.publish(qgc_msg)
            kayak_mode_msg = KayakMode(
                timestamp=int(time.time() * 1e6),
                tiger_mode=Kayak_mode.Kayak_HOLD_POSITION.value
            )
            self.kayak_mode_publisher.publish(kayak_mode_msg)


def main(args=None):
    """BME280 sensor node."""
    rclpy.init(args=args)                   # Starts ROS2 comm
    node = I2CDevices()                     # Node constructor ("node name")
    rclpy.spin(node)                        # Execution loop
    rclpy.shutdown()                        # Ends comm


if __name__ == '__main__':
    main()
