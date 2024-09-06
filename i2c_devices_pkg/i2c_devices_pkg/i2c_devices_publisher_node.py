"""I2C devices publisher node."""
# !/usr/bin/env python3
from lar_interfaces.msg import BmeData  # , Imu, TimeSample - if BMI is needed
from lar_interfaces.srv import SendBmeData  # , SendBmiData - if BMI is needed

import rclpy
from rclpy.node import Node


class I2CDevicesPublisher(Node):
    """I2C devices publisher node."""

    def __init__(self):
        """I2C devices publisher initial node, service, and publisher."""
        super().__init__('i2c_devices_publisher_node')
        self.get_logger().info('i2c_devices_publisher_node has been started')

        self.bme_client = self.create_client(SendBmeData, 'Measure_BME_280')
        while not self.bme_client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for BME280 server...')
        self.bme_request = SendBmeData.Request()
        self.bme_publisher_ = self.create_publisher(BmeData, 'BME_measurements', 10)

        self.timer_ = self.create_timer(2, self.i2c_publisher)  # Call function self.publish_news every 2s

    def i2c_publisher(self):
        """I2C publisher callback function."""
        self.call_bme280()

    def call_bme280(self):
        """BME280 send request for measurement."""
        self.bme_request.send_data = True
        future = self.bme_client.call_async(self.bme_request)
        future.add_done_callback(self.callback_call_bme280)

    def callback_call_bme280(self, future):
        """BME280 Service response callback."""
        try:
            self.bme_response = future.result()
            self.bme_publisher_.publish(self.bme_response.bme_data)
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)


def main(args=None):
    """I2C devices publisher node."""
    rclpy.init(args=args)               # Starts ROS2 comm
    node = I2CDevicesPublisher()        # Node constructor ("node name")
    rclpy.spin(node)                    # Execution loop
    rclpy.shutdown()                    # Ends comm


if __name__ == '__main__':
    main()
