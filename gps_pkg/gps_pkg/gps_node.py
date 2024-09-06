#!/usr/bin/env python3
import logging
import serial
from pyubx2 import UBXReader
import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, ByteMultiArray

class GpsDriver(Node):
    def __init__(self):
        super().__init__("GPS_Driver")
        self.gps_init()
        self.get_logger().info("Node created")
        self.publisher_ = self.create_publisher(NavSatFix, "GPS_measurements", qos_profile=qos.qos_profile_sensor_data)
        self.publisher_raw_data = self.create_publisher(ByteMultiArray, "Raw_GPS_measurements", 100)
        self.timer_ = self.create_timer(1, self.gps_publisher)
        self.get_logger().info("UBlox GPS measurements start")

    def gps_init(self):
        port = "/dev/ttyACM0"
        baud = 9600
        self.gps_device = serial.Serial(port, baud)

    def get_header(self):
        """Returns ROS message header"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        return header

    def gps_raw_publisher(self):
        msg = ByteMultiArray()
        msg.data = list(self.raw_data)
        self.publisher_raw_data.publish(msg)

    def gps_publisher(self):
        ubr = UBXReader(self.gps_device)
        (self.raw_data, self.parsed_data) = ubr.read()
        self.gps_raw_publisher()
        msg_recv = UBXReader.parse(self.raw_data)
        msg = NavSatFix()
        fix = NavSatStatus()
        fix.service = msg_recv.numSV
        msg.header = self.get_header()
        msg.header.frame_id = "NEO-M8N"
        msg.status = fix
        msg.latitude = float(msg_recv.lat)
        msg.longitude = float(msg_recv.lon)
        msg.altitude = float(msg_recv.height)
        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.publisher_.publish(msg)
        self.get_logger().info(f"Altitude: {msg.altitude}")

def main(args=None):
    rclpy.init(args=args)
    node = GpsDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
