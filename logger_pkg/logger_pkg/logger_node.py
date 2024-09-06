#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lar_interfaces.msg import TimeSample, BmeData, Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import ByteMultiArray
from hc_comm_utils import TcpClient
import os
from os import path
from datetime import datetime
import struct
from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr
from rosbags.typesys.types import std_msgs__msg__ByteMultiArray as BMA

# Constants
CONTROLLER_IP = "127.0.0.1"
CONTROLLER_PORT = 34866
GPS_IP = "127.0.0.1"
GPS_PORT = 34865


class Logger(Node):
    def __init__(self):
        super().__init__("logger")
        self.init_date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        self.dir_path = "/home/hc-up/repos/Kayak_ros/src/Kayak_logger_pkg/Kayak_logger_pkg/logs/"
        self.cur_date = self.init_date
        self.gps_bag_data = []
        self.bme_bag_data = []
        self.subscribers_init()
        self.tcp_init()
        self.rosbag_timer_ = self.create_timer(30, self.rosbag_callback)

    def get_cur_time(self):
        return datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")

    def set_cur_date(self):
        self.cur_date = self.get_cur_time()

    def rosbag_callback(self):
        self.gps_rosbag_logger()
        self.bme_rosbag_logger()

    def gps_rosbag_logger(self):
        data = b''.join(self.gps_bag_data)
        with Writer(self.dir_path + f"/ROSBAGS/GPS_logger_{self.get_cur_time()}_rosbag") as gps_rosbag_writer:
            gps_connection = gps_rosbag_writer.add_connection("Raw_GPS_measurements", BMA.__msgtype__, 'cdr', '')
            gps_rosbag_writer.write(gps_connection, 500, data)
        self.gps_bag_data = []

    def bme_rosbag_logger(self):
        data = b''.join(self.bme_bag_data)
        with Writer(self.dir_path + f"/ROSBAGS/BME_logger_{self.get_cur_time()}_rosbag") as bme_rosbag_writer:
            bme_connection = bme_rosbag_writer.add_connection("BME_measurements", BMA.__msgtype__, 'cdr', '')
            bme_rosbag_writer.write(bme_connection, 500, data)
        self.bme_bag_data = []

    def check_logger_tcp(self, msg: bytes):
        self.logger_control.pipeline.put(msg)
        self.log_controller()

    def GPS_TCP_publisher(self, tcp_message: bytes):
        self.GPS_TCP_client.pipeline.put(tcp_message)

    def callback_bme(self, msg: BmeData):
        date_time = self.cur_date
        s = str(msg)
        b = [msg.temperature, msg.pressure, msg.humidity]
        buf = struct.pack('%sf' % len(b), *b)
        self.bme_bag_data.extend((buf, "end message".encode()))

        file_path = self.dir_path + f"BME_logger_{date_time}.txt"
        mode = 'a' if path.exists(file_path) else 'w'
        with open(file_path, mode) as self.BME_logger:
            self.BME_logger.writelines(s + "\n")

        self.get_logger().info("Logging BME data to file")

    def callback_gps_raw(self, msg: ByteMultiArray):
        date_time = self.cur_date
        bytesToSend = b''.join(msg.data)
        self.GPS_TCP_client.write_to_host(bytesToSend)
        self.gps_bag_data.extend((bytesToSend, "end message".encode()))

        file_path = self.dir_path + f"GPS_logger_{date_time}.bin"
        mode = 'ab' if path.exists(file_path) else 'wb'
        with open(file_path, mode) as self.GPS_Raw_logger:
            self.GPS_Raw_logger.write(bytesToSend)

        self.get_logger().info("Logging raw GPS data to file")

    def create_log_files(self):
        date_time = self.init_date
        self.get_logger().info(self.dir_path)
        with open(self.dir_path + f"GPS_logger_{date_time}.bin", 'bw') as self.GPS_Raw_logger:
            self.GPS_Raw_logger.writelines(b'')

    def log_controller(self):
        while not self.logger_control.pipeline.empty():
            LOG_message = self.logger_control.pipeline.get(block=False).decode("utf-8")
            self.get_logger().info("Processing log controller message")
            if LOG_message == "new loggers\n":
                self.create_log_files()
            elif LOG_message == "gps\n":
                self.set_cur_date()
            elif LOG_message == "close gps bag\n":
                self.gps_rosbag_writer.close()
            elif LOG_message == "new gps bag\n":
                pass

    def subscribers_init(self):
        self.bme_subscriber = self.create_subscription(BmeData, "BME_measurements", self.callback_bme, 10)
        self.GPS_Raw_subscriber = self.create_subscription(ByteMultiArray, "Raw_GPS_measurements",
                                                           self.callback_gps_raw, 10)

    def tcp_init(self):
        self.logger_control = TcpClient(CONTROLLER_IP, CONTROLLER_PORT, self.check_logger_tcp, True)
        self.GPS_TCP_client = TcpClient(GPS_IP, GPS_PORT, self.GPS_TCP_publisher, True)


def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    node.get_logger().info("Welcome to the logging center")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
