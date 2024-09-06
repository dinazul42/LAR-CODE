# !/usr/bin/env python3
import math
import queue
import time
from hc_comm_utils import SerialDevice, Device, DeviceType, lar_dialect, TcpClient, TcpDevice
import json
from threading import Thread
import logging

import rclpy
from rclpy.node import Node
from lar_interfaces.msg import Joystick

from enum import Enum

HOST = "192.168.0.81"
PORT = 22

class KleinSonar(TcpDevice): '''The KleinSonar class inherits from TcpDevice and is intended to represent a Klein sonar device.
 The constructor initializes the sonar device by connecting to it.'''

    def __init__(self, device_type: DeviceType, device_name: str, hostname: str = "", port: int = 0,
                 tx_log_filename: str = "", rx_log_filename: str = "", use_tx_queue: bool = False):
        """
        :param device_type: DeviceType enum.
        :param str device_name:
        :param int device_pID: The id of the sonar.
        :param int baud: baudrate
        :param str port: comm port
        :param str tx_log_filename: log filename to write transmitted bytes
        :param str rx_log_filename: log filename to write received bytes
        :param bool use_tx_queue: should data be sent to the device using a queue. If so send data by adding to queue.
        """


        self.connect_device()

    def parse_data(self, data):
        pass


class Sonar(Node):
    """Sonar node."""

    def __init__(self):
        """Sonar node initialization, subscriber."""
        super().__init__('sonar_node')



        self.get_logger().info('Sonar node has been started')

def main(args=None):
    """Sonar node."""
    rclpy.init(args=args)       # Starts ros2 comm
    node = Sonar()             # Node constructor ("node name")
    rclpy.spin(node)            # Executtion loop
    rclpy.shutdown()            # Ends comm


if __name__ == '__main__':
    main()