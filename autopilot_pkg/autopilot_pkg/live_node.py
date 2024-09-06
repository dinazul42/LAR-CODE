"""
This file is responsible for system safety.

The node monitors data from sensors and if there
is an exception that requires a mission cancellation,
the node sends an ABORT message.
"""

import json
import threading as th
import time
from datetime import datetime
from lar_interfaces.msg import Parameter, BmeData, Imu, Vector3, KayakMode, WaypointReached, Status
from example_interfaces.msg import Float32
from autopilot_pkg.lar_common import ErrorCode, HardLimits, KayakMode

import rclpy
from rclpy.node import Node
from rclpy import qos

class LiveSafety(Node):
    """Live node."""

    def __init__(self):
        """Live initial node."""
        super().__init__('live_node')

        try:
            with open('/home/lar/Repositories/ros2_kayak/src/autopilot_pkg/autopilot_pkg/parameters.json', 'r') as f:
                self.parameters = json.load(f)
        except FileNotFoundError:
            self.get_logger().error("Parameter file not found.")
            self.parameters = {}

        # Subscribers
        self.param_subscriber = self.create_subscription(Parameter, 'parameter', self.callback_parameter, 10)
        self.bme_subscriber = self.create_subscription(BmeData, 'BME_measurements', self.callback_bme, qos_profile=qos.qos_profile_sensor_data)
        self.kayak_mode_subscriber = self.create_subscription(KayakMode, 'kayak_mode', self.callback_change_mode, 10)
        self.gps_subscriber = self.create_subscription(Vector3, 'GPS_measurements', self.callback_gps, qos_profile=qos.qos_profile_sensor_data)
        self.waypoint_subscriber = self.create_subscription(WaypointReached, 'waypoint_reached', self.initialize_timer, 10)

        # Publishers
        self.error_publisher = self.create_publisher(Status, 'status', 10)
        self.time_counter_publisher = self.create_publisher(Float32, 'timer', 10)

        # Initial flags and variables
        self.error_msg = Status()
        self.timer_msg = Float32()
        self.bme_measurement_received = False
        self.gps_msg_received = False
        self.bme_noise_counter = 0
        self.bme_ok = False
        self.gps_ok = False
        self.time_limit_ok = True
        self.timeout_ok = True
        self.timeout_val = 5  # TODO

        param = self.parameters.get('3')
        self.time_counter = int(param.get('param_value')) if param else 0

        # Timers
        self.limit_timer = self.create_timer(1, self.time_limit)
        self.time_counter_timer = self.create_timer(3, self.callback_publish_timer)
        self.ok_timer = self.create_timer(2, self.check_all_ok)
        self.gps_timer = self.create_timer(1, self.check_gps_arrivals)
        self.bme_timer = self.create_timer(2, self.check_bme_arrivals)
        self.curr_mode = KayakMode.HOLD_POSITION

        self.get_logger().info('live_node has been started')

    def callback_publish_timer(self):
        self.timer_msg.data = float(self.time_counter)
        self.time_counter_publisher.publish(self.timer_msg)

    def callback_bme(self, msg):
        """Callback for BME data."""
        self.bme_measurement_received = True
        if ((msg.timestamp == 0)
           or (msg.temperature > HardLimits.Max_Temperature)
           or ((msg.pressure / 1000) > HardLimits.Max_Pressure)
           or (msg.humidity > HardLimits.Max_Humidity)):
            self.bme_noise_counter += 1

        if self.bme_noise_counter >= 3:
            self.publish_error(ErrorCode.BME_Error, msg)
            self.bme_ok = False
        else:
            self.bme_ok = True
            self.bme_noise_counter = 0

    def callback_gps(self, msg):
        """Callback for GPS data."""
        self.gps_msg_received = True
        self.gps_ok = True

    def callback_change_mode(self, msg):
        """Callback for mode change."""
        if ((msg.kayak_mode == KayakMode.GUIDED_MISSION and self.curr_mode == KayakMode.HOLD_POSITION)
           or (msg.kayak_mode == KayakMode.HOLD_POSITION and self.curr_mode != KayakMode.HOLD_POSITION)):
            self.time_limit_ok = True
            param = self.parameters.get('3')
            self.time_counter = int(param.get('param_value')) if param else 0

        self.curr_mode = msg.kayak_mode

    def time_limit(self):
        """Check if time limit is reached."""
        if self.curr_mode == KayakMode.GUIDED_MISSION:
            self.time_counter -= 1
        if self.time_counter <= 0:
            self.publish_error(ErrorCode.Mission_Timeout_Error)
            self.time_limit_ok = False

    def initialize_timer(self, msg=None):
        """Every time we reach a waypoint we initialize the time_counter."""
        if msg and msg.reached:
            param = self.parameters.get('3')
            self.time_counter = int(param.get('param_value')) if param else 0

    def check_gps_arrivals(self):
        """Check if GPS messages are arriving."""
        if not self.gps_msg_received:
            self.publish_error(ErrorCode.GPS_Connection_Error)
            self.gps_ok = False
        self.gps_msg_received = False

    def check_bme_arrivals(self):
        """Check if BME messages are arriving."""
        if not self.bme_measurement_received:
            self.publish_error(ErrorCode.BME_Error)
            self.bme_ok = False
        self.bme_measurement_received = False

    def check_all_ok(self):
        """Check that all is ok every 2 seconds."""
        self.get_logger().info(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        self.get_logger().info(f"{self.gps_ok} {self.bme_ok} {self.timeout_ok}")
        if self.gps_ok and self.bme_ok and self.timeout_ok:
            self.publish_error(ErrorCode.No_Error)

    def callback_parameter(self, msg=None):
        """When there is a change in some parameter, this function updates the local parameter dictionary."""
        try:
            with open('/home/lar/Repositories/ros2_kayak/src/autopilot_pkg/autopilot_pkg/parameters.json', 'r') as f:
                self.parameters = json.load(f)
        except FileNotFoundError:
            self.get_logger().error("Parameter file not found.")

    def publish_error(self, error_code, msg=None):
        """Publish error message."""
        self.error_msg.timestamp = int(time.time() * 1e6)
        self.error_msg.status_code = error_code
        if msg:
            self.get_logger().info(f'Error: {error_code}, Message: {msg}')
        self.error_publisher.publish(self.error_msg)

def main(args=None):
    """Live node (safety)."""
    rclpy.init(args=args)  # Starts ros2 comm
    node = LiveSafety()  # Node constructor
    rclpy.spin(node)
    rclpy.shutdown()  # Ends comm

if __name__ == '__main__':
    main()
