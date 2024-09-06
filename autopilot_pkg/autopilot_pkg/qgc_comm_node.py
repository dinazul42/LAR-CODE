"""QGC communication node."""
# !/usr/bin/env python3
from autopilot_pkg.lar_common import QGCMode, KayakMode
from hc_comm_utils import UdpServer, lar_dialect
from sensor_msgs.msg import NavSatFix
from lar_interfaces.msg import BmeData, Joystick, QgcMode, KayakMode as KayakModeMsg, Waypoint
from lar_interfaces.srv import Parameters
import rclpy
from rclpy import qos
from rclpy.node import Node
import time

class QGCComm(Node):
    """QGC communication node."""

    def __init__(self):
        """QGC communication initial node, services, subscribers and publisher."""
        super().__init__('qgc_comm_node')
        self.server = UdpServer(14575, self.parser, verbosity=1)  # This is assumed for all Kayaks
        self.mav = lar_dialect.MAVLink(self, srcSystem=1)  # write function in UDP

        # Publishers and subscribers
        self.bme_subscriber = self.create_subscription(BmeData, 'BME_measurements', self.chamber_status_msg, qos_profile=qos.qos_profile_sensor_data)
        self.gps_subscriber = self.create_subscription(NavSatFix, "GPS_measurements", self.gps_callback, qos_profile=qos.qos_profile_sensor_data)
        self.joystick_publisher = self.create_publisher(Joystick, 'joystick_data', 10)
        self.kayak_mode_subscriber = self.create_subscription(KayakModeMsg, 'kayak_mode', self.callback_change_mode, 10)
        self.kayak_mode_publisher = self.create_publisher(KayakModeMsg, 'kayak_mode', 10)
        self.waypoint_publisher = self.create_publisher(Waypoint, 'waypoint', 10)
        self.qgc_mode_publisher = self.create_publisher(QgcMode, 'qgc_mode', 10)

        # Services
        self.parameters_client = self.create_client(Parameters, 'parameters')
        while not self.parameters_client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Parameters server...')
        self.param_request = Parameters.Request()

        # ROS2 messages
        self.joystick_msg = Joystick()
        self.qgc_mode_msg = QgcMode()
        self.kayak_mode_msg = KayakModeMsg()

        # Constants
        self.current_qgc_mode = QGCMode.PREFLIGHT
        self.current_kayak_mode = KayakMode.HOLD_POSITION
        self.next_qgc_mode = QGCMode.PREFLIGHT
        self.next_kayak_mode = KayakMode.HOLD_POSITION

        self.waypoints_list = []
        self.total_num_of_waypoints = 0
        self.waypoints_seq = 0
        self.operator_request = False
        self.waypoint_ready = False
        self.joy_flag = False
        self.first_joy_msg = False
        self.joy_msg_timer = 0

        self.timer = self.create_timer(1, self.heartbeat_msg)  # heartbeat message timer

        self.get_logger().info('qgc_comm_node has been started')

    def check_state(self):
        """Check and update the state based on QGC mode."""
        if self.next_qgc_mode == QGCMode.PREFLIGHT and self.current_kayak_mode != KayakMode.HOLD_POSITION:
            self.current_kayak_mode = KayakMode.HOLD_POSITION
            self.current_qgc_mode = QGCMode.PREFLIGHT

        if self.next_qgc_mode == QGCMode.PAUSE and self.current_kayak_mode != KayakMode.IDLE:
            self.current_kayak_mode = KayakMode.IDLE
            self.current_qgc_mode = QGCMode.PAUSE

        if self.next_qgc_mode == QGCMode.PLAY and self.current_kayak_mode != KayakMode.GUIDED_MISSION and self.waypoint_ready:
            self.current_kayak_mode = KayakMode.GUIDED_MISSION
            self.current_qgc_mode = QGCMode.PLAY

        if self.next_qgc_mode == QGCMode.PAUSE and self.current_kayak_mode == KayakMode.IDLE and self.joy_flag:
            self.current_kayak_mode = KayakMode.JOYSTICK
            self.current_qgc_mode = QGCMode.PAUSE

    def heartbeat_msg(self):
        """Send heartbeat message using lar_dialect send function."""
        self.kayak_mode_msg.kayak_mode = self.current_kayak_mode
        self.kayak_mode_publisher.publish(self.kayak_mode_msg)

        self.qgc_mode_msg.qgc_mode = self.current_qgc_mode
        self.qgc_mode_publisher.publish(self.qgc_mode_msg)

        curr_time = int(time.time() * 1e6)
        if (curr_time - self.joy_msg_timer > 2 * 1e6) and self.first_joy_msg:
            self.joy_flag = False
            self.first_joy_msg = False
            self.get_logger().info("Exiting Joystick mode")
            self.check_state()

        self.mav.heartbeat_send(
            lar_dialect.MAV_TYPE_SUBMARINE, lar_dialect.MAV_AUTOPILOT_GENERIC, self.current_qgc_mode,
            1, lar_dialect.MAV_STATE_POWEROFF, mavlink_version=3, force_mavlink1=True)

    def parser(self, input_bytes: bytes):
        """UDP callback function, handle received messages."""
        try:
            msg_lst = self.mav.parse_buffer(input_bytes)
            if msg_lst is not None:
                for msg in msg_lst:
                    if msg.id != 0:
                        self.get_logger().info(f"Received message: {msg}")

                    if msg.id == lar_dialect.MAVLINK_MSG_ID_HEARTBEAT:
                        pass

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_SET_MODE:
                        self.qgc_mode_msg.qgc_mode = msg.base_mode
                        self.qgc_mode_publisher.publish(self.qgc_mode_msg)
                        self.operator_request = True
                        self.next_qgc_mode = msg.base_mode
                        self.check_state()

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                        self.param_request.get_set = True
                        self.param_request.get_param = msg.param_index
                        future = self.parameters_client.call_async(self.param_request)
                        future.add_done_callback(self.callback_parameters)

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                        for i in range(7):  # TODO set number of parameters
                            self.param_request.get_set = True
                            self.param_request.get_param = i
                            future = self.parameters_client.call_async(self.param_request)
                            future.add_done_callback(self.callback_parameters)

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_PARAM_SET:
                        self.param_request.get_set = False
                        self.param_request.set_param = msg.param_id
                        self.param_request.param_value = msg.param_value
                        future = self.parameters_client.call_async(self.param_request)
                        future.add_done_callback(self.callback_parameters)

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_MISSION_ITEM:
                        if msg.seq == self.waypoints_seq:
                            self.waypoints_list[self.waypoints_seq] = msg
                            self.waypoints_seq += 1
                            if self.total_num_of_waypoints != self.waypoints_seq:
                                self.mav.mission_request_send(0, 0, self.waypoints_seq)
                            else:
                                if None not in self.waypoints_list:
                                    self.waypoint_ready = True
                                    self.mav.mission_ack_send(0, 0, lar_dialect.MAV_MISSION_ACCEPTED)
                                    self.publish_waypoints_to_guidance()

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_MISSION_REQUEST:
                        waypoint = self.waypoints_list[msg.seq]
                        self.mav.mission_item_send(
                            0, 0, waypoint.seq, waypoint.frame, waypoint.command, waypoint.current,
                            waypoint.autocontinue, waypoint.param1, waypoint.param2, waypoint.param3,
                            waypoint.param4, waypoint.x, waypoint.y, waypoint.z)

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                        self.mav.mission_count_send(0, 0, self.total_num_of_waypoints, force_mavlink1=True)

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_MISSION_COUNT:
                        self.total_num_of_waypoints = msg.count
                        self.waypoints_seq = 0
                        self.waypoints_list = [None] * self.total_num_of_waypoints
                        self.mav.mission_request_send(0, 0, self.waypoints_seq)

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_MISSION_ACK:
                        pass

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                        self.publish_joystick_data(msg)
                        if not self.joy_flag:
                            self.get_logger().info("Entering Joystick mode")
                            self.joy_flag = True
                            self.first_joy_msg = True
                            self.operator_request = True
                            self.joy_msg_timer = int(time.time() * 1e6)
                            self.check_state()
                        elif self.joy_flag:
                            self.joy_msg_timer = int(time.time() * 1e6)

                    elif msg.id == lar_dialect.MAVLINK_MSG_ID_COMMAND_LONG:
                        self.mav.command_long_send(
                            0, 0, msg.command, msg.confirmation, msg.param1, msg.param2, msg.param3, msg.param4,
                            msg.param5, msg.param6, msg.param7, force_mavlink1=True)

        except lar_dialect.MAVError as e:
            self.get_logger().error(f"MAVLink error: {e}")
        except Exception as e:
            self.get_logger().error(f"Parser exception: {e}")

    def write(self, data: bytes):
        """UDP Write function."""
        self.server.write(data, ('168.254.1.82', 14570))

    def chamber_status_msg(self, msg=None):
        """Send BME280 data via UDP."""
        temperature = int(msg.temperature) * 100  # Temperature inside chamber. [C*100]
        pressure = int(msg.pressure / 68.9476) * 10  # Chamber Pressure. [PSI*10 (0-25)]
        humidity = int(msg.humidity) * 100  # Humidity inside chamber. [%*100 (0-100%)]
        self.mav.chamber_status_send(555, 1, pressure, temperature, humidity, force_mavlink1=True)

    def callback_parameters(self, future):
        """Parameters service response callback."""
        try:
            self.param_response = future.result()
            self.mav.param_value_send(
                self.param_response.param_id.encode(), self.param_response.param_value, self.param_response.param_type,
                self.param_response.param_count, self.param_response.param_index, force_mavlink1=True)
        except Exception as e:
            self.get_logger().error(f"callback_parameters - Service call failed: {e}")

    def gps_callback(self, msg):
        """GPS callback function."""
        try:
            self.mav.global_position_int_send(
                0, int(msg.latitude * 1e7), int(msg.longitude * 1e7), int(msg.altitude * 1e3), 0, 0, 0, 0, 0, 0, 0, force_mavlink1=True)
        except Exception as e:
            self.get_logger().error(f"GPS callback exception: {e}")

    def callback_change_mode(self, msg=None):
        """Change QGC mode according to Kayak mode."""
        if msg.kayak_mode == KayakMode.GUIDED_MISSION:
            self.current_qgc_mode = QGCMode.PLAY
        elif msg.kayak_mode == KayakMode.IDLE or msg.kayak_mode == KayakMode.JOYSTICK \
              or msg.kayak_mode == KayakMode.FIXED_MANEUVER:
            self.current_qgc_mode = QGCMode.PAUSE
        else:
            self.current_qgc_mode = QGCMode.PREFLIGHT

    def publish_waypoints_to_guidance(self):
        """Publish waypoints list to guidance node."""
        for waypoint in self.waypoints_list:
            waypoint_msg = Waypoint()
            waypoint_msg.seq = waypoint.seq
            waypoint_msg.command = waypoint.command
            waypoint_msg.alt_depth = waypoint.autocontinue
            waypoint_msg.stay_at_pos_time = waypoint.param1
            waypoint_msg.uncertainty_radius = waypoint.param2
            waypoint_msg.tool_speed = waypoint.param4
            waypoint_msg.latitude = waypoint.x
            waypoint_msg.longitude = waypoint.y
            waypoint_msg.altitude = waypoint.z

            self.waypoint_publisher.publish(waypoint_msg)

    def publish_joystick_data(self, msg):
        """Publish joystick data msg to motors node."""
        self.joystick_msg.chan1 = msg.chan1_raw
        self.joystick_msg.chan2 = msg.chan2_raw
        self.joystick_msg.chan3 = msg.chan3_raw
        self.joystick_msg.chan4 = msg.chan4_raw
        self.joystick_msg.chan5 = msg.chan5_raw
        self.joystick_msg.chan6 = msg.chan6_raw
        self.joystick_msg.chan7 = msg.chan7_raw
        self.joystick_msg.chan8 = msg.chan8_raw
        self.joystick_publisher.publish(self.joystick_msg)

def main(args=None):
    """QGC communication node."""
    rclpy.init(args=args)  # Starts ros2 comm
    node = QGCComm()  # Node constructor ("node name")
    rclpy.spin(node)
    rclpy.shutdown()  # Ends comm

if __name__ == '__main__':
    main()
