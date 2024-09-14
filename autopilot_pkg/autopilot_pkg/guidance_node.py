#!/usr/bin/env python3
from dataclasses import dataclass
import json
from datetime import datetime

from autopilot_pkg.lar_common import Error_code, Kayak_mode

from hc_comm_utils import lar_dialect
from lar_interfaces.msg import Waypoint, KayakMode, Joystick, MotorGuidance, Parameter, GuidanceData, Vector3, WaypointReached, PidLog, TwinMotorCmd

import rclpy
from rclpy.node import Node
from rclpy import qos
import math
import threading as th
import time
from functools import partial
import random   
import os
import utm
from ament_index_python.packages import get_package_share_directory
autopilot_pkg_path = get_package_share_directory('autopilot_pkg')
parameter_file_path = os.path.join(autopilot_pkg_path, 'autopilot_pkg', 'parameters.json')


@dataclass
class WaypointData():
    """This class defines new waypoint."""
    seq: int
    command: int
    alt_depth: int
    stay_at_pos_time: float
    uncertainty_radius: float
    tool_speed: float
    latitude: float
    longitude: float
    altitude: float

@dataclass
class Location:
    '''This class defines a 3-dimensional location vector (x=lat, y=lon, z=alt)'''
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class YPR:
    '''This class defines the orientation of the platform via Yaw Pitch Roll angles'''
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

kf1 = 1
pid_heading_param = [7.5*kf1, 0.75*kf1, 15*kf1, 0.3, 0, 0, 0]

class KayakPID:
    '''This class defines the PID controller and its methods'''
    def __init__(self, kp, ki, kd, ki_max, windup_gain=0, prev_error=0, i_accumulate=0, has_windup=False, pid_init=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ki_max = ki_max
        self.windup_gain = windup_gain
        self.prev_error = prev_error
        self.i_accumulate = i_accumulate
        self.has_windup = has_windup
        self.pid_init = pid_init

    def step(self, error, maxx, minx):
        self.i_accumulate += (error * self.ki)  # Accumulates the error multiplied with the integral gain (ki)
        ki_min = -1 * self.ki_max  # Limits the integral accumulation to specified range -ki_max to ki_max
        self.i_accumulate = self.constrain_float(self.i_accumulate, self.ki_max, ki_min)
        tmp = error * self.kp + (error - self.prev_error) / 0.1 * self.kd + self.i_accumulate
        self.prev_error = self.prev_error * 0.9 + error * 0.1

        if self.has_windup:
            if tmp > maxx:
                self.i_accumulate += (maxx - tmp) * self.windup_gain
            if tmp < minx:
                self.i_accumulate += (minx - tmp) * self.windup_gain

        return self.constrain_float(tmp, maxx, minx)

    def pid_initialize(self, prev_accumulator):
        self.pid_init = 0
        self.i_accumulate = prev_accumulator
        self.prev_error = 0

    @staticmethod
    def constrain_float(inx, maxx, minx):
        return max(minx, min(inx, maxx))

class Guidance(Node):
    """Guidance node."""
    def __init__(self):
        """Guidance initial node."""
        super().__init__('guidance_node')
        print(f"Trying to open: /home/lar-kayak/src/autopilot_pkg/autopilot_pkg/parameters.json")
        with open(parameter_file_path, 'r') as f:
            self.parameters = json.load(f)
        self.waypoint_subscriber = self.create_subscription(Waypoint, 'waypoint', self.callback_get_waypoint, 10)
        self.kayak_mode_subscriber = self.create_subscription(KayakMode, 'kayak_mode', self.callback_change_mode, 10)
        self.param_subscriber = self.create_subscription(Parameter, 'parameter', self.callback_parameter, 10)
        self.gps_subscriber = self.create_subscription(Vector3, "GPS_measurements", self.gps_callback, qos_profile=qos.qos_profile_sensor_data)
        self.motor_guidance_data_pub = self.create_publisher(MotorGuidance, "motor_guidance_data", 10)
        self.guidance_data_pub = self.create_publisher(GuidanceData, "guidance_data", 10)
        self.twin_motor_pub = self.create_publisher(TwinMotorCmd, "twin_motor_rpm", qos_profile=qos.qos_profile_sensor_data)
        self.waypoint_reached_publisher = self.create_publisher(WaypointReached, 'waypoint_reached', 10)
        self.mission_finished_publisher = self.create_publisher(WaypointReached, 'mission_finished', 10)
        self.pid_data_publisher = self.create_publisher(PidLog, 'pid_data_log', 10)
        self.timer_ = self.create_timer(1/20, self.navigation_mission)
        
        # Initializing motor PIDs
        self.heading_pid = KayakPID(*pid_heading_param[:4])

        # Initializing flags and variables
        self.first_att_msg_time = 0
        self.prev_time = 0
        self.delta_time = 0
        self.ins_mode = 1
        self.dist_to_dest_prev = 0
        self.motors_stopped = True
        self.time_diff = []
        self.curr_time = time.time()
        self.current_kayak_mode = Kayak_mode.Kayak_HOLD_POSITION.value
        self.current_mode = None
        self.prev_kayak_mode = 0
        self.waypoints_list = []
        self.total_num_of_waypoints = 0
        self.waypoints_seq = 0
        self.dest_bearing = 0
        self.bearing_calc = False
        self.got_waypoints = False
        self.in_mission = False
        self.target_waypoint = None
        self.waypoint_radius = 30
        self.current_location = Location()
        self.continue_mission = False
        self.current_ypr = YPR()
        self.current_depth = 0
        self.bag_flag = 0
        self.loop_thread = None
        self.last_mode_time = None
        self.current_heading = 0
        self.heading_cmd = 0
        self.angle_tol = 5  # Tolerance for heading angle in degrees
        self.max_turn_rate = 30  # Maximum turn rate in degrees per second
        self.rpm_max = 100
        self.rpm_left = 0
        self.rpm_right = 0

        # Initializing ROS2 messages
        self.joystick_msg = Joystick()
        self.kayak_mode_msg = KayakMode()
        self.motor_guidance_msg = MotorGuidance()
        self.guidance_data_msg = GuidanceData()
        self.waypoint_reached_msg = WaypointReached()
        self.mission_finished_msg = WaypointReached()

        self.get_logger().info('guidance_node has been started')

    def distance_heading(self, curr_lat, curr_long, dest_lat, dest_long, waypoint):
        '''This function is used to calculate the distance and the heading angle error to the next waypoint'''
        dist_to_dest = self.get_distance(curr_lat, curr_long, dest_lat, dest_long)
        heading_error = -self.wrap_180_cd(self.wrap_180_cd(self.current_heading) -  self.wrap_180_cd(self.dest_bearing))
        return dist_to_dest, heading_error

    def vel_est(self, tool_speed):
        motor_rpm = tool_speed * 252.5253
        return 0 if motor_rpm < 1000 else motor_rpm * 0.0004 - 0.05

    def stop_motors(self):
        '''This function gives the JET motor a command to stop'''
        self.get_logger().info('stop!!')
        timestamp = int(time.time() * 1e6)
        for chan in range(1, 9):
            setattr(self.joystick_msg, f'chan{chan}', 1500)
        self.joystick_msg.timestamp = timestamp
        self.joystick_publisher_.publish(self.joystick_msg)

    @staticmethod
    def wrap_180_cd(error):
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    def get_bearing(self, curr_lat, curr_long, dest_lat, dest_long):
        '''This function calculates the angle between the current position and the next waypoint'''
        curr_lat_r = math.radians(curr_lat)
        dest_lat_r = math.radians(dest_lat)
        curr_long_r = math.radians(curr_long)
        dest_long_r = math.radians(dest_long)

        x = math.cos(curr_lat_r) * math.sin(dest_lat_r) - math.sin(curr_lat_r) * math.cos(dest_lat_r) * math.cos(dest_long_r - curr_long_r)
        y = math.sin(dest_long_r - curr_long_r) * math.cos(dest_lat_r)
        return math.degrees(math.atan2(y, x))

    def get_distance(self, curr_lat, curr_long, dest_lat, dest_long):
        '''This function is called in the distance_heading function to calculate the distance to waypoint'''
        curr_lat_r = math.radians(curr_lat)
        dest_lat_r = math.radians(dest_lat)
        curr_long_r = math.radians(curr_long)
        dest_long_r = math.radians(dest_long)
        ra_ear = 6371000

        xx = math.sin((curr_lat_r - dest_lat_r) / 2) ** 2 + math.cos(dest_lat_r) * math.cos(curr_lat_r) * math.sin(
            (curr_long_r - dest_long_r) / 2) ** 2
        yy = 2 * math.atan2(math.sqrt(xx), math.sqrt(1 - xx))
        return yy * ra_ear

    def pid_log(self, pid, identifier: str):
        '''This function logs the data for each PID logger instance and publishes it to pid_data_log'''
        msg = PidLog()
        current_time = int(time.time() * 1e6)
        msg.timestamp = current_time
        msg.identifier = identifier
        msg.accumulator = float(pid.i_accumulate)
        msg.preverror = float(pid.prev_error)
        self.pid_data_publisher.publish(msg)

    def callback_get_waypoint(self, wp_msg):
        """Receive waypoints and create waypoint data class."""
        if wp_msg.seq == 0:
            self.waypoints_list.clear()

        waypoint = WaypointData(wp_msg.seq, wp_msg.command, wp_msg.alt_depth, wp_msg.stay_at_pos_time,
                                wp_msg.uncertainty_radius, wp_msg.tool_speed, wp_msg.latitude,
                                wp_msg.longitude, wp_msg.altitude)
        
        self.waypoints_list.append(waypoint)

    def gps_callback(self, msg):
        '''This callback takes the position from the gps and sets it to self.current_location'''
        self.current_location.x = msg.x
        self.current_location.y = msg.y
        self.current_location.z = 0.0
        self.current_speed = msg.z

    def callback_parameter(self, msg=None):
        """When there is a change in some parameter, this function updates the local parameter dictionary."""
        with open('/home/lar/Repositories/ros2_kayak/src/autopilot_pkg/autopilot_pkg/parameters.json', 'r') as f:
            self.parameters = json.load(f)

        # Update depth parameters
        self.update_pid_parameters('15', '16', '17', '18', self.depth_pid)

        # Update heading parameters
        self.update_pid_parameters('19', '20', '21', '22', self.heading_pid)

        # Update pitch parameters
        self.update_pid_parameters('23', '24', '25', '26', self.pitch_pid)

        # Update roll parameters
        self.update_pid_parameters('31', '32', '33', '34', self.roll_pid)

    def update_pid_parameters(self, kp_key, kd_key, ki_key, ki_max_key, pid_controller):
        kp_param = self.parameters.get(kp_key)
        kd_param = self.parameters.get(kd_key)
        ki_param = self.parameters.get(ki_key)
        ki_max_param = self.parameters.get(ki_max_key)
        pid_controller.kp = kp_param.get('param_value')
        pid_controller.kd = kd_param.get('param_value')
        pid_controller.ki = ki_param.get('param_value')
        pid_controller.ki_max = ki_max_param.get('param_value')
        pid_controller.pid_initialize(0)

    def callback_mode_change(self, msg):
        '''This function is a callback to a change in the Kayak_mode'''
        self.prev_kayak_mode = self.current_kayak_mode
        self.last_mode_time = datetime.now()        
        if msg.kayak_mode == Kayak_mode.Kayak_GUIDED_MISSION.value:
            '''If the Kayak_mode is set to guided mission raise the in_mission, bearing_calc and motors_stopped flags'''
            self.get_logger().info(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
            self.current_mode = msg.kayak_mode
            self.in_mission = True
            self.motors_stopped = False
            self.got_waypoints = False
            self.bearing_calc = False
            self.navigation_mission()
        elif msg.kayak_mode == Kayak_mode.Kayak_HOLD_POSITION.value:
            self.in_mission = False
            self.bag_flag = 0
            if not self.motors_stopped:
                self.stop_motors()
                self.motors_stopped = True
        elif msg.kayak_mode == Kayak_mode.Kayak_IDLE.value:
            self.in_mission = False
            if not self.motors_stopped:
                self.get_logger().info("Guidance node pause")
                self.stop_motors()
                self.motors_stopped = True

    def navigation_mission(self):
        '''This function is the navigation loop and is set on a timer to be called while in_mission'''
        if self.in_mission:
            self.prev_time = self.curr_time
            self.curr_time = time.time()
            self.delta_time = self.curr_time - self.prev_time
            if not self.got_waypoints:  # Checks if there is a waypoint that should be reached
                if self.waypoints_list:
                    self.target_waypoint = self.waypoints_list[0]
                    self.waypoint_radius = self.target_waypoint.uncertainty_radius
                    self.got_waypoints = True
            if self.got_waypoints:
                self.dest_bearing = self.get_bearing(self.current_location.x, self.current_location.y, self.target_waypoint.latitude, self.target_waypoint.longitude)
                dist_to_dest, heading_error = self.distance_heading(self.current_location.x, self.current_location.y, self.target_waypoint.latitude, self.target_waypoint.longitude, self.target_waypoint)
                if dist_to_dest < self.waypoint_radius:  # If the Kayak is within the waypoint radius
                    self.waypoint_arrival_time = time.time()
                    self.waypoint_reached_msg.reached = True
                    self.waypoint_reached_publisher.publish(self.waypoint_reached_msg)
                    self.get_logger().info("------------reached waypoint-----------")
                    self.waypoints_list.pop(0)  # Remove the reached waypoint from the waypoints list
                    if not self.waypoints_list:  # If the list of waypoints is now empty, the mission is over.
                        self.get_logger().info("-------------shutting down mission---------------")
                        self.mission_finished_msg.reached = True
                        self.mission_finished_publisher.publish(self.mission_finished_msg)
                        self.stop_motors()
                        self.motors_stopped = True
                        self.in_mission = False
                        self.current_kayak_mode = Kayak_mode.Kayak_HOLD_POSITION.value
                        self.current_mode = self.current_kayak_mode
                    else:  # If there are still waypoints in the waypoint list, raise the flags to calculate the next waypoint bearing
                        self.got_waypoints = False
                else:  # This part runs until the Kayak is within the waypoint radius
                    headind_th = th.Thread(target=self.pid_log, args=(self.heading_pid, "heading"))
                    headind_th.start()

                    self.heading_cmd = self.heading_pid.step(math.radians(heading_error), 0.5, -0.5)
                    self.current_heading = self.heading_cmd
                    if self.heading_cmd < -self.angle_tol:
                        self.rpm_left = max(0, self.rpm_max - (self.max_turn_rate * abs(heading_error) / 180))
                        self.rpm_right = self.rpm_max
                    elif self.heading_cmd > self.angle_tol:
                        self.rpm_left = self.rpm_max
                        self.rpm_right = max(0, self.rpm_max - (self.max_turn_rate * abs(heading_error) / 180))
                    else:
                        self.rpm_left = self.rpm_max
                        self.rpm_right = self.rpm_max

def main(args=None):
    """Guidance node."""
    rclpy.init(args=args)  # Starts ros2 comm
    node = Guidance()  # Node constructor ("node name")
    rclpy.spin(node)
    rclpy.shutdown()  # Ends comm

if __name__ == '__main__':
    main()
