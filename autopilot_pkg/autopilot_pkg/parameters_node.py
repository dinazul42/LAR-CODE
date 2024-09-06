"""parameters node."""
# !/usr/bin/env python3
import json
from enum import Enum, auto

from lar_interfaces.msg import Parameter
from lar_interfaces.srv import Parameters

import rclpy
from rclpy.node import Node

class ParametersID(Enum):
    """Parameters enum."""
    CRUISE_SPEED_ID = auto()
    CRUISE_MIN_SPEED_ID = auto()
    MAXBAT_VOLTAGE_ID = auto()
    MINBAT_VOLTAGE_ID = auto()
    LIMIT_DEPTH_ID = auto()
    LIMIT_TIME_ID = auto()
    LIMIT_P0_ID = auto()
    LIMIT_RELAY_BCJO = auto()
    PID_V_PITCH_KP_ID = auto()
    PID_V_PITCH_KD_ID = auto()
    PID_V_PITCH_KI_ID = auto()
    PID_V_PITCH_KI_MAX_ID = auto()
    PID_V_PITCH_MIN_ID = auto()
    PITCH_V_MAX_ID = auto()
    PID_V_DEPTH_KP_ID = auto()
    PID_V_DEPTH_KD_ID = auto()
    PID_V_DEPTH_KI_ID = auto()
    PID_V_DEPTH_KI_MAX_ID = auto()
    PID_V_ROLL_KP_ID = auto()
    PID_V_ROLL_KD_ID = auto()
    PID_V_ROLL_KI_ID = auto()
    PID_V_ROLL_KI_MAX_ID = auto()
    PID_V_HDG_KP_ID = auto()
    PID_V_HDG_KD_ID = auto()
    PID_V_HDG_KI_ID = auto()
    PID_V_HDG_KI_MAX_ID = auto()
    PID_V2_HDG_KP_ID = auto()
    PID_V2_HDG_KD_ID = auto()
    PID_V2_HDG_KI_ID = auto()
    PID_V2_HDG_KI_MAX_ID = auto()
    PID_V3_HDG_KP_ID = auto()
    PID_V3_HDG_KD_ID = auto()
    PID_V3_HDG_KI_ID = auto()
    PID_V3_HDG_KI_MAX_ID = auto()
    PID_V2_PITCH_KP_ID = auto()
    PID_V2_PITCH_KD_ID = auto()
    PID_V2_PITCH_KI_ID = auto()
    PID_V2_PITCH_KI_MAX_ID = auto()
    PID_V2_PITCH_MIN_ID = auto()
    PITCH_V2_MAX_ID = auto()
    PID_V2_DEPTH_KP_ID = auto()
    PID_V2_DEPTH_KD_ID = auto()
    PID_V2_DEPTH_KI_ID = auto()
    PID_V2_DEPTH_KI_MAX_ID = auto()
    PID_V2_ROLL_KP_ID = auto()
    PID_V2_ROLL_KD_ID = auto()
    PID_V2_ROLL_KI_ID = auto()
    PID_V2_ROLL_KI_MAX_ID = auto()
    PID_V3_PITCH_KP_ID = auto()
    PID_V3_PITCH_KD_ID = auto()
    PID_V3_PITCH_KI_ID = auto()
    PID_V3_PITCH_KI_MAX_ID = auto()
    PID_V3_PITCH_MIN_ID = auto()
    PITCH_V3_MAX_ID = auto()
    PID_V3_DEPTH_KP_ID = auto()
    PID_V3_DEPTH_KD_ID = auto()
    PID_V3_DEPTH_KI_ID = auto()
    PID_V3_DEPTH_KI_MAX_ID = auto()
    PID_V3_ROLL_KP_ID = auto()
    PID_V3_ROLL_KD_ID = auto()
    PID_V3_ROLL_KI_ID = auto()
    PID_V3_ROLL_KI_MAX_ID = auto()
    WINDUP_HDG_ID = auto()
    WINDUP_HDG_GAIN_ID = auto()
    PARAM_INS_EULER_RST_ID = auto()
    PARAM_INS_POS_RST_ID = auto()
    WING_LEFT_ZERO_ID = auto()
    WING_RIGHT_ZERO_ID = auto()
    WING_UPPER_ZERO_ID = auto()
    WING_SERVO_OFFSET_ID = auto()
    TARGET_Z_ID = auto()
    SET_SPEED_ID = auto()
    STEP_MAIN_MOTOR_ID = auto()
    SD_NUM_ID = auto()
    LIMIT_MAX_PRSR_ID = auto()
    CRUISE_SPEED_FACT_ID = auto()
    WING_UPPER_OFF_ID = auto()
    WING_DIVE_POS_ID = auto()
    DIVE_CNST_DTH_ID = auto()
    DIVE_CNST_WP_ID = auto()
    SET_THR_DELAY_ID = auto()
    SET_START_SPD_ID = auto()
    SHUTDOWN_ID = auto()
    ONBOARD_PARAM_COUNT = auto()  # do not touch. Do not put values below this. You may add on top of it.

class ParametersNode(Node):
    """parameters node."""

    def __init__(self):
        """Parameters initial node, services, subscribers and publisher."""
        super().__init__('parameters_node')
        self.server_ = self.create_service(Parameters, 'parameters', self.callback_parameters)
        self.get_logger().info('Parameters server has been started')

        self.parameter_msg = Parameter()
        self.parameter_publisher_ = self.create_publisher(Parameter, 'parameter', 10)

        self.parameter_timer = self.create_timer(20, self.parameter_pub)  # call pub with frequency of 20Hz

        self.parameter_list = [0] * 10  # [None] * ParametersID.ONBOARD_PARAM_COUNT.value
        try:
            with open('/home/lar/Repositories/ros2_kayak/src/autopilot_pkg/autopilot_pkg/parameters.json', 'r') as f:
                self.parameters = json.load(f)
        except FileNotFoundError:
            self.get_logger().error("Parameter file not found.")
            self.parameters = {}
        self.get_logger().info('parameters_node has been started')

    def callback_parameters(self, request, response):
        """Parameters callback function from QGC parameters request."""
        try:
            if request.get_set:  # True = get ; False = set
                param = self.parameters.get(str(request.get_param))
            else:
                for key, param in self.parameters.items():
                    if param.get('param_id') == request.set_param:
                        break
                param['param_value'] = request.param_value
                self.parameters[key] = param
                with open('/home/lar/Repositories/ros2_kayak/src/autopilot_pkg/autopilot_pkg/parameters.json', 'w') as f:
                    json.dump(self.parameters, f, separators=(',\n', ': '))

                self.parameter_msg.param_id = param.get('param_id')
                self.parameter_msg.param_value = param.get('param_value')
                self.parameter_msg.param_type = param.get('param_type')
                self.parameter_msg.param_count = len(self.parameters)
                self.parameter_msg.param_index = int(key)

                self.parameter_publisher_.publish(self.parameter_msg)
                self.get_logger().info(str(self.parameter_msg))

            response.param_id = str(param.get('param_id'))
            response.param_value = float(param.get('param_value'))
            response.param_type = param.get('param_type')
            response.param_count = len(self.parameters)
            response.param_index = request.get_param

        except Exception as e:
            self.get_logger().error(f'Parameter value error: {e}')
            response.param_id = 'error'
            response.param_value = 0.0
            response.param_type = 0
            response.param_count = 0
            response.param_index = request.get_param

        return response

    def parameter_pub(self):
        try:
            with open('/home/lar/Repositories/ros2_kayak/src/autopilot_pkg/autopilot_pkg/parameters.json', 'r') as f:
                params = json.load(f)
            for key, param in params.items():
                param_msg = Parameter()
                param_msg.param_id = param.get('param_id')
                param_msg.param_value = float(param.get('param_value'))
                param_msg.param_type = param.get('param_type')
                param_msg.param_count = len(params)
                param_msg.param_index = int(key)
                self.parameter_publisher_.publish(param_msg)
        except FileNotFoundError:
            self.get_logger().error("Parameter file not found.")

def main(args=None):
    """Parameters node."""
    rclpy.init(args=args)  # Starts ros2 comm
    node = ParametersNode()  # Node constructor ("node name")
    rclpy.spin(node)
    rclpy.shutdown()  # Ends comm

if __name__ == '__main__':
    main()
