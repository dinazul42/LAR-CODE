import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from motors_pkg.torqeedo_motor_data_handler import TorqeedoMotorDataHandler
from motors_pkg.Enums import TorqeedoMotorCmdEnum, TorqeedoMotorConstantsEnum
from lar_interfaces.msg import Joystick, TwinMotorCmd, KayakMode
from autopilot_pkg.lar_common import Kayak_mode
from hc_comm_utils import SerialAPI

class ServoMotor(SerialAPI):
    def __init__(self, port: str, baud: int, parser_function, connect: bool = False, tx_log: str = "",
                 rx_log: str = "", verbosity: int = 1, read_timeout_sec=1, write_timeout_sec=0):
        super().__init__(port, baud, parser_function, connect, tx_log, rx_log, verbosity, read_timeout_sec, write_timeout_sec)
        self.speed = 0

    def update_speed(self, speed: int):
        if speed > TorqeedoMotorConstantsEnum.MAX_FORWARD_SPEED.value:
            self.speed = TorqeedoMotorConstantsEnum.MAX_FORWARD_SPEED.value
        elif speed < TorqeedoMotorConstantsEnum.MAX_BACKWARD_SPEED.value:
            self.speed = TorqeedoMotorConstantsEnum.MAX_BACKWARD_SPEED.value
        else:
            self.speed = speed

class Motors(Node):
    def __init__(self):
        super().__init__('motors_node')
        self.ServoMotorDeviceLeft = ServoMotor('/dev/ttyUSB0', 19200, self.parse_data)
        self.data_handler = TorqeedoMotorDataHandler()
        self.ServoMotorDeviceLeft.connect_to_host()

        self.kayak_mode_subscriber = self.create_subscription(KayakMode, 'kayak_mode', self.kayak_mode_callback, 10)
        self.joystick_subscriber = self.create_subscription(Joystick, 'joystick_data', self.callback_joystick, 10)
        self.twin_motor_subscriber = self.create_subscription(TwinMotorCmd, "twin_motor_rpm", self.twin_motor_callback, qos_profile=qos.qos_profile_sensor_data)

        self.curr_kayak_mode = Kayak_mode.Kayak_HOLD_POSITION.value
        self.get_logger().info('Motors node has been started')

    def kayak_mode_callback(self, msg):
        self.curr_kayak_mode = msg.kayak_mode
        if self.curr_kayak_mode in [Kayak_mode.Kayak_HOLD_POSITION.value, Kayak_mode.Kayak_IDLE.value]:
            self.ServoMotorDeviceLeft.update_speed(0)

    def parse_data(self, data: bytes):
        self.data_handler.add_data_to_buffer(data)
        self.data_handler.parse_buffer()
        parsed_data = self.data_handler.rx_queue.get()
        self.get_logger().info(str(parsed_data))

    def callback_joystick(self, msg):
        self.ServoMotorDeviceLeft.update_speed(int(msg.chan3 * 2 - 3000))
        speed_packet_left = self.data_handler.create_packet(TorqeedoMotorCmdEnum.SET_SPEED_ID, self.ServoMotorDeviceLeft.speed)
        self.ServoMotorDeviceLeft.tx_pipeline.put(speed_packet_left.bytes_to_send)
        time.sleep(0.01)

    def rpm_value_scaling(self, x):
        return (((x - 0) * (2000 - 1000)) / (100 - 0)) + 1000

    def twin_motor_callback(self, msg):
        self.ServoMotorDeviceLeft.update_speed(int(self.rpm_value_scaling(msg.left_motor_rpm) * 2 - 3000))
        speed_packet_left = self.data_handler.create_packet(TorqeedoMotorCmdEnum.SET_SPEED_ID, self.ServoMotorDeviceLeft.speed)
        self.ServoMotorDeviceLeft.tx_pipeline.put(speed_packet_left.bytes_to_send)
        time.sleep(0.01)

    def start_motor(self):
        while True:
            speed_packet = self.data_handler.create_packet(TorqeedoMotorCmdEnum.SET_SPEED_ID, self.ServoMotorDeviceLeft.speed)
            for _ in range(TorqeedoMotorConstantsEnum.SPEED_PACKETS_NUMBER.value):
                self.ServoMotorDeviceLeft.tx_pipeline.put(speed_packet.bytes_to_send)
                time.sleep(0.05)
            thruster_status_packet = self.data_handler.create_packet(TorqeedoMotorCmdEnum.THRUSTER_STATUS_ID, self.ServoMotorDeviceLeft.speed)
            self.ServoMotorDeviceLeft.tx_pipeline.put(thruster_status_packet.bytes_to_send)
            time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = Motors()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
