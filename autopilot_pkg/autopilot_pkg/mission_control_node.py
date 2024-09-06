#!/usr/bin/env python3

from autopilot_pkg.lar_common import QGCMode, KayakMode
from lar_interfaces.msg import QgcMode, KayakMode as KayakModeMsg
import rclpy
from rclpy.node import Node

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        self.current_kayak_mode = KayakMode.IDLE

        self.kayak_mode_msg = KayakModeMsg()
        # Subscribe to data from live and QGC messages
        self.mode_subscriber = self.create_subscription(QgcMode, 'qgc_mode', self.change_mode, 10)

        # Publish mode at 5Hz
        self.kayak_mode_publisher = self.create_publisher(KayakModeMsg, 'kayak_mode', 10)
        self.send_kayak_mode_timer = self.create_timer(0.2, self.send_kayak_mode)  # Call function at 5Hz

        self.get_logger().info('mission_control_node has been started')

    def change_mode(self, msg):
        """Callback to change the kayak mode based on QGC mode message."""
        self.get_logger().info(f"Received QGC mode: {msg.qgc_mode}")

        if msg.qgc_mode == QGCMode.PLAY:
            self.current_kayak_mode = KayakMode.GUIDED_MISSION
        elif msg.qgc_mode == QGCMode.STOP:
            self.current_kayak_mode = KayakMode.HOLD_POSITION
        elif msg.qgc_mode == QGCMode.PAUSE:
            self.current_kayak_mode = KayakMode.IDLE

    def send_kayak_mode(self):
        """Publish the current kayak mode."""
        self.kayak_mode_msg.kayak_mode = self.current_kayak_mode
        self.kayak_mode_publisher.publish(self.kayak_mode_msg)
        # self.get_logger().info(f"Published Kayak mode: {self.kayak_mode_msg.kayak_mode}")

def main(args=None):
    """Main function to start the mission control node."""
    rclpy.init(args=args)  # Start ROS2 communication
    node = MissionControl()  # Node constructor ("node name")
    rclpy.spin(node)
    rclpy.shutdown()  # End communication

if __name__ == '__main__':
    main()
