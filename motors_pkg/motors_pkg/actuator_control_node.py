import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32  # Float32 for desired angle, Int32 for A2D data
import os

class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')

        # Subscriber for the desired actuator angle
        self.angle_subscription = self.create_subscription(
            Float32,                 # Message type
            'actuator_angle',        # Topic name
            self.actuator_angle_callback,  # Callback function
            10                       # QoS profile
        )

        # Subscriber for the A2D feedback
        self.a2d_subscription = self.create_subscription(
            Int32,                   # Message type
            'a2d_topic',             # Topic name
            self.a2d_callback,       # Callback function
            10                       # QoS profile
        )

        # PWM setup
        self.base_path = "/sys/class/pwm/pwmchip0/pwm0/"
        os.system("echo 0 > /sys/class/pwm/pwmchip0/export")  # Export channel
        self.write_value(self.base_path + "period", "20000000")  # Set period to 20 ms

        # Define the angle range and corresponding PWM duty cycle range
        self.min_angle = -43
        self.max_angle = 43
        self.min_duty_cycle = 1000000  # 1 ms pulse for -43 degrees
        self.max_duty_cycle = 2000000  # 2 ms pulse for 43 degrees

        # Publisher for the current actuator position
        self.position_publisher = self.create_publisher(
            Float32,               # Message type
            'actuator_position',   # Topic name
            10                     # QoS profile
        )

        # Timer to publish the actuator position periodically
        self.timer = self.create_timer(0.1, self.publish_position)

    def write_value(self, path, value):
        try:
            with open(path, "w") as file:
                file.write(str(value))
        except Exception as e:
            self.get_logger().error(f"Failed to write to {path}: {e}")

    def actuator_angle_callback(self, msg):
        self.get_logger().info(f'Received angle command: {msg.data}')
        self.set_actuator_angle(msg.data)

    def set_actuator_angle(self, angle):
        if self.min_angle <= angle <= self.max_angle:
            # Map the angle to the PWM duty cycle
            duty_cycle = ((angle - self.min_angle) / (self.max_angle - self.min_angle) *
                          (self.max_duty_cycle - self.min_duty_cycle) + self.min_duty_cycle)
            self.write_value(self.base_path + "duty_cycle", str(duty_cycle))
            self.write_value(self.base_path + "enable", "1")  # Enable PWM with new duty cycle
        else:
            self.get_logger().error("Invalid angle")

    def a2d_callback(self, msg):
        a2d_value = msg.data  # Receive the A2D data
        angle = self.convert_a2d_to_angle(a2d_value)
        self.get_logger().info(f'Received A2D value: {a2d_value}, corresponding angle: {angle}')
        # Optional: Use the angle from A2D feedback to adjust the actuator
        # self.set_actuator_angle(angle)

    def convert_a2d_to_angle(self, a2d_value):
        # Update conversion for 16-bit ADC
        max_a2d = 65535  # Max A2D reading for a 16-bit ADC
        min_angle = -43
        max_angle = 43
        return (a2d_value / max_a2d) * (max_angle - min_angle) + min_angle

    def publish_position(self):
        # Read the current position from the ADC
        adc_value = self.read_adc_value()
        # Convert the ADC value to an angle
        angle = self.convert_a2d_to_angle(adc_value)
        # Publish the current position as a Float32 message
        position_msg = Float32()
        position_msg.data = angle
        self.position_publisher.publish(position_msg)
        self.get_logger().info(f'Published position: {angle} degrees')

    def read_adc_value(self):
        # Placeholder for actual ADC reading logic
        return 32768  # Example value, should be replaced with actual ADC read

def main(args=None):
    rclpy.init(args=args)
    actuator_control_node = ActuatorControlNode()
    rclpy.spin(actuator_control_node)
    actuator_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
