import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.automated_mode = False
        self.dead_man_switch = False

    def joy_callback(self, msg):
        # Button 'X' (index 0 on many controllers) for automated mode
        if msg.buttons[0] == 1:
            self.automated_mode = True
            self.get_logger().info("Automated mode enabled")
        # Button 'O' (index 1) for manual mode
        elif msg.buttons[1] == 1:
            self.automated_mode = False
            self.get_logger().info("Manual mode enabled")


def main():
    rclpy.init()
    node = JoyControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 topic echo /cmd_vel