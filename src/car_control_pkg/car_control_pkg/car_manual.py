import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from car_control_pkg.utils import parse_control_signal, get_action_mapping
from car_control_pkg.car_control_common import (
    CarControlPublishers,
)  # Import the common class


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")

        # Get publishers from common module
        self.rear_wheel_pub, self.front_wheel_pub = (
            CarControlPublishers.create_publishers(self)
        )

        # Create subscription using common method
        self.subscription = CarControlPublishers.create_control_subscription(
            self, self.key_callback
        )

    def key_callback(self, msg: String):
        # 使用 parse_control_signal 將字串解析成 mode 和 command
        mode, command = parse_control_signal(msg.data)
        if mode is None or command is None:
            return

        # 只處理 "Manual Control" 模式的指令
        if mode == "Manual Control":
            self.key_control(command)

    def publish_control(self, action):
        # Use common method for publishing
        CarControlPublishers.publish_control(
            self, action, self.rear_wheel_pub, self.front_wheel_pub
        )

    def key_control(self, key):
        # Keep your existing key mapping logic
        if key == "w":
            self.publish_control("FORWARD")
        elif key == "s":
            self.publish_control("BACKWARD")
        elif key == "a":
            self.publish_control("LEFT_FRONT")
        elif key == "d":
            self.publish_control("RIGHT_FRONT")
        elif key == "e":
            self.publish_control("COUNTERCLOCKWISE_ROTATION")
        elif key == "r":
            self.publish_control("CLOCKWISE_ROTATION")
        elif key == "z" or key == "q":
            self.publish_control("STOP")


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt，節點關閉中...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
