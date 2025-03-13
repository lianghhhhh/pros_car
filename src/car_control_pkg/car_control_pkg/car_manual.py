import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from car_control_pkg.utils import parse_control_signal, get_action_mapping


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")
        # 訂閱 car_control_signal，格式如 "Manual Control:<command>"
        self.subscription = self.create_subscription(
            String, "car_control_signal", self.key_callback, 10
        )
        # Publisher for 車輪控制
        self.rear_wheel_pub = self.create_publisher(
            Float32MultiArray, "car_C_rear_wheel", 10
        )
        self.front_wheel_pub = self.create_publisher(
            Float32MultiArray, "car_C_front_wheel", 10
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
        vel = get_action_mapping(action)
        rear_msg = Float32MultiArray()
        front_msg = Float32MultiArray()
        front_msg.data = vel[0:2]
        rear_msg.data = vel[2:4]
        self.rear_wheel_pub.publish(rear_msg)
        self.front_wheel_pub.publish(front_msg)

    def key_control(self, key):
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
