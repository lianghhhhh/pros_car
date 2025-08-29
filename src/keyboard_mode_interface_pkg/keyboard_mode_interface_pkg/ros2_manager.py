from rclpy.node import Node
from std_msgs.msg import String
from keyboard_mode_interface_pkg.arm_action_client import ArmActionClient
from keyboard_mode_interface_pkg.car_action_client import CarActionClient


class ROS2Manager(Node):
    def __init__(self):
        super().__init__("ros2_manager")

        # Publisher（發送指令給其他節點）
        self.car_control_publisher = self.create_publisher(
            String, "car_control_signal", 10
        )

        self.arm_control_publisher = self.create_publisher(
            String, "arm_control_signal", 10
        )

        # create car action client
        self.car_action_client = CarActionClient(self)

        # create arm action client
        self.arm_action_client = ArmActionClient(self)

    def publish_car_signal(self, command):
        """發布選單的指令"""
        msg = String()
        msg.data = command
        self.car_control_publisher.publish(msg)

    def publish_arm_signal(self, command):
        msg = String()
        msg.data = command
        self.arm_control_publisher.publish(msg)
