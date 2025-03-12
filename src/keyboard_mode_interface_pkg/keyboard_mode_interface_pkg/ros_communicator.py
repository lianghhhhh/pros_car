import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ROSCommunicator(Node):
    def __init__(self):
        super().__init__("ros2_manager")

        # Publisher（發送指令給其他節點）
        self.car_control_publisher = self.create_publisher(
            String, "car_control_signal", 10
        )

        self.subscription = self.create_subscription(
            String, "menu_feedback", self.listener_callback, 10
        )

    def car_control_publish(self, command):
        """
        publish car control signal
        command format : "{mode}:keyboard"
        """
        msg = String()
        msg.data = command
        self.car_control_publisher.publish(msg)

    def listener_callback(self, msg):
        """監聽來自其他節點的回應"""
        self.get_logger().info(f"收到回應: {msg.data}")
