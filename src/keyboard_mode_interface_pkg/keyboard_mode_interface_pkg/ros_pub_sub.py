import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ROS2Manager(Node):
    def __init__(self):
        super().__init__("ros2_manager")

        # Publisher（發送指令給其他節點）
        self.car_control_publisher = self.create_publisher(
            String, "car_control_signal", 10
        )

        # Subscriber（接收來自其他節點的訊息）
        self.subscription = self.create_subscription(
            String, "menu_feedback", self.listener_callback, 10
        )

    def publish_car_signal(self, command):
        """發布選單的指令"""
        msg = String()
        msg.data = command
        self.car_control_publisher.publish(msg)

    def listener_callback(self, msg):
        """監聽來自其他節點的回應"""
        self.get_logger().info(f"收到回應: {msg.data}")
