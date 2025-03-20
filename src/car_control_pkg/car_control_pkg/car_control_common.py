import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from car_control_pkg.utils import get_action_mapping


class CarControlPublishers:
    """Class to manage common car control publishers and methods"""

    @staticmethod
    def create_publishers(node):
        """Create and return common publishers for car control"""
        rear_wheel_pub = node.create_publisher(
            Float32MultiArray, "car_C_rear_wheel", 10
        )
        front_wheel_pub = node.create_publisher(
            Float32MultiArray, "car_C_front_wheel", 10
        )

        return rear_wheel_pub, front_wheel_pub

    @staticmethod
    def create_control_subscription(node, callback):
        """Create subscription for car control signals"""
        return node.create_subscription(String, "car_control_signal", callback, 10)

    @staticmethod
    def publish_control(node, action, rear_wheel_pub, front_wheel_pub):
        """Publish control commands to wheel publishers"""
        vel = get_action_mapping(action)
        rear_msg = Float32MultiArray()
        front_msg = Float32MultiArray()
        front_msg.data = vel[0:2]
        rear_msg.data = vel[2:4]
        rear_wheel_pub.publish(rear_msg)
        front_wheel_pub.publish(front_msg)
