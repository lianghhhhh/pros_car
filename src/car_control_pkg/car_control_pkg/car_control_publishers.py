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
    def publish_control(node, action, rear_wheel_pub, front_wheel_pub=None):
        """
        If the action is a string, it will be converted to a velocity array using the action mapping.
        If the action is a list, it will be used as the velocity array directly.
        """
        if not isinstance(action, str):
            vel = [action[0], action[1], action[0], action[1]]

        else:
            vel = get_action_mapping(action)

        if front_wheel_pub is None:
            # Only rear wheel publisher is available
            rear_msg = Float32MultiArray()
            rear_msg.data = vel  # Use entire velocity array [0:4]
            rear_wheel_pub.publish(rear_msg)
            node.get_logger().debug(f"Publishing all control data to rear wheel: {vel}")
        else:
            # Both publishers are available
            rear_msg = Float32MultiArray()
            front_msg = Float32MultiArray()
            front_msg.data = vel[0:2]
            rear_msg.data = vel[2:4]
            rear_wheel_pub.publish(rear_msg)
            front_wheel_pub.publish(front_msg)
            node.get_logger().debug(
                f"Publishing split control data: front={vel[0:2]}, rear={vel[2:4]}"
            )