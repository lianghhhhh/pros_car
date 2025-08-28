from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from car_control_pkg.utils import get_action_mapping

class BaseCarControlNode(Node):
    """Base class for car control nodes providing common functionality"""

    def __init__(self, node_name):
        super().__init__(node_name)
        # Create common publishers
        self.rear_wheel_pub = self.create_publisher(Float32MultiArray, "car_C_rear_wheel", 10)
        self.front_wheel_pub = self.create_publisher(Float32MultiArray, "car_C_front_wheel", 10)

    def publish_control(self, action):
        """
        If the action is a string, it will be converted to a velocity array using the action mapping.
        If the action is a list, it will be used as the velocity array directly.
        """
        if not isinstance(action, str):
            vel = [action[0], action[1], action[0], action[1]]

        else:
            vel = get_action_mapping(action)

        if self.front_wheel_pub is None:
            # Only rear wheel publisher is available
            rear_msg = Float32MultiArray()
            rear_msg.data = vel  # Use entire velocity array [0:4]
            self.rear_wheel_pub.publish(rear_msg)
            self.get_logger().debug(f"Publishing all control data to rear wheel: {vel}")
        else:
            # Both publishers are available
            rear_msg = Float32MultiArray()
            front_msg = Float32MultiArray()
            front_msg.data = vel[0:2]
            rear_msg.data = vel[2:4]
            self.rear_wheel_pub.publish(rear_msg)
            self.front_wheel_pub.publish(front_msg)
            self.get_logger().debug(
                f"Publishing split control data: front={vel[0:2]}, rear={vel[2:4]}"
            )

