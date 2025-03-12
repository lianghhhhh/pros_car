from rclpy.node import Node
from std_msgs.msg import String
from car_control_pkg.ros_communicator_config import ACTION_MAPPINGS
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path


class ROSCommunicator(Node):
    def __init__(self):
        super().__init__("ros2_manager")
        self.get_logger().info("car control node starting...")

        # --- Publisher Initialization ---
        self.publisher_dict = {
            "car_control": {
                "topic": "/car_control",
                "msg_type": String,
            },
            "car_C_rear_wheel": {
                "topic": "/car_C_rear_wheel",
                "msg_type": Float32MultiArray,
            },
            "car_C_front_wheel": {
                "topic": "/car_C_front_wheel",
                "msg_type": Float32MultiArray,
            },
            "selected_target_marker": {
                "topic": "/selected_target_marker",
                "msg_type": Marker,
            },
            "confirmed_initial_plan": {
                "topic": "/confirmed_initial_plan",
                "msg_type": Path,
            },
            "goal_pose": {
                "topic": "/goal_pose",
                "msg_type": PoseStamped,
            },
        }

        # Initialize Publishers
        self.publisher_instances = {
            key: self.create_publisher(pub["msg_type"], pub["topic"], 10)
            for key, pub in self.publisher_dict.items()
        }

        # --- Subscriber Initialization ---
        self.latest_data = {}

        self.subscriber_dict = {
            "car_control": {
                "topic": "/car_control",
                "msg_type": String,
                "callback": self._car_control_callback,
            },
            "amcl_pose": {
                "topic": "/amcl_pose",
                "msg_type": PoseWithCovarianceStamped,
                "callback": self._amcl_pose_callback,
            },
            "goal_pose": {
                "topic": "/goal_pose",
                "msg_type": PoseStamped,
                "callback": self._goal_pose_callback,
            },
            "lidar": {
                "topic": "/scan",
                "msg_type": LaserScan,
                "callback": self._lidar_callback,
            },
            "received_global_plan": {
                "topic": "/received_global_plan",
                "msg_type": Path,
                "callback": self._received_global_plan_callback,
            },
            "yolo_detection_position": {
                "topic": "/yolo/detection/position",
                "msg_type": PointStamped,
                "callback": self._yolo_detection_position_callback,
            },
            "camera_x_multi_depth": {
                "topic": "/camera/x_multi_depth_values",
                "msg_type": Float32MultiArray,
                "callback": self._camera_x_multi_depth_callback,
            },
        }

        # Initialize Subscribers
        for key, sub in self.subscriber_dict.items():
            self.latest_data[key] = None
            self.create_subscription(sub["msg_type"], sub["topic"], sub["callback"], 10)

    # --- Callback Functions ---
    def _car_control_callback(self, msg):
        self.latest_data["car_control"] = msg.data

    def _amcl_pose_callback(self, msg):
        self.latest_data["amcl_pose"] = msg

    def _goal_pose_callback(self, msg):
        self.latest_data["goal_pose"] = msg

    def _lidar_callback(self, msg):
        self.latest_data["lidar"] = msg

    def _received_global_plan_callback(self, msg):
        self.latest_data["received_global_plan"] = msg

    def _yolo_detection_position_callback(self, msg):
        self.latest_data["yolo_detection_position"] = msg

    def _camera_x_multi_depth_callback(self, msg):
        self.latest_data["camera_x_multi_depth"] = msg

    # --- Getter Functions ---
    def get_latest_data(self, key):
        return self.latest_data.get(key)

    # --- Publisher Functions ---
    def publish_data(self, key, data):
        try:
            publisher = self.publisher_instances.get(key)
            if publisher:
                publisher.publish(data)
            else:
                self.get_logger().error(f"No publisher found for key: {key}")
        except Exception as e:
            self.get_logger().error(f"Could not publish data for {key}: {e}")

    def publish_car_control(self, action_key, publish_rear=True, publish_front=True):
        if action_key not in ACTION_MAPPINGS:
            return

        velocities = ACTION_MAPPINGS[action_key]
        msg_rear = Float32MultiArray()
        msg_front = Float32MultiArray()

        msg_rear.data = velocities[:2]
        msg_front.data = velocities[2:]

        if publish_rear:
            self.publish_data("car_C_rear_wheel", msg_rear)
        if publish_front:
            self.publish_data("car_C_front_wheel", msg_front)

    def publish_selected_target_marker(self, x, y, z=0.0):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.publish_data("selected_target_marker", marker)

    def publish_confirmed_initial_plan(self, path_msg: Path):
        """
        確認路徑使用
        """
        self.publish_data("confirmed_initial_plan", path_msg)

    # publish goal_pose
    def publish_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        # self.publisher_goal_pose.publish(goal_pose)
        self.publish_data("goal_pose", goal_pose)
