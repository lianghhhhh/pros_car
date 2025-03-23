import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from car_control_pkg.utils import get_action_mapping, parse_control_signal


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
        Publish control commands to wheel publishers.
        If only rear_wheel_pub is provided, all data is sent to it.
        """
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


class BaseCarControlNode(Node):
    """Base class for car control nodes providing common functionality"""

    def __init__(self, node_name, enable_nav_subscribers=False):
        super().__init__(node_name)

        # Create common publishers
        self.rear_wheel_pub, self.front_wheel_pub = (
            CarControlPublishers.create_publishers(self)
        )

        # Create subscription to control signals
        self.subscription = CarControlPublishers.create_control_subscription(
            self, self.key_callback
        )

        # Navigation data storage
        self.latest_amcl_pose = None
        self.latest_goal_pose = None
        self.latest_global_plan = None
        self.stored_global_plan = None  # For storing a fixed plan
        self.plan_lock_active = False  # Flag to control plan updates
        self.latest_camera_depth = None
        self.latest_yolo_info = None

        # Create navigation data subscribers if enabled
        if enable_nav_subscribers:
            self._create_navigation_subscribers()

    def _create_navigation_subscribers(self):
        """Create all subscribers needed for navigation"""
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, 10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_pose_callback, 10
        )

        self.plan_sub = self.create_subscription(
            Path, "/received_global_plan", self._global_plan_callback, 1
        )

        self.camera_depth_sub = self.create_subscription(
            Float32MultiArray,
            "/camera/x_multi_depth_values",
            self._camera_depth_callback,
            10,
        )

        self.yolo_sub = self.create_subscription(
            Float32MultiArray, "/yolo/target_info", self._yolo_callback, 10
        )

        self.get_logger().info("Navigation subscribers created")

    # Callback methods for navigation data
    def _amcl_callback(self, msg):
        """Store latest AMCL pose"""
        self.latest_amcl_pose = msg

    def _goal_pose_callback(self, msg):
        """Store latest AMCL pose"""
        self.latest_goal_pose = msg

    def _global_plan_callback(self, msg):
        """Store latest global plan"""
        self.latest_global_plan = msg

        # If we haven't stored an initial plan yet, store this one
        if self.stored_global_plan is None:
            self.store_current_plan()

    def _camera_depth_callback(self, msg):
        """Store latest camera depth data"""
        self.latest_camera_depth = list(msg.data)

    def _yolo_callback(self, msg):
        """Store latest YOLO target info"""
        self.latest_yolo_info = list(msg.data)

    # Helper methods for navigation data access
    def get_car_position_and_orientation(self):
        """
        Get current car position and orientation

        Returns:
            Tuple containing (position, orientation) or (None, None) if data unavailable
        """
        if self.latest_amcl_pose:
            pos = self.latest_amcl_pose.pose.pose.position
            orientation = self.latest_amcl_pose.pose.pose.orientation
            return list(pos.x, pos.y, pos.z), list(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        return None, None

    def get_path_points(self, dynamic=True):
        """
        Get path points from global plan

        Args:
            dynamic: If True, always use the latest plan from Nav2
                   If False, use the stored plan (won't update with new Nav2 plans)

        Returns:
            List of (x, y) tuples or empty list if unavailable
        """
        path_points = []

        # Determine which plan to use
        plan_to_use = self.latest_global_plan if dynamic else self.stored_global_plan

        # If we want a static plan but don't have one, fall back to latest
        if not dynamic and not self.stored_global_plan:
            plan_to_use = self.latest_global_plan
            self.get_logger().warn("No stored plan available, using latest plan")

        # Extract points from the selected plan
        if plan_to_use and plan_to_use.poses:
            for pose in plan_to_use.poses:
                pos = pose.pose.position
                path_points.append(list(pos.x, pos.y))

        return path_points

    def store_current_plan(self):
        """Store the current global plan as the fixed plan"""
        if self.latest_global_plan:
            import copy

            self.stored_global_plan = copy.deepcopy(self.latest_global_plan)
            self.plan_lock_active = True
            self.get_logger().info("Global plan stored (locked)")
            return True
        else:
            self.get_logger().warn("No global plan available to store")
            return False

    def clear_stored_plan(self):
        """Clear the stored plan and revert to dynamic updates"""
        self.stored_global_plan = None
        self.plan_lock_active = False
        self.get_logger().info("Stored plan cleared (using dynamic updates)")

    # Common methods for all car control nodes
    def key_callback(self, msg):
        """Parse control signal and delegate to handle_command"""
        mode, command = parse_control_signal(msg.data)  # Parse signal
        if mode is None or command is None:
            return

        # Call the handle_command method that derived classes implement
        self.handle_command(mode, command)

    def publish_control(self, action):
        """Common method to publish control actions"""
        CarControlPublishers.publish_control(
            self, action, self.rear_wheel_pub, self.front_wheel_pub
        )

    # If you inherit from this class, you must implement this method
    def handle_command(self, mode, command):
        """Handle parsed commands - to be implemented by subclasses"""
        # Default implementation does nothing
        pass
