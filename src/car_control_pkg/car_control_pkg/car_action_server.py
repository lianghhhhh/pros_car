import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import NavGoal
from car_control_pkg.car_control_common import BaseCarControlNode
from car_control_pkg.nav2_utils import (
    get_yaw_from_quaternion,
    get_direction_vector,
    get_angle_to_target,
    calculate_angle_point,
    quaternion_to_euler,
    calculate_angle_to_target,
    round_to_decimal_places,
    cal_distance,
    calculate_diff_angle,
)
import time


class NavigationActionServer(Node):
    def __init__(self, car_control_node):
        super().__init__("navigation_action_server_node")
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.car_control_node = car_control_node
        self.get_logger().info("Navigation Action Server initialized")
        self.index = 0
        self.cancel_flag = 0

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        requested_mode = goal_request.mode
        if requested_mode == "Manual_Nav":
            car_position, _ = self.car_control_node.get_car_position_and_orientation()
            path_points = self.car_control_node.get_path_points(
                dynamic=True, include_orientation=True
            )
            if not car_position or not path_points:
                self.get_logger().error("Cannot start navigation: Missing data")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Enter the cancel callback")
        self.car_control_node.publish_control("STOP")
        self.cancel_flag = 1
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Navigation action callback"""
        self.index = 0  # Reset for new navigation

        # Create result message
        result = NavGoal.Result()

        # Initial checks: Get position and path
        car_position, car_orientation = (
            self.car_control_node.get_car_position_and_orientation()
        )
        if not car_position:
            self.get_logger().error("Navigation failed: No position data")
            result.success = False
            result.message = "Cannot obtain car position data"
            goal_handle.abort()
            return result

        path_points = self.car_control_node.get_path_points(
            dynamic=True, include_orientation=True
        )
        if not path_points:
            self.get_logger().error("Navigation failed: No path points available")
            result.success = False
            result.message = "No path available for navigation"
            goal_handle.abort()
            return result

        # self.get_logger().info(f"Starting navigation with {len(path_points)} waypoints")

        # Create rate controller
        rate = self.create_rate(10)
        self.cancel_flag = 0
        # Main navigation loop
        while rclpy.ok():
            # First give executor time to process callbacks
            rate.sleep()

            if self.cancel_flag == 0:
                car_position, car_orientation = (
                    self.car_control_node.get_car_position_and_orientation()
                )
                if not car_position:
                    self.get_logger().warn("Lost position data during navigation")
                    self.car_control_node.publish_control("STOP")
                    result.success = False
                    result.message = "Lost position data during navigation"
                    goal_handle.abort()
                    return result

                # Extract position and orientation
                car_position = car_position[0:3]
                car_orientation = car_orientation[2:4]

                # Calculate next target point
                target_x, target_y = self.get_next_target_point(
                    car_position, path_points=path_points
                )
                final_pos = path_points[-1]["position"][0:2]
                target_distance = cal_distance(car_position, final_pos)

                # Check if goal reached
                if target_x is None or target_distance < 0.5:
                    # self.get_logger().info(
                    #     f"Goal reached! Distance: {target_distance:.2f}m"
                    # )
                    self.car_control_node.publish_control("STOP")
                    result.success = True
                    result.message = "Navigation completed successfully"
                    goal_handle.succeed()
                    return result

                # Calculate angle difference to determine control action
                diff_angle = calculate_diff_angle(
                    car_position, car_orientation, target_x, target_y
                )

                # Determine control action
                if -20 < diff_angle < 20:
                    action_key = "FORWARD"
                elif diff_angle < -20 and diff_angle > -180:
                    action_key = "CLOCKWISE_ROTATION"
                elif diff_angle > 20 and diff_angle < 180:
                    action_key = "COUNTERCLOCKWISE_ROTATION"
                else:
                    action_key = "STOP"

                # Send control command
                self.car_control_node.publish_control(action_key)

                # Send feedback
                feedback_msg = NavGoal.Feedback()
                feedback_msg.distance_to_goal = float(target_distance)
                goal_handle.publish_feedback(feedback_msg)
            else:
                self.car_control_node.publish_control("STOP")

    def get_next_target_point(
        self, car_position, path_points, min_required_distance=0.5
    ):
        if path_points is None:
            self.get_logger().error(
                "Error: global_plan_msg is None or poses is missing!"
            )
            return None, None
        while self.index < len(path_points) - 1:
            target_x = path_points[self.index]["position"][0]
            target_y = path_points[self.index]["position"][1]
            distance_to_target = cal_distance(car_position, (target_x, target_y))
            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                return target_x, target_y
        return None, None
