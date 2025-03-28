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
                include_orientation=True
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

        path_points = self.car_control_node.get_path_points(include_orientation=True)
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
                if not car_position:
                    self.get_logger().warn("Lost position data during navigation")
                    self.car_control_node.publish_control("STOP")
                    result.success = False
                    result.message = "Lost position data during navigation"
                    goal_handle.abort()
                    return result
                car_position_tmp, car_orientation_tmp = (
                    self.car_control_node.get_car_position_and_orientation()
                )
                car_position = [car_position_tmp.x, car_position_tmp.y]
                car_orientation = [car_orientation_tmp.z, car_orientation_tmp.w]
                path_points = self.car_control_node.get_path_points()
                goal_pose_tmp = self.car_control_node.get_goal_pose()
                goal_pose = [goal_pose_tmp.x, goal_pose_tmp.y]
                target_distance = cal_distance(car_position[0:2], goal_pose[0:2])
                if target_distance < 0.5:
                    action_key = "STOP"
                else:
                    target_points, orientation_points = self.get_next_target_point(car_position=car_position, path_points=path_points)
                    if target_points is None:
                        # We've reached the goal or have no more waypoints
                        self.get_logger().info("No more target points - goal reached or path complete")
                        self.car_control_node.publish_control("STOP")
                        result.success = True
                        result.message = "Navigation goal reached"
                        goal_handle.succeed()
                        return result

                    # Continue with navigation if we have a valid target point
                    diff_angle = calculate_diff_angle(car_position[0:2], car_orientation[0:2], target_points[0:2])
                    if diff_angle < 20 and diff_angle > -20:
                        action_key = "FORWARD"
                    elif diff_angle < -20 and diff_angle > -180:
                        action_key = "CLOCKWISE_ROTATION"
                    elif diff_angle > 20 and diff_angle < 180:
                        action_key = "COUNTERCLOCKWISE_ROTATION"
                    self.car_control_node.publish_control(action_key)
                feedback_msg = NavGoal.Feedback()
                # feedback_msg.distance_to_goal = float(target_distance)
                feedback_msg.distance_to_goal = float(0.0)
                goal_handle.publish_feedback(feedback_msg)
            else:
                self.car_control_node.publish_control("STOP")

    def get_next_target_point(self, car_position, path_points, min_required_distance=0.5):
        """Get next target point along the path"""
        if path_points is None or not path_points:
            self.get_logger().error("Error: No path points available!")
            return None, None

        # Check if we've reached the end of the path
        if self.index >= len(path_points) - 1:
            self.get_logger().info("Reached last waypoint")
            return None, None

        # Process normal waypoint
        while self.index < len(path_points) - 1:
            try:
                target_x = path_points[self.index]["position"][0]
                target_y = path_points[self.index]["position"][1]
                orientation_x = path_points[self.index]["orientation"][0]
                orientation_y = path_points[self.index]["orientation"][1]

            except (KeyError, IndexError) as e:
                self.get_logger().error(f"Invalid path point format: {e}")
                self.index += 1
                continue

            distance_to_target = cal_distance(car_position, (target_x, target_y))

            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                return [target_x, target_y], [orientation_x, orientation_y]

        # If we reach here, we're at the last point
        try:
            last_idx = len(path_points) - 1
            last_x = path_points[last_idx]["position"][0]
            last_y = path_points[last_idx]["position"][1]
            last_ox = path_points[last_idx]["orientation"][0]
            last_oy = path_points[last_idx]["orientation"][1]

            # Check if we're close enough to last point
            final_dist = cal_distance(car_position, (last_x, last_y))
            if (final_dist >= min_required_distance):
                return [last_x, last_y], [last_ox, last_oy]
        except (KeyError, IndexError):
            pass

        return None, None
