import rclpy
from rclpy.node import Node
from car_control_pkg.nav2_utils import (
    cal_distance,
    calculate_diff_angle,
)

class NavigationController:
    """Handles the core navigation logic for the car"""

    def __init__(self, car_control_node):
        """Initialize the navigation controller

        Args:
            car_control_node: Instance of BaseCarControlNode
        """
        self.car_control_node = car_control_node
        self.is_canceled = False
        self.waypoint_index = 0

    def check_prerequisites(self):
        """
        Check if all prerequisites for navigation are met

        Returns:
            tuple: (is_valid, error_message)
        """
        # Check if we have position data
        car_position, _ = self.car_control_node.get_car_position_and_orientation()
        if not car_position:
            return False, "Cannot obtain car position data"

        # Check if we have path points
        path_points = self.car_control_node.get_path_points(include_orientation=True)
        if not path_points:
            return False, "No path available for navigation"

        return True, None

    def navigation_step(self):
        """Execute a single step of navigation"""
        # Check for cancellation
        if self.is_canceled:
            return 'error', "Navigation was canceled", 0.0

        # Get current position
        car_position, car_orientation = self.car_control_node.get_car_position_and_orientation()
        if not car_position:
            return 'error', "Lost position data during navigation", 0.0

        # Get path points
        path_points = self.car_control_node.get_path_points(include_orientation=True)
        if not path_points:
            return 'error', "Path data not available", 0.0

        # ✅ FIRST PRIORITY: Check if we've reached the goal pose
        goal_pose = self.car_control_node.get_goal_pose()
        if goal_pose:
            try:
                # Calculate distance to final goal
                target_distance = cal_distance(
                    [car_position.x, car_position.y],
                    [goal_pose.x, goal_pose.y]
                )

                # Check if we're close enough to goal to stop
                if target_distance < 0.5:
                    # Send STOP command multiple times to ensure it's received
                    for _ in range(3):
                        self.car_control_node.publish_control("STOP")

                    # Log with high visibility
                    self.car_control_node.get_logger().info(
                        f"🎯 GOAL REACHED! Distance: {target_distance:.2f}m - STOPPING CAR"
                    )

                    # Mark navigation as complete
                    return 'completed', f"Goal reached successfully (dist={target_distance:.2f}m)", target_distance
            except Exception as e:
                self.car_control_node.get_logger().error(f"Error checking goal distance: {e}")

        # Rest of your navigation code should continue only if we haven't reached the goal...

        # Get next target point
        current_target = None
        if self.waypoint_index < len(path_points):
            current_target = path_points[self.waypoint_index]

        # If no more waypoints, we're done
        if current_target is None:
            return 'completed', "Reached end of path", target_distance

        # Extract position from waypoint
        try:
            target_x = current_target["position"][0]
            target_y = current_target["position"][1]
        except (KeyError, IndexError):
            self.waypoint_index += 1
            return 'continue', None, target_distance

        # Calculate distance to waypoint
        distance_to_waypoint = cal_distance(
            [car_position.x, car_position.y],
            [target_x, target_y]
        )

        # If we're close enough to waypoint, move to next one
        if distance_to_waypoint < 0.5:
            self.waypoint_index += 1
            return 'continue', None, target_distance

        # Calculate steering angle
        diff_angle = calculate_diff_angle(
            [car_position.x, car_position.y],
            [car_orientation.z, car_orientation.w],
            [target_x, target_y]
        )

        # Determine control action
        if diff_angle < 20 and diff_angle > -20:
            action_key = "FORWARD"
        elif diff_angle < -20 and diff_angle > -180:
            action_key = "CLOCKWISE_ROTATION"
        elif diff_angle > 20 and diff_angle < 180:
            action_key = "COUNTERCLOCKWISE_ROTATION"
        else:
            action_key = "STOP"

        # Execute control action
        self.car_control_node.publish_control(action_key)

        return 'continue', None, target_distance
