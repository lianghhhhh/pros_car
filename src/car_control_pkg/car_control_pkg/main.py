import rclpy
from rclpy.executors import MultiThreadedExecutor
from car_control_pkg.nav_control_node import NavControlNode
from car_control_pkg.manual_control_node import ManualControlNode
from car_control_pkg.car_action_server import NavigationActionServer


def main(args=None):
    rclpy.init(args=args)
    car_control_node = NavControlNode(enable_nav_subscribers=True)
    manual_control_node = ManualControlNode()
    action_server = NavigationActionServer(car_control_node=car_control_node)
    executor = MultiThreadedExecutor()
    executor.add_node(car_control_node)
    executor.add_node(action_server)
    executor.add_node(manual_control_node)
    # Use a multi-threaded executor, no spin_once() calls in the execute_callback
    try:
        executor.spin()
    except KeyboardInterrupt:
        action_server.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
