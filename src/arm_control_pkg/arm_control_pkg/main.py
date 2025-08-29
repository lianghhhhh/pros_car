import rclpy
from rclpy.executors import MultiThreadedExecutor
from arm_control_pkg.utils import load_arm_parameters
from arm_control_pkg.arm_manual import ManualControlNode
from arm_control_pkg.arm_commute_node import ArmCummuteNode
from arm_control_pkg.arm_angle_control import ArmAngleControl
from arm_control_pkg.arm_action_server import ArmActionServer
from arm_control_pkg.pybullet_ik import PybulletRobotController
from arm_control_pkg.arm_auto_controller import ArmAutoController


def main(args=None):
    rclpy.init(args=args)
    arm_params = load_arm_parameters("arm_control_pkg")
    arm_angle_control = ArmAngleControl(arm_params=arm_params)
    arm_commute_node = ArmCummuteNode(
        arm_params=arm_params, arm_angle_control=arm_angle_control
    )
    pybulletRobotController = PybulletRobotController(
        arm_params=arm_params,
        arm_angle_control_node=arm_angle_control,
    )
    arm_auto_controller = ArmAutoController(
        arm_commute_node=arm_commute_node,
        pybulletRobotController=pybulletRobotController,
        arm_angle_control=arm_angle_control,
    )
    arm_action_server = ArmActionServer(
        arm_auto_controller=arm_auto_controller
    )
    arm_manual_node = ManualControlNode(
        arm_commute_node=arm_commute_node,
        arm_angle_control_node=arm_angle_control,
        arm_params=arm_params,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(arm_commute_node)
    executor.add_node(arm_manual_node)
    executor.add_node(arm_action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # action_server.get_logger().info("Keyboard interrupt, shutting down...")
        pass
    finally:
        # action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
