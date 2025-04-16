from action_interface.action import ArmGoal
from arm_control_pkg.pybullet_ik import PybulletRobotController


class ArmAutoController:
    def __init__(
        self,
        arm_angle_control_node,
        arm_params,
        arm_commute_node,
    ):
        self.arm_angle_control_node = arm_angle_control_node
        self.arm_params = arm_params.get_arm_params()
        self.arm_commute_node = arm_commute_node
        self.pybullet_robot_controller = PybulletRobotController(
            arm_params=arm_params,
        )

    def arm_wave(self):

        pass
