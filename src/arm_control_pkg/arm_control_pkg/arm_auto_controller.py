from action_interface.action import ArmGoal
import time


class ArmAutoController:
    def __init__(self, arm_params, arm_commute_node, pybulletRobotController):
        self.arm_params = arm_params.get_arm_params()
        self.pybullet_robot_controller = pybulletRobotController
        self.arm_commute_node = arm_commute_node

    def arm_wave(self):
        pass

    def arm_ik_move(self):
        # t = self.pybullet_robot_controller.move_end_effector_laterally()
        # print(t[0:5])
        # self.pybullet_robot_controller.setJointPosition(position=t[0:5])
        t = self.pybullet_robot_controller.offset_from_end_effector(
            y_offset=0.1, z_offset=0.1
        )
        print(t[0:5])
        self.pybullet_robot_controller.setJointPosition(position=t[0:5])
        self.pybullet_robot_controller.draw_link_axes(link_name="camera_1")
        return ArmGoal.Result(success=True, message="success")

    def test(self):

        # for obj in pybullet world position test

        # imu_data = self.arm_commute_node.get_latest_imu_data()
        # obj_position_data = self.arm_commute_node.get_latest_object_coordinates(
        #     label="fire"
        # )
        # extrinsics = self.pybullet_robot_controller.calculate_imu_extrinsics(
        #     imu_world_quaternion=imu_data, link_name="camera_1", visualize=False
        # )
        # obj_pos_in_pybullet = self.pybullet_robot_controller.transform_object_to_world(
        #     T_world_to_imu=extrinsics,
        #     object_coords_imu=obj_position_data,
        #     visualize=True,
        # )
        # robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
        #     target_position=obj_pos_in_pybullet
        # )
        # for i in robot_angle:
        #     self.pybullet_robot_controller.setJointPosition(position=i)
        #     time.sleep(0.1)

        # for obj forward move test
        obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
            distance=0.4,
        )
        # robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
        #     target_position=obj_pos
        # )
        ans = self.pybullet_robot_controller.is_link_close_to_position(
            link_name="base_link", target_position=obj_pos, threshold=0.6
        )
        print(ans)
        # for i in robot_angle:
        #     self.pybullet_robot_controller.setJointPosition(position=i)
        #     time.sleep(0.1)

        # self.pybullet_robot_controller.setJointPosition(position=robot_angle)

        # move test
        # t = self.pybullet_robot_controller.generateInterpolatedTrajectory(
        #     [0.3, 0.3, 0.3]
        # )
        # for i in t:
        #     self.pybullet_robot_controller.setJointPosition(position=i)
        #     time.sleep(0.1)
        # self.pybullet_robot_controller.draw_link_axes(link_name="camera_1")
        return ArmGoal.Result(success=True, message="success")
