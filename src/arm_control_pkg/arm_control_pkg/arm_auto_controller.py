from action_interface.action import ArmGoal
import time
import math


class ArmAutoController:
    def __init__(
        self, arm_params, arm_commute_node, pybulletRobotController, arm_agnle_control
    ):
        self.arm_params = arm_params.get_arm_params()
        self.pybullet_robot_controller = pybulletRobotController
        self.arm_commute_node = arm_commute_node
        self.arm_agnle_control = arm_agnle_control

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

    def radians_to_degrees(self, radians_list):
        """Converts a list of angles from radians to degrees."""
        if not isinstance(radians_list, (list, tuple)):
            # Handle potential errors if input is not a list/tuple
            print("Error: Input must be a list or tuple of radians.")
            return []  # Or raise an error
        try:
            degrees_list = [math.degrees(rad) for rad in radians_list]
            return degrees_list
        except TypeError as e:
            print(
                f"Error converting radians to degrees: {e}. Ensure all elements are numbers."
            )
            return []  # Or raise an error

    def test_func(self):
        imu_data = self.arm_commute_node.get_latest_imu_data()
        obj_position_data = self.arm_commute_node.get_latest_object_coordinates(
            label="fire"
        )
        extrinsics = self.pybullet_robot_controller.calculate_imu_extrinsics(
            imu_world_quaternion=imu_data, link_name="camera_1", visualize=False
        )
        obj_pos_in_pybullet = self.pybullet_robot_controller.transform_object_to_world(
            T_world_to_imu=extrinsics,
            object_coords_imu=obj_position_data,
            visualize=True,
        )
        print(obj_pos_in_pybullet)
        is_close_pos = self.pybullet_robot_controller.is_link_close_to_position(
            link_name="base_link", target_position=obj_pos_in_pybullet, threshold=0.8
        )
        if is_close_pos:
            robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
                target_position=obj_pos_in_pybullet, steps=10
            )
            for i in robot_angle:
                self.pybullet_robot_controller.setJointPosition(position=i)
                degree = self.radians_to_degrees(i)
                degree[-1] = 90
                self.arm_agnle_control.arm_all_change(degree)
                self.arm_commute_node.publish_arm_angle()
                time.sleep(0.2)
        else:
            print("not close to the object")

    def test(self):
        self.test_func()
        # for obj in pybullet world position test---------------------

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

        # --------------------------------------------------------

        # for obj forward move test------------------------------------------

        # obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
        #     distance=0.4,
        # )
        # robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
        #     target_position=obj_pos
        # )

        # for obj reach test
        # ans = self.pybullet_robot_controller.is_link_close_to_position(
        #     link_name="base_link", target_position=obj_pos, threshold=0.6
        # )

        # for i in robot_angle:
        #     self.pybullet_robot_controller.setJointPosition(position=i)
        #     time.sleep(0.1)

        # ----------------------------------------------------------------

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
