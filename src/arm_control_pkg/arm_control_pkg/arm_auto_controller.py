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

    def test(self):
        while 1:
            self.follow_obj()
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

    def follow_obj(self, label="fire", target_depth=0.3):
        """
        跟隨指定標籤的物體，並嘗試保持固定的深度距離。
        當物體位置已經接近目標位置時，將不移動機械臂。

        Args:
            label (str): 要跟隨的物體標籤
            target_depth (float): 想要保持的目標深度距離（單位：米）

        Returns:
            ArmGoal.Result: 操作結果
        """
        # 獲取物體座標 (相機座標系)
        obj_position_data = self.arm_commute_node.get_latest_object_coordinates(
            label=label
        )

        if obj_position_data is None or len(obj_position_data) < 3:
            print(f"無法獲取 '{label}' 的座標")
            return ArmGoal.Result(success=False, message="No object detected")

        current_depth = obj_position_data[0]  # 目前物體的深度
        obj_y = obj_position_data[1]  # 物體的左右位置
        obj_z = obj_position_data[2]  # 物體的上下位置

        # 計算需要的 x_offset 來達到目標深度
        depth_diff = current_depth - target_depth
        # 定義閾值，當偏差小於閾值時認為已經達到目標
        depth_threshold = 0.1
        lateral_threshold = 0.1
        print("depth_threshold, obj_y, obj_z", [depth_threshold, obj_y, obj_z])
        # 檢查是否所有方向都已接近目標
        if (
            abs(depth_diff) <= depth_threshold
            and abs(obj_y) <= lateral_threshold
            and abs(obj_z) <= lateral_threshold
        ):
            print("已達到目標位置，無需移動")
            return ArmGoal.Result(success=True, message="Already at target position")
        # else:
        #     # 打印未達成的條件和差距
        #     print("未達到目標位置，需要移動:")
        #     if abs(depth_diff) > depth_threshold:
        #         print(
        #             f"  - 深度方向: 差距 {abs(depth_diff):.3f}m (閾值: {depth_threshold}m)"
        #         )
        #     if abs(obj_y) > lateral_threshold:
        #         print(
        #             f"  - 左右方向: 差距 {abs(obj_y):.3f}m (閾值: {lateral_threshold}m)"
        #         )
        #     if abs(obj_z) > lateral_threshold:
        #         print(
        #             f"  - 上下方向: 差距 {abs(obj_z):.3f}m (閾值: {lateral_threshold}m)"
        #         )
        # 調整因子來平滑移動，避免過度調整 (可以根據需要調整)
        x_adjust_factor = 0.3  # 深度調整因子
        y_adjust_factor = 0.3  # y方向調整因子
        z_adjust_factor = 0.3  # z方向調整因子

        # 計算三個方向的偏移
        x_offset = depth_diff * x_adjust_factor
        y_offset = obj_y * y_adjust_factor  # 物體在左側為正，在右側為負
        z_offset = obj_z * z_adjust_factor  # 物體在上方為正，在下方為負

        # print(f"計算偏移: x={x_offset:.3f}, y={y_offset:.3f}, z={z_offset:.3f}")

        # 使用 offset_from_end_effector 計算目標位置，並可視化
        target_position = self.pybullet_robot_controller.offset_from_end_effector(
            x_offset=x_offset,
            y_offset=y_offset,
            z_offset=z_offset,
            visualize=True,
            mark_color=[0, 1, 0],  # 綠色標記
        )

        # # 生成移動軌跡，使用較小的步數使運動更平滑
        # robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
        #     target_position=target_position, steps=5  # 使用較少的步數，移動更快速
        # )

        # # 執行移動
        # if robot_angle:
        #     for i in robot_angle:
        #         self.move_real_and_virtual(radian=i)
        #         time.sleep(0.05)  # 縮短等待時間，使運動更流暢

        #     return ArmGoal.Result(
        #         success=True, message=f"Successfully followed {label}"
        #     )
        # else:
        #     return ArmGoal.Result(
        #         success=False, message="Failed to generate trajectory"
        #     )

    def ik_move_func(self):
        # use ik move to obj position, but not excute
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
                self.move_real_and_virtual(radian=i)
                time.sleep(0.2)
        else:
            print("not close to the object")

    def move_real_and_virtual(self, radian):
        # for synchronous move real and virtual robot
        self.pybullet_robot_controller.setJointPosition(position=radian)
        degree = self.radians_to_degrees(radian)
        degree[-1] = 90
        self.arm_agnle_control.arm_all_change(degree)
        self.arm_commute_node.publish_arm_angle()

    def move_forward_backward(self, direction="forward", distance=0.1):
        """
        控制手臂向前或向後移動。

        Args:
            direction (str): 移動方向，"forward" 或 "backward"
            distance (float): 移動距離（以米為單位），對於後退方向會自動轉換為負值

        Returns:
            ArmGoal.Result: 包含操作結果的對象
        """
        # 根據方向確定距離值（前進為正，後退為負）
        actual_distance = distance if direction == "forward" else -abs(distance)
        if direction == "forward":
            z_offset = 0.05
        else:
            z_offset = -0.05
        # 標記目標點位置
        obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
            distance=actual_distance, z_offset=z_offset
        )

        # 生成插值軌跡
        robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=obj_pos, steps=5
        )

        # 執行運動
        for i in robot_angle:
            # 如果需要同步真實機械臂，可以用 move_real_and_virtual
            # 否則只移動模擬中的機械臂
            self.move_real_and_virtual(radian=i)
            time.sleep(0.1)

        return ArmGoal.Result(success=True, message=f"Successfully moved {direction}")

    def move_end_effector_direction(self, direction="up"):
        # generate
        pos = self.pybullet_robot_controller.move_ee_relative_example(
            direction=direction,
            distance=0.05,
        )
        robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=pos, steps=5
        )
        for i in robot_angle:
            self.move_real_and_virtual(radian=i)
            time.sleep(0.1)
        return ArmGoal.Result(success=True, message="success")
