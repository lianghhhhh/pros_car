# Control arm depending on self.move_real_and_virtual
import math
import time
from action_interface.action import ArmGoal
from arm_control_pkg.utils import get_yaw_from_quaternion, normalize_angle

class ArmAutoController:
    def __init__(
        self, arm_commute_node, pybulletRobotController, arm_angle_control
    ):
        self.pybullet_robot_controller = pybulletRobotController
        self.arm_commute_node = arm_commute_node
        self.arm_angle_control = arm_angle_control
        self.depth = 100.0

    def catch(self):
        label = "ball"
        while self.depth > 0.4:
            print(self.depth)
            try:
                self.depth = self.arm_commute_node.get_latest_object_coordinates(label=label)[0]
            except:
                continue
        while 1: 
            print("follow_obj")
            if self.follow_obj(label=label)  == True:
                break

        # reset depth
        self.depth = 100.0
        data = self.arm_commute_node.get_latest_object_coordinates(label=label)
        depth = data[0]
        obj_pos = self.pybullet_robot_controller.markPointInFrontOfEndEffector(
            distance=depth + 0.15,z_offset=0.15
        )
        robot_angle = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=obj_pos,steps=10
        )
        for i in robot_angle:
            self.move_real_and_virtual(radian=i)
            time.sleep(0.1)
        self.grap()
        time.sleep(1.0)
        self.init_pose(grap=True)
        time.sleep(1.0)
        self.rotate_car()
        self.rotate_wrist()
        time.sleep(0.2)
        for i in range(10):
            self.arm_commute_node.publish_pos()
            time.sleep(0.1)
        return ArmGoal.Result(success=True, message="success")

    def rotate_wrist(self):
        self.arm_angle_control.arm_index_change(3, 90)
        self.arm_commute_node.publish_arm_angle()

    def rotate_car(self):
        # 取得當前車體朝向
        _, rotation = self.arm_commute_node.get_car_position_and_orientation()
        current_yaw = get_yaw_from_quaternion(rotation)

        # 設定目標朝向（反向 180 度）
        target_yaw = normalize_angle(current_yaw + math.pi)

        # 開始旋轉
        self.arm_commute_node.publish_control(vel=[5.0, -5.0, 5.0, -5.0])

        while True:
            _, rotation = self.arm_commute_node.get_car_position_and_orientation()
            yaw = get_yaw_from_quaternion(rotation)
            yaw_error = normalize_angle(target_yaw - yaw)

            print(f"Current Yaw: {math.degrees(yaw):.2f}, Target: {math.degrees(target_yaw):.2f}, Error: {math.degrees(yaw_error):.2f}")

            if abs(yaw_error) < math.radians(5):  # 誤差小於 5 度即停止
                break
            time.sleep(0.1)

        for i in range(5):
            # 停止轉動
            self.arm_commute_node.publish_control(vel=[0.0, 0.0, 0.0, 0.0])
            time.sleep(0.1)

    def car2_position(self):
        # 給 car2 的座標
        pass

    def arm_wave(self):
        pass

    def arm_ik_move(self):
        t = self.pybullet_robot_controller.offset_from_end_effector(
            y_offset=0.1, z_offset=0.1
        )
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

    def grap(self):
        self.arm_angle_control.arm_index_change(4, 0)
        self.arm_commute_node.publish_arm_angle()

    def init_pose(self, grap=False):
        self.arm_angle_control.arm_default_change()
        angle = self.arm_angle_control.get_arm_angles()
        if grap:
            self.arm_angle_control.arm_index_change(4, 30)
            self.arm_commute_node.publish_arm_angle()
            time.sleep(1.0)
        self.arm_commute_node.publish_arm_angle()
        joints_reset_degrees = angle
        joints_reset_radians = [math.radians(angle) for angle in joints_reset_degrees]
        self.pybullet_robot_controller.setJointPosition(position=joints_reset_radians)
        return ArmGoal.Result(success=True, message="success")

    def test(self):
        # self.rotate_car()
        self.arm_commute_node.publish_pos()
        return ArmGoal.Result(success=True, message="success")

    def look_up(self):
        self.arm_angle_control.arm_index_change(2, 140)
        self.arm_commute_node.publish_arm_angle()

    def _is_at_target(
        self,
        depth: float,
        y: float,
        z: float,
        target_depth: float,
        depth_thresh: float,
        lateral_thresh: float,
    ) -> bool:
        """
        判斷當前 (depth, y, z) 是否已經進入允收範圍。
        """
        return (
            # abs(depth - target_depth) <= depth_thresh
            abs(y) <= 0.02
            and abs(z) <= 0.1
        )

    def follow_obj(self, label="ball", target_depth=0.3):
        # 參數設定
        depth_threshold = 0.05
        lateral_threshold = 0.05
        x_adjust_factor = 0.3
        y_adjust_factor = 0.3
        z_adjust_factor = 0.3

        # 1. 讀座標
        data = self.arm_commute_node.get_latest_object_coordinates(label=label)
        if not data or len(data) < 3:
            return ArmGoal.Result(success=False, message="No object detected")
        current_depth, obj_y, obj_z = data

        # 2. 初次檢查
        if self._is_at_target(
            current_depth,
            obj_y,
            obj_z,
            target_depth,
            depth_threshold,
            lateral_threshold,
        ):
            print("檢查到已經在目標位置")
            return True

        # 3. 計算偏移
        depth_diff = current_depth - target_depth
        x_offset = depth_diff * x_adjust_factor
        y_offset = obj_y * y_adjust_factor
        z_offset = obj_z * z_adjust_factor

        # 4. 設定絕對深度（可選）
        target_pos = self.pybullet_robot_controller.offset_from_end_effector(
            x_offset=x_offset,
            y_offset=y_offset,
            z_offset=z_offset,
            visualize=True,
            mark_color=[0, 1, 0],
        )
        target_pos[0] = 0.2  # 若要固定深度

        # 5. 生成與執行軌跡
        traj = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            target_position=target_pos, steps=10
        )
        if not traj:
            return True

        for angle in traj:
            self.move_real_and_virtual(radian=angle)
            time.sleep(0.05)

            # 6. 每步驟都再檢查一次
            new_data = self.arm_commute_node.get_latest_object_coordinates(label=label)
            if new_data and len(new_data) >= 3:
                nd, ny, nz = new_data
                if self._is_at_target(
                    nd, ny, nz, target_depth, depth_threshold, lateral_threshold
                ):
                    print("中途已達到目標位置，提早停止")
                    return True
                    break

        # return True

    def ik_move_func(self):
        # use ik move to obj position, but not excute
        # This must use imu data
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
        self.arm_angle_control.arm_all_change(degree)
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
