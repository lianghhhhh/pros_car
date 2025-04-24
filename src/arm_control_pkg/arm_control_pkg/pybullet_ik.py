# ############################################
# A Robot controller for kinematics, dynamics
# and control based on pyBullet framework
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

# Input:
# 1. robot_type: specify urdf file initials eg. if urdf file name is 'ur5.urdf', specify 'ur5'
# 2. controllable_joints: joint indices of controllable joints. If not specified, by default all joints indices except first joint (first joint is fixed joint between robot stand and base)
# 3. end-eff_index: specify the joint indices for end-effector link. If not specified, by default the last controllable_joints is considered as end-effector joint
# 4. time_Step: time step for simulation

import pybullet as p
import pybullet_data
import numpy as np
import time
import random
import os
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import math
from scipy.spatial.transform import Rotation as R


class PybulletRobotController:
    def __init__(
        self,
        arm_params,
    ):
        self.arm_params = arm_params.get_arm_params()
        # self.robot_type = "ur5"
        robot_description_path = get_package_share_directory("robot_description")
        self.urdf_path = os.path.join(robot_description_path, "urdf", "target.urdf")
        self.robot_id = None
        self.num_joints = None
        self.controllable_joints = self.arm_params["pybullet"]["controllable_joints"]
        self.end_eff_index = self.arm_params["pybullet"]["end_eff_index"]
        self.time_step = float(self.arm_params["pybullet"]["time_step"])
        self.previous_ee_position = None
        self.initial_height = self.arm_params["pybullet"]["initial_height"]
        self.createWorld(
            GUI=self.arm_params["pybullet"]["gui"],
            view_world=self.arm_params["pybullet"]["view_world"],
        )

    def generateInterpolatedTrajectory(self, target_position, steps=50):
        current_position = self.solveForwardPositonKinematics(self.getJointStates()[0])[
            0:3
        ]

        # 計算每一步的位移向量
        step_vector = (np.array(target_position) - np.array(current_position)) / steps
        self.markTarget(target_position)

        # 用於存儲每一步的關節角度（以弧度表示）
        joint_angles_in_radians = []

        # 逐步靠近目標
        for i in range(steps):
            # 計算當前目標位置
            intermediate_position = np.array(current_position) + (i + 1) * step_vector
            # 計算 IK 解
            joint_angles = self.solveInversePositionKinematics(intermediate_position)

            # 將當前關節角度應用到機器人
            if joint_angles and len(joint_angles) >= len(self.controllable_joints):
                # 直接存儲弧度值
                joint_angles_in_radians.append(
                    joint_angles[: len(self.controllable_joints)]
                )

                # self.setJointPosition(joint_angles[:len(self.controllable_joints)])
                time.sleep(0.1)  # 加入延遲觀察
            else:
                print("無法找到合適的解。")
                break

        return joint_angles_in_radians

    def solveInversePositionKinematics(self, end_eff_pose):
        """
        計算逆向運動學以獲取關節角度，基於給定的末端執行器姿勢。

        Args:
            end_eff_pose (list): 末端執行器的目標位置和姿勢，
                                格式為 [x, y, z, roll, pitch, yaw] (6 個元素) 或 [x, y, z] (3 個元素)。

        Returns:
            list: 對應的關節角度。
        """
        if len(end_eff_pose) == 6:
            joint_angles = p.calculateInverseKinematics(
                self.robot_id,
                self.end_eff_index,
                targetPosition=end_eff_pose[0:3],
                targetOrientation=p.getQuaternionFromEuler(end_eff_pose[3:6]),
            )
        else:
            joint_angles = p.calculateInverseKinematics(
                self.robot_id, self.end_eff_index, targetPosition=end_eff_pose[0:3]
            )

        # 標記末端執行器的位置路徑
        # self.markEndEffectorPath()
        return joint_angles

    # function for setting joint positions of robot in pybullet
    def setJointPosition(self, position, kp=1.0, kv=1.0):
        # print('Joint position controller')
        zero_vec = [0.0] * len(self.controllable_joints)
        p.setJointMotorControlArray(
            self.robot_id,
            self.controllable_joints,
            p.POSITION_CONTROL,
            targetPositions=position,
            targetVelocities=zero_vec,
            positionGains=[kp] * len(self.controllable_joints),
            velocityGains=[kv] * len(self.controllable_joints),
        )
        for _ in range(100):  # to settle the robot to its position
            p.stepSimulation()

    def solveForwardPositonKinematics(self, joint_pos):
        # get end-effector link state
        eeState = p.getLinkState(self.robot_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
        eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
        return eePose

    # function to initiate pybullet and engine and create world
    def createWorld(self, GUI=True, view_world=False):
        # load pybullet physics engine
        if GUI:
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        GRAVITY = -9.8
        p.setGravity(0, 0, GRAVITY)
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.time_step, numSolverIterations=100, numSubSteps=10
        )
        p.setRealTimeSimulation(True)
        p.loadURDF("plane.urdf")
        rotation = R.from_euler("z", 90, degrees=True).as_quat()

        # loading robot into the environment
        # urdf_file = "urdf/" + self.robot_type + ".urdf"
        self.robot_id = p.loadURDF(
            self.urdf_path,
            useFixedBase=True,
            basePosition=[0, 0, self.initial_height],
            baseOrientation=rotation,
        )

        self.num_joints = p.getNumJoints(self.robot_id)  # Joints
        print("#Joints read from pybullet:", self.num_joints)
        if self.controllable_joints is None:
            self.controllable_joints = list(range(1, self.num_joints - 1))
        print("#Controllable Joints:", self.controllable_joints)
        if self.end_eff_index is None:
            self.end_eff_index = self.controllable_joints[-1]
        print("#End-effector:", self.end_eff_index)
        self.num_joints = p.getNumJoints(self.robot_id)
        # print(f"總關節數量: {self.num_joints}")
        self.controllable_joints = list(range(1, self.num_joints - 1))
        # print(f"可控制的關節索引: {self.controllable_joints}")
        # print(f"需要提供的初始位置數量: {len(self.controllable_joints)}")

        if view_world:
            while True:
                p.stepSimulation()
                time.sleep(self.time_step)
