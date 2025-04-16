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
        print("#Joints:", self.num_joints)
        if self.controllable_joints is None:
            self.controllable_joints = list(range(1, self.num_joints - 1))
        print("#Controllable Joints:", self.controllable_joints)
        if self.end_eff_index is None:
            self.end_eff_index = self.controllable_joints[-1]
        print("#End-effector:", self.end_eff_index)
        self.num_joints = p.getNumJoints(self.robot_id)
        print(f"總關節數量: {self.num_joints}")
        self.controllable_joints = list(range(1, self.num_joints - 1))
        print(f"可控制的關節索引: {self.controllable_joints}")
        print(f"需要提供的初始位置數量: {len(self.controllable_joints)}")

        if view_world:
            while True:
                p.stepSimulation()
                time.sleep(self.time_step)
