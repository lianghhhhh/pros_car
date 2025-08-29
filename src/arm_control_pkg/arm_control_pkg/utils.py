import os
import math
import yaml
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory

def load_arm_parameters(package_name):
    """Load arm parameters from YAML file as a dictionary"""
    try:
        # Get the path to the package directory
        package_path = os.path.join(
            get_package_share_directory(package_name), "config"
        )
        # Construct the full path to the YAML file
        yaml_file_path = os.path.join(package_path, "arm_config.yaml")

        # Load the YAML file
        with open(yaml_file_path, "r") as file:
            arm_params = yaml.safe_load(file)
        
        return arm_params

    except Exception as e:
        raise RuntimeError(f"Failed to load arm parameters: {e}")

def get_yaw_from_quaternion(quat_or_tuple):
    if hasattr(quat_or_tuple, 'x'):
        x, y, z, w = quat_or_tuple.x, quat_or_tuple.y, quat_or_tuple.z, quat_or_tuple.w
    else:
        x, y, z, w = quat_or_tuple
    r = R.from_quat([x, y, z, w])
    euler = r.as_euler('xyz', degrees=False)
    return euler[2]  # Yaw


def normalize_angle(angle):
    """將角度正規化到 -pi 到 pi"""
    return math.atan2(math.sin(angle), math.cos(angle))