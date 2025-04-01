def validate_joint_limits(self, positions, arm_params):
    """Validate joint positions against limits from YAML config

    Args:
        positions: List of joint positions to validate (uses self.joint_positions if None)

    Returns:
        list: List of validated joint positions with values clamped to limits
    """

    # Get joints count from config
    joints_count = arm_params["global"]["joints_count"]

    # Ensure the positions list has enough elements
    if len(positions) < joints_count:
        positions.extend([90.0] * (joints_count - len(positions)))

    # Process each joint
    for i in range(len(positions)):
        joint_key = str(i)

        if "joints" in arm_params and joint_key in arm_params["joints"]:
            min_angle = arm_params["joints"][joint_key].get("min_angle", 0)
            max_angle = arm_params["joints"][joint_key].get("max_angle", 180)
        else:
            # Default limits if not specified
            min_angle = 0
            max_angle = 180

        # Clamp the value to limits
        original = positions[i]
        positions[i] = max(min_angle, min(positions[i], max_angle))

        # Log if we had to clamp the value
        if positions[i] != original:
            self.get_logger().info(
                f"Joint {i} angle {original} clamped to {positions[i]} (limits: {min_angle}-{max_angle})"
            )

    return positions
