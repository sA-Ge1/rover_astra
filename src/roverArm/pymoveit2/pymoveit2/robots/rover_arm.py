from typing import List


MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

# Optional prefix for joint/link naming
prefix: str = ""

def joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "shoulder_joint",
        prefix + "upper_arm_joint",
        prefix + "elbow_joint",
        prefix + "wrist_joint",
        prefix + "ee_rotation_joint_z"
    ]

# Base link name
def base_link_name(prefix: str = prefix) -> str:
    return prefix + "base_link"

# End effector name
def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "ee_rotation_z"

# Gripper joint names
def gripper_joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "gripper_joint_1", 
        prefix + "gripper_joint_2"
    ]
