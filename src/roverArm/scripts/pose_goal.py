import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import rover_arm as robot
from scipy.spatial.transform import Rotation as R


def go_to_pose(moveit2, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """
    Move the robot to a specified pose.
    
    Args:
        moveit2: MoveIt2 interface instance
        x, y, z: Target position coordinates in meters
        roll, pitch, yaw: Target orientation in radians
    
    Returns:
        bool: True if movement was successful, False otherwise
    """
    try:
        # Convert Euler angles to quaternion
        r = R.from_euler('xyz', [roll, pitch, yaw])
        quat_xyzw = r.as_quat()

        # Move to pose
        result = moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=quat_xyzw,
            cartesian=False,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0
        )

        if result:
            moveit2.wait_until_executed()
            return True
        return False
        
    except Exception as e:
        print(f"Error in go_to_pose: {e}")
        return False


def create_moveit2_interface(node):
    """
    Create and configure a MoveIt2 interface.
    
    Args:
        node: ROS2 node instance
    
    Returns:
        MoveIt2: Configured MoveIt2 interface
    """
    try:
        moveit2 = MoveIt2(
            node=node,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
        )

        # Set movement constraints
        moveit2.max_velocity = 1.0
        moveit2.max_acceleration = 1.0
        
        return moveit2
        
    except Exception as e:
        print(f"Error creating MoveIt2 interface: {e}")
        raise
