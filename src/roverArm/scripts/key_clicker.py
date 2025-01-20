#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import rover_arm as robot
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from key_data import KeyboardCoordinates
import time
def main():
    rclpy.init()
    
    # Create node
    node = Node("key_movement_sequence")
    
    # Initialize keyboard coordinates
    keyboard = KeyboardCoordinates()
    word = "ASTRA"
    
    # Default orientation for key pressing
    key_orientation = [0.0, -1.57, 0.0]  # [roll, pitch, yaw] in radians
    r_key = R.from_euler('xyz', key_orientation)
    key_quat = r_key.as_quat()
    
    # Create callback group
    callback_group = ReentrantCallbackGroup()
    
    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    
    # Create publisher for key press signal
    key_press_pub = node.create_publisher(
        Bool,
        '/tap',
        10
    )
    
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()
    
    # Set movement speed
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0
    
    # Create tap message
    tap_msg = Bool()
    
    try:
        # Process each letter in the word
        for letter in word:
            # Get coordinates for the current letter
            coordinates = keyboard.get_coordinates(letter)
            
            if not isinstance(coordinates, tuple):
                node.get_logger().error(f"Invalid coordinates for key {letter}")
                continue
                
            # Convert coordinates to meters
            x = coordinates[0]/100.0
            y = coordinates[1]/100.0
            z = 0.0  # Adjust this height as needed
            
            # Move to key position
            node.get_logger().info(f"Moving to key {letter} at position [{x}, {y}, {z}]")
            
            # First move to a position slightly above the key
            approach_position = [x, y, z + 0.035]
            moveit2.move_to_pose(
                position=approach_position,
                quat_xyzw=key_quat,
                cartesian=False,
                cartesian_max_step=0.0025,
            )
            moveit2.wait_until_executed()
            
            # Then move down to press the key
       #     moveit2.move_to_pose(
       #         position=[x, y, z],
       #         quat_xyzw=key_quat,
       #         cartesian=True,  # Use cartesian movement for precise key press
       #         cartesian_max_step=0.0025,
       #     )
       #     moveit2.wait_until_executed()
            
            # Signal key press
            tap_msg.data = True
            key_press_pub.publish(tap_msg)
            time.sleep(2.0)
       #    # Move back up slightly
       #    moveit2.move_to_pose(
       #        position=approach_position,
       #        quat_xyzw=key_quat,
       #        cartesian=True,
       #        cartesian_max_step=0.0025,
       #    )
            
            node.get_logger().info(f"Completed movement for key {letter}")
            
        node.get_logger().info("Completed typing sequence")
        
    except Exception as e:
        node.get_logger().error(f"An error occurred: {str(e)}")
    finally:
        rclpy.shutdown()
        executor_thread.join()
        exit(0)

if __name__ == "__main__":
    main()