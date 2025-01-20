Rover arm:

Keyboard clicking
Move to pose
Waypoint
Required scripts are in scripts folder
Using trac ik(install it seperatly if facing errors), can use kdl as alternative(slower)
Joint publisher (serial communication with esp32),topic to indicate tap key, and end effector open close state


To start:
ros2 launch arm_moveit_config demo.launch.py
