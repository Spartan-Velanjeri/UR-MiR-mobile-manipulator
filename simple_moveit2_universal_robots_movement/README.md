# simple_moveit2_universal_robot_movement

This repository will simply move a ur3e to a target pose. 
The hello_moveit_ur file is explained here: https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html

The difference to the hello_moveit file is, that there is the ur_manipulator instead of the pandas_arm used.


**Requirements** 

I installed ROS2 Humble: https://docs.ros.org/en/humble/index.html

I installed the ROS2 UR Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

**How to**

Start `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.102 launch_rviz:=false initial_joint_controller:=joint_trajectory_controller` to connect to the Universal Robot. 

Start the `external control urcap` on the UR to allow external control. 

Start the `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true` to cofigure moveit. 

Start the hello_moveit_ur file with a launchfile `ros2 launch hello_moveit_ur hello_moveit_ur_launch.py`.

The launchfile provides the required parameters.
