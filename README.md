# mobile_manipulator
This is a ROS2 package for MiR 250 + UR5e with ros2_control, Gazebo and Ignition Gazebo simulation.

# Installation

## Preliminaries
## ROS2
If you haven't already installed [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your PC, you need to add the ROS2 apt repository.

## Source install
```
# create a ros2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/

# clone mir_robot into the ros2 workspace
git clone https://git.rwth-aachen.de/rudresh.lonkar/mir_250_ros2/ src/mir_robot

# use vcs to fetch linked repos
# $ sudo apt install python3-vcstool
vcs import < src/mir_robot/ros2.repos src --recursive

# use rosdep to install all dependencies (including ROS itself)
sudo apt update
sudo apt install -y python3-rosdep
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# build all packages in the workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
```
You must source the workspace in each terminal you want to work in:
```
source ~/ros2_ws/install/setup.bash
```

# Gazebo demo (mapping)
```
### gazebo:
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=maze

### mapping (slam_toolbox)
ros2 launch mir_navigation mapping.py use_sim_time:=true slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml

### navigation (optional)
ros2 launch mir_navigation navigation.py use_sim_time:=true cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

# Gazebo demo (Navigation with existing map)
```
### gazebo
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz


### localization (existing map)
ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml

### navigation
ros2 launch mir_navigation navigation.py use_sim_time:=true
```
# Notes

1. If you get an error with respect to Gazebo Classic: Cannot launch gzclient on a launch file - results in shared_ptr assertion error, All you have to do is, source the gazebo classic. 

    `. /usr/share/gazebo/setup.sh `