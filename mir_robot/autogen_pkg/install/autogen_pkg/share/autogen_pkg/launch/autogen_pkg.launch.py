# ***** LAUNCH FILE GENERATED BY AUTOMATON*****


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
    
def generate_launch_description():
    
    
    use_sim_time = LaunchConfiguration('use_sim_time',default=True)

    robot_path = os.path.join(get_package_share_directory('autogen_pkg'))
    xacro_file = os.path.join(robot_path, 'urdf', 'mir_100_v1.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    
    
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    launch_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
    )
    
    
    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'autogen_pkg',
                '-file', xacro_file],
    output='screen'
    )
    
    
    rviz_config_file = os.path.join(robot_path,'rviz','default.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d',rviz_config_file]
    )
    
    
    return LaunchDescription([
        # Launch gazebo environment
        launch_gazebo_world,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity,
        rviz_node,
    ])
    
    