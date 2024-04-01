import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e",
            " ",
            "prefix:=",
            'ur_',
            " ",
        ]
    )

    #robot_description = {"robot_description": robot_description_content}
    mir_description_dir = get_package_share_directory('mir_description')

    robot_description_content2 = ParameterValue(
        Command(
        [
            "xacro ", 
            os.path.join(mir_description_dir, "urdf", "mir.urdf.xacro")  # mob_man.urdf
        ]
        ),
        value_type=str
    )

    #robot_description = {"robot_description": robot_description_content}
    robot_description = {"robot_description": robot_description_content2}


    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            'ur_',
            " ",
        ]
    )
    """ robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    } """

    moveit_config_package = get_package_share_directory('ur_moveit_config')
    robot_description_semantic_content2 = ParameterValue(
        Command(
        [
            "xacro ", 
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "srdf", "mir_100.srdf"] # ur.srdf
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            "ur_",
            " ",
        ]
        ),
        value_type=str
    )

    #robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content2}
    return robot_description_semantic
    
def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    demo_node = Node(
        package="hello_moveit_ur",
        executable="hello_moveit_ur",
        name="hello_moveit_ur",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return launch.LaunchDescription([demo_node])
