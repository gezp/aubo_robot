'''Launch ur10 ignition_simulator with ros joint trajectory controller and state publisher'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    pkg_aubo_robot_driver = get_package_share_directory('aubo_robot_driver')
    robot_config_path = os.path.join(pkg_aubo_robot_driver, 'config', 'aubo_i5.yaml')
    #  joint controller for aubo
    joint_controller = Node(
        package='aubo_robot_driver', 
        executable='joint_controller',
        name="aubo_i5_joint_controller",
        parameters=[robot_config_path],
        output='screen') 
    ld.add_action(joint_controller)          
    return ld
