"""Launch MoveIt2 move_group action server and the required bridges between Ignition and ROS 2"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    pkg_aubo_robot_ign = get_package_share_directory('aubo_robot_ign')
    # Launch Arguments
    urdf = os.path.join(pkg_aubo_robot_ign,"resource","urdf","aubo_i5.urdf")
    rviz2_config = os.path.join(pkg_aubo_robot_ign,"launch", "test.rviz")

    # Robot state publisher
    robot_state_publisher = Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             name="robot_state_publisher",
             output="screen",
             arguments=[urdf])
    ld.add_action(robot_state_publisher)
    #static Robot state publisher
    static_transform_publisher=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "1.4",
                        "0.0", "0.0", "0.0",
                        "world", "base_link"])
    ld.add_action(static_transform_publisher)
    # MoveIt2 move_group action server
    move_group=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_aubo_robot_ign,"launch", "aubo_i5_move_group_server.launch.py")
            ),
            # Simulation time does not function properly (as of Nov 2020), see https://github.com/AndrejOrsula/ign_moveit2/issues/4
        )
    ld.add_action(move_group)
    # RViz2
    rviz2=Node(package="rviz2",
             executable="rviz2",
             name="rviz2",
             output="log",
             arguments=["--display-config", rviz2_config])   
    ld.add_action(rviz2)
    return ld
