"""_summary_: launch  navigation stack for the robot."""
import os
from symbol import parameters

from yaml import Node
import rclpy
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """_summary_: Function to launch the navigation stack for the robot.
    Returns:
        _type_: None
    """
    nav2_params_file = os.path.join(get_package_share_directory("pepper_nav"), "params", "nav2_params.yaml")
    # log a message to the console to indicate launch is starting with rclpy
    rclpy.logging.get_logger('launch').info('Launching navigation stack...')
    nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                ])
            ),
        launch_arguments={
                "map_topic": "/map",
                "params_file": nav2_params_file
                }.items()
        )
    return LaunchDescription([
        nav
    ])