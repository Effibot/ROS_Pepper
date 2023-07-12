# Launch file for Pepper navigation
# We want to launch the navigation stack along with the ekf_localization node
# and the laser_1to2_repub node in order to get the laser scan data from the
# Pepper robot in order to make a map of the environment and localize the robot

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
        pkg_dir = get_package_share_directory('pepper_nav')
        launch_dir = os.path.join(pkg_dir, 'launch')
        repub_dir = os.path.join(get_package_share_directory('laser_1to2_repub'), 'launch')