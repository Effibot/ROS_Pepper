from math import e
from turtle import rt
import rclpy
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # log a message to the console to indicate launch is starting with rclpy
    rclpy.logging.get_logger("launch").info("Launching rtabmap...")
    # remapping topic names
    remappings = [
        ("odom", "/odom"),
        ("scan", "/laser"),
        ("depth/image", "/camera/depth/image_raw"),
        ("rgb/image", "/camera/front/image_raw"),
        ("rgb/camera_info", "/camera/front/camera_info"),
        ("imu", "/imu"),
    ]

    config = os.path.join(
        get_package_share_directory("pepper_nav"), "params", "rtabmap.yaml"
    )

    # defining rtabmap_ros parameters
    parameters = [config]
    # rgbd node
    # rgbdnode =  Node(
    #        package='rtabmap_sync', executable='rgbd_sync', outp#ut='screen',
    #        parameters=[{'approx_sync':false, 'use_sim_time':true}],
    #        remappings=remappings)
    # defining rtabmap_ros node
    rtabnode = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        parameters=parameters,
        output="screen",
        remappings=remappings,
        arguments=["-d", "--delete_db_on_start"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "qos", default_value="2",
                description="QoS used for input sensor topics"
            ),
            rtabnode,
        ]
    )