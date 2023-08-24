import rclpy, os, yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # get the path to the share directory to load parameters
    pkg_dir = get_package_share_directory("urg_slam")
    slam_params = [os.path.join(pkg_dir, "config", "slam.yaml")]
    # odom_params= [os.path.join(pkg_dir, "config", "odom.yaml")]
    urg_params = os.path.join(pkg_dir, "config", "lidar.yaml")

    # do some remappings
    remappings = [
        ("scan", "/scan"),
        ("odom", "/odom"),
        ("imu", "/imu"),
    ]

    rclpy.logging.get_logger("launch").info("Launching URG-04LX-UG01")
    # open urg params
    with open(urg_params, "r") as file:
        config_params = yaml.safe_load(file)["urg_node2"]["ros__parameters"]

    # urg_node2をライフサイクルノードとして起動
    lifecycle_node = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name=LaunchConfiguration("node_name"),
        remappings=[("scan", LaunchConfiguration("scan_topic_name"))],
        parameters=[config_params],
        namespace="",
        output="screen",
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    rclpy.logging.get_logger("launch").info("Launching rtabmap...")
    # define rtabmap params and node
    rtabnode = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        parameters=slam_params,
        output="screen",
        remappings=remappings,
        arguments=["-d", "--delete_db_on_start"],
    )

    odom_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="rtabmap_odom",
        output="screen",
    )

    urg_slam_node = Node(
        package="urg_slam",
        executable="urg_slam",
        name="urg_slam",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "qos",
                default_value="2",
                description="QoS used for input sensor topics",
            ),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("node_name", default_value="urg_node2"),
            DeclareLaunchArgument("scan_topic_name", default_value="scan"),
            #urg_slam_node,
            lifecycle_node,
            urg_node2_node_configure_event_handler,
            urg_node2_node_activate_event_handler,
            odom_node,
            rtabnode,
        ]
    )
