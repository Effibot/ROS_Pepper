# Action Server to bridge ROS and NAOqi
# This server receives a ROS message and sends it to NAOqi
# using the SubProcess module, in order to run a series of python scripts
# written in NAOqi's python SDK, which runs in Python 2.7
# The server also receives a response from NAOqi and sends it back to ROS

import subprocess
import sys
import os
import time
import threading

from naoqi_bridge_msgs.msg import *
from naoqi_bridge_msgs.srv import *
from naoqi_bridge_msgs.action import *
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PoseServer(Node):
    def __init__(self):
        super().__init__("pose_action_server")
        self._action_server = ActionServer(
            self,
            "pose_bridge",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Action server ready")
