"""
Package to republish laser scan data that comes from a ROS1 node to a ROS2 node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class Laser1to2Republisher(Node):
    def __init__(self):
        super().__init__("laser_1to2_repub")
        self.create_subscription(LaserScan, "pepper/laser_2", self.scan_callback, 10)
        # self.create_subscription(LaserScan, "laser", self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, "scan", 10)

    def scan_callback(self, msg):
        self.get_logger().info("I heard: " + str(msg.ranges[0]))
        self.pub.publish(msg)


def main():
    print("Hi from laser_1to2_repub.")
    rclpy.init()
    node = Laser1to2Republisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
