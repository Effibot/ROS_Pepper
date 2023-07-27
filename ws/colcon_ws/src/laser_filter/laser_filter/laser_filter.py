import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Header as header


class LaserFilter(Node):
    def __init__(self, msg_window_size=1):
        super().__init__("laser_filter")
        # subscribe to the laser scan topic
        self.sub = self.create_subscription(LaserScan, "/laser", self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, "scan", 10)
        # create the msg window where the laser scan msgs will be stored
        self.msg_matrix_queue = np.empty((0, 61), dtype=np.float32)
        self.msg = np.zeros((61), dtype=np.float32)
        # create the empty space array that will be used to fill the empty space in the laser scan msgs
        # self.empty_space = -1 * np.ones((8), dtype=np.float32)
        # maximum number of laser scan msgs to store in the msg window before filtering
        self.max_window_size = msg_window_size
        self.curr_size = 0
        # create masks to extract a portion of the laser scan msg:
        # right: 0-15, front: 23-38, left: 46-61
        # self.right_mask = np.concatenate((np.ones(15,dtype=np.int8), np.zeros(46, dtype=np.int8)))
        # self.front_mask = np.concatenate((np.zeros(23,dtype=np.int8), np.ones(15, dtype=np.int8), np.zeros(23, dtype=np.int8)))
        # self.left_mask = np.concatenate((np.zeros(46,dtype=np.int8), np.ones(15, dtype=np.int8)))
        # define some constants for the laser scan msg - from naoqi-driver2 code
        self.msg_constants = {
            "angle_min": -2.094399929046631,
            "angle_max": 2.094399929046631,
            "angle_increment": 0.06866884976625443,
            "time_increment": 0.0,
            "range_min": 0.10000000149011612,
            "range_max": 3.0,
        }
        self.msg_header = header()
        self.msg_header.frame_id = "base_footprint"

    def scan_callback(self, msg):
        # listen to the laser scan msgs and store them in the msg window
        if len(self.msg_matrix_queue) < self.max_window_size:
            # there is still space in the current window
            self.append_msg(msg)
        elif len(self.msg_matrix_queue) == self.max_window_size:
            # the current window is full, so we need to filter the msgs
            self.filter_msgs()
        else:
            # this should never happen
            rclpy.logerr("Error: msg_matrix_queue has more than max_window_size elements.")

    def append_msg(self, msg):
        # append the msg to the msg window
        self.msg_matrix_queue = np.vstack((np.array(msg.ranges), self.msg_matrix_queue))

    def filter_msgs(self):
        """
        Filter the msgs in the msg window using a median filter, then publish
        a new laser scan msg, which is the median of the msgs in the window.

        At the same time, remove the oldest msg in the window.
        """

        # compute the median of each column in the msg window
        medians = np.median(self.msg_matrix_queue, axis=0)
        # create a new laser scan msg
        filtered_msg = LaserScan()
        filtered_msg.ranges = medians.tolist()
        filtered_msg.angle_min = self.msg_constants["angle_min"]
        filtered_msg.angle_max = self.msg_constants["angle_max"]
        filtered_msg.angle_increment = self.msg_constants["angle_increment"]
        filtered_msg.time_increment = self.msg_constants["time_increment"]
        filtered_msg.range_min = self.msg_constants["range_min"]
        filtered_msg.range_max = self.msg_constants["range_max"]
        self.msg_header.stamp = self.get_clock().now().to_msg()
        filtered_msg.header = self.msg_header
        # publish the new laser scan msg
        self.pub.publish(filtered_msg)


def main():
    print("Hi from laser_filter.")
    rclpy.init()
    node = LaserFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
