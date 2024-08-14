#!/usr/bin/env python3

import copy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node

class NoisyCloud (Node):

    actual_num = 0

    def __init__(self):

        #Initialize node
        super().__init__("noisy_cloud")

        #Subscription to point cloud (buffer of 10 messages)
        self.location_node_ = self.create_subscription(PointCloud2, "/cloud_in", self.add_noise, 10)

    def add_noise (self, msg : PointCloud2):
        print(msg)