#!/usr/bin/env python3

import rclpy
import rosbag2_py
import os
import sys

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# Obtaining the path for the recorgins folder
recordings_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../recordings"))

# Adding the recordings path to python execution
sys.path.append(recordings_path)

# Setting recording path
bag_path = "rosbag.db3"

class InputData (Node):
    
    def __init__(self):

        # Initialize node
        super().__init__("input_data_node")
        
        # Create subscription to /cloud_in, triggers pose_callback
        self.location_node_subscriber = self.create_subscription(PointCloud2, "new_pose", self.input_data_callback, 10)
        
        # Create a publisher that will provide the actual pose
        self.input_data_publisher= self.create_publisher(PointCloud2, 'clean_pcl', 10)
        
        # Initialize the data that will be published
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions()
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        
    def input_data_callback():
        print("ok")
