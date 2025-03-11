#!/usr/bin/env python3

import rclpy
import rclpy.serialization
import rosbag2_py
import os
import sys
import struct

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# Obtaining the path for the recorgins folder
recordings_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../src/recordings"))

# Adding the recordings path to python execution
sys.path.append(recordings_path)

# Setting recording path
bag_path = os.path.abspath(os.path.join(recordings_path , "rosbag.db3"))

pcl_topic = "/cloud_in"
pose_topic = "/gt_odom"

class InputData (Node):
    
    def __init__(self):

        # Initialize node
        super().__init__("input_data_node")
        
        # Create subscription to /new_pose, triggers input_data_callback
        self.location_node_subscriber = self.create_subscription(PoseStamped, "new_pose", self.input_data_callback, 10)
        # Create a publisher that will provide the clean pcl
        self.input_data_publisher_pcl = self.create_publisher(PointCloud2, 'clean_pcl', 10)
        # Create a publisher that will provide the clean pose
        self.input_data_publisher_pose = self.create_publisher(PoseWithCovarianceStamped, 'clean_pose', 10)
        
        # Initialize the data that will be published
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format = "cdr", output_serialization_format = "cdr")
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)
        
        # Publish the firsts poses and pcl
        self.input_data_callback(None)


    def input_data_callback (self, msg):
        
        # We prepare to publish the first pcd
        if self.reader.has_next():
            
            # Read the next message in the queue
            topic, msg, t = self.reader.read_next()
            
            # Filter topics
            if topic == pcl_topic:
                
                # Try to deserialize the topic
                try:
                    # Converts the message from bytes to PointCloud2
                    deserialized_msg = rclpy.serialization.deserialize_message(msg, PointCloud2)
                    
                    self.input_data_publisher_pcl.publish(deserialized_msg)
                
                except Exception as e:
                    self.get_logger().error(f"Failed to deserialize or publish message: {e}")
                    
            elif topic == pose_topic:
                
                # Try to deserialize the topic
                try:
                    # Converts the message from bytes to PoseWithCovarianceStamped
                    deserialized_msg = rclpy.serialization.deserialize_message(msg, PoseWithCovarianceStamped)
 
                    self.input_data_publisher_pose.publish(deserialized_msg)
                    
                    self.input_data_callback(None)
                
                except Exception as e:
                    self.get_logger().error(f"Failed to deserialize or publish message: {e}")
                
            else: 
                self.input_data_callback(None)
                    
                
        
        
        
        
