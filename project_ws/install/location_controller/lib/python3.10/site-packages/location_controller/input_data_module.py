#!/usr/bin/env python3

import rclpy
import rclpy.serialization
import logging
import rosbag2_py
import os
import sys
import tf2_geometry_msgs

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped


# Obtaining the path for the recorgins folder
recordings_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../src/recordings"))

# Adding the recordings path to python execution
sys.path.append(recordings_path)

logger = logging.getLogger("InputData")

class InputData (Node):
    
    # Topic names
    PCL_TOPIC = "/cloud_in"
    POSE_TOPIC = "/gt_odom"
    TF_TOPIC = "/tf"
    FRAME_ID = "map"
    
    # Setting recording path
    BAG_PATH = os.path.abspath(os.path.join(recordings_path , "rosbag.db3"))
    
    def __init__(self):
        
        """
        Initializes the InputDataNode class.
        This constructor sets up the ROS2 node, initializes publishers and subscribers, 
        and prepares the rosbag reader for data processing. It also publishes the first 
        pose and point cloud data upon initialization.
        Attributes:
            location_node_subscriber (Subscription): Subscriber to the "new_pose" topic, 
                which triggers the `input_data_callback` method.
            input_data_publisher_pcl (Publisher): Publisher for the "clean_pcl" topic, 
                which provides the cleaned point cloud data.
            input_data_publisher_pose (Publisher): Publisher for the "clean_pose" topic, 
                which provides the cleaned pose data.
            reader (rosbag2_py.SequentialReader): Reader for accessing data from the 
                specified rosbag file.
            temp_pose (PoseStamped or None): Temporary storage for the current pose.
            index (int): Counter to track the current index of processed data.
        """

        # Initialize node
        super().__init__("input_data_node")
        
        # Create subscription to /new_pose, triggers input_data_callback
        self.location_node_subscriber = self.create_subscription(PoseStamped, "new_pose", self.input_data_callback, 10)
        
        # Create a publisher that will provide the clean pcl
        self.input_data_publisher_pcl = self.create_publisher(PointCloud2, 'clean_pcl', 10)
        
        # Create a publisher that will provide the clean pose
        self.input_data_publisher_pose = self.create_publisher(PoseWithCovarianceStamped, 'clean_pose', 10)
        
        # Initialize the data that will be published
        storage_options = rosbag2_py.StorageOptions(uri=self.BAG_PATH, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format = "cdr", output_serialization_format = "cdr")
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)
        
        # Temp pose
        self.temp_pose = None
        self.temp_tf = None
        self.index = 0
        
        # Publish the first pose and pcl
        self.input_data_callback(None)

    #Function to call back when the poses are published
    def input_data_callback (self, msg):
        
        """
        Callback function to process input data messages from a recorded bag file.
        This function reads messages from a bag file using a reader object and processes
        them based on their topic. It handles two types of topics: PointCloud2 and 
        PoseWithCovarianceStamped. The processed messages are published to corresponding 
        ROS 2 topics.
        Args:
            msg: The incoming message (not used in this implementation, as the function 
                 reads messages directly from the bag file).
        Behavior:
            - If the topic is a PointCloud2 message:
                - Deserializes the message.
                - Updates the frame ID to "map".
                - Publishes the deserialized PointCloud2 message.
                - Publishes the latest pose associated with the point cloud.
                - Increments the message index.
            - If the topic is a PoseWithCovarianceStamped message:
                - Deserializes the message.
                - Updates the frame ID to "map".
                - Updates the timestamp with the current message index.
                - Stores the pose temporarily for future use.
                - Recursively continues reading the file until a PointCloud2 message is found.
            - If the topic is neither of the above:
                - Recursively continues reading the file until a PointCloud2 message is found.
        Exceptions:
            - Logs an error if deserialization or publishing fails for any message.
        Note:
            This function assumes that the reader object and publishers are properly 
            initialized and that the FRAME_ID, PCL_TOPIC, and POSE_TOPIC constants are 
            correctly defined.
        """
    
        # We prepare to publish the first pcd
        if self.reader.has_next():
            
            # Read the next message in the queue
            topic, msg, _ = self.reader.read_next()
            
            # Filter topics
            if topic == self.PCL_TOPIC:
                
                # Try to deserialize the topic
                try:
                    # Converts the message from bytes to PointCloud2
                    deserialized_msg = rclpy.serialization.deserialize_message(msg, PointCloud2)
                    
                    # We change the frame id to make sure its "map"
                    deserialized_msg.header.frame_id = self.FRAME_ID
                    
                    # We add an index to track pcls
                    deserialized_msg.header.stamp.sec = self.index
                    
                    # Transform the pcl to position it correctly on the map
                    # transformed_pcl = self.transform_pcl(self.temp_tf, deserialized_msg)
                    
                    # Publish the point cloud
                    self.input_data_publisher_pcl.publish(deserialized_msg)
                    
                    # Publish the latest pose associated to the pcl
                    self.input_data_publisher_pose.publish(self.temp_pose)                 
                    
                    # Increase the msg count
                    self.index += 1
                
                except Exception as e:
                    logger.error(f"Failed to deserialize or publish message: {e}")
                    
            elif topic == self.POSE_TOPIC:
                
                # Try to deserialize the topic
                try:
                    # Converts the message from bytes to PoseWithCovarianceStamped
                    deserialized_msg = rclpy.serialization.deserialize_message(msg, PoseWithCovarianceStamped)
                    
                    # We add an index to track poses
                    deserialized_msg.header.stamp.sec = self.index
                    
                    # We change the frame id to make sure its "map"
                    deserialized_msg.header.frame_id = self.FRAME_ID
 
                    self.temp_pose = deserialized_msg
                    # self.input_data_publisher_pose.publish(deserialized_msg)
                    
                    # Continue reading the file until we find the next PCL
                    self.input_data_callback(None)
                
                except Exception as e:
                    logger.error(f"Failed to deserialize or publish message: {e}")
                    
            elif topic == self.TF_TOPIC:
                
                # Try to deserialize the topic
                try:
                    # Converts the message from bytes to PoseWithCovarianceStamped
                    deserialized_msg = rclpy.serialization.deserialize_message(msg, TransformStamped)
 
                    self.temp_tf = deserialized_msg
                    # self.input_data_publisher_pose.publish(deserialized_msg)
                    
                    # Continue reading the file until we find the next PCL
                    self.input_data_callback(None)
                
                except Exception as e:
                    logger.error(f"Failed to deserialize or publish message: {e}")
                
            else: 
                # Continue reading the file until we find the next PCL
                self.input_data_callback(None)
                    
                    
    def transform_pcl (self, transform, pcl):
        
        """
        Transforms a PointCloud2 message to a new frame using a given transformation.
        This function takes a PointCloud2 message and applies a transformation to each point
        in the cloud, converting it to a new coordinate frame. The transformed points are 
        returned as a new PointCloud2 message.
        Args:
            transform (geometry_msgs.msg.TransformStamped): The transformation to apply, 
                which specifies the target frame and the transformation parameters.
            pcl (sensor_msgs.msg.PointCloud2): The input point cloud to be transformed.
        Returns:
            sensor_msgs.msg.PointCloud2: A new PointCloud2 message containing the transformed 
            points in the target frame.
        Raises:
            Exception: If an error occurs during the transformation process, such as invalid 
            input data or transformation failure.
        Notes:
            - The input point cloud is expected to have fields "x", "y", and "z".
            - The transformed point cloud will have its frame_id set to "map".
        """
        
        try:

            # Convert point cloud to list of (x, y, z)
            points = np.array(list(pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True)))

            transformed_points = []
            
            for p in points:
                
                point = tf2_geometry_msgs.PointStamped()
                point.header.frame_id = pcl.header.frame_id
                x, y ,z = p
                point.point.x, point.point.y, point.point.z = (float(x), float(y),float(z))
                point_transformed = tf2_geometry_msgs.do_transform_point(point, transform)
                transformed_points.append((point_transformed.point.x, point_transformed.point.y, point_transformed.point.z))

            # Create new PointCloud2 message
            transformed_cloud = pc2.create_cloud_xyz32(pcl.header, transformed_points)
            transformed_cloud.header.frame_id = "map"
            
            return transformed_cloud

        except Exception as e:
            logger.error(f"TF Transform Error: {e}")
                
        
        
        
        
