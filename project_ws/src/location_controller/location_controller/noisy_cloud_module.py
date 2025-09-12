#!/usr/bin/env python3

import rclpy
import yaml
import os
import numpy as np
import open3d as o3d

from location_controller.utils import pointcloud2_to_open3d, preprocess_point_cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node

# Global parameters
YAML_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../src/config/general_config.yaml"))

        
# Configuration parameters 
with open(YAML_PATH, "r") as file:
    config = yaml.safe_load(file)

class NoisyCloud (Node):

    # Parameters
    mean = 0.0
    std_dev = 0.04

    def __init__(self, input_topic : str):
        
        """
        Initializes the NoisyCloudModule node.
        Args:
            input_topic (str): The name of the input ROS topic to subscribe to for receiving PointCloud2 messages.
        Attributes:
            noisy_cloud_ (Subscription): A ROS 2 subscription to the specified input topic for receiving PointCloud2 messages.
            noisy_cloud_publisher_ (Publisher): A ROS 2 publisher to publish the noisy PointCloud2 messages on the 'noisy_cloud' topic.
        """

        #Initialize node
        super().__init__("NoisyData")

        #Subscription to point cloud (buffer of 10 messages)
        self.noisy_cloud_ = self.create_subscription(PointCloud2, input_topic, self.add_noise, 10)

        # Create a publisher that will provide the noisy cloud
        self.noisy_cloud_publisher_ = self.create_publisher(PointCloud2, 'noisy_pcl', 10)


    def add_noise (self, msg : PointCloud2):
        
        """
        Adds Gaussian noise to the y-axis of a PointCloud2 message.
        This function takes a PointCloud2 message, applies Gaussian noise to the 
        y-axis of each point, and publishes the resulting noisy point cloud.
        Args:
            msg (PointCloud2): The input PointCloud2 message containing the original point cloud data.
        Returns:
            None
        Notes:
            - The Gaussian noise is applied using the mean and standard deviation 
              (`self.mean` and `self.std_dev`) defined in the class instance.
            - The noisy point cloud is published using the `self.noisy_cloud_publisher_` publisher.
        """
        
        # Transform the pcd into a stream of data
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Convert the stream into a numpy array
        points = np.array([ [x, y, z] for x, y, z in pc_data ], dtype=np.float32)
        
        # Apply noise to the points in the y axis
        points_noisy = self.add_noise_to_pcd(points) 
        
        # Rebuild the pcd from the noisy data array
        noisy_cloud = pc2.create_cloud_xyz32(msg.header, points_noisy)    
        
        # Publish the pcd
        self.noisy_cloud_publisher_.publish(noisy_cloud)
        
        
    def add_noise_to_pcd (self, points, mean = config["NOISE_MEAN"], std_dev = config["NOISE_STD_DEV"], prob = config["NOISE_PROBABILITY"]):
        """
        Añade ruido gaussiano a una fracción de puntos de un array Nx3.
        
        - points: np.ndarray (N,3)
        - mean: media del ruido
        - std_dev: desviación típica del ruido
        - prob: probabilidad de aplicar ruido a cada punto
        """
        
        N = points.shape[0]

        # Máscara booleana de qué puntos reciben ruido
        mask = np.random.rand(N) < prob

        # Ruido gaussiano en los 3 ejes
        noise = np.random.normal(mean, std_dev, size=(N, 3))

        # Aplicar ruido solo a los puntos seleccionados
        points_noisy = points.copy()
        points_noisy[mask] += noise[mask]

        return points_noisy