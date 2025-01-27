#!/usr/bin/env python3

import rclpy
import numpy as np
import open3d as o3d

from location_controller.utils import draw_registration_result, pointcloud2_to_open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node

class NoisyCloud (Node):
    
    # Parameters
    mean = 0.0
    std_dev = 0.05

    def __init__(self, input_topic : str):

        #Initialize node
        super().__init__("noisy_cloud")

        #Subscription to point cloud (buffer of 10 messages)
        self.noisy_cloud_ = self.create_subscription(PointCloud2, input_topic, self.add_noise, 10)

        # Create a publisher that will provide the noisy cloud
        self.noisy_cloud_publisher_ = self.create_publisher(PointCloud2, 'noisy_cloud', 10)

    def add_noise (self, msg : PointCloud2):
        
        # Transform the pcd into a stream of data
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Transform the pcd data into an array
        points = np.array(list(pc_data))
        
        # Apply noise to the points in the y axis
        points = [(x, y + np.random.normal(self.mean, self.std_dev), z) for x, y, z in points]
        
        # Rebuild the pcd from the noisy data array
        noisy_cloud = pc2.create_cloud_xyz32(msg.header, points)
        
        
        noisy_cloud_o3d = pointcloud2_to_open3d(noisy_cloud)
        
        o3d.visualization.draw_geometries([noisy_cloud_o3d],
                                           zoom=0.4459,
                                           front=[0.9288, -0.2951, -0.2242],
                                           lookat=[1.6784, 2.0612, 1.4451],
                                           up=[-0.3402, -0.9189, -0.1996])
        
        
        # Publish the pcd
        self.noisy_cloud_publisher_.publish(noisy_cloud)