#!/usr/bin/env python3

import numpy as np
import struct
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CloudConverter (Node):
     
    def __init__(self):
        
        #Initialize node
        super.__init__("cloud_converter")
        
        #Subscription to the original cloud, and call to the cloud conversion method
        self.cloud_subscriber_ = self.create_subscription(PointCloud2, "/cloud_in", self.cloud_conversion, 10)

        #Publisher for the converter o3d cloud
        self.cloud_publisher_ = self.create_publisher(o3d.geometry.PointCloud(), '/o3d_cloud', 10)

    def cloud_conversion(self, msg : PointCloud2):
        
        #Publish the converted cloud
        self.cloud_publisher_.publish(pointcloud2_to_open3d(msg))


def pointcloud2_to_open3d(self, data_msg):

    pcd = o3d.geometry.PointCloud()
    
    float_values = []
    for i in range(0, len(data_msg.data), 4):
        float_value = struct.unpack('f', data_msg.data[i:i+4])[0]
        float_values.append(float_value)

    points = np.array(float_values, dtype=np.float32).reshape(-1, data_msg.point_step // 4)

    pcd.points = o3d.utility.Vector3dVector(points[:,:3])

    # Extract RGB data from the data field if available
    rgb_offset = None

    for field in data_msg.fields:
        if field.name == 'rgb':
            rgb_offset = field.offset

    if rgb_offset is not None:

        rgb_stride = data_msg.point_step // 4
        colors = []
        for i in range(points.shape[0]):
            colors.append(struct.unpack('I', struct.pack('f', points[i, 4]))[0])
        colors = np.array(colors, dtype=np.uint32).reshape(-1, 1)
        r = (colors[:, 0] >> 16) & 0xFF
        g = (colors[:, 0] >> 8) & 0xFF
        b = colors[:, 0] & 0xFF   

        
        colors = np.array([r.reshape(-1), g.reshape(-1), b.reshape(-1)]).reshape((-1,3))
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(float) / 255.0)

    return pcd