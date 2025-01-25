import numpy as np
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class PointCloud2ToOpen3DConverter (Node):
    
    def __init__(self):
        
        #Initialize node
        super().__init__("pointcloud2_to_open3d_converter")
        
        #Create subscription to /cloud_in, triggers pose_callback
        self.cloud_in_subs = self.create_subscription(PointCloud2, "/cloud_in", self.pose_callback, 10)
        self.cloud_out_pub = self.create_publisher(o3d.geometry.PointCloud, "O3d_cloud_out", 10)
        
    def pose_callback (self, msg : PointCloud2):
        
        pcd = self.pointcloud2_to_open3d(msg)
        self.cloud_out_pub.publish(pcd)

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