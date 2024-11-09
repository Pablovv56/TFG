#!/usr/bin/env python3

import rclpy
import numpy as np
import copy as cp
import open3d as o3d
import struct
import tf_transformations as tf
import time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node

class LocationNode (Node):

    # Last point cloud data recieved
    actual_PCD = None
    # Last pose data recieved calculated
    actual_pose = np.identity(4)

    def __init__(self, input_topic : str):

        # Initialize node
        super().__init__("localization_node")

        # Create subscription to /cloud_in, triggers pose_callback
        self.location_node_ = self.create_subscription(PointCloud2, input_topic, self.pose_callback, 1000)
        
        # Create a publisher that will provide the actual pose
        self.location_publisher_ = self.create_publisher(PoseStamped, 'new_pose', 10)

    def pose_callback (self, msg : PointCloud2):

        start_time = time.time()
        
        # If this is the first message, store it
        if self.actual_PCD is None:

            self.actual_PCD = self.pointcloud2_to_open3d(msg)

        # If this is not the first message, compare it with the previous one
        else:
            
            # Copy of the previous step
            source_temp = self.actual_PCD
            # Copy of the actual step
            target_temp = self.pointcloud2_to_open3d(msg)
            
            voxel_size = 0.1
            threshold = 0.04
            # Preprocess clouds for registration
            source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(source_temp, target_temp, voxel_size)
            print(f"prepare_dataset done at {time.time() - start_time:.4f} seconds")

            # Initial alignment (Ransac or Fast global registration)
            initial_alignment = self.ransac_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
            print(f"Initial alignment done at {time.time() - start_time:.4f} seconds")
            
            # Line to view initial aligment result
            # self.draw_registration_result(source_temp, target_temp, initial_alignment.transformation)
            
            """ # Downsample target point cloud before normal estimation
            target_temp_downsampled = target_temp.voxel_down_sample(voxel_size)
            print(f"Target downsampled at {time.time() - start_time:.4f} seconds") """
            
            # Compute normals for the target point cloud
            target_temp.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
            print(f"Normals computed at {time.time() - start_time:.4f} seconds")

            # Ensure the normals are oriented consistently
            target_temp.orient_normals_consistent_tangent_plane(100)
            print(f"Normals oriented at {time.time() - start_time:.4f} seconds")
            
            #ICP registration result using Point to Plane method
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source_temp, target_temp, threshold, initial_alignment.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPlane())
            print(f"ICP registration done at {time.time() - start_time:.4f} seconds")
            
            # self.draw_registration_result(source_temp, target_temp, reg_p2p.transformation)
            
            # Update the actual pose with the new transformation
            self.actual_pose = np.dot(self.actual_pose, reg_p2p.transformation)
            print(f"Pose updated at {time.time() - start_time:.4f} seconds")
            
            # Convert actual pose to PoseStamped
            stamped_pose = self.matrix_to_pose_stamped(self.actual_pose)
            print(f"Pose published at {time.time() - start_time:.4f} seconds")
            
            # Publish the stamped pose
            self.location_publisher_.publish(stamped_pose)
            

    def ransac_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size):

        # Cloud downsampling ratio to determine correspondance between clouds
        distance_threshold = voxel_size * 1.5

        # Calculates the transformation matrix using ransac
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        
        return result
    
    def execute_fast_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size):

        # Cloud downsampling ratio to determine correspondance between clouds
        distance_threshold = voxel_size * 0.5
        
        # Calculates the transformation matrix using fast global registration
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        
        return result
    
    def prepare_dataset(self, source_pcd, target_pcd, voxel_size):
        
        # Preprocess pcd with given parameters
        source_down, source_fpfh = self.preprocess_point_cloud(source_pcd, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target_pcd, voxel_size)

        return source_down, target_down, source_fpfh, target_fpfh

    def preprocess_point_cloud(self, pcd, voxel_size):

        # Donwsamples with using voxel size ratio
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        # Estimate normal with radius as search criteria
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        # Coumputes Fast Point Features Histograms with selected search radius
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
        return pcd_down, pcd_fpfh
    

    def matrix_to_pose_stamped(self, matrix, frame_id="map"):
        
        print(matrix)
        # Extraer la traslación
        translation = tf.translation_from_matrix(matrix)
        
        # Extraer la rotación en forma de cuaternión
        quaternion = tf.quaternion_from_matrix(matrix)
        
        # Crear el mensaje PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        # Asignar la traslación
        pose_stamped.pose.position.x = translation[0]
        pose_stamped.pose.position.y = translation[1]
        pose_stamped.pose.position.z = translation[2]
        
        # Asignar la rotación
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        
        # 
        
        return pose_stamped

    
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
        
    def draw_registration_result(self, source, target, transformation):
        source_temp = cp.deepcopy(source)
        target_temp = cp.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                        zoom=0.4459,
                                        front=[0.9288, -0.2951, -0.2242],
                                        lookat=[1.6784, 2.0612, 1.4451],
                                        up=[-0.3402, -0.9189, -0.1996])
    