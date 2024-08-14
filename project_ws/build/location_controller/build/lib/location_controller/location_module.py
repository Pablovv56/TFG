#!/usr/bin/env python3

import copy as cp
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node

class LocationNode (Node):

    actual_num = 0
    actual_PCD = None

    def __init__(self):

        #Initialize node
        super().__init__("localization_node")

        #Create subscription to /cloud_in, triggers pose_callback
        self.location_node_ = self.create_subscription(PointCloud2, "/cloud_in", self.pose_callback, 10)

    def pose_callback (self, msg : PointCloud2):

        #If
        if self.actual_num == 0:

            self.actual_PCD = cp.deepcopy(msg)

        else:
            
            #Copy of the previous step
            source_temp = self.actual_PCD
            #Copy of the actual step
            target_temp = cp.deepcopy(msg)
            
            voxel_size = 0.1
            threshold = 0.04
            #Preprocess clouds for registration
            source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(source_temp, target_temp, voxel_size)

            #Initial alignment (Ransac or Fast global registration)
            initial_alignment = self.execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

            #ICP registration result using Point to Plane method
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source_temp, target_temp, threshold, initial_alignment,
                o3d.pipelines.registration.TransformationEstimationPointToPlane())



    def ransac_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size):

        #Cloud downsampling ratio to determine correspondance between clouds
        distance_threshold = voxel_size * 1.5

        #Calculates the transformation matrix using ransac
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

        #Cloud downsampling ratio to determine correspondance between clouds
        distance_threshold = voxel_size * 0.5
        
        #Calculates the transformation matrix using fast global registration
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        
        return result
    
    def prepare_dataset(self, source_pcd, target_pcd, voxel_size):
        
        #Preprocess pcd with given parameters
        source_down, source_fpfh = self.preprocess_point_cloud(source_pcd, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target_pcd, voxel_size)

        return source_down, target_down, source_fpfh, target_fpfh

    def preprocess_point_cloud(pcd, voxel_size):

        #Donwsamples with using voxel size ratio
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        #Estimate normal with radius as search criteria
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))


        radius_feature = voxel_size * 5
        #Coumputes Fast Point Features Histograms with selected search radius
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
        return pcd_down, pcd_fpfh