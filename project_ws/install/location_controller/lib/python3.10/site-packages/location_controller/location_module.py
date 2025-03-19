#!/usr/bin/env python3

import rclpy
import time
import logging
import numpy as np
import copy as cp
import open3d as o3d
import tf_transformations as tf

from location_controller.utils import draw_registration_result, pointcloud2_to_open3d
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node

# Global parameters

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,  # Set the logging level to DEBUG to capture all log messages
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # Log format
    handlers=[
        logging.StreamHandler()  # Output logs to the console
    ]
)

# Initialize logger
logger = logging.getLogger("LocatioNode")

class LocationNode (Node):

    # Class variables
    
    # Adjust fitting parameters
    VOXEL_SIZE = 0.05
    THRESHOLD = 0.01
    VOXEL_REDUCTION_RATIO = 1

    # Preprocess Point Cloud
    NORMAL_FACTOR = 2
    FEATURE_FACTOR = 5

    def __init__(self, input_topic : str):
        
        """
        Initializes the localization node.
        Args:
            input_topic (str): The name of the input topic to subscribe to for PointCloud2 messages.
        Attributes:
            location_node_ (Subscription): Subscription to the specified input topic, triggering the pose_callback.
            location_publisher_ (Publisher): Publisher for broadcasting the calculated pose as a PoseStamped message.
            index (int): Counter for indexing received messages.
            actual_PCD (PointCloud2 or None): Stores the most recent PointCloud2 data received.
            actual_pose (numpy.ndarray): Stores the most recent calculated pose as a 4x4 transformation matrix.
            mean_error (float): Tracks the mean error for debugging purposes (only initialized if debug mode is enabled).
        """

        # Initialize node
        super().__init__("localization_node")

        # Create subscription to /cloud_in, triggers pose_callback
        self.location_node_ = self.create_subscription(PointCloud2, input_topic, self.pose_callback, 1000)
        
        # Create a publisher that will provide the actual pose
        self.location_publisher_ = self.create_publisher(PoseStamped, 'new_pose', 10)

        # Variable for indexing messages
        self.index = 0
    
        # Last point cloud data recieved
        self.actual_PCD = None
        
        # Last pose data recieved calculated
        self.actual_pose = np.identity(4)
        
        if logger.isEnabledFor(logging.DEBUG):
            # Mean error
            self.mean_error = 0
            
        logger.info("Node Initialization Complete")


    def pose_callback (self, msg : PointCloud2):
        
        """
        Callback function to process incoming PointCloud2 messages and perform point cloud registration.
        This function handles the localization process by comparing the incoming point cloud data with 
        the previously stored point cloud. It performs preprocessing, initial alignment, and iterative 
        closest point (ICP) registration to compute the transformation between the two point clouds. 
        The computed transformation is used to update the current pose of the system.
        Args:
            msg (PointCloud2): The incoming point cloud message.
        Workflow:
            1. If this is the first message, store the point cloud and publish the initial pose.
            2. If this is not the first message:
                - Preprocess the source and target point clouds (downsampling and feature extraction).
                - Perform initial alignment using RANSAC or Fast Global Registration.
                - Compute normals for the target point cloud.
                - Perform ICP registration to refine the alignment.
                - Update the current pose using the computed transformation.
                - Publish the updated pose as a PoseStamped message.
        Debugging:
            - Logs the time taken for each step if the `debug` flag is enabled.
            - Computes and logs the Root Mean Square Error (RMSE) of the registration.
            - Optionally visualizes the registration result if the RMSE exceeds the mean error or is zero.
        Notes:
            - The function assumes that the incoming point cloud is in the same coordinate frame as the 
              previously stored point cloud.
            - The transformation matrix is inverted before updating the current pose.
            - The function uses Open3D for point cloud processing and registration.
        Raises:
            None
        """

        if logger.isEnabledFor(logging.DEBUG):
            start_time = time.time()
            self.logging_data = ""
        
        # If this is the first message, store it
        if self.actual_PCD is None:

            # Store the actual pcd
            self.actual_PCD = pointcloud2_to_open3d(msg)
            
            # Convert the matrix pose to a stamped pose
            stamped_pose = self.matrix_to_pose_stamped(self.actual_pose)
            
            # Publish the stamped pose
            self.location_publisher_.publish(stamped_pose)
            
            logger.info("First Message published")

        # If this is not the first message, compare it with the previous one
        else:
            # Update variable for indexing the poses
            self.index += 1

            # Copy of the previous step
            source_temp = self.actual_PCD
            
            # Copy of the actual step
            target_temp = pointcloud2_to_open3d(msg)
            
            # Preprocess clouds for registration (Downsampling)
            source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(source_temp, target_temp)
            
            if logger.isEnabledFor(logging.DEBUG):
                self.logging_data += "Downsampling: " + str(time.time() - start_time) + "\n"

            # Initial alignment (Ransac or Fast global registration)
            initial_alignment = self.ransac_global_registration(source_down, target_down, source_fpfh, target_fpfh)
            
            if logger.isEnabledFor(logging.DEBUG):
                self.logging_data += "Initial_alingment: " + str(time.time() - start_time) + "\n"

            # ICP registration result using Point to Plane method
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source_down, target_down, self.THRESHOLD, initial_alignment.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPlane())
            
            if logger.isEnabledFor(logging.DEBUG):
                self.logging_data += "ICP: " + str(time.time() - start_time) + "\n"
            
            # Code for debugging
            if logger.isEnabledFor(logging.DEBUG):
                
                self.mean_error += (reg_p2p.inlier_rmse - self.mean_error) / self.index
                                
                self.logging_data += "RMSE: " + str(reg_p2p.inlier_rmse) + " / Mean:" + str(self.mean_error) + "\n"
                
                # if (self.mean_error < reg_p2p.inlier_rmse) or (reg_p2p.inlier_rmse == 0.0):
                #    draw_registration_result(source_down, target_down, reg_p2p.transformation)
            
            # Compute the inverse of the transformation matrix
            transformation_inv = np.linalg.inv(reg_p2p.transformation)
            
            if reg_p2p.inlier_rmse != 0.0:
                # Update the actual pose with the new transformation
                self.actual_pose = np.dot(self.actual_pose, transformation_inv)
                
            if logger.isEnabledFor(logging.DEBUG):
                self.logging_data += "Pose_updated: " + str(time.time() - start_time) + "\n"
            
                # We update the PCD for the next step
                self.actual_PCD = target_temp
            
            elif logger.isEnabledFor(logging.DEBUG):
                logger.warning("WARNING: Pose not updated due to missalignment (RMSE == 0.0)")
            
            # Convert actual pose to PoseStamped
            stamped_pose = self.matrix_to_pose_stamped(self.actual_pose)

            # Publish the stamped pose
            self.location_publisher_.publish(stamped_pose)
            
            if logger.isEnabledFor(logging.DEBUG):
                self.logging_data += "Pose_published: " + str(time.time() - start_time) + "\n"
                
                logger.debug("\n" + self.logging_data)
                     
            logger.info(f"MSG Nº {self.index} published")
            
                      
    def ransac_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size = VOXEL_SIZE, ratio = VOXEL_REDUCTION_RATIO):
        
        """
        Perform global registration of two point clouds using the RANSAC algorithm.
        This method uses RANSAC-based feature matching to estimate the transformation
        matrix that aligns the source point cloud to the target point cloud.
        Args:
            source_down (open3d.geometry.PointCloud): The downsampled source point cloud.
            target_down (open3d.geometry.PointCloud): The downsampled target point cloud.
            source_fpfh (open3d.pipelines.registration.Feature): The FPFH features of the source point cloud.
            target_fpfh (open3d.pipelines.registration.Feature): The FPFH features of the target point cloud.
            voxel_size (float, optional): The voxel size used for downsampling and determining
                the distance threshold. Defaults to VOXEL_SIZE.
        Returns:
            open3d.pipelines.registration.RegistrationResult: The result of the RANSAC registration,
            containing the estimated transformation matrix and inlier information.
        Notes:
            - The `distance_threshold` is set to 1 by default, which can be adjusted based on
              the voxel size and the desired level of correspondence.
            - The RANSAC algorithm uses a combination of correspondence checkers:
              - Edge length checker with a threshold of 0.9.
              - Distance checker with the specified `distance_threshold`.
            - The RANSAC convergence criteria are set to a maximum of 100,000 iterations
              and a confidence level of 0.999.
        """

        # Cloud downsampling ratio to determine correspondance between clouds
        distance_threshold =  voxel_size * ratio

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
    
    
    def execute_fast_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size = VOXEL_SIZE, ratio = VOXEL_REDUCTION_RATIO):
        """
        Executes the Fast Global Registration (FGR) algorithm to align two point clouds.
        This function computes the transformation matrix that aligns the source point cloud
        to the target point cloud using feature-based matching and the Fast Global Registration method.
        Args:
            source_down (open3d.geometry.PointCloud): The downsampled source point cloud.
            target_down (open3d.geometry.PointCloud): The downsampled target point cloud.
            source_fpfh (open3d.pipelines.registration.Feature): The FPFH features of the source point cloud.
            target_fpfh (open3d.pipelines.registration.Feature): The FPFH features of the target point cloud.
            voxel_size (float, optional): The voxel size used for downsampling. Defaults to VOXEL_SIZE.
        Returns:
            open3d.pipelines.registration.RegistrationResult: The result of the registration, 
            including the transformation matrix and fitness score.
        """

        # Cloud downsampling ratio to determine correspondance between clouds
        distance_threshold =  voxel_size * ratio
        
        # Calculates the transformation matrix using fast global registration
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance = distance_threshold))
        
        return result
    
    
    def prepare_dataset(self, source_pcd, target_pcd, voxel_size = VOXEL_SIZE, normal_factor = NORMAL_FACTOR, feature_factor = FEATURE_FACTOR):
        
        """
        Prepares the dataset for point cloud registration by preprocessing the source and target point clouds.
        Args:
            source_pcd (open3d.geometry.PointCloud): The source point cloud to be processed.
            target_pcd (open3d.geometry.PointCloud): The target point cloud to be processed.
            voxel_size (float, optional): The voxel size used for downsampling the point clouds. Defaults to VOXEL_SIZE.
            normal_factor (float, optional): The factor used for estimating normals. Defaults to NORMAL_FACTOR.
            feature_factor (float, optional): The factor used for computing FPFH features. Defaults to FEATUR_FACTOR.
        Returns:
            tuple: A tuple containing:
                - source_down (open3d.geometry.PointCloud): The downsampled source point cloud.
                - target_down (open3d.geometry.PointCloud): The downsampled target point cloud.
                - source_fpfh (open3d.pipelines.registration.Feature): The FPFH features of the source point cloud.
                - target_fpfh (open3d.pipelines.registration.Feature): The FPFH features of the target point cloud.
        """
        
        # Preprocess pcd with given parameters
        source_down, source_fpfh = self.preprocess_point_cloud(source_pcd, voxel_size, normal_factor, feature_factor)
        target_down, target_fpfh = self.preprocess_point_cloud(target_pcd, voxel_size, normal_factor, feature_factor)

        return source_down, target_down, source_fpfh, target_fpfh


    def preprocess_point_cloud(self, pcd, voxel_size, normal_factor, feature_factor ):
        
        """
        Preprocesses a point cloud by downsampling, estimating normals, and computing FPFH features.
        Args:
            pcd (open3d.geometry.PointCloud): The input point cloud to preprocess.
            voxel_size (float): The voxel size used for downsampling the point cloud.
            normal_factor (float): A multiplier for the voxel size to determine the radius for normal estimation.
            feature_factor (float): A multiplier for the voxel size to determine the radius for FPFH feature computation.
        Returns:
            tuple: A tuple containing:
                - pcd_down (open3d.geometry.PointCloud): The downsampled point cloud.
                - pcd_fpfh (open3d.pipelines.registration.Feature): The computed FPFH features of the downsampled point cloud.
        """

        # Donwsamples with using voxel size ratio
        pcd_down = pcd.voxel_down_sample(voxel_size * normal_factor)

        radius_normal = voxel_size * self.NORMAL_FACTOR
        
        # Estimate normal with radius as search criteria
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * feature_factor
        
        # Coumputes Fast Point Features Histograms with selected search radius
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
        return pcd_down, pcd_fpfh
    
    
    def matrix_to_pose_stamped(self, matrix, frame_id="map"):
        
        # Extraer la traslación
        translation = tf.translation_from_matrix(matrix)
        
        # Extraer la rotación en forma de cuaternión
        quaternion = tf.quaternion_from_matrix(matrix)

        # Crear el mensaje PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp.sec = self.index
        
        # Asignar la traslación
        pose_stamped.pose.position.x = translation[0]
        pose_stamped.pose.position.y = translation[1]
        pose_stamped.pose.position.z = translation[2]
        
        # Asignar la rotación
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        
        return pose_stamped