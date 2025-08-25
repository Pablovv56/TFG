#!/usr/bin/env python3
import os
import rclpy
import time
import logging
import yaml
import numpy as np
import copy as cp
import open3d as o3d


from location_controller.utils import pointcloud2_to_open3d, open3d_to_pointcloud2, matrix_to_pose_stamped, preprocess_point_cloud
from location_controller.key_frame_selector import KeyFrameSelector
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node

# Global parameters
YAML_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../src/config/general_config.yaml"))

# Configure logging
logging.basicConfig(
    level=logging.INFO,  # Set the logging level to DEBUG to capture all log messages
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # Log format
    handlers=[
        logging.StreamHandler()  # Output logs to the console
    ]
)

# Initialize logger
logger = logging.getLogger("LocatioNode")

class LocationNode (Node):
    
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
        
        # Create subscription to /cloud_in, triggers pose_callback
        self.map_subscriber = self.create_subscription(PointCloud2, "/bonxai_point_cloud_centers", self.map_callback, 1000)
        
        # Create a publisher that will provide the actual pose
        self.location_publisher_ = self.create_publisher(PoseStamped, 'new_pose', 10)
        
        # Create a publisher that will provide the actual pose
        self.transformed_pcl_publisher_ = self.create_publisher(PointCloud2, 'transformed_pcl', 10)

        # Variable for indexing messages
        self.index = 0
        
        # Variable that stores the index of the last key_frame
        self.last_update = 1
        
        # Last Point Cloud data recieved
        self.last_PCD = None
        
        # Variable for storing the key frame selector class
        self.key_frame_class = None
        
        # Last map update recieved
        self.map = None
        
        # Last pose data recieved calculated
        self.actual_pose = [[ 0.99630944 ,-0.08583165 ,-0.00065074 , 0.10285288],
                            [ 0.08583165 , 0.9961947  , 0.01513443 , 0.01087134],
                            [-0.00065074 ,-0.01513443 , 0.99988526 , 0.01813573],
                            [ 0.         , 0.         , 0.         , 1.        ]]
            
        # Configuration parameters 
        with open(YAML_PATH, "r") as file:
            self.config = yaml.safe_load(file)
            
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

        # If debug mode is enabled, log the start time
        if logger.isEnabledFor(logging.DEBUG):
            start_time = time.time()
            self.logging_data = ""
        
        # If this is the first message, store it
        if self.map is None:
            
            # We compute tdownsample, transform and compoute the normals
            transformed_down_pcd = preprocess_point_cloud(msg, transformation = self.actual_pose)
            
            # Store the last point cloud data
            self.last_PCD = cp.deepcopy(transformed_down_pcd)
            
            # We initialize the map
            self.map = cp.deepcopy(transformed_down_pcd)
            
            # Inicializamos la clase de key frame selector
            self.key_frame_class = KeyFrameSelector(self.actual_pose, transformed_down_pcd)
            
            # Convert from open3d to PointCloud2
            transformed_map_pcd2 = open3d_to_pointcloud2(transformed_down_pcd)
            
            # Publish the transformed pcd
            self.transformed_pcl_publisher_.publish(transformed_map_pcd2)
            
            if logger.isEnabledFor(logging.DEBUG):
                elapsed_time = time.time() - start_time
                self.logging_data += f"Elapsed time: {elapsed_time:.2f} seconds.\n"
            
            # Log the first message
            logger.info("First Message published")

        # If this is not the first message, compare it with the previous one
        else:
            
            # Update variable for indexing the poses
            self.index += 1
        
            # Transform the point cloud to the current pose and downsample it
            new_transformed_down_pcd = preprocess_point_cloud(msg, self.actual_pose)

            # Compute the new pose and map using ICP logic
            new_pose, new_map = self.icp_logic(new_transformed_down_pcd, msg)

            # Update the actual pose
            self.actual_pose = new_pose

            # Convert from open3d 
            transformed_map_pcd2 = open3d_to_pointcloud2(new_map)
            
            # Publish the transformed pcd
            self.transformed_pcl_publisher_.publish(transformed_map_pcd2)
            
            # Publish the elapsed time if debug mode is enabled        
            if logger.isEnabledFor(logging.DEBUG):
                elapsed_time = time.time() - start_time
                self.logging_data += f"Elapsed time: {elapsed_time:.2f} seconds.\n"
                    
            # Log the msg number        
            logger.info(f"MSG Nº {self.index} published")
    
            
    def map_callback (self, msg: PointCloud2):
        
        # Save the map 
        self.map = pointcloud2_to_open3d(msg)
        
        # Compute the normals of the map
        self.map.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=  self.config["NORMAL_FACTOR"] * self.config["VOXEL_SIZE"], max_nn=30))
        
        # Update the key frame selector with the new map
        self.key_frame_class.set_map(self.map)
        
        # Convert actual pose to PoseStamped
        stamped_pose = matrix_to_pose_stamped(self.actual_pose, self.index)

        # Publish the stamped pose
        self.location_publisher_.publish(stamped_pose)
 
    
    def icp_logic(self, actual_transformed_down_pcd, original_msg):
        
        """
        Main logic for performing ICP registration and updating the key frame.
        This method checks if the new frame is a valid key frame based on the distance
        and angle to the current key frame, and updates the key frame if necessary.
        Args:
            actual_transformed_down_pcd (open3d.geometry.PointCloud): The preprocessed point cloud
            to be registered against the current key frame.
        Returns:
            tuple: A tuple containing:
                - new_pose (numpy.ndarray): The updated pose after registration.
                - new_map (open3d.geometry.PointCloud): The transformed point cloud after registration.
        """

        # Compute frame to frame ICP registration
        frame_to_frame_reg = o3d.pipelines.registration.registration_icp(
                                    source = actual_transformed_down_pcd, 
                                    target = self.last_PCD,
                                    max_correspondence_distance = self.config["FTF_CORRESPONDENCE_DISTANCE"],
                                    estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                                                relative_fitness=1e-3,
                                                                                                relative_rmse=1e-3
                                                                                                )                      
                                )   
        
        print("FTF Fitness: " + str(frame_to_frame_reg.fitness) + " / RMSE: " + str(frame_to_frame_reg.inlier_rmse))
        
        # If the fitness is below the threshold or the RMSE is above, we consider if it is a key frame
        if (frame_to_frame_reg.fitness < self.config["FITNESS_THRESHOLD"]) or \
            (frame_to_frame_reg.inlier_rmse > self.config["RMSE_THRESHOLD"]) or \
            (self.index % self.config["MAX_FRAMES_BETWEEN_KEYFRAMES"]) == 0:

            logger.info("\033[31m Checking if it is a key frame...\033[0m")

            # We check if the new frame is a key frame
            is_key_frame, new_pose, new_map = self.key_frame_class.is_new_key_frame(original_msg)

            # If it is a key frame,             
            if is_key_frame:
                
                logger.info("\033[32mKey frame added\033[0m")
                
                # Update the index of the last key frame added
                self.last_update = self.index
                
                self.last_PCD = self.key_frame_class.get_key_frame()
                
                # We return the key frame calculated pose and map
                return new_pose, new_map
                
        # Calculate the new pose
        new_pose = np.dot(self.actual_pose, frame_to_frame_reg.transformation)
        
        # Transform the last pcd recieved to the new pose
        transformed_map_pcd2 = actual_transformed_down_pcd.transform(frame_to_frame_reg.transformation)
        
        # Save the last point cloud data
        self.last_PCD = cp.deepcopy(transformed_map_pcd2)
        
        # As the new frame is not a key frame, we return the last one os that the map does not change
        return new_pose, self.key_frame_class.get_key_frame()