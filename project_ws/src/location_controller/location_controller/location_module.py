#!/usr/bin/env python3
import os
import rclpy
import time
import logging
import yaml
import numpy as np
import copy as cp
import open3d as o3d


from location_controller.utils import pointcloud2_to_open3d, open3d_to_pointcloud2, matrix_to_pose_stamped, preprocess_point_cloud, pose_distance, draw_registration_result, crop_point_cloud, compute_cloud_novelty_ratio
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
        super().__init__("LocalizationNode")

        # Create subscription to /cloud_in, triggers pose_callback
        self.location_node_ = self.create_subscription(PointCloud2, input_topic, self.pose_callback, 1000)
        
        # Create subscription to /cloud_in, triggers pose_callback
        self.map_subscriber = self.create_subscription(PointCloud2, "/bonxai_point_cloud_centers", self.map_callback, 1000)
        
        # Create a publisher that will provide the actual pose
        self.location_publisher_ = self.create_publisher(PoseStamped, 'new_pose', 10)
        
        # Create a publisher that will provide the actual pose
        self.transformed_pcl_publisher_ = self.create_publisher(PointCloud2, 'new_pcl', 10)

        # Variable for indexing messages
        self.index = 0
        
        # Variable that stores the index of the last key_frame
        self.last_map_update = 1
        
        # Last Point Cloud data recieved
        self.previous_pcl = None
        
        # Initial pose of the key frame
        self.key_pose = None
        
        # Initial key frame
        self.key_frame = None
        
        # Last map update recieved
        self.map = None
        
        # Last pose data recieved calculated
        self.actual_pose = np.identity(4)
        """[[ 0.99630944 ,-0.08583165 ,-0.00065074 , 0.10285288],
            [ 0.08583165 , 0.9961947  , 0.01513443 , 0.01087134],
            [-0.00065074 ,-0.01513443 , 0.99988526 , 0.01813573],
            [ 0.         , 0.         , 0.         , 1.        ]]"""
            
        # Configuration parameters 
        with open(YAML_PATH, "r") as file:
            self.config = yaml.safe_load(file)
            
        if logger.isEnabledFor(logging.DEBUG):
            
            # Frame to frame logging variables
            self.ftf_fitness_error_counter = 0
            self.ftf_fitness_mean_error = 0
            
            self.ftf_rmse_error_counter = 0
            self.ftf_rmse_mean_error = 0
            
            self.ftf_distance_error_counter = 0
            self.ftf_distance_mean_error = 0
            
            self.ftf_angle_error_counter = 0
            self.ftf_angle_mean_error = 0
            
            # Key frame logging variables
            self.kf_index = 0
            
            self.kf_distance_error_counter = 0
            self.kf_distance_mean_error = 0
            
            self.kf_angle_error_counter = 0
            self.kf_angle_mean_error = 0
            
            # Map overlap logging variables
            self.map_index = 0
            
            self.map_overlap_error_counter = 0
            self.map_overlap_mean_error = 0
            
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
            new_preprocessed_frame = preprocess_point_cloud(msg = msg, transformation = self.actual_pose)
            
            # Store the last point cloud data
            self.previous_pcl = cp.deepcopy(new_preprocessed_frame)
            
            # Initialize the key pose
            self.key_pose = self.actual_pose
            
            # Initialize the key frame
            self.key_frame = cp.deepcopy(new_preprocessed_frame)
            
            # Convert from open3d to PointCloud2
            publish_map = open3d_to_pointcloud2(new_preprocessed_frame)
            
            # Publish the transformed pcd
            self.transformed_pcl_publisher_.publish(publish_map)
            
            if logger.isEnabledFor(logging.DEBUG):
                elapsed_time = time.time() - start_time
                self.logging_data += f"Elapsed time: {elapsed_time:.2f} seconds.\n"
                logger.debug(self.logging_data)
            
            # Log the msg number        
            logger.info(f"MSG Nº {self.index} published \n")

        # If this is not the first message, compare it with the previous one
        else:
            
            # Update variable for indexing the poses
            self.index += 1
        
            # Transform the point cloud to the current pose and downsample it
            new_preprocessed_frame = preprocess_point_cloud(msg = msg, transformation = self.actual_pose)

            # Compute the new pose and map using ICP logic
            new_pose, new_map = self.icp_logic(new_preprocessed_frame, msg)

            # Update the actual pose
            self.actual_pose = new_pose

            # If the new frame has not been accepted as a map frame, only post the pose
            if new_map is None:
                    
                # Convert actual pose to PoseStamped
                publish_pose = matrix_to_pose_stamped(self.actual_pose, self.index)

                # Publish the stamped pose
                self.location_publisher_.publish(publish_pose)
            
            else:
                
                # Convert from open3d 
                publish_map = open3d_to_pointcloud2(new_map)
                
                # Publish the transformed pcd
                self.transformed_pcl_publisher_.publish(publish_map)
            
            
            # Publish the elapsed time if debug mode is enabled        
            if logger.isEnabledFor(logging.DEBUG):
                elapsed_time = time.time() - start_time
                self.logging_data += f"\tElapsed time: {elapsed_time:.2f} seconds.\n"
                logger.debug(self.logging_data)
                    
            # Log the msg number        
            logger.info(f"MSG Nº {self.index} published \n")
            logger.info("------------------------------------------------------------------------------")
    
            
    def map_callback (self, msg: PointCloud2):
        
        # Save the map 
        self.map = pointcloud2_to_open3d(msg)
        
        # Compute the normals of the map
        self.map.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius = self.config["MAP_NORMAL_FACTOR"] * self.config["VOXEL_SIZE"], max_nn=30))
        
        # Convert actual pose to PoseStamped
        publish_pose = matrix_to_pose_stamped(self.actual_pose, self.index)

        # Publish the stamped pose
        self.location_publisher_.publish(publish_pose)
 
 
    def icp_logic(self, new_preprocess_frame, original_frame):
        
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

        # If its been less than MAX_FRAMES_BETWEEN_KEYFRAMES since the last key frame, we skip the key frame check
        if (((self.index - self.last_map_update) % self.config["FTF_MAX_FRAMES_BETWEEN_KEYFRAMES"]) != 0):
        
            ftf_new_pose, ftf_new_frame, ftf_registration = self.aling_frames(new_preprocess_frame, self.previous_pcl, self.actual_pose, self.config["FTF_CORRESPONDENCE_DISTANCE"])

            # Calculate the distance and angle between the new pose and the actual pose
            distance, angle = pose_distance(self.actual_pose, ftf_new_pose)
        
            if logger.isEnabledFor(logging.DEBUG):
                
                logging_stats = "\tFTF STATS:\n"
                
                if (ftf_registration.fitness < self.config["FTF_FITNESS_THRESHOLD"]):
                    self.ftf_fitness_error_counter += 1
                    logging_stats += f"\033[31m\tFitness: {str(ftf_registration.fitness)}\033[0m | "
                else:
                    logging_stats += f"\033[32m\tFitness: {str(ftf_registration.fitness)}\033[0m | "
                
                self.ftf_fitness_mean_error = ((self.ftf_fitness_mean_error * (self.index - 1)) + ftf_registration.fitness) / self.index
                logging_stats += f"Fitness errors: {self.ftf_fitness_error_counter} | Fitness mean: {self.ftf_fitness_mean_error} \n"
                    
                if (ftf_registration.inlier_rmse > self.config["FTF_RMSE_THRESHOLD"]):
                    self.ftf_rmse_error_counter += 1
                    logging_stats += f"\033[31m\tRMSE: {str(ftf_registration.inlier_rmse)}\033[0m | "
                else:
                    logging_stats += f"\033[32m\tRMSE: {ftf_registration.inlier_rmse}\033[0m | "
                    
                self.ftf_rmse_mean_error = ((self.ftf_rmse_mean_error * (self.index - 1)) + ftf_registration.inlier_rmse) / self.index
                logging_stats += f"RMSE errors: {self.ftf_rmse_error_counter} | RMSE mean: {self.ftf_rmse_mean_error} \n"
                    
                if (distance > self.config["FTF_DISTANCE_THRESHOLD"]):
                    self.ftf_distance_error_counter += 1
                    logging_stats += f"\033[31m\tDistance: {str(distance)}\033[0m | "
                else:
                    logging_stats += f"\033[32m\tDistance: {str(distance)}\033[0m | "
                    
                self.ftf_distance_mean_error = ((self.ftf_distance_mean_error * (self.index - 1)) + distance) / self.index
                logging_stats += f"Distance errors: {self.ftf_distance_error_counter} | Distance mean: {self.ftf_distance_mean_error} \n"
                    
                if (angle > self.config["FTF_ANGLE_THRESHOLD"]):
                    self.ftf_angle_error_counter += 1
                    logging_stats += f"\033[31m\tAngle: {str(angle)}\033[0m | "
                else:
                    logging_stats += f"\033[32m\tAngle: {str(angle)}\033[0m | "
                              
                self.ftf_angle_mean_error = ((self.ftf_angle_mean_error * (self.index - 1)) + angle) / self.index
                logging_stats += f"Angle errors: {self.ftf_angle_error_counter} | Angle mean: {self.ftf_angle_mean_error}"
            
                logger.debug(logging_stats)
                
            # If the fitness and inlier_rmse are good enough and the transition is smooth, we skip the key frame check
            if (ftf_registration.fitness > self.config["FTF_FITNESS_THRESHOLD"]) and \
               (ftf_registration.inlier_rmse < self.config["FTF_RMSE_THRESHOLD"]) and \
               (distance < self.config["FTF_DISTANCE_THRESHOLD"]) and \
               (angle < self.config["FTF_ANGLE_THRESHOLD"]):

                # We accept the new pose
                final_pose = ftf_new_pose
                
                # Save the last point cloud data
                self.previous_pcl = cp.deepcopy(ftf_new_frame)

                logger.info("\033[34mFrame-to-Frame ICP Accepted\033[0m")
                
                # The FTF registration is good, so we return the new pose and the new frame
                return final_pose, None
                
        # If we reach this point, we need to check if the new frame is a key frame
        is_key_frame, final_pose, final_frame = self.is_new_key_frame(original_frame)

        # If it is not a key frame, we update the last point cloud data
        self.previous_pcl = cp.deepcopy(final_frame)

        # If it is a key frame, we update the last key frame index and keep the new frame            
        if is_key_frame:
            
            # Update the index of the last key frame added
            self.last_map_update = self.index
        
        # If it is not a key frame, we keep the last pose and the last point cloud data
        else:
            
            # Revert to the last pose key frame
            final_frame = None
            
            logger.info("\033[33mKey-Frame ICP Rejected\033[0m")
        
        # As the new frame is not a key frame, we return the last one os that the map does not change
        return final_pose, final_frame
    
    
    def is_new_key_frame(self, raw_new_frame):
        """
        Determine if the new frame is a new key frame based on the distance to the current key frame.
        """
        
        new_frame = preprocess_point_cloud(raw_new_frame, transformation = self.key_pose)
        
        kf_pose, kf_frame, _ = self.aling_frames(new_frame, self.key_frame, self.key_pose, self.config["KF_CORRESPONDENCE_DISTANCE"])
        
        # Calculate the overlap ratio with the key frame
        distance, angle = pose_distance(kf_pose, self.key_pose)
        
        if logger.isEnabledFor(logging.DEBUG):
            
            logging_stats = "KF STATS:\n"
            
            if (distance < self.config["KF_DISTANCE_THRESHOLD"]):
                self.kf_distance_error_counter += 1
                logging_stats += f"\033[32m\t\tDistance: {str(distance)} | Counter: {self.kf_distance_error_counter} \033[0m"
            else:
                logging_stats += f"\033[31m\t\tDistance: {str(distance)} | Counter: {self.kf_distance_error_counter} \033[0m"
                
            self.kf_distance_mean_error = ((self.kf_distance_mean_error * (self.index - 1)) + distance) / self.index
            logging_stats += f"Distance mean: {self.kf_distance_mean_error}\n"
            
            if (angle < self.config["KF_ANGLE_THRESHOLD"]):
                self.kf_angle_error_counter += 1
                logging_stats += f"\033[32m\t\tAngle: {str(angle)} | Counter: {self.kf_angle_error_counter} \033[0m"
            else:
                logging_stats += f"\033[31m\t\tAngle: {str(angle)} | Counter: {self.kf_angle_error_counter} \033[0m"
                
            self.kf_angle_mean_error = ((self.kf_angle_mean_error * (self.index - 1)) + angle) / self.index
            logging_stats += f"Angle mean: {self.kf_angle_mean_error}\n"
                            
            logger.debug(logging_stats)
            
        # If the distance is greater than the threshold, we have a new key frame
        if ((distance > self.config["KF_DISTANCE_THRESHOLD"]) or \
            (angle > self.config["KF_ANGLE_THRESHOLD"])):
            
            logger.info("\033[32mKey frame added\033[0m")
            
            temp_map = cp.deepcopy(self.map)
        
            temp_map = crop_point_cloud(temp_map, self.key_pose)
        
            map_pose, map_frame, _ = self.aling_frames(kf_frame, temp_map, kf_pose, self.config["MAP_CORRESPONDENCE_DISTANCE"])
                
            o3d.geometry.PointCloud.estimate_normals(map_frame, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.config["VOXEL_SIZE"] * self.config["NORMAL_FACTOR"], max_nn=30))
                
            self.key_pose = map_pose
                
            self.key_frame = cp.deepcopy(map_frame)
                
            # Calculate the overlap ratio with the key frame
            map_overlap_ratio = compute_cloud_novelty_ratio(map_frame, self.map)
            
            if logger.isEnabledFor(logging.DEBUG):
            
                logging_stats = "MAP STATS:\n"
                
                if (map_overlap_ratio > self.config["MAP_NOVELTY_RATIO_THRESHOLD"]):
                    self.map_overlap_error_counter += 1
                    logging_stats += f"\033[32m\t\tNovelty: {str(map_overlap_ratio)} | Counter: {self.map_overlap_error_counter} \033[0m"
                else:
                    logging_stats += f"\033[31m\t\tNovelty: {str(map_overlap_ratio)} | Counter: {self.map_overlap_error_counter} \033[0m"
                    
                self.map_overlap_mean_error = ((self.map_overlap_mean_error * (self.index - 1)) + map_overlap_ratio) / self.index
                logging_stats += f"Novelty mean: {self.map_overlap_mean_error}\n"
                                
                self.index += 1
                            
                logger.debug(logging_stats)
            
            if map_overlap_ratio > self.config["MAP_NOVELTY_RATIO_THRESHOLD"]:
                
                logger.info("\033[32mMap frame added\033[0m")
                
                return True, map_pose, map_frame
            
            return False, map_pose, map_frame
                    
        return False, kf_pose, kf_frame
    
    
    def aling_frames (self, source_frame, target_frame, actual_pose, correspondence_distance):
        
        # Compute frame to frame ICP registration
        registration_result = o3d.pipelines.registration.registration_icp(
                                    source = source_frame, 
                                    target = target_frame,
                                    max_correspondence_distance = correspondence_distance,
                                    estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50, 
                                                                                                relative_fitness=1e-3,
                                                                                                relative_rmse=1e-3
                                                                                                )                      
                                )   
    
        # Calculate the new pose temporarily to check if the movement is not too large
        updated_pose = np.dot(actual_pose, registration_result.transformation)
        
        # Transform the last pcd recieved to the new pose
        updated_frame = cp.deepcopy(source_frame.transform(registration_result.transformation))
        
        return updated_pose, updated_frame, registration_result