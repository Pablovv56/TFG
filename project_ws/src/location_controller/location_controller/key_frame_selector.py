import os
import math
import random
import yaml
import logging
import copy as cp
import numpy as np
import open3d as o3d

from location_controller.utils import preprocess_point_cloud, compute_cloud_novelty_ratio, crop_point_cloud, pose_distance, draw_registration_result

# Global parameters
YAML_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../src/config/general_config.yaml"))

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,  # Set the logging level to DEBUG to capture all log messages
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # Log format
    handlers=[
        logging.StreamHandler()  # Output logs to the console
    ]
)

# Initialize logger
logger = logging.getLogger("KeyFrameSelector")

class KeyFrameSelector:
    
    def __init__(self, initial_pose, initial_key_frame):
        
        # Initial pose of the key frame
        self.key_pose = initial_pose
        
        # Initial key frame
        self.key_frame = cp.deepcopy(initial_key_frame)
        
        # Initial map (same as the initial key frame)
        self.map = cp.deepcopy(initial_key_frame)

        # Load configuration parameters        
        with open(YAML_PATH, "r") as file:
            self.config = yaml.safe_load(file)
            
        if logger.isEnabledFor(logging.DEBUG):
            
            self.index = 1
            
            self.distance_counter_error = 0
            self.distance_mean_error = 0
            
            self.angle_counter_error = 0
            self.angle_mean_error = 0
            
            self.map_overlap_error_counter = 0
            self.map_overlap_mean_error = 0
        
    
    def is_new_key_frame(self, raw_new_frame):
        """
        Determine if the new frame is a new key frame based on the distance to the current key frame.
        """
        
        new_frame = preprocess_point_cloud(raw_new_frame, transformation = self.key_pose)
        
        # Refine the alignment using ICP        
        reg_p2p_kf = o3d.pipelines.registration.registration_icp(
            source = new_frame, 
            target = self.key_frame,
            max_correspondence_distance = self.config["KF_CORRESPONDENCE_DISTANCE"],
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50, 
                                                                        relative_fitness=1e-6,
                                                                        relative_rmse=1e-6
                                                                        )                      
        )
        
        # Transform the new frame using the refined alignment
        new_frame_alinged_kf = new_frame.transform(reg_p2p_kf.transformation)
        
        # Current pose aligned with the refined alignment
        new_pose_alinged_kf = np.dot(reg_p2p_kf.transformation, self.key_pose)
        
        # Calculate the overlap ratio with the key frame
        distance, angle = pose_distance(new_pose_alinged_kf, self.key_pose)
        
        if logger.isEnabledFor(logging.DEBUG):
            
            logging_stats = "KF STATS:\n"
            
            if (distance < self.config["KF_DISTANCE_THRESHOLD"]):
                self.distance_counter_error += 1
                logging_stats += f"\033[32m\t\tDistance: {str(distance)} | Counter: {self.distance_counter_error} \033[0m"
            else:
                logging_stats += f"\033[31m\t\tDistance: {str(distance)} | Counter: {self.distance_counter_error} \033[0m"
                
            self.distance_mean_error = ((self.distance_mean_error * (self.index - 1)) + distance) / self.index
            logging_stats += f"Distance mean: {self.distance_mean_error}\n"
            
            if (angle < self.config["KF_ANGLE_THRESHOLD"]):
                self.angle_counter_error += 1
                logging_stats += f"\033[32m\t\tAngle: {str(angle)} | Counter: {self.angle_counter_error} \033[0m"
            else:
                logging_stats += f"\033[31m\t\tAngle: {str(angle)} | Counter: {self.angle_counter_error} \033[0m"
                
            self.angle_mean_error = ((self.angle_mean_error * (self.index - 1)) + angle) / self.index
            logging_stats += f"Angle mean: {self.angle_mean_error}\n"
                            
            logger.debug(logging_stats)
            
        # If the distance is greater than the threshold, we have a new key frame
        if ((distance > self.config["KF_DISTANCE_THRESHOLD"]) or \
            (angle > self.config["KF_ANGLE_THRESHOLD"])):
            
            temp_map = cp.deepcopy(self.map)
        
            temp_map = crop_point_cloud(temp_map, self.key_pose)
        
            # Perform ICP registration
            reg_p2p_map = o3d.pipelines.registration.registration_icp(
                        source = new_frame_alinged_kf, 
                        target = temp_map,
                        max_correspondence_distance = self.config["MAP_CORRESPONDENCE_DISTANCE"],
                        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                                    relative_fitness=1e-6,
                                                                                    relative_rmse=1e-6
                                                                                    )                      
                    )
            
            # Current pose aligned with the refined alignment
            new_pose_alinged_map = np.dot(reg_p2p_map.transformation, new_pose_alinged_kf)
                
            self.key_pose = new_pose_alinged_map
                
            # Transform the new frame using the refined alignment
            new_frame_alinged_map = new_frame_alinged_kf.transform(reg_p2p_map.transformation)
                
            o3d.geometry.PointCloud.estimate_normals(new_frame_alinged_map, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.config["VOXEL_SIZE"] * self.config["NORMAL_FACTOR"], max_nn=30))
                
            self.key_frame = cp.deepcopy(new_frame_alinged_map)
                
            # Calculate the overlap ratio with the key frame
            map_overlap_ratio = compute_cloud_novelty_ratio(new_frame_alinged_kf, temp_map)
            
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
                
                
                return True, new_pose_alinged_map, new_frame_alinged_map
            
            return False, new_pose_alinged_map, new_frame_alinged_map
                    
        return False, new_pose_alinged_kf, new_frame_alinged_kf


    def set_map(self, new_map):
        """
        Sets the current map to a new map.
        """
        
        self.map = cp.deepcopy(new_map)
        
    
    def get_key_pose(self):
        """
        Returns the current key frame pose.
        """
        return self.key_pose
    
    
    def get_key_frame(self):
        """
        Returns the current key frame.
        """
        return cp.deepcopy(self.key_frame)