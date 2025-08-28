import os
import math
import random
import yaml
import logging
import copy as cp
import numpy as np
import open3d as o3d

from location_controller.utils import ransac_global_registration, preprocess_point_cloud, pose_distance, multi_res_icp, draw_registration_result

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
            
            self.overlap_error_counter = 0
            self.rmse_error_counter = 0
            self.distance_error_counter = 0
            self.angle_error_counter = 0
        
    
    def is_new_key_frame(self, original_new_frame):
        """
        Determine if the new frame is a new key frame based on the distance to the current key frame.
        """

        # Preprocess the original frame
        new_frame = preprocess_point_cloud(original_new_frame, self.key_pose)
        
        # Initial alignment using RANSAC
        initial_alingment = ransac_global_registration(new_frame, self.key_frame)
        
        # Transform the new frame and the current pose using the initial alignment
        new_frame_corrected = new_frame.transform(initial_alingment.transformation)
        
        # Current pose corrected with the initial alignment
        current_pose_corrected = np.dot(initial_alingment.transformation, self.key_pose)
        
        # Refine the alignment using ICP        
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source = new_frame_corrected, 
            target = self.key_frame,
            max_correspondence_distance = self.config["KF_CORRESPONDENCE_DISTANCE"],
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                        relative_fitness=1e-6,
                                                                        relative_rmse=1e-6
                                                                        )                      
        )
        
        # Transform the new frame using the refined alignment
        new_frame_alinged = new_frame.transform(reg_p2p.transformation)
        
        # Current pose aligned with the refined alignment
        current_pose_alinged = np.dot(reg_p2p.transformation, current_pose_corrected)
            
        # Calculate the distance between the new pose and the key frame
        distance, angle = pose_distance(self.key_pose, current_pose_alinged)
        
        # Calculate the overlap ratio with the key frame
        overlap_ratio = self.compute_cloud_overlap_ratio(new_frame_alinged)
        
        if logger.isEnabledFor(logging.DEBUG):
            
            logging_stats = "FK STATS:\n"
            
            if (overlap_ratio < self.config["KF_OVERLAP_RATIO_THRESHOLD"]):
                self.overlap_error_counter += 1
                logging_stats += f"\033[32m\t\tOverlap: {str(overlap_ratio)} | Counter: {self.overlap_error_counter} \n\033[0m"
            else:
                logging_stats += f"\033[31m\t\tOverlap: {str(overlap_ratio)} | Counter: {self.overlap_error_counter} \n\033[0m"
            
            if (distance > self.config["KF_DISTANCE_THRESHOLD"]):
                self.distance_error_counter += 1
                logging_stats += f"\033[32m\t\tDistance: {str(distance)} | Counter: {self.distance_error_counter}\n\033[0m"
            else:
                logging_stats += f"\033[31m\t\tDistance: {str(distance)} | Counter: {self.overlap_error_counter} \n\033[0m"
                
            if (angle > self.config["KF_ANGLE_THRESHOLD"]):
                self.angle_error_counter += 1
                logging_stats += f"\033[32m\t\tAngle: {str(angle)} | Counter: {self.angle_error_counter} \n\033[0m"
            else:
                logging_stats += f"\033[31m\t\tAngle: {str(angle)} | Counter: {self.angle_error_counter} \n\033[0m"
                            
            logger.debug(logging_stats)
            
            # If the distance is greater than the threshold, we have a new key frame
            if ((distance > self.config["KF_DISTANCE_THRESHOLD"]) or 
                (angle > self.config["KF_ANGLE_THRESHOLD"]) or 
                (overlap_ratio < self.config["KF_OVERLAP_RATIO_THRESHOLD"])):
                
                # Update the key frame, the pose and fuse it with the map
                new_pose_final, new_frame_final = self.update_key_frame(current_pose_alinged, new_frame_alinged)
                
                return True, new_pose_final, new_frame_final
        
        return False, current_pose_alinged, new_frame_alinged
    
         
    def compute_cloud_overlap_ratio(self, new_frame, distance_threshold=0.2, max_samples=2000):
        """
        Calcula el grado de solapamiento entre dos nubes de puntos.
        
        cloud_src, cloud_tgt: o3d.geometry.PointCloud
        distance_threshold: distancia máxima para considerar que un punto tiene correspondencia
        max_samples: número máximo de puntos a muestrear (para acelerar el cálculo)
        
        Return:
            overlap_ratio: float en [0,1]
        """
        n_src = len(new_frame.points)
        if n_src == 0 or len(self.key_frame.points) == 0:
            return 0.0
        
        # Muestreo para acelerar
        if n_src > max_samples:
            idx = random.sample(range(n_src), max_samples)
            sampled_src = o3d.geometry.PointCloud()
            sampled_src.points = o3d.utility.Vector3dVector(np.asarray(new_frame.points)[idx])
        else:
            sampled_src = new_frame
        
        # Distancia punto más cercano
        dists = np.asarray(sampled_src.compute_point_cloud_distance(self.key_frame))
        
        # Porcentaje de puntos con distancia menor a threshold
        overlap_ratio = float((dists <= distance_threshold).sum()) / float(dists.size)
        
        return overlap_ratio
 

    def update_key_frame(self, pose, frame):
        
        # Perform ICP registration
        reg_p2p = o3d.pipelines.registration.registration_icp(
                    source = frame, 
                    target = self.map,
                    max_correspondence_distance = self.config["KF_CORRESPONDENCE_DISTANCE"],
                    estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                                relative_fitness=1e-6,
                                                                                relative_rmse=1e-6
                                                                                )                      
                )
        
        current_pose = np.dot(pose, reg_p2p.transformation)
    
        current_key_frame = frame.transform(reg_p2p.transformation)
        
        o3d.geometry.PointCloud.estimate_normals(current_key_frame, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.config["VOXEL_SIZE"] * self.config["NORMAL_FACTOR"], max_nn=30))
        
        self.key_pose = current_pose
        
        self.key_frame = cp.deepcopy(current_key_frame)
                
        return current_pose, current_key_frame


    def set_map(self, new_map):
        """
        Sets the current map to a new map.
        """
        self.map = cp.deepcopy(new_map)
        
    
    def get_key_frame(self):
        """
        Returns the current key frame.
        """
        return cp.deepcopy(self.key_frame)
    
    
    def set_key_frame_and_pose(self, new_key_frame, new_key_pose):
        """
        Sets the current key frame and its pose to new values.
        """
        self.key_frame = cp.deepcopy(new_key_frame)
        self.key_pose = new_key_pose