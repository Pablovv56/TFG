import os
import math
import random
import yaml
import copy as cp
import numpy as np
import open3d as o3d

from location_controller.utils import ransac_global_registration, preprocess_point_cloud, draw_registration_result

# Global parameters
YAML_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../src/config/general_config.yaml"))


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
        
    
    def is_new_key_frame(self, original_new_frame):
        """
        Determine if the new frame is a new key frame based on the distance to the current key frame.
        """

        new_frame = preprocess_point_cloud(original_new_frame, self.key_pose)
        
        initial_alingment = ransac_global_registration(new_frame, self.key_frame)
        
        new_frame_corrected = new_frame.transform(initial_alingment.transformation)
        
        current_pose_corrected = np.dot(initial_alingment.transformation, self.key_pose)
        
        #draw_registration_result(new_frame_corrected, self.key_frame)
        
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source = new_frame_corrected, 
            target = self.key_frame,
            max_correspondence_distance = 0.5 * self.config["VOXEL_SIZE"],
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                        relative_fitness=1e-6,
                                                                        relative_rmse=1e-6
                                                                        )                      
        )
        
        new_frame_alinged = new_frame.transform(reg_p2p.transformation)
        
        current_pose_alinged = np.dot(reg_p2p.transformation, current_pose_corrected)
        
        #draw_registration_result(new_frame_alinged, self.key_frame)
        
        if (reg_p2p.inlier_rmse < self.config["RMSE_THRESHOLD"]):
                
            # Calculate the distance between the new pose and the key frame
            distance, angle = self.pose_distance(current_pose_alinged)
            
            # Calculate the overlap ratio with the key frame
            overlap_ratio = self.compute_cloud_overlap_ratio(new_frame_alinged)
            
            if ((distance > self.config["DISTANCE_THRESHOLD"]) or 
                (angle > self.config["ANGLE_THRESHOLD"]) or 
                (overlap_ratio < self.config["OVERLAP_RATIO_THRESHOLD"])):
                
                new_pose_final, new_frame_final = self.update_key_frame(current_pose_alinged, new_frame_alinged)
                
                return True, new_pose_final, new_frame_final
          
        print("\033[33mDiscarded key frame\033[0m" + str(reg_p2p.fitness) + " / RMSE: " + str(reg_p2p.inlier_rmse))
          
        return False, None, None
    
         
    def pose_distance(self, pose2):
        """
        Calcula distancia traslacional (m) y rotacional (deg) entre dos poses 4x4.
        T1, T2: np.ndarray 4x4
        """
        
        T1 = np.array(self.key_pose, dtype=np.float64)
        T2 = np.array(pose2, dtype=np.float64)
        
        assert T1.shape == (4, 4) and T2.shape == (4, 4), "Las poses deben ser 4x4"
        
        # Traslación
        p1 = T1[0:3, 3]
        p2 = T2[0:3, 3]
        trans_dist = np.linalg.norm(p2 - p1)
        
        # Rotación relativa
        R1 = T1[0:3, 0:3]
        R2 = T2[0:3, 0:3]
        R_rel = R1.T @ R2  # rotación que lleva R1 a R2
        
        # Asegurar rango numérico
        trace_val = np.trace(R_rel)
        trace_val = np.clip(trace_val, -1.0, 3.0)
        
        # Ángulo (en radianes → grados)
        rot_angle_rad = math.acos((trace_val - 1.0) / 2.0)
        rot_angle_deg = math.degrees(rot_angle_rad)
        
        return trans_dist, rot_angle_deg
   
    
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
        
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source = frame, 
            target = self.map,
            max_correspondence_distance = 0.5 * self.config["VOXEL_SIZE"],
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                        relative_fitness=1e-6,
                                                                        relative_rmse=1e-6
                                                                        )                      
        )
        
        current_pose_alinged = np.dot(reg_p2p.transformation, pose)
        
        self.key_pose = current_pose_alinged
        
        final_new_key_frame = frame.transform(reg_p2p.transformation)
        
        self.key_frame = cp.deepcopy(final_new_key_frame)
        
        draw_registration_result(final_new_key_frame, self.map)
        
        return current_pose_alinged, final_new_key_frame


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