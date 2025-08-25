import open3d as o3d

from location_controller.utils import draw_registration_result

def retry_with_initial_alingment(self, map, pcd, mean_error):# If the registration error is worst than the mean error, we try to use RANSAC to find a better registration
        
        if ((mean_error * self.ERROR_FACTOR) < reg_p2p.inlier_rmse) or (reg_p2p.inlier_rmse == 0.0):
            
            map_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                map, 
                o3d.geometry.KDTreeSearchParamHybrid(radius=self.VOXEL_SIZE * self.NORMAL_FACTOR, max_nn=100))
            
            pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                pcd, 
                o3d.geometry.KDTreeSearchParamHybrid(radius=self.VOXEL_SIZE * self.NORMAL_FACTOR, max_nn=100))
            
            initial_guess = self.ransac_global_registration(pcd, map, pcd_fpfh, map_fpfh, voxel_size=self.VOXEL_SIZE, ratio=self.VOXEL_REDUCTION_RATIO)
            
            draw_registration_result(self.map, pcd, None)
            
            draw_registration_result(self.map, pcd, initial_guess.transformation)
            
            reg_p2p = o3d.pipelines.registration.registration_icp(
                            source = pcd, 
                            target = self.map,
                            init = initial_guess.transformation,
                            max_correspondence_distance = self.THRESHOLD,
                            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                                            relative_fitness=1e-6,
                                                                                            relative_rmse=1e-7
                                                                                        )
                    )
            
            draw_registration_result(self.map, pcd, reg_p2p.transformation)