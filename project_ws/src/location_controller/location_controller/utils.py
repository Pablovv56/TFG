import os
import yaml
import rclpy
import struct
import math
import open3d as o3d
import copy as cp
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
import tf_transformations as tf
from geometry_msgs.msg import PoseStamped, TransformStamped
   
# GLOBAL PARAMETERS
YAML_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../src/config/general_config.yaml"))

with open(YAML_PATH, "r") as file:
    CONFIG = yaml.safe_load(file)
    
def pointcloud2_to_open3d(data_msg):
    
    """
    Converts a ROS PointCloud2 message to an Open3D PointCloud object.
    Args:
        data_msg (sensor_msgs.msg.PointCloud2): The PointCloud2 message containing the point cloud data.
    Returns:
        open3d.geometry.PointCloud: The converted Open3D PointCloud object.
    Notes:
        - The function extracts the 3D points from the PointCloud2 message and assigns them to the Open3D PointCloud.
        - If RGB data is available in the PointCloud2 message, it is extracted and assigned as colors to the Open3D PointCloud.
        - The RGB values are normalized to the range [0, 1] for compatibility with Open3D.
    Raises:
        struct.error: If there is an issue unpacking the binary data from the PointCloud2 message.
        ValueError: If the data cannot be reshaped properly based on the point step.
    Example:
        # Assuming `data_msg` is a valid PointCloud2 message:
        pcd = pointcloud2_to_open3d(data_msg)
    """
    
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


def open3d_to_pointcloud2(o3d_cloud, frame_id="map"):
    
    """
    Converts an Open3D point cloud to a ROS2 PointCloud2 message.
    Args:
        o3d_cloud (open3d.geometry.PointCloud): The input Open3D point cloud object.
        frame_id (str, optional): The frame ID to associate with the PointCloud2 message. 
                                  Defaults to "map".
    Returns:
        sensor_msgs.msg.PointCloud2: A ROS2 PointCloud2 message containing the point cloud data.
    Notes:
        - The function assumes the input Open3D point cloud contains 3D points.
        - The PointCloud2 message is created with fields for x, y, and z coordinates.
        - The `is_dense` field is set to True, indicating no invalid points in the cloud.
    """
    
    # Extract points as Nx3 numpy array
    points = np.asarray(o3d_cloud.points)
    # Optionally, extract colors or other fields

    # Create PointCloud2 message
    msg = sensor_msgs.msg.PointCloud2()
    msg.header = std_msgs.msg.Header()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.header.frame_id = frame_id

    msg.height = 1
    msg.width = points.shape[0]
    msg.is_dense = True
    msg.is_bigendian = False

    # Define fields (x, y, z)
    msg.fields = [
        sensor_msgs.msg.PointField(name='x', offset=0, datatype=7, count=1),
        sensor_msgs.msg.PointField(name='y', offset=4, datatype=7, count=1),
        sensor_msgs.msg.PointField(name='z', offset=8, datatype=7, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
    msg.data = np.asarray(points, np.float32).tobytes()

    return msg


def matrix_to_pose_stamped(matrix, index = 0, frame_id="map"):
    
    """
    Converts a transformation matrix into a ROS PoseStamped message.
    Args:
        matrix (numpy.ndarray): A 4x4 transformation matrix representing the pose.
        frame_id (str, optional): The reference frame ID for the PoseStamped message. 
                                    Defaults to "map".
    Returns:
        PoseStamped: A ROS PoseStamped message containing the position and orientation 
                        extracted from the transformation matrix.
    Notes:
        - The translation component is extracted from the matrix and assigned to the 
            position field of the PoseStamped message.
        - The rotation component is extracted as a quaternion and assigned to the 
            orientation field of the PoseStamped message.
        - The header of the PoseStamped message includes the provided frame_id and 
            a timestamp based on the `self.index` attribute.
    """
    
    # Extraer la traslación
    translation = tf.translation_from_matrix(matrix)
    
    # Extraer la rotación en forma de cuaternión
    quaternion = tf.quaternion_from_matrix(matrix)

    # Crear el mensaje PoseStamped
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp.sec = index
    
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


def draw_registration_result(source, target, transformation = None):
    
    """
    Visualizes the registration result of two 3D point clouds after applying a transformation.
    This function takes two 3D point clouds (`source` and `target`) and a transformation matrix.
    It applies the transformation to the source point cloud, assigns uniform colors to both
    point clouds for differentiation, and visualizes them using Open3D's visualization tools.
    Args:
        source (open3d.geometry.PointCloud): The source point cloud to be transformed.
        target (open3d.geometry.PointCloud): The target point cloud to be used as a reference.
        transformation (numpy.ndarray): A 4x4 transformation matrix to be applied to the source point cloud.
    Returns:
        None: This function does not return any value. It displays a visualization window.
    """

    source_temp = cp.deepcopy(source)
    target_temp = cp.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    if transformation is not None:
        target_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                    zoom=0.4459,
                                    front=[0.9288, -0.2951, -0.2242],
                                    lookat=[1.6784, 2.0612, 1.4451],
                                    up=[-0.3402, -0.9189, -0.1996])
    
    
def preprocess_point_cloud(msg, downsample = True, clean_pcd = True, transformation = None,  voxel_size = CONFIG["VOXEL_SIZE"], normal_factor = CONFIG["NORMAL_FACTOR"]):
    
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
    
    # Copy of the actual step in open3d type
    pcd = pointcloud2_to_open3d(msg)
    
    if downsample:
        # Downsample the point cloud using voxel grid filtering
        pcd = pcd.voxel_down_sample(voxel_size)
    
    if clean_pcd:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
    if transformation is not None:
        # Move the map to the position from where it was seen
        pcd = pcd.transform(transformation)
    
    # Radius for normal estimation
    radius_normal = voxel_size * normal_factor
    
    # Estimate normal with radius as search criteria
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    return pcd


def crop_point_cloud(pcd, pose=(0,0,0), voxel_size = CONFIG["VOXEL_SIZE"], normal_factor = CONFIG["NORMAL_FACTOR"],
                     x_min=CONFIG["KF_FUSION_CROP_SIZE_X_MIN"], x_max=CONFIG["KF_FUSION_CROP_SIZE_X_MAX"],
                     y_min=CONFIG["KF_FUSION_CROP_SIZE_Y_MIN"], y_max=CONFIG["KF_FUSION_CROP_SIZE_Y_MAX"],
                     z_min=CONFIG["KF_FUSION_CROP_SIZE_Z_MIN"], z_max=CONFIG["KF_FUSION_CROP_SIZE_Z_MAX"]):
    """
    Recorta la nube de puntos en un cubo/caja definido alrededor de un centro.
    
    pcd: o3d.geometry.PointCloud
    center: tuple (cx, cy, cz) en las mismas coordenadas que la nube
    x_min, x_max, ...: límites de recorte RELATIVOS al centro. Usa np.inf para no recortar en ese eje.
    
    Devuelve: nueva PointCloud recortada
    """
    center = pose[:3, 3]  # extraer traslación (x, y, z)
    pts = np.asarray(pcd.points)
    
    if pts.shape[0] == 0:
        return o3d.geometry.PointCloud()  # nube vacía
    
    # Trasladar puntos a coordenadas relativas al centro
    rel_pts = pts - center[np.newaxis, :]
    
    mask = ((rel_pts[:,0] >= x_min) & (rel_pts[:,0] <= x_max) &
            (rel_pts[:,1] >= y_min) & (rel_pts[:,1] <= y_max) &
            (rel_pts[:,2] >= z_min) & (rel_pts[:,2] <= z_max))
    
    cropped_pcd = o3d.geometry.PointCloud()
    cropped_pcd.points = o3d.utility.Vector3dVector(pts[mask])
    
    # Mantener colores si existen
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        cropped_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
        # Radius for normal estimation
    radius_normal = voxel_size * normal_factor
    
        # Estimate normal with radius as search criteria
    cropped_pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    return cropped_pcd


def pose_distance(pose1, pose2):
        """
        Calcula distancia traslacional (m) y rotacional (deg) entre dos poses 4x4.
        T1, T2: np.ndarray 4x4
        """
        
        T1 = np.array(pose1, dtype=np.float64)
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


def compute_cloud_novelty_ratio(new_frame, key_frame, distance_threshold=0.2, max_samples=2000):
    """
    Calcula el grado de solapamiento entre dos nubes de puntos.

    new_frame: o3d.geometry.PointCloud
    key_frame: o3d.geometry.PointCloud
    distance_threshold: distancia máxima para considerar que un punto tiene correspondencia
    max_samples: número máximo de puntos a muestrear (para acelerar el cálculo)
    
    Return:
        overlap_ratio: float en [0,1]
    """
    pts_new = np.asarray(new_frame.points)
    pts_key = np.asarray(key_frame.points)

    n_src = pts_new.shape[0]
    if n_src == 0 or pts_key.shape[0] == 0:
        return 0.0

    # Muestreo para acelerar
    if n_src > max_samples:
        idx = np.random.choice(n_src, max_samples, replace=False)
        pts_new = pts_new[idx]

    sampled_src = o3d.geometry.PointCloud()
    sampled_src.points = o3d.utility.Vector3dVector(pts_new)

    # Distancia al punto más cercano en key_frame
    dists = np.asarray(sampled_src.compute_point_cloud_distance(key_frame))

    # Porcentaje de puntos con distancia menor a threshold
    overlap_ratio = 1 - float((dists <= distance_threshold).sum()) / float(dists.size)
    return overlap_ratio
 