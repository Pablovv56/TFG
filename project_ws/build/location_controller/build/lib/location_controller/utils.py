import struct
import open3d as o3d
import copy as cp
import numpy as np
    
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

def draw_registration_result(source, target, transformation):
    
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
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                    zoom=0.4459,
                                    front=[0.9288, -0.2951, -0.2242],
                                    lookat=[1.6784, 2.0612, 1.4451],
                                    up=[-0.3402, -0.9189, -0.1996])