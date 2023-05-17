

# Reslove paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Constants
from vision.constants import *

SOURCE_PATH = MODEL['X2-Y2-Z2']['pointcloud_file']
TARGET_PATH = PLY_AFTER_CLEAN_PATH

# Import
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation
from vision.scripts.utils.Logger import Logger as log
from vision.scripts.utils.TimeExecution import TimeExecution
from vision.scripts.camera.PointCloud import PointCloud

def downsample_point_cloud(point_cloud):
    """
    @brief Downsample the point cloud
    @param point_cloud: point cloud
    @return downsampled_point_cloud (np.array): downsampled point cloud
    """

    # Downsample the point cloud
    downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size=0.001)
    log.info(f"Point cloud downsampled: {len(downsampled_point_cloud.points)} points")

    return downsampled_point_cloud

def compute_FPFH_features(point_cloud):
    """
    @brief Compute FPFH features
    @param point_cloud: point cloud
    @return fpfh_features (np.array): FPFH features
    """

    time_execution = TimeExecution()

    # Compute normals
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )

    # Compute FPFH features
    radius_feature = 0.2
    fpfh_features = o3d.pipelines.registration.compute_fpfh_feature(
        point_cloud,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=30)
    )
    log.debug(f"FPFH features computed in {time_execution.get_duration():.3f}s")

    return fpfh_features

def match_FPFH_features(source_cloud, target_cloud, fpfh_source, fpfh_target):
    """
    @brief Perform FPFH feature matching
    @param source_cloud: source point cloud
    @param target_cloud: target point cloud
    @param fpfh_source (np.array): FPFH features of the source point cloud
    @param fpfh_target (np.array): FPFH features of the target point cloud
    @return transformation_matrix (np.array): 4x4 transformation matrix
    """

    time_execution = TimeExecution()

    distance_threshold = 0.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_cloud, target_cloud, fpfh_source, fpfh_target,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.999)
    )

    transformation_matrix = result.transformation
    log.debug(f"FPFH features matched in {time_execution.get_duration():.3f}s")

    return transformation_matrix

def icp_alignment(source_cloud, target_cloud, transformation_matrix):
    """
    @brief Alignment using ICP
    @param source_cloud: source point cloud
    @param target_cloud: target point cloud
    @param transformation_matrix (np.array): 4x4 transformation matrix
    @return transformation_matrix (np.array): 4x4 transformation matrix
    """

    time_execution = TimeExecution()

    max_correspondence_distance = 0.5
    result_icp = o3d.pipelines.registration.registration_icp(
        source_cloud, target_cloud, max_correspondence_distance,
        transformation_matrix,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    transformation_matrix = result_icp.transformation
    log.debug(f"ICP alignment performed in {time_execution.get_duration():.3f}s")

    return transformation_matrix

def compute_6dof_transformation(transformation_matrix):
    """
    @brief Compute the 6DOF transformation parameters
    @param transformation_matrix_icp (np.array): 4x4 transformation matrix
    @return translation (np.array): 3x1 translation vector
    @return euler_angles (np.array): 3x1 Euler angles
    """

    translation = transformation_matrix[:3, 3]
    rotation_matrix = transformation_matrix[:3, :3]

    # Compute the Euler angles from the rotation matrix
    euler_angles = np.degrees(rotation_matrix_to_euler_angles(rotation_matrix))

    # Print the 6DOF transformation parameters
    log.info(f"Translation: {translation}")
    log.info(f"Euler angles: {euler_angles}")

    return translation, euler_angles

def rotation_matrix_to_euler_angles(rotation):
    """
    @brief Extract the Euler angles from the rotation matrix
    @param rotation (np.array): 3x3 rotation matrix
    @return euler_angles (np.array): 3x1 Euler angles
    """

    sy = np.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])

    if sy < 1e-6:
        # Singular case (cos(theta) = 0)
        x = np.arctan2(rotation[1, 2], rotation[1, 1])
        y = np.arctan2(-rotation[2, 0], sy)
        z = 0.0
    else:
        x = np.arctan2(rotation[2, 1], rotation[2, 2])
        y = np.arctan2(-rotation[2, 0], sy)
        z = np.arctan2(rotation[1, 0], rotation[0, 0])

    euler_angles = np.array([x, y, z])
    return euler_angles

def load_point_cloud(point_cloud):
    """
    @brief Load point clouds regard to PLY file or point cloud taken from PLY file
    @param point_cloud (str or list): path to PLY file or point cloud taken from PLY file
    @return point_cloud (open3d.geometry.PointCloud): point cloud
    """
    
    # If point_cloud is a string, it is a path to a PLY file
    if isinstance(point_cloud, str):
        point_cloud = o3d.io.read_point_cloud(point_cloud)
    
    # If point_cloud is a list, it is a point cloud taken from PLY file
    elif isinstance(point_cloud, list):
        point_cloud = np.array(point_cloud)
        point_cloud = o3d.geometry.PointCloud(point_cloud)

    else:
        raise TypeError("point_cloud must be a string of PLY path or a list")

    # Print the point cloud information
    log.info(f'Point cloud loaded: {len(point_cloud.points)} points')

    return point_cloud

# def visualize_point_clouds(point_clouds):
#     """
#     @brief Visualize the point clouds
#     @param point_clouds (list): list of point clouds
#     """

#     # Create a visualization window
#     vis = o3d.visualization.Visualizer()
#     vis.create_window()

#     # Add the point clouds to the visualization
#     for point_cloud in point_clouds:
#         vis.add_geometry(point_cloud)

#     # Add coordinate frame
#     frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6)
#     vis.add_geometry(frame)

#     # Set the camera viewpoint
#     vis.get_view_control().set_front([0, 0, -1])
#     vis.get_view_control().set_lookat([0, 0, 0])
#     vis.get_view_control().set_up([0, -1, 0])
#     vis.get_view_control().set_zoom(0.8)

#     # Run the visualization
#     vis.run()
#     vis.destroy_window()

# Convert open3d.geometry.PointCloud to list of point_cloud
def convert_open3d_pointcloud_to_list(point_cloud):
    """
    @brief Convert open3d.geometry.PointCloud to list of point_cloud
    @param point_cloud (open3d.geometry.PointCloud): point cloud
    @return point_cloud (list): list of point_cloud
    """
    point_cloud = np.array(point_cloud.points)
    return point_cloud


# Main
if __name__ == "__main__":

    # Load the point clouds
    source_cloud = load_point_cloud(SOURCE_PATH)
    target_cloud = load_point_cloud(TARGET_PATH)

    source_cloud.paint_uniform_color([1, 0, 0])  # Set color of full model point cloud to red
    target_cloud.paint_uniform_color([0, 0, 1])  # Set color of fragment point cloud to blue

    # Downsample the point clouds
    # source_cloud = downsample_point_cloud(source_cloud)
    # target_cloud = downsample_point_cloud(target_cloud)

    # Compute the FPFH features
    fpfh_source = compute_FPFH_features(source_cloud)
    fpfh_target = compute_FPFH_features(target_cloud)

    # Perform FPFH feature matching
    transformation_matrix = match_FPFH_features(source_cloud, target_cloud, fpfh_source, fpfh_target)

    # Alignment using ICP
    transformation_matrix = icp_alignment(source_cloud, target_cloud, transformation_matrix)

    # Compute the 6DOF transformation parameters
    translation, euler_angles = compute_6dof_transformation(transformation_matrix)

    # Transform the source point cloud
    transformed_cloud = source_cloud
    transformed_cloud.transform(transformation_matrix)

    # Convert open3d.geometry.PointCloud to list of point_cloud
    source_cloud = convert_open3d_pointcloud_to_list(source_cloud)
    target_cloud = convert_open3d_pointcloud_to_list(target_cloud)

    # Visualize the point clouds
    PointCloud.visualize_pointcloud([source_cloud, target_cloud])