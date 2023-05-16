

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

# Import
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation

def rotation_matrix_to_euler_angles(rotation):
    # Extract the Euler angles from the rotation matrix
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

# Load the full 3D model point cloud
full_model_cloud = o3d.io.read_point_cloud(MODEL['X1-Y3-Z2-FILLET']['pointcloud_file'])

# Load the fragment point cloud
fragment_cloud = o3d.io.read_point_cloud(PLY_AFTER_CLEAN_PATH)

# Compute normals for the full 3D model
full_model_cloud.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)

# Compute normals for the fragment point cloud
fragment_cloud.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)

# Compute FPFH features for the full 3D model
radius_feature = 0.2
fpfh_full_model = o3d.pipelines.registration.compute_fpfh_feature(
    full_model_cloud,
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=30)
)

# Compute FPFH features for the fragment point cloud
fpfh_fragment = o3d.pipelines.registration.compute_fpfh_feature(
    fragment_cloud,
    o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=30)
)

# Perform feature matching
distance_threshold = 0.05
result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    full_model_cloud, fragment_cloud, fpfh_full_model, fpfh_fragment,
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

# Refine the alignment using ICP
max_correspondence_distance = 0.05
result_icp = o3d.pipelines.registration.registration_icp(
    full_model_cloud, fragment_cloud, max_correspondence_distance,
    transformation_matrix,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

transformation_matrix_icp = result_icp.transformation

# Compute the 6DOF transformation parameters
translation = transformation_matrix_icp[:3, 3]
rotation_matrix = transformation_matrix_icp[:3, :3]

# Compute the Euler angles from the rotation matrix
euler_angles = np.degrees(rotation_matrix_to_euler_angles(rotation_matrix))

# Print the 6DOF transformation parameters
print("Translation:", translation)
print("Rotation (degrees):", euler_angles)

# Visualize the point clouds
full_model_cloud.paint_uniform_color([1, 0, 0])  # Set color of full model point cloud to red
fragment_cloud.paint_uniform_color([0, 0, 1])  # Set color of fragment point cloud to blue

# Transform the full model point cloud using the computed transformation matrix
full_model_cloud_transformed = full_model_cloud.transform(transformation_matrix_icp)

# Create a visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the point clouds to the visualization
vis.add_geometry(full_model_cloud_transformed)
vis.add_geometry(fragment_cloud)

# Set the camera viewpoint
vis.get_view_control().set_front([0, 0, -1])
vis.get_view_control().set_lookat([0, 0, 0])
vis.get_view_control().set_up([0, -1, 0])
vis.get_view_control().set_zoom(0.8)

# Run the visualization
vis.run()
vis.destroy_window()