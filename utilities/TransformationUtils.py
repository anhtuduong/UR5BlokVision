
# Reslove paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path to ROOT

# Import
import numpy as np
from utilities.Logger import Logger as log

class TransformationUtils:
    """
    @brief Transformation utilities
    """

    def compute_6DoF(transformation_matrix):
        """
        @brief Compute the 6DOF transformation parameters
        @param transformation_matrix_icp (np.array): 4x4 transformation matrix
        @return translation (np.array): 3x1 translation vector
        @return euler_angles (np.array): 3x1 Euler angles
        """

        translation = transformation_matrix[:3, 3]
        rotation_matrix = transformation_matrix[:3, :3]

        # Compute the Euler angles from the rotation matrix
        euler_angles = np.degrees(TransformationUtils.rotation_matrix_to_euler_angles(rotation_matrix))

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