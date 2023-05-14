
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Log paths
IMG_ZED_PATH = os.path.abspath(os.path.join(ROOT, "logs/img_ZED_cam.png"))
POINT_CLOUD_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud.txt"))
PLY_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud.ply"))
PLY_FROM_ROS_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_from_ros.ply"))
PLY_AFTER_TRANSFORM_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_after_transform.ply"))
PLY_AFTER_CLEAN_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_after_clean.ply"))

# Names of all blocks
BLOCK_NAMES = [ 
    'X1-Y1-Z2',
    'X1-Y2-Z1',
    'X1-Y2-Z2',
    'X1-Y2-Z2-CHAMFER',
    'X1-Y2-Z2-TWINFILLET',
    'X1-Y3-Z2',
    'X1-Y3-Z2-FILLET',
    'X1-Y4-Z1',
    'X1-Y4-Z2',
    'X2-Y2-Z2',
    'X2-Y2-Z2-FILLET'
]

# Mesh and point cloud paths of all blocks
MODEL = {
    'X1-Y1-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y1-Z2/mesh/X1-Y1-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y1-Z2/pointcloud_X1-Y1-Z2.ply'),
    },
    'X1-Y2-Z1': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z1/mesh/X1-Y2-Z1.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z1/pointcloud_X1-Y2-Z1.ply'),
    },
    'X1-Y2-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2/mesh/X1-Y2-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2/pointcloud_X1-Y2-Z2.ply'),
    },
    'X1-Y2-Z2-CHAMFER': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-CHAMFER/mesh/X1-Y2-Z2-CHAMFER.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-CHAMFER/pointcloud_X1-Y2-Z2-CHAMFER.ply'),
    },
    'X1-Y2-Z2-TWINFILLET': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-TWINFILLET/mesh/X1-Y2-Z2-TWINFILLET.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-TWINFILLET/pointcloud_X1-Y2-Z2-TWINFILLET.ply'),
    },
    'X1-Y3-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2/mesh/X1-Y3-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2/pointcloud_X1-Y3-Z2.ply'),
    },
    'X1-Y3-Z2-FILLET': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2-FILLET/mesh/X1-Y3-Z2-FILLET.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2-FILLET/pointcloud_X1-Y3-Z2-FILLET.ply'),
    },
    'X1-Y4-Z1': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z1/mesh/X1-Y4-Z1.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z1/pointcloud_X1-Y4-Z1.ply'),
    },
    'X1-Y4-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z2/mesh/X1-Y4-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z2/pointcloud_X1-Y4-Z2.ply'),
    },
    'X2-Y2-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2/mesh/X2-Y2-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2/pointcloud_X2-Y2-Z2.ply'),
    },
    'X2-Y2-Z2-FILLET': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2-FILLET/mesh/X2-Y2-Z2-FILLET.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2-FILLET/pointcloud_X2-Y2-Z2-FILLET.ply'),
    }
}

# Main
if __name__ == "__main__":
    print(PLY_FROM_ROS_PATH)