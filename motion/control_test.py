
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
import rospy as ros
import numpy as np
from locosim.robot_control.base_controllers.components.controller_manager import ControllerManager
import locosim.robot_control.base_controllers.params as conf
from utilities.Logger import Logger as log

robot_name = 'ur5'

ros.init_node('control_test', anonymous=True)
controller_manager = ControllerManager(conf.robot_params[robot_name])
controller_manager.initPublishers(robot_name)


