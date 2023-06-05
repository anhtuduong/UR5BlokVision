"""!
@package tests.test_kinematics
@file tests/test_kinematics.py
@brief This module contains the unit tests for the kinematics module.
@author Anh Tu Duong
@date 2023-05-04
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
import unittest
import numpy as np
import rospkg

from motion.inv_kinematics import robotKinematics
from locosim.robot_control.base_controllers.utils.common_functions import getRobotModel
from locosim.robot_control.base_controllers.utils.math_tools import Math

class TestKinematics(unittest.TestCase):
    """
    @brief This class contains the unit tests for the kinematics module.
    """

    def __init__(self, methodName: str = "runTest") -> None:
        super().__init__(methodName)
        robot_name = "ur5"
        xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/ur5.urdf.xacro'
        self.robot = getRobotModel(robot_name, generate_urdf=True, xacro_path=xacro_path)
        self.ee_frame = 'tool0'
        self.math_utils = Math()

    def test_inv_kinematics_pos_rot(self):
        """
        @brief This method tests the inverse kinematics method
            robotKinematics.endeffectorInverseKinematicsLineSearch()
            with position and orientation.
        """
        #if you use a reasonable guess it will converge quicker
        q_guess = np.array([0.4, -1.1,  1.0, -6.,  1,  1.0])
        #q_guess = np.zeros(6)

        ee_pos_des = np.array([0.30276, 0.60204, 0.89147])
        #This is the result it should get close 
        # qtest = np.array([ 0.76689, -1.17447,  1.08174, -6.18935,  2.33769,  1.57316])

        # #orient reference
        # rpy_des = np.array([0, -1.57 , 0])
        # # compute rotation matrix representing the desired orientation from Euler Angles
        # w_R_e_des = self.math_utils.eul2Rot(rpy_des)   
        # print(w_R_e_des)

        kin = robotKinematics(self.robot, self.ee_frame)

        q, ik_success, out_of_workspace = kin.endeffectorInverseKinematicsLineSearch(ee_pos_des,
                                                                                     self.ee_frame,
                                                                                     q_guess,
                                                                                     verbose = True,
                                                                                     wrap = True)
        print('Result is: ' , q)
