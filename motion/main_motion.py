
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
import rospy as ros
import locosim.robot_control.base_controllers.params as conf
import numpy as np
from motion.utils import *
from utils_ur5.Logger import Logger as log

# Ros msg and srv
from geometry_msgs.msg import Pose
from ros_impedance_controller.srv import MoveJoints, MoveJointsRequest, MoveJointsResponse
from ros_impedance_controller.srv import MoveTo, MoveToRequest, MoveToResponse
from ros_impedance_controller.srv import generic_float
from sensor_msgs.msg import JointState

class Motion():
    """
    """

    def __init__(self):
        """
        """
        # Start motion node
        ros.init_node('motion_node', anonymous=True)

        # Init variables
        self.robot_name = 'ur5'
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.dt = conf.robot_params[self.robot_name]['dt']
        self.rate = ros.Rate(1 / self.dt)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        if self.real_robot:
            self.v_des = 0.2
        else:
            self.v_des = 0.6

        self.q = np.zeros(len(self.joint_names))
        self.qd = np.zeros(len(self.joint_names))
        self.tau = np.zeros(len(self.joint_names))


        # Ros subscribers
        self.joint_states_sub = ros.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # Ros service clients
        ros.wait_for_service('/ur5/move_joints')
        self.move_joints_srv = ros.ServiceProxy('/ur5/move_joints', MoveJoints)
        ros.wait_for_service('/ur5/move_to')
        self.move_to_srv = ros.ServiceProxy('/ur5/move_to', MoveTo)
        ros.wait_for_service('move_gripper')
        self.move_gripper_srv = ros.ServiceProxy('/ur5/move_gripper', generic_float)

    def joint_states_callback(self, msg):
        """
        """
        for msg_idx in range(len(msg.name)):          
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]: 
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def move_joints(self, q_des):
        """
        """
        # Create a request object for MoveJoints
        req = MoveJointsRequest()
        req.q_des = q_des
        req.dt = self.dt
        req.v_des = self.v_des

        # Call the service
        log.debug_highlight('Start movement')
        res = self.move_joints_srv(req)

        if res.success:
            log.info(f'Movement succeeded')
        else:
            log.error(f'Movement failed')
        return res
        
    def move_to(self, pose_target):
        """
        """
        if isinstance(pose_target, Pose):
            pose_target = Pose_to_list(pose_target)
        elif isinstance(pose_target, list):
            pass
        else:
            raise TypeError('pose_target must be a Pose or a list')
        
        # Create a request object for MoveTo
        req = MoveToRequest()
        req.pose_target = pose_target
        req.dt = self.dt
        req.v_des = self.v_des

        # Call the service
        log.debug_highlight('Start movement')
        res = self.move_to_srv(req)

        if res.success:
            log.info(f'Movement succeeded')
        else:
            log.error(f'Movement failed')
        return res

    def move_gripper(self, diameter):
        """
        """
        log.debug_highlight(f'Start gripper')
        # Create a request object
        request = generic_float._request_class()
        request.data = diameter

        # Send the request to the service
        res = self.move_gripper_srv(request)

        if res:
            log.info(f'Gripper command sent: {diameter}')
        else:
            log.error(f'Gripper command failed: {diameter}')

    

    def run(self):
        """
        """
        # test moving to position
        # position = np.array([0.30276, 0.60204, 0.89147])
        # rotation = np.array([-0.337, -2.2038, -10.481])
        pose_target = Pose()
        pose_target.position.x = 0.30276
        pose_target.position.y = 0.50204
        pose_target.position.z = 0.95
        pose_target.orientation.x = -0.616939
        pose_target.orientation.y = 0.785472
        pose_target.orientation.z = 0.0251763
        pose_target.orientation.w = 0.0422654

        # for pose in p.bridge_trajectory:
        #     p.move_joints(p.dt, p.v_des, pose, rate)

        # p.move_joints(p.dt, p.v_des, p.q_guess['pick'], rate)
        # pose_target.position.z += 0.1
        # self.move_to(pose_target)

        # pose_target.position.z -= 0.1
        # self.move_to(pose_target)

        self.move_gripper(100)

        # pose_target.position.z += 0.1
        # self.move_to(pose_target)

        self.move_gripper(30)
        

        self.rate.sleep()

# ----------- MAIN ----------- #
if __name__ == '__main__':
    motion = Motion()
    motion.run()