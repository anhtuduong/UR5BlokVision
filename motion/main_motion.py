
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
        self.ee_pose_sub = ros.Subscriber('/ur5/ee_pose', Pose, self.ee_pose_callback)

        # Ros service clients
        ros.wait_for_service('/ur5/move_joints')
        self.move_joints_srv = ros.ServiceProxy('/ur5/move_joints', MoveJoints)
        ros.wait_for_service('/ur5/move_to')
        self.move_to_srv = ros.ServiceProxy('/ur5/move_to', MoveTo)
        ros.wait_for_service('move_gripper')
        self.move_gripper_srv = ros.ServiceProxy('/ur5/move_gripper', generic_float)

        # Predefined poses
        self.q_pick = conf.robot_params[self.robot_name]['q_pick']
        self.q_middle = conf.robot_params[self.robot_name]['q_middle']
        self.q_place = conf.robot_params[self.robot_name]['q_place']

    def joint_states_callback(self, msg):
        """
        """
        for msg_idx in range(len(msg.name)):          
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]: 
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def ee_pose_callback(self, msg):
        """
        """
        self.ee_pose = msg

    def move_joints(self, q_des, text = ''):
        """
        """
        # Create a request object for MoveJoints
        req = MoveJointsRequest()
        req.q_des = q_des
        req.dt = self.dt
        req.v_des = self.v_des

        # Call the service
        log.debug_highlight(f'Start movement! {text}')
        res = self.move_joints_srv(req)

        if res.success:
            log.info(f'Movement succeeded! {text}')
        else:
            log.error(f'Movement failed! {text}')
        return res
        
    def move_to(self, pose_target, text = ''):
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
        log.debug_highlight(f'Start movement! {text}')
        res = self.move_to_srv(req)

        if res.success:
            log.info(f'Movement succeeded! {text}')
        else:
            log.error(f'Movement failed {text}')
        return res

    def move_gripper(self, diameter, text = ''):
        """
        """
        log.debug_highlight(f'Start gripper')
        # Create a request object
        request = generic_float._request_class()
        request.data = diameter

        # Send the request to the service
        res = self.move_gripper_srv(request)

        if res:
            log.info(f'Gripper command sent: {diameter}mm. {text}')
        else:
            log.error(f'Gripper command failed: {diameter}mm. {text}')

    

    def run(self):
        """
        """
        # # test moving to current perpendicular
        # pose_target = Pose()
        # pose_target.position.x = 0.6716437995358168
        # pose_target.position.y = 0.5476672561804214
        # pose_target.position.z = 1.1371617396564637
        # pose_target.orientation.x = 0.0
        # pose_target.orientation.y = 1.0
        # pose_target.orientation.z = 0.0
        # pose_target.orientation.w = 0.0

        # self.move_to(pose_target, 'Perpendicular to the table')
        # log.debug(f'q: {self.q}')

        # test moving to lego
        pick = Pose()
        pick.position.x = 0.0732
        pick.position.y = 0.4539
        pick.position.z = 0.945
        pick.orientation.x = 0.0
        pick.orientation.y = 1.0
        pick.orientation.z = 0.0
        pick.orientation.w = 0.0

        pick.position.z += 0.1
        self.move_to(pick, 'Above the object')

        pick.position.z -= 0.1
        self.move_to(pick, 'On the object')

        self.move_gripper(40, 'Grasp the object')

        pick.position.z += 0.1
        self.move_to(pick, 'Pick the object up')

        self.move_joints(self.q_place, 'Move to place above')

        # Get current ee pose
        place = self.ee_pose
        place.position.z = 0.95
        self.move_to(place, 'Move to place')

        self.move_gripper(70, 'Release the object')

        place.position.z += 0.1
        self.move_to(place, 'Move up')

        

# ----------- MAIN ----------- #
if __name__ == '__main__':
    motion = Motion()
    motion.run()