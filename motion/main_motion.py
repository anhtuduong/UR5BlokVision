
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
from motion.utils import *
from utils_ur5.Logger import Logger as log

# Ros msg and srv
from geometry_msgs.msg import Pose
from ros_impedance_controller.srv import MoveJoints, MoveJointsRequest, MoveJointsResponse
from ros_impedance_controller.srv import MoveTo, MoveToRequest, MoveToResponse
from ros_impedance_controller.srv import generic_float

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
        self.dt = conf.robot_params[self.robot_name]['dt']
        self.rate = ros.Rate(1 / self.dt)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        if self.real_robot:
            self.v_des = 0.2
        else:
            self.v_des = 0.6

        # Ros service clients
        ros.wait_for_service('/ur5/move_joints')
        self.move_joints_srv = ros.ServiceProxy('/ur5/move_joints', MoveJoints)
        ros.wait_for_service('/ur5/move_to')
        self.move_to_srv = ros.ServiceProxy('/ur5/move_to', MoveTo)
        ros.wait_for_service('move_gripper')
        self.move_gripper_srv = ros.ServiceProxy('move_gripper', generic_float)
        
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

    def gripper_move(self, diameter):
        """
        """
        log.debug_highlight(f'Start gripper, diameter = {diameter}')
        # Create a request object
        request = generic_float._request_class()
        request.data = diameter

        # Send the request to the service
        response = self.move_gripper_srv(request)

        log.debug(f'response = {response}')
    

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
        pose_target.position.z += 0.1
        self.move_to(pose_target)

        pose_target.position.z -= 0.1
        self.move_to(pose_target)

        self.move_gripper_srv(30)
        

        self.rate.sleep()

# ----------- MAIN ----------- #
if __name__ == '__main__':
    motion = Motion()
    motion.run()