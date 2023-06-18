
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

# Ros msg and srv
from geometry_msgs.msg import Pose
from motion_ur5.srv import MoveJoints, MoveJointsRequest, MoveJointsResponse
from motion_ur5.srv import MoveTo, MoveToRequest, MoveToResponse

class Motion():
    """
    """

    def __init__(self):
        """
        """
        # Start motion node
        ros.init_node('motion_node', anonymous=True)

        # Init variables
        self.rate = ros.Rate(1 / conf.robot_params[self.robot.robot_name]['dt'])
        self.dt = conf.robot_params[self.robot.robot_name]['dt']
        self.real_robot = conf.robot_params[self.robot.robot_name]['real_robot']
        if self.real_robot:
            self.v_des = 0.2
        else:
            self.v_des = 0.6

        # Ros service clients
        ros.wait_for_service('/ur5/move_joints')
        self.move_joints = ros.ServiceProxy('/ur5/move_joints', MoveJoints)
        ros.wait_for_service('/ur5/move_to')
        self.move_to = ros.ServiceProxy('/ur5/move_to', MoveTo)
        

    

    def run(self):
        """
        """
        # test moving to position
        # position = np.array([0.30276, 0.60204, 0.89147])
        # rotation = np.array([-0.337, -2.2038, -10.481])
        pose_target = Pose()
        pose_target.position.x = 0.30276
        pose_target.position.y = 0.50204
        pose_target.position.z = 0.957
        pose_target.orientation.x = -0.616939
        pose_target.orientation.y = 0.785472
        pose_target.orientation.z = 0.0251763
        pose_target.orientation.w = 0.0422654

        # for pose in p.bridge_trajectory:
        #     p.move_joints(p.dt, p.v_des, pose, rate)

        # p.move_joints(p.dt, p.v_des, p.q_guess['pick'], rate)
        pose_target.position.z += 0.1
        self.robot.move_to(pose_target, self.dt, self.v_des, self.rate)
        pose_target.position.z -= 0.1
        self.robot.move_to(pose_target, self.dt, self.v_des, self.rate)
        # p.gripper_move(30)
        # p.move_joints(p.dt, p.v_des, p.q_guess['place'], rate)
        # p.gripper_move(60)

        # Create a request object for MoveTo
        req = MoveToRequest()
        

        self.rate.sleep()

# ----------- MAIN ----------- #
if __name__ == '__main__':
    pass