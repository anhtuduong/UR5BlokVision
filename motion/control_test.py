
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
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from utils.Logger import Logger as log

LOOP_RATE = 1000
# pub_des_jstate = rospy.Publisher('/ur5/joint_group_pos_controller/command', Float64MultiArray, queue_size=1)
pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)

def send_joint_state(q):
    loop_rate = ros.Rate(LOOP_RATE)
    joint_state_msg_robot = Float64MultiArray()
    joint_state_msg_robot.data = q[:6]
    # joint_state_msg_robot.data = np.append(joint_state_msg_robot.data, 3.49)
    # if grasp:
    #     data_to_append = [map_to_gripper_joints(45)] * 3
    #     joint_state_msg_robot.data = np.append(joint_state_msg_robot.data, data_to_append)
    # else:
    # data_to_append = [map_to_gripper_joints(100)] * 3
    # joint_state_msg_robot.data = np.append(joint_state_msg_robot.data, data_to_append)
    
    pub_des_jstate.publish(joint_state_msg_robot)
    log.debug(f"Sent joint state: {joint_state_msg_robot.data}")
    loop_rate.sleep()

def send_des_jstate(q_des, qd_des, tau_ffwd):
    # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
    msg = JointState()
    msg.position = q_des
    msg.velocity = qd_des
    msg.effort = tau_ffwd                
    pub_des_jstate.publish(msg)
    log.debug(f"Sent joint state: {msg}")

# Main
if __name__ == "__main__":
    ros.init_node('control_test')
    # send_joint_state(np.array([-0.32, -0.78, -2.56, -1.63, -1.57, 3.49]))
    position = np.array([-0.22, -0.68, -2.56, -1.73, -1.77, 2.49])
    velocity = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    effort = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    send_des_jstate(position, velocity, effort)