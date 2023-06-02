
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
import rospy
from kinematics import *
from std_msgs.msg import Float64MultiArray, Int32
from motion.msg import pos
from ros_impedance_controller.srv import generic_float
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils.Logger import Logger as log

# ----------- DEFINES ----------- #
# Loop rate of the node
LOOP_RATE = 1000
# Number of joints
JOINTS = 9

# ----------- GLOBAL VARIABLES ----------- #
# Position of the end-effector
pose = Pose()
# Class of the block
class_id = 0
# Flag to check if it has to grasp
grasp = 0
# Publisher for the desired joint state
pub_des_jstate = None
# Publisher for the ack
ack_pos = None
# Subscriber for the position msg
sub_pos = None
# Service call for the gripper
client = None
# Flag to check if it is in simulation
real_robot = 0
# Initial joint configuration
TH0 = np.zeros(6)
# Flag to check if it is the first time that the node is called
first = True
maxT = 6


# ----------- FUNCTION PROTOTYPES ----------- #

def map_to_gripper_joints(diameter):
    alpha = (diameter - 22) / (130 - 22) * (-np.pi) + np.pi
    return alpha

def compute_orientation_error_w(w_R_e, w_R_d):
    e_R_d = np.dot(w_R_e.T, w_R_d)
    errorW = np.zeros(3)

    cos_dtheta = (e_R_d[0, 0] + e_R_d[1, 1] + e_R_d[2, 2] - 1) / 2
    axis = np.zeros(3)
    aux = np.array([[e_R_d[2, 1], -e_R_d[1, 2]],
                    [e_R_d[0, 2], -e_R_d[2, 0]],
                    [e_R_d[1, 0], -e_R_d[0, 1]]])
    sin_dtheta = (np.sum(aux**2) * 0.5) ** 0.5

    dtheta = np.arctan2(sin_dtheta, cos_dtheta)
    if dtheta == 0:
        errorW = np.zeros(3)
    else:
        axis = 1 / (2 * sin_dtheta) * np.array([e_R_d[2, 1] - e_R_d[1, 2],
                                                e_R_d[0, 2] - e_R_d[2, 0],
                                                e_R_d[1, 0] - e_R_d[0, 1]])
        errorW = np.dot(w_R_e, dtheta * axis)

    return errorW

def inv_diff_kinematic_control_complete(q, xe, xd, vd, w_R_e, rot_end):
    k = 1e-5  # damping coefficient
    w_R_d = eul2rotm(rot_end)
    error_o = compute_orientation_error_w(w_R_e, w_R_d)
    J = jacobian(q)  # get the jacobian matrix
    ve = np.zeros(6)

    kp = np.identity(3) * 5  # position gain
    kphi = np.identity(3) * 30  # orientation gain

    if np.linalg.norm(error_o) > 1:
        error_o = 0.1 * error_o / np.linalg.norm(error_o)

    ve[:3] = vd + kp.dot(xd - xe)
    ve[3:] = kphi.dot(error_o)

    qdot = np.linalg.inv(J + np.identity(6) * k).dot(ve)

    for i in range(6):
        if qdot[i] > np.pi:
            qdot[i] = 1.5
        elif qdot[i] < -np.pi:
            qdot[i] = -1.5

    return qdot

def pd(t, pos_end, xe0):
    t_norm = t / maxT
    if t_norm > 1:
        return pos_end
    else:
        return t_norm * pos_end + (1 - t_norm) * xe0

def inv_diff_kinematic_control_sim_complete(pos_end, rot_end, dt):
    global first, TH0

    now = frame()   # current frame
    start = frame()   # start frame
    if first:
        TH0 = np.array([-0.32, -0.78, -2.56, -1.63, -1.57, 3.49])  # initial joint configuration
    first = False
    start = forward_kinematics(TH0)

    qk = TH0.copy()  # set the initial joint config
    qk1 = np.zeros(6)  # joint config

    kp = np.identity(3) * 5  # position gain
    kphi = np.identity(3) * 30  # orientation gain

    joint_velocity_coeff = np.zeros(6)  # joint velocities coefficients

    vd = np.zeros(3)  # desired linear velocity

    # loop
    for i in np.arange(dt, maxT + dt, dt):
        now = forward_kinematics(qk)  # get the current frame

        vd = (pd(i, pos_end, start.xyz) - pd(i - dt, pos_end, start.xyz)) / dt  # desired linear velocity

        # coefficient
        joint_velocity_coeff = inv_diff_kinematic_control_complete(qk, now.xyz, pd(i, pos_end, start.xyz), vd, now.rot, rot_end)

        # euler integration
        qk1 = qk + joint_velocity_coeff * dt
        qk = qk1
        send_joint_state(qk)

    TH0 = qk
    log.debug(f"Final joint configuration: {TH0}")

def pos_callback(msg):
    global pose, class_id
    pose.position = np.array([msg.x, msg.y, msg.z])
    pose.orientation = np.array([msg.roll, msg.pitch, msg.yaw])
    class_id = msg.class_id

    move()


def send_joint_state(q):
    loop_rate = rospy.Rate(LOOP_RATE)
    joint_state_msg_robot = Float64MultiArray()
    joint_state_msg_robot.data = q[:5]
    joint_state_msg_robot.data = np.append(joint_state_msg_robot.data, 3.49)

    # if not real_robot:
    #     if grasp:
    #         finger = grasp_in_sim(45)
    #         joint_state_msg_robot.data = np.append(joint_state_msg_robot.data, finger)
    #     else:
    #         finger = grasp_in_sim(100)
    #         joint_state_msg_robot.data = np.append(joint_state_msg_robot.data, finger)
    
    pub_des_jstate.publish(joint_state_msg_robot)
    log.debug(f"Sent joint state: {joint_state_msg_robot.data}")
    loop_rate.sleep()

def move():
    loop_rate = rospy.Rate(LOOP_RATE)
    dt = 0.001  # time step
    target = np.zeros(3)
    target[:2] = pose.position[:2]
    target[2] = 0.6
    inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)

    target = pose.position
    inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)

    grasp = 1
    if real_robot:
        graspit()
    else:
        send_joint_state(TH0)
    for i in range(50):
        loop_rate.sleep()

    target = np.array([0, -0.4, 0.6])
    inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)

    switch_target = {
        0: np.array([0.4, 0, 0.82]),
        1: np.array([0.4, -0.05, 0.82]),
        2: np.array([0.4, -0.1, 0.82]),
        3: np.array([0.4, -0.15, 0.82]),
        4: np.array([0.4, -0.20, 0.82]),
        5: np.array([0.4, -0.25, 0.82]),
        6: np.array([0.4, -0.30, 0.82]),
        7: np.array([0.4, -0.35, 0.82]),
        8: np.array([0.4, -0.40, 0.82]),
        9: np.array([0.3, -0.40, 0.82]),
        10: np.array([0.3, -0.35, 0.82]),
    }

    target = switch_target.get(class_id, np.zeros(3))
    inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)

    grasp = 0
    if real_robot:
        graspit()
    else:
        send_joint_state(TH0)
    for i in range(50):
        loop_rate.sleep()

    target[2] = 0.65
    inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)
    send_joint_state(TH0)

    target = np.array([0, -0.4, 0.6])
    inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)

    starting_position()
    # ack()

def starting_position():
    dt = 0.001  # time step
    target = np.array([-0.4, -0.4, 0.6])
    log.debug(f"starting_position {target}")
    inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)

# def ack():
#     loop_rate = rospy.Rate(LOOP_RATE)
#     ack = Int32()
#     ack.data = 1
#     for i in range(40):
#         loop_rate.sleep()
#     ack_pos.publish(ack)

def graspit():
    gripper_diameter = generic_float()
    if grasp:
        gripper_diameter.request.data = 60
        client.call(gripper_diameter)
    else:
        gripper_diameter.request.data = 100
        client.call(gripper_diameter)

def grasp_in_sim(diameter):
    finger_1 = map_to_gripper_joints(diameter)
    finger_2 = map_to_gripper_joints(diameter)
    finger_3 = map_to_gripper_joints(diameter)
    return finger_1, finger_2, finger_3


# ----------- MAIN ----------- #

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('custom_joint_publisher')
    log.debug("Node initialized")

    # Create ROS publishers and subscribers
    pub_des_jstate = rospy.Publisher('/ur5/joint_group_pos_controller/command', Float64MultiArray, queue_size=10)
    # ack_pos = rospy.Publisher('/motion/ack', Int32, queue_size=1)
    # sub_pos = rospy.Subscriber('/motion/pos', pos, pos_callback)

    client = rospy.ServiceProxy('/move_gripper', generic_float)

    TH0 = np.array([-0.32, -0.78, -2.56, -1.63, -1.57, 3.49])  # Initial joint configuration
    log.debug(f"Initial joint configuration set to {TH0}")

    starting_position()

    rate = rospy.Rate(LOOP_RATE)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
