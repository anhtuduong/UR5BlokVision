# Import system
import os
import sys
import time

# Resolve paths
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Ros utils
import rospy as ros

# Robot utils
from locosim.ur5_controller import UR5Controller
import locosim.robot_control.base_controllers.params as conf


# Other utils
import numpy as np
from utils_ur5.Logger import Logger as log

# Constants
from constants import *

# ----------- TALKER ----------- #
def talker(p):
    """
    """
    # start thread
    p.start()

    # check real robot
    if p.real_robot:
        p.startRealRobot()
    else:
        additional_args = ['gripper:=' + str(p.gripper)]
                        #    'soft_gripper:=' + str(conf.robot_params[p.robot_name]['soft_gripper'])]
                            #, 'gui:=false']
        p.startSimulator(world_name = p.world_name,
                                      use_torque_control = p.use_torque_control,
                                      additional_args = additional_args)

    # specify xacro location
    xacro_path = XACRO_PATH
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    p.startupProcedure()

    # sleep to avoid that the real robot crashes on the table
    time.sleep(3.)

    # loop frequency
    rate = ros.Rate(1 / conf.robot_params[p.robot_name]['dt'])

    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    p.q_des = np.copy(p.q_des_q0)

    # use the point to point position controller
    if not p.use_torque_control:
        p.switch_controller("joint_group_pos_controller")

    # homing procedure
    if p.homing_flag:
        if p.real_robot:
            v_des = 0.2
        else:
            v_des = 0.6
        p.homing_procedure(conf.robot_params[p.robot_name]['dt'],
                                        v_des,
                                        conf.robot_params[p.robot_name]['q_0'],
                                        rate)

    


    # test Vision
    # vision = Vision()
    # block_list = vision.get_block_list()
    # for block in block_list:
    #     pose_target = Pose()
    #     pose_target.position.x = block.position[0]
    #     pose_target.position.y = block.position[1]
    #     pose_target.position.z = block.position[2]
    #     p.move_to(pose_target, p.dt, p.v_des, rate)

    # launch task planner node (implement the state machine)

    # launch motion node (takes care of moving the end-effector, remember to add a rate.sleep!)

    # launch vision node (visual pipelines to detect object, made with service call)

    # control loop (runs every dt seconds)
    while not ros.is_shutdown():
        p.updateKinematicsDynamics()

        # # get end effector pose
        # ee_pos, ee_rot = p.get_ee_position_rotation()
        # log.info(f'End effector position: {ee_pos}')
        # log.info(f'End effector rotation: {ee_rot}')

        ## set joints here
        # p.q_des = p.q_des_q0  + 0.1 * np.sin(2*np.pi*0.5*p.time)
        # p.qd_des = 0.1 * 2 * np.pi * 0.5* np.cos(2 * np.pi * 0.5 * p.time)*np.ones(p.robot.na)

        ##test gripper
        # in Simulation remember to set gripper_sim : True in params.yaml!
        # if p.time>5.0 and (gripper_on == 0):
        #     print("gripper 30")
        #     p.controller_manager.gm.move_gripper(10)
        #     gripper_on = 1
        # if (gripper_on == 1) and p.time>10.0:
        #     print("gripper 100")
        #     p.controller_manager.gm.move_gripper(100)
        #     gripper_on = 2
        #need to uncomment this to be able to send joints references (leave it commented if you have an external node setting them)
        #p.controller_manager.sendReference(p.q_des, p.qd_des, p.h)

        if p.real_robot:
            p.ros_pub.add_arrow(p.x_ee + p.base_offset,
                                             p.contactForceW / (6 * p.robot.robot_mass),
                                             "green")

        # log variables
        p.logData()
        # plot end-effector
        p.ros_pub.add_marker(p.x_ee + p.base_offset)
        p.ros_pub.publishVisual()

        # wait for syncronization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3)  # to avoid issues of dt 0.0009999

# ----------- RUN MAIN ----------- #
if __name__ == "__main__":
    
    p = UR5Controller()

    try:
        talker(p)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

        # TODO: plotting does not work
        # if conf.plotting:
        #     p.plotStuff([time_start, time_stop])