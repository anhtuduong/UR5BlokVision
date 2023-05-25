#!/usr/bin/env python3

import rospy
from motion.msg import pos
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

# ------------------- GLOBAL VARIABLES ------------------- #

# Publisher for the position of the block
pub_pos = None
# Publisher for the ack
pub_ack = None
# Subscriber for the vision topic
sub_vision = None
# Subscriber for the ack topic
sub_ack = None
# Subscriber for the stop topic
sub_stop = None
# Flag to indicate that the vision msg is received
vision_received = 0
# Flag to indicate that the vision is on
vision_on = 1
# Position of the block
block_pos = np.zeros(3)
# Rotation of the block
block_rot = np.zeros(3)
# Class of the block
block_class = 0
# Flag to indicate that the motion is finished
ready = 1
# Flag to indicate that the task manager has to stop
stop = 0

Z = 0.835

REAL_ROBOT = 0

# ------------------- FUNCTIONS ------------------- #

def world_to_base(xw):
    """
    Convert the position of the block from the world frame to the base frame
    """
    T = np.array([[1, 0, 0, 0.5],
                  [0, -1, 0, 0.35],
                  [0, 0, -1, 1.75],
                  [0, 0, 0, 1]])

    xt = np.dot(np.linalg.inv(T), np.append(xw, 1))
    xb = xt[:3]
    return xb


def camera_to_world(xw):
    """
    Convert the position of the block from the camera frame (real robot) to the world frame
    """
    T = np.array([[0.866, 0, 0.5, -0.4],
                  [0, 1, 0, 0.53],
                  [-0.5, 0, 0.866, 1.4],
                  [0, 0, 0, 1]])

    xt = np.dot(T, np.append(xw, 1))
    xb = xt[:3]
    return xb


def vision_callback(msg):
    """
    Callback function for the vision topic, sets the detected position of the block
    """
    global vision_received, block_pos, block_rot, block_class
    vision_received = 1
    block_pos = np.array([msg.x, msg.y, msg.z])
    block_rot = np.array([msg.roll, msg.pitch, msg.yaw])
    block_class = msg.class_id


def ack_callback(msg):
    """
    Callback function for the ack topic, sets the ready variable -> finished the motion
    """
    global ready
    ready = msg.data
    ack()


def stop_callback(msg):
    """
    Callback function for the stop topic, sets the stop variable -> stop the motion -> finished the task
    """
    global stop
    stop = msg.data


def ack():
    """
    This function is used to send the ack to the taskManager
    - send it when the robot has finished the motion task
    """
    ack = Int32()
    ack.data = 1
    pub_ack.publish(ack)


def motion_planner():
    global vision_received, vision_on, block_pos, block_rot, block_class, ready, stop, Z
    rospy.init_node('motion_planner')
    
    global pub_pos
    global pub_ack, sub_vision, sub_ack, sub_stop

    pub_pos = rospy.Publisher('/motion/pos', pos, queue_size=1)
    pub_ack = rospy.Publisher('/vision/ack', Int32, queue_size=1)

    sub_vision = rospy.Subscriber('/vision/pos', pos, vision_callback)
    sub_ack = rospy.Subscriber('/motion/ack', Int32, ack_callback)
    sub_stop = rospy.Subscriber('/taskManager/stop', Int32, stop_callback)

    loop_rate = rospy.Rate(1000)

    msg = pos()

    while not rospy.is_shutdown():
        while pub_pos.get_num_connections() < 1:
            loop_rate.sleep()

        if not vision_on:  # If the vision is not on, the user has to insert the position of the block
            if ready:  # If the motion is finished, the user can insert the position of the block
                ready = 0
                x, y, z, roll, pitch, yaw = map(float, input("Insert the position (x y z roll pitch yaw): ").split())
                msg.x = x
                msg.y = y
                msg.z = z
                msg.roll = roll
                msg.pitch = pitch
                msg.yaw = yaw
                msg.class_id = int(input("Insert the class id: "))
                pub_pos.publish(msg)
        else:  # Vision is on
            if vision_received and ready and not stop:  # If vision msg is received and the motion is finished, send the position to the motion
                ready = 0
                print("Blocks coordinates received from vision")
                print("Block position:", block_pos)
                print("Block rotation:", block_rot)
                print("Block class:", block_class)

                if REAL_ROBOT:
                    block_pos = camera_to_world(block_pos)
                else:
                    block_pos = world_to_base(block_pos)

                msg.x = block_pos[0]
                msg.y = block_pos[1]
                msg.z = Z
                msg.roll = block_rot[0]
                msg.pitch = block_rot[1]
                msg.yaw = block_rot[2]
                msg.class_id = block_class

                vision_received = 0
                pub_pos.publish(msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass