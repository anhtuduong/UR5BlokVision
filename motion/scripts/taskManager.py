#!/usr/bin/env python

import rospy
from motion.msg import pos
from std_msgs.msg import Int32
import numpy as np

# ------------------- GLOBAL VARIABLES ------------------- #

pub_pos = rospy.Publisher('/motion/pos', pos, queue_size=1)
pub_ack = rospy.Publisher('/vision/ack', Int32, queue_size=1)
sub_vision = rospy.Subscriber('/vision/pos', pos, vision_callback)
sub_ack = rospy.Subscriber('/motion/ack', Int32, ack_callback)
sub_stop = rospy.Subscriber('/taskManager/stop', Int32, stop_callback)
vision_received = 0
vision_on = 1
block_pos = np.zeros(3)
block_rot = np.zeros(3)
block_class = 0
ready = 1
stop = 0
Z = 0.835

LOOP_RATE = 1000
REAL_ROBOT = 0

# ------------------- FUNCTIONS DEFINITIONS ------------------- #

def world_to_base(xw):
    T = np.array([[1, 0, 0, 0.5],
                  [0, -1, 0, 0.35],
                  [0, 0, -1, 1.75],
                  [0, 0, 0, 1]])
    xt = np.dot(np.linalg.inv(T), np.append(xw, 1))
    xb = xt[:3]
    return xb

def camera_to_world(xw):
    T = np.array([[0.866, 0, 0.5, -0.4],
                  [0, 1, 0, 0.53],
                  [-0.5, 0, 0.866, 1.4],
                  [0, 0, 0, 1]])
    xt = np.dot(T, np.append(xw, 1))
    xb = xt[:3]
    return xb

def vision_callback(msg):
    global vision_received, block_pos, block_rot, block_class
    vision_received = 1
    block_pos = np.array([msg.x, msg.y, msg.z])
    block_rot = np.array([msg.roll, msg.pitch, msg.yaw])
    block_class = msg.class_id

def ack_callback(msg):
    global ready
    ready = msg.data
    ack()

def ack():
    ack = Int32()
    ack.data = 1
    pub_ack.publish(ack)

def stop_callback(msg):
    global stop
    stop = msg.data

# ------------------- MAIN ------------------- #

if __name__ == '__main__':
    rospy.init_node('motion_planner')
    
    rate = rospy.Rate(LOOP_RATE)
    
    while not rospy.is_shutdown():
        while pub_pos.get_num_connections() < 1:
            rate.sleep()
        
        if not vision_on:
            if ready:
                ready = 0
                x, y, z, roll, pitch, yaw = input("Insert the position (x y z roll pitch yaw): ").split()
                x, y, z, roll, pitch, yaw = float(x), float(y), float(z), float(roll), float(pitch), float(yaw)
                class_id = int(input("Insert the class id: "))
                
                msg = pos()
                msg.x = x
                msg.y = y
                msg.z = z
                msg.roll = roll
                msg.pitch = pitch
                msg.yaw = yaw
                msg.class_id = class_id
                
                pub_pos.publish(msg)
        else:
            if vision_received and ready and not stop:
                ready = 0
                print("Blocks coordinates received from vision")
                print("Block position:", block_pos)
                print("Block rotation:", block_rot)
                print("Block class:", block_class)

            if REAL_ROBOT:
                block_pos = camera_to_world(block_pos)
            else:
                block_pos = world_to_base(block_pos)
            
            msg = pos()
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
    rate.sleep()
            
