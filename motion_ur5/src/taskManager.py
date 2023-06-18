
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
import numpy as np
from motion.msg import pos
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from utils.Logger import Logger as log

LOOP_RATE = 1000
REAL_ROBOT = 0

def worldToBase(xw):
    T = np.array([[1, 0, 0, 0.5],
                  [0, -1, 0, 0.35],
                  [0, 0, -1, 1.75],
                  [0, 0, 0, 1]])
    xt = np.linalg.inv(T) @ np.array([xw[0], xw[1], xw[2], 1])
    xb = xt[:3]
    return xb

def cameraToWorld(xw):
    T = np.array([[0.866, 0, 0.5, -0.4],
                  [0, 1, 0, 0.53],
                  [-0.5, 0, 0.866, 1.4],
                  [0, 0, 0, 1]])
    xt = T @ np.array([xw[0], xw[1], xw[2], 1])
    xb = xt[:3]
    return xb

# def visionCallback(msg):
#     global vision_received, block_pos, block_rot, block_class
#     vision_received = 1
#     block_pos = np.array([msg.x, msg.y, msg.z])
#     block_rot = np.array([msg.roll, msg.pitch, msg.yaw])
#     block_class = msg.class_id

# def ackCallback(msg):
#     global ready
#     ready = msg.data
#     ack()

# def ack():
#     loop_rate = rospy.Rate(LOOP_RATE)
#     ack_msg = Int32()
#     ack_msg.data = 1
#     pub_ack.publish(ack_msg)
#     loop_rate.sleep()

# def stopCallback(msg):
#     global stop
#     stop = msg.data

pub_pos = rospy.Publisher("/motion/pos", pos, queue_size=1)
# sub_vision = rospy.Subscriber("/vision/pos", pos, visionCallback)
# pub_ack = rospy.Publisher("/vision/ack", Int32, queue_size=1)
# sub_ack = rospy.Subscriber("/motion/ack", Int32, ackCallback)
# sub_stop = rospy.Subscriber("/taskManager/stop", Int32, stopCallback)
# vision_received = 0
# vision_on = 0
block_pos = Point()
block_rot = Point()
block_class = 0
ready = 1
stop = 0


rospy.init_node("motion_planner")
loop_rate = rospy.Rate(LOOP_RATE)

while not rospy.is_shutdown():
    while pub_pos.get_num_connections() < 1:
        loop_rate.sleep()
        log.debug(f'Number of connections: {pub_pos.get_num_connections()}')

    log.debug(f'Number of connections: {pub_pos.get_num_connections()}')

    #     print("Insert the position (x y z roll pitch yaw): ")
    #     x, y, z, roll, pitch, yaw = map(float, input().split())
    #     print("Insert the class id: ")
    #     class_id = int(input())

    #     msg = pos()
    #     msg.x = x
    #     msg.y = y
    #     msg.z = z
    #     msg.roll = roll
    #     msg.pitch = pitch
    #     msg.yaw = yaw
    #     msg.class_id = class_id

    #     pub_pos.publish(msg)

    #     # if vision_received and ready and not stop:
    #     #     # ready = 0
    #     #     print("Blocks coordinates received from vision")
    #     #     print("Block position: {}".format(block_pos))
    #     #     print("Block rotation: {}".format(block_rot))
    #     #     print("Block class: {}".format(block_class))

    #     #     if REAL_ROBOT:
    #     #         block_pos = cameraToWorld(block_pos)
    #     #     else:
    #     #         block_pos = worldToBase(block_pos)

    #     #     msg = pos()
    #     #     msg.position.x = block_pos.x
    #     #     msg.position.y = block_pos.y
    #     #     msg.position.z = block_pos.z
    #     #     msg.orientation = block_rot
    #     #     msg.class_id = block_class

    #     #     # vision_received = 0
    #     #     pub_pos.publish(msg)

    # loop_rate.sleep()

