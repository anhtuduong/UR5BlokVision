
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import rospy as ros
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import Constraints, OrientationConstraint

import numpy as np
from utils_ur5.Logger import Logger as log

def get_trajectory(pose_target: geometry_msgs.msg.Pose):

    log.debug_highlight(f'Start planning trajectory')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ur5_arms"  # Or whichever group you defined in your SRDF
    group = moveit_commander.MoveGroupCommander(group_name)
    log.debug(f"Group name: {group_name}")

    group.set_pose_target(pose_target)
    log.debug(f"Pose target:\n{pose_target}")

    # Set the current joint state as the starting point for planning
    # group.set_start_state(current_joint_state)

    # # Define the orientation constraint
    # orientation_constraint = Constraints()
    # orientation_constraint.orientation_constraints.append(OrientationConstraint())
    # orientation_constraint.orientation_constraints[0].link_name = group.get_end_effector_link()
    # orientation_constraint.orientation_constraints[0].header.frame_id = group.get_planning_frame()
    # orientation_constraint.orientation_constraints[0].orientation.x = 0.0
    # orientation_constraint.orientation_constraints[0].orientation.y = 0.0
    # orientation_constraint.orientation_constraints[0].orientation.z = 0.0
    # orientation_constraint.orientation_constraints[0].orientation.w = 1.0
    # orientation_constraint.orientation_constraints[0].absolute_x_axis_tolerance = 1.0
    # orientation_constraint.orientation_constraints[0].absolute_y_axis_tolerance = 1.0
    # orientation_constraint.orientation_constraints[0].absolute_z_axis_tolerance = 1.0
    # orientation_constraint.orientation_constraints[0].weight = 0.1

    # # Apply the orientation constraint
    # group.set_path_constraints(orientation_constraint)

    # Plan
    success, plan, planning_time, _ = group.plan()

    if success:
        joint_trajectory = plan.joint_trajectory
        log.info(f'PLANNING SUCCESSFUL in {planning_time} seconds')
        return joint_trajectory
    else:
        log.error(f'PLANNING FAILED in {planning_time} seconds')
        return None

if __name__ == '__main__':
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0.119298
    pose_target.position.y = 0.69092
    pose_target.position.z = 1.04841
    pose_target.orientation.x = -0.612378
    pose_target.orientation.y = 0.785267
    pose_target.orientation.z = -0.0308864
    pose_target.orientation.w = 0.0859964
    joint_states = get_trajectory(pose_target)
