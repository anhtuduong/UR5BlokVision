
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupActionGoal

def move_robot_arm(translation, rotation):
    # Initialize ROS node
    rospy.init_node('move_robot_arm_node', anonymous=True)

    # Create a publisher for the robot arm's goal pose
    pub = rospy.Publisher('/move_group/goal', MoveGroupActionGoal, queue_size=10)

    # Create a PoseStamped message for the desired end effector pose
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = 'base_link'  # Replace with the appropriate frame for your robot
    pose_msg.pose.position.x = translation[0]
    pose_msg.pose.position.y = translation[1]
    pose_msg.pose.position.z = translation[2]
    pose_msg.pose.orientation.x = rotation[0]
    pose_msg.pose.orientation.y = rotation[1]
    pose_msg.pose.orientation.z = rotation[2]
    pose_msg.pose.orientation.w = rotation[3]

    # Create a MoveGroupActionGoal message
    goal_msg = MoveGroupActionGoal()
    goal_msg.goal_id.stamp = rospy.Time.now()
    goal_msg.goal_id.id = 'goal'
    goal_msg.goal.request.group_name = 'manipulator'  # Replace with the name of your robot arm's planning group
    goal_msg.goal.request.num_planning_attempts = 1
    goal_msg.goal.request.allowed_planning_time = 5.0
    goal_msg.goal.request.planner_id = ''

    # Set the target pose for the robot arm
    goal_msg.goal.request.goal_constraints.position_constraints.append(pose_msg)

    # Publish the goal message
    pub.publish(goal_msg)
    rospy.sleep(1.0)  # Wait for the message to be processed by the planning node

if __name__ == '__main__':
    # Prompt the user to enter translation and rotation values
    translation = input("Enter the translation [x, y, z]: ")
    rotation = input("Enter the rotation [x, y, z, w]: ")

    # Call the function to move the robot arm
    move_robot_arm(translation, rotation)
