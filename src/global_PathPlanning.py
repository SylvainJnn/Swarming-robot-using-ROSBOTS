#! /usr/bin/env python
import rospy

# for the Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal


# for the path planning
#import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# ===========================
# Initialization of the nodes
# ===========================

rospy.init_node('rosbot_path_planning')
pub_goal = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=1)
goal = PoseStamped()

goal_global = MoveBaseActionGoal()

# ====================
# Creation of the goal
# ====================

goal.header.frame_id = "map"
goal.header.stamp = rospy.Time.now()

goal.pose.position.x = 2
goal.pose.position.y = -4
goal.pose.position.z = 0

goal.pose.orientation.x = 0
goal.pose.orientation.y = 0
goal.pose.orientation.z = 0
goal.pose.orientation.w = 1

# ====================
# New kind of goal
# ====================

goal_global.header = goal.header

goal_global.goal_id.stamp = goal_global.header.stamp
goal_global.goal_id.id = "Los nuemros unos"

goal_global.goal.target_pose = goal


# =====================
# publish on the rosbot
# =====================


#velocity.linear.x = 0.1

print("published")

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    pub_goal.publish(goal_global)
    rate.sleep()
    #pub.publish(velocity)
    print("doing my job")


