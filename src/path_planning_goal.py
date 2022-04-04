#!/usr/bin/env python



# ===========================
# Robot navigation on the map
# ===========================

import rospy

# for the Odometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

# for the path planning
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# ===========================
# Initialization of the nodes
# ===========================

rospy.init_node('rosbot_path_planning')
pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
pub_goal = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
velocity = Twist()
goal = PoseStamped()


# ====================
# Creation of the goal
# ====================

goal.header.frame_id = "map"
goal.header.stamp = rospy.Time.now()

goal.pose.position.x = 2
goal.pose.position.y = 2
goal.pose.position.z = 0

goal.pose.orientation.x = 0
goal.pose.orientation.y = 0
goal.pose.orientation.z = 0
goal.pose.orientation.w = 1





# =====================
# publish on the rosbot
# =====================


#velocity.linear.x = 0.1

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    
    rate.sleep()
    #pub.publish(velocity)
    pub_goal.publish(goal)
