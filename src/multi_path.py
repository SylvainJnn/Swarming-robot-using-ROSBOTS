#!/usr/bin/env python3

# =============================================
# Multi Rosbots Control : Go in different goals
# =============================================

import rospy
from geometry_msgs.msg import PoseStamped

# ==========================
# Initialization of the node
# ==========================

rospy.init_node('multi_path')
pub_goal_1 = rospy.Publisher('rosbot1/move_base_simple/goal',PoseStamped,queue_size = 1)
pub_goal_2 = rospy.Publisher('rosbot2/move_base_simple/goal',PoseStamped,queue_size = 1)
goal1 = PoseStamped()
goal2 = PoseStamped()

# =====================
# Creation of the goals
# =====================

# goal 1

goal1.header.frame_id = "map"
goal1.header.stamp = rospy.Time.now()

goal1.pose.position.x = 2
goal1.pose.position.y = 2
goal1.pose.position.z = 0

goal1.pose.orientation.x = 0
goal1.pose.orientation.y = 0
goal1.pose.orientation.z = 0
goal1.pose.orientation.w = 1

# goal 2

goal2.header.frame_id = "map"
goal2.header.stamp = rospy.Time.now()

goal2.pose.position.x = 2
goal2.pose.position.y = -4
goal2.pose.position.z = 0

goal2.pose.orientation.x = 0
goal2.pose.orientation.y = 0
goal2.pose.orientation.z = 0
goal2.pose.orientation.w = 1

# =====================
# publish on the rosbot
# =====================

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()
    
    pub_goal_1.publish(goal1)
    pub_goal_2.publish(goal2)
  

