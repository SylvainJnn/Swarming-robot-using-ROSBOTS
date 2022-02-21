#!/usr/bin/env python3

# =====================
# Multi Rosbots Control
# =====================

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# ==========================
# Initialization of the node
# ==========================

rospy.init_node('rosbot_test_zero')
pub_robot1 = rospy.Publisher('rosbot1/cmd_vel',Twist,queue_size = 1)
pub_robot2 = rospy.Publisher('rosbot2/cmd_vel',Twist,queue_size = 1)
velocity1 = Twist()
velocity2 = Twist()

# =====================
# publish on the rosbot
# =====================


velocity1.linear.x = -0.1
velocity2.linear.x = +0.1

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()
	    
    pub_robot1.publish(velocity1)
    pub_robot2.publish(velocity2)


    
