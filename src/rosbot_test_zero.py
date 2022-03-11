#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# ==========================
# Initialization of the node
# ==========================

rospy.init_node('rosbot_test_zero')
pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
velocity = Twist()


# =====================
# publish on the rosbot
# =====================

velocity.linear.x = 0.6

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()
    pub.publish(velocity)
