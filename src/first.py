#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def getposition(info_pos):
    print(info_pos.twist)

rospy.init_node("first_callback")
odom_sub = rospy.Subscriber('/odom', Odometry, getposition)
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
speed = Twist()

rate = rospy.Rate(1)
count = 0
while(not rospy.is_shutdown()):
    rate.sleep()
    
    count +=1

    if(count<3):
        speed.linear.x = 0.12
    
    elif(count<6):
        speed.linear.x  = -0.9

    else:
        count=0
        speed.linear.x = 0

    vel_pub.publish(speed)
    print("running from pc")
