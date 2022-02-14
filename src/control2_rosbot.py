#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


rospy.init_node("first_callback")
vel_pub1 = rospy.Publisher('rosbot1/cmd_vel', Twist, queue_size=1)
vel_pub2 = rospy.Publisher('rosbot2/cmd_vel', Twist, queue_size=1)
speed1 = Twist()
speed2 = Twist()


rate = rospy.Rate(1)
count = 0
while(not rospy.is_shutdown()):
    rate.sleep()
    
    count +=1

    if(count<3):
        speed1.linear.x = 0.25
        speed2.linear.x = 0
    
    elif(count<6):
        speed1.linear.x = 0
        speed2.linear.x = 0.3
    
    elif(count<9):
        speed1.linear.x = -0.25
        speed2.linear.x = -0.25

    else:
        count=0
        speed1.linear.x = 0
        speed2.linear.x = 0


    vel_pub2.publish(speed2)
    vel_pub1.publish(speed1)
    print("running from pc")
