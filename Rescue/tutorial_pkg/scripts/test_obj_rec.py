#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import time



class ObjectViz:
    def __init__(self):
        self.objects_suscriber = rospy.Subscriber('/objects', Float32MultiArray, self.callback_odom)    
        self.action_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=55)
        self.id = 0.0
       
    def callback_odom(self,msg):
    	
        if(len(msg.data) > 0):
        	self.id = msg.data[0]
        


   
    
    def object_recognition_stop(self):
        set_vel = Twist()
        print(self.id)
        if(self.id == 1.0 or self.id == 2.0):
        	print("bg")
        	set_vel.linear.x = 0
        	set_vel.linear.y = 0
        	set_vel.linear.z = 0
        	self.action_publisher.publish(set_vel)

rec = ObjectViz()
rospy.init_node('cv')
while(1):
	rec.object_recognition_stop()

            


        






    
    
    

    



