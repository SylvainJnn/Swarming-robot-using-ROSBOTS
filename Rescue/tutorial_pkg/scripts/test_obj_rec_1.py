#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import time 
import cv2









class ObjectViz:
	def __init__(self):
		self.fl_suscriber = rospy.Subscriber('/range/fl', Float32MultiArray, self.callback_fl)
		self.fr_suscriber = rospy.Subscriber('/range/fr', Float32MultiArray, self.callback_fr) 
		self.objects_suscriber = rospy.Subscriber('/objects', Float32MultiArray, self.callback_odom)    
		self.action_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=55)
		self.id = 0.0
		self.object_width = 0.0
		self.object_height = 0.0
		self.camera_center = 320 
		self.max_ang_vel = 0.6
		self.min_ang_vel = 0.4
		self.ang_vel = 0
		self.x_pos = 0
		self.speed_coefficient = float(self.camera_center / self.max_ang_vel / 4)
		self.inPts = np.float32([[[0,0],[0,self.object_height],[self.object_width,self.object_height],[self.object_width,0]]])
		self.outPts = np.matrix([],np.float32)
		self.homography_matrix = []
		self.sensor_fl = 0
		self.sensor_fr = 0
		self.avg_dist = 0
		self.desired_dist = 0.2


        
       
	def callback_odom(self,msg):
    	
		if(len(msg.data) > 0):
			self.id = msg.data[0]
			self.object_width = msg.data[1]
			self.object_height = msg.data[2]
			self.homography_matrix=np.array([[msg.data[3],msg.data[6],msg.data[9]],        
			[msg.data[4],msg.data[7],msg.data[10]],
			[msg.data[5],msg.data[8],msg.data[11]]
			],np.float32) 		
		else:
			self.id = 0.0
	def callback_fl(self,msg):     
		self.sensor_fl = msg.range

	def callback_fr(self,msg): 
		self.sensor_fr = msg.range
        


   
    
	def object_recognition_stop(self):
        
		set_vel = Twist()
		if(self.id == 1.0 or self.id == 2.0):
			self.outPts = cv2.perspectiveTransform(self.inPts,self.homography_matrix)
			print(self.outPts)
			x_pos = int(self.outPts[0][0][0] + self.outPts[0][1][0] + self.outPts[0][2][0] + self.outPts[0][3][0])/4

			self.ang_vel = -(x_pos - self.camera_center) / self.speed_coefficient
			if (self.ang_vel >= -(self.min_ang_vel / 2) and self.ang_vel <= (self.min_ang_vel / 2)):
				set_vel.angular.z = 0
				if (self.sensor_fr > 0 and self.sensor_fl > 0):
					self.avg_dist = (self.sensor_fr + self.sensor_fl) / 2
					set_vel.linear.x = (self.avg_dist - self.desired_dist) / 4
				else:
					set_vel.linear.x = 0

			elif (self.ang_vel >= self.max_ang_vel):
				set_vel.angular.z = self.max_ang_vel

			elif (self.ang_vel <= -self.max_ang_vel):
				set_vel.angular.z = -self.max_ang_vel

			else:
				set_vel.angular.z = self.ang_vel
		else:
			set_vel.angular.x = 0
			set_vel.angular.z = 0

		self.action_publisher.publish(set_vel)
            

rec = ObjectViz()
rospy.init_node('cv')
while(1):
	rec.object_recognition_stop()
	

            


        






    
    
    

    



