#! /usr/bin/env python3
import rospy
import roslib
import math
import tf
import yaml


from spot_msgs.msg import FiducialLocalization
from std_srvs.srv import Empty

# Works by calling a service to save the fiducials from tf frame 
# rosservice call /save_fiducial "fiducial_name: ["fiducial_206", "fiducial_208", "fiducial_209"]"

class FiducialToFile():

    def __init__(self):
        self.tf_listener = None      # Tf listener
        self.yaml_location = None    # Location to store ymal files 
        self.frame_to = None         # Frame the fiducial will be transformed to ("map")
        self.all_fiducials = []

    # Save fiduails using a service
    def save_fiducials(self, req):
        param_file = {}
        for x in self.all_fiducials:
            self.tf_listener.waitForTransform(self.frame_to, x, rospy.Time(), rospy.Duration(4.0)) # Wait for the transform
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.frame_to, x, rospy.Time(0)) # Get the transform
                param_file[x] = [trans, rot] # Store it in the dictionary 
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("tf exception")
        
        with open(self.yaml_location, 'w') as file:
            yaml.dump(param_file, file) #  Load the parameters in to the file


    def save_fiducials_to_list(self, req):
        for x in req.fiducial_names:
            if not x in self.all_fiducials:
                self.all_fiducials.append(x)
  
            
    def main(self):
        rospy.init_node('save_fiducial')
        self.yaml_location = '/home/administrator/catkin_ws/src/navigation_2d_spot/config/fiducial_param.yaml' 
        self.tf_listener = tf.TransformListener()  
        self.frame_to = '/map' 

        rospy.Service('/save_fiducials', Empty, self.save_fiducials)
        rospy.Subscriber('/spot/fiducial', FiducialLocalization, self.save_fiducials_to_list)

        rospy.spin()



if __name__ == "__main__":
    SR = FiducialToFile()
    SR.main()
