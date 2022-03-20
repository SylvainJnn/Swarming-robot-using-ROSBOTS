#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
 
import tf
import turtlesim.msg
 
def handle_turtle_pose(msg, rosbot_name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     rosbot_name,
                     "world")

if __name__ == '__main__':
    rospy.init_node('rosbot_tf_broadcaster')
    rosbot_name = rospy.get_param('~rosbot')
    rospy.Subscriber('/%s/pose' % rosbot_name,
                        turtlesim.msg.Pose,#wrong !
                      handle_turtle_pose,
                      rosbot_name)
    rospy.spin()