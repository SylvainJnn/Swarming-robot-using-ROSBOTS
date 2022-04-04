#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
import tf
import math
#include <tf/transform_broadcaster.h>



def main():
	rospy.init_node("drive_controller")
	rate = rospy.Rate(100)
	print_b=rospy.get_param("print_brightness", False)
	pose_sub = rospy.Subscriber("/pose", PoseStamped, pose_callback)
	while(not rospy.is_shutdown()):
		rate.sleep()
        

def pose_callback(msg):
    br = tf.TransformBroadcaster()
    x_q = msg.pose.orientation.x
    y_q = msg.pose.orientation.y
    z_q = msg.pose.orientation.z
    w_q = msg.pose.orientation.w


    x_org = msg.pose.position.x
    y_org = msg.pose.position.y

    br.sendTransform((msg.x, msg.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                        rospy.Time.now(),)



if __name__ == "__main__":
	main()



tf::Transform transform;
tf::Quaternion q;

void pose_callback(const geometry_msgs::PoseStampedPtr &pose)
{
   static tf::TransformBroadcaster br;

   transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0));
   transform.setRotation(q);

   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "drive_controller");
   ros::NodeHandle n("~");
   ros::Subscriber pose_sub = n.subscribe("/pose", 1, pose_callback);