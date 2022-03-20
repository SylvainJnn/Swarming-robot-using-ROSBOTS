#! /usr/bin/env python
import rospy


rospy.init_node('rosbot_goals_client')
rospy.wait_for_service('/rosbots_goals_server')