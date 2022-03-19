#!/usr/bin/env python2.7

# ===========================
# Robot navigation on the map
# ===========================

from turtle import st
import rospy

# for the path planning
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback


# ===========================
# Initialization of the nodes
# ===========================

rospy.init_node('rosbot_path_planning')
# create the connection to the action server
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# waits until the action server is up and running
client.wait_for_server()
# creates a Goal to send to the action server
Goal = MoveBaseGoal()


# ====================
# Creation of the goal
# ====================
'''
rosmsg show move_base_msgs/MoveBaseGoal
geometry_msgs/PoseStamped target_pose
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
'''

# creates a Goal to send to the action server
Goal.target_pose.header.stamp = rospy.Time.now()
Goal.target_pose.header.frame_id = 'map'

Goal.target_pose.pose.position.x = 2
Goal.target_pose.pose.position.y = 3
Goal.target_pose.pose.position.z = 0

Goal.target_pose.pose.orientation.x = 0
Goal.target_pose.pose.orientation.y = 0
Goal.target_pose.pose.orientation.z = 0
Goal.target_pose.pose.orientation.w = 1

def next_goal():
  print('=== NEXT GOAL ===')
  Goal.target_pose.pose.position.x -=0.5
  Goal.target_pose.pose.position.y = 0
  # sends the Goal to the action server, specifying which feedback function
  # to call when feedback received
  client.send_goal(Goal, feedback_cb=feedback_callback)

# ===========================
# Get the goal status
# ===========================
def feedback_callback(feedback):
  state = client.get_state()
  if state != 3:
    print('[Result] State: %d, going to goal..'%(state))
  else:
    print('[Result] State: %d, reached the goal!'%(state))
    next_goal()


# ===============================
# publish the goal on the rosbot
# ===============================

while not rospy.is_shutdown():
  client.send_goal(Goal, feedback_cb=feedback_callback)
  client.wait_for_result()