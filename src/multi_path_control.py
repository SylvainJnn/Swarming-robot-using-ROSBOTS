#!/usr/bin/env python

# =============================================
# Multi Rosbots Control : Go in different goals
# =============================================
from turtle import st
import rospy
# for the path planning
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback

# ==========================
# Initialization of the node
# ==========================

rospy.init_node('multi_path_planning')
# create the connections to the action server
client1 = actionlib.SimpleActionClient('rosbot1/move_base', MoveBaseAction)
client2 = actionlib.SimpleActionClient('rosbot2/move_base', MoveBaseAction)
# waits until the action server is up and running
client1.wait_for_server()
client2.wait_for_server()
# creates Goals to send to the action server
Goal1 = MoveBaseGoal()
Goal2 = MoveBaseGoal()


# =====================
# Creation of the goals
# =====================


# Goal 1
Goal1.target_pose.header.stamp = rospy.Time.now()
Goal1.target_pose.header.frame_id = 'map'

Goal1.target_pose.pose.position.x = 2
Goal1.target_pose.pose.position.y = 2
Goal1.target_pose.pose.position.z = 0

Goal1.target_pose.pose.orientation.x = 0
Goal1.target_pose.pose.orientation.y = 0
Goal1.target_pose.pose.orientation.z = 0
Goal1.target_pose.pose.orientation.w = 1

def next_goal1():
  print('=== NEXT GOAL1 ===')
  Goal1.target_pose.pose.position.x -=0.5
  Goal1.target_pose.pose.position.y = 0.625
  client1.send_goal(Goal1, feedback_cb=feedback_callback1)
  #client1.wait_for_result()


# Goal 2
Goal2.target_pose.header.stamp = rospy.Time.now()
Goal2.target_pose.header.frame_id = 'map'

Goal2.target_pose.pose.position.x = 2
Goal2.target_pose.pose.position.y = -4
Goal2.target_pose.pose.position.z = 0

Goal2.target_pose.pose.orientation.x = 0
Goal2.target_pose.pose.orientation.y = 0
Goal2.target_pose.pose.orientation.z = 0
Goal2.target_pose.pose.orientation.w = 1

def next_goal2():
  print('=== NEXT GOAL2 ===')
  Goal2.target_pose.pose.position.x -=0.5
  Goal2.target_pose.pose.position.y = -0.625
  client2.send_goal(Goal2, feedback_cb=feedback_callback2)
  client2.wait_for_result()

# ===========================
# Get the goal status
# ===========================

def feedback_callback1(feedback1):
    state1 = client1.get_state()
    if state1 != 3:
        print('[Rosbot1] State: %d, going to goal..'%(state1))
        print('====================================')
    else:
        print('[Rosbot1] State: %d, reached the goal!'%(state1))
        print('====================================')
        next_goal1()


def feedback_callback2(feedback2):
    state2 = client2.get_state()
    if state2 != 3:
        print('[Rosbot2] State: %d, going to goal..'%(state2))
        print('====================================')
    else:
        print('[Rosbot2] State: %d, reached the goal!'%(state2))
        print('====================================')
        next_goal2()

# =====================
# publish on the rosbot
# =====================

while not rospy.is_shutdown():
    client1.send_goal(Goal1, feedback_cb=feedback_callback1)
    client2.send_goal(Goal2, feedback_cb=feedback_callback2)
    #client1.wait_for_result()
    client2.wait_for_result()