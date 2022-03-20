#! /usr/bin/env python
import rospy

# for the Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal


# for the path planning
#import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# ===========================
# Initialization of the nodes
# ===========================

rospy.init_node('rosbot_path_planning')
pub_goal2 = rospy.Publisher('rosbot2/move_base/goal',MoveBaseActionGoal,queue_size=1)
pub_goal1 = rospy.Publisher('/rosbot1/move_base/goal',MoveBaseActionGoal,queue_size=1)

goal = PoseStamped()
goal2 = PoseStamped()

goal_global = MoveBaseActionGoal()
goal_global2 = MoveBaseActionGoal()

# ====================
# goal1
# ====================

goal.header.frame_id = "map"
goal.header.stamp = rospy.Time.now()

goal.pose.position.x = -2
goal.pose.position.y = -2
goal.pose.position.z = 0

goal.pose.orientation.x = 0
goal.pose.orientation.y = 0
goal.pose.orientation.z = 0
goal.pose.orientation.w = 1


goal_global.header = goal.header

goal_global.goal_id.stamp = goal_global.header.stamp
goal_global.goal_id.id = "Los nuemros unos"

goal_global.goal.target_pose = goal



# ====================
# goal2
# ====================

goal2.header.frame_id = "map"
goal2.header.stamp = rospy.Time.now()

goal2.pose.position.x = 2
goal2.pose.position.y = 2
goal2.pose.position.z = 0

goal2.pose.orientation.x = 0
goal2.pose.orientation.y = 0
goal2.pose.orientation.z = 0
goal2.pose.orientation.w = 1


goal_global2.header = goal2.header

goal_global2.goal_id.stamp = goal_global2.header.stamp
goal_global2.goal_id.id = "Los nuemros dos"

goal_global2.goal.target_pose = goal2

#velocity.linear.x = 0.1

print("published")

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    pub_goal1.publish(goal_global)
    pub_goal2.publish(goal_global2)
    rate.sleep()
    #pub.publish(velocity)
    print("doing my job")


