#! /usr/bin/env python
import rospy

# for the Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
state = None

def get_status(msg):
    global state
    state = msg.status_list[0].status #get the status of the goal, 3 is find

#first need to publish soemthing before checking the call back --> it gives error right now

# ===========================
# Initialization of the nodes
# ===========================

rospy.init_node('rosbot_path_planning')
pub_goal = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=1)
subscriber_move_base_status = rospy.Subscriber('/move_base/status',GoalStatusArray,get_status, queue_size=1)
goal = PoseStamped()

goal_global = MoveBaseActionGoal()

# ====================
# Creation of the goal
# ====================

goal.header.frame_id = "map"
goal.header.stamp = rospy.Time.now()

goal.pose.position.x = -1
goal.pose.position.y = 0
goal.pose.position.z = 0

goal.pose.orientation.x = 0
goal.pose.orientation.y = 0
goal.pose.orientation.z = 0
goal.pose.orientation.w = 1

# ====================
# New kind of goal
# ====================

goal_global.header = goal.header

goal_global.goal_id.stamp = goal_global.header.stamp
goal_global.goal_id.id = "Los nuemros unos"

goal_global.goal.target_pose = goal


# =====================
# publish on the rosbot
# =====================


#velocity.linear.x = 0.1

print("published")

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    pub_goal.publish(goal_global)
    rate.sleep()
    #pub.publish(velocity)
    print("doing my job")
    print("state of the goal is ", state)


