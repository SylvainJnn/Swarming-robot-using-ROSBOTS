#! /usr/bin/env python
import rospy

# for the Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
state = None

def get_status(msg):
    global state, current_goal_id
    state = msg.status_list[0].status #get the status of the goal, 3 is find
    current_goal_id = msg.status_list[0].goal_id.id
    #print(current_goal_id)

#first need to publish soemthing before checking the call back --> it gives error right now
def pub_firsttime():
    while(state != 3):
        pub_goal.publish(goal_global)
        rospy.sleep(1)
        print("I'm on it 1")

def pub_function():
    while(current_goal_id != new_goal_id):
        pub_goal.publish(goal_global)
        rospy.sleep(1)
        print("i'm checking bro")
    #we can add a if to check if msg.goal id the the same as new_goal
    while(state != 3):
        pub_goal.publish(goal_global)
        rospy.sleep(1)
        print("I'm on it")


# ===========================
# Initialization of the nodes
# ===========================

rospy.init_node('rosbot_path_planning')
pub_goal = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=1)
subscriber_move_base_status = rospy.Subscriber('/move_base/status',GoalStatusArray,get_status, queue_size=1)
goal = PoseStamped()

goal_global = MoveBaseActionGoal()
previous_goal_id = None
new_goal_id = None
# ====================
# Creation of the goal
# ====================

goal.header.frame_id = "map"
goal.header.stamp = rospy.Time.now()

goal.pose.position.x = 2
goal.pose.position.y = 4
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
previous_goal_id = new_goal_id
new_goal_id = goal_global.goal_id

goal_global.goal.target_pose = goal


# =====================
# publish on the rosbot
# =====================


#velocity.linear.x = 0.1

print("published")

rate = rospy.Rate(1)

pub_goal.publish(goal_global)

pub_firsttime()

print("trop fort ce gara")

rospy.sleep(2)
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
goal_global.goal_id.id = "Los nuemros DOS"
previous_goal_id = new_goal_id
new_goal_id = "Los nuemros DOS"


goal_global.goal.target_pose = goal

pub_function()

print("il a pecho son 06")
