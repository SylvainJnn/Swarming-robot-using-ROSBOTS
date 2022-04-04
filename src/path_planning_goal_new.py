#!/usr/bin/env python

# ===========================
# Robot navigation on the map
# ===========================

from turtle import st
import rospy

# for the Odometry
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
#from tf.transformations import euler_from_quaternion, quaternion_from_euler

# for the path planning
#import actionlib
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseActionFeedback


# ===========================
# Get the goal status
# ===========================

'''
rosmsg show actionlib_msgs/GoalStatusArray
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalStatus[] status_list
  uint8 PENDING=0
  uint8 ACTIVE=1
  uint8 PREEMPTED=2
  uint8 SUCCEEDED=3
  uint8 ABORTED=4
  uint8 REJECTED=5
  uint8 PREEMPTING=6
  uint8 RECALLING=7
  uint8 RECALLED=8
  uint8 LOST=9
  actionlib_msgs/GoalID goal_id
    time stamp
    string id
  uint8 status
  string text
'''
state = 0
def get_status(msg):
  global state
  state = msg.status_list[0].status 
  goal_ID = msg.status_list[0].goal_id.id
  #print('Goal ID: ', goal_ID)
  print('Goal Status: ',state)
  #return state


# ===========================
# Initialization of the nodes
# ===========================

rospy.init_node('rosbot_path_planning')
#pub_vel = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
#velocity = Twist()
pub_goal = rospy.Publisher('move_base/goal',MoveBaseActionGoal,queue_size=1)
subscriber_status = rospy.Subscriber('/move_base/status',GoalStatusArray,get_status, queue_size=1)
PoseStamped_obj = PoseStamped()
goal = MoveBaseActionGoal()


# ====================
# Creation of the goal
# ====================

'''
rosmsg show geometry_msgs/PoseStamped
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
PoseStamped_obj.header.frame_id = "map"
PoseStamped_obj.header.stamp = rospy.Time.now()
PoseStamped_obj.pose.position.x = 2
PoseStamped_obj.pose.position.y = 3
PoseStamped_obj.pose.position.z = 0

PoseStamped_obj.pose.orientation.x = 0
PoseStamped_obj.pose.orientation.y = 0
PoseStamped_obj.pose.orientation.z = 0
PoseStamped_obj.pose.orientation.w = 1


# ====================
# Publish the goal
# ====================

'''
rosmsg show move_base_msgs/MoveBaseActionGoal
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
move_base_msgs/MoveBaseGoal goal
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
goal.header = PoseStamped_obj.header
goal.goal_id.stamp = goal.header.stamp
goal.goal_id.id = "My ID"
goal.goal.target_pose = PoseStamped_obj

def next_goal():
  PoseStamped_obj.header.frame_id = "map"
  PoseStamped_obj.header.stamp = rospy.Time.now()
  PoseStamped_obj.pose.position.x = -0.5
  PoseStamped_obj.pose.position.y = 0
  PoseStamped_obj.pose.position.z = 0

  PoseStamped_obj.pose.orientation.x = 0
  PoseStamped_obj.pose.orientation.y = 0
  PoseStamped_obj.pose.orientation.z = 0
  PoseStamped_obj.pose.orientation.w = 1

  # Publish the goal
  goal.header = PoseStamped_obj.header
  goal.goal_id.stamp = goal.header.stamp
  goal.goal_id.id = "My New ID"
  goal.goal.target_pose = PoseStamped_obj


# =====================
# publish on the rosbot
# =====================

#velocity.linear.x = 0.1

while not rospy.is_shutdown():
  #pub.publish(velocity)
  pub_goal.publish(goal)
  rospy.sleep(0.25)
  if state != 3:
    print("Going to goal ...")
  else:
    print('Reached the goal! Going to next goal ..')
    next_goal()





'''
#def get_odometry(odom_data):
    #get the current pose then move to a new pose
    #movements_no = 0
    #while movements_no < 3:
        # # get both of linear and angular current pose of the rosbot 
        # linear_x_pos= odom_data.pose.pose.position.x
        # linear_y_pos= odom_data.pose.pose.position.y
        # linear_z_pos= odom_data.pose.pose.position.z

        # angular_x_pos = odom_data.pose.pose.orientation.x
        # angular_y_pos = odom_data.pose.pose.orientation.y
        # angular_z_pos= - odom_data.pose.pose.orientation.z


        # # get all Euler angles (roll. pitch. yaw) from /odom topic and convert them to quaternion angles 
        # (q_x, q_y, q_z, q_w)= quaternion_from_euler(angular_x_pos, angular_y_pos, angular_z_pos)

        #goal.pose.position.x = linear_x_pos+0.5 #2
        #goal.pose.position.y = linear_y_pos+0.5 #2
        #goal.pose.position.z = linear_z_pos+0.5 #0

        #goal.pose.orientation.x = q_x+0.5 #0
        #goal.pose.orientation.y = q_y+0.5 #0
        #goal.pose.orientation.z = q_z+0.5 #0
        #goal.pose.orientation.w = q_w+0.5 #1

        # publish the new pose on the rosbot
        #pub_goal.publish(goal)
        
        #movements_no += 1

#odom_sub = rospy.Subscriber('/odom',Odometry , get_odometry)
'''