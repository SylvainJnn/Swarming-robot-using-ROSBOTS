#! /usr/bin/env python
import rospy

# for the Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray



class multi_goal_path_planning:
    def __init__(self):
        rospy.init_node('rosbot_path_planning')
        self.pub_goal = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=1)
        self.subscriber_move_base_status = rospy.Subscriber('/move_base/status',GoalStatusArray,self.get_status, queue_size=1)
        
        self.goal_pose = PoseStamped()
        self.goal_action = MoveBaseActionGoal()
    
        self.previous_goal_id = None
        self.new_goal_id = None
        self.state = None
        self.goal_counter = 0

        #rospy.sleep(2)
        self.send_goal()
        rospy.sleep(1)
        self.send_goal()

    def get_status(self, status_msg):
        self.state = status_msg.status_list[0].status #get the status of the goal, 3 is find
        self.current_goal_id = status_msg.status_list[0].goal_id.id


    #first need to publish soemthing before checking the call back --> it gives error right now
    def pub_firsttime(self):
        while(self.state != 3):
            self.pub_goal.publish(self.goal_action)
            rospy.sleep(1)
            print("I'm on it 1")
        self.goal_counter += 1

    def pub_function(self):
        #divided into 2 functions
        while(self.current_goal_id != self.new_goal_id):
            self.pub_goal.publish(self.goal_action)
            rospy.sleep(1)
            print("i'm checking bro")
            print("goal number ", self.goal_counter)
            print(self.current_goal_id , "\n\n",self.new_goal_id,self.current_goal_id != self.new_goal_id)
        #we can add a if to check if msg.goal id the the same as new_goal
        while(self.state != 3):
            self.pub_goal.publish(self.goal_action)#normally we don't need this line anymore
            rospy.sleep(1)
            print("I'm on it 2")
        self.goal_counter += 1

    def update_goal(self, x, y, z, roll, pitch, yaw, w):#goal_action is a MoveBaseActionGoal, it includes a PoseStamped object (=goal_pose) with the same information
        # ...
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.header.stamp = rospy.Time.now()

        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z

        self.goal_pose.pose.orientation.x = roll
        self.goal_pose.pose.orientation.y = pitch
        self.goal_pose.pose.orientation.z = yaw
        self.goal_pose.pose.orientation.w = w

        #...
        self.goal_action.header = self.goal_pose.header

        self.goal_action.goal_id.stamp = self.goal_action.header.stamp
        self.goal_action.goal_id.id = "goal number" + str(self.goal_counter)
        self.new_goal_id = self.goal_action.goal_id.id

        self.goal_action.goal.target_pose = self.goal_pose

    def send_goal(self):

        self.update_goal(1,0,0,0,0,0,1)
    
        if(self.goal_counter == 0):
            self.pub_firsttime()
        
        else:
            self.update_goal(2,2,0,0,0,0,1)
            self.pub_function()



if __name__ == "__main__":
    my_goals = multi_goal_path_planning()
    print("I'm running")
    rospy.spin()