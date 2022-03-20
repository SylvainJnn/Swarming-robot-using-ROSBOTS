#! /usr/bin/env python
import rospy

# for the Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray

from robot_goal import robot_goal

class multi_goal_path_planning:
    def __init__(self, rosbot_number):
        self.rosbot_name = self.give_rosbot_name(rosbot_number)
        self.node_name = 'rosbot_path_planning_'+str(1)
        print(self.node_name)
        rospy.init_node(self.node_name)
        self.pub_goal = rospy.Publisher(self.rosbot_name + '/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.subscriber_move_base_status = rospy.Subscriber(self.rosbot_name + '/move_base/status', GoalStatusArray, self.get_status, queue_size=1)
        
        self.goal_pose = PoseStamped()
        self.goal_action = MoveBaseActionGoal()
    
        self.previous_goal_id = None
        self.new_goal_id = None
        self.state = None
        self.goal_counter = 0

        self.goal_list = [] #self.array([])

    def give_rosbot_name(self, rosbot_number):
        if(rosbot_number == 0):
            return("")
        else:
            return("/rosbot" + str(rosbot_number))

    def get_status(self, status_msg):
        self.state = status_msg.status_list[0].status #get the status of the goal, 3 is find
        self.current_goal_id = status_msg.status_list[0].goal_id.id

    def check_robot(self): # check /status and computer vision
        None    #return true or false i guess

    #first need to publish soemthing before checking the call back --> it gives error right now
    def pub_firsttime(self):#if it is the first time we just wait for the state to not be 3
        while(self.state != 3):
            self.pub_goal.publish(self.goal_action)
            rospy.sleep(1)
            print(self.rosbot_name )
        self.goal_counter += 1      #goal is reached --> increment the counter

    def pub_function(self):
        #divided into 2 functions
        while(self.current_goal_id != self.new_goal_id):    #same_id?()#check if the goal has been update by checking if the id of the goal in the topic is the same as the one we sent from this file 
            print(self.rosbot_name )
            self.pub_goal.publish(self.goal_action)         #publish the goal
            rospy.sleep(1)
            print("i'm checking bro")
            print("goal number ", self.goal_counter)
            print(self.current_goal_id , "\n\n",self.new_goal_id,self.current_goal_id != self.new_goal_id)
        #we can add a if to check if msg.goal id the the same as new_goal
        while(self.state != 3):                             #the goal is published --> we check if it arrived, later we can create a function that check both the status and the computer vision
            self.pub_goal.publish(self.goal_action)#normally we don't need this line anymore
            rospy.sleep(1)
            print("I'm on it 2")
        self.goal_counter += 1      #goal is reached --> increment the counter

    #=============== GOAL #===============
    def create_goal(self, x, y, z, roll, pitch, yaw, w):#create a goal and put it in the goal list
        new_goal = robot_goal(x, y, z, roll, pitch, yaw, w)
        self.goal_list.append(new_goal)

    def update_goal(self, next_robot_goal):#goal_action is a MoveBaseActionGoal, it includes a PoseStamped object (=goal_pose) with the same information
        # ...
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.header.stamp = rospy.Time.now()

        self.goal_pose.pose.position.x = next_robot_goal.x
        self.goal_pose.pose.position.y = next_robot_goal.y
        self.goal_pose.pose.position.z = next_robot_goal.z

        self.goal_pose.pose.orientation.x = next_robot_goal.roll
        self.goal_pose.pose.orientation.y = next_robot_goal.pitch
        self.goal_pose.pose.orientation.z = next_robot_goal.yaw
        self.goal_pose.pose.orientation.w = next_robot_goal.w

        #...
        self.goal_action.header = self.goal_pose.header

        self.goal_action.goal_id.stamp = self.goal_action.header.stamp
        self.goal_action.goal_id.id = "goal number" + str(self.goal_counter)
        self.new_goal_id = self.goal_action.goal_id.id

        self.goal_action.goal.target_pose = self.goal_pose  #update the goal_action

    def send_goal(self):#function that call the publisher 
        self.update_goal(self.goal_list[self.goal_counter])#put it somewhere else ?//#to update the goal we can either call self.goal_list[-1] wich take the last one if is add goal step by step OR we call self.goal_list[self.goal_counter]
            
        if(self.goal_counter == 0):#if it is the irst time we call a specific publisher function
            self.update_goal(self.goal_list[0])
            self.pub_firsttime()
        
        else:
            self.pub_function()

    def send_goal_forloop(self):
        for i in range(len(self.goal_list)):
            self.send_goal()
            print("I'm running the foor loop")

    def main(self):
        #rospy.sleep(2)
        self.create_goal(1,0,0,0,0,0,1)
        self.create_goal(2,2,0,0,0,0,1)
        self.create_goal(-1,0,0,0,0,0,1)
        rospy.sleep(1)
        self.send_goal_forloop()
        rospy.spin()

    def main1(self):
        self.create_goal(-2,-2,0,0,0,0,1)
        self.send_goal()

    def main2(self):
        self.create_goal(2,2,0,0,0,0,1)
        self.send_goal()

if __name__ == "__main__":
    #my_goals = multi_goal_path_planning(0)
    #my_goals.main()
    robot1 = multi_goal_path_planning(1)
    robot2 = multi_goal_path_planning(2)
    robot1.main1()
    robot2.main2()

if __name__ == "__main2__":#for service does rospy.Service(...) need to be the same as rospy.nodes() ? 
    goal_service = rospy.Service('/rosbots_goals_server', , A function that call the class (or the class directly ? ))
    rospy.spin()
