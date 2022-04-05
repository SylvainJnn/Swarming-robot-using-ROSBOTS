#!/usr/bin/env python

# =============================================
# Multi Rosbots Control : Go in different goals
# =============================================
from turtle import st
import rospy
# for the path planning
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback
from robot_goal import robot_goal
# ==========================

class multi_goal_path_planning:
    def __init__(self, rosbot_number):
        self.rosbot_name = self.give_rosbot_name(rosbot_number)
        self.node_name = 'rosbot_path_planning_'+str(1)
        rospy.init_node(self.node_name)      # Initialization of the node
        # create the connections to the action server
        self.client = actionlib.SimpleActionClient(self.rosbot_name + '/move_base', MoveBaseAction)
        # waits until the action server is up and running
        self.client.wait_for_server()

        self.rosbot_number = rosbot_number # a variable to recieve the robot's number
        
        self.Goal = MoveBaseGoal()  # create Goals to send to the action server
        
        self.goal_list = [] #self.array([])
        self.goal_counter = 0

        #create a "ctrl_c" variable and initialise it to False to be used in shutting down our node later.
        self.ctrl_c = False
        #use the following method to register a "shutdownhook" function which will be called when rospy begins shutdown
        rospy.on_shutdown(self.shutdownhook)

    def give_rosbot_name(self, rosbot_number):
        if(rosbot_number == 0):
            return("")
        else:
            return("/rosbot" + str(rosbot_number))

    def shutdownhook(self): #define the shutdownhook function
        self.shutdown_function()  #call the shutdown process (defined later in the code)
        self.ctrl_c = True     #set the ctrl_c variable to True to stop the main() method (also defined later).

    def shutdown_function(self): #define the actual shutdown process
        print("shutting down the programme ...")

    def main(self): #define the functionality of the main loop, which will run continuously until the node is shut down
        # To send the rosbots to different goals (uncomment)
        if(self.rosbot_number == 1):
            self.create_goal(2,2,0,0,0,0,1)
            self.create_goal(2,4,0,0,0,0,1)
            self.create_goal(2,6,0,0,0,0,1)
        elif (self.rosbot_number == 2):
            self.create_goal(-2,-2,0,0,0,0,1)
            self.create_goal(-2,-4,0,0,0,0,1)
            self.create_goal(-2,-6,0,0,0,0,1)
        else: 
            self.create_goal(2,-2,0,0,0,0,1)
            self.create_goal(2,-4,0,0,0,0,1)
            self.create_goal(2,-6,0,0,0,0,1)
        # To send the rosbots to the same goal (uncomment)
        #self.create_goal(1,0,0,0,0,0,1) #call the function that creats initial goals
        # self.create_goal(2,2,0,0,0,0,1)
        # self.create_goal(2,3,0,0,0,0,1)
        # self.create_goal(-1,0,0,0,0,0,1)

        self.update_goal(self.goal_list[self.goal_counter])
        self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback)

    # =====================
    # Creation of the goals
    # =====================
    def create_goal(self, x, y, z, roll, pitch, yaw, w):#create a goal and put it in the goal list
        new_goal = robot_goal(x, y, z, roll, pitch, yaw, w)
        self.goal_list.append(new_goal)

    
    
    def update_goal(self, next_robot_goal): # to update to new goal
        self.Goal.target_pose.header.stamp = rospy.Time.now()
        self.Goal.target_pose.header.frame_id = 'map'
        self.Goal.target_pose.pose.position.x = next_robot_goal.x
        self.Goal.target_pose.pose.position.y = next_robot_goal.y
        self.Goal.target_pose.pose.position.z = next_robot_goal.z
        self.Goal.target_pose.pose.orientation.x = next_robot_goal.roll
        self.Goal.target_pose.pose.orientation.y = next_robot_goal.pitch
        self.Goal.target_pose.pose.orientation.z = next_robot_goal.yaw
        self.Goal.target_pose.pose.orientation.w = next_robot_goal.w

    # ===========================
    # Get the goal status
    # ===========================
    def feedback_callback(self, feedback):
        feedback = self.client.get_state()
        if feedback != GoalStatus.SUCCEEDED:
            if self.goal_counter == 1:
                print('=== START ===')
            else: 
                print('=== NEXT GOAL ===')
            print('[Rosbot%d] going to goal number %d'%(self.rosbot_number,self.goal_counter))
            self.goal_list[self.goal_counter].print_goal()
        self.goal_counter += 1 
        self.client.wait_for_result()
        if(self.goal_counter < len(self.goal_list)):
            self.update_goal(self.goal_list[self.goal_counter])
            self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback)
        else:
            print("==== NO MORE GOALS! ====")
            return()




if __name__ == '__main__': #check to ensure that the script being run is the main executable
    #create instances of the multi_goal_path_planning() class 
    class_instance = multi_goal_path_planning(1) 
    class_instance2 = multi_goal_path_planning(2)
    class_instance3 = multi_goal_path_planning(3) #uncomment to work with the 3rd robot
    try:
        class_instance3.main() #uncomment to work with the 3rd robot
        class_instance2.main()
        class_instance.main()
        rospy.spin()            # Create a loop that will keep the program in execution
    except rospy.ROSInterruptException:
        pass
