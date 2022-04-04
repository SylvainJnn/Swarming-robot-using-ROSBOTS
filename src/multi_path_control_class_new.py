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
        self.create_goal() #call the function that creats initial goals
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
        self.ctrl_c = True     #set the ctrl_c variable to True to stop the main_loop() method (also defined later).

    def shutdown_function(self): #define the actual shutdown process
        print("shutting down the programme ...")

    def main_loop(self): #define the functionality of the main loop, which will run continuously until the node is shut down
        #while not self.ctrl_c:
        self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback)
        #self.client.wait_for_result()
        #print("i'm here")
        #self.next_goal() 


    # def goal_list(self):
    #     g_list = [-1, 0, 1, 3]
    # # =====================
    # Creation of the goals
    # =====================
    def create_goal(self): # to create initial goals
        # Robot1 Goal
        self.Goal.target_pose.header.stamp = rospy.Time.now()
        self.Goal.target_pose.header.frame_id = 'map'
        self.Goal.target_pose.pose.position.x = -1
        self.Goal.target_pose.pose.position.y = 0
        self.Goal.target_pose.pose.position.z = 0
        self.Goal.target_pose.pose.orientation.x = 0
        self.Goal.target_pose.pose.orientation.y = 0
        self.Goal.target_pose.pose.orientation.z = 0
        self.Goal.target_pose.pose.orientation.w = 1
        # Robot2 Goal
        if(self.rosbot_number == 2):
            self.Goal.target_pose.pose.position.x = 2
            self.Goal.target_pose.pose.position.y = 2



    def next_goal(self): #to publish new goals
        self.Goal.target_pose.header.stamp = rospy.Time.now()
        if(self.rosbot_number == 1):
            print('=== NEXT GOAL1 ===')
            #self.Goal.target_pose.pose.position.x = -2
            self.Goal.target_pose.pose.position.y = -2
            self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback)
        elif(self.rosbot_number == 2):
            print('=== NEXT GOAL2 ===')
            #self.Goal.target_pose.pose.position.x = -2
            self.Goal.target_pose.pose.position.y = 3
            self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback)
        

    # ===========================
    # Get the goal status
    # ===========================
    def feedback_callback(self, feedback):
        feedback = self.client.get_state()
        if feedback != 3:
            print('[Rosbot%d] State: %d, going to goal..'%(self.rosbot_number,feedback))
            print('====================================')
        else:
            print('[Rosbot%d] State: %d, reached the goal!'%(self.rosbot_number, feedback))
            print('*************************************')
        self.client.wait_for_result()
        self.next_goal()
        #if --> new goal in the goal list, goal list provide by another program ? 




if __name__ == '__main__': #check to ensure that the script being run is the main executable
    class_instance = multi_goal_path_planning(1) #create an instance of the multi_goal_path_planning() class 
    class_instance2 = multi_goal_path_planning(2)
    try:
        class_instance2.main_loop()
        class_instance.main_loop()
        rospy.spin()            # Create a loop that will keep the program in execution
    except rospy.ROSInterruptException:
        pass
