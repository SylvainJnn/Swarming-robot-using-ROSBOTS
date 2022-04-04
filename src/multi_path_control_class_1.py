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
class multi_goal_path_planning:
    def __init__(self, rosbot_number):
        self.rosbot_name = self.give_rosbot_name(rosbot_number)
        self.node_name = 'multi_path_planning_'+str(1)
        #print(self.node_name)
        rospy.init_node(self.node_name)
        # create the connections to the action server
        self.client = actionlib.SimpleActionClient(self.rosbot_name + '/move_base', MoveBaseAction)
        # waits until the action server is up and running
        self.client.wait_for_server()
        # creates Goals to send to the action server
        self.Goal = MoveBaseGoal()
        self.Goal2 = MoveBaseGoal()

        self.create_goal() #call the function that creats initial goals

        #create a "ctrl_c" variable and initialise it to False to be used in shutting down our node later.
        self.ctrl_c = False
        #use the following method to register a "shutdownhook" function which will be called when rospy begins shutdown
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self): #define the shutdownhook function
        self.shutdown_function()  #call the shutdown process (defined later in the code)
        self.ctrl_c = True     #set the ctrl_c variable to True to stop the main_loop() method (also defined later).

    def shutdown_function(self): #define the actual shutdown process
        print("shutting down the programme ...")

    def give_rosbot_name(self, rosbot_number):
        if(rosbot_number == 0):
            return("")
        else:
            return("/rosbot" + str(rosbot_number))

    # def main_loop(self): #define the functionality of the main loop, which will run continuously until the node is shut down
    #     while not self.ctrl_c:
    #         self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback1)
    #         self.client.send_goal(self.Goal2, feedback_cb=self.feedback_callback2)
    #         #self.client.wait_for_result()
    #         #self.client2.wait_for_result()
    #         self.next_goal1()
    #         #self.next_goal2()
    #         rospy.spin()              # Create a loop that will keep the program in execution
    
    def main1(self): #define the functionality of the main loop, which will run continuously until the node is shut down
        while not self.ctrl_c:
            self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback1)
            #self.client.wait_for_result()
            self.next_goal1()
            rospy.spin() 

    
    def main2(self): #define the functionality of the main loop, which will run continuously until the node is shut down
        while not self.ctrl_c:
            self.client.send_goal(self.Goal2, feedback_cb=self.feedback_callback2)
            #self.client2.wait_for_result()
            self.next_goal2()
            rospy.spin() 


    # =====================
    # Creation of the goals
    # =====================
    def create_goal(self):
        # Goal 1
        self.Goal.target_pose.header.stamp = rospy.Time.now()
        self.Goal.target_pose.header.frame_id = 'map'
        self.Goal.target_pose.pose.position.x = 2
        self.Goal.target_pose.pose.position.y = 2
        self.Goal.target_pose.pose.position.z = 0
        self.Goal.target_pose.pose.orientation.x = 0
        self.Goal.target_pose.pose.orientation.y = 0
        self.Goal.target_pose.pose.orientation.z = 0
        self.Goal.target_pose.pose.orientation.w = 1

        # Goal 2
        self.Goal2.target_pose.header.stamp = rospy.Time.now()
        self.Goal2.target_pose.header.frame_id = 'map'
        self.Goal2.target_pose.pose.position.x = 2
        self.Goal2.target_pose.pose.position.y = -4
        self.Goal2.target_pose.pose.position.z = 0
        self.Goal2.target_pose.pose.orientation.x = 0
        self.Goal2.target_pose.pose.orientation.y = 0
        self.Goal2.target_pose.pose.orientation.z = 0
        self.Goal2.target_pose.pose.orientation.w = 1


    def next_goal1(self):
        print('=== NEXT Goal1 ===')
        self.Goal.target_pose.pose.position.x -=0.5
        self.Goal.target_pose.pose.position.y = 0.625
        self.client.send_goal(self.Goal, feedback_cb=self.feedback_callback1)
        #self.client.wait_for_result()

    def next_goal2(self):
        print('=== NEXT GOAL2 ===')
        self.Goal2.target_pose.pose.position.x -=0.5
        self.Goal2.target_pose.pose.position.y = -0.625
        self.client.send_goal(self.Goal2, feedback_cb=self.feedback_callback2)
        #self.client2.wait_for_result()


    # ===========================
    # Get the goal status
    # ===========================
    def feedback_callback1(self, feedback1):
        feedback1 = self.client.get_state()
        if feedback1 != 3:
            print('[Rosbot1] State: %d, going to goal..'%(feedback1))
            print('====================================')
        else:
            print('[Rosbot1] State: %d, reached the goal!'%(feedback1))
            print('====================================')

    def feedback_callback2(self, feedback2):
        feedback2 = self.client.get_state()
        if feedback2 != 3:
            print('[Rosbot2] State: %d, going to goal..'%(feedback2))
            print('====================================')
        else:
            print('[Rosbot2] State: %d, reached the goal!'%(feedback2))
            print('====================================')
            #self.next_goal2()



if __name__ == '__main__': #check to ensure that the script being run is the main executable
    rosbot1 = multi_goal_path_planning(1) #create an instance of the multi_goal_path_planning() class
    rosbot2 = multi_goal_path_planning(2)
    try:
        rosbot1.main1()
        rosbot2.main2()
    except rospy.ROSInterruptException:
    
        pass