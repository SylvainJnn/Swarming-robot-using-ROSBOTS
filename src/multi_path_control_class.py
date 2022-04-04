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
    def __init__(self):
        rospy.init_node('multi_path_planning')
        # create the connections to the action server
        self.client1 = actionlib.SimpleActionClient('rosbot1/move_base', MoveBaseAction)
        self.client2 = actionlib.SimpleActionClient('rosbot2/move_base', MoveBaseAction)
        # waits until the action server is up and running
        self.client1.wait_for_server()
        self.client2.wait_for_server()
        # creates Goals to send to the action server
        self.Goal1 = MoveBaseGoal()
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

    def main_loop(self): #define the functionality of the main loop, which will run continuously until the node is shut down
        while not self.ctrl_c:
            self.client1.send_goal(self.Goal1, feedback_cb=self.feedback_callback1)
            self.client2.send_goal(self.Goal2, feedback_cb=self.feedback_callback2)
            #self.client1.wait_for_result()
            self.client2.wait_for_result()
            self.next_goal1()
            #self.next_goal2()
            rospy.spin()              # Create a loop that will keep the program in execution


    # =====================
    # Creation of the goals
    # =====================
    def create_goal(self):
        # Goal 1
        self.Goal1.target_pose.header.stamp = rospy.Time.now()
        self.Goal1.target_pose.header.frame_id = 'map'
        self.Goal1.target_pose.pose.position.x = 2
        self.Goal1.target_pose.pose.position.y = 2
        self.Goal1.target_pose.pose.position.z = 0
        self.Goal1.target_pose.pose.orientation.x = 0
        self.Goal1.target_pose.pose.orientation.y = 0
        self.Goal1.target_pose.pose.orientation.z = 0
        self.Goal1.target_pose.pose.orientation.w = 1

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
        print('=== NEXT GOAL1 ===')
        self.Goal1.target_pose.pose.position.x -=0.5
        self.Goal1.target_pose.pose.position.y = 0.625
        self.client1.send_goal(self.Goal1, feedback_cb=self.feedback_callback1)
        self.next_goal2()
        #self.client1.wait_for_result()

    def next_goal2(self):
        print('=== NEXT GOAL2 ===')
        self.Goal2.target_pose.pose.position.x -=0.5
        self.Goal2.target_pose.pose.position.y = -0.625
        self.client2.send_goal(self.Goal2, feedback_cb=self.feedback_callback2)
        #self.client2.wait_for_result()


    # ===========================
    # Get the goal status
    # ===========================
    def feedback_callback1(self, feedback1):
        feedback1 = self.client1.get_state()
        if feedback1 != 3:
            print('[Rosbot1] State: %d, going to goal..'%(feedback1))
            print('====================================')
        else:
            print('[Rosbot1] State: %d, reached the goal!'%(feedback1))
            print('====================================')

    def feedback_callback2(self, feedback2):
        feedback2 = self.client2.get_state()
        if feedback2 != 3:
            print('[Rosbot2] State: %d, going to goal..'%(feedback2))
            print('====================================')
        else:
            print('[Rosbot2] State: %d, reached the goal!'%(feedback2))
            print('====================================')
            #self.next_goal2()



if __name__ == '__main__': #check to ensure that the script being run is the main executable
    class_instance = multi_goal_path_planning() #create an instance of the multi_goal_path_planning() class 
    try:
        class_instance.main_loop()
    except rospy.ROSInterruptException:
    
        pass