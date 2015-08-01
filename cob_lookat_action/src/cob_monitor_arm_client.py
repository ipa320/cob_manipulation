#! /usr/bin/env python

import random

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import *
from moveit_msgs.msg import *
import cob_lookat_action.msg

def cob_lookat_action_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('lookat_action', cob_lookat_action.msg.LookAtAction)
    
    focus_pub = rospy.Publisher('/lookat_target', PoseStamped)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = cob_lookat_action.msg.LookAtGoal()
    focus = PoseStamped()
    focus.header.stamp = rospy.Time.now()
    focus.header.frame_id = "arm_7_link"
    focus.pose.orientation.w = 1.0
    goal.target = focus
    #print "GOAL: ", goal
    
    while not rospy.is_shutdown():
        #client.cancel_goal()
        
        focus.header.stamp = rospy.Time.now()
    
        ##Debug
        #focus_pub.publish(goal.target)
        #rospy.sleep(2.0)
     
        # Sends the goal to the action server.
        client.send_goal(goal)
        
        #rospy.sleep(0.1)


    # Waits for the server to finish performing the action.
    client.wait_for_result()        
    
    result = client.get_result()
    print result

    return result.success

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('cob_lookat_action_client')
        result = cob_lookat_action_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
