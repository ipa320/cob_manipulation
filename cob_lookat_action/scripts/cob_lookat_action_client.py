#! /usr/bin/env python

import random

import rospy
import actionlib

import cob_lookat_action.msg

def cob_lookat_action_client():
    client = actionlib.SimpleActionClient('lookat_action', cob_lookat_action.msg.LookAtAction)
    print "Waiting for Server..."
    client.wait_for_server()
    print "...done!"

    # Creates a goal to send to the action server.
    goal = cob_lookat_action.msg.LookAtGoal()
    goal.target_frame = "lookat_target"
    goal.pointing_frame = "sensorring_base_link"
    goal.pointing_axis_type = 0   # X_POSITIVE
    #print "GOAL: ", goal

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    result = client.get_result()
    print result

    return result.success

if __name__ == '__main__':
    try:
        rospy.init_node('cob_lookat_action_client')
        result = cob_lookat_action_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
