#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from cob_mmcontroller.msg import *

def main():
    rospy.init_node('learnModelPrior_client')
    client = actionlib.SimpleActionClient('learn_model_prior', LearnModelPriorAction)
    client.wait_for_server()

    goal = LearnModelPriorGoal()
    # Fill in the goal here
    client.send_goal(goal, feedback_cb=print_feedback)
    print client.get_state()
    while client.get_state() == 0 or client.get_state() == 1:#== actionlib.SimpleGoalState.PENDING or client.get_state() == actionlib.SimpleGoalState.ACTIVE:
        #print client.get_goal_status_text()
        client.wait_for_result(rospy.Duration.from_sec(1.0))
    print "RESULT:"
    print client.get_result()

def print_feedback(feedback):
    print "Feedback:"
    print feedback


if __name__ == '__main__':
    main()
