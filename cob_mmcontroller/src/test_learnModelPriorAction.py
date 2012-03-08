#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from optparse import OptionParser

from cob_mmcontroller.msg import *

def main():
    parser = OptionParser()
    parser.add_option('-d', '--database', dest = 'database', default = "",
                      action = 'store', help = "file name of database to load prior models from")
    (options, args) = parser.parse_args()

    rospy.init_node('learnModelPrior_client')
    client = actionlib.SimpleActionClient('learn_model_prior', LearnModelPriorAction)
    client.wait_for_server()
    print "Server OK"

    goal = LearnModelPriorGoal()
    goal.database = options.database
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
