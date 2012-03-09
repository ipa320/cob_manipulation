#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from optparse import OptionParser

from cob_mmcontroller.msg import *

def main():
    parser = OptionParser()
    parser.add_option('-a', '--action_var', dest = 'action_var', default = 0.0,
                      action = 'store', help = "describes the movement relative to the start pose 
                      (e.g. opening angle in rad for a rotational articulation or distance in m 
                      for a prismatic one)")
    parser.add_option('-i', '--model_id', dest = 'model_id', default = -1,
                      action = 'store', help = "ID of model according to which movement is executed")
    parser.add_option('-d', '--database', dest = 'database', default = "",
                      action = 'store', help = "file name of database to load prior models from")
    parser.add_option('-t', '--target_duration', dest = 'duration', default = 20.0,
                      action = 'store', help = "duration in seconds until movement shoulld be finished")
    (options, args) = parser.parse_args()

    rospy.init_node('moveModelPrior_client')
    client = actionlib.SimpleActionClient('move_model_prior', MoveModelPriorAction)
    client.wait_for_server()
    print "Server OK"

    # set up goal
    goal = moveModelPriorGoal()
    if options.database == "":
        rospy.errlog("No database is given to load prior models from!")
        rospy.signal_shutdown("Database is missing!")
    goal.database = options.database
    if options.model_id < 0:
        rospy.errlog("No model ID is given according to which a movement could be executed!")
        rospy.signal_shutdown("Model ID is missing!")
    goal.model_id = options.model_id
    goal.action_variable = options.action_var
    goal.duration = options.duration

    client.send_goal(goal, feedback_cb=print_feedback)
    print client.get_state()
    while client.get_state() == 0 or client.get_state() == 1:
        client.wait_for_result(rospy.Duration.from_sec(1.0))
    print "RESULT:"
    print client.get_result()

def print_feedback(feedback):
    print "Feedback:"
    print feedback


if __name__ == '__main__':
    main()
