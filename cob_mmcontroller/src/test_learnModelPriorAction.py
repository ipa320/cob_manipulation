#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from cob_mmcontroller.msg import *

if __name__ == '__main__':
    rospy.init_node('learnModelPrior_client')
    client = actionlib.SimpleActionClient('learn_model_prior', LearnModelPriorAction)
    client.wait_for_server()

    goal = LearnModelPriorGoal()
    # Fill in the goal here
    client.send_goal(goal)
    #print client.get_goal_status_text()
    client.wait_for_result()
    print client.get_result()
