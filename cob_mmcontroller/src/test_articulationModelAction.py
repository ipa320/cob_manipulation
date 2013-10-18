#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

import rosbag

import sys
from optparse import OptionParser

from cob_mmcontroller.msg import *
from cob_mmcontroller.srv import *
from articulation_msgs.msg import ParamMsg

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option('-f', '--filename', dest = 'bagfilename',
                      action = 'store', help = 'bag file name')
    parser.add_option('-c', '--config', dest = 'config',
                      action = 'store',
                      help = 'mechanism configuration. for example opening angle of a door in rad. no default')
    parser.add_option('-d', '--duration', dest = 'duration',
                      default = 10.0, action = 'store',
                      help = 'duration until target postion should be reached. default = 10.0')
    (options, args) = parser.parse_args()
    
    rospy.init_node('articulationModel_client')
    client = actionlib.SimpleActionClient('moveModel', ArticulationModelAction)
    client.wait_for_server()

    goal = ArticulationModelGoal()
    # Fill in the goal here
    goal.model.id = 1

    bag = rosbag.Bag(options.bagfilename)
    for topic, msg, t in bag.read_messages(topics='model'):
        goal.model = msg

    goal.model.params.append(ParamMsg('action', float(options.config), 1))
    goal.target_duration.secs = float(options.duration) 

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(float(options.duration) + 5.0))
    print client.get_result()
