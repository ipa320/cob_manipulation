#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from cob_mmcontroller.msg import *
from articulation_msgs.msg import ParamMsg

if __name__ == '__main__':
    rospy.init_node('articulationModel_client')
    client = actionlib.SimpleActionClient('moveModel', ArticulationModelAction)
    client.wait_for_server()

    goal = ArticulationModelGoal()
    # Fill in the goal here
    goal.model_id = 1
    goal.model.name = "rotational"
    goal.model.params.append(ParamMsg('angle', -1.57, 1))
    goal.model.params.append(ParamMsg('rot_center.x', 0.0, 1)) 
    goal.model.params.append(ParamMsg('rot_center.y', 0.5, 1)) 
    goal.model.params.append(ParamMsg('rot_center.z', 0.0, 1)) 
    goal.target_duration.data.secs = 10.0

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
