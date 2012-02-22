#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from cob_mmcontroller.msg import *

if __name__ == '__main__':
    rospy.init_node('articulationModel_client')
    client = actionlib.SimpleActionClient('moveModel', ArticulationModelAction)
    client.wait_for_server()

    goal = ArticulationModelGoal()
    # Fill in the goal here
    goal.model_id = 1
    goal.model.name = "rotational"
    goal.model.params.append({name: "angle", value: -1.57})
    goal.model.params.append({'name': "rot_center.x", 'value': 0.0}) 
    goal.model.params.append({'name': "rot_center.y", 'value': 0.5}) 
    goal.model.params.append({'name': "rot_center.z", 'value': 0.0}) 
    goal.target_duration = 10.0

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
