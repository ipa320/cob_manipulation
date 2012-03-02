#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from optparse import OptionParser

from cob_mmcontroller.msg import *
from cob_mmcontroller.srv import *
from articulation_msgs.msg import ParamMsg

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option('-a', '--angle', dest = 'angle',
                      default = 1.57, action = 'store', 
                      help = 'for example opening angle of a door')
    parser.add_option('-f', '--filename', dest = 'filename',
                      default = 'no_name', action = 'store',
                      help = 'informative and unique filename')
    parser.add_option('-x', '--rot_center_x', dest = 'rot_center_x',
                      default = 0.0, action = 'store',
                      help = 'distance in x-coordinate from gripper to rotation center. default = 0.0')
    parser.add_option('-y', '--rot_center_y', dest = 'rot_center_y',
                      default = 0.0, action = 'store',
                      help = 'distance in y-coordinate from gripper to rotation center. default = 0.0')
    parser.add_option('-z', '--rot_center_z', dest = 'rot_center_z',
                      default = 0.0, action = 'store',
                      help = 'distance in x-coordinate from gripper to rotation center. default = 0.0')
    parser.add_option('-d', '--duration', dest = 'duration',
                      default = 10.0, action = 'store',
                      help = 'duration until target postion should be reached. default = 10.0')
    
    (options, args) = parser.parse_args()

    rospy.init_node('articulationModel_client')
    client = actionlib.SimpleActionClient('moveModel', ArticulationModelAction)
    client.wait_for_server()

    record_srv = rospy.ServiceProxy('/record_track', RecordTrack)

    request = RecordTrackRequest()
    request.file_name.data = options.filename
    response = record_srv(request)

    goal = ArticulationModelGoal()
    # Fill in the goal here
    goal.model_id = 1
    goal.model.name = "rotational"
    goal.model.params.append(ParamMsg('angle', float(options.angle), 1))
    goal.model.params.append(ParamMsg('rot_center.x', float(options.rot_center_x), 1)) 
    goal.model.params.append(ParamMsg('rot_center.y', float(options.rot_center_y), 1)) 
    goal.model.params.append(ParamMsg('rot_center.z', float(options.rot_center_z), 1)) 
    goal.target_duration.data.secs = float(options.duration)

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(float(options.duration) + 5.0))
    print client.get_result()

    response = record_srv(request)
    print response
