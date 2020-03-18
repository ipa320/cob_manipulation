#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import random

import rospy
import actionlib

import cob_lookat_action.msg

def cob_lookat_action_client():
    client = actionlib.SimpleActionClient('lookat_action', cob_lookat_action.msg.LookAtAction)
    print("Waiting for Server...")
    client.wait_for_server()
    print("...done!")

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
    print(result)

    return result.success

if __name__ == '__main__':
    try:
        rospy.init_node('cob_lookat_action_client')
        result = cob_lookat_action_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
