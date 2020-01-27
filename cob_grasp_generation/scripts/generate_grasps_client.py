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


import rospy

import actionlib
import moveit_msgs.msg
import cob_grasp_generation.msg

def generate_grasps_client():
    client = actionlib.SimpleActionClient('generate_grasps', cob_grasp_generation.msg.GenerateGraspsAction)
    client.wait_for_server()

    goal = cob_grasp_generation.msg.GenerateGraspsGoal()
    #goal.object_name="yellowsaltcube"
    #goal.object_name="hotpot"
    #goal.object_name="hotpot2"
    #goal.object_name="instanttomatosoup"
    goal.object_name="cokeplasticsmall"

    goal.gripper_type = "sdh"

    #ToDo: set the other OpenRAVE parameters for grasp_generation
    goal.viewer = True
    goal.replan = True

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('generate_grasps_client')
        result = generate_grasps_client()
        print("Result:")
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
