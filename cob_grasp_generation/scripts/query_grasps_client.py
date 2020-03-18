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

def query_grasps_client():
    client = actionlib.SimpleActionClient('query_grasps', cob_grasp_generation.msg.QueryGraspsAction)
    client.wait_for_server()

    goal = cob_grasp_generation.msg.QueryGraspsGoal()
    #goal.object_name="peanuts"
    #goal.gripper_type = "sdh"
    goal.object_name="pringles"
    goal.gripper_type = "sdhx"
    #goal.grasp_id = 2
    goal.num_grasps = 0
    goal.threshold = 0

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('query_grasps_client')
        result = query_grasps_client()
        print("Result:")
        print(result)
        #print len(result.grasp_list)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
