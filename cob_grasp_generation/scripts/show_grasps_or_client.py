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

def show_grasps_client():
    client = actionlib.SimpleActionClient('show_grasps_or', cob_grasp_generation.msg.ShowGraspsAction)
    client.wait_for_server()

    #object_name = raw_input("Insert object name: ")
    #grasp_id = int(raw_input("Insert grasp_id: "))
    object_name = "yellowsaltcube"
    grasp_id = 0

    gripper_type = "sdh"

    while not rospy.is_shutdown():
        print(grasp_id)

        # Set the goal here: object_name, grasp_id, sort-by-quality
        goal = cob_grasp_generation.msg.ShowGraspsGoal(object_name, gripper_type, grasp_id, True)

        client.send_goal(goal)
        client.wait_for_result()
        success = client.get_result().success
        if not success:
            break

        input("Enter for next grasp...")
        grasp_id = grasp_id + 1
    
    print("no more grasps")


if __name__ == '__main__':
    try:
        rospy.init_node('show_grasp_client')
        result = show_grasps_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
