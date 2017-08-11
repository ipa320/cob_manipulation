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
from cob_grasp_generation import grasp_query_utils

class CobGraspQueryActionServer(object):

  def __init__(self):

    self._as_query = actionlib.SimpleActionServer('query_grasps', cob_grasp_generation.msg.QueryGraspsAction, execute_cb=self.query_cb, auto_start = False)
    self._as_query.start()

    print("CobGraspQueryActionServer: actions started...")

  def query_cb(self, goal):
    success = False
    grasp_list = []

    rospy.loginfo('Querying grasps for object %s using gripper_type %s' % (goal.object_name, goal.gripper_type))

    if grasp_query_utils.check_database(goal.object_name, goal.gripper_type):
        rospy.loginfo('GraspTable for object %s exist in the database.', goal.object_name)
        rospy.loginfo('Returning grasp list for selected object.')
        grasp_list = grasp_query_utils.get_grasps(goal.object_name, goal.gripper_type, goal.grasp_id, goal.num_grasps, goal.threshold)
    else:
        rospy.logwarn('GraspTable for object %s does not exist!',goal.object_name)

    if not (grasp_list == []):
        success = True

    result   = cob_grasp_generation.msg.QueryGraspsResult()
    result.success = success
    result.grasp_list = grasp_list

    if success:
      rospy.loginfo('Query: Succeeded')
      self._as_query.set_succeeded(result)
    else:
      rospy.logwarn('Query: Failed')
      self._as_query.set_aborted(result)

if __name__ == '__main__':
  rospy.init_node('query_grasps')
  CobGraspQueryActionServer()
  rospy.spin()
