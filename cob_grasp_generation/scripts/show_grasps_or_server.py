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
from cob_manipulation_msgs.msg import ShowGraspsAction, ShowGraspsResult
from cob_grasp_generation import or_grasp_generation, grasp_query_utils

class ShowGraspsOrServer(object):

  def __init__(self):
    self.orgg = or_grasp_generation.ORGraspGeneration()

    self._as_show = actionlib.SimpleActionServer('show_grasps_or', ShowGraspsAction, execute_cb=self.show_cb, auto_start = False)
    self._as_show.start()

    print("ShowGraspsOrServer: action started...")


  def show_cb(self, goal):
    success = False

    rospy.loginfo('Show grasp %i for object %s using gripper_type %s' % (goal.grasp_id, goal.object_name, goal.gripper_type))

    if grasp_query_utils.check_database(goal.object_name, goal.gripper_type):
      self.orgg.setup_environment(goal.object_name, goal.gripper_type, viewer=True)
      rospy.loginfo('Display Grasp. Object: %s | ID: %i' % (goal.object_name, goal.grasp_id))
      self.orgg.show_grasp(goal.object_name, goal.gripper_type, goal.grasp_id, goal.sort_by_quality)
      success = True
    else:
      rospy.logerr('GraspTable for Object %s does not exist!' % (goal.object_name))

    result = ShowGraspsResult()
    result.success = success

    if success:
      rospy.loginfo('Show: Succeeded')
      self._as_show.set_succeeded(result)
    else:
      rospy.logwarn('Show: Failed')
      self._as_show.set_aborted(result)


if __name__ == '__main__':
  rospy.init_node('show_grasps_or_server')
  ShowGraspsOrServer()
  rospy.spin()
