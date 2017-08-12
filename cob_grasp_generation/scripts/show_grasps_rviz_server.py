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


import rospy, roslib

import actionlib
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import visualization_msgs.msg
import cob_grasp_generation.msg
from cob_grasp_generation import grasp_query_utils

import tf
import tf2_ros
from urdf_parser_py.urdf import URDF

class ShowGraspsRvizServer(object):

  def __init__(self):
    self.joint_names = []
    self.joint_positions = []
    self.joint_mimic = []
    for joint in URDF.from_parameter_server().joints:
      if joint.mimic:
        self.joint_mimic.append(joint)
      elif joint.type == 'revolute':
        self.joint_names.append(joint.name)
        if joint.limit.lower < 0.0 and 0.0 < joint.limit.upper:
          self.joint_positions.append(0.0)
        else:
          self.joint_positions.append((joint.limit.upper + joint.limit.lower)/2.0)

    self.js = sensor_msgs.msg.JointState()
    self.js.name = self.joint_names
    self.js.position = self.joint_positions

    self.t = geometry_msgs.msg.TransformStamped()
    self.t.header.frame_id = "object_link"
    self.t.child_frame_id = "grasp_link"
    self.t.transform.translation.x = 0.0
    self.t.transform.translation.y = 0.0
    self.t.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    self.t.transform.rotation.x = q[0]
    self.t.transform.rotation.y = q[1]
    self.t.transform.rotation.z = q[2]
    self.t.transform.rotation.w = q[3]

    self.marker = visualization_msgs.msg.Marker()
    self.marker.header.frame_id = "object_link"
    self.marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE
    self.marker.pose.orientation.w = 1.0
    self.marker.scale = geometry_msgs.msg.Vector3(1,1,1)
    self.marker.color = std_msgs.msg.ColorRGBA(1,1,1,1)

    self._br = tf2_ros.TransformBroadcaster()
    self._js_pub = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=1)
    self._marker_pub = rospy.Publisher('/object_marker', visualization_msgs.msg.Marker, queue_size=1)
    self._timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    self._as_show = actionlib.SimpleActionServer('show_grasps_rviz', cob_grasp_generation.msg.ShowGraspsAction, execute_cb=self.show_cb, auto_start = False)
    self._as_show.start()

    print("ShowGraspsRvizServer: action started...")

  def show_cb(self, goal):
    success = False

    rospy.loginfo('Show grasp %i for object %s using gripper_type %s' % (goal.grasp_id, goal.object_name, goal.gripper_type))

    if grasp_query_utils.check_database(goal.object_name, goal.gripper_type):
      self.marker.mesh_resource = "package://cob_grasp_generation/files/meshes/"+goal.object_name+".stl"
      grasp_list = grasp_query_utils.get_grasps(goal.object_name, goal.gripper_type, goal.grasp_id, 1)
      if grasp_list:
        #print grasp_list
        self.js.name = grasp_list[0].pre_grasp_posture.joint_names              #TODO: joint names in urdf are "side-independend"
        self.js.position = grasp_list[0].grasp_posture.points[0].positions
        for joint in self.joint_mimic:
          idx = grasp_list[0].pre_grasp_posture.joint_names.index(joint.mimic.joint)
          self.js.name.append(joint.name)
          self.js.position.append(grasp_list[0].grasp_posture.points[0].positions[idx])
        self.t.transform.translation = grasp_list[0].grasp_pose.pose.position
        self.t.transform.rotation = grasp_list[0].grasp_pose.pose.orientation
        success = True
      else:
        success = False
    else:
      rospy.logerr('GraspTable for Object %s does not exist!' % (goal.object_name))

    result   = cob_grasp_generation.msg.ShowGraspsResult()
    result.success = success

    if success:
      rospy.loginfo('Show: Succeeded')
      self._as_show.set_succeeded(result)
    else:
      rospy.logwarn('Show: Failed')
      self._as_show.set_aborted(result)

  def timer_cb(self, event):
    self.t.header.stamp = event.current_real    
    self.js.header.stamp = event.current_real
    self.marker.header.stamp = event.current_real

    self._br.sendTransform(self.t)
    self._js_pub.publish(self.js)
    self._marker_pub.publish(self.marker)


if __name__ == '__main__':
  rospy.init_node('show_grasps_rviz_server')
  ShowGraspsRvizServer()
  rospy.spin()
