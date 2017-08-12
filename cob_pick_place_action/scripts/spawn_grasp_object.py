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
import roslib
from tf.transformations import *
from simple_moveit_interface import *


def gen_pose(frame_id="/odom_combined", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

if __name__ == '__main__':
	rospy.init_node('spawn_grasp_object')
	while rospy.get_time() == 0.0: pass

	psi = get_planning_scene_interface()
	rospy.sleep(1.0)

	### ADDING OBJECT
	object_name = "pringles"
	pose = gen_pose(frame_id="base_footprint", pos=[-0.7, 0.5, 1.05], euler=[0, 0, 0])
	filename = roslib.packages.get_pkg_dir('cob_grasp_generation')+'/files/meshes/'+object_name+'.stl'
	#psi.add_mesh(object_name, pose, filename)	#assimp error
	psi.add_box(object_name, pose, size=(0.1, 0.1, 0.1))

	rospy.sleep(1.0)

