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
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from simple_moveit_interface import get_planning_scene_interface, add_ground, clear_objects


def gen_pose(frame_id="/odom_combined", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

if __name__ == '__main__':
	rospy.init_node('load_scene')
	while rospy.get_time() == 0.0: pass

	psi = get_planning_scene_interface()
	rospy.sleep(1.0)

	### Add a floor
	add_ground()

	### Add primitives (box, sphere, cylinder)
	pose = gen_pose(pos=[-0.9, 0.8, 0.5])
	psi.add_box("box", pose, size=(0.3, 0.5, 0.7))

	pose = gen_pose(pos=[0.0, 0.0, 2.5])
	psi.add_sphere("sphere", pose, radius=(0.3))

	# CollisionChecking with cylinder primitives does not work correctly, thus this is not supported! Use a mesh instead!

	### ADDING A MESH
	#ToDo: use rospack to find package
	#ToDo: meshes cannot be scaled yet
	pose = gen_pose(pos=[0.6, 0.0, 0.9], euler=[-1.57, 0, 3.14])
	filename = '/opt/ros/groovy/stacks/simulator_gazebo/gazebo_worlds/Media/models/mug-test.stl'
	psi.add_mesh("mug", pose, filename)

	rospy.sleep(1.0)

	### ADDING DETECT and URDF not supported anymore
	#ToDo: Add if needed

	### ATTACHING AN OBJECT
	#ToDo: attach without creating objects again
	pose = gen_pose(pos=[0.4, 0.1, 1.0])
	psi.attach_box("tray_link", "box2", pose, (0.1,0.1,0.1))

	pose = gen_pose(pos=[0.5, -0.9, 0.9], euler=[-1.57, 0, 3.14])
	filename = '/opt/ros/groovy/stacks/simulator_gazebo/gazebo_worlds/Media/models/mug-test.stl'
	touch_links = ["sdh_grasp_link", "sdh_palm_link", "sdh_finger_11_link", "sdh_finger_12_link", "sdh_finger_13_link", "sdh_finger_21_link", "sdh_finger_22_link", "sdh_finger_23_link", "sdh_thumb_1_link", "sdh_thumb_2_link", "sdh_thumb_3_link"]
	psi.attach_mesh("arm_7_link", "mug", pose, filename, touch_links)

	rospy.sleep(1.0)

	### DETACHING AN OBJECT
	#ToDo: detach without stating link?
	psi.remove_attached_object("arm_7_link", "mug")

	rospy.sleep(1.0)

	### REMOVING AN OBJECT
	psi.remove_world_object("sphere")

	### CLEAR ALL OBJECTS
	clear_objects()

